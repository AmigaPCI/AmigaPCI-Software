#include <stdint.h>
#include <libopencm3/stm32/pwr.h>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/f2/rtc.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <string.h>
#include "clock.h"
#include "cmdline.h"
#include "main.h"
#include "config.h"
#include "printf.h"
#include "irq.h"
#include "rtc.h"
#include "timer.h"
#include "gpio.h"
#include "uart.h"
#include "utils.h"

#define LSE_HZ               32768  // STM32F2 user manual says 32.768 kHz
#define LSI_HZ               32000  // STM32F2 reference says 32 kHz
#define CLOCK_HSE_HZ       8000000  // External 8 MHz clock

/* Short and long spin loop timeouts */
#define RTC_TIMEOUT          0x00010000  // Simple loop timeout (~25ms)
#define RTC_TIMEOUT_SHORT    0x00001000  // Simple loop timeout (~2ms)
#define RTC_TIMEOUT_MS       25          // Long timeout

/* Divisor limits for RTC_HSE from HSE */
#define RTC_HSE_DIV_MIN      2
#define RTC_HSE_DIV_MAX      31

#define TIME_STR_NO_MSEC     ((uint) -1)   // Do not display milliseconds

#define DAYS_2024            19722       // Number of days between 1970 and 2024

/*
 * libopencm3 does not yet define these for STM32F2
 *
 * Issue #1588, PR #1589
 * https://github.com/libopencm3/libopencm3/pull/1589
 */
#ifndef RCC_BDCR_RTCSEL_SHIFT
#define RCC_BDCR_RTCSEL_SHIFT			8
#define RCC_BDCR_RTCSEL_MASK			0x3
#define RCC_BDCR_RTCSEL_NONE			0
#define RCC_BDCR_RTCSEL_LSE			1
#define RCC_BDCR_RTCSEL_LSI			2
#define RCC_BDCR_RTCSEL_HSE			3
#endif

/* Do-nothing function provided by libopencm3 */
void     null_handler(void);
static uint utc_offset = 0;

/* Adjustments for high resolution timer */
#if 0
static int64_t tick_drift = 0;
static int32_t sync_interval = 0;
static uint64_t last_drift_update = 0;
#endif

/*
 * rtc_prediv_sync is the final divisor (PREDIV_S) by which the RTC input
 * clock is scaled to count seconds.  It is the maximum value that the
 * RTC_SSR will hold.  Dividing by this value gives us a subseconds percent.
 * This value is calculated based on the input clock by rtc_clock_config().
 */
static uint rtc_prediv_sync;   // Will be calculated

/**
 * rtc_binary_to_bcd() converts an unsigned integer to BCD.  BCD is the
 *                     format used by the the STM32F2 Real-Time Clock.
 *                     Two-digit BCD is supported.
 *
 * @param [in]  value - Unsigned integer to convert.
 *
 * @return      Two-digit BCD value.
 */
uint8_t
rtc_binary_to_bcd(uint value)
{
    return ((uint8_t)((value / 10) << 4) + (value % 10));
}

/**
 * rtc_bcd_to_binary() converts a BCD value to an unsigned integer.  BCD is
 *                     the format used by the the STM32F2 Real-Time Clock.
 *                     Two-digit BCD is supported.
 *
 * @param [in]  value - BCD value to convert.
 *
 * @return      Unsigned integer.
 */
uint8_t
rtc_bcd_to_binary(uint8_t value)
{
    return (((value >> 4) * 10) + (value & 0xf));
}


/**
 * rtc_init_mode() enters or exits RTC initialization mode.  When the RTC
 *                 is in initialization mode, the clock does not advance.
 *
 * @param [in]  enter - TRUE to enter initialization mode and stop the clock.
 *                      FALSE to exit initialization mode and start the clock.
 *
 * @return      RC_SUCCESS - Mode change successful.
 * @return      RC_TIMEOUT - RTC did not enter INIT mode.
 */
static rc_t
rtc_init_mode(bool_t enter)
{
    uint32_t timeout;

    if (enter) {
        /* Enter initialization mode and stop the clock */
        if ((RTC_ISR & RTC_ISR_INITF) == 0) {
            /* Set the Initialization mode */
            RTC_ISR = RTC_ISR_INIT;

            /* Wait until RTC is in INIT state */
            for (timeout = RTC_TIMEOUT; timeout > 0; timeout--) {
                if ((RTC_ISR & RTC_ISR_INITF) != 0)
                    break;
            }
        } else {
            /* Clear "timestamp is ready" bit as clock just started */
            RTC_ISR &= ~RTC_ISR_RSF;
        }
    } else {
        /* Exit initialization mode and start the clock */
        RTC_ISR &= ~RTC_ISR_INIT;
    }

    return (RC_SUCCESS);
}

/**
 * rtc_allow_writes() locks or unlocks RTC registers to disable or enable
 *                    software updates to those registers.  The default
 *                    mode is that registers are locked until an update is
 *                    necessary.
 *
 * @param [in]  allow - TRUE to enable writes.
 *                      FALSE to disable writes.
 *
 * @return      None.
 */
void
rtc_allow_writes(int allow)
{
    if (allow) {
        /* Disable the write protection for RTC registers */
        RTC_WPR = 0xCA;
        RTC_WPR = 0x53;
    } else {
        RTC_WPR = 0xFF;
    }
}

/*
 * rtc_set_calibrate() will enable or disable STM32 RTC calibration output,
 *                     which is a 512 Hz signal emitted on PC13.
 */
static void
rtc_set_calibrate(uint enable)
{
    rtc_allow_writes(1);
    if (enable)
        RTC_CR |= RTC_CR_COE;
    else
        RTC_CR &= ~RTC_CR_COE;
    rtc_allow_writes(0);

    if (enable)
        gpio_setmode(STMRSTA_PORT, STMRSTA_PIN, GPIO_SETMODE_AF_AF1);
    else
        gpio_setmode(STMRSTA_PORT, STMRSTA_PIN, GPIO_SETMODE_INPUT_PU);
}

static void
rtc_set_calibrate_offset(int offset)
{
    uint32_t calibr = RTC_CALIBR;
    uint32_t calibr_new;
    int      val;
    int      val_new;

    if (calibr & RTC_CALIBR_DCS) {
        /* Negative frequency calibration */
        val = (calibr & RTC_CALIBR_DC_MASK) * -2;
    } else {
        /* Positive frequency calibration */
        val = (calibr & RTC_CALIBR_DC_MASK) * 4;
    }

    if ((val < 0) || (val + offset < 0))  {
        /* Adjusting calibration which is currently negative */
        if (offset == -1)
            offset = -2;
        else if (offset == 1)
            offset = 2;
    } else {
        /* Adjusting calibration which is currently positive */
        if (offset == -1)
            offset = -4;
        else if (offset == 1)
            offset = 4;
    }

    if (offset == 0) {
        val_new = 0;
    } else {
        val_new = val + offset;
    }

    if (val_new < 0) {
        /* Negative frequency calibration */
        if (val_new < -62)
            val_new = -62;
        calibr_new = ((0 - val_new) / 2) | RTC_CALIBR_DCS;
    } else {
        /* Positive frequency calibration */
        if (val_new > 126)
            val_new = 126;
        calibr_new = (val_new / 4);
    }

    rtc_allow_writes(1);
    rtc_init_mode(TRUE);
    RTC_CALIBR = calibr_new;
    rtc_init_mode(FALSE);
    rtc_allow_writes(0);
    printf("\nval=%d -> %d  cal=%04lx -> %04lx\n",
           val, val_new, calibr, calibr_new);
}

static rc_t
rtc_check_date(uint *year, uint mon, uint day)
{
    if (*year >= 2000)
        *year %= 100;

    if ((*year > 99) || (mon > 12) || (day > 31) ||
        ((mon == 2) && (day > 29)) ||
        ((mon == 4 || mon == 6 || mon == 9 || mon == 11) && (day > 30))) {
        printf("Invalid date %u-%02u-%02u\n", *year, mon, day);
        return (RC_FAILURE);
    }

    return (RC_SUCCESS);
}

static rc_t
rtc_check_time(uint hour, uint min, uint sec)
{
    if ((hour > 23) || (min > 59) || (sec > 59)) {
        printf("Invalid time %u:%02u:%02u\n", hour, min, sec);
        return (RC_FAILURE);
    }

    return (RC_SUCCESS);
}

/**
 * rtc_set_date() sets the current date in the STM32F2 Real-Time Clock.
 *
 * @param [in]  year - The year number (either 0..99 -- century may be provided
 *                     but is currently ignored).
 * @param [in]  mon  - The month number (1..12).
 * @param [in]  day  - The day number (1..31).  Depending on the month number,
 *                     some day numbers are invalid (for example day 30 in
 *                     month 2).
 * @param [in]  dow  - The day-of-week number (1..7).
 *
 * @return      None.
 */
void
rtc_set_date(uint year, uint mon, uint day, uint dow)
{
    if (rtc_check_date(&year, mon, day) != RC_SUCCESS)
        return;      // error printed

    if (rtc_init_mode(TRUE) != RC_SUCCESS)
        return;

    RTC_DR = (rtc_binary_to_bcd(year) << 16) |
             (dow << 13)                     |
             (rtc_binary_to_bcd(mon) << 8)   |
             (rtc_binary_to_bcd(day));

    (void) rtc_init_mode(FALSE);
}

/**
 * rtc_set_time() sets the current time in the STM32F2 Real-Time Clock.
 *
 * @param [in]  hour      - The hour number in 24-hour format (0..23).
 * @param [in]  min       - The minute number (0..59).
 * @param [in]  sec       - The second number (0..59).
 * @param [in]  is_24hour - 1 = 24-hour mode
 * @param [in]  ampm      - 12-hour AM / PM status (1 = PM).
 *
 * @return      None.
 */
void
rtc_set_time(uint hour, uint min, uint sec, uint is_24hour, uint ampm)
{
    if (rtc_check_time(hour, min, sec) != RC_SUCCESS)
        return;     // error printed

    if (rtc_init_mode(TRUE) != RC_SUCCESS)
        return;

    if (is_24hour)
        RTC_CR &= ~RTC_CR_FMT;
    else
        RTC_CR |= RTC_CR_FMT;

    RTC_TR = (rtc_binary_to_bcd(hour) << 16) |
             (rtc_binary_to_bcd(min)  << 8)  |
             (rtc_binary_to_bcd(sec))        |
             (ampm ? RTC_TR_PM : 0);

    (void) rtc_init_mode(FALSE);
}

/**
 * rtc_set_datetime() sets the current date and time in the STM32F2 Real-Time
 *                    Clock. The STM32 hardware seems to require a minimum
 *                    period between locking and unlocking the RTC registers, so
 *                    calling rtc_set_time() consecutively after rtc_set_date()
 *                    instead of calling this combined function may fail to
 *                    apply the time settings.
 *
 * @param [in]  year - The year number (either 0..99 -- century may be provided
 *                     but is currently ignored).
 * @param [in]  mon  - The month number (1..12).
 * @param [in]  day  - The day number (1..31).  Depending on the month number,
 *                     some day numbers are invalid (for example day 30 in
 *                     month 2).
 * @param [in]  hour - The hour number in 24-hour format (0..23).
 * @param [in]  min  - The minute number (0..59).
 * @param [in]  sec  - The second number (0..59).
 *
 * @return      None.
 */
void
rtc_set_datetime(uint year, uint mon, uint day, uint hour, uint min, uint sec)
{
    uint dow   = 1;  // Monday
    uint ampm  = 0;  // 24-hour notation

    if ((rtc_check_date(&year, mon, day) != RC_SUCCESS) ||
        (rtc_check_time(hour, min, sec) != RC_SUCCESS))
        return;      // error printed

    if (rtc_init_mode(TRUE) != RC_SUCCESS)
        return;

    RTC_DR = (rtc_binary_to_bcd(year)  << 16) |
             (dow << 13)                      |
             (rtc_binary_to_bcd(mon) << 8)  |
             (rtc_binary_to_bcd(day));
    RTC_TR = (rtc_binary_to_bcd(hour) << 16) |
             (rtc_binary_to_bcd(min)  << 8)  |
             (rtc_binary_to_bcd(sec))        |
             (ampm ? RTC_TR_PM : 0);

    (void) rtc_init_mode(FALSE);
}

static uint
wait_for_osc_ready(enum rcc_osc osc)
{
    uint timeout = 1000000;
    while (!rcc_is_osc_ready(osc))
        if (timeout-- == 0)
            return (1);
    return (0);
}

/**
 * rtc_clock_config() configures and enables the RTC clock.  The RTC clock
 *                    is driven by one of three clock sources (LSI, LSE,
 *                    or HSE).
 *
 * @param [in]  clk_source   - The clock source to use, which is one of:
 *                             RCC_BDCR_RTCSEL_HSE - High Speed External clock.
 *                             RCC_BDCR_RTCSEL_LSI - Low Speed Internal clock.
 *                             RCC_BDCR_RTCSEL_LSE - Low Speed External clock.
 * @param [out] prediv_sync  - The programmed synchronous divisor chosen for
 *                             the clock to provide the 1 Hz seconds clock of
 *                             the RTC.  This divisor may be used by other
 *                             functions to compute the subseconds value
 *                             by reading the RTC_SSR register.
 * @param [out] prediv_async - The programmed asynchronous divisor chosen for
 *                             the clock.  This output of this divisor feeds
 *                             the synchronous divisor, which should then
 *                             provide the 1 Hz seconds clock of the RTC.
 *
 * @return      RC_SUCCESS - Initialization completed successfully.
 * @return      RC_FAILURE - Invalid input clock for RTC divisor.
 */
static rc_t
rtc_clock_config(uint clk_source, uint *prediv_async, uint *prediv_sync)
{
    uint input_clk;
    uint min;
    uint clk_error;
    uint clk;
    uint hse_div = 1;

    /*
     * The goal is to get input_clk to 1 Hz as accurately as possible
     */
    switch (clk_source) {
        case RCC_BDCR_RTCSEL_LSE:
            /*
             * LSE is an optional 32.768 kHz external crystal (LSE crystal)
             * which optionally drives the RTC clock.
             */
            input_clk = LSE_HZ;
            clk       = input_clk;

            /* Enable LSE and wait until it is ready */
            rcc_osc_on(RCC_LSE);
            if (wait_for_osc_ready(RCC_LSE)) {
                printf("RTC LSE clock failed to come ready\n");
                return (RC_FAILURE);
            }
            break;

        case RCC_BDCR_RTCSEL_LSI:
            /*
             * LSI is a low-speed internal 32.768 kHz RC which drives the
             * independent watchdog and optionally the RTC.  The LSI is
             * described as being not very accurate, but always available
             * since it's integrated into the microprocessor.  cdh measured
             * one and found it to be about 33.375 kHz, which is 1.0856% fast.
             */
            input_clk = LSI_HZ;
            clk       = input_clk;

            /* Enable LSI and wait until it is ready */
            rcc_osc_on(RCC_LSI);
            rcc_wait_for_osc_ready(RCC_LSI);
            break;

        default:
        case RCC_BDCR_RTCSEL_HSE:
            /*
             * HSE is an external 4-26 MHz clock driven by either an external
             * user clock or an external crystal/ceramic resonator.  The RTC
             * divisors can handle a maximum clock of about 16 MHz.  The HSE
             * divisor must drop the HSE down to a maximum of about 512 kHz,
             * which means it's 8 at a minimum.
             */
            input_clk = CLOCK_HSE_HZ;  // Measured external clock source
            clk       = input_clk;

#define RTC_PRER_PREDIV_A_MAX (RTC_PRER_PREDIV_A_MASK - 1)
#define RTC_PRER_PREDIV_S_MAX (RTC_PRER_PREDIV_S_MASK - 1)
#ifdef RTC_HSE_BEST_FACTOR
            /* Try to find a perfect factor between 31..8 for hse_div */
            hse_div = RTC_HSE_DIV_MAX;
            min = (clk + RTC_PRER_PREDIV_S_MAX * RTC_PRER_PREDIV_A_MAX - 1) /
                  (RTC_PRER_PREDIV_S_MAX * RTC_PRER_PREDIV_A_MAX);
            if (min < RTC_HSE_DIV_MIN)
                min = RTC_HSE_DIV_MIN;
            while (hse_div >= min) {
                if ((clk % hse_div) == 0)
                    break;
                hse_div--;
            }
            if (hse_div <= min)
                hse_div = 30;  // Failed -- give up and divide by 30
            clk /= hse_div;
#else
            /* cdh: Does the RTC input clock really have to be 1 MHz?? */
            hse_div = (input_clk + 500000) / 1000000;
            clk /= hse_div;
#endif
            /*
             * HSE is selected as RTC clock source; configure HSE divisor
             * for RTC clock.
             */
            rcc_set_rtcpre(hse_div);
            break;
    }

    /* Try to find a perfect factor between 128..2 for clk */
    min = (clk + RTC_PRER_PREDIV_S_MAX - 1) / RTC_PRER_PREDIV_S_MAX;
    if (min < 1)
        min = 1;
    *prediv_async = RTC_PRER_PREDIV_A_MAX;
    while (*prediv_async >= min) {
        if ((clk % *prediv_async) == 0)
            break;
        (*prediv_async)--;
    }
    if (*prediv_async < min)
        *prediv_async = RTC_PRER_PREDIV_A_MAX;

    /* The sync predivisor can simply be calculated as the remainder */
    clk         /= *prediv_async;
    *prediv_sync = clk;
    clk_error    = input_clk - hse_div * (*prediv_async) * (*prediv_sync);

    dprintf(DF_RTC,
            "RTC input_clk    = %u Hz\n"
            "RTC hse_div      = %u\n"
            "RTC prediv_async = %u\n"
            "RTC prediv_sync  = %u\n",
            input_clk, hse_div, *prediv_async, *prediv_sync);
    if (*prediv_sync > RTC_PRER_PREDIV_S_MAX) {
        printf("FAIL: RTC prediv_sync = %u is too large\n", *prediv_sync);
        return (RC_FAILURE);
    }
    if (clk_error > 100)
        printf("RTC clk_error    = %u cycles per Hz\n", clk_error);
    else
        dprintf(DF_RTC, "RTC clk_error    = %u cycles per Hz\n", clk_error);

    /*
     * Current implementation may not be accurate
     * HSE = 8 MHz / 16
     * Sync Prediv = 0xfff; Async Prediv = 0x7f
     *     time start     03:06:36
     *     time now       11:25:06  diff=29910
     *     time reported  11:02:02  diff=28526 (-4.626%)
     *
     * Frtcclk  = HSE (8 MHz on AmigaPCI)
     * Fck_apre = Frtcclk  / (ASYNC_PREDIV + 1)
     * Fck_spre = Fck_apre / (SYNC_PREDIV + 1)
     * Fck_spre must be 1 Hz.
     */

    /*
     * clk_source bits
     *
     *                  bit
     * RCC_BDCR_LSEON   0   - External low-speed oscillator enable   (rw)
     * RCC_BDCR_LSERDY  1   - External low-speed oscillator ready    (ro)
     * RCC_BDCR_LSEBYP  2   - External low-speed oscillator bypassed (rw)
     * RTC_BDCR_RTCSEL* 9-8 - RTC clock source (0=None, 1=LSE oscillator,
     *                                2=LSI oscillator, 3=HSE oscillator)
     */
#define RTCSEL_MASK (RCC_BDCR_RTCSEL_MASK << RCC_BDCR_RTCSEL_SHIFT)
    RCC_BDCR  = (RCC_BDCR & ~RTCSEL_MASK) |
                (clk_source << RCC_BDCR_RTCSEL_SHIFT) |  // RTC clock source
                RCC_BDCR_RTCEN;  // Enable RTC

    /* Verify RTC clock source was selected */
    if (((RCC_BDCR & RTCSEL_MASK) >> RCC_BDCR_RTCSEL_SHIFT) != clk_source) {
        /* Need to reset the backup domain */
        uint32_t tr = RTC_TR;
        uint32_t dr = RTC_DR;
        tr = RTC_TR;
        dr = RTC_DR;
        dprintf(DF_RTC, "tr=%06lx dr=%06lx\n", tr, dr);
        rcc_backupdomain_reset();
        RCC_BDCR = clk_source;       // Select RTC clock source
        RCC_BDCR |= RCC_BDCR_RTCEN;  // Enable RTC
        rtc_allow_writes(TRUE);
        RTC_TR = tr;
        RTC_DR = dr;
        rtc_allow_writes(FALSE);
    }
    return (RC_SUCCESS);
}

/**
 * rtc_get_time() acquires the current time from the Real-Time Clock of
 *                the STM32F2 CPU.
 *
 * @param [out] year - The current four-digit year (1970-2106).
 * @param [out] mon  - The current month (1-12).
 * @param [out] day  - The current day of month (1-31).
 * @param [out] hour - The current hour (0-23).
 * @param [out] min  - The current minute (0-59).
 * @param [out] sec  - The current second (range: 0-59).
 * @param [out] msec - The milliseconds value (range: 0-999).
 *
 * @return      None.
 */
void
rtc_get_time(uint *year, uint *mon, uint *day,
             uint *hour, uint *min, uint *sec, uint *msec)
{
    uint64_t timeout = timer_tick_plus_msec(RTC_TIMEOUT_MS);
    uint32_t ssr;
    uint32_t tr;
    uint32_t dr;

    /* Wait until RSF is set (timestamp available) */
    while ((RTC_ISR & RTC_ISR_RSF) == 0) {
        if (timer_tick_has_elapsed(timeout)) {
            printf("RTC RSF timeout\n");
            break;
        }
    }

    /* Read RTC timestamp registers */
    ssr = RTC_SSR;            // Countdown timer for sub-seconds
    tr  = RTC_TR;             // RTC Time-stamp
    dr  = RTC_DR;             // RTC Date-stamp
    RTC_ISR &= ~RTC_ISR_RSF;

    /* Convert from RTC format to binary */
    *hour = rtc_bcd_to_binary((uint8_t)(tr >> 16) & 0x3f);
    *min  = rtc_bcd_to_binary((uint8_t)(tr >> 8) & 0x7f);
    *sec  = rtc_bcd_to_binary((uint8_t)tr & 0x7f);
    *year = rtc_bcd_to_binary((uint8_t)(dr >> 16));
    *mon  = rtc_bcd_to_binary((uint8_t)(dr >> 8) & 0x1f);
    *day  = rtc_bcd_to_binary((uint8_t)dr & 0x3f);

    /* Validate and correct time values */
    if (*hour > 23)
        *hour = 0;
    if (*min > 59)
        *min = 0;
    if (*sec > 59)
        *sec = 0;
    if (*year < 12 || *year > 99)
        *year = 24;  // Assume 2024
    if (*mon < 1 || *mon > 12)
        *mon = 1;
    if (*day < 1 || *day > 31)
        *day = 1;
    *year += 2000;

    if (msec != NULL) {
#ifdef STM32F2
        /* Millisecond resolution is not supported on STM32F2 */
        *msec = 0;
        (void) ssr;
#else
        uint ssr_divisor = rtc_prediv_sync ? rtc_prediv_sync : 1;

        *msec = 999 - (ssr * 1000 / ssr_divisor);
        if (*msec > 999)  // Underflow (shouldn't happen except for clock halt)
            *msec = 999;
#endif
    }
}

/*
 * Sum of days in each month and all prior months.  February is assumed
 * to have 28 days, so if this is a leap year, the result for any month
 * following February must be fixed up by 1 day.  Whether to add a day
 * may be determined as follows:
 *     add_day = (is_leap_year(year) && (mon > 2)) ? 1 : 0;
 */
static const uint16_t month_doy[] =
{ 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };

/*
 * is_leap_year() indicates whether the specified year is a leap year.
 *
 * @param [in]  year  - The year to test.
 *
 * @return      TRUE  - The specified year is a leap year.
 * @return      FALSE - The specified year is not a leap year.
 */
static bool_t
is_leap_year(uint year)
{
    if (((year % 4) == 0) && (((year % 100) != 0) || ((year % 400) == 0)))
        return (TRUE);
    return (FALSE);
}

/*
 * days_since_1970() returns the number of days between the start of 1970 and
 *                   the start of the specified year.  Note that the days in
 *                   the specified year are not included.  This function is only
 *                   accurate up to year 2100.
 *
 * @param [in]  year  - The year to test.
 *
 * @return      The number of days between the start of 1970 and the start
 *              of the specified year.
 */
static uint
days_since_1970(uint year)
{
    uint leap_years;

    if (year < 1971)
        return (0);

    leap_years = (year - 1969) / 4;
    return ((year - 1970) * 365 + leap_years);
}

/*
 * rtc_to_utc() converts individual values representing year, month, day,
 *              hour, minute, and second in local time format into UTC
 *              seconds since 1970 (UNIX timestamp).
 *
 * @param [in]  year - The four-digit year (1970-2106).
 * @param [in]  mon  - The month (1-12).
 * @param [in]  day  - The day of month (1-31).
 * @param [in]  hour - The hour (0-23).
 * @param [in]  min  - The minute (0-59).
 * @param [in]  sec  - The second (range: 0-59).
 *
 * @return      The UTC seconds since 1970.
 */
uint32_t
rtc_to_utc(uint year, uint mon, uint day, uint hour, uint min, uint sec)
{
    uint leap = (is_leap_year(year) && (mon > 2)) ? 1 : 0;

#if 0
    dprintf(DF_RTC, "RTC: %04u-%02u-%02u %02u:%02u:%02u leap=%u since1970=%u\n",
            year, mon, day, hour, min, sec, leap, days_since_1970(year));
#endif

    return (sec +
            60 * (min +
            60 * (hour +
            24 * (day - 1 +
            month_doy[mon - 1] + leap + days_since_1970(year)))));
}

/*
 * utc_to_rtc() converts a UTC seconds since 1970 (UNIX timestamp) value into
 *              individual values representing year, month, day, hour, minute,
 *              and second in local time format.
 *
 * @param [in]  secs - UTC seconds since 1970.
 * @param [out] year - The resulting four-digit year (1970-2106).
 * @param [out] mon  - The resulting month (1-12).
 * @param [out] day  - The resulting day of month (1-31).
 * @param [out] hour - The resulting hour (0-23).
 * @param [out] min  - The resulting minute (0-59).
 * @param [out] sec  - The resulting second (range: 0-59).
 *
 * @return      None.
 */
void
utc_to_rtc(uint32_t secs, uint *year, uint *mon, uint *day, uint *hour,
           uint *min, uint *sec)
{
    uint month;
    uint days;
    uint leap;

    /* Extract seconds, minutes, and hours */
    *sec  = secs % 60;
    secs /= 60;
    *min  = secs % 60;
    secs /= 60;
    *hour = secs % 24;
    days  = secs / 24;  // Days since 1970

    /* Extract year. Optimize for dates after 2025. */
    if (days >= DAYS_2024) {
        *year = 2024;
        leap = 1;
        days -= DAYS_2024;
    } else {
        *year = 1970;
        leap = 0;
    }
    while (days > 365 + leap) {
        days -= (365 + leap);
        (*year)++;
        leap = is_leap_year(*year) ? 1 : 0;
    }
    leap = is_leap_year(*year);

    /* Locate current month */
    for (month = 1; month < 12; month++)
        if (month_doy[month] + ((month >= 2) ? leap : 0) >= days)
            break;
    *mon = month;

    /* Current day is remainder (one-based, not zero-based) */
    *day = days - month_doy[month - 1] - ((month > 2) ? leap : 0);
}

/*
 * time_get_utc() returns the current UTC seconds since 1970 (UNIX timestamp)
 *                and milliseconds values. This function uses the STM32 RTC,
 *                which is synced at second resolution to the Amiga clock.
 *                Although the RTC stops ticking across a reset, it does
 *                provide a starting point. For higher resolution timestamps,
 *                the time_get_utc_usec() function is preferred.
 *
 * @param [out] msec - Resulting milliseconds value (if not NULL).
 *
 * @return      UTC seconds since 1970.
 * @see         time_get_utc_usec().
 */
static uint32_t
time_get_utc(uint *msec)
{
    uint year;
    uint mon;
    uint day;
    uint hour;
    uint min;
    uint sec;

    rtc_get_time(&year, &mon, &day, &hour, &min, &sec, msec);

    return (rtc_to_utc(year, mon, day, hour, min, sec));
}

#if 0
/**
 * time_get_utc_usec() returns the number of microseconds since 1970 from
 *                     the high resolution timer.
 *
 * @return      The number of microseconds since the UNIX epoch.
 * @see         time_get_utc().
 */
uint64_t
time_get_utc_usec(void)
{
    uint64_t current_usec = timer_hr_to_usec(timer_hr_get());
    int64_t usec_elapsed = current_usec - last_drift_update;
    int64_t added_skew;
    if (sync_interval != 0) {
        added_skew = tick_drift * usec_elapsed / sync_interval / 1000000;
    } else {
        added_skew = 0;
    }
    if (((tick_drift > 0) && (added_skew > tick_drift)) ||
        ((tick_drift < 0) && (added_skew < tick_drift))) {
        added_skew = tick_drift;
    }
    return (timer_usec_base + added_skew + current_usec);
}
#endif

/*
 * time_str() converts seconds and milliseconds time values into a string
 *            composed of the current date and time.  The seconds value is
 *            in UNIX time format (seconds since 1970).  The format is as
 *            follows:  yyyy-mm-dd hh:mm:ss.MMM
 *
 * @param [in]  secs   - The time as seconds since 1970.
 * @param [in]  msec   - A milliseconds addition to seconds.  Specify
 *                       a value of 1000 or higher to disable output
 *                       of the milliseconds value.
 * @param [out] buf    - A buffer for the time string.
 * @param [in]  buflen - The length of the buffer.
 *
 * @return      Time string (same pointer as buf)
 */
char *
time_str(uint32_t secs, uint32_t msec, char *buf, size_t buflen)
{
    uint year;
    uint mon;
    uint day;
    uint hour;
    uint min;
    uint sec;

    utc_to_rtc(secs + utc_offset, &year, &mon, &day, &hour, &min, &sec);

#ifdef STM32F2
    /* STM32F2xx RTC does not have millisecond resolution */
    msec = TIME_STR_NO_MSEC;
#endif
    /* Milliseconds output format desired */
    if (msec < TIME_STR_NO_MSEC) {
        (void) snprintf(buf, buflen, "%04u-%02u-%02u %02u:%02u:%02u.%03u",
                        year, mon, day, hour, min, sec, (uint) msec);
    } else {
        (void) snprintf(buf, buflen, "%04u-%02u-%02u %02u:%02u:%02u",
                        year, mon, day, hour, min, sec);
    }
    return (buf);
}

/**
 * rtc_print() displays the current wall clock time of the STM32F2 RTC.
 *             This function is called for the user "time" command.
 *
 * @param [in]  which - 0 = physical RTC
 *                      1 = converted time
 * @param [in]  newline - End output with a newline.
 *
 * @return      None.
 */
void
rtc_print(uint which, uint newline)
{
    char buf[32];
    uint sec;
    uint msec;

    if (which == 0) {
        uint year;
        uint mon;
        uint day;
        uint hour;
        uint min;
        uint leap;
        rtc_get_time(&year, &mon, &day, &hour, &min, &sec, &msec);
        leap = (is_leap_year(year) && (mon > 2)) ? 1 : 0;
        printf("%04u-%02u-%02u %02u:%02u:%02u leap=%u since1970=%u",
                year, mon, day, hour, min, sec, leap, days_since_1970(year));
    } else {
        sec = time_get_utc(&msec);
        time_str(sec, msec, buf, sizeof (buf));
        printf("%s (0x%08x)", buf, sec);
    }
    if (newline)
        putchar('\n');
}

#if 0
/**
 * update_clock_skew() updates values for the drift that took place between
 *                     the Amiga and hr timer over the last sync_interval and
 *                     the most recent sync_interval value.
 *
 * @param [in]  drift - Difference between the Amiga and hr timer in
 *                      microseconds.
 * @param [in]  tick_sync_interval - Time since the last Amiga time sync in
 *                                 seconds.
 * @param [in]  t_usec - Current high resolution time in microseconds.
 *
 * @return      None.
 */
void
update_clock_skew(int64_t drift, int32_t tick_sync_interval, uint64_t t_usec)
{
    if ((drift > 1000000) || (drift < -1000000))
        drift = tick_drift;  // Drift > 1 second is treated as a time set

    /* Average in drift to reduce oscillation */
    tick_drift = (drift + tick_drift) / 2;

    sync_interval = tick_sync_interval;
    last_drift_update = t_usec;
    dprintf(DF_RTC, "update_clock_skew %lld %d %llu\n", (long long) drift,
            tick_sync_interval, (long long) t_usec);
}
#endif

/*
 * rtc_read_nvram()
 *
 * @param [in]  reg - NVRAM register (0 .. 10)
 *
 * @return      value - Value stored in battery-backed NVRAM.
 */
uint32_t
rtc_read_nvram(uint reg)
{
    return (RTC_BKPXR(reg));
}

/*
 * rtc_read_nvram()
 *
 * @param [in]  reg   - NVRAM register (0 .. 10)
 * @param [in]  value - Value to write to battery-backed NVRAM.
 */
void
rtc_write_nvram(uint reg, uint32_t value)
{
    RTC_BKPXR(reg) = value;
}

/**
 * time_calc_summary() returns a human-readable idea of time drift.
 *
 * @param [in]  secs  - Number of elapsed seconds.
 * @param [in]  drift - Number of microseconds drift.
 *
 * @param [out] buf   - Output string.
 */
static void
time_calc_summary(uint secs, int64_t drift, char *buf)
{
    int64_t calc;
    int64_t value;
    if ((drift == 0) || (secs == 0)) {
        strcpy(buf, "no drift");
        return;
    }
    calc = drift * 60 * 60;         // 1 hour
    value = calc / secs / 1000000;  // drift per hour
    if ((value >= 30) || (value <= -30)) {
        sprintf(buf, "%lld sec/hour", value);
        return;
    }
    calc *= 24;                     // 24 hours
    value = calc / secs / 1000000;  // drift per day
    if ((value >= 10) || (value <= -10)) {
        sprintf(buf, "%lld sec/day", value);
        return;
    }
    calc = calc * 365 + calc / 4;   // 1 year
    value = calc / secs / 1000000;  // drift per year
    sprintf(buf, "%lld sec/year", value);
}

/**
 * rtc_calibrate() enables 512 Hz PC13 for RTC calibration and starts a
 *                 running comparison the real-time clock (RTC) against the
 *                 high speed tick, reporting a running difference and
 *                 per-second difference in microseconds.
 *
 * @param       None.
 *
 * @return      None.
 */
void
rtc_calibrate(void)
{
    int64_t  running_diff = 0;
    int64_t  instant_diff = 0;
    uint64_t tick_now;
    uint64_t tick_last;
    uint     count = 0;
    uint     sec;
    uint     saw_first = 0;
    uint64_t diff_usec;
    uint     sec_last = RTC_TR & 0x7f;
    uint     secs = 0;
    char     summary[64];
    uint     enable_calibrate = gpio_get(GPIOC, GPIO13);
    int      ch;

    if (enable_calibrate) {
        printf("500 Hz calibration output is present on PC13 _STMRSTA\n");
        rtc_set_calibrate(1);
    } else {
        printf("_STMRSTA is low: can not enable calibration output\n");
    }
    printf("Comparing RTC with tick (cumulative and instantaneous). "
           "Press ^C to end.\n");
    tick_last = timer_tick_get();
    while (1) {
        sec = RTC_TR & 0x7f;
        if (sec_last != sec) {
            sec_last = sec;
            tick_now = timer_tick_get();
            printf("\r");
            rtc_print(1, 0);
            if (saw_first) {
                diff_usec = timer_tick_to_usec(tick_now - tick_last);
                instant_diff = diff_usec - 1000000;
                running_diff += instant_diff;
                secs++;
                time_calc_summary(secs, running_diff, summary);
                printf("  [%s] sum=%lld diff=%lld       ",
                       summary, running_diff, instant_diff);
            } else {
                saw_first = 1;
            }
            tick_last = tick_now;
            goto check_abort;
        }
        if (count++ > 10000000) {
            printf(",");
check_abort:
            if (input_break_pending()) {
                printf("^C\n");
                break;
            }
            ch = getchar();
            switch (ch) {
                case '+':
                    rtc_set_calibrate_offset(1);
                    break;
                case '-':
                    rtc_set_calibrate_offset(-1);
                    break;
                case '=':
                    rtc_set_calibrate_offset(0);
                    break;
                case '[':
                    rtc_set_calibrate_offset(-10);
                    break;
                case ']':
                    rtc_set_calibrate_offset(10);
                    break;
                case '{':
                    rtc_set_calibrate_offset(-40);
                    break;
                case '}':
                    rtc_set_calibrate_offset(40);
                    break;
            }
            count = 0;
        }
    }
    if (enable_calibrate)
        rtc_set_calibrate(0);
}

/*
 * rtc_init() initializes and enables the STM32F2 Real-Time Clock hardware.
 *
 * This function requires no arguments.
 *
 * @return      RC_SUCCESS - Initialization completed successfully.
 * @return      RC_FAILURE - Invalid input clock for RTC divisor.
 * @return      RC_TIMEOUT - RTC did not enter INIT mode.
 */
void
rtc_init(void)
{
    rc_t rc;
    uint prediv_async;
    uint clk_source;
    uint backup_source;

    /* Verify the HSE is enabled and stable */
    if ((RCC_CR & RCC_CR_HSEON) && (RCC_CR & RCC_CR_HSERDY)) {
        backup_source = RCC_BDCR_RTCSEL_HSE;
    } else {
        /* Fall back to LSI for the RTC source if the HSE is not stable */
        backup_source = RCC_BDCR_RTCSEL_LSI;
    }

    clk_source = RCC_BDCR_RTCSEL_LSE;
    if (config.board_type == BOARD_TYPE_KEYJAM)
        clk_source = RCC_BDCR_RTCSEL_HSE;

    /*
     * After a reset, the backup domain (RTC registers, RTC backup data
     * registers and backup SRAM) is protected against any possible unwanted
     * write access.  To access the backup domain, we first must enable
     * its clock.
     */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);

    /*
     * Disable backup domain write protection (enable writes to the RTC).
     */
    PWR_CR |= PWR_CR_DBP;

    /* Configure and enable RTC clock */
    rc = rtc_clock_config(clk_source, &prediv_async, &rtc_prediv_sync);
    if (rc != RC_SUCCESS)
        rc = rtc_clock_config(backup_source, &prediv_async, &rtc_prediv_sync);
    if (rc != RC_SUCCESS)
        return;

    /* Disable write protection for RTC registers */
    rtc_allow_writes(TRUE);

    /* Set Initialization mode */
    if ((rc = rtc_init_mode(TRUE)) != RC_SUCCESS)
        return;

    /* Set the time format to 24-hour */
    RTC_CR &= ~RTC_CR_FMT;

    /* Disable PC13 500 Hz clock calibration output */
    RTC_CR &= ~(RTC_CR_COE);

    RTC_CR &= ~(RTC_CR_WUTIE | RTC_CR_WUTE);    // Disable Wakeup timer
    RTC_CR &= ~(RTC_CR_ALRAIE | RTC_CR_ALRAE);  // Disable Alarm A
    RTC_CR &= ~(RTC_CR_ALRBIE | RTC_CR_ALRBE);  // Disable Alarm B

    /* Wait for ALRAWF, ALRBWF, and WUTWF to be set */
    uint count = 0;
    while ((RTC_ISR & 7) != 7)
        if (count++ > 10000000)
            break;
    if ((RTC_ISR & 7) != 7) {
        printf("RTC_ISR=%lx\n", RTC_ISR);
    }
    // Alarm values can now be changed

    /*
     * Set up wake-up timer.
     * The RTC clock is 32768 Hz, divided by 16 is 2048 Hz.
     * If wakeup occurs every 32 clocks, that is 256 Hz, which is
     * an interval of every 3.9 msec.
     */
    RTC_CR = (RTC_CR & ~7) | 0;          // RTC/16 clock selected
    RTC_WUTR = 32 - 1;                   // Wakeup interval (32 clocks)

    RTC_ALRMAR =  // RTC_ALRMXR_MSK1 |  // Ignore seconds
                  RTC_ALRMXR_MSK2 |  // Ignore minutes
                  RTC_ALRMXR_MSK3 |  // Ignore hours
                  RTC_ALRMXR_MSK4;   // Ignore day

    RTC_ALRMBR =  // RTC_ALRMXR_MSK1 |  // Ignore seconds
                  RTC_ALRMXR_MSK2 |  // Ignore minutes
                  RTC_ALRMXR_MSK3 |  // Ignore hours
                  RTC_ALRMXR_MSK4;   // Ignore day

    RTC_CR |= RTC_CR_WUTIE | RTC_CR_WUTE;

    RTC_CR |= RTC_CR_TSIE;    // Timestamp interrupt enable
    RTC_CR |= RTC_CR_WUTIE;   // Wake up interrupt enable
    RTC_CR |= RTC_CR_ALRAIE;  // Alarm A interrupt enable
    RTC_CR |= RTC_CR_ALRBIE;  // Alarm B interrupt enable

    RTC_CR |= RTC_CR_TSE;     // Timestamp enable
    RTC_CR |= RTC_CR_WUTE;    // Wake up timer enablke
    RTC_CR |= RTC_CR_ALRAE;   // Alarm A enable
    RTC_CR |= RTC_CR_ALRBE;   // Alarm B enable

    /* Configure the RTC prescaler (MUST be done as two separate writes) */
    RTC_PRER  = ((prediv_async - 1) << 16);
    RTC_PRER |= (rtc_prediv_sync - 1);


    /* Exit Initialization mode */
    (void) rtc_init_mode(FALSE);

#ifdef WIPE_DATE
    /*
     * XXX: Only set the date if the current value is invalid.
     */
    rtc_set_datetime(12, 1, 1, 0, 0, 0);
#endif

    /* Enable write protection for RTC registers */
    rtc_allow_writes(FALSE);

    /* Setup the RTC interrupts */
    nvic_set_priority(NVIC_RTC_WKUP_IRQ, 0x30);
    nvic_enable_irq(NVIC_RTC_WKUP_IRQ);
#if 0
    nvic_set_priority(NVIC_RTC_ALARM_IRQ, 0x30);
    nvic_enable_irq(NVIC_RTC_ALARM_IRQ);
#endif

    /* Clear interrupt status */
    RTC_ISR &= ~(RTC_ISR_TSOVF | RTC_ISR_TSF | RTC_ISR_WUTF |
                 RTC_ISR_ALRAF | RTC_ISR_ALRBF);

    /* Enable battery backup of SRAM */
    PWR_CSR |= PWR_CSR_BRE;

    rtc_print(1, 1);
}
