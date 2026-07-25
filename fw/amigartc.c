#include <stdint.h>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/f2/rtc.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

#include <string.h>
#include "printf.h"
#include "main.h"
#include "amigartc.h"
#include "bec_cmd.h"
#include "crc32.h"
#include "config.h"
#include "gpio.h"
#include "uart.h"
#include "utils.h"
#include "rtc.h"
#include "timer.h"
#include "cmdline.h"
#include "msg.h"
#include "power.h"
#include "utils.h"
#include "version.h"

/* Enable RP5C01 response */
#define RESPOND_AS_RP5C01

/* RP5C01 Debug output */
#undef DEBUG_RP5C01_ACCESS
#ifdef DEBUG_RP5C01_ACCESS
#define DPRINTF(x...) printf(x)
#else
#define DPRINTF(x...) do { } while (0)
#endif

#define RP_MAGIC_HI 0
#define RP_MAGIC_LO 1

/* Enable capture of RP5C01 accesses in interrupt handler */
#define INTERRUPT_CAPTURE_RP5C01

#define likely(x)   __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

#define RAM_FUNCTION __attribute__((section(".ramtext")))

#ifdef INTERRUPT_CAPTURE_RP5C01
static uint16_t amigartc_cap[256];
static uint16_t amigartc_tick[256];
static uint8_t  amigartc_prod;
#endif

volatile uint8_t rtc_data[4][0x10];
volatile uint8_t rtc_cur_bank;
static volatile uint8_t rtc_timer_en;
static volatile uint8_t rtc_touched;
static volatile uint8_t rtc_ram_touched;

/*
 * RP5C01 TEST and RESET registers always read as 0, regardless of
 * what was written there.
 */
static const uint8_t rtc_mask[4][0x10] = {
    { 0xf, 0x7, 0xf, 0x7, 0xf, 0x3, 0x7, 0xf,
      0x3, 0xf, 0x1, 0xf, 0xf, 0xf, 0x0, 0x0 },
    { 0x0, 0x0, 0xf, 0x7, 0xf, 0x3, 0x7, 0xf,
      0x3, 0x0, 0x1, 0x3, 0x0, 0xf, 0x0, 0x0 },
    { 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf,
      0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0, 0x0 },
    { 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf,
      0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0, 0x0 },
};

static const uint8_t bec_magic[] = { 0xc, 0xd, 0x6, 0x8 };

uint8_t         bec_msg_inbuf[280];
uint8_t         bec_msg_outbuf[280];
uint            bec_msg_out_max;    // Message length in nibbles
uint            bec_msg_out;        // Current send position in nibbles
uint            bec_msg_in;         // Current receive position in nibbles
static uint64_t bec_msg_in_timeout;
uint64_t        bec_msg_out_timeout;
char            bec_errormsg_delayed[80];  // Error message for slow path


/*
 * RP5C01 registers (each register is 4 bits)
 *
 *      MODE0              MODE1                MODE2   MODE3   Amiga Addr
 *   0  1-second counter   x (0)                RAM     RAM     dc0001
 *   1  10-second counter  x (0)                RAM     RAM     dc0005
 *   2  1-minute counter   1-minute alarm       RAM     RAM     dc0009
 *   3  10-minute counter  10-minute alarm      RAM     RAM     dc000d
 *   4  1-hour counter     1-hour alarm         RAM     RAM     dc0011
 *   5  10-hour counter    10-hour alarm        RAM     RAM     dc0015
 *   6  day-of-week count  day-of-week alarm    RAM     RAM     dc0019
 *   7  1-day counter      1-day alarm          RAM     RAM     dc001d
 *   8  10-day counter     10-day alarm         RAM     RAM     dc0021
 *   9  1-month counter    x (0)                RAM     RAM     dc0025
 *   A  10-month counter   12/24 hour+AM/PM     RAM     RAM     dc0029
 *   B  1-year counter     Leap year counter    RAM     RAM     dc002d
 *   C  10-year counter    x (0 for AmigaOS)    RAM     RAM     dc0031
 *   D  MODE register      MODE register        ditto   ditto   dc0035
 *   E  TEST register      TEST register        ditto   ditto   dc0039
 *   F  RESET register     RESET register       ditto   ditto   dc003d
 *
 * Control registers    D3           D2           D1           D0
 * D  MODE register     TIMER_EN     ALARM_EN     MODE_M1      MODE_M0
 * E  TEST register     TEST3        TEST2        TEST1        TEST0
 * F  RESET register    1 Hz         16 Hz        TIMER_RESET  ALARM_RESET
 *
 * D  MODE register     TIMER_EN    ALARM_EN    MODE_M1     MODE_M0
 *      TIMER_EN=1 to enable the RTCc counters
 *                 When this bit is 0, the RTC registers will stop updates.
 *                 This is so a user program can acquire a consistent time.
 *      ALARM_EN=1 to enable alarm output (not connected on Amiga)
 *      M1=0 M0=0  MODE00: Setting and read of time
 *      M1=0 M0=1  MODE01: Setting and read of alarm, 12/24 hour, and leap year
 *      M1=1 M0=0  MODE10: RAM Block10 (12 registers x 4 bits)
 *      M1=1 M0=1  MODE11: RAM Block11 (12 registers x 4 bits)
 *
 *      Documentation implies that setting TIMER_EN=0 for less than a second
 *      will not result in loss of time (the lower order tick still occurs,
 *      and will be applied after TIMER_EN=1)
 *
 * TEST register
 *      Undocumented
 *
 * RESET register
 *      D0=1 reset all alarm registers
 *      D1=1 reset divider stages for second and smaller units
 *      D2=0 16Hz pulse at ALARM output pin
 *      D3=0 1Hz pulse at ALARM output pin
 *
 * 12/24 hour+AM/PM register
 *      D0=1 for 24-hour system; D0=0 for 12-hour system
 *      D1=1 for p.m. (12-hour system); D1=0 for a.m. (12-hour system)
 *
 * Leap year counter
 *      Counter for years since last leap year (valid values 0-3)
 *      00 = This year is a leap year
 *
 *
 * AmigaPCI STM32 magic message interface is on MODE1 register 9
 * When no message is pending from STM32, all reads of MODE 1 register 9
 * will return 0.
 * See ap_cmd.h for additional details.
 */


// GPIO_IDR(A4_PORT);
//
// set 1:
//      GPIO_BSRR(GPIOx) = GPIO_Pins;
// set 0:
//      GPIO_BSRR(GPIOx) = GPIO_Pins << 16;

static inline void
set_rtc_dx_output(void)
{
    /*
     * MODER uses 2 bits for every GPIO.
     *
     * D16_PIN | D17_PIN | D18_PIN | D19_PIN
     * xxxx xxxx 1111 xxxx -> xxxx xxxx xxxx xxxx 0101 0101 xxxx xxxx
     */
    GPIO_MODER(D16_PORT) = (GPIO_MODER(D16_PORT) & ~0x0000ff00) | 0x00005500;
}

static inline void
set_rtc_dx_input(void)
{
    GPIO_MODER(D16_PORT) &= ~0x0000ff00; // (D16_PIN .. D19_PIN) * 2 bits
}

static void
amigartc_snoop_show(uint16_t *cap, uint count, int debug)
{
    uint cur;
    printf("Addr=");
    for (cur = 0; cur < count; cur++) {
        if (debug || ((cap[cur] & RTCEN_PIN) == 0))
            printf("%x", (cap[cur] >> 10) & 0xf);  // A2 = GPIO8
        else
            putchar(' ');
    }
    printf("\nData=");
    for (cur = 0; cur < count; cur++) {
        if (debug || ((cap[cur] & RTCEN_PIN) == 0))
            printf("%x", (cap[cur] >> 4) & 0xf);  // D16 = GPIO4
        else
            putchar(' ');
    }
    printf("\nOper=");
    for (cur = 0; cur < count; cur++) {
        if ((cap[cur] & RTCEN_PIN) != 0)
            putchar(' ');
        else if ((cap[cur] & R_WA_PIN))
            putchar('R');
        else
            putchar('W');
    }
    printf("\n");
}

void
amigartc_snoop(int debug)
{
    uint gpio_value;
    uint gpio_last = 0;
    uint count = 0;
    uint timeout = 2000000;
    uint first_cap = 1;
    uint16_t cap[512];
    printf("Watching A2-A5 and D16-D19. Press ^C to end\n");

    nvic_disable_irq(NVIC_EXTI0_IRQ);

    while (1) {
        if (--timeout == 0) {
dump_cap:
            if (input_break_pending()) {
                printf("^C\n");
                break;
            }
            if (count > first_cap) {
                amigartc_snoop_show(cap, count, debug);
                count = 0;
                first_cap = 0;
            }
            timeout = 2000000;
        }
        gpio_value = GPIO_IDR(A4_PORT);
        gpio_value &= (A2_PIN | A3_PIN | A4_PIN | A5_PIN |
                       D16_PIN | D17_PIN | D18_PIN | D19_PIN |
                       R_WA_PIN | RTCEN_PIN);
        if (((debug > 1) && (gpio_last != gpio_value)) ||
            ((((gpio_value & RTCEN_PIN) == 0) ||
             ((gpio_last & RTCEN_PIN) == 0)) &&
            (gpio_last != gpio_value))) {
            gpio_last = gpio_value;
            cap[count++] = gpio_value;
            if (count == ARRAY_SIZE(cap))
                goto dump_cap;  // No more space
        }
    }
#ifdef RESPOND_AS_RP5C01
    nvic_enable_irq(NVIC_EXTI0_IRQ);
#endif
}

/*
 * amigartc_log() displays the Amiga RTC access log
 */
void
amigartc_log(void)
{
#ifdef INTERRUPT_CAPTURE_RP5C01
    uint cur = (amigartc_prod + 1) % ARRAY_SIZE(amigartc_cap);
    while (cur != amigartc_prod) {
        if ((amigartc_cap[cur] & RTCEN_PIN) == 0) {
            uint16_t v = amigartc_cap[cur];

            printf("%5u %04x %c %x = %x\n", amigartc_tick[cur], v,
                   (v & R_WA_PIN) ? 'R' : 'W',
                   (v >> 10) & 0xf,
                   (v >> 4) & 0xf);
        }
        cur = (cur + 1) % ARRAY_SIZE(amigartc_cap);
    }
    memset(&amigartc_cap, 0xff, sizeof (amigartc_cap));
#else
    printf("RTC capture not enabled at compile-time\n");
#endif
}

RAM_FUNCTION
void
exti0_isr(void)
{
    uint count = 0;
    uint32_t gpio_value = (volatile uint32_t) GPIO_IDR(A4_PORT);
    uint32_t ngpio_value;

#define exti_reset_request(x) EXTI_PR = x
    exti_reset_request(EXTI0);

    /* This loop can service back-to-back read and write requests */
    while ((gpio_value & RTCEN_PIN) == 0) {
        if (likely(gpio_value & R_WA_PIN)) {
            /* Amiga reading from RP5C01 */
            set_rtc_dx_output();
            uint addr = (gpio_value >> 10) & 0xf;
            uint data = rtc_data[rtc_cur_bank][addr];
            GPIO_BSRR(D16_PORT) = 0x00f00000 | (data << 4);  // Set data pins
            gpio_value = (gpio_value & ~0x00f0) | (data << 4);

            /* Wait for _RTCEN to deassert */
            count = 0;
            while ((GPIO_IDR(A4_PORT) & RTCEN_PIN) == 0)
                if (count++ == 100) {
                    if (power_state == POWER_STATE_ON)
                        printf("H");
                    break;
                }

            if ((addr == RP_MAGIC_LO) && (rtc_cur_bank == 1) &&
                (bec_msg_out != 0)) {
                bec_msg_out += 2;
                if (bec_msg_out < bec_msg_out_max) {
                    data = bec_msg_outbuf[bec_msg_out / 2];
                    rtc_data[1][RP_MAGIC_HI] = data >> 4;
                    rtc_data[1][RP_MAGIC_LO] = data & 0xf;
                } else if (bec_msg_out >= bec_msg_out_max) {
                    bec_msg_out = 0;  // End of message
                    rtc_data[1][RP_MAGIC_HI] = 0;
                    rtc_data[1][RP_MAGIC_LO] = 0;
                }
            }
        } else {
            /* Amiga writing to RP5C01 */
            uint addr;
            uint data;
            uint bank;

            /* If data pins were being driven, flip them to inputs */
            if ((GPIO_MODER(D16_PORT) & 0x00ff00) != 0) {
                set_rtc_dx_input();  // Stop driving data pins
                __asm__ volatile("dmb");
                printf("M");
                gpio_value = GPIO_IDR(A4_PORT);
            }
            addr = (gpio_value >> 10) & 0xf;
            data = (gpio_value >> 4) & 0xf;
            bank = rtc_cur_bank;
            rtc_data[bank][addr] = data & rtc_mask[bank][addr];
            switch (addr) {
                default:
                    if ((bank == 2) || (bank == 3))
                        rtc_ram_touched++;
                    else
                        rtc_touched++;
                    break;
                case 0x0:  // AmigaPCI STM32 message interface
                    if (bank == 1) {
                        bec_msg_inbuf[bec_msg_in / 2] = (data << 4);
                    }
                    break;

                case 0x1:  // AmigaPCI STM32 message interface
                    if (bank == 1) {
                        if (bec_msg_in < 4) {
                            if ((bec_msg_inbuf[bec_msg_in / 2] !=
                                 (bec_magic[bec_msg_in] << 4)) ||
                                (data != bec_magic[bec_msg_in + 1])) {
                                bec_msg_in = 0;
                                break;
                            }
                        }
                        if (bec_msg_in >= ARRAY_SIZE(bec_msg_inbuf) * 2) {
                            uint expected = BEC_MSG_HDR_LEN +
                                            BEC_MSG_CRC_LEN + bec_msg_inbuf[3];
                            if (bec_msg_in / 2 >= expected)
                                break;  // Got message
                            bec_msg_in = 0;
                            break;
                        }
                        bec_msg_inbuf[bec_msg_in / 2] |= data;
                        bec_msg_in += 2;
                        if (bec_msg_in / 2 >=
                            (BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN)) {
                            uint expected = BEC_MSG_HDR_LEN +
                                            BEC_MSG_CRC_LEN +
                                            ((bec_msg_inbuf[3] << 8) |
                                             bec_msg_inbuf[4]);
                            msg_source = 0;
                            if ((bec_msg_in / 2 >= expected) &&
                                msg_process_fast()) {
                                bec_msg_in = 0;
                                bec_msg_in_timeout = 0;
                            }
                        }
                    }
                    break;
                case 0x0d:  // MODE register
#define RTC_MODE_TIMER_EN    BIT(3)
#define RTC_MODE_TIMER_MODE (BIT(0) | BIT(1))
                    rtc_cur_bank = data & RTC_MODE_TIMER_MODE;
                    rtc_timer_en = data & RTC_MODE_TIMER_EN;
                    rtc_data[0][addr] = data;
                    rtc_data[1][addr] = data;
                    rtc_data[2][addr] = data;
                    rtc_data[3][addr] = data;
                    break;
                case 0x0e:  // TEST register
                    break;
                case 0x0f:  // RESET register
                    if (data & BIT(0)) {
                        /* Reset alarm registers */
                        rtc_data[0][0xd] &= ~BIT(2);  // ALARM_EN
                        rtc_data[1][0xd]  = rtc_data[0][0xd];
                        rtc_data[2][0xd]  = rtc_data[0][0xd];
                        rtc_data[3][0xd]  = rtc_data[0][0xd];
                        rtc_data[1][0x2]  = 0;  // 1-minute alarm
                        rtc_data[1][0x3]  = 0;  // 10-minute alarm
                        rtc_data[1][0x4]  = 0;  // 1-hour alarm
                        rtc_data[1][0x5]  = 0;  // 10-hour alarm
                        rtc_data[1][0x6]  = 0;  // day-of-week alarm
                        rtc_data[1][0x7]  = 0;  // 1-day alarm
                        rtc_data[1][0x8]  = 0;  // 10-day alarm
                    }
                    if (data & BIT(1)) {
                        /* Reset cascade register triggered */
                        amigartc_reset();
                    }
                    break;
            }

            /* Wait for _RTCEN to deassert */
            count = 0;
            while ((GPIO_IDR(A4_PORT) & RTCEN_PIN) == 0)
                if (count++ == 100) {
                    if (power_state == POWER_STATE_ON)
                        printf("H");
                    break;
                }
            DPRINTF(" %x=%x", addr, data);
        }
#ifdef INTERRUPT_CAPTURE_RP5C01
        amigartc_cap[amigartc_prod] = gpio_value;
        amigartc_tick[amigartc_prod] = TIM_CNT(TIM2);
        amigartc_prod = (amigartc_prod + 1) % ARRAY_SIZE(amigartc_cap);
#endif
        set_rtc_dx_input();  // Stop driving data pins
        return;

        gpio_value = ngpio_value;
        if ((GPIO_IDR(STMRSTA_PORT) & STMRSTA_PIN) == 0) {
            break;  // Amiga in reset or off -- stop interrupt service
        }
    }

    set_rtc_dx_input();  // Stop driving data pins
}

/*
 * STM32 RTC RAM map (must be writen as 32-bit values)
 *
 * RTC_BKPXR(x): 20 x 32-bit values
 *     0: Magic number 0xafc05039
 *     1: RP5C01 Alarm date/time
 *          Bits 0-3   alarm 1-minute
 *          Bits 4-7   alarm 10-minute
 *          Bits 8-11  alarm 1-hour
 *          Bits 12-15 alarm 10-hour
 *          Bits 16-19 alarm day-of-week
 *          Bits 20-23 alarm 1-day
 *          Bits 24-27 alarm 10-day
 *     2: RP5C01 SRAM BLOCK 10, first half
 *          Bits 0-3    A=0   Bits 16-19  A=4
 *          Bits 4-7    A=1   Bits 20-23  A=5
 *          Bits 8-11   A=2   Bits 24-27  A=6
 *          Bits 12-15  A=3   Bits 28-31  A=7
 *     3: RP5C01 SRAM BLOCK 10, second half
 *          Bits 0-3    A=8   Bits 12-15  A=B
 *          Bits 4-7    A=9   Bits 16-19  A=C
 *          Bits 8-11   A=A
 *     4: RP5C01 SRAM BLOCK 11, first half
 *          Bits 0-3    A=0   Bits 16-19  A=4
 *          Bits 4-7    A=1   Bits 20-23  A=5
 *          Bits 8-11   A=2   Bits 24-27  A=6
 *          Bits 12-15  A=3   Bits 28-31  A=7
 *     5: RP5C01 SRAM BLOCK 11, second half
 *          Bits 0-3    A=8   Bits 12-15  A=B
 *          Bits 4-7    A=9   Bits 16-19  A=C
 *          Bits 8-11   A=A
 */
#define RTC_RAM_MAGIC 0xafc05039
static void
amigartc_copy_ram_rp5c01_to_stm32(void)
{
    rtc_allow_writes(TRUE);
    RTC_BKPXR(0) = ~RTC_RAM_MAGIC;
    RTC_BKPXR(2) = rtc_data[2][0] |
                   (rtc_data[2][1] << 4) |
                   (rtc_data[2][2] << 8) |
                   (rtc_data[2][3] << 12) |
                   (rtc_data[2][4] << 16) |
                   (rtc_data[2][5] << 20) |
                   (rtc_data[2][6] << 24) |
                   (rtc_data[2][7] << 28);
    RTC_BKPXR(3) = rtc_data[2][8] |
                   (rtc_data[2][9] << 4) |
                   (rtc_data[2][10] << 8) |
                   (rtc_data[2][11] << 12) |
                   (rtc_data[2][12] << 16);
    RTC_BKPXR(4) = rtc_data[3][0] |
                   (rtc_data[3][1] << 4) |
                   (rtc_data[3][2] << 8) |
                   (rtc_data[3][3] << 12) |
                   (rtc_data[3][4] << 16) |
                   (rtc_data[3][5] << 20) |
                   (rtc_data[3][6] << 24) |
                   (rtc_data[3][7] << 28);
    RTC_BKPXR(5) = rtc_data[3][8] |
                   (rtc_data[3][9] << 4) |
                   (rtc_data[3][10] << 8) |
                   (rtc_data[3][11] << 12) |
                   (rtc_data[3][12] << 16);
    RTC_BKPXR(0) = RTC_RAM_MAGIC;
    rtc_allow_writes(FALSE);
}

static void
amigartc_copy_time_rp5c01_to_stm32(void)
{
    uint second   = rtc_data[0][0]  + rtc_data[0][1]  * 10;
    uint minute   = rtc_data[0][2]  + rtc_data[0][3]  * 10;
    uint hour     = rtc_data[0][4]  + rtc_data[0][5]  * 10;
    uint day      = rtc_data[0][7]  + rtc_data[0][8]  * 10;
    uint month    = rtc_data[0][9]  + rtc_data[0][10] * 10;
    uint year     = rtc_data[0][11] + rtc_data[0][12] * 10;
    uint dow      = rtc_data[0][6];
    uint hour_24  = rtc_data[1][0xa] & BIT(0);
    uint am_pm    = rtc_data[1][0xa] & BIT(1);

    if ((rtc_data[0][1] > 5) ||        // second
        (rtc_data[0][0] > 9) ||        // second
        (second > 59) ||               // second
        (rtc_data[0][3] > 5) ||        // minute
        (rtc_data[0][2] > 9) ||        // minute
        (minute > 59) ||               // minute
        (rtc_data[0][5] > 2) ||        // hour
        (rtc_data[0][4] > 9) ||        // hour
        (hour > 23) ||                 // hour
        (rtc_data[0][8] > 3) ||        // day
        (rtc_data[0][7] > 9) ||        // day
        (day < 1) || (day > 31) ||     // day
        (rtc_data[0][10] > 1) ||       // month
        (rtc_data[0][9] > 9) ||        // month
        (month < 1) || (month > 12) || // month
        (year < 25) || (year > 30) ||  // year
        (rtc_data[0][12] > 9) ||       // year
        (rtc_data[0][11] > 9)) {       // year
        printf("Not saving invalid Amiga RTC date\n");
        printf("%u-%u-%u %u:%u:%u\n", year, month, day, hour, minute, second);
        for (uint r = 0; r < 2; r++) {
            for (uint c = 0; c < 13; c++)
                printf(" %02x", rtc_data[r][c]);
            printf("\n");
        }
        return;
    }
// dprintf(DF_RTC, "%u-%u-%u %u:%u:%u", year, month, day, hour, minute, second);

    rtc_allow_writes(TRUE);
    rtc_set_date(year, month, day, dow);
    rtc_set_time(hour, minute, second, am_pm, hour_24);
    rtc_allow_writes(FALSE);

    RTC_BKPXR(1) = rtc_data[1][2] |
                   (rtc_data[1][3] << 4) |
                   (rtc_data[1][4] << 8) |
                   (rtc_data[1][5] << 12) |
                   (rtc_data[1][6] << 16) |
                   (rtc_data[1][7] << 20) |
                   (rtc_data[1][8] << 24);
}

static void
amigartc_copy_time_stm32_to_rp5c01(void)
{
    uint tr       = RTC_TR;
    uint dr       = RTC_DR;
    uint alarm    = RTC_BKPXR(1);
    uint second   = rtc_bcd_to_binary(tr & 0x7f);
    uint minute   = rtc_bcd_to_binary((tr >> 8) & 0x7f);
    uint hour     = rtc_bcd_to_binary((tr >> 16) & 0x3f);
    uint day      = rtc_bcd_to_binary(dr & 0x3f);
    uint month    = rtc_bcd_to_binary((dr >> 8) & 0x1f);
    uint year     = rtc_bcd_to_binary(dr >> 16);
    uint dow      = (dr >> 13) & 7;
    uint hour_24  = (RTC_CR & RTC_CR_FMT) ? 1 : 0;
    uint am_pm    = ((dr >> 22) & 1) << 1;  // Shift left for BIT(1) position
    uint leap_c;

    if (year >= 70)
        leap_c = (year - 72) % 4;
    else
        leap_c = year % 4;  // XXX: How to represent 2000 as not a leap year?

#if 0
printf(" %02u:%02u:%02u", hour, minute, second);
printf(" %02u-%02u-%02u", year, month, day);
#endif
    rtc_data[0][0]  = second % 10;
    rtc_data[0][1]  = second / 10;
    rtc_data[0][2]  = minute % 10;
    rtc_data[0][3]  = minute / 10;
    rtc_data[0][4]  = hour % 10;
    rtc_data[0][5]  = hour / 10;
    rtc_data[0][6]  = dow;
    rtc_data[0][7]  = day % 10;
    rtc_data[0][8]  = day / 10;
    rtc_data[0][9]  = month % 10;
    rtc_data[0][10] = month / 10;
    rtc_data[0][11] = year % 10;
    rtc_data[0][12] = year / 10;
    rtc_data[1][10] = hour_24 | am_pm;
    rtc_data[1][11] = leap_c;

    if (RTC_BKPXR(0) != RTC_RAM_MAGIC)  // Invalid magic
        return;

    rtc_data[1][2] = alarm & 0xf;           // 1-minute alarm register
    rtc_data[1][3] = (alarm >> 4) & 0xf;    // 10-minute alarm register
    rtc_data[1][4] = (alarm >> 8) & 0xf;    // 1-hour alarm register
    rtc_data[1][5] = (alarm >> 12) & 0xf;   // 10-hour alarm register
    rtc_data[1][6] = (alarm >> 16) & 0xf;   // Day-of-the-week register
    rtc_data[1][7] = (alarm >> 20) & 0xf;   // 1-day alarm register
    rtc_data[1][8] = (alarm >> 24) & 0xf;   // 10-day alarm register
}

static void
amigartc_copy_ram_stm32_to_rp5c01(void)
{
    uint32_t temp;
    if (RTC_BKPXR(0) != RTC_RAM_MAGIC)  // Invalid magic
        return;

    temp = RTC_BKPXR(2);
    rtc_data[2][0] = temp & 0xf;
    rtc_data[2][1] = (temp >> 4) & 0xf;
    rtc_data[2][2] = (temp >> 8) & 0xf;
    rtc_data[2][3] = (temp >> 12) & 0xf;
    rtc_data[2][4] = (temp >> 16) & 0xf;
    rtc_data[2][5] = (temp >> 20) & 0xf;
    rtc_data[2][6] = (temp >> 24) & 0xf;
    rtc_data[2][7] = (temp >> 28) & 0xf;
    temp = RTC_BKPXR(3);
    rtc_data[2][8] = temp & 0xf;
    rtc_data[2][9] = (temp >> 4) & 0xf;
    rtc_data[2][10] = (temp >> 8) & 0xf;
    rtc_data[2][11] = (temp >> 12) & 0xf;
    rtc_data[2][12] = (temp >> 16) & 0xf;
    temp = RTC_BKPXR(4);
    rtc_data[3][0] = temp & 0xf;
    rtc_data[3][1] = (temp >> 4) & 0xf;
    rtc_data[3][2] = (temp >> 8) & 0xf;
    rtc_data[3][3] = (temp >> 12) & 0xf;
    rtc_data[3][4] = (temp >> 16) & 0xf;
    rtc_data[3][5] = (temp >> 20) & 0xf;
    rtc_data[3][6] = (temp >> 24) & 0xf;
    rtc_data[3][7] = (temp >> 28) & 0xf;
    temp = RTC_BKPXR(5);
    rtc_data[3][8] = temp & 0xf;
    rtc_data[3][9] = (temp >> 4) & 0xf;
    rtc_data[3][10] = (temp >> 8) & 0xf;
    rtc_data[3][11] = (temp >> 12) & 0xf;
    rtc_data[3][12] = (temp >> 16) & 0xf;
}

// XXX: rtc_wkup_isr() is triggered by RTC_CR and RTC_WUTR
//      Maximum rate appears to be 0.5 Hz. Will probably need to change
//      to using some other timer so that the RP5C01 time is updated
//      every second.
void
rtc_wkup_isr(void)
{
//  printf(" WKUP");
    exti_reset_request(EXTI22);
    RTC_ISR &= ~RTC_ISR_WUTF;

    /* Sweep STM32 RTC to RP5C01 RAM */
    if (rtc_timer_en && !rtc_touched) {
        uint8_t second = RTC_TR & 0x7f;
        static uint8_t second_last;

        if (second_last != second) {
            second_last = second;
            amigartc_copy_time_stm32_to_rp5c01();
        }
    }
}

#if 0
void
rtc_alarm_isr(void)
{
    exti_reset_request(EXTI17);
    RTC_ISR &= ~(RTC_ISR_ALRAF | RTC_ISR_ALRBF);

    printf(" ALRM");
}
#endif

void
amigartc_print(void)
{
    uint hour_24  = rtc_data[1][0xa] & BIT(0);
    uint am_pm    = rtc_data[1][0xa] & BIT(1);
    uint year     = rtc_data[0][11] + rtc_data[0][12] * 10;

    printf("%02u%x%x-%x%x-%x%x %x%x:%x%x:%x%x dow=%x %u-hr%s  "
           "alarm=%x%x-%x%x %x%x:%x%x dow=%x EN=%u\n",
           (year >= 78) ? 19 : 20,
           rtc_data[0][12], rtc_data[0][11],  // year
           rtc_data[0][10], rtc_data[0][9],   // month
           rtc_data[0][8], rtc_data[0][7],    // day
           rtc_data[0][5], rtc_data[0][4],    // hour
           rtc_data[0][3], rtc_data[0][2],    // minute
           rtc_data[0][1], rtc_data[0][0],    // second
           rtc_data[0][6],                    // dow
           hour_24 ? 24 : 12, hour_24 ? "" : am_pm ? " p.m." : " a.m.",
           rtc_data[1][10], rtc_data[1][9],   // alarm month
           rtc_data[1][8], rtc_data[1][7],    // alarm day
           rtc_data[1][5], rtc_data[1][4],    // alarm hour
           rtc_data[1][3], rtc_data[1][2],    // alarm minute
           rtc_data[1][6],                    // alarm dow
           !!rtc_timer_en);
}

void
amigartc_poll(void)
{
    if (bec_errormsg_delayed[0] != '\0') {
        puts(bec_errormsg_delayed);
        bec_errormsg_delayed[0] = '\0';
    }

    if (rtc_touched && rtc_timer_en) {
        rtc_touched = 0;

        if (power_state == POWER_STATE_ON) {
            /* Sweep RP5C01 date/time to STM32 RTC */
            amigartc_copy_time_rp5c01_to_stm32();
            amigartc_print();
        } else {
            /* Discard RP5C01 time, which was possibly corrupted by power loss */
            rtc_touched = 0;
        }
    }
    if (rtc_ram_touched) {
        rtc_ram_touched = 0;

        if (power_state == POWER_STATE_ON) {
            /* Sweep RP5C01 RAM to STM32 RTC */
            amigartc_copy_ram_rp5c01_to_stm32();
            printf("RP->STM32 RAM\n");
        } else {
            /* Discard RP5C01 ram, which was possibly corrupted by power loss */
            rtc_ram_touched = 0;
        }
    }
    if (bec_msg_in >= (BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN) * 2) {
        uint expected = BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN +
                        ((bec_msg_inbuf[3] << 8) | bec_msg_inbuf[4]);
#if 0
{
// XXX: CDH debug
    static uint oexpected;
    static uint omsg_in;
    if (oexpected != expected) {
        oexpected = expected;
        printf("e %x\n", expected);
    }
    if (omsg_in != bec_msg_in) {
        omsg_in = bec_msg_in;
        printf("i %x\n", bec_msg_in);
    }
}
#endif
        if (bec_msg_in >= expected * 2) {
            msg_process_slow();
            bec_msg_in = 0;
            bec_msg_in_timeout = 0;
        } else if (bec_msg_in_timeout == 0) {
            bec_msg_in_timeout = timer_tick_plus_msec(1000);
        } else if (timer_tick_has_elapsed(bec_msg_in_timeout)) {
            uint pos;
            printf("Msg in timeout: got %u of %u\n  ",
                   bec_msg_in / 2, expected);
            for (pos = 0; pos < bec_msg_in / 2; pos++)
                printf(" %02x", bec_msg_inbuf[pos]);
            printf("\n");
            bec_msg_in_timeout = 0;
            bec_msg_in = 0;
        }
    }
    if ((bec_msg_out != 0) && (bec_msg_out_timeout != 0) &&
        timer_tick_has_elapsed(bec_msg_out_timeout)) {
        printf("Msg out timeout: sent %u of %u\n",
               (bec_msg_out - 1) / 2, bec_msg_out_max / 2);
        bec_msg_out = 0;
    }
}

#if 0
void
exti4_isr(void)
{
    uint16_t val;
    val = gpio_get(PotX_PORT, PotX_PIN | PotY_PIN);
    exti_reset_request(EXTI4);
//  val = gpio_get(PotX_PORT, PotX_PIN | PotY_PIN);
    if (val == 0) {
        /* Both driven, drive U D L R */
        gpio_setv(FORWARD_PORT, FORWARD_PIN | BACK_PIN | LEFT_PIN | RIGHT_PIN, 0);
        timer_delay_usec(1000);
printf("(4)");
        gpio_setv(FORWARD_PORT, FORWARD_PIN | BACK_PIN | LEFT_PIN | RIGHT_PIN, 1);
        printf("t");
    } else {
        printf(" %x", val);
    }
}

void
exti9_5_isr(void)
{
    uint16_t val;
    exti_reset_request(EXTI5);
    val = gpio_get(PotX_PORT, PotX_PIN | PotY_PIN);
    if (val == 0) {
        /* Both driven, drive U D L R */
        gpio_setv(FORWARD_PORT, FORWARD_PIN | BACK_PIN | LEFT_PIN | RIGHT_PIN, 0);
        timer_delay_usec(1000);
printf("(5)");
        gpio_setv(FORWARD_PORT, FORWARD_PIN | BACK_PIN | LEFT_PIN | RIGHT_PIN, 1);
        printf("t");
    } else {
        printf(" %x", val);
    }
}
#endif

/*
 * amigartc_reply_pending() will kick off a message reply by pre-loading
 *                          the first response byte.
 */
void
amigartc_reply_pending(void)
{
    rtc_data[1][RP_MAGIC_HI] = bec_msg_outbuf[0] >> 4;
    rtc_data[1][RP_MAGIC_LO] = bec_msg_outbuf[0] & 0xf;
    bec_msg_out = 1;
}

/*
 * amigartc_reset() resets runtime state of the Amiga RTC when the Amiga is
 *                  in reset. It does not clobber the current time.
 */
void
amigartc_reset(void)
{
    uint cur;
    bec_msg_out = 0;
    rtc_data[1][RP_MAGIC_HI] = 0;  // BEC message interface
    rtc_data[1][RP_MAGIC_LO] = 0;  // BEC message interface
    for (cur = 0; cur < 4; cur++) {
        rtc_data[cur][0xd] = 8;    // MODE register (clock running)
        rtc_data[cur][0xe] = 0;    // TEST register
        rtc_data[cur][0xf] = 0;    // RESET register
    }
}

void
amigartc_init(void)
{
    rtc_cur_bank    = 0;
    rtc_touched     = 0;
    rtc_ram_touched = 0;
    rtc_timer_en    = 1;

    amigartc_reset();
    amigartc_copy_time_stm32_to_rp5c01();
    amigartc_copy_ram_stm32_to_rp5c01();

    rcc_periph_clock_enable(RCC_SYSCFG);

    nvic_set_priority(NVIC_EXTI0_IRQ, 0x10);
#ifdef RESPOND_AS_RP5C01
    nvic_enable_irq(NVIC_EXTI0_IRQ);
#endif

    /* Map PB0 to EXTI0 */
    exti_select_source(EXTI0, GPIOB);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI0);
    exti_reset_request(EXTI0);

    /* Wakeup */
    exti_set_trigger(EXTI22, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI22);
    exti_reset_request(EXTI22);

    /*
     * EXTI line 0 is connected to GPIO PB0 (_RTCEN) by the above code
     * EXTI line 17 is connected to the RTC Alarm event
     * EXTI line 21 is connected to the RTC Tamper and TimeStamp events
     * EXTI line 22 is connected to the RTC Wakeup event
     *
     * The EXIT 0 (PB0) interrupt is used to handle incoming reads or writes
     * of the emulated RP5C01. This interrupt is handled by exti0_isr().
     *
     * The EXTI 22 (RTC wakeup event) is used to update the RP5C01 emulated
     * registers. This interrupt triggers interrupts to handler rtc_wkup_isr().
     */

    msg_init();
#ifdef INTERRUPT_CAPTURE_RP5C01
    memset(&amigartc_cap, 0xff, sizeof (amigartc_cap));
#endif
}
