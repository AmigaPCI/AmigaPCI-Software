/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Functions to monitor sensors and report readings.
 */

#include <stdlib.h>
#include <string.h>
#include "cmdline.h"
#include "printf.h"
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "power.h"
#include "sensor.h"
#include "fan.h"
#include "timer.h"
#include "config.h"

#define CHANNEL_FAN_TACH 0x20
#define CHANNEL_FAN_PWM  0x21

#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define GPP(g, p) ((g << 4) | (p))

typedef struct {
    const char *s_name;
    uint8_t     s_adc_channel;  // ADC channel (0 - 18)
    uint8_t     s_gpio_packed;  // GPIO pin (0 = virtual)
    uint8_t     s_power_domain; // Power domain (0 = all, 1=PS, 2=Regulators)
    int16_t     s_mul;          // Scale reading multiplier (< 6500)
    int16_t     s_div;          // Scale reading divisor    (> 0)
    int         s_add;          // Add constant to reading after multiply
    int         s_limit_min;    // Minimum in milli-units
    int         s_limit_max;    // Maximum in milli-units
} sensor_t;

static const sensor_t apci_sensors[] = {
//  { "VrefInt", ADC_CHANNEL_VREF, 0x00, 0, 1, 1, 0, 1200, 1220 },
    { "VrefInt", ADC_CHANNEL_VREF, 0x00, 0, 1, 1, 0, 1180, 1240 },
    { "TEMP",    ADC_CHANNEL_TEMP, 0x00, 0, 10000 / TEMP_AVGSLOPE, 1,
            -1 * TEMP_V25 * 10000 / TEMP_AVGSLOPE + 25 * 100000, 0, 60000 },
    { "VBAT",    ADC_CHANNEL_VBAT, 0x00, 0, 2, 1, 0, 2000, 5100 },
    { "3V3SB",   ADC_CHANNEL_VREF_IN, 0x00, 0, 1, 1, 0, 3000, 3400 },
    { "V5SB",    1,  GPP(PA, 1), 0, 2, 1, 0,   4500,   5500 },
    { "V5",      0,  GPP(PA, 0), 1, 2, 1, 0,   4500,   5500 },
    { "V3P3",    2,  GPP(PA, 2), 1, 2, 1, 0,   3200,   3600 },
    { "V1P2",    3,  GPP(PA, 3), 2, 1, 1, 0,   1100,   1300 },
//  { "MONx",    6,  GPP(PA, 6), 0, 2, 1, 0,      0,   5000 },
//  { "MONy",    7,  GPP(PA, 7), 0, 1, 1, 0,      0,   5000 },
    { "V12",     14, GPP(PC, 4), 1, 61, 10, 0,  11400,  13000 },
    { "V-12",    15, GPP(PC, 5), 1, 61, 10, -1670000, -13000, -10000 },
    { "Fan",     CHANNEL_FAN_TACH, 0x00, 1, 1, 1, 0,   100000, 4000000 },
    { "Fan %",   CHANNEL_FAN_PWM, 0x00, 1, 1, 1, 0,    0, 100000 },
};

static const sensor_t keyjam_sensors[] = {
//  { "VrefInt", ADC_CHANNEL_VREF, 0x00, 0, 1, 1, 0, 1200, 1220 },
    { "VrefInt", ADC_CHANNEL_VREF, 0x00, 0, 1, 1, 0, 1180, 1240 },
    { "TEMP",    ADC_CHANNEL_TEMP, 0x00, 0, 10000 / TEMP_AVGSLOPE, 1,
            -1 * TEMP_V25 * 10000 / TEMP_AVGSLOPE + 25 * 100000, 0, 60000 },
    { "VBAT",    ADC_CHANNEL_VBAT, 0x00, 0, 2, 1, 0, 2000, 5100 },
    { "V3P3",    ADC_CHANNEL_VREF_IN, 0x00, 0, 1, 1, 0, 3000, 3400 },
};

static const sensor_t *sensors = apci_sensors;
static uint sensor_count = ARRAY_SIZE(apci_sensors);

#define SENSOR_COUNT_MAX ARRAY_SIZE(apci_sensors)

/* Caution: These must be kept in sync with BS_STATE_* in bec_cmd.h */
#define SS_FLAG_UNDER_LIMIT        0x0001
#define SS_FLAG_WARNED_OVER_LIMIT  0x0002
#define SS_FLAG_OVER_LIMIT         0x0004
#define SS_FLAG_WARNED_UNDER_LIMIT 0x0008
#define SS_FLAG_IGNORED            0x0010

typedef struct {
    int  ss_reading;
    uint ss_flags;
} sensor_state_t;
sensor_state_t sensor_states[SENSOR_COUNT_MAX];

uint64_t adc_startup_time;

static int
average_cpu_temp(int reading)
{
#define CPU_TEMP_BUCKETS 64
    static int total;
    reading += (int) config.cpu_temp_bias * 100000;
    if (total == 0)
        total = reading / 8 * CPU_TEMP_BUCKETS;
    else
        total += reading / 8 - total / CPU_TEMP_BUCKETS;
    return (total / CPU_TEMP_BUCKETS * 8);
}

/*
 * print_limit
 * -----------
 * Display sensor limit
 */
static void
print_limit(int value, const char *suffix)
{
    int units;

    units = value / 1000;

    if (*suffix == 'C') {
        uint tenths = (abs(value) % 1000) / 100;
        printf("%3d.%u %-3s", units, tenths, suffix);
    } else if ((*suffix == 'R') || (*suffix == '%')) {
        units = value / 1000;
        printf("%5u %-3s", units, suffix);  // RPM or %
    } else {
        uint hundredths = (abs(value) % 1000) / 10;
        if ((units < -9) || (units > 9))
            printf("%3d.%u %-3s", units, hundredths / 10, suffix);
        else
            printf("%2d.%02u %-3s", units, hundredths, suffix);
    }
}

/*
 * print_reading
 * -------------
 * Display sensor reading, with appropriate rounding.
 */
static void
print_reading(int value, const char *suffix)
{
    uint units;
    char buf[16];

#undef DEBUG_READING_CONVERSION
#ifdef DEBUG_READING_CONVERSION
    printf("[%8d]", value);
#endif
    if (*suffix == 'C') {
        uint tenths;
        if (value >= 0)
            value += 5000;
        else
            value -= 5000;
        units = abs(value) / 100000;
        tenths = (abs(value) % 100000) / 10000;
        sprintf(buf, "%s%d.%u %-3s",
                (value < 0) ? "-" : "", units, tenths, suffix);
    } else if ((*suffix == 'R') || (*suffix == '%')) {
        value += 50000;
        units = value / 100000;
        sprintf(buf, "%u %-3s", units, suffix);  // RPM or %
    } else {
        uint hundredths;
        if (value >= 0)
            value += 500;
        else
            value -= 500;
        units = abs(value) / 100000;
        hundredths = (abs(value) % 100000) / 1000;
        sprintf(buf, "%s%d.%02u %-3s",
                (value < 0) ? "-" : "", units, hundredths, suffix);
    }
    printf("%10s", buf);
}

void
sensor_check_readings(void)
{
    uint cur;
    uint adc_which = 0;

    for (cur = 0; cur < sensor_count; cur++) {
        int reading;
        int limit_min = sensors[cur].s_limit_min;
        int limit_max = sensors[cur].s_limit_max;
        switch (sensors[cur].s_adc_channel) {
            case CHANNEL_FAN_TACH:
                reading = fan_get_rpm() * 100000;
                fan_get_limits(&limit_min, &limit_max);
                break;
            case CHANNEL_FAN_PWM:
                reading = fan_get_percent() * 100000;
                break;
            default:
                reading = adc_get_reading(adc_which++) * sensors[cur].s_mul /
                          sensors[cur].s_div + sensors[cur].s_add;
                break;
        }
        if (sensors[cur].s_adc_channel == ADC_CHANNEL_TEMP)
            reading = average_cpu_temp(reading);

        sensor_states[cur].ss_reading = reading;

        /* Check limits */
        if (reading / 100 < limit_min) {
            sensor_states[cur].ss_flags |= SS_FLAG_UNDER_LIMIT;
        } else if (reading / 100 > limit_max) {
            sensor_states[cur].ss_flags |= SS_FLAG_OVER_LIMIT;
        } else {
            /* Sensor is in normal state */
            sensor_states[cur].ss_flags &= ~(SS_FLAG_OVER_LIMIT |
                                             SS_FLAG_UNDER_LIMIT |
                                             SS_FLAG_IGNORED);
        }

        /* Check if fault can be ignored */
        if ((sensors[cur].s_power_domain == 0) ||
            (power_state == POWER_STATE_ON) ||
            (power_state == POWER_STATE_FAULT) ||
            (power_state == POWER_STATE_FAULT_ON)) {
            /*
             * This sensor should not be ignored either because it's in
             * the always-on power domain (0), or the power supply is in
             * an "on" state.
             */
            sensor_states[cur].ss_flags &= ~SS_FLAG_IGNORED;
        } else if (sensor_states[cur].ss_flags & (SS_FLAG_UNDER_LIMIT |
                                                  SS_FLAG_OVER_LIMIT)) {
            /*
             * Sensor has a tripped limit, but we are ignoring it because
             * the power domain is not up.
             */
            sensor_states[cur].ss_flags |= SS_FLAG_IGNORED;
        }
    }
}

uint
sensor_get_power_state(void)
{
    uint cur;
    uint count_bad = 0;
    uint count_good = 0;

    if (adc_startup_time != 0)
        return (POWER_STATE_INITIAL);  // Waiting for ADCs to start up

    /*
     * Check all secondary rails.
     * If any are down, then the power supply is in a fault state
     * or is off.
     */
    for (cur = 0; cur < sensor_count; cur++) {
        if (sensors[cur].s_power_domain == 1) {
            /* This is the power supply-is-on domain */
            if (sensor_states[cur].ss_flags &
                (SS_FLAG_UNDER_LIMIT | SS_FLAG_OVER_LIMIT)) {
                count_bad++;
            } else {
                count_good++;
            }
        }
    }

    if ((count_bad == 0) && (count_good > 2))
        return (POWER_STATE_ON);   // All on

    if (count_bad > 3)
        return (POWER_STATE_OFF);  // Probably off

    return (POWER_STATE_FAULT);
}

static const char *
sensor_suffix(uint pos)
{
    switch (sensors[pos].s_adc_channel) {
        default:
            return ("V");
        case ADC_CHANNEL_TEMP:
            return ("C");
        case CHANNEL_FAN_TACH:
            return ("RPM");
        case CHANNEL_FAN_PWM:
            return ("%");
    }
}

uint
sensor_get(const char *name, uint *value, const char **type)
{
    uint pos;
    for (pos = 0; pos < sensor_count; pos++) {
        if (strcmp(name, sensors[pos].s_name) == 0) {
            *type = sensor_suffix(pos);
            *value = sensor_states[pos].ss_reading;
            return (0);
        }
    }
    return (1);
}

/*
 * sensor_get_info
 * ---------------
 * Retrieves the name, current reading, limits, and state of the sensor at
 * the specified position. Returns 1 if there is no sensor at that position.
 */
uint
sensor_get_info(uint pos, sensor_info_t *info)
{
    int limit_min;
    int limit_max;

    if (pos >= sensor_count)
        return (1);

    limit_min = sensors[pos].s_limit_min;
    limit_max = sensors[pos].s_limit_max;
    if (sensors[pos].s_adc_channel == CHANNEL_FAN_TACH)
        fan_get_limits(&limit_min, &limit_max);

    info->si_name      = sensors[pos].s_name;
    info->si_reading   = sensor_states[pos].ss_reading / 100;
    info->si_limit_min = limit_min;
    info->si_limit_max = limit_max;
    info->si_type      = *sensor_suffix(pos);
    info->si_domain    = sensors[pos].s_power_domain;
    info->si_flags     = sensor_states[pos].ss_flags;
    return (0);
}

static void
sensor_show_state(uint pos, uint with_limits)
{
    int reading = sensor_states[pos].ss_reading;
    const char *status;
    const char *suffix = sensor_suffix(pos);

    if (sensor_states[pos].ss_flags & SS_FLAG_IGNORED)
        status = "Ignored";
    else if (sensor_states[pos].ss_flags & SS_FLAG_UNDER_LIMIT)
        status = "Under limit";
    else if (sensor_states[pos].ss_flags & SS_FLAG_OVER_LIMIT)
        status = "Over limit";
    else
        status = "Normal";

    printf("%-15s", sensors[pos].s_name);
    print_reading(reading, suffix);
    printf(" %-12s", status);
    if (with_limits) {
        int limit_min = sensors[pos].s_limit_min;
        int limit_max = sensors[pos].s_limit_max;
        switch (sensors[pos].s_adc_channel) {
            case CHANNEL_FAN_TACH:
                fan_get_limits(&limit_min, &limit_max);
                break;
        }
        printf(" [ ");
        print_limit(limit_min, suffix);
        printf(" - ");
        print_limit(limit_max, suffix);
        printf(" ]");
    }
    printf("\n");
}

void
sensor_show(void)
{
    uint     cur;
    sensor_check_readings();
    printf("   Sensor       Reading      Status             Limits\n"
           "-------------- ---------- ------------ "
           "-------------------------\n");
    for (cur = 0; cur < sensor_count; cur++) {
        sensor_show_state(cur, 1);
    }
}

void
sensor_poll(void)
{
    uint cur;

    /* Wait for ADCs to stabilize */
    if (adc_startup_time != 0) {
        if (!timer_tick_has_elapsed(adc_startup_time))
            return;
        adc_startup_time = 0;
        power_init();
    }

    sensor_check_readings();

    for (cur = 0; cur < sensor_count; cur++) {
        uint report_state = 0;
        if (sensor_states[cur].ss_flags & SS_FLAG_IGNORED)
            continue;

        /* Check for over limit */
        if (sensor_states[cur].ss_flags & SS_FLAG_WARNED_UNDER_LIMIT) {
            if ((sensor_states[cur].ss_flags & SS_FLAG_UNDER_LIMIT) == 0) {
                if ((sensor_states[cur].ss_flags & SS_FLAG_OVER_LIMIT) == 0) {
                    report_state = 1;  // normal
                }
                sensor_states[cur].ss_flags &= ~SS_FLAG_WARNED_UNDER_LIMIT;
            }
        } else if (sensor_states[cur].ss_flags & SS_FLAG_UNDER_LIMIT) {
            sensor_states[cur].ss_flags |= SS_FLAG_WARNED_UNDER_LIMIT;
            report_state = 1;
        }

        /* Check for under limit */
        if (sensor_states[cur].ss_flags & SS_FLAG_WARNED_OVER_LIMIT) {
            if ((sensor_states[cur].ss_flags & SS_FLAG_OVER_LIMIT) == 0) {
                if ((sensor_states[cur].ss_flags & SS_FLAG_UNDER_LIMIT) == 0) {
                    report_state = 1;  // normal
                }
                sensor_states[cur].ss_flags &= ~SS_FLAG_WARNED_OVER_LIMIT;
            }
        } else if (sensor_states[cur].ss_flags & SS_FLAG_OVER_LIMIT) {
            sensor_states[cur].ss_flags |= SS_FLAG_WARNED_OVER_LIMIT;
            report_state = 1;
        }

        if (report_state) {
            sensor_show_state(cur, 0);
        }
    }
}

void
sensor_init(void)
{
    uint cur;
    uint adc_which = 0;

    if (config.board_type == BOARD_TYPE_KEYJAM) {
        sensors = keyjam_sensors;
        sensor_count = ARRAY_SIZE(keyjam_sensors);
    } else {
        sensors = apci_sensors;
        sensor_count = ARRAY_SIZE(apci_sensors);
    }

    /* Initialize all sensors */
    for (cur = 0; cur < sensor_count; cur++) {
        // Eventually handle other sensors, such as Fan RPM & PWM
        switch (sensors[cur].s_adc_channel) {
            case CHANNEL_FAN_TACH:
            case CHANNEL_FAN_PWM:
                /* Non-ADC sensor */
                break;
            default:
                adc_setup_sensor(adc_which++, sensors[cur].s_gpio_packed,
                                 sensors[cur].s_adc_channel);
                break;
        }
    }
    adc_init();
    adc_startup_time = timer_tick_plus_msec(1);
}
