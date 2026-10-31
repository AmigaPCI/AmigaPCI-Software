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

#ifndef _SENSOR_H
#define _SENSOR_H

void sensor_init(void);
void sensor_poll(void);
void sensor_show(void);
uint sensor_get_power_state(void);
void sensor_check_readings(void);
uint sensor_get(const char *name, uint *value, const char **type);

typedef struct {
    const char *si_name;      // Sensor name
    int         si_reading;   // Reading in milli-units
    int         si_limit_min; // Minimum in milli-units
    int         si_limit_max; // Maximum in milli-units
    char        si_type;      // Sensor type (V C R or %)
    uint8_t     si_domain;    // Power domain (0=all, 1=PS, 2=regs)
    uint        si_flags;     // Sensor state (SS_FLAG_*, same as BS_STATE_*)
} sensor_info_t;

uint sensor_get_info(uint pos, sensor_info_t *info);

#endif /* _SENSOR_H */
