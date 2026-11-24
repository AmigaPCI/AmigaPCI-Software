/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2020.
 *
 * ---------------------------------------------------------------------
 *
 * STM32 Real-Time Clock handling.
 */

#ifndef _RTC_H
#define _RTC_H

void rtc_init(void);
char *time_str(uint32_t secs, uint32_t msec, char *buf, size_t buflen);
void rtc_allow_writes(int allow);
void rtc_set_date(uint year, uint mon, uint day, uint dow);
void rtc_set_time(uint hour, uint min, uint sec, uint is_24hour, uint ampm);
void rtc_set_datetime(uint year, uint mon, uint day,
                      uint hour, uint min, uint sec);
void rtc_get_time(uint *year, uint *mon, uint *day,
                  uint *hour, uint *min, uint *sec, uint *msec);
uint32_t rtc_to_utc(uint year, uint mon, uint day, uint hour, uint min,
                    uint sec);
void utc_to_rtc(uint32_t secs, uint *year, uint *mon, uint *day, uint *hour,
                uint *min, uint *sec);
void rtc_print(uint which, uint newline);
uint32_t rtc_read_nvram(uint reg);
void rtc_write_nvram(uint reg, uint32_t value);
void rtc_calibrate(void);
uint8_t rtc_binary_to_bcd(uint value);
uint8_t rtc_bcd_to_binary(uint8_t value);

#endif /* _RTC_H */
