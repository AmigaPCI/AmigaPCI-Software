/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Analog to digital conversion for sensors.
 */

#ifndef _ADC_H
#define _ADC_H

#if defined(STM32F407xx)
#define ADC_CHANNEL_TEMP ADC_CHANNEL_TEMP_F40
#endif

/* These limits are from the ST-Micro datasheets for the various parts */
#if defined(STM32F407xx) || defined(STM32F2)
#define TEMP_V25           76000     // 0.76V
#define TEMP_AVGSLOPE      25        // 2.5 mV/C
#define SCALE_VREF         12100     // 1.21V

#elif defined(STM32F1)
/* Verified STM32F103xE and STM32F107xC are identical */
#define TEMP_V25           141000    // 1.34V-1.52V; 1.41V seems more accurate
#define TEMP_AVGSLOPE      43        // 4.3 mV/C
#define SCALE_VREF         12000     // 1.20V

#else
#error STM32 architecture temp sensor slopes must be known
#endif

#include <libopencm3/stm32/adc.h>

#if defined(STM32F407xx)
#define ADC_CHANNEL_TEMP ADC_CHANNEL_TEMP_F40
#endif

#define ADC_CHANNEL_VREF_IN 20  // Computed VCC Vref input based on Vrefint

void adc_init(void);
void adc_shutdown(void);
int  adc_get_reading(uint which);
void adc_setup_sensor(uint which, uint gpio_pack, uint adc_channel);

#endif /* _ADC_H */
