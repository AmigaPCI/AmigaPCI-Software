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

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "board.h"
#include "cmdline.h"
#include "printf.h"
#include "main.h"
#include "adc.h"
#include "timer.h"
#include "gpio.h"
#include "power.h"
#include "utils.h"
#include <libopencm3/stm32/f2/dma.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// #define VBAT_READ_INTERVAL 10000 // msec
#define VBAT_READ_INTERVAL 1000 // msec

/* These limits from the ST-Micro datasheets for the various parts */
#define TEMP_BASE          25000     // Base temperature is 25C

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

#define V5_DIVIDER_SCALE   2 / 10000 // (1k / 1k)

#define PA 0
#define PB 1
#define PC 2
#define PD 3
#define PE 4
#define PF 5
#define GPP(g, p) ((g << 4) | (p))

/*
 * Temperature sensor, from the user manaul
 *      Temp(C) = ((Vsense - V25) / Avg_Slope) + 25
 * Temperature sensor, from the datasheet
 *      V25 = 0.760V. Avg_Slope = 2.5 mV
 * Example
 *      Vsense=957.  (957 - 760) / 25 + 25 = 32.88 C
 * Table above does multiply, divide, and add in that order.
 * The following should give the desired effect:
 *             Vsense * 1000 / 25 - 5400
 *
 * VMON12 divider is 5.1K / 1K, which yields 1.9672131148V with 12V input.
 *             The fraction 2459 / 1250 is 1.9672, which is close.
 *
 * VMON-12 divider is 1K / 5.1K, giving -1.9672131148 + 3.3V = 1.3327868852
 *        At -12V. The fraction -833 / 625 is -1.332, which is close
 */

/*
 * STM32F2xx ADC pins from STM32F205 datasheet
 *
 * ADC123 means all three ADCs (ADC1, ADC2, and ADC3) support this channel.
 * ADC12 means ADC1 and ADC2 support this channel and GPIO combination.
 * ADC3 means only ADC3 supports this channel and GPIO combination.
 * IN0 through IN15 specify the input channel of the ADC.
 *
 *    GPIO  ADCs  Chan   GPIO  ADCs  Chan    GPIO  ADCs Chan
 *    ---- ------ ----   ---- ------ ----    ----  ---- ----
 *    PA0  ADC123_IN0    PB0  ADC12_IN8      PF3   ADC3_IN9
 *    PA1  ADC123_IN1    PB1  ADC12_IN9      PF4   ADC3_IN14
 *    PA2  ADC123_IN2    PC0  ADC123_IN10    PF5   ADC3_IN15
 *    PA3  ADC123_IN3    PC1  ADC123_IN11    PF6   ADC3_IN4
 *    PA4  ADC12_IN4     PC2  ADC123_IN12    PF7   ADC3_IN5
 *    PA5  ADC12_IN5     PC3  ADC123_IN13    PF8   ADC3_IN6
 *    PA6  ADC12_IN6     PC4  ADC12_IN14     PF9   ADC3_IN7
 *    PA7  ADC12_IN7     PC5  ADC12_IN15     PF10  ADC3_IN8
 *
 * Special inputs to ADC1:
 *      16=Temperature sensor
 *      17=Vref
 *      18=Vbat (Vbat/2)
 *         NOTE: Do not leave Vbat enabled except when wanting to sample it.
 *               Otherwise it will continuously draw battery power.
 *
 * DMA is used to extract values from the ADCs as per the reference manual,
 * this is required if sampling more than a single channel per ADC.
 */

#define CHANNEL_MAX 12

/* Buffer for the channel definitions */
static uint8_t adc_channels[CHANNEL_MAX];

/* Buffer to store the results of the ADC conversion */
volatile uint16_t adc_buffer[CHANNEL_MAX];
uint16_t adc_snapshot[CHANNEL_MAX];
static uint8_t channel_count = 0;
static uint16_t vbat_cache;

void
adc_setup_sensor(uint which, uint gpio_pack, uint adc_channel)
{
    adc_channels[which] = adc_channel;
    if (gpio_pack != 0) {
        uint32_t gpio = gpio_num_to_gpio((gpio_pack >> 4) - 1);
        uint32_t pin = gpio_pack & 0xf;
        gpio_setmode(gpio, BIT(pin), GPIO_SETMODE_ANALOG);
    }
    if (channel_count <= which) {
        channel_count = which + 1;
        if (channel_count > ARRAY_SIZE(adc_channels)) {
            printf("Channel count %u exceeds ADC channel buffers\n", which + 1);
            channel_count = ARRAY_SIZE(adc_channels);
        }
    }
}

/*
 *      STM32F207 DMA channels -- Table 22. DMA1 request mapping
 *
 * Chan Stream0  Stream1  Stream2  Stream3  Stream4  Stream5  Stream6  Stream7
 * 0    SPI3_RX           SPI3_RX  SPI2_RX  SPI2_TX  SPI3_TX           SPI3_TX
 * 1    I2C1_RX           TIM7_UP           TIM7_UP  I2C1_RX  I2C1_TX  I2C1_TX
 * 2    TIM4_CH1          I2S3ExRX TIM4_CH2 I2S2ExTX I2C3ExTX TIM4_UP  TIM4_CH3
 * 3    I2S3ExRX TIM2_UP  I2C3_RX  I2S2ExRX I2C3_TX  TIM2_CH1 TIM2_CH2 TIM2_UP
 * 3             TIM2_CH3                                     TIM2_CH4 TIM2_CH4
 * 4    UART5_RX USART3TX UART4_RX USART3TX UART4_TX USART2RX USART2TX UART5_TX
 * 5                      TIM3_CH4          TIM3_CH1 TIM3_CH2          TIM3_CH3
 * 5                      TIM3_UP           TIM3TRIG
 * 6    TIM5_CH3 TIM5_CH4 TIM5_CH1 TIM5_CH4 TIM5_CH2          TIM5_UP
 * 6    TIM5_UP  TIM5TRIG          TIM5TRIG
 * 7             TIM8_UP  I2C2_TX  I2C2_RX  USART3TX DAC1     DAC2     I2C2_TX
 *
 *
 *      STM32F207 DMA channels -- Table 23. DMA2 request mapping
 *
 * Chan Stream0  Stream1  Stream2  Stream3  Stream4  Stream5  Stream6  Stream7
 * 0    ADC1              TIM8_CH1-3        ADC1              TIM1_CH1-3
 * 1             DCMI     ADC2     ADC2                                DCMI
 * 2    ADC3     ADC3                                CRYP_OUT CRYP_IN  HASH_IN
 * 3    SPI1_RX           SPI1_RX  SPI1_TX           SPI1_TX
 * 4                      USART1RX SDIO              USART1RX SDIO     USART1TX
 * 5             USART6RX USART6RX                            USART6TX USART6TX
 * 6    TIM1TRIG TIM1_CH1 TIM1_CH2 TIM1_CH1 TIM1_CH4 TIM1_UP  TIM1_CH3
 * 7             TIM8_UP  TIM8_CH1 TIM8_CH2 TIM8_CH3                   TIM8_CH4
 * Chan 6 Stream 4 also has TIM1_TRIG and TIM1_COM
 * Chan 7 Stream 7 also has TIM8_TRIG and TIM8_COM
 */
void
adc_init(void)
{
#if defined(STM32F2)
    uint32_t adcbase = ADC1;
    uint32_t dma     = DMA2;
    uint     stream  = 4;
    uint     channel = 0;

    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA2);

// ADC reset is 40023824 (RCC_APB2RSTR bit 8)
// ADC1 is 40023844 (RCC_APB2ENR bit 8)
    dma_disable_stream(dma, stream);

    adc_power_off(adcbase);  // Turn off ADC during configuration

    dma_set_peripheral_address(dma, stream, (uintptr_t) &ADC_DR(adcbase));
    dma_set_memory_address(dma, stream, (uintptr_t) &adc_buffer[0]);
    dma_set_transfer_mode(dma, stream, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_number_of_data(dma, stream, channel_count);
    dma_channel_select(dma, stream, channel);
    dma_disable_peripheral_increment_mode(dma, stream);
    dma_enable_memory_increment_mode(dma, stream);
    dma_set_peripheral_size(dma, stream, DMA_SxCR_PSIZE_16BIT);
    dma_set_memory_size(dma, stream, DMA_SxCR_PSIZE_16BIT);
    dma_enable_circular_mode(dma, stream);
    dma_set_priority(dma, stream, DMA_SxCR_PL_HIGH);
    dma_enable_direct_mode(dma, stream);
    dma_set_fifo_threshold(dma, stream, DMA_SxFCR_FTH_2_4_FULL);
    dma_set_memory_burst(dma, stream, DMA_SxCR_MBURST_SINGLE);
    dma_set_peripheral_burst(dma, stream, DMA_SxCR_PBURST_SINGLE);
    dma_enable_stream(dma, stream);

    adc_disable_dma(adcbase);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);
    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);

    adc_enable_scan_mode(adcbase);
    adc_set_continuous_conversion_mode(adcbase);
    adc_disable_external_trigger_regular(adcbase);
    adc_disable_external_trigger_injected(adcbase);
    adc_set_right_aligned(adcbase);

    adc_enable_temperature_sensor();
    adc_enable_vbat_sensor();

    /* Enable repeated DMA from ADC */
    adc_set_dma_continue(adcbase);
    adc_enable_dma(adcbase);
    adc_power_on(adcbase);  // Enable ADC

    /* Assign the channels to be monitored by this ADC */
    adc_set_regular_sequence(adcbase, channel_count, adc_channels);

    /* 56 cycles per conversion */
//  adc_set_sample_time_on_all_channels(adcbase, ADC_SMPR_SMP_56CYC);
    adc_set_sample_time_on_all_channels(adcbase, ADC_SMPR_SMP_480CYC);
    adc_set_resolution(adcbase, ADC_CR1_RES_12BIT);
    /* Start the ADC and triggered DMA */
    adc_start_conversion_regular(adcbase);

// adc_enable_vbat_sensor();
#elif defined(STM32F1)
    uint32_t adcbase = ADC1;

    /* STM32F1... */
    uint32_t dma = DMA1;  // STM32F1xx RM Table 78 Summary of DMA1 requests...
    uint32_t channel = DMA_CHANNEL1;

    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA1);
    adc_power_off(adcbase);  // Turn off ADC during configuration
    rcc_periph_reset_pulse(RST_ADC1);
    adc_disable_dma(adcbase);

    dma_disable_channel(dma, channel);
    dma_channel_reset(dma, channel);
    dma_set_peripheral_address(dma, channel, (uintptr_t)&ADC_DR(adcbase));
    dma_set_memory_address(dma, channel, (uintptr_t)adc_buffer);
    dma_set_read_from_peripheral(dma, channel);
    dma_set_number_of_data(dma, channel, channel_count);
    dma_disable_peripheral_increment_mode(dma, channel);
    dma_enable_memory_increment_mode(dma, channel);
    dma_set_peripheral_size(dma, channel, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(dma, channel, DMA_CCR_MSIZE_16BIT);
    dma_enable_circular_mode(dma, channel);
    dma_set_priority(dma, channel, DMA_CCR_PL_MEDIUM);
    dma_enable_channel(dma, channel);

    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);  // Independent ADCs

    adc_enable_scan_mode(adcbase);

    adc_set_continuous_conversion_mode(adcbase);
    adc_set_sample_time_on_all_channels(adcbase, ADC_SMPR_SMP_239DOT5CYC);
    adc_disable_external_trigger_regular(adcbase);
    adc_disable_external_trigger_injected(adcbase);
    adc_set_right_aligned(adcbase);
    adc_enable_external_trigger_regular(adcbase, ADC_CR2_EXTSEL_SWSTART);

    adc_set_regular_sequence(adcbase, channel_count, (uint8_t *)adc_channels);
    adc_enable_temperature_sensor();

    adc_enable_dma(adcbase);

    adc_power_on(adcbase);
    adc_reset_calibration(adcbase);
    adc_calibrate(adcbase);

    /* Start the ADC and triggered DMA */
    adc_start_conversion_regular(adcbase);
#endif  /* STM2F1 */
}

void
adc_shutdown(void)
{
    adc_power_off(ADC1);
    adc_disable_dma(ADC1);
    dma_disable_stream(DMA2, 4);
}

static int adc_scale;  // ADC scale value used to adjust sensor readings

/*
 * adc_get_scale
 * -------------
 * Updates the current scale value based on the ADC sensor reading of
 * the Vrefint ADC. This value is based on the internal reference voltage
 * and is then used to appropriately scale all other ADC readings.
 */
static void
adc_update_scale(uint16_t adc0_value)
{
    int tscale;

    if (adc0_value == 0)
        adc0_value = 1;

    tscale = SCALE_VREF * 4096 / 3.3 / adc0_value;

    if ((adc_scale < tscale * 7 / 8) || (adc_scale > tscale * 9 / 8))
        adc_scale = tscale;  // wildly different, just take new value
    else
        adc_scale += (tscale - adc_scale) / 16;
// printf("[%x,%x,%x]", adc0_value, tscale, adc_scale);
}

static uint16_t
adc_average_vref(uint16_t vref)
{
    static uint16_t adc_vref;
    if (adc_vref == 0)
        adc_vref = vref;
    else
        adc_vref += (vref - adc_vref) / 2;
    return (adc_vref);
}

int
adc_get_reading(uint cur)
{
    int calc;
    if (cur == 0) {
        /* Capture ADCs all at once */
        memcpy(adc_snapshot, (void *)adc_buffer, sizeof (adc_buffer));
        adc_update_scale(adc_snapshot[0]);
#ifdef DEBUG_ADC
        printf("ADC scale=%u\n", adc_scale);
#endif
    }
    switch (adc_channels[cur]) {
        case ADC_CHANNEL_VBAT: {
            static uint64_t vbat_refresh_timer;
            static uint8_t  vbat_mode;
            /*
             * Special handling for battery channel, as it should be read
             * infrequently to avoid draining the battery.
             *
             * When vbat_mode is 0
             *      If the timer has elapsed, update the cached timer value.
             *      and set vbat_mode to 0. Set a timer-until-next enable.
             * When vbat_mode is 1,
             *      The timer tells when it's time to read the sensor again.
             *      Once the timer has elapsed, then enable the VBAT sensor,
             *      set vbat_mode to 1, and start a timer-until-ready
             */
            if (vbat_mode == 0) {
                if (timer_tick_has_elapsed(vbat_refresh_timer)) {
                    vbat_cache = adc_snapshot[cur];
                    adc_disable_vbat_sensor();
                    vbat_mode = 1;
                    vbat_refresh_timer = timer_tick_plus_msec(VBAT_READ_INTERVAL);
                }
            } else if (timer_tick_has_elapsed(vbat_refresh_timer)) {
                adc_enable_vbat_sensor();
                vbat_mode = 0;
                vbat_refresh_timer = timer_tick_plus_msec(1);  // until good
            }
            adc_snapshot[cur] = vbat_cache;
            break;
        }
        case ADC_CHANNEL_VREF:
//          adc_snapshot[cur] = SCALE_VREF * 4096 / 3.3 / adc_scale;
            adc_snapshot[cur] = adc_average_vref(adc_snapshot[cur]);
            break;
        case ADC_CHANNEL_VREF_IN:
            /*
             * Calibrated VDDA from internal VREFINT (nominally 1.21 V).
             *   VDDA = VREFINT_NOM * 4096 / adc[0]
             * Using the running scale factor:
             *   VDDA_mV = adc_scale * 10 * 3.3
             */
            adc_snapshot[cur] = 4096;  // Full scale = Vref
            break;
    }

    calc = adc_snapshot[cur] * adc_scale * 33 / 4096;
#ifdef DEBUG_ADC
    printf("[%03x] %9d ", adc[pos], calc);
#endif
    return (calc);
}
