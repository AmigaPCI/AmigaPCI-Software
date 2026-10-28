/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Clock functions.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/f2/rcc.h>
#include "board.h"
#include "clock.h"

#ifdef STM32F1
/*
 * STM32F107 Clock structure
 *
 *                                        /--PLLCLK
 *                                       /   72MHz
 *                                      /
 *    HSE---Prediv1_Mux--PREDIV1--PLLMul----PLL_VCO---USB Prescaler---USB
 *    8MHz               /1       x9   \   72MHz x2   /3              48MHz
 *                                      \
 *                                       \--PLLCLK
 *                                          72MHz
 *
 *
 *                                          /-----HCLK core 72MHz
 *                                         /--/1--Cortex timer 72MHz
 *                                        /-------FCLK 72MHz
 *                                       /
 *    PLLCLK--SYSCLK--AHB Prescaler--HCLK-----APB1 Pre------PCLK1 36 MHz
 *            72MHz   /1             72MHz\   /2      \--x2-APB1 Timer 72 MHz
 *                                         \
 *                                          \-APB2 Pre------PCLK2 72 MHz
 *                                            /1      \
 *                                                     \----APB2 Timer 72 MHz
 *                                                      \
 *                                                       \--ADC Pre---ADC 1,2
 *                                                          /6        12MHz
 *
 *    HSE---PREDIV2---VCOinput2------PLL2MUl----PLL2CLK (ignored)
 *    8MHz  /1         8MHz      \    x8         64
 *                                \
 *                                 \--PLL3Mul-Mul-VCO    (ignored)
 *                                    x8      x2  128MHz
 *
 * USB must always be 48 MHz
 *      USBPRE (USB Prescaler) may be either /1 or /1.5
 *
 *      48 * 2 / 2 = 48 MHz    Supported
 *      48 * 3 / 2 = 72 MHz    Supported
 *      48 * 4 / 2 = 96 MHz    Not possible
 *      48 * 5 / 2 = 129 MHz   Not possible
 *      48 * 6 / 2 = 144 MHz   Not possible
 *
 */
static const struct rcc_clock_scale rcc_clock_config = {
    /* HSE=8 PLL=72 USB=48 APB1=36 APB2=72 ADC=12 */
/*
 *  These don't need to be set because they are not driving a
 *  peripheral that this program requires.
 *
 *  .pll2_mul         = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8, // 8 * 8 = 64 MHz
 *  .pll3_mul         = RCC_CFGR2_PLL3MUL_PLL3_CLK_MUL8, // 8 * 8 * 2 = 128 MHz
 *  .prediv2          = RCC_CFGR2_PREDIV2_NODIV,         // 8 / 1 = 8 MHz
 */

    .prediv1_source   = RCC_CFGR2_PREDIV1SRC_HSE_CLK,  // 8 MHz
    .prediv1          = RCC_CFGR2_PREDIV_NODIV,        // 8 / 1 = 8 MHz
    .pll_source       = RCC_CFGR_PLLSRC_PREDIV1_CLK,   // 8 / 1 = 8 MHz
    .pll_mul          = RCC_CFGR_PLLMUL_PLL_CLK_MUL9,  // 9 * 8 = 72 MHz

    .hpre             = RCC_CFGR_HPRE_NODIV,           // 72 / 1 = 72 MHz Core
    .ppre1            = RCC_CFGR_PPRE_DIV2,            // 72 / 2 = 36 MHz APB1
    .ppre2            = RCC_CFGR_PPRE_NODIV,           // 72 / 1 = 72 MHz APB2
    .adcpre           = RCC_CFGR_ADCPRE_DIV6,          // 72 / 6 = 12 MHz ADC
    .usbpre           = RCC_CFGR_USBPRE_PLL_VCO_CLK_DIV3, // 72 * 2 / 3 = 48 MHz

    .flash_waitstates = 2,
    .ahb_frequency    = 72000000,
    .apb1_frequency   = 36000000,
    .apb2_frequency   = 72000000,
};
#elif defined(STM32F2)
/*
 * STM32F205 Clock structure
 *
 *             /---WatchdogEn---IWDGCLK
 *    LSI-----<
 *  32kHz      \
 *    LSE-----RTCSEL---RTCEN---RTCCLK
 * 32768Hz      |
 *              |HSE_RTC                   /1
 *       Div2to31                   /-----HPRE-----HCLK
 *       /                         /------DIV/8----SysTimer
 *      /     HSI                 /
 *      |      |  SW             /            /---------------APB1 Periphs
 *    HSE---SysclkMUX---AHB PRESC---APB1 PRESC---APB1 Presc---APB1 Timers
 *   8MHz      |        /1..512  \
 *      \      /                  \-APB2 PRESC---APB2 Presc---APB2 Timers
 *       \    /                               \---------------APB2 Periphs
 *       VCOPLL-------USB
 *       /            48MHz
 *    HSI
 *  16Mhz
 *
 *                  *** VCOPLL Breakout ***         ---PLLQ---USB
 *    HSE----\                                     /  /1..15  48MHz
 *    8MHz    \              1MHz      192MHz     /
 *            PLLSRC----PLLM------PLLN--------PLLP------------SysclkMUX
 *            /  Mux   /2..63    *2..432    /2,4,6,8          96MHz
 *    HSI----/
 *    16MHz
 *
 * fHCLKInternal  AHB clock frequency: 96 MHz
 * fPCLK1Internal APB1 clock frequency: 24 MHz
 * fPCLK2Internal APB2 clock frequency: 48 MHz
 *
 * HCLK is 96 MHz rather than the rated maximum of 120 MHz: a low-speed
 * device behind a hub (MacAlly keyboard behind two hubs) frequently hangs
 * the OTG host core (erratum: host packet transmission may hang through a
 * hub to a low-speed device) at 120 MHz, also at 144 and 168 MHz, but
 * very rarely at 96 MHz with this PLL setup (VCO 192 MHz). The same 96 MHz
 * from VCO 384 MHz, PLLP /4 did not do as well.
 */
const struct rcc_clock_scale rcc_clock_config = {
    /* HSE=8 CPU_HCLK=96 APB1=24 APB2=48 */
    .pllm  = 8,    // VCO input frequency = 1 MHz
    .plln  = 192,  // 64 MHz < VCO output (192 MHz) < 432 MHz
    .pllp  = 2,    // 192 / 2 = 96 MHz to SysclkMUX
    .pllq  = 4,    // 192 / 4 = 48 MHz to USB
    .hpre  = RCC_CFGR_HPRE_NODIV,
    .ppre1 = RCC_CFGR_PPRE_DIV4,  // APB1 = 24 MHz
    .ppre2 = RCC_CFGR_PPRE_DIV2,  // APB2 = 48 MHz
    .flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
    .apb1_frequency = 24000000,  // SysclkMUX / 4
    .apb2_frequency = 48000000,  // SysclkMUX / 2
};
#else
#define rcc_clock_config rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]
#endif

void
clock_init(void)
{
    rcc_clock_setup_hse_3v3(&rcc_clock_config);
}

uint32_t
clock_get_hclk(void)
{
    return (8000000 / rcc_clock_config.pllm *
            rcc_clock_config.plln / rcc_clock_config.pllp);
}

uint32_t
clock_get_apb1(void)
{
    return (rcc_clock_config.apb1_frequency);
}

uint32_t
clock_get_apb2(void)
{
    return (rcc_clock_config.apb2_frequency);
}
