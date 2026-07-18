/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Low level STM32 GPIO access.
 */

#include "printf.h"
#include "main.h"
#include "gpio.h"
#include "utils.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

/*
 * STM32F2 Alternate Functions
 *  AF0  System
 *  AF1  TIM1, TIM2
 *  AF2  TIM3, TIM4, TIM5
 *  AF3  TIM8, TIM9, TIM10, TIM11
 *  AF4  I2C1, I2C2, I2C3
 *  AF5  SPI1, SPI2
 *  AF6  SPI3
 *  AF7  USART1, USART2, USART3
 *  AF8  USART4, USART5, USART6
 *  AF9  CAN1, CAN1, TIM12, TIM13, TIM14
 *  AF10 OTG_FS, OTG_HS
 *  AF11 ETH
 *  AF12 FSMC, SDIO, OTH_HS
 *  AF13 DCMI
 *  AF14 -
 *  AF15 EVENTOUT
 */

/**
 * spread8to32() will spread an 8-bit value to odd bits of a 32-bit value
 *
 * This is useful for STM32 registers where the combination of four
 * sequential bits will represent settings for a single GPIO pin.
 *
 * Algorithm
 *     00000000000000000000000011111111  Initial data
 *     00000000000011110000000000001111  (0x000000f0 << 12) | 0x0000000f
 *     00000011000000110000001100000011  (0x000c000c << 6) | 0x00030003
 *     00010001000100010001000100010001  (0x02020202 << 3) | 0x02020202
 */
static uint32_t
spread8to32(uint32_t v)
{
    v = ((v & 0x000000f0) << 12) | (v & 0x0000000f);
    v = ((v & 0x000c000c) << 6) | (v & 0x00030003);
    v = ((v & 0x22222222) << 3) | (v & 0x11111111);
    return (v);
}

/**
 * spread16to32() will spread a 16-bit value to odd bits of a 32-bit value
 *
 * This is useful for STM32 registers where the combination of two
 * sequential bits will represent a mode for a single GPIO pin.
 *
 * Algorithm
 *     00000000000000001111111111111111  Initial data
 *     00000000111111110000000011111111  (0x0000ff00 << 8) | 0x000000ff
 *     00001111000011110000111100001111  (0x00f000f0 << 4) | 0x000f000f
 *     00110011001100110011001100110011  (0x0c0c0c0c << 2) | 0x03030303
 *     01010101010101010101010101010101  (0x22222222 << 1) | 0x11111111
 */
static uint32_t
spread16to32(uint32_t v)
{
    v = ((v & 0x0000ff00) << 8) | (v & 0x000000ff);
    v = ((v & 0x00f000f0) << 4) | (v & 0x000f000f);
    v = ((v & 0x0c0c0c0c) << 2) | (v & 0x03030303);
    v = ((v & 0x22222222) << 1) | (v & 0x11111111);
    return (v);
}

/*
 * gpio_set_1
 * ----------
 * Drives the specified GPIO bits to 1 values without affecting other bits.
 */
static void
gpio_set_1(uint32_t GPIOx, uint16_t GPIO_Pins)
{
    GPIO_BSRR(GPIOx) = GPIO_Pins;
}

/*
 * gpio_set_0
 * ----------
 * Drives the specified GPIO bits to 0 values without affecting other bits.
 */
static void
gpio_set_0(uint32_t GPIOx, uint16_t GPIO_Pins)
{
    GPIO_BSRR(GPIOx) = GPIO_Pins << 16;
}

/*
 * gpio_setv
 * ---------
 * Sets the specified GPIO bits to 0 or 1 values without affecting other bits.
 */
void
gpio_setv(uint32_t GPIOx, uint16_t GPIO_Pins, int value)
{
    if (value == 0)
        gpio_set_0(GPIOx, GPIO_Pins);
    else
        gpio_set_1(GPIOx, GPIO_Pins);
}

/*
 * gpio_getv
 * ---------
 * Gets the current output values (not input values) of the specified GPIO
 * port and pins.
 */
static uint
gpio_getv(uint32_t GPIOx, uint pin)
{
    return (GPIO_ODR(GPIOx) & BIT(pin));
}

/*
 * gpio_num_to_gpio
 * ----------------
 * Convert the specified GPIO number to its respective port address.
 */
uint32_t
gpio_num_to_gpio(uint num)
{
    static const uint32_t gpios[] = {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH
    };
    return (gpios[num]);
}

char *
gpio_to_str(uint32_t port, uint16_t pin)
{
    uint gpio;
    uint bit;
    static char name[8];
    static const uint32_t gpios[] = {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH
    };
    for (gpio = 0; gpio < ARRAY_SIZE(gpios); gpio++)
        if (gpios[gpio] == port)
            break;
    for (bit = 0; bit < 16; bit++)
        if (pin & BIT(bit))
            break;
    sprintf(name, "P%c%u", gpio + 'A', bit);
    return (name);
}

/* Values here are selected from MODER OTYPER PUPDR xx x xx */
static const char gpio_mode_short[][4] = {
    "I",   "PU",   "PD",   "Ix",   // 00 0 Input  Flt PU   PD   Rsvd
    "I",   "PU",   "PD",   "Ix",   // 00 1 Input  Flt PU   PD   Rsvd
    "O",   "OPU",  "OPD",  "Ox",   // 01 0 Output PP  PU   PD   Rsvd
    "D",   "DPU",  "DPD",  "Dx",   // 01 1 Output OD  PU   PD   Rsvd
    "AF",  "AU",   "AD",   "AFx",  // 10 0 AltFun PP  PU   PD   Rsvd
    "af",  "au",   "ad",   "afx",  // 10 1 AltFun OD  PU   PD   Rsvd
    "A",   "Ax",   "Ax",   "Ax",   // 11 0 Analog A   Rsvd Rsvd Rsvd
    "A",   "Ax",   "Ax",   "Ax",   // 11 1 Analog A   Rsvd Rsvd Rsvd
};

static const char * const gpio_mode_long[] = {
    "Input", "Input Pullup", "Input Pulldown", "Rsvd",
    "Input", "Input Pullup", "Input Pulldown", "Rsvd",
    "Output", "Output Pullup", "Output Pulldown", "Rsvd",
    "Open Drain", "Open Drain Pullup", "Open Drain Pulldown", "Rsvd",
    "AltFunc", "AltFunc Pullup", "AltFunc Pulldown", "Rsvd",
    "AltFunc Open Drain", "AltFunc Open Drain Pullup",
        "AltFunc Open Drain Pulldown", "Rsvd",
    "Analog", "Rsvd", "Rsvd", "Rsvd",
    "Analog", "Rsvd", "Rsvd", "Rsvd",
};

/*
 * The following table is derived from the STM32F205 reference manual table 14
 *
 * MODER     OTYPER  PUPDR     Configuration     Output
 * 00:INPUT  x       00:None   Input Floating    I
 * 00:INPUT  x       01:PU     Input PU          PU
 * 00:INPUT  x       10:PD     Input PD          PD
 * 00:INPUT  x       11:Rsvd   Reserved (Float)  -
 * 01:OUTPUT 0:PP    00:None   Output PP         O
 * 01:OUTPUT 0:PP    01:PU     Output PP+PU      OPU
 * 01:OUTPUT 0:PP    10:PD     Output PP+PD      OPD
 * 01:OUTPUT 0:PP    11:Rsvd   Reserved          -
 * 01:OUTPUT 1:OD    00:None   Output OD         D
 * 01:OUTPUT 1:OD    01:PU     Output OD+PU      DPU
 * 01:OUTPUT 1:OD    10:PD     Output OD+PD      DPD
 * 01:OUTPUT 1:OD    11:Rsvd   Reserved (OD)     -
 * 10:AF     0:PP    00:None   AF PP             AF
 * 10:AF     0:PP    01:PU     AF PP+PU          APU
 * 10:AF     0:PP    10:PD     AF PP+PD          APD
 * 10:AF     0:PP    11:Rsvd   Reserved          -
 * 10:AF     1:OD    00:None   AF OD             AD
 * 10:AF     1:OD    01:PU     AF OD+PU          ADU
 * 10:AF     1:OD    10:PD     AF OD+PD          ADD
 * 10:AF     1:OD    11:Rsvd   Reserved          -
 * 11:ANALOG x       00:None   Analog I/O        A
 * 11:ANALOG x       01:Rsvd   Reserved          -
 * 11:ANALOG x       10:Rsvd   Reserved          -
 * 11:ANALOG x       11:Rsvd   Reserved          -
 *
 * MODER       OTYPER   PUPDR     OSPEEDR            Subject to datasheet
 * 00 INPUT    0 PP     00 None   00 Low speed       (2 MHz)
 * 01 OUTPUT   1 OD     01 PU     01 Medium speed    (25 MHz)
 * 10 AF                10 PD     10 High speed      (50 MHz)
 * 11 ANALOG            11 Rsvd   11 Very high speed (100 MHz)
 */


typedef struct {
    char    name[10];
    uint8_t port;
    uint8_t pin;
} gpio_names_t;

#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_H 7
static const gpio_names_t gpio_names[] = {
    { "CONS_TX",    GPIO_C, 10 },
    { "CONS_RX",    GPIO_C, 11 },
    { "PWRSW",      GPIO_C, 12 },
    { "PS_ON",      GPIO_D, 2 },
    { "D16",        GPIO_B, 4 },
    { "D17",        GPIO_B, 5 },
    { "D18",        GPIO_B, 6 },
    { "D19",        GPIO_B, 7 },
    { "FANPWM",     GPIO_B, 8 },
    { "FANTACH",    GPIO_B, 9 },
    { "STMRSTA",    GPIO_C, 13 },
    { "LSCLKIN",    GPIO_C, 14 },
    { "LSCLKOUT",   GPIO_C, 15 },
    { "HSCLKIN",    GPIO_H, 0 },
    { "HSCLKOUT",   GPIO_H, 1 },
    { "Forward",    GPIO_C, 0 },
    { "Down",       GPIO_C, 0 },
    { "Back",       GPIO_C, 1 },
    { "Up",         GPIO_C, 1 },
    { "Left",       GPIO_C, 2 },
    { "Right",      GPIO_C, 3 },
    { "VMON5",      GPIO_A, 0 },
    { "VMON5SB",    GPIO_A, 1 },
    { "VMON3V3",    GPIO_A, 2 },
    { "VMON12",     GPIO_A, 3 },
    { "PotX",       GPIO_A, 4 },
    { "Button2",    GPIO_A, 4 },
    { "MMB",        GPIO_A, 4 },
    { "PotY",       GPIO_A, 5 },
    { "Button1",    GPIO_A, 5 },
    { "RMB",        GPIO_A, 5 },
    { "VMON12",     GPIO_C, 4 },
    { "VMON-12",    GPIO_C, 5 },
    { "RTCEN",      GPIO_B, 0 },
    { "R_WA",       GPIO_B, 1 },
    { "HIDEN",      GPIO_B, 2 },
    { "A2",         GPIO_B, 10 },
    { "A3",         GPIO_B, 11 },
    { "A4",         GPIO_B, 12 },
    { "A5",         GPIO_B, 13 },
    { "USB2_DM",    GPIO_B, 14 },
    { "USB2_DP",    GPIO_B, 15 },
    { "Fire",       GPIO_C, 6 },
    { "LMB",        GPIO_C, 6 },
    { "Button0",    GPIO_C, 6 },
    { "KBRST",      GPIO_C, 7 },
    { "KBData",     GPIO_C, 8 },
    { "KBCLK",      GPIO_C, 9 },
    { "POWER_LED",  GPIO_A, 8 },
    { "USBENABLE",  GPIO_A, 9 },
    { "USB1_DM",    GPIO_A, 11 },
    { "USB1_DP",    GPIO_A, 12 },
    { "STM_DIO",    GPIO_A, 13 },
    { "STM_SWCLK",  GPIO_A, 14 },
    { "SCL",        GPIO_A, 10 },
    { "SDA",        GPIO_A, 15 },
};

/*
 * gpio_name_match
 * ---------------
 * Convert a text name for a GPIO to the actual port and pin used.
 * This function returns 0 on match and non-zero on failure to match.
 */
uint
gpio_name_match(const char **namep, uint16_t pins[NUM_GPIO_BANKS])
{
    const char *name = *namep;
    const char *ptr;
    uint cur;
    uint len;
    uint wildcard = 0;
    uint matched = 0;
    uint lport = 0xff;
    uint lpin  = 0xff;
    for (ptr = name; *ptr != ' '; ptr++) {
        if (((*ptr < '0') || ((*ptr > '9') && (*ptr < 'A')) ||
             (*ptr > 'z') || ((*ptr > 'Z') && (*ptr < 'a'))) &&
            (*ptr != '_')) {
            break;  // Not alphanumeric
        }
    }
    len = ptr - name;
    if (strncmp(name, "?", len) == 0) {
        printf("GPIO names\n ");
        for (cur = 0; cur < ARRAY_SIZE(gpio_names); cur++) {
            if ((lport == gpio_names[cur].port) &&
                (lpin  == gpio_names[cur].pin))
                continue;
            printf(" %s", gpio_names[cur].name);
            lport = gpio_names[cur].port;
            lpin  = gpio_names[cur].pin;
        }
        printf("\n");
        return (1);
    }
    if (*ptr == '*') {
        ptr++;
        wildcard = 1;
    }

    for (cur = 0; cur < ARRAY_SIZE(gpio_names); cur++) {
        if ((strncasecmp(name, gpio_names[cur].name, len) == 0) &&
            (wildcard || (gpio_names[cur].name[len] == '\0'))) {
            uint port = gpio_names[cur].port;
            uint pin = gpio_names[cur].pin;
            if (port >= NUM_GPIO_BANKS)
                return (1);
            if ((lport == port) && (lpin == pin))
                continue;
            pins[port] |= BIT(gpio_names[cur].pin);
            lport = gpio_names[cur].port;
            lpin  = gpio_names[cur].pin;
            matched++;
        }
    }
    if (matched == 0)
        return (1);  // No match
    *namep = ptr;
    return (0);
}

const char *
gpio_to_name(int port, int pin)
{
    uint cur;
    for (cur = 0; cur < ARRAY_SIZE(gpio_names); cur++) {
        if ((port == gpio_names[cur].port) && (pin == gpio_names[cur].pin))
            return (gpio_names[cur].name);
    }
    return (NULL);
}

/*
 * gpio_setmode
 * ------------
 * Sets the complex input/output mode of one or more GPIOs.
 *
 * The GPIO_Pins parameter is a mask GPIOs (eg: GPIO8 | GPIO9)
 *
 * Values are based on MODER OTYPER  PUPDR OSPEEDR
 *                     xx    x       xx    xx
 *                     Upper-Nibble  Lower-Nibble
 (*
 * Use the GPIO_SETMODE_x macros in gpio.h
 */
void
gpio_setmode(uint32_t GPIOx, uint16_t GPIO_Pins, uint mode)
{
    uint32_t spread = spread16to32(GPIO_Pins);
    uint32_t mask   = spread * 0x3;

#undef GPIO_LL_DEBUG
#ifdef GPIO_LL_DEBUG
    printf("mode=%04x: MODER=%08lx OTYPER=%08lx PUPDR=%04lx OSPEEDR=%08lx\n",
           mode, GPIO_MODER(GPIOx), GPIO_OTYPER(GPIOx), GPIO_PUPDR(GPIOx),
           GPIO_OSPEEDR(GPIOx));
#endif
    GPIO_MODER(GPIOx) = (GPIO_MODER(GPIOx) & ~mask) |
                        (((mode >> 5) & 3) * spread);
    if (mode & BIT(4))  // Set OTYPER
        GPIO_OTYPER(GPIOx) |= GPIO_Pins;
    else
        GPIO_OTYPER(GPIOx) &= ~GPIO_Pins;
    GPIO_PUPDR(GPIOx) = (GPIO_PUPDR(GPIOx) & ~mask) |
                        (((mode >> 2) & 3) * spread);
    GPIO_OSPEEDR(GPIOx) = (GPIO_OSPEEDR(GPIOx) & ~mask) |
                          ((mode & 3) * spread);
#ifdef GPIO_LL_DEBUG
    printf("       --> MODER=%08lx OTYPER=%08lx PUPDR=%04lx OSPEEDR=%08lx\n",
           GPIO_MODER(GPIOx), GPIO_OTYPER(GPIOx), GPIO_PUPDR(GPIOx),
           GPIO_OSPEEDR(GPIOx));
#endif

    if ((mode & 0x60) == (GPIO_SETMODE_ALTFUNC_2 & 0x60)) {
        uint32_t afspread;
        uint32_t af = (mode >> 8) & 0xf;
        if ((GPIO_Pins & 0x00ff) != 0) {
            afspread = spread8to32(GPIO_Pins & 0x00ff);
            mask = afspread * 0xf;
            GPIO_AFRL(GPIOx) = (GPIO_AFRL(GPIOx) & ~mask) | (af * afspread);
        }
        if ((GPIO_Pins & 0xff00) != 0) {
            afspread = spread8to32(GPIO_Pins >> 8);
            mask = afspread * 0xf;
            GPIO_AFRH(GPIOx) = (GPIO_AFRH(GPIOx) & ~mask) | (af * afspread);
        }
    }
#undef GPIO_DEBUG
#ifdef GPIO_DEBUG
    uint pin;
    for (pin = 0; pin < 16; pin++) {
        uint x;
        if ((GPIO_Pins & BIT(pin)) == 0)
            continue;
        printf("GPIO %s ", gpio_to_str(GPIOx, BIT(pin)));
        x = (mode >> 5) & 0x3;
        printf("MODER=%s", (x == 0) ? "I" : (x == 1) ? "O" :
                            (x == 2) ? "AF" : "A");
        if ((x == 1) && (x == 2)) {  // Output or AF
            x = (mode >> 4) & 1;
            printf(" OTYPER=%s", (x == 0) ? "PP" : "OD");
        }

        x = (mode >> 2) & 3;
        if ((((mode >> 5) & 3) == 3) && (x != 0)) // Analog only allows X=0
            x = 3;
        if (x != 0)
            printf(" PUPDR=%s", (x == 1) ? "PU" : (x == 2) ? "PD" : "Rsvd");
        if (((mode >> 5) & 3) == 2) {
            printf(" AF%u", (mode >> 8) & 0xf);
        }
        printf("\n");
    }
#endif
}

/*
 * gpio_getmode
 * ------------
 * Get the input/output mode of the specified GPIO pins.
 *
 * Bit values returned are based on MODER OTYPER  PUPDR OSPEEDR
 *                                  xx    x       xx    xx
 *                                  Upper-Nibble  Lower-Nibble
 *
 * Note that the pin parameter is not a GPIO mask. It's a number 0-15.
 */
uint
gpio_getmode(uint32_t GPIOx, uint pin)
{
    uint mode = (((GPIO_MODER(GPIOx) >> (pin * 2)) & 3) << 5) |
                (((GPIO_OTYPER(GPIOx) >> pin) & 1) << 4) |
                (((GPIO_PUPDR(GPIOx) >> (pin * 2)) & 3) << 2) |
                ((GPIO_OSPEEDR(GPIOx) >> (pin * 2)) & 3);

    if ((mode & 0x60) == (GPIO_SETMODE_ALTFUNC_2 & 0x60)) {
        if (pin < 8)
            mode |= (((GPIO_AFRL(GPIOx) >> (pin * 4)) & 0xf) << 8);
        else
            mode |= (((GPIO_AFRH(GPIOx) >> ((pin - 8) * 4)) & 0xf) << 8);
    }
    return (mode);
}

/*
 * gpio_show
 * ---------
 * Display current values and input/output state of GPIOs.
 */
void
gpio_show(int whichport, int pins)
{
    int port;
    int pin;
    uint mode;
    uint array_mode;
    uint print_all = (whichport < 0) && (pins == 0xffff);

    if (print_all) {
#if 0
        printf("Socket OE=PA0 LED=PB8 KBRST=PB4\n"
               "Socket A0-A15=PC0-PC15 A13-A19=PA1-PA7 D31=PB12\n"
               "Flash  D0-D15=PD0-PD15 D16-D31=PE0-PE15\n"
               "Flash  A18=PB10 RP=PB1 RB=PB15\n"
               "Flash  A19=PB11 OE=PB13 WE=PB14 OEWE=PB9\n"
               "USB    V5=PA9 CC1=PA8 CC2=PA10 DM=PA11 DP=PA12\n");
#endif
        printf("\nMODE  ");
        for (pin = 15; pin >= 0; pin--)
            printf("%4d", pin);
        printf("\n");
    }

    for (port = 0; port < 5; port++) {
        uint32_t gpio = gpio_num_to_gpio(port);
        if ((whichport >= 0) && (port != whichport))
            continue;
        if (print_all)
            printf("GPIO%c ", 'A' + port);
        for (pin = 15; pin >= 0; pin--) {
            const char *mode_txt;
            if ((BIT(pin) & pins) == 0)
                continue;
            mode = gpio_getmode(gpio, pin);
            array_mode = (mode >> 2) & (ARRAY_SIZE(gpio_mode_short) - 1);
            if (print_all) {
                mode_txt = gpio_mode_short[array_mode];
            } else {
                mode_txt = gpio_mode_long[array_mode];
            }
            /* Pull-up or pull down depending on output register state */
            if (print_all) {
                if (((mode & 0x60) == (GPIO_SETMODE_OUTPUT_2 & 0x60)) &&
                    ((mode & 3) != 0) &&
                    (strlen(mode_txt) < 3)) {
                    /* Output or AltFunc and > 2 MHz -- Show speed */
                    printf("%3s%c", mode_txt,
                           ((mode & 3) == 1) ? '2' :
                           ((mode & 3) == 2) ? '5' : '1');  // 2 25 50 100
                } else if ((mode & 0x60) == (GPIO_SETMODE_ALTFUNC_2 & 0x60)) {
                    printf(" AF%x", (mode >> 8) & 0xf);
                } else {
                    printf("%4s", mode_txt);
                }
            } else {
                const char *name;
                char *mode_speed = "";
                char mode_altfunc[8];
                char mode_extra[8];
                uint pinstate = !!(gpio_get(gpio, BIT(pin)));
                if (((mode & 0x60) == (GPIO_SETMODE_OUTPUT_2 & 0x60)) ||
                    ((mode & 0x60) == (GPIO_SETMODE_ALTFUNC_2 & 0x60))) {
                    /* Output or AltFunc -- show speed */
                    switch (mode & 0x3) {
                        case GPIO_SETMODE_OUTPUT_2 & 3:
                            mode_speed = "2MHz ";
                            break;
                        case GPIO_SETMODE_OUTPUT_25 & 3:
                            mode_speed = "25MHz ";
                            break;
                        case GPIO_SETMODE_OUTPUT_50 & 3:
                            mode_speed = "50MHz ";
                            break;
                        case GPIO_SETMODE_OUTPUT_100 & 3:
                            mode_speed = "100MHz ";
                            break;
                    }
                }
                mode_altfunc[0] = '\0';
                if ((mode & 0x60) == (GPIO_SETMODE_ALTFUNC_2 & 0x60)) {
                    sprintf(mode_altfunc, " AF%u", (mode >> 8) & 0xf);
                }
                mode_extra[0] = '\0';
                if ((mode & 0x60) == (GPIO_SETMODE_OUTPUT_2 & 0x60)) {
                    /* Output -- check for inconsistency */
                    uint outval = !!gpio_getv(gpio, pin);
                    if (outval != pinstate)
                        sprintf(mode_extra, "=%u>", outval);
                }
                printf("P%c%d=%s%s %s(%s%d)", 'A' + port, pin, mode_txt,
                       mode_altfunc, mode_speed, mode_extra, pinstate);
                name = gpio_to_name(port, pin);
                if (name != NULL)
                    printf(" %s", name);
                printf("\n");
            }
        }
        if (print_all)
            printf("\n");
    }

    if (!print_all)
        return;

    printf("\nState ");
    for (pin = 15; pin >= 0; pin--)
        printf("%4d", pin);
    printf("\n");

    for (port = 0; port < 5; port++) {
        uint32_t gpio = gpio_num_to_gpio(port);
        printf("GPIO%c ", 'A' + port);
        for (pin = 15; pin >= 0; pin--) {
            uint pinstate = !!(gpio_get(gpio, BIT(pin)));
            mode = gpio_getmode(gpio, pin);
            if ((mode & 0x60) == (GPIO_SETMODE_OUTPUT_2 & 0x60)) {
                /* Output mode */
                uint outval = !!gpio_getv(gpio, pin);
                if (outval != pinstate) {
                    printf(" %u>%u", outval, pinstate);
                    continue;
                }
            }
            printf("%4d", pinstate);
        }
        printf("\n");
    }
}

/*
 * gpio_assign
 * -----------
 * Assign a GPIO input/output state or output value according to the
 * user-specified string.
 */
void
gpio_assign(int whichport, int pins, const char *assign)
{
    uint mode;
    uint gpio;
    uint pin;
    uint set_pins;
    if ((*assign == '?') || (*assign == '\0')) {
show_valid_modes:
        printf("Valid modes: "
               "0 1  I PU PD  O OPU OPD  D DPU DPD  AFx AUx ADx  A\n");
        return;
    }
    gpio = gpio_num_to_gpio(whichport);
    for (mode = 0; mode < ARRAY_SIZE(gpio_mode_short); mode++) {
        if (strcasecmp(gpio_mode_short[mode], assign) == 0) {
            gpio_setmode(gpio, pins, mode << 2);
            return;
        }
    }
    switch (*assign) {
        case 'a':
        case 'A':
            if (assign[1] == '\0') {
                gpio_setmode(gpio, pins, GPIO_SETMODE_ANALOG);
                return;
            } else if ((assign[1] == 'F') || (assign[1] == 'f') ||
                       (assign[1] == 'U') || (assign[1] == 'u')) {
                uint afunc = atoi(assign + 2);
                uint base = GPIO_SETMODE_ALTFUNC_2;

                if ((assign[1] == 'U') || (assign[1] == 'u'))
                    base = GPIO_SETMODE_ALTFUNC_ODRAIN_2;
                if (afunc > 15)
                    break;  // Invalid AF number
                if ((afunc == 0) && (assign[2] != '0') && (assign[2] != '\0'))
                    break;  // Bad digit following "AF"

                gpio_setmode(gpio, pins, base | (afunc << 8));
                return;
            }
            break;
        case 'i':
        case 'I':
            /* Input */
            if (assign[1] == '\0') {
                gpio_setmode(gpio, pins, GPIO_SETMODE_INPUT);
                return;
            }
            break;
        case 'o':
        case 'O':
            /* Output */
            switch (assign[1]) {
                case '\0':
                    gpio_setmode(gpio, pins, GPIO_SETMODE_OUTPUT_2);
                    return;
                case '2':
                    gpio_setmode(gpio, pins, GPIO_SETMODE_OUTPUT_25);
                    return;
                case '5':
                    gpio_setmode(gpio, pins, GPIO_SETMODE_OUTPUT_50);
                    return;
                case '1':
                    gpio_setmode(gpio, pins, GPIO_SETMODE_OUTPUT_100);
                    return;
                case 'D':
                case 'd':
                    /* Output open-drain */
                    switch (assign[2]) {
                        case '\0':
                            gpio_setmode(gpio, pins,
                                         GPIO_SETMODE_OUTPUT_ODRAIN_2);
                            return;
                        case '2':
                            gpio_setmode(gpio, pins,
                                         GPIO_SETMODE_OUTPUT_ODRAIN_25);
                            return;
                        case '5':
                            gpio_setmode(gpio, pins,
                                         GPIO_SETMODE_OUTPUT_ODRAIN_50);
                            return;
                        case '1':
                            gpio_setmode(gpio, pins,
                                         GPIO_SETMODE_OUTPUT_ODRAIN_100);
                            return;
                    }
                    break;
            }
            break;
        case '0':
            gpio_setv(gpio, pins, 0);
change_to_output:
            switch (assign[1]) {
                case '\0':  // 2 MHz
                    set_pins = 0;
                    for (pin = 0; pin < 16; pin++) {
                        if ((pins & BIT(pin)) == 0)
                            continue;
                        mode = gpio_getmode(gpio, pin);
                        if ((mode & 0x60) != (GPIO_SETMODE_OUTPUT_2 & 0x60))
                            set_pins |= BIT(pin);
                    }
                    if (set_pins != 0)
                        gpio_setmode(gpio, set_pins, GPIO_SETMODE_OUTPUT_2);
                    break;
                case '0':  // 2 MHz (force)
                    gpio_setmode(gpio, pins, GPIO_SETMODE_OUTPUT_2);
                    break;
                case '2':  // 25 MHz
                    gpio_setmode(gpio, pins, GPIO_SETMODE_OUTPUT_25);
                    break;
                case '5':  // 50 MHz
                    gpio_setmode(gpio, pins, GPIO_SETMODE_OUTPUT_50);
                    break;
                case '1':  // 100 MHz
                    gpio_setmode(gpio, pins, GPIO_SETMODE_OUTPUT_100);
                    break;
                default:
                    goto invalid;
            }
            return;
        case '1':
            gpio_setv(gpio, pins, 1);
            goto change_to_output;
        case 'p':
        case 'P':
            if (assign[2] == '\0') {
                switch (assign[1]) {
                    case 'u':
                    case 'U':
                        gpio_setmode(gpio, pins, GPIO_SETMODE_INPUT_PU);
                        return;
                    case 'd':
                    case 'D':
                        gpio_setmode(gpio, pins, GPIO_SETMODE_INPUT_PD);
                        return;
                    default:
                        break;
                }
            }
            break;
        default:
            break;
    }

invalid:
    printf("Invalid mode %s for GPIO\n", assign);
    goto show_valid_modes;
}

/*
 * gpio_init_early
 * ---------------
 * Set up required early GPIO states.
 */
void
gpio_init_early(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_GPIOH);
}

/*
 * gpio_init
 * ---------
 * Initialize most board GPIO states.
 */
void
gpio_init(void)
{
    if (config.board_type == BOARD_TYPE_KEYJAM) {
        /* Mouse/Joystick Port 0 */
        gpio_setv(GPIOC, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO6, 1);  // P0 DULRF
        gpio_setmode(GPIOC, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO6,
                     GPIO_SETMODE_OUTPUT_ODRAIN_25 | GPIO_SETMODE_PU);

        /* Joystick Port 1 */
        gpio_setv(GPIOB, GPIO0 | GPIO1 | GPIO2 | GPIO4 | GPIO5 |     // P1 FLRUD
                  GPIO6 | GPIO7 | GPIO8 | GPIO9, 1);        // P1 PxPy, P0 PxP7
        gpio_setmode(GPIOB, GPIO0 | GPIO1 | GPIO2 | GPIO4 | GPIO5 |
                     GPIO6 | GPIO7 | GPIO8 | GPIO9,
                     GPIO_SETMODE_OUTPUT_ODRAIN_25 | GPIO_SETMODE_PU);

        /* Keyboard port */
        gpio_setv(KBRST_PORT, KBRST_PIN | KBDATA_PIN | KBCLK_PIN, 1);
        gpio_setmode(KBRST_PORT, KBRST_PIN | KBDATA_PIN | KBCLK_PIN,
                     GPIO_SETMODE_OUTPUT_ODRAIN_25 | GPIO_SETMODE_PU);

        /* USB power enables */
        gpio_setv(GPIOA, GPIO9 | GPIO10, 0);  // active high
        gpio_setmode(GPIOA, GPIO9 | GPIO10, GPIO_SETMODE_OUTPUT_2);

        /* USER and DFU Buttons are externally pulled down */
        gpio_setmode(GPIOC, GPIO13 | GPIO14, GPIO_SETMODE_INPUT);
        return;
    }

    if (config.board_type == BOARD_TYPE_AMIGAPCI) {
        /* Attempt to capture previous state */
        uint psoff = gpio_get(PSON_PORT, PSON_PIN);
        gpio_setv(PSON_PORT, PSON_PIN, psoff ? 1 : 0);
        gpio_setmode(PSON_PORT, PSON_PIN, GPIO_SETMODE_OUTPUT_2);
    }

    gpio_setmode(PWRSW_PORT, PWRSW_PIN, GPIO_SETMODE_INPUT_PU);  // Power button

    gpio_setmode(FANTACH_PORT, FANTACH_PIN,
                 GPIO_SETMODE_PU |
                 GPIO_SETMODE_ALTFUNC_2 | GPIO_SETMODE_AF_AF2);
    gpio_setmode(FANPWM_PORT, FANPWM_PIN,
                 GPIO_SETMODE_PU |
                 GPIO_SETMODE_ALTFUNC_25 | GPIO_SETMODE_AF_AF3);

    gpio_setmode(STMRSTA_PORT, STMRSTA_PIN, GPIO_SETMODE_INPUT_PU);

    gpio_setv(FORWARD_PORT,
              FORWARD_PIN | BACK_PIN | LEFT_PIN | RIGHT_PIN | FIRE_PIN, 1);
    gpio_setmode(FORWARD_PORT, FORWARD_PIN | BACK_PIN | LEFT_PIN | RIGHT_PIN |
                 FIRE_PIN, GPIO_SETMODE_OUTPUT_ODRAIN_25 | GPIO_SETMODE_PU);

    gpio_setv(PotX_PORT, PotX_PIN | PotY_PIN, 1);
    gpio_setmode(PotX_PORT, PotX_PIN | PotY_PIN,
                 GPIO_SETMODE_OUTPUT_ODRAIN_25 | GPIO_SETMODE_PU);

    gpio_setv(KBRST_PORT, KBRST_PIN | KBDATA_PIN | KBCLK_PIN, 1);
    gpio_setmode(KBRST_PORT, KBRST_PIN | KBDATA_PIN | KBCLK_PIN,
                 GPIO_SETMODE_OUTPUT_ODRAIN_25 | GPIO_SETMODE_PU);

    gpio_setmode(VMON5_PORT, VMON5_PIN | VMON5SB_PIN | VMON3V3_PIN |
                 VMON1V2_PIN, GPIO_SETMODE_INPUT);
    gpio_setmode(VMON12_PORT, VMON12_PIN | VMONNEG12_PIN, GPIO_SETMODE_INPUT);

    gpio_setv(I2C_SCL_PORT, I2C_SCL_PIN | I2C_SDA_PIN, 1);
    gpio_setmode(I2C_SCL_PORT, I2C_SCL_PIN | I2C_SDA_PIN,
                 GPIO_SETMODE_OUTPUT_ODRAIN_25 | GPIO_SETMODE_PU);

    gpio_setmode(RTCEN_PORT, RTCEN_PIN | R_WA_PIN, GPIO_SETMODE_INPUT_PU);

    if (config.board_type == BOARD_TYPE_AMIGAPCI) {
        gpio_setv(HIDEN_PORT, HIDEN_PIN, 1);
        gpio_setmode(HIDEN_PORT, HIDEN_PIN, GPIO_SETMODE_OUTPUT);
    } else {
        gpio_setv(HIDEN_PORT, KEYJAM_HIDEN_PIN, 1);
        gpio_setmode(HIDEN_PORT, KEYJAM_HIDEN_PIN, GPIO_SETMODE_OUTPUT);
    }

    gpio_setv(D16_PORT, D16_PIN | D17_PIN | D18_PIN | D19_PIN, 1);
    gpio_setmode(D16_PORT, D16_PIN | D17_PIN | D18_PIN | D19_PIN,
                 GPIO_SETMODE_INPUT_PU);
    gpio_setmode(A2_PORT, A2_PIN | A3_PIN | A4_PIN | A5_PIN,
                 GPIO_SETMODE_INPUT_PU);

    if (config.board_type == BOARD_TYPE_APCIDEV) {
        // AmigaPCI STM32 dev board has this backwards
        /* USB_ENABLE needs to be open drain on dev board to fully turn off */
        gpio_setv(USB_ENABLE_PORT, USB_ENABLE_PIN, 1);  // active low
        gpio_setmode(USB_ENABLE_PORT, USB_ENABLE_PIN,
                     GPIO_SETMODE_OUTPUT_ODRAIN);
    } else {
        /* USB_ENABLE needs to be push-pull on AmigaPCI board to turn on */
        gpio_setv(USB_ENABLE_PORT, USB_ENABLE_PIN, 0);  // active high
        gpio_setmode(USB_ENABLE_PORT, USB_ENABLE_PIN, GPIO_SETMODE_OUTPUT_2);
    }
}
