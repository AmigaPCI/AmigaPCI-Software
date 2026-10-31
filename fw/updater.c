/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * STM32 firmware updater.
 */

typedef unsigned int uint;

#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include "clock.h"
#include "uart.h"
#include "gpio.h"
#include "printf.h"
#include "timer.h"
#include "keyboard.h"
#include "amigartc.h"
#include "persist.h"

#define CONSOLE_USART USART3

static void
uart_wait_done(uint32_t usart)
{
    /* Wait until the data has been transferred into the shift register. */
    int count = 0;

    while ((USART_SR(usart) & USART_SR_TC) == 0)
        if (count++ == 2000)
            break;  // Misconfigured hardware?
}

static void
uart_wait_send_ready(uint32_t usart)
{
    /* Wait until the data has been transferred into the shift register. */
    int count = 0;

    while ((USART_SR(usart) & USART_SR_TXE) == 0)
        if (count++ == 1000)
            break;  // Misconfigured hardware?
}

static void
uart_send(uint32_t usart, uint16_t data)
{
    USART_DR(usart) = (data & USART_DR_MASK);
}

static void
uart_send_blocking(uint32_t usart, uint16_t data)
{
    uart_wait_send_ready(usart);
    uart_send(usart, data);
}

void
uart_putchar(int ch)
{
    if (ch == '\n')
        uart_send_blocking(CONSOLE_USART, '\r');
    uart_send_blocking(CONSOLE_USART, (uint16_t) ch);
}

void
uart_puts(const char *str)
{
    while (*str != '\0')
        uart_putchar(*(str++));
}

static int
uart_getchar(void)
{
    if (USART_SR(CONSOLE_USART) & (USART_SR_RXNE | USART_SR_ORE)) {
        return (USART_DR(CONSOLE_USART) & USART_DR_MASK);
    }
    return (-1);
}

void
uart_puthex(uint32_t x)
{
    uint32_t digit;
    char buf[32];
    char *ptr = buf + sizeof (buf) - 1;
    *ptr = '\0';
    for (digit = 0; digit < 8; digit++) {
        *(--ptr) = "0123456789abcdef"[x & 0xf];
        x >>= 4;
    }
    uart_puts(ptr);
}

void
uart_init(void)
{
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_GPIOC);

    /* USART3 will use PC10 as TX and PC11 as RX */
    gpio_set_af(GPIOC, GPIO_AF7, GPIO10);  // PC10 AltFunc7 = USART3 TX
    gpio_set_af(GPIOC, GPIO_AF7, GPIO11);  // PC11 AltFunc7 = USART3 RX
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO11);

    /* Setup UART parameters. */
    usart_set_baudrate(CONSOLE_USART, 115200);
    usart_set_databits(CONSOLE_USART, 8);
    usart_set_stopbits(CONSOLE_USART, USART_STOPBITS_1);
    usart_set_mode(CONSOLE_USART, USART_MODE_TX_RX);
    usart_set_parity(CONSOLE_USART, USART_PARITY_NONE);
    usart_set_flow_control(CONSOLE_USART, USART_FLOWCONTROL_NONE);
    usart_enable(CONSOLE_USART);
}

static void
set_powerled(int state)
{
    if (state) {
        gpio_clear(POWER_LED_PORT, POWER_LED_PIN);
    } else {
        gpio_set(POWER_LED_PORT, POWER_LED_PIN);
    }
}

static void
led_init(void)
{
    gpio_mode_setup(POWER_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    POWER_LED_PIN);
}

__attribute__((format(__printf__, 1, 2)))
int printf(const char *fmt, ...)
{
    uart_puts(fmt);
    return (0);
}

void
main(void)
{
    uint run_timer = 0;
    uint64_t timeout;

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_GPIOH);

    persist_init();
    clock_init();

    uart_init();
    uart_puts("Updater waiting.\n");

    /* POWER_LED */
    led_init();
    set_powerled(1);          // Power LED on
    timer_init();
    amigartc_init();
    keyboard_init();
    timeout = timer_tick_plus_msec(10 * 60 * 1000);  // 10 minute timeout

    while (1) {
        set_powerled((run_timer & 0x000000ff) ? 0 : 1);  // flicker
        keyboard_poll();
        amigartc_poll();
        if ((run_timer & 0x000000ff) == 0) {
            if (timer_tick_has_elapsed(timeout))
                break;
            if (uart_getchar() == 0x03) {  // ^C
                uart_puts("^C Abort.\n");
                goto finish;
            }
        }
        run_timer++;
    }

    uart_puts("Updater timeout.\n");
finish:
    uart_wait_done(CONSOLE_USART);
    scb_reset_system();
}
