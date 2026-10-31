/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * Amiga keyboard interface handling.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "config.h"
#include "keyboard.h"
#include "kbrst.h"
#include "power.h"
#include "timer.h"
#include "utils.h"
#include "uart.h"
#include "usb.h"
#include "gpio.h"
#include "mouse.h"
#include "irq.h"
#include "hid_kbd_codes.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include "amiga_kbd_codes.h"
#include "amigartc.h"
#include "bec_cmd.h"
#include "msg.h"

/* Keyboard scancode capture globals */
static uint64_t amiga_kbd_msg_in_timeout;  // Message from Amiga timeout

static uint8_t  bitcap_buf[sizeof (bec_msg_inbuf)];
static uint     kbd_msg_rx_cur;

static inline void
set_kbclk_0(void)
{
    *ADDR32(BND_IO(KBCLK_PORT + GPIO_ODR_OFFSET, low_bit(KBCLK_PIN))) = 0;
}

static inline void
set_kbclk_1(void)
{
    *ADDR32(BND_IO(KBCLK_PORT + GPIO_ODR_OFFSET, low_bit(KBCLK_PIN))) = 1;
}

static inline void
set_kbdat_0(void)
{
    *ADDR32(BND_IO(KBDATA_PORT + GPIO_ODR_OFFSET, low_bit(KBDATA_PIN))) = 0;
}

static inline void
set_kbdat_1(void)
{
    *ADDR32(BND_IO(KBDATA_PORT + GPIO_ODR_OFFSET, low_bit(KBDATA_PIN))) = 1;
}

static inline uint
get_kbdat(void)
{
    return (*ADDR32(BND_IO(KBDATA_PORT + GPIO_IDR_OFFSET, low_bit(KBDATA_PIN))));
}

static inline uint
get_kbclk(void)
{
    return (*ADDR32(BND_IO(KBCLK_PORT + GPIO_IDR_OFFSET, low_bit(KBCLK_PIN))));
}

static inline uint
get_kbclk_output_value(void)
{
    return (*ADDR32(BND_IO(KBDATA_PORT + GPIO_ODR_OFFSET, low_bit(KBDATA_PIN))));
}

static inline void
kbd_receive_debug(uint which, uint value)
{
#if 0
    uint addr;
    switch (which) {
        default:
        case 0:
            addr = BND_IO(D16_PORT + GPIO_ODR_OFFSET, low_bit(D16_PIN));
            break;
        case 1:
            addr = BND_IO(D17_PORT + GPIO_ODR_OFFSET, low_bit(D17_PIN));
            break;
        case 2:
            addr = BND_IO(D18_PORT + GPIO_ODR_OFFSET, low_bit(D18_PIN));
            break;
        case 3:
            addr = BND_IO(D19_PORT + GPIO_ODR_OFFSET, low_bit(D19_PIN));
            break;
    }
    *ADDR32(addr) = value;
#endif
}

static void
keyboard_handle_msg_timeout(void)
{
    if (kbd_msg_rx_cur != 0) {
        if (amiga_kbd_msg_in_timeout == 0) {
            amiga_kbd_msg_in_timeout = timer_tick_plus_msec(1000);
        } else {
            if (timer_tick_has_elapsed(amiga_kbd_msg_in_timeout)) {
                /* Message did not arrive in time */
#if 0
                printf("E %x ", kbd_msg_rx_cur);
                uint max = (kbd_msg_rx_cur + 7) / 8;
                for (uint pos = 0; pos < max; pos++)
                    printf(" %02x", bitcap_buf[pos]);
                printf("\n");
#endif
                amiga_kbd_msg_in_timeout = 0;
                kbd_msg_rx_cur = 0;
                return;
            }
        }
    }
}

static void
keyboard_handle_msg_from_amiga(void)
{
    uint rxpos = kbd_msg_rx_cur >> 3;
    if (rxpos < BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN)
        return;  // Minimum message
    uint expected = BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN +
                    ((bitcap_buf[3] << 8) | bitcap_buf[4]);
    if (rxpos < expected)
        return;
    memcpy(bec_msg_inbuf, bitcap_buf, rxpos);

    amiga_kbd_msg_in_timeout = 0;
    kbd_msg_rx_cur = 0;

    msg_source = 1;
    if (msg_process_fast() == 0) {
        /*
         * bec_msg_in is needed by amigartc_poll() to decide whether
         * to call msg_process_slow().
         */
        bec_msg_in = rxpos * 2;
    }
}

void
exti9_5_isr(void)
{
    /*
     * More than a single EXTI line (5 - 9) share this vector.
     * If more are mapped, the handler must check the EXTI->PR (pending
     * register) to determine the EXTI line responsible for the interrupt.
     */
    kbd_receive_debug(1, 0);
    uint16_t clkdat = gpio_get(KBCLK_PORT, KBCLK_PIN | KBDATA_PIN);
    uint     next;
    exti_reset_request(EXTI9);
    if ((clkdat & KBCLK_PIN) == 0) {
        kbd_receive_debug(1, 1);
        return;
    }
#if 0
    if (amiga_keyboard_ack_expected) {
        amiga_keyboard_ack_expected = 0;
        kbd_receive_debug(1, 1);
        return;
    }
#endif
    next = kbd_msg_rx_cur + 1;
    if (next >= ARRAY_SIZE(bitcap_buf) * 8) {
        kbd_receive_debug(1, 1);
        return;  // No space
    }

    bitcap_buf[kbd_msg_rx_cur >> 3] <<= 1;
    if (clkdat & KBDATA_PIN)
        bitcap_buf[kbd_msg_rx_cur >> 3] |= 1;
    kbd_msg_rx_cur = next;

    keyboard_handle_msg_from_amiga();
    kbd_receive_debug(1, 1);
}

void
keyboard_reply_msg(void)
{
    uint pos;
    uint mask;

    exti_disable_request(EXTI9);
    for (pos = 0; pos < bec_msg_out_max; pos++) {
        uint8_t code = bec_msg_outbuf[pos];
        for (mask = 0x80; mask != 0; mask >>= 1) {
            if (code & mask)
                set_kbdat_1();
            else
                set_kbdat_0();
            timer_delay_usec(19);
            set_kbclk_0();
            timer_delay_usec(20);
            if ((code & mask) && (get_kbdat() == 0)) {
                timer_delay_usec(10);
                if (get_kbdat() == 0) {
                    uart_puts("KBDAT stuck low at ");
                    uart_puthex(pos);
                    uart_putchar('.');
                    uart_puthex(mask);
                    uart_puts(" of ");
                    uart_puthex(bec_msg_out_max);
                    uart_puts(" [");
                    uart_puthex(code);
                    uart_puts("]\n");
                    set_kbclk_1();
                    timer_delay_usec(10);
                    goto send_fail;
                }
            }
            set_kbclk_1();
            timer_delay_usec(20);
        }
        timer_delay_usec(0);
    }
send_fail:
    set_kbdat_1();

    exti_reset_request(EXTI9);
    exti_enable_request(EXTI9);
}

void
keyboard_poll(void)
{
    /* Handle incoming BEC message timeout */
    keyboard_handle_msg_timeout();
}

void
keyboard_init(void)
{
#if 0
    /* Map KBDATA (PC8) to EXTI8 */
    exti_select_source(EXTI8, KBDATA_PORT);  // GPIOC
    exti_set_trigger(EXTI8, EXTI_TRIGGER_BOTH);
    exti_enable_request(EXTI8);
    exti_reset_request(EXTI8);
#endif
    gpio_set(KBDATA_PORT, KBDATA_PIN | KBCLK_PIN);
    gpio_mode_setup(KBDATA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    KBDATA_PIN | KBCLK_PIN);
    gpio_set_output_options(KBDATA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ,
                            KBDATA_PIN | KBCLK_PIN);

    /* Map KBCLK (PC9) to EXTI9 */
    exti_select_source(EXTI9, KBCLK_PORT);  // GPIOC
    exti_set_trigger(EXTI9, EXTI_TRIGGER_RISING);

    /* Enable KBCLK interrupt handler */
    exti_reset_request(EXTI9);
    exti_enable_request(EXTI9);
    nvic_clear_pending_irq(NVIC_EXTI9_5_IRQ);
    nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x30);
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);

#if 0
    printf("BND KBCLK_IN=%x KBCLK_OUT=%x KBDAT_IN=%x KBDAT_OUT=%x\n",
           BND_IO(KBCLK_PORT + GPIO_IDR_OFFSET, low_bit(KBCLK_PIN)),
           BND_IO(KBCLK_PORT + GPIO_ODR_OFFSET, low_bit(KBCLK_PIN)),
           BND_IO(KBDATA_PORT + GPIO_IDR_OFFSET, low_bit(KBDATA_PIN)),
           BND_IO(KBDATA_PORT + GPIO_ODR_OFFSET, low_bit(KBDATA_PIN)));
#endif

#if 0
    printf("GPIO D16 D17 debug\n");
    gpio_setv(D16_PORT, D16_PIN | D17_PIN, 1);
    gpio_setmode(D16_PORT, D16_PIN | D17_PIN | D18_PIN | D19_PIN,
                 GPIO_SETMODE_OUTPUT_25);
#endif
}
