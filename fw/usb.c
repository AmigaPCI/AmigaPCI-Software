/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * STM32 USB stack support
 */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "gpio.h"
#include "printf.h"
#include "utils.h"
#include "timer.h"
#include "clock.h"
#include "config.h"
#include "usb.h"
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

#ifdef USE_TINYUSB
#include "tinyusb.h"
#endif
#ifdef USE_STMCUBEUSB
#include "cubeusb.h"
#endif

uint usb_debug_mask;
uint usb_keyboard_terminal;
volatile uint8_t usb_keyboard_count;
volatile uint8_t usb_mouse_count;
volatile uint8_t usb_joystick_count;
uint8_t usb_is_powered;
static uint64_t usb_power_timer;

typedef struct {
    uint16_t regs_offset;
    uint8_t  regs_start_end;
    uint8_t  regs_stride;
    char     regs_name[12];
} usb_regs_t;

static const usb_regs_t usb_regs[] = {
    { 0x000, 0x01,    0, "GOTGCTL" },
    { 0x004, 0x01,    0, "GOTGINT" },
    { 0x008, 0x01,    0, "GAHBCFG" },
    { 0x00c, 0x01,    0, "GUSBCFG" },
    { 0x010, 0x01,    0, "GRSTCTL" },
    { 0x014, 0x01,    0, "GINTSTS" },
    { 0x018, 0x01,    0, "GINTMSK" },
    { 0x01c, 0x01,    0, "GRXSTSR" },
    { 0x024, 0x01,    0, "GRXFSIZ" },
    { 0x028, 0x01,    0, "DIEPTXF0" },
    { 0x02c, 0x01,    0, "HNPTXSTS" },
    { 0x038, 0x01,    0, "GCCFG" },
    { 0x03c, 0x01,    0, "CID" },
    { 0x100, 0x01,    0, "HPTXFSIZ" },
    { 0x104, 0x14,    4, "DIEPTXF%x" },
    /*
     * Host mode registers. Only the channel registers implemented by both
     * cores are read: on the OTG_FS core, channel offsets 0x04 and
     * 0x14-0x1c are unimplemented and appear to alias core registers
     * (they read back the GRXSTSR and TX FIFO status values). Reading an
     * alias of GRXSTSP (the receive status pop register) would pop an
     * entry from the receive FIFO under the driver; a "dl 50000500 100"
     * over that range hung the BEC with a device active.
     */
    { 0x400, 0x01,    0, "HCFG" },
    { 0x404, 0x01,    0, "HFIR" },
    { 0x408, 0x01,    0, "HFNUM" },
    { 0x410, 0x01,    0, "HPTXSTS" },
    { 0x414, 0x01,    0, "HAINT" },
    { 0x418, 0x01,    0, "HAINTMSK" },
    { 0x440, 0x01,    0, "HPRT" },
    { 0x500, 0x08, 0x20, "HCCHAR%x" },
    { 0x508, 0x08, 0x20, "HCINT%x" },
    { 0x50c, 0x08, 0x20, "HCINTMSK%x" },
    { 0x510, 0x08, 0x20, "HCTSIZ%x" },
    { 0x800, 0x01,    0, "DCFG" },
    { 0x804, 0x01,    0, "DCTL" },
    { 0x808, 0x01,    0, "DSTS" },
    { 0x810, 0x01,    0, "DIEPMSK" },
    { 0x814, 0x01,    0, "DOEPMSK" },
    { 0x818, 0x01,    0, "DAINT" },
    { 0x81c, 0x01,    0, "DAINTMSK" },
    { 0x828, 0x01,    0, "DVBUSDIS" },
    { 0x82c, 0x01,    0, "DVBUSPULSE" },
    { 0x834, 0x01,    0, "DIEPEMPMSK" },
    { 0x900, 0x04, 0x20, "DIEPCTL%x" },
    { 0x908, 0x04, 0x20, "DIEPINT%x" },
    { 0x910, 0x04, 0x20, "DIEPTSIZ%x" },
    { 0x918, 0x04, 0x20, "DTXFSTS%x" },
    { 0xb00, 0x04, 0x20, "DOEPCTL%x" },
    { 0xb08, 0x04, 0x20, "DOEPINT%x" },
    { 0xb10, 0x04, 0x20, "DOEPTSIZ%x" },
    { 0xe00, 0x01,    0, "PCGCCTL" },
};

void
usb_show_regs(void)
{
    uint pos;
    uint host;
    uint cur;
    char namebuf[12];

    printf("%26s%23s\n", "USB0", "USB1");
    for (pos = 0; pos < ARRAY_SIZE(usb_regs); pos++) {
        uint start = usb_regs[pos].regs_start_end >> 4;
        uint end   = usb_regs[pos].regs_start_end & 0xf;
        uint off = 0;
        for (cur = start; cur < end; cur++) {
            snprintf(namebuf, sizeof (namebuf), usb_regs[pos].regs_name, cur);
            printf("  %-10s ", namebuf);
            for (host = 0; host < 2; host++) {
                uint32_t base = (host == 0) ? USB_OTG_FS_BASE : USB_OTG_HS_BASE;
                uint32_t addr  = base + usb_regs[pos].regs_offset + off;
                printf("[%08lx] %08lx", addr, *ADDR32(addr));
                if (host == 0)
                    printf("    ");
            }
            printf("\n");
            off += usb_regs[pos].regs_stride;
        }
    }
    for (host = 0; host < 2; host++) {
        uint32_t base = (host == 0) ? USB_OTG_FS_BASE : USB_OTG_HS_BASE;
        uint32_t val = *ADDR32(base + 0x4); // GOTGINT
        printf("\n  USB%u GOTGINT %08lx", host, val);
        if (val & (1 << 19))
            printf(" DBCDNE");
        if (val & (1 << 18))
            printf(" ADTOCHG");
        if (val & (1 << 17))
            printf(" HNGDET");
        if (val & (1 << 8))
            printf(" SRSSCHG");
        if (val & (1 << 2))
            printf(" SEDET");

        val = *ADDR32(base + 0x14); // GINTSTS
        printf("\n  USB%u GINTSTS %08lx", host, val);
        if (val & (1 << 31))
            printf(" WKUPINT");
        if (val & (1 << 30))
            printf(" SRQINT");
        if (val & (1 << 29))
            printf(" DISCINT");
        if (val & (1 << 28))
            printf(" CIDSCHG");
        if (val & (1 << 26))
            printf(" PTXFE");
        if (val & (1 << 25))
            printf(" HCINT");
        if (val & (1 << 24))
            printf(" HPRTINT");
        if (val & (1 << 21))
            printf(" IPXFR");
        if (val & (1 << 20))
            printf(" IISOIXFR");
        if (val & (1 << 19))
            printf(" OEPINT");
        if (val & (1 << 18))
            printf(" IEPINT");
        if (val & (1 << 15))
            printf(" EOPF");
        if (val & (1 << 14))
            printf(" ISOODRP");
        if (val & (1 << 13))
            printf(" ENUMDNE");
        if (val & (1 << 12))
            printf(" USBRST");
        if (val & (1 << 11))
            printf(" USBSUSP");
        if (val & (1 << 10))
            printf(" ESUSP");
        if (val & (1 << 7))
            printf(" GONAKEFF");
        if (val & (1 << 6))
            printf(" GINAKEFF");
        if (val & (1 << 5))
            printf(" NPTXFE");
        if (val & (1 << 4))
            printf(" RXFLVL");
        if (val & (1 << 3))
            printf(" SOF");
        if (val & (1 << 2))
            printf(" OTGINT");
        if (val & (1 << 1))
            printf(" MMIS");
        if (val & (1 << 0))
            printf(" CMOD");
        printf("\n");
    }
}

/*
 * usb_set_power
 * -------------
 * Turns on or off power to the USB device ports.
 *
 * If state = 1, power is enabled.
 * If state = 0, power is disabled.
 */
void
usb_set_power(int state)
{
    uint enable = (state == USB_SET_POWER_ON) ? 1 : 0;

    if (config.board_type == BOARD_TYPE_APCIDEV)
        enable = !enable; // AmigaPCI STM32 dev board has this backwards

    if (config.board_type == BOARD_TYPE_KEYJAM)
        gpio_setv(GPIOA, GPIO9 | GPIO10, enable);
    else
        gpio_setv(USB_ENABLE_PORT, USB_ENABLE_PIN, enable);

    usb_power_timer = timer_tick_plus_msec(500);
    usb_is_powered = state;
}

void
usb_init(void)
{
    rcc_periph_clock_enable(RCC_SYSCFG);
    rcc_periph_clock_enable(RCC_OTGFS);
    rcc_periph_clock_enable(RCC_OTGHS);

    gpio_setmode(USB1_DM_PORT, USB1_DM_PIN | USB1_DP_PIN,
                 GPIO_SETMODE_ALTFUNC_100 | GPIO_SETMODE_AF_AF10);
    gpio_setmode(USB2_DM_PORT, USB2_DM_PIN | USB2_DP_PIN,
                 GPIO_SETMODE_ALTFUNC_100 | GPIO_SETMODE_AF_AF12);

    nvic_set_priority(NVIC_OTG_FS_IRQ, 0x40);
    nvic_set_priority(NVIC_OTG_HS_IRQ, 0x40);
    nvic_set_priority(NVIC_OTG_HS_WKUP_IRQ, 0x40);
    nvic_set_priority(NVIC_OTG_HS_EP1_IN_IRQ, 0x40);
    nvic_set_priority(NVIC_OTG_HS_EP1_OUT_IRQ, 0x40);

    nvic_enable_irq(NVIC_OTG_FS_IRQ);    // USB0 ISR
    nvic_enable_irq(NVIC_OTG_HS_IRQ);    // USB1 ISR
    nvic_enable_irq(NVIC_OTG_HS_WKUP_IRQ);
    nvic_enable_irq(NVIC_OTG_HS_EP1_IN_IRQ);
    nvic_enable_irq(NVIC_OTG_HS_EP1_OUT_IRQ);

#ifdef USE_TINYUSB
    tinyusb_init();
#endif
#ifdef USE_STMCUBEUSB
    cubeusb_init();
#endif

    usb_set_power(1);
}

void
usb_shutdown(uint mode)
{
    if (mode == 1) {
        nvic_disable_irq(NVIC_OTG_FS_IRQ);  // USB0 ISR
        nvic_disable_irq(NVIC_OTG_HS_IRQ);  // USB1 ISR
        nvic_disable_irq(NVIC_OTG_HS_WKUP_IRQ);
        nvic_disable_irq(NVIC_OTG_HS_EP1_IN_IRQ);
        nvic_disable_irq(NVIC_OTG_HS_EP1_OUT_IRQ);

        rcc_periph_clock_disable(RCC_OTGFS);
        rcc_periph_clock_disable(RCC_OTGHS);
    } else {
#ifdef USE_TINYUSB
        tinyusb_shutdown();
#endif
#ifdef USE_STMCUBEUSB
        cubeusb_shutdown();
#endif
    }
    usb_set_power(0);
}

void
usb_poll(void)
{
    if (usb_power_timer && !timer_tick_has_elapsed(usb_power_timer))
        return;

#ifdef USE_TINYUSB
    tinyusb_poll();
#endif
#ifdef USE_STMCUBEUSB
    cubeusb_poll();
#endif
}
