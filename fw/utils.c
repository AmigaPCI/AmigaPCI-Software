/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2020.
 *
 * ---------------------------------------------------------------------
 *
 * Generic POSIX function emulation.
 */

#include "printf.h"
#include "utils.h"
#include <stdbool.h>
#include <string.h>
#include "board.h"
#include "main.h"
#include "clock.h"
#include "gpio.h"

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/dbgmcu.h>
#define HCLK_FREQ clock_get_hclk()
#define APB1_FREQ clock_get_apb1()
#define APB2_FREQ clock_get_apb2()
#define DBGMCU_DEVID (DBGMCU_IDCODE & DBGMCU_IDCODE_DEV_ID_MASK)
#define DBGMCU_REVID ((DBGMCU_IDCODE & DBGMCU_IDCODE_REV_ID_MASK) >> 16)
#if defined(STM32F4)
#define FLASH_BASE 0x08000000U
#endif

static void SystemInit_post(void) { }

#ifdef STM32F103xE
#define COMPILE_CPU "STM32F103xE"
#elif defined(STM32F105xC)
#define COMPILE_CPU "STM32F105xC"
#elif defined(STM32F105xE)
#define COMPILE_CPU "STM32F105xE"
#elif defined(STM32F107xC)
#define COMPILE_CPU "STM32F107xC"
#elif defined(STM32F100xB)
#define COMPILE_CPU "STM32F100xB"
#elif defined(STM32F100xE)
#define COMPILE_CPU "STM32F100xE"
#elif defined(STM32F1) || defined(STM32F1XX)
#define COMPILE_CPU "STM32F1"

#elif defined(STM32F203)
#define COMPILE_CPU "STM32F203"
#elif defined(STM32F205)
#define COMPILE_CPU "STM32F205"
#elif defined(STM32F207)
#define COMPILE_CPU "STM32F207"
#elif defined(STM32F2) || defined(STM32F2XX)
#define COMPILE_CPU "STM32F2"

#elif defined(STM32F407xx)
#define COMPILE_CPU "STM32F407"
#elif defined(STM32F4) || defined(STM32F4XX)
#define COMPILE_CPU "STM32F4"
#else
#define COMPILE_CPU "unknown"
#endif

#ifndef SYSCFG_MEMRMP
#define SYSCFG_MEMRMP MMIO32(SYSCFG_BASE + 0x00)
#define SYSCFG_MEMRMP_MEM_MODE_0 0x01
#define SYSCFG_MEMRMP_MEM_MODE   0x03
#endif

/* Force a variable into the custom "persist" section */
#define SRAM_PERSIST __attribute__((section(".persist,\"aw\",%nobits@")))

/*
 * The System memory base appears at different addresses on different
 * STM32 processors.
 */
#if defined(STM32F103xE)
#define SYSTEM_MEMORY_BASE 0x1ffff000  // STM32F103
#elif defined(STM32F105xE) || defined(STM32F107xC)
#define SYSTEM_MEMORY_BASE 0x1fffb000  // STM32F105/STM32F107
#elif defined(STM32F205)
#define SYSTEM_MEMORY_BASE 0x1fff0000  // STM32F2xx
#else
#define SYSTEM_MEMORY_BASE 0x1fff0000  // STM32F4xx
#endif

#define RESET_TO_DFU_ROM_MAGIC 0xd0df00ba  // STM32 ROM DFU programmer
#define RESET_TO_DFU_MAGIC     0xd0df0bee  // Stand-alone DFU (in firmware)

/*
 * #define STM32F0_UDID_BASE            0x1ffff7ac
 * #define STM32F1_UDID_BASE            0x1ffff7e8
 * #define STM32F2_UDID_BASE            0x1fff7a10
 * #define STM32F3_UDID_BASE            0x1ffff7ac
 * #define STM32F4_UDID_BASE            0x1fff7a10
 * #define STM32F7_UDID_BASE            0x1ff0f420
 */
#define STM32_UDID_LEN                  12    // 96 bits
#define STM32_UDID_BASE                 DESIG_UNIQUE_ID_BASE

SRAM_PERSIST static uint32_t system_reset_to_dfu_magic;
char cpu_serial_str[32];

extern uint _binary_objs_usbdfu_bin_start;
extern uint _binary_objs_usbdfu_bin_end;
extern uint _binary_objs_usbdfu_bin_size;

/* Force exceptions/interrupts to use new VTOR */
static void dmb(void)
{
    __asm__ volatile("dmb");
}

void
reset_check(void)
{
#if 0
    if (system_reset_to_dfu_magic == RESET_TO_DFU_ROM_MAGIC) {
#else
    if ((system_reset_to_dfu_magic == RESET_TO_DFU_ROM_MAGIC) ||
        (system_reset_to_dfu_magic == RESET_TO_DFU_MAGIC)) {
#endif
        system_reset_to_dfu_magic = 1;

        /* Map system memory at address 0 */
        SYSCFG_MEMRMP &= ~SYSCFG_MEMRMP_MEM_MODE;
        SYSCFG_MEMRMP |= SYSCFG_MEMRMP_MEM_MODE_0;

#if 0
        /* XXX: The following did not increase changes of DFU working */

        /* Power LED (TEST) */
        rcc_periph_clock_enable(RCC_GPIOA);
        gpio_setmode(GPIOA, GPIO8, GPIO_SETMODE_OUTPUT_2);
        gpio_setv(GPIOA, GPIO8, 0);

        /* Power off */
        rcc_periph_clock_enable(RCC_GPIOD);
        gpio_setmode(GPIOD, GPIO2, GPIO_SETMODE_OUTPUT_2);
        gpio_setv(GPIOD, GPIO2, 1);

        /* Stop PB10/PB11 PB1/PB13 */
        rcc_periph_clock_enable(RCC_GPIOB);
        gpio_setmode(GPIOB, GPIO10 | GPIO11 | GPIO5 | GPIO13,
                     GPIO_SETMODE_OUTPUT_2);
        gpio_setv(GPIOB, GPIO10 | GPIO11 | GPIO5 | GPIO13, 0);
#endif

        uint32_t  addr = SYSTEM_MEMORY_BASE;
        uint32_t *base = (uint32_t *)(uintptr_t) addr;

        /* Set the vector table pointer */
        SCB_VTOR = addr;
        dmb();

        /* Set the stack pointer */
        __asm__("MSR msp, %0" : : "r" (ADDR32(base)[0]));   // SP = base[0]
        /* Set the program counter (jump) */
        __asm__("BX %0\n\t" : : "r" (ADDR32(base)[1]));     // PC = base[1]
#if 0
    } else if (system_reset_to_dfu_magic == RESET_TO_DFU_MAGIC) {
        system_reset_to_dfu_magic = 3;

        uint len = ADDR8(&_binary_objs_usbdfu_bin_end) -
                   ADDR8(&_binary_objs_usbdfu_bin_start);
        uint32_t  addr = 0x20000000;  // SRAM Base
        uint32_t *base = ADDR32(addr);

        /* Copy USB DFU to SRAM */
        memcpy(base, &_binary_objs_usbdfu_bin_start, len);

        /* Set the vector table pointer */
        SCB_VTOR = addr;
        dmb();

        /* Set the stack pointer */
        __asm__("MSR msp, %0" : : "r" (ADDR32(base)[0]));   // SP = base[0]
        /* Set the program counter (jump) */
        __asm__("BX %0\n\t" : : "r" (ADDR32(base)[1]));     // PC = base[1]
#endif
    } else {
        system_reset_to_dfu_magic = 2;

        /*
         * STM32F107/STM32F407 ROM DFU does not correctly re-initialize
         * VTOR following exit from DFU mode, it must be done here.
         */
        SCB_VTOR = FLASH_BASE;  // Set the vector table pointer
        dmb();

        /* Continue normal HAL SystemInit() */
        SystemInit_post();
    }
}

void
reset_cpu(void)
{
    scb_reset_system();
}

void
reset_dfu(int in_rom)
{
    if (in_rom)
        system_reset_to_dfu_magic = RESET_TO_DFU_ROM_MAGIC;
    else
        system_reset_to_dfu_magic = RESET_TO_DFU_MAGIC;
    scb_reset_system();
}

uint reset_state_flags;

void
get_reset_reason(void)
{
    reset_state_flags = RCC_CSR;
    RCC_CSR = RCC_CSR_RMVF;  // Clear reset flags

    if (reset_state_flags & RCC_CSR_PORRSTF)
        cold_poweron = 1;
}

void
show_reset_reason(void)
{
    if (reset_state_flags & RCC_CSR_LPWRRSTF)
        printf("    %s\n", "Low-power reset");
    if (reset_state_flags & RCC_CSR_WWDGRSTF)
        printf("    %s\n", "Window Watchdog reset");
    if (reset_state_flags & RCC_CSR_IWDGRSTF)
        printf("    %s\n", "Independent Watchdog reset");
    if (reset_state_flags & RCC_CSR_PORRSTF) {
        printf("    %s\n", "Power-on reset");
    } else if (reset_state_flags & RCC_CSR_SFTRSTF) {
        printf("    %s\n", "Software reset");
    } else if (reset_state_flags & RCC_CSR_PINRSTF) {
        printf("    %s\n", "NRST pin reset");
    }
}

/**
 * get_cpu_serial() reads the STM32 Unique Device ID from the CPU's system
 *                  memory area of the Flash memory module. It converts
 *                  the ID to a printable string.
 *
 * @param [out] buf - A buffer to hold the converted serial number.
 *
 * @return      None.
 */
static void
get_cpu_serial(char *buf)
{
    uint len = 0;
    uint pos;

    for (pos = 0; pos < STM32_UDID_LEN; pos++) {
        uint8_t temp  = *ADDR8(STM32_UDID_BASE + pos);
        uint8_t ch_hi = (temp >> 4) + '0';
        uint8_t ch_lo = (temp & 0xf) + '0';

        if (temp == 0xff)
            continue;
        if (((temp >= '0') && (temp <= '9')) ||
            ((temp >= 'A') && (temp <= 'Z')) ||
            ((temp >= 'a') && (temp <= 'z'))) {
            /* Show ASCII directly */
            buf[len++] = (char) temp;
            continue;
        }
        if (ch_hi > '9')
            ch_hi += 'a' - '0' - 10;
        if (ch_lo > '9')
            ch_lo += 'a' - '0' - 10;

        buf[len++] = ch_hi;
        buf[len++] = ch_lo;
    }
    buf[len++] = '\0';
}

void
identify_cpu(void)
{
    const char *runtime_cpu;
    const char *cortex;
    switch (SCB_CPUID) {
        case 0x410fc241:
            runtime_cpu = "STM32F4";
            break;
        case 0x411fc231:
            runtime_cpu = "STM32F1";
            break;
        case 0x412fc230:
            runtime_cpu = "STM32F2/STM32L";
            break;
        case 0x412fc231:
            runtime_cpu = "GD32F1";
            break;
        default:
            runtime_cpu = "?";
            break;
    }
    switch ((SCB_CPUID >> 4) & 0xfff) {
        case 0xc20:
            cortex = "M0";
            break;
        case 0xc21:
            cortex = "M1";
            break;
        case 0xc23:
            cortex = "M3";
            break;
        case 0xc24:
            cortex = "M4";
            break;
        case 0xc27:
            cortex = "M7";
            break;
        case 0xc60:
            cortex = "M0+";
            break;
        default:
            cortex = "?";
            break;
    }
    printf("    CPUID=%08lx Dev=%04lx Rev=%04lx\n",
           SCB_CPUID, DBGMCU_DEVID, DBGMCU_REVID);
    printf("    Cortex %s %s", cortex, runtime_cpu);
    if (DBGMCU_DEVID != 0) {
        const char *core_type;
        const char *core_rev = "?";
        switch (DBGMCU_DEVID) {
            case 0x0410:
                core_type = "Medium-density";
                switch (DBGMCU_REVID) {
                    case 0x0000:
                        core_rev = "A";
                        break;
                    case 0x2000:
                        core_rev = "B";
                        break;
                    case 0x2001:
                        core_rev = "Z";
                        break;
                    case 0x2003:
                        core_rev = "1, 2, 3, X or Y";
                        break;
                    default:
                        break;
                }
                break;
            case 0x0411:  // STM32F4
                switch ((SCB_CPUID >> 4) & 0xfff) {
                    case 0xc23:
                        core_type = "STM32F205";
                        switch (DBGMCU_REVID) {
                            case 0x1000:
                                core_rev = "A";
                                break;
                            case 0x1001:
                                core_rev = "Z";
                                break;
                            case 0x2000:
                                core_rev = "B";
                                break;
                            case 0x2001:
                                core_rev = "U";
                                break;
                            case 0x2003:
                                core_rev = "X";
                                break;
                            case 0x2007:
                                core_rev = "1";
                                break;
                            case 0x200f:
                                core_rev = "V";
                                break;
                            case 0x201f:
                                core_rev = "2 or 3";
                                break;
                            case 0x202f:
                                core_rev = "4";
                                break;
                            case 0x203f:
                                core_rev = "5";
                                break;
                            case 0x204f:
                                core_rev = "6";
                                break;
                            break;
                        }
                        break;
                    case 0xc24:
                        core_type = "STM32F457";
                        break;
                    default:
                        core_type = "?";
                        break;
                }
                break;
            case 0x0412:
                core_type = "Low-density";
                switch (DBGMCU_REVID) {
                    case 0x1000:
                        core_rev = "A";
                        break;
                    default:
                        break;
                }
                break;
            case 0x0413:  // STM32F405/STM32F407 and STM32F415/STM32F417
                core_type = "STM32F405/07";
                switch (DBGMCU_REVID) {
                    case 0x1000:
                        core_rev = "A";
                        break;
                    case 0x1001:
                        core_rev = "Z";
                        break;
                    case 0x1003:
                        core_rev = "1";
                        break;
                    case 0x1007:
                        core_rev = "2";
                        break;
                    case 0x100F:
                        core_rev = "Y/4";
                        break;
                    case 0x101F:
                        core_rev = "5/6";
                        break;
                    default:
                        break;
                }
                break;
            case 0x0414:
                core_type = "High-density";
                switch (DBGMCU_REVID) {
                    case 0x1000:
                        core_rev = "A or 1";
                        break;
                    case 0x1001:
                        core_rev = "Z";
                        break;
                    case 0x1003:
                        core_rev = "1, 2, 3, X or Y";
                        break;
                    default:
                        break;
                }
                break;
            case 0x0418:
                core_type = "Connectivity";
                switch (DBGMCU_REVID) {
                    case 0x1000:
                        core_rev = "A";
                        break;
                    case 0x1001:
                        core_rev = "Z";
                        break;
                    default:
                        break;
                }
                break;
            case 0x0419:  // STM32F42xxx and STM32F43xxx
                core_type = "STM32F42/F43";
                switch (DBGMCU_REVID) {
                    case 0x1000:
                        core_rev = "A";
                        break;
                    case 0x1003:
                        core_rev = "Y";
                        break;
                    case 0x1007:
                        core_rev = "1";
                        break;
                    case 0x2001:
                        core_rev = "3";
                        break;
                    case 0x2003:
                        core_rev = "5/B";
                        break;
                    default:
                        break;
                }
                break;
            case 0x0430:
                core_type = "XL-density";
                switch (DBGMCU_REVID) {
                    case 0x1000:
                        core_rev = "A or 1";
                        break;
                    default:
                        break;
                }
                break;
            default:
                core_type = "Unknown-density";
                break;
        }
        printf("    %s rev %s\n", core_type, core_rev);
    }
    printf("    HCLK=%ld MHz  APB1=%ld MHz  APB2=%ld MHz\n",
           HCLK_FREQ / 1000000, APB1_FREQ / 1000000, APB2_FREQ / 1000000);

#if 0
#ifdef STM32F1XX
    printf("    FlashSize=%u KB FlashPage=%u KB\n", *ADDR16(FLASHSIZE_BASE),
            FLASH_PAGE_SIZE / 1024);
#else
    printf("    FlashSize=%u KB\n", *ADDR16(FLASHSIZE_BASE));
#endif
#endif
    get_cpu_serial(cpu_serial_str);
}

/* Deal with annoying newlib warnings */
void _close(void);
void _close(void) { }
void _close_r(void) __attribute__((alias("_close")));
void _lseek(void)   __attribute__((alias("_close")));
void _read(void)    __attribute__((alias("_close")));
void _write(void)   __attribute__((alias("_close")));
