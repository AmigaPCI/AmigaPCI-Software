/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2020.
 *
 * ---------------------------------------------------------------------
 *
 * Command implementations.
 */

#include "printf.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "board.h"
#include "main.h"
#include "cmdline.h"
#include <stdbool.h>
#include "timer.h"
#include "uart.h"
#include "cmds.h"
#include "gpio.h"
#include "amigartc.h"
#include "pcmds.h"
#include "adc.h"
#include "utils.h"
#include "usb.h"
#include "irq.h"
#include "config.h"
#include "fan.h"
#include "hiden.h"
#include "kbrst.h"
#include "keyboard.h"
#include "led.h"
#include "mouse.h"
#include "power.h"
#include "rtc.h"
#include "sensor.h"
#include "spi.h"
#include "spi_flash.h"

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/memorymap.h>
#define GPIOA_BASE GPIO_PORT_A_BASE
#define GPIOB_BASE GPIO_PORT_B_BASE
#define GPIOC_BASE GPIO_PORT_C_BASE
#define GPIOD_BASE GPIO_PORT_D_BASE
#define GPIOE_BASE GPIO_PORT_E_BASE

#ifndef SRAM_BASE
#define SRAM_BASE 0x20000000U
#endif

#define ROM_BANKS 8

#if defined(STM32F4)
#define FLASH_BASE FLASH_MEM_INTERFACE_BASE
#endif

const char cmd_amiga_help[] =
"amiga keyboard - act as Amiga keyboard\n"
"amiga status   - show Amiga status";

const char cmd_cpu_help[] =
"cpu hardfault - cause CPU hard fault (bad address)\n"
"cpu regs      - show CPU registers";

const char cmd_fan_help[] =
"fan auto        - automatically adjust fan speed\n"
"fan on          - turn fan on\n"
"fan off         - turn fan off\n"
"fan speed <num> - set fan speed to specified %%\n";

const char cmd_gpio_help[] =
"gpio [name=value/mode/?] - display or set GPIOs";

const char cmd_reset_help[] =
"reset              - reset CPU\n"
"reset amiga [hold] - reset Amiga using KBRST (hold leaves it in reset)\n"
"reset amiga long   - reset Amiga with a long reset (change ROM image)\n"
"reset dfu[rom]     - reset into DFU programming mode\n"
"reset reason       - show STM32 previous reset reason\n"
"reset usb          - reset and restart USB interface";

const char cmd_set_help[] =
"set cpu_temp_bias <num>  - Bias (+/-) for CPU temperature\n"
"set debug <flags> [save] - Debug flags\n"
"set defaults [key|mouse] - Force settings to default values\n"
"set fan_rpm_max <num>    - Fan maximum speed in RPM\n"
"set fan_speed <num>|auto - Fan speed\n"
"set fan_speed_min <num>  - Fan speed minimum percent\n"
"set fan_temp_max <num>   - CPU temp for max fan speed\n"
"set fan_temp_min <num>   - CPU temp for min fan speed\n"
"set flags <flags> [save] - Config flags\n"
"set mouse_div_x <num>    - Mouse X speed divisor\n"
"set mouse_div_y <num>    - Mouse Y speed divisor\n"
"set mouse_mul_x <num>    - Mouse X speed multiplier\n"
"set mouse_mul_y <num>    - Mouse Y speed multiplier\n"
"set name <name>          - Board name\n"
"set pson <num>           - Power on mode (1=On at AC restore)\n"
"set time <y/m/d>|<h:m:s> - RTC time and/or date";

const char cmd_spi_help[] =
"spi erase <dev> <addr> <len> - erase SPI flash or 128K sect; <len> optional\n"
"spi id <dev>                 - report SPI flash chip vendor and id\n"
"spi list                     - show SPI devices per chip select\n"
"spi read <dev> <addr> <len>  - read binary data from SPI flash (to terminal)\n"
"spi write <dev> <addr> <len> - write binary data to SPI flash (from terminal)";

const char cmd_power_help[] =
"power cycle - cycle the power supply off/on\n"
"power on    - turn on power supply\n"
"power off   - turn off power supply\n"
"power show  - display current power status";

const char cmd_snoop_help[] =
"snoop        - capture and report ROM transactions\n"
"snoop addr   - hardware capture A0-A19\n"
"snoop lo     - hardware capture A0-A15 D0-D15\n"
"snoop hi     - hardware capture A0-A15 D16-D31";

const char cmd_usb_help[] =
   "usb debug <mask> - set debug mask\n"
   "usb disable      - reset and disable USB\n"
   "usb kbd [on|off] - take input from USB keyboard\n"
   "usb ls [v] [c]   - list USB devices present\n"
   "usb off          - power off USB ports\n"
   "usb on           - power on USB ports\n"
   "usb regs         - display USB device registers\n"
   "usb reset        - reset and restart USB device\n"
   "usb stats        - USB statistics";

typedef struct {
    const char *const name;
    uintptr_t         addr;
} memmap_t;

static const memmap_t memmap[] = {
    { "ADC1",   ADC1_BASE },
    { "ADC2",   ADC2_BASE },
    { "ADC3",   ADC3_BASE },
    { "AHB1",   PERIPH_BASE_AHB1 },
    { "AHB2",   PERIPH_BASE_AHB2 },
    { "APB1",   PERIPH_BASE_APB1 },
    { "APB2",   PERIPH_BASE_APB2 },
#ifdef STM32F1
    { "AFIO",   AFIO_BASE },
    { "BKP",    BACKUP_REGS_BASE },
#endif
#if defined(STM32F103xE) || defined(STM32F2)
    { "BKP",    RTC_BKP_BASE },
#endif
    { "DAC",    DAC_BASE },
    { "DMA1",   DMA1_BASE },
    { "DMA2",   DMA2_BASE },
    { "EXTI",   EXTI_BASE },
    { "FLASH",  FLASH_BASE },
    { "FPEC",   FLASH_MEM_INTERFACE_BASE },
    { "GPIOA",  GPIOA_BASE },
    { "GPIOB",  GPIOB_BASE },
    { "GPIOC",  GPIOC_BASE },
    { "GPIOD",  GPIOD_BASE },
    { "GPIOE",  GPIOE_BASE },
    { "IWDG",   IWDG_BASE },
    { "NVIC",   NVIC_BASE },
    { "PWR",    POWER_CONTROL_BASE },
    { "RCC",    RCC_BASE },
    { "RTC",    RTC_BASE },
    { "SCB",    SCB_BASE },
    { "SPI1",   SPI1_BASE  },
    { "SRAM",   SRAM_BASE },
    { "SYSCFG", SYSCFG_BASE },
    { "TIM1",   TIM1_BASE },
    { "TIM2",   TIM2_BASE },
    { "TIM3",   TIM3_BASE },
    { "TIM4",   TIM4_BASE },
    { "TIM5",   TIM5_BASE },
    { "TIM8",   TIM8_BASE },
    { "USART1", USART1_BASE },
    { "USART3", USART3_BASE },
    { "USB0",   USB0_BASE },
    { "USB1",   USB1_BASE },
    { "WWDG",   WWDG_BASE },
};

static uint
time_check(const char *text, uint diff, uint min, uint max)
{
    int errs = 0;
    if ((min <= diff) && (max >= diff)) {
        printf("PASS: ");
    } else {
        printf("FAIL: ");
        errs++;
    }
    printf("%-24s %u usec\n", text, diff);
    return (errs);
}

static rc_t
timer_test(void)
{
    uint64_t start;
    uint64_t diff;
    uint     errs = 0;
    uint     count;

    start = timer_tick_get();
    for (count = 1000; count > 0; count--)
        if (timer_tick_get() != start)
            break;
    if (count == 0) {
        printf("Timer tick is not advancing\n");
        return (RC_FAILURE);
    }

    start = timer_tick_get();
    timer_delay_ticks(0);
    diff = timer_tick_to_usec(timer_tick_get() - start);
    errs += time_check("timer_delay_ticks(0)", (uint)diff, 0, 5);

    start = timer_tick_get();
    timer_delay_ticks(100);
    diff = timer_tick_to_usec(timer_tick_get() - start);
    errs += time_check("timer_delay_ticks(100)", (uint)diff, 2, 5);

    start = timer_tick_get();
    timer_delay_usec(1);
    diff = timer_tick_to_usec(timer_tick_get() - start);
    errs += time_check("timer_delay_usec(1)", (uint)diff, 1, 5);

    start = timer_tick_get();
    timer_delay_usec(10);
    diff = timer_tick_to_usec(timer_tick_get() - start);
    errs += time_check("timer_delay_usec(10)", (uint)diff, 10, 15);

    start = timer_tick_get();
    timer_delay_usec(1000);
    diff = timer_tick_to_usec(timer_tick_get() - start);
    errs += time_check("timer_delay_usec(1000)", (uint)diff, 1000, 1005);

    start = timer_tick_get();
    timer_delay_msec(1);
    diff = timer_tick_to_usec(timer_tick_get() - start);
    errs += time_check("timer_delay_msec(1)", (uint)diff, 1000, 1005);

    start = timer_tick_get();
    timer_delay_msec(10);
    diff = timer_tick_to_usec(timer_tick_get() - start);
    errs += time_check("timer_delay_msec(10)", (uint)diff, 10000, 10007);

    start = timer_tick_get();
    timer_delay_msec(1000);
    diff = timer_tick_to_usec(timer_tick_get() - start);
    errs += time_check("timer_delay_msec(1000)", (uint)diff, 1000000, 1000007);

    // XXX: Replace one second above with RTC tick?

    if (errs > 0)
        return (RC_FAILURE);
    return (RC_SUCCESS);
}

static rc_t
timer_watch(void)
{
    bool_t   fail = FALSE;
    uint64_t last = timer_tick_get();
    uint64_t now;

    while (1) {
        now = timer_tick_get();
        if (last >= now) {
            printf("\nLast=%llx now=%llx Current=%012llx",
                   (long long) last, (long long) now, timer_tick_get());
        } else {
            if ((last >> 32) != (now >> 32))
                putchar('.');
            last = now;
        }
        if (input_break_pending()) {
            printf("^C\n");
            break;
        }
    }
    return ((fail == TRUE) ? RC_FAILURE : RC_SUCCESS);
}

static rc_t
cmd_time_set(int argc, char * const *argv)
{
    int arg;
    for (arg = 1; arg < argc; arg++) {
        uint year, mon, day;
        uint hour, min, sec;
        uint ch;

        if (sscanf(argv[arg], "%u%1[-/]%u%1[-/]%u",
                   &year, &ch, &mon, &ch, &day) == 5) {
            rtc_allow_writes(TRUE);
            rtc_set_date(year, mon, day, 0);
            rtc_allow_writes(FALSE);
        } else if (sscanf(argv[arg], "%u:%u:%u", &hour, &min, &sec) == 3) {
            rtc_allow_writes(TRUE);
            rtc_set_time(hour, min, sec, 0, 0);
            rtc_allow_writes(FALSE);
        } else {
            printf("Unknown time set argument %s\n", argv[arg]);
            return (RC_BAD_PARAM);
        }
    }
    rtc_print(1, 1);

    return (RC_SUCCESS);
}

rc_t
cmd_time(int argc, char * const *argv)
{
    rc_t rc;

    if (argc <= 1)
        return (RC_USER_HELP);

    if (strncmp(argv[1], "calibrate", 3) == 0) {
        rtc_calibrate();
        rc = RC_SUCCESS;
    } else if (strcmp(argv[1], "cmd") == 0) {
        uint64_t time_start;
        uint64_t time_diff;

        if (argc <= 2) {
            printf("error: time cmd requires command to execute\n");
            return (RC_USER_HELP);
        }
        time_start = timer_tick_get();
        rc = cmd_exec_argv(argc - 2, argv + 2);
        time_diff = timer_tick_get() - time_start;
        printf("%llu us\n", timer_tick_to_usec(time_diff));
        if (rc == RC_USER_HELP)
            rc = RC_FAILURE;
    } else if (strcmp(argv[1], "log") == 0) {
        amigartc_log();
        rc = RC_SUCCESS;
    } else if (strncmp(argv[1], "now", 1) == 0) {
        uint64_t now = timer_tick_get();
        printf("tick   0x%llx uptime=%llu usec\nRTC    ",
               now, timer_tick_to_usec(now));
        rtc_print(0, 1);
        printf("utime  ");
        rtc_print(1, 1);
        printf("RP5C01 ");
        amigartc_print();
        rc = RC_SUCCESS;
    } else if (strcmp(argv[1], "set") == 0) {
        rc = cmd_time_set(argc - 1, argv + 1);
    } else if (strncmp(argv[1], "show", 2) == 0) {
        timer_show();
        rc = RC_SUCCESS;
    } else if (strncmp(argv[1], "snoop", 2) == 0) {
        int debug = 0;
        int arg;
        for (arg = 2; arg < argc; arg++)
            if (*argv[2] == 'd')
                debug++;
        amigartc_snoop(debug);
        rc = RC_USR_ABORT;
    } else if (strncmp(argv[1], "test", 1) == 0) {
        rc = timer_test();
    } else if (strncmp(argv[1], "watch", 1) == 0) {
        rc = timer_watch();
    } else {
        printf("Unknown argument %s\n", argv[1]);
        return (RC_USER_HELP);
    }
    return (rc);
}

rc_t
cmd_spi(int argc, char * const *argv)
{
    enum {
        OP_NONE,
        OP_READ,
        OP_WRITE,
        OP_ERASE_CHIP,
        OP_ERASE_SECTOR,
        OP_IDENTIFY,
    } op_mode = OP_NONE;
    rc_t        rc;
    const char *arg = argv[0];
    char       *this_cmd = "spi";
    char       *temp_cmd;
    uint32_t    dev = 0;
    uint32_t    addr = 0;
    uint32_t    len = 0;

    if (strcmp(arg, "set") == 0)
        this_cmd = "set";
    temp_cmd = this_cmd;
    while (*arg != '\0') {
        if (*arg != *temp_cmd)
            break;
        arg++;
        temp_cmd++;
    }
    if (*arg == '\0') {
        argv++;
        argc--;
        if (argc < 1) {
            printf("error: spi command requires operation to perform\n");
            return (RC_USER_HELP);
        }
        arg = argv[0];
    }
    if (strncmp(arg, "erase", 2) == 0) {
        if (argc < 2) {
            printf("error: spi erase requires <dev> and either chip or "
                   "<addr> argument\n");
            return (RC_USER_HELP);
        }
        if (strcmp(argv[1], "chip") == 0) {
            op_mode = OP_ERASE_CHIP;
            argc--;
            argv++;
        } else {
            op_mode = OP_ERASE_SECTOR;
        }
    } else if (strcmp("id", arg) == 0) {
        op_mode = OP_IDENTIFY;
    } else if (strcmp("list", arg) == 0) {
        spi_list_chipsel();
        return (RC_SUCCESS);
    } else if (strcmp("read", arg) == 0) {
        op_mode = OP_READ;
    } else if (strcmp("write", arg) == 0) {
        op_mode = OP_WRITE;
    } else {
        printf("error: unknown spi operation %s\n", arg);
        return (RC_USER_HELP);
    }

    if (argc > 1) {
        rc = parse_value(argv[1], (uint8_t *) &dev, 4);
        if (rc != RC_SUCCESS)
            return (rc);
    }

    if (argc > 2) {
        rc = parse_value(argv[2], (uint8_t *) &addr, 4);
        if (rc != RC_SUCCESS)
            return (rc);
    }

    if (argc > 3) {
        rc = parse_value(argv[3], (uint8_t *) &len, 4);
        if (rc != RC_SUCCESS)
            return (rc);
    }

    switch (op_mode) {
        case OP_IDENTIFY:
            rc = spi_flash_id(dev);
            break;
        case OP_READ:
            if (argc != 4) {
                printf("error: spi %s requires <dev> <addr> <len>\n", arg);
                return (RC_USER_HELP);
            }
            rc = spi_flash_read_binary(dev, addr, len);
            break;
        case OP_WRITE:
            if (argc != 4) {
                printf("error: spi %s requires <dev> <addr> <len>\n", arg);
                return (RC_USER_HELP);
            }
            rc = spi_flash_write_binary(dev, addr, len);
            break;
        case OP_ERASE_CHIP:
            printf("Chip erase\n");
            if (argc != 2) {
                printf("error: spi erase chip requires <dev>\n");
                return (RC_USER_HELP);
            }
            rc = spi_flash_erase(dev, 0, 0);
            break;
        case OP_ERASE_SECTOR:
            printf("Sector erase %lx", addr);
            if (len > 0)
                printf(" len %lx", len);
            printf("\n");
            if ((argc < 3) || (argc > 4)) {
                printf("error: spi erase sector requires <dev> <addr> and "
                       "allows optional <len>\n");
                return (RC_USER_HELP);
            }
            rc = spi_flash_erase(dev, addr, len);
            break;
        default:
            printf("BUG: op_mode\n");
            return (RC_FAILURE);
    }

    if (rc != 0)
        printf("FAILURE %d\n", rc);
    return (rc);
}

rc_t
cmd_map(int argc, char * const *argv)
{
    uint third = (ARRAY_SIZE(memmap) + 2) / 3;
    uint ent;
    for (ent = 0; ent < third; ent++) {
        printf("    %-6s %08x", memmap[ent].name, memmap[ent].addr);
        if (ent + third < ARRAY_SIZE(memmap))
            printf("    %-6s %08x",
                   memmap[ent + third].name, memmap[ent + third].addr);
        if (ent + third * 2 < ARRAY_SIZE(memmap))
            printf("    %-6s %08x",
                   memmap[ent + third * 2].name, memmap[ent + third * 2].addr);
        printf("\n");
    }
    return (RC_SUCCESS);
}

static void
shutdown_all(void)
{
    uart_flush();
    usb_shutdown(1);
    timer_delay_msec(30);
//  adc_shutdown();
    timer_shutdown();
    /* Put all peripherals in reset */
    /* Clear NVIC? */
}

rc_t
cmd_reset(int argc, char * const *argv)
{
    if (argc < 2) {
        printf("Resetting...\n");
        shutdown_all();
        reset_cpu();
        return (RC_FAILURE);
    } else if (strcmp(argv[1], "amiga") == 0) {
        uint hold = 0;
        uint longreset = 0;
        while (argc > 2) {
            if (strcmp(argv[2], "hold") == 0)
                hold = 1;
            else if (strcmp(argv[2], "long") == 0)
                longreset = 1;
            else
                printf("Invalid reset amiga \"%s\"\n", argv[2]);
            argc--;
            argv++;
        }
        kbrst_amiga(hold, longreset);
        if (hold)
            printf("Holding Amiga in reset\n");
        else
            printf("Resetting Amiga\n");
    } else if (strncmp(argv[1], "dfu", 3) == 0) {
        uint isrom = (argv[1][3] == 'r');
        printf("Resetting to DFU%s...\n", isrom ? " in ROM" : "");
        shutdown_all();
        reset_dfu(isrom);
    } else if (strcmp(argv[1], "reason") == 0) {
        show_reset_reason();
    } else if (strcmp(argv[1], "usb") == 0) {
        timer_delay_msec(1);
        usb_shutdown(0);
//      usb_signal_reset_to_host(1);
        usb_init();
    } else {
        printf("Unknown argument %s\n", argv[1]);
        return (RC_USER_HELP);
    }
    return (RC_SUCCESS);
}

rc_t
cmd_amiga(int argc, char * const *argv)
{
    if (argc < 2)
        return (RC_USER_HELP);
    if (strncmp(argv[1], "keyboard", 1) == 0) {
        keyboard_term();
    } else if (strncmp(argv[1], "status", 1) == 0) {
        power_show();
        if (power_state != POWER_STATE_OFF) {
            if (amiga_in_reset)
                printf("Reset state     Amiga is in reset\n");
        }
        printf("USB Powered     %s\n", usb_is_powered ? "Yes" : "No");
        printf("USB Keyboards   %u\n", usb_keyboard_count);
        printf("USB Mice        %u\n", usb_mouse_count);
        printf("USB Joysticks   %u\n", usb_joystick_count);
        printf("HID Enabled     %s\n", hiden_is_set ? "Yes" : "No");
        printf("Amiga Keyboard  %s sync, %s wake\n",
               amiga_keyboard_lost_sync ? "Lost" :
               amiga_keyboard_has_sync ? "Has" : "No",
               amiga_keyboard_sent_wake ? "Sent" : "Did not send");
    } else {
        printf("Unknown argument %s\n", argv[1]);
        return (RC_USER_HELP);
    }
    return (RC_SUCCESS);
}

rc_t
cmd_cpu(int argc, char * const *argv)
{
    if (argc < 2)
        return (RC_USER_HELP);
    if (strncmp(argv[1], "regs", 1) == 0) {
        fault_show_regs(NULL);
    } else if (strncmp(argv[1], "hardfault", 2) == 0) {
        fault_hard();
    } else {
        printf("Unknown argument %s\n", argv[1]);
        return (RC_USER_HELP);
    }
    return (RC_SUCCESS);
}

rc_t
cmd_fan(int argc, char * const *argv)
{
    int narg = 0;
    uint fan_speed = -1;
    if (argc < 2)
        return (RC_USER_HELP);
    if (strcmp(argv[1], "auto") == 0) {
        fan_speed = BIT(7) | 100;
    } else if (strcmp(argv[1], "off") == 0) {
        fan_speed = 0;
        narg = 2;
    } else if (strcmp(argv[1], "on") == 0) {
        fan_speed = 100;
        narg = 2;
    } else if (strcmp(argv[1], "speed") == 0) {
        int pos = 0;
        int speed;
        if (argc < 3) {
            printf("fan percent value required (0 - 100)\n");
            return (RC_USER_HELP);
        }
        if ((sscanf(argv[2], "%d%n", &speed, &pos) != 1) ||
            (argv[2][pos] != '\0') || (speed < 0) || (speed > 100)) {
            printf("Invalid fan percent value %s\n", argv[2]);
            return (RC_USER_HELP);
        }
        fan_speed = speed;
        narg = 3;
    } else {
        printf("Unknown argument %s\n", argv[1]);
        return (RC_USER_HELP);
    }
    fan_set(fan_speed);
    if (narg < argc) {
        if (strcasecmp(argv[narg], "save") == 0) {
            config.fan_speed = fan_speed;
            config_updated();
        }
    }

    return (RC_SUCCESS);
}

rc_t
cmd_power(int argc, char * const *argv)
{
    const char *cmd;
    cmd = skip(argv[0], "power");
    if (*cmd == '\0') {
        if (argc < 2)
            return (RC_USER_HELP);
        argc--;
        argv++;
        cmd = argv[0];
    }
    if (strcmp(cmd, "cycle") == 0) {
        power_set(POWER_STATE_CYCLE);
    } else if (strcmp(cmd, "on") == 0) {
        power_set(POWER_STATE_ON);
    } else if (strcmp(cmd, "off") == 0) {
        power_set(POWER_STATE_OFF);
    } else if (strncmp(cmd, "show", 1) == 0) {
        power_show();
        sensor_show();
    } else {
        printf("Unknown argument %s\n", cmd);
        return (RC_USER_HELP);
    }
    return (RC_SUCCESS);
}

rc_t
cmd_usb(int argc, char * const *argv)
{
    if (strcmp(argv[0], "usb") == 0) {
        argc--;
        argv++;
    }
    if (argc == 0)
        return (RC_USER_HELP);
    if (strncmp(argv[0], "debug", 2) == 0) {
        uint debug_mask;
        int  pos = 0;
        if (argc < 2) {
            printf("usb %s requires an argument\n", argv[0]);
            return (RC_USER_HELP);
        }
        if ((sscanf(argv[1], "%x%n", &debug_mask, &pos) != 1) ||
            (argv[1][pos] != '\0')) {
            printf("Invalid argument to debug: %s\n", argv[1]);
            return (RC_USER_HELP);
        }
        usb_debug_mask = debug_mask;
    } else if (strncmp(argv[0], "disable", 3) == 0) {
        timer_delay_msec(1);
        usb_shutdown(0);
//      usb_signal_reset_to_host(0);
        return (RC_SUCCESS);
    } else if ((strcmp(argv[0], "kbd") == 0) ||
               (strncmp(argv[0], "keyboard", 2) == 0)) {
        if (argc < 2) {
            printf("USB keyboard input: %s\n",
                   usb_keyboard_terminal ? "on" : "off");
        } else if (strcmp(argv[1], "on") == 0) {
            usb_keyboard_terminal = 1;
        } else if (strcmp(argv[1], "off") == 0) {
            usb_keyboard_terminal = 0;
        } else {
            printf("Invalid argument %s\n", argv[1]);
            return (RC_BAD_PARAM);
        }
    } else if (strncmp(argv[0], "ls", 2) == 0) {
        uint verbose = 0;
        while (argc >= 2) {
            if (strncmp(argv[1], "verbose", 1) == 0)
                verbose++;
            if (strncmp(argv[1], "config", 1) == 0)
                verbose |= 0x100;
            argc--;
            argv++;
        }
        usb_ls(verbose);
    } else if (strcmp(argv[0], "off") == 0) {
        usb_set_power(0);
    } else if (strcmp(argv[0], "on") == 0) {
        usb_set_power(1);
    } else if (strncmp(argv[0], "regs", 3) == 0) {
        usb_show_regs();
    } else if (strcmp(argv[0], "reset") == 0) {
        timer_delay_msec(1);
        usb_shutdown(0);
//      usb_signal_reset_to_host(1);
        usb_init();
        return (RC_SUCCESS);
    } else if (strncmp(argv[0], "stat", 2) == 0) {
        usb_show_stats();
    } else {
        printf("Unknown argument %s\n", argv[0]);
        return (RC_USER_HELP);
    }
    return (RC_SUCCESS);
}

rc_t
cmd_gpio(int argc, char * const *argv)
{
    int arg;
    if (argc < 2) {
        gpio_show(-1, 0xffff);
        return (RC_SUCCESS);
    }

    for (arg = 1; arg < argc; arg++) {
        const char *ptr = argv[arg];
        int port = -1;
        uint16_t pins[NUM_GPIO_BANKS];
        const char *assign = NULL;

        memset(pins, 0, sizeof (pins));

        if (gpio_name_match(&ptr, pins) != 0) {
            if ((*ptr == 'p') || (*ptr == 'P'))
                ptr++;

            /* Find port, if specified */
            if ((*ptr >= 'a') && (*ptr <= 'f'))
                port = *(ptr++) - 'a';
            else if ((*ptr >= 'A') && (*ptr <= 'F'))
                port = *(ptr++) - 'A';

            /* Find pin, if specified */
            if ((*ptr >= '0') && (*ptr <= '9')) {
                uint pin = *(ptr++) - '0';
                if ((*ptr >= '0') && (*ptr <= '9'))
                    pin = pin * 10 + *(ptr++) - '0';
                if (pin > 15) {
                    printf("Invalid argument %s\n", argv[arg]);
                    return (RC_BAD_PARAM);
                }
                pins[port] = BIT(pin);
            } else if (*ptr == '*') {
                ptr++;
                pins[port] = 0xffff;
            }
        }

        if (*ptr == '=') {
            assign = ptr + 1;
            ptr = "";
            if (port == -1) {
                uint tport;
                for (tport = 0; tport < NUM_GPIO_BANKS; tport++)
                    if (pins[tport] != 0)
                        break;
                if (tport == NUM_GPIO_BANKS) {
                    printf("You must specify the GPIO to assign: %s\n",
                           argv[arg]);
                    return (RC_BAD_PARAM);
                }
            }
        }
        if (*ptr != '\0') {
            printf("Invalid argument %s\n", argv[arg]);
            return (RC_BAD_PARAM);
        }

        if (assign != NULL) {
            if (port == -1) {
                for (port = 0; port < NUM_GPIO_BANKS; port++) {
                    if (pins[port] != 0) {
                        gpio_assign(port, pins[port], assign);
                    }
                }
            } else {
                gpio_assign(port, pins[port], assign);
            }
        } else {
            if (port == -1) {
                for (port = 0; port < NUM_GPIO_BANKS; port++)
                    gpio_show(port, pins[port]);
            } else {
                gpio_show(port, pins[port]);
            }
        }
    }

    return (RC_SUCCESS);
}

static const char *const debug_flag_bits[] = {
    "RTC", "AmigaKeyboard", "AmigaMouse", "AmigaJoystick",
        "USB", "USBConn", "USBKeyboard", "USBMouse",
    "USBReport", "DecodeMisc", "DecodeMouse", "DecodeJoystick",
        "DecodeKeyboard", "USBControl", "", "",
    "I2C", "I2CLL", "HIDEN", "BECMsg",
        "Fan", "SPI", "", "",
    "", "", "", "",
        "", "", "", "",
};

static const char *const config_flag_bits[] = {
    "InvertX", "InvertY", "InvertW", "InvertP",
        "SwapXY", "SwapWP", "KeyupWP", "GamepadMouse",
    "HaveFan", "KeyboardNoSync", "KeyboardSwapAlt", "",
        "", "", "", "",
    "", "", "", "",
        "", "", "", "",
    "", "", "", "",
        "", "", "", "",
};

static void
decode_bits(const char *const *bits, uint32_t flags)
{
    uint bit;
    uint printed = 0;

    for (bit = 0; bit < 32; bit++) {
        if (flags & BIT(bit)) {
            if (printed++)
                printf(", ");
            if (bits[bit][0] == '\0') {
                printf("bit%u", bit);
            } else {
                printf("%s", bits[bit]);
            }
        }
    }
}

static uint
match_bits(const char *const *bits, const char *name)
{
    uint bit;
    for (bit = 0; bit < 32; bit++) {
        if (strcasecmp(name, bits[bit]) == 0)
            return (bit);
    }
    return (bit);
}

static void
merge_args(char *buf, uint buflen, uint argc, char * const *argv)
{
    uint arg;
    uint len;

    buflen--;  // Save space for ending '\0'
    for (arg = 0; arg < argc; arg++) {
        if (buflen == 0)
            break;
        if (arg != 0) {
            *(buf++) = ' ';
            buflen--;
            if (buflen == 0)
                break;
        }
        len = strlen(argv[arg]);
        if (len > buflen)
            len = buflen;
        memcpy(buf, argv[arg], len);
        buf    += len;
        buflen -= len;
        if (buflen == 0)
            break;
    }
    *buf = '\0';
}

#define CFOFF(x) offsetof(config_t, x), sizeof (config.x)

#define MODE_DEC          0       // Show value in decimal
#define MODE_HEX          BIT(0)  // Show value in hexadecimal
#define MODE_STRING       BIT(1)  // Show string
#define MODE_BIT_FLAGS    BIT(2)  // Decode debug flags
#define MODE_FAN_AUTO     BIT(3)  // Interpret BIT(7) as "auto"; 0 = decimal
#define MODE_SIGNED       BIT(4)  // Decimal value is signed
typedef struct {
    const char *cs_name;
    const char *cs_desc;
    uint16_t    cs_offset;  // Offset into config structure
    uint8_t     cs_size;    // Size of config value in bytes
    uint8_t     cs_mode;    // Mode bits for display (1=hex)
} config_set_t;
static const config_set_t config_set[] = {
    { "board_rev",      "Board revision",
      CFOFF(board_rev), MODE_DEC },
    { "board_type",     "Board type (1=AmigaPCI, 2=APCIDev, 3=KeyJam)",
      CFOFF(board_type), MODE_DEC },
    { "cpu_temp_bias",  "Bias (+/-) for CPU temperature",
      CFOFF(cpu_temp_bias), MODE_SIGNED },
    { "debug",         "",
      CFOFF(debug_flag), MODE_HEX | MODE_BIT_FLAGS },
    { "fan_rpm_max",  "Fan maximum speed in RPM",
      CFOFF(fan_rpm_max), MODE_DEC },
    { "fan_speed",     "Fan speed",
      CFOFF(fan_speed), MODE_FAN_AUTO },
    { "fan_speed_min", "Fan speed minimum percent",
      CFOFF(fan_speed_min), MODE_DEC },
    { "fan_temp_max",  "CPU temp for max fan speed",
      CFOFF(fan_temp_max), MODE_DEC },
    { "fan_temp_min",  "CPU temp for min fan speed",
      CFOFF(fan_temp_min), MODE_DEC },
    { "flags",          "",
      CFOFF(flags), MODE_HEX | MODE_BIT_FLAGS },
    { "i2c_max_speed",  "I2C maximum speed (Hz)",
      CFOFF(i2c_max_speed), MODE_DEC },
    { "i2c_min_speed",  "I2C minimum speed (Hz)",
      CFOFF(i2c_min_speed), MODE_DEC },
    { "mouse_div_x",    "Mouse X speed divisor",
      CFOFF(mouse_div_x), MODE_DEC },
    { "mouse_div_y",    "Mouse Y speed divisor",
      CFOFF(mouse_div_y), MODE_DEC },
    { "mouse_mul_x",    "Mouse X speed multiplier",
      CFOFF(mouse_mul_x), MODE_DEC },
    { "mouse_mul_y",    "Mouse Y speed multiplier",
      CFOFF(mouse_mul_y), MODE_DEC },
    { "name",          "Board name",
      CFOFF(name), MODE_STRING },
    { "pson",          "Power on at AC restored",
      CFOFF(ps_on_mode), MODE_DEC },
};

rc_t
cmd_set(int argc, char * const *argv)
{
    if (argc <= 1) {
        uint pos;
        for (pos = 0; pos < ARRAY_SIZE(config_set); pos++) {
            char buf[32];
            const config_set_t *c = &config_set[pos];
            uint cs_mode = c->cs_mode;
            uint value = 0;
            void *src = (void *) ((uintptr_t) &config + c->cs_offset);
            if (cs_mode & MODE_STRING) {
                /* String */
                sprintf(buf, "%s \"%.*s\"",
                        c->cs_name, c->cs_size, (char *)src);
            } else if (cs_mode & MODE_HEX) {
                /* Hexadecimal */
                memcpy(&value, src, c->cs_size);
                sprintf(buf, "%s %0*x", c->cs_name, c->cs_size * 2, value);
            } else {
                /* Decimal : MODE_DEC */
                memcpy(&value, src, c->cs_size);
                if ((cs_mode & MODE_FAN_AUTO) && (value & BIT(7))) {
                    sprintf(buf, "%s %s", c->cs_name, "auto");
                } else if (cs_mode & MODE_SIGNED) {
                    if (c->cs_size == 1)
                        value = (int8_t) value;
                    else if (c->cs_size == 2)
                        value = (int16_t) value;
                    sprintf(buf, "%s %d", c->cs_name, value);
                } else {
                    sprintf(buf, "%s %u", c->cs_name, value);
                }
            }
            printf("%s%*s%s", buf, 24 - strlen(buf), "", c->cs_desc);
            if (cs_mode & MODE_BIT_FLAGS) {
                if (strncmp(c->cs_name, "debug", 5) == 0) {
                    /* Decode debug flags */
                    if (value == 0)
                        printf("Debug flags");
                    decode_bits(debug_flag_bits, value);
                } else if (strncmp(c->cs_name, "flags", 4) == 0) {
                    /* Decode config flags */
                    if (value == 0)
                        printf("Config flags");
                    decode_bits(config_flag_bits, value);
                }
            }
            printf("\n");
        }
        return (RC_SUCCESS);
    }
    if ((strcmp(argv[1], "help") == 0) ||
               (strcmp(argv[1], "?") == 0)) {
        return (RC_USER_HELP);
    } else if (strcmp(argv[1], "cpu_temp_bias") == 0) {
        int pos = 0;
        int bias;
        if (argc != 3) {
            printf("CPU temp bias value required (-100 - 100)\n");
            return (RC_BAD_PARAM);
        }
        if ((sscanf(argv[2], "%d%n", &bias, &pos) != 1) ||
                   (argv[2][pos] != '\0') || (bias < -100) || (bias > 100)) {
            printf("Invalid CPU tempo bias value %s\n", argv[2]);
            return (RC_USER_HELP);
        }
        config.cpu_temp_bias = bias;
        config_updated();
    } else if (strcmp(argv[1], "debug") == 0) {
        int      arg;
        int      pos = 0;
        int      add_sub = 0;
        uint     bit;
        uint     do_save = 0;
        uint     did_set = 0;
        uint32_t value;
        uint32_t nvalue = 0;
        if (argc <= 2) {
            printf("Debug flags are a combination of bits: "
                   "specify all bit numbers or names\n");
            for (bit = 0; bit < 32; bit++)
                if (debug_flag_bits[bit][0] != '\0')
                    printf("  %c %2x  %s\n",
                           config.debug_flag & BIT(bit) ? '*' : ' ',
                           bit, debug_flag_bits[bit]);
            printf("Current debug %08lx  ", config.debug_flag);
            decode_bits(debug_flag_bits, config.debug_flag);
            printf("\n");
            return (RC_SUCCESS);
        }
        for (arg = 2; arg < argc; arg++) {
            char *flagname = argv[arg];
            if (*flagname == '+') {
                flagname++;
                add_sub = 1;
            } else if (*flagname == '-') {
                flagname++;
                add_sub = -1;
            }
            if (*flagname == '\0')
                continue;
            if (strcasecmp(flagname, "save") == 0) {
                do_save = 1;
                continue;
            } else if ((bit = match_bits(debug_flag_bits, flagname)) < 32) {
                if (did_set == 0)
                    nvalue = 0;
                nvalue |= BIT(bit);
                did_set = 1;
            } else {
                if ((sscanf(flagname, "%x%n", &value, &pos) != 1) ||
                    (flagname[pos] != '\0')) {
                    printf("Invalid argument: %s\n", flagname);
                    return (RC_USER_HELP);
                }
                if ((pos >= 4) || (value >= 32))
                    nvalue = value;
                else
                    nvalue |= BIT(value);
                did_set = 1;
            }
        }
        if (add_sub > 0)
            nvalue = config.debug_flag | nvalue;
        else if (add_sub < 0)
            nvalue = config.debug_flag & ~nvalue;

        if (config.debug_flag != nvalue) {
            config.debug_flag = nvalue;
            printf("Debug flags %08lx ", nvalue);
            decode_bits(debug_flag_bits, nvalue);
            printf("\n");
        }
        if (do_save)
            config_updated();
    } else if (strncmp(argv[1], "defaults", 7) == 0) {
        if (argc > 2) {
            if (strncmp(argv[2], "keymap", 3) == 0) {
                printf("Resetting keymap\n");
                keyboard_set_defaults();
                config_updated();
            } else if (strncmp(argv[2], "mouse", 2) == 0) {
                mouse_set_defaults();
                config_updated();
            } else {
                printf("Unknown argument %s\n", argv[2]);
                return (RC_USER_HELP);
            }
        } else {
            /* Reset all defaults */
            uint board_type = config.board_type;
            uint board_rev = config.board_rev;
            config_set_defaults();
            config.board_type = board_type;
            config.board_rev = board_rev;
        }
    } else if (strcmp(argv[1], "fan_rpm_max") == 0) {
        int pos = 0;
        int rpm;
        if (argc != 3) {
            printf("fan RPM value required (0 - 32000)\n");
            return (RC_BAD_PARAM);
        }
        if ((sscanf(argv[2], "%d%n", &rpm, &pos) != 1) ||
                   (argv[2][pos] != '\0') || (rpm < 0) || (rpm > 32000)) {
            printf("Invalid fan RPM value %s\n", argv[2]);
            return (RC_USER_HELP);
        }
        config.fan_rpm_max = rpm;
        config_updated();
    } else if (strcmp(argv[1], "fan_speed") == 0) {
        int pos = 0;
        int speed;
        if (argc != 3) {
            printf("fan percent value required (0 - 100) or auto\n");
            return (RC_BAD_PARAM);
        }
        if (strcmp(argv[2], "auto") == 0) {
            speed = BIT(7) | 100;
        } else if ((sscanf(argv[2], "%d%n", &speed, &pos) != 1) ||
                   (argv[2][pos] != '\0') || (speed < 0) || (speed > 100)) {
            printf("Invalid fan percent value %s\n", argv[2]);
            return (RC_USER_HELP);
        }
        fan_set(speed);
        config.fan_speed = speed;
        config_updated();
    } else if (strcmp(argv[1], "fan_speed_min") == 0) {
        int pos = 0;
        int speed;
        if (argc != 3) {
            printf("fan percent value required (0 - 100)\n");
            return (RC_BAD_PARAM);
        }
        if ((sscanf(argv[2], "%d%n", &speed, &pos) != 1) ||
            (argv[2][pos] != '\0') || (speed < 0) || (speed > 100)) {
            printf("Invalid fan percent value %s\n", argv[2]);
            return (RC_USER_HELP);
        }
        config.fan_speed_min = speed;
        config_updated();
    } else if ((strcmp(argv[1], "fan_temp_min") == 0) ||
               (strcmp(argv[1], "fan_temp_max") == 0)) {
        int pos = 0;
        int value;
        if (argc != 3) {
            printf("Integer temperature percent required (0 - 127)\n");
            return (RC_BAD_PARAM);
        }
        if ((sscanf(argv[2], "%d%n", &value, &pos) != 1) ||
            (argv[2][pos] != '\0') || (value < 0) || (value > 127)) {
            printf("Invalid temperature value %s\n", argv[2]);
            return (RC_USER_HELP);
        }
        if (strcmp(argv[1], "fan_temp_min") == 0)
            config.fan_temp_min = value;
        else
            config.fan_temp_max = value;
        config_updated();
    } else if (strncmp(argv[1], "flags", 4) == 0) {
        int      arg;
        int      pos = 0;
        int      add_sub = 0;
        uint     bit;
        uint     do_save = 0;
        uint     did_set = 0;
        uint32_t value;
        uint32_t nvalue = 0;
        if (argc <= 2) {
            printf("Config flags are a combination of bits: "
                   "specify all bit numbers or names\n");
            for (bit = 0; bit < 32; bit++)
                if (config_flag_bits[bit][0] != '\0')
                    printf(" %c %2x  %s\n",
                           config.flags & BIT(bit) ? '*' : ' ',
                           bit, config_flag_bits[bit]);
            printf("Current config %08lx  ", config.flags);
            decode_bits(config_flag_bits, config.flags);
            printf("\n");
            return (RC_SUCCESS);
        }
        for (arg = 2; arg < argc; arg++) {
            char *flagname = argv[arg];
            if (*flagname == '+') {
                flagname++;
                add_sub = 1;
            } else if (*flagname == '-') {
                flagname++;
                add_sub = -1;
            }
            if (*flagname == '\0')
                continue;
            if (strcasecmp(flagname, "save") == 0) {
                do_save = 1;
                continue;
            } else if ((bit = match_bits(config_flag_bits, flagname)) < 32) {
                if (did_set == 0)
                    nvalue = 0;
                nvalue |= BIT(bit);
                did_set = 1;
            } else {
                if ((sscanf(flagname, "%x%n", &value, &pos) != 1) ||
                    (flagname[pos] != '\0')) {
                    printf("Invalid argument: %s\n", flagname);
                    return (RC_USER_HELP);
                }
                if ((pos >= 4) || (value >= 32))
                    nvalue = value;
                else
                    nvalue |= BIT(value);
                did_set = 1;
            }
        }
        if (add_sub > 0)
            nvalue = config.flags | nvalue;
        else if (add_sub < 0)
            nvalue = config.flags & ~nvalue;

        if (config.flags != nvalue) {
            config.flags = nvalue;
            printf("Config flags %08lx ", nvalue);
            decode_bits(config_flag_bits, nvalue);
            printf("\n");
        }
        if (do_save)
            config_updated();
    } else if (strcmp(argv[1], "name") == 0) {
        if (argc <= 2) {
            config_name(NULL);
        } else {
            char name[32];
            merge_args(name, sizeof (name), argc - 2, argv + 2);
            config_name(name);
        }
        return (RC_SUCCESS);
    } else if (strcmp(argv[1], "pson") == 0) {
        if (argc <= 2)
            return (RC_USER_HELP);
        if (strcmp(argv[2], "on") == 0) {
            config.ps_on_mode |= 1;
        } else if (strcmp(argv[2], "off") == 0) {
            config.ps_on_mode &= 1;
        } else {
            uint mode;
            int  pos = 0;
            if ((sscanf(argv[2], "%x%n", &mode, &pos) != 1) ||
                (argv[2][pos] != '\0') || (mode > 0xff)) {
                printf("Invalid argument: %s\n", argv[2]);
                return (RC_USER_HELP);
            }
            config.ps_on_mode = mode;
            config_updated();
        }
    } else if (strcmp(argv[1], "time") == 0) {
        return (cmd_time_set(argc - 1, argv + 1));
    } else {
        uint which;
        uint n = 1;
        uint do_save = 0;
        if (strcmp(argv[n], "save") == 0) {
            do_save++;
            n++;
        }
        for (which = 0; which < ARRAY_SIZE(config_set); which++) {
            const char *cs_name = config_set[which].cs_name;
            if (strcmp(cs_name, argv[n]) == 0) {
                uint16_t    cs_offset = config_set[which].cs_offset;
                uint8_t     cs_size   = config_set[which].cs_size;
                uint8_t     cs_mode   = config_set[which].cs_mode;
                const char *mode_str = "%d%n";
                int         value;
                int         pos = 0;
                if (strcmp(argv[n + 2], "save") == 0) {
                    do_save++;
                    argc--;
                }
                if (argc - n != 2) {
                    printf("%s value required\n", cs_name);
                    return (RC_BAD_PARAM);
                }
                if (cs_mode & MODE_HEX)
                    mode_str = "%x%d";
                if ((sscanf(argv[n + 1], mode_str, &value, &pos) != 1) ||
                    (argv[n + 1][pos] != '\0') ||
                    (((cs_mode & MODE_SIGNED) == 0) && (value < 0))) {
invalid_value:
                    printf("Invalid value %s for %s\n", argv[n + 1], cs_name);
                    return (RC_USER_HELP);
                }
                uint8_t *ptr = ((uint8_t *)&config) + cs_offset;
                switch (cs_size) {
                    default:
                    case 1:
                        if (cs_mode & MODE_SIGNED) {
                            if ((value > 127) || (value < -128))
                                goto invalid_value;
                        } else {
                            if (value >= 0x100)
                                goto invalid_value;
                        }
                        *ptr = value;
                        break;
                    case 2:
                        if (cs_mode & MODE_SIGNED) {
                            if ((value > 32767) || (value < -32768))
                                goto invalid_value;
                        } else {
                            if (value >= 0x10000)
                                goto invalid_value;
                        }
                        *(uint16_t *)ptr = value;
                        break;
                    case 4:
                        *(uint32_t *)ptr = value;
                        break;
                }
                if (do_save)
                    config_updated();
                return (RC_SUCCESS);
            }
        }
        printf("set \"%s\" unknown argument\n", argv[1]);
        return (RC_USER_HELP);
    }

    return (RC_SUCCESS);
}


rc_t
cmd_snoop(int argc, char * const *argv)
{
    printf("Snoop not implemented\n");
    return (RC_USER_HELP);
#if 0
    uint mode = CAPTURE_SW;
    if (argc > 1) {
        if (strcmp(argv[1], "addr") == 0) {
            mode = CAPTURE_ADDR;
        } else if (strncmp(argv[1], "low", 2) == 0) {
            mode = CAPTURE_DATA_LO;
        } else if (strncmp(argv[1], "high", 2) == 0) {
            mode = CAPTURE_DATA_HI;
        } else {
            printf("snoop \"%s\" unknown argument\n", argv[1]);
            return (RC_USER_HELP);
        }
    }
    bus_snoop(mode);

    return (RC_SUCCESS);
#endif
}
