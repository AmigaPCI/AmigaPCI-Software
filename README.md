# AmigaPCI/AmigaPCI-Software
AmigaPCI BEC (Board Environment Controller) STM32 firmware, Amiga utilities,
and host PC software.

## Firmware
STM32F205 firmware for the AmigaPCI project

The AmigaPCI embedded STM32 MCU is responsible for Amiga keyboard,
mouse, RTC, power supply, single fan, clock, and board reset control.

This firmware implements a command line which is unavailable to
the user unless they connect a TTL serial device (such as FT232R).
