/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Amiga and USB HID joystick / gamepad handling.
 */

#include <stdint.h>
#include "main.h"
#include "config.h"
#include "joystick.h"
#include "keyboard.h"
#include "mouse.h"
#include "gpio.h"
#include "hiden.h"
#include "printf.h"
#include "utils.h"
#include "amiga_kbd_codes.h"

uint8_t joystick_asserted;

#define BUTTON_CODE_UP    (0x1c | KEYCAP_BUTTON)
#define BUTTON_CODE_DOWN  (0x1d | KEYCAP_BUTTON)
#define BUTTON_CODE_LEFT  (0x1e | KEYCAP_BUTTON)
#define BUTTON_CODE_RIGHT (0x1f | KEYCAP_BUTTON)

void
joystick_action(uint which, uint up, uint down, uint left, uint right,
                uint32_t buttons)
{
    static uint8_t  last_up;
    static uint8_t  last_down;
    static uint8_t  last_left;
    static uint8_t  last_right;
    static uint32_t last_buttons;
    uint32_t        macro;
    uint            change = 0;

    joystick_asserted = up | down | left | right || !!buttons;

    if (last_up != up) {
        capture_scancode(BUTTON_CODE_UP | (up ? KEYCAP_DOWN : KEYCAP_UP));
        if (which == 0)
            mouse_put_macro(ASE_JOYSTICK0_UP, up, last_up);
        else
            mouse_put_macro(ASE_JOYSTICK1_UP, up, last_up);
        last_up = up;
        change = 1;
        dprintf(DF_AMIGA_JOYSTICK, "%sJ%c", up ? "" : "-", 'U');
    }
    if (last_down != down) {
        capture_scancode(BUTTON_CODE_DOWN | (down ? KEYCAP_DOWN : KEYCAP_UP));
        if (which == 0)
            mouse_put_macro(ASE_JOYSTICK0_DOWN, down, last_down);
        else
            mouse_put_macro(ASE_JOYSTICK1_DOWN, down, last_down);
        last_down = down;
        change = 1;
        dprintf(DF_AMIGA_JOYSTICK, "%sJ%c", down ? "" : "-", 'D');
    }
    if (last_left != left) {
        capture_scancode(BUTTON_CODE_LEFT | (left ? KEYCAP_DOWN : KEYCAP_UP));
        if (which == 0)
            mouse_put_macro(ASE_JOYSTICK0_LEFT, left, last_left);
        else
            mouse_put_macro(ASE_JOYSTICK1_LEFT, left, last_left);
        last_left = left;
        change = 1;
        dprintf(DF_AMIGA_JOYSTICK, "%sJ%c", left ? "" : "-", 'L');
    }
    if (last_right != right) {
        capture_scancode(BUTTON_CODE_RIGHT | (right ? KEYCAP_DOWN : KEYCAP_UP));
        if (which == 0)
            mouse_put_macro(ASE_JOYSTICK0_RIGHT, right, last_right);
        else
            mouse_put_macro(ASE_JOYSTICK1_RIGHT, right, last_right);
        last_right = right;
        change = 1;
        dprintf(DF_AMIGA_JOYSTICK, "%sJ%c", right ? "" : "-", 'R');
    }

    buttons |= mouse_buttons_add;

    if (buttons != last_buttons) {
        uint bit;
        for (bit = 0; bit < 32; bit++) {
            uint     is_pressed  = (buttons & BIT(bit)) ? 1 : 0;
            uint     was_pressed = (last_buttons & BIT(bit)) ? 1 : 0;
            macro = config.buttonmap[bit + 32];
            if (macro == 0)
                macro = 0x80 + bit;  // Not reassigned: default to self
            else if (macro <= 4)
                macro--;
            if (is_pressed != was_pressed) {
                if (config.debug_flag & DF_AMIGA_JOYSTICK) {
                    uint bnum = bit;
                    if (!is_pressed)
                        putchar('-');
                    putchar('B');
                    if (bnum >= 10) {
                        putchar('0' + bnum / 10);
                        bnum %= 10;
                    }
                    putchar('0' + bnum);
                }
                capture_scancode((bit + 0x20) | KEYCAP_BUTTON |
                                 (is_pressed ? KEYCAP_DOWN : KEYCAP_UP));
                mouse_put_macro(macro, is_pressed, was_pressed);
            }
        }
        last_buttons = buttons;
        change = 1;
    }
    if (change)
        hiden_set(1);
}
