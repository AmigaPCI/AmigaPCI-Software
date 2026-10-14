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
#ifndef _JOYSTICK_H
#define _JOYSTICK_H

void joystick_action(uint which, uint up, uint down, uint left, uint right,
                     uint32_t buttons);

extern uint8_t joystick_asserted;

#endif /* _JOYSTICK_H */
