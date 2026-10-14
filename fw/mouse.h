/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Amiga and USB HID mouse handling.
 */
#ifndef _MOUSE_H
#define _MOUSE_H

void mouse_action(uint which, int off_x, int off_y, int off_wheel, int off_pan);
void mouse_action_button(uint which, uint32_t buttons);
void mouse_init(void);
void mouse_poll(void);
void mouse_put_macro(uint32_t macro, uint is_pressed, uint was_pressed);
void mouse_put_scancode(uint8_t scancode, uint is_pressed, uint was_pressed);
void mouse_set_defaults(void);
void mouse_get_default_buttons(uint start, uint count, uint8_t *buf);
extern uint32_t mouse_buttons_add;
extern uint8_t mouse_asserted;

#endif /* _MOUSE_H */
