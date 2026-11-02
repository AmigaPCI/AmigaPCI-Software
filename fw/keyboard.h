/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Amiga and USB HID keyboard handling.
 */
#ifndef _KEYBOARD_H
#define _KEYBOARD_H

/* These definitiions are for USB HID keyboards */
#define KEYBOARD_MODIFIER_LEFTCTRL   BIT(0)  // Left Control
#define KEYBOARD_MODIFIER_LEFTSHIFT  BIT(1)  // Left Shift
#define KEYBOARD_MODIFIER_LEFTALT    BIT(2)  // Left Alt
#define KEYBOARD_MODIFIER_LEFTMETA   BIT(3)  // Left Meta (Windows)
#define KEYBOARD_MODIFIER_RIGHTCTRL  BIT(4)  // Right Control
#define KEYBOARD_MODIFIER_RIGHTSHIFT BIT(5)  // Right Shift
#define KEYBOARD_MODIFIER_RIGHTALT   BIT(6)  // Right Alt
#define KEYBOARD_MODIFIER_RIGHTMETA  BIT(7)  // Right Meta

typedef struct {
    uint8_t modifier;   // Keyboard modifier (KEYBOARD_MODIFIER_* masks)
    uint8_t reserved;   // Reserved for OEM use, always set to 0
    uint8_t keycode[6]; // Key codes of the currently pressed keys
} usb_keyboard_report_t;

void keyboard_put_amiga(uint8_t code);  // Queue Amiga keystroke
void keyboard_put_macro(uint32_t macro, uint is_pressed);  // Queue Amiga macro
void keyboard_usb_input(usb_keyboard_report_t *report);  // USB keyboard input
void keyboard_usb_input_mm(uint16_t *ch, uint count);    // USB multimedia input
void keyboard_usb_input_sysctl(uint16_t buttons);        // USB system ctl key
void keyboard_term(void);  // ASCII terminal input to Amiga
void keyboard_get_default_keys(uint start, uint count, uint8_t *buf);
void keyboard_set_defaults(void);
uint keyboard_reset_warning(void);
uint keyboard_get_capture(uint maxcount, uint16_t *buf);
void keyboard_reply_msg(void);
void keyboard_poll(void);
void keyboard_init(void);
uint8_t capture_scancode(uint16_t keycode);

extern uint8_t  amiga_keyboard_sent_wake;
extern uint8_t  amiga_keyboard_has_sync;
extern uint8_t  amiga_keyboard_lost_sync;
extern volatile uint64_t keyboard_cap_timeout;
extern volatile uint8_t  keyboard_cap_src_req;

/* Newmouse keycodes */
#define NM_WHEEL_UP       (0x7A)
#define NM_WHEEL_DOWN     (0x7B)
#define NM_WHEEL_LEFT     (0x7C)
#define NM_WHEEL_RIGHT    (0x7D)
#define NM_BUTTON_FOURTH  (0x7E)
#define NM_BUTTON_FIFTH   (0x7F)

#define KEYCAP_DOWN   0x0000  // Key pressed
#define KEYCAP_UP     0x0100  // Key released
#define KEYCAP_BUTTON 0x0200  // Key is actually a mouse or joystick button


#endif /* _KEYBOARD_H */
