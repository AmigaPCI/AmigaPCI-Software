/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Amiga keyboard scancode definitions.
 */
#ifndef _AMIGA_KBD_CODES_H
#define _AMIGA_KBD_CODES_H

#define AS_BACKTICK    (0x00)  // '~' and '`'
#define AS_1           (0x01)  // '!' and '1'
#define AS_2           (0x02)  // '@' and '2'
#define AS_3           (0x03)  // '#' and '3'
#define AS_4           (0x04)  // '$' and '4'
#define AS_5           (0x05)  // '%' and '5'
#define AS_6           (0x06)  // '^' and '6'
#define AS_7           (0x07)  // '&' and '7'
#define AS_8           (0x08)  // '*' and '8'
#define AS_9           (0x09)  // '(' and '9'
#define AS_0           (0x0a)  // ')' and '0'
#define AS_MINUS       (0x0b)  // '_' and '-'
#define AS_EQUAL       (0x0c)  // '+' and '='
#define AS_BACKSLASH   (0x0d)  // '|' and '\'
// undefined           (0x0e)
#define AS_KP_0        (0x0f)  // Keypad '0'
#define AS_Q           (0x10)  // 'Q' and 'q'
#define AS_W           (0x11)  // 'W' and 'w'
#define AS_E           (0x12)  // 'E' and 'e'
#define AS_R           (0x13)  // 'R' and 'r'
#define AS_T           (0x14)  // 'T' and 't'
#define AS_Y           (0x15)  // 'Y' and 'y'
#define AS_U           (0x16)  // 'U' and 'u'
#define AS_I           (0x17)  // 'I' and 'i'
#define AS_O           (0x18)  // 'O' and 'o'
#define AS_P           (0x19)  // 'P' and 'p'
#define AS_LBRACKET    (0x1a)  // '{' and '['
#define AS_RBRACKET    (0x1b)  // '}' and ']'
#define AS_RESET_BTN   (0x1c)  // Reset button (AmigaPCI reset button)
#define AS_KP_1        (0x1d)  // Keypad '1'
#define AS_KP_2        (0x1e)  // Keypad '2'
#define AS_KP_3        (0x1f)  // Keypad '3'
#define AS_A           (0x20)  // 'A' and 'a'
#define AS_S           (0x21)  // 'S' and 's'
#define AS_D           (0x22)  // 'D' and 'd'
#define AS_F           (0x23)  // 'F' and 'f'
#define AS_G           (0x24)  // 'G' and 'g'
#define AS_H           (0x25)  // 'H' and 'h'
#define AS_J           (0x26)  // 'J' and 'j'
#define AS_K           (0x27)  // 'K' and 'k'
#define AS_L           (0x28)  // 'L' and 'l'
#define AS_SEMICOLON   (0x29)  // ';' and ':'
#define AS_APOSTROPHE  (0x2a)  // ''' and '"'
#define AS_EXTRA1      (0x2b)  // Key next to Enter
#define AS_POWER_BTN   (0x2c)  // Power button (AmigaPCI power button)
#define AS_KP_4        (0x2d)  // Keypad '4'
#define AS_KP_5        (0x2e)  // Keypad '5'
#define AS_KP_6        (0x2f)  // Keypad '6'
#define AS_EXTRA2      (0x30)  // Key next to Left Shift
#define AS_Z           (0x31)  // 'Z' and 'z'
#define AS_X           (0x32)  // 'X' and 'x'
#define AS_C           (0x33)  // 'C' and 'c'
#define AS_V           (0x34)  // 'V' and 'v'
#define AS_B           (0x35)  // 'B' and 'b'
#define AS_N           (0x36)  // 'N' and 'n'
#define AS_M           (0x37)  // 'M' and 'm'
#define AS_COMMA       (0x38)  // '<' and ','
#define AS_DOT         (0x39)  // '>' and '.'
#define AS_SLASH       (0x3a)  // '>' and '/'
// undefined           (0x3b)
#define AS_KP_DOT      (0x3c)  // Keypad '.'
#define AS_KP_7        (0x3d)  // Keypad '7'
#define AS_KP_8        (0x3e)  // Keypad '8'
#define AS_KP_9        (0x3f)  // Keypad '9'
#define AS_SPACE       (0x40)  // Space
#define AS_BACKSPACE   (0x41)  // Backspace
#define AS_TAB         (0x42)  // Tab
#define AS_KP_ENTER    (0x43)  // Keypad Enter
#define AS_ENTER       (0x44)  // Enter
#define AS_ESC         (0x45)  // ESC
#define AS_DELETE      (0x46)  // Delete
#define AS_INSERT      (0x47)  // Insert       (not on classic keyboards)
#define AS_PAGEUP      (0x48)  // Page Up      (not on classic keyboards)
#define AS_PAGEDOWN    (0x49)  // Page Down    (not on classic keyboards)
#define AS_KP_MINUS    (0x4a)  // Keypad '-'
#define AS_F11         (0x4b)  // F11          (not on classic keyboards)
#define AS_UP          (0x4c)  // Cursor Up
#define AS_DOWN        (0x4d)  // Cursor Down
#define AS_RIGHT       (0x4e)  // Cursor Right
#define AS_LEFT        (0x4f)  // Cursor Left
#define AS_F1          (0x50)  // F1
#define AS_F2          (0x51)  // F2
#define AS_F3          (0x52)  // F3
#define AS_F4          (0x53)  // F4
#define AS_F5          (0x54)  // F5
#define AS_F6          (0x55)  // F6
#define AS_F7          (0x56)  // F7
#define AS_F8          (0x57)  // F8
#define AS_F9          (0x58)  // F9
#define AS_F10         (0x59)  // F10
#define AS_KP_LPAREN   (0x5a)  // Keypad '('
#define AS_KP_RPAREN   (0x5b)  // Keypad ')'
#define AS_KP_DIV      (0x5c)  // Keypad '/'
#define AS_KP_MUL      (0x5d)  // Keypad '*'
#define AS_KP_PLUS     (0x5e)  // Keypad '+'
#define AS_HELP        (0x5f)  // Help
#define AS_LEFTSHIFT   (0x60)  // Left Shift
#define AS_RIGHTSHIFT  (0x61)  // Right Shift
#define AS_CAPSLOCK    (0x62)  // Caps Lock
#define AS_CTRL        (0x63)  // Ctrl
#define AS_LEFTALT     (0x64)  // Left Alt
#define AS_RIGHTALT    (0x65)  // Right Alt
#define AS_LEFTAMIGA   (0x66)  // Left Amiga
#define AS_RIGHTAMIGA  (0x67)  // Right Amiga
// undefined           (0x68)
// undefined           (0x69)
// undefined           (0x6a)
#define AS_MENU        (0x6b)  // Menu         (not on classic keyboards)
// undefined           (0x6c)
#define AS_PRINTSCR    (0x6d)  // Print screen (not on classic keyboards)
#define AS_BREAK       (0x6e)  // Break        (not on classic keyboards)
#define AS_F12         (0x6f)  // F12          (not on classic keyboards)
#define AS_HOME        (0x70)  // Home         (not on classic keyboards)
#define AS_END         (0x71)  // End          (not on classic keyboards)
#define AS_STOP        (0x72)  // Stop         (CDTV & CD32)
#define AS_PLAYPAUSE   (0x73)  // Play/Pause   (CDTV & CD32)
#define AS_PREVTRACK   (0x74)  // Prev Track   (CDTV & CD32)  << REW
#define AS_NEXTTRACK   (0x75)  // Next Track   (CDTV & CD32)  >> FF
#define AS_SHUFFLE     (0x76)  // Shuffle      (CDTV & CD32)  Random Play
#define AS_REPEAT      (0x77)  // Repeat       (CDTV & CD32)
#define AS_RESET_WARN  (0x78)  // Reset warning
// undefined           (0x79)
#define AS_WHEEL_UP    (0x7a)  // Mouse Wheel Up    (NM_WHEEL_UP)
#define AS_WHEEL_DOWN  (0x7b)  // Mouse Wheel Down  (NM_WHEEL_DOWN)
#define AS_WHEEL_LEFT  (0x7c)  // Mouse Wheel Left  (NM_WHEEL_LEFT)
#define AS_WHEEL_RIGHT (0x7d)  // Mouse Wheel Right (NM_WHEEL_RIGHT)
#define AS_BUTTON_4    (0x7e)  // Mouse Button 4    (NM_BUTTON_FOURTH)
#define AS_BUTTON_5    (0x7f)  // Mouse Button 5    (NM_BUTTON_FIFTH)

#define AS_LOST_SYNC   (0xf9)  // Keyboard lost sync with Amiga
#define AS_BUFOVERFLOW (0xfa)  // Keyboard output buffer overflow
// undefined           (0xfb)
#define AS_POST_FAIL   (0xfc)  // Keyboard selftest failed
#define AS_POWER_INIT  (0xfd)  // Keyboard powerup start key stream
#define AS_POWER_DONE  (0xfe)  // Keyboard powerup done key stream
#define AS_NONE        (0xff)  // Not a valid keycode

/* Pseudo-scancodes for mouse buttons and joystick directions */
#define ASE_BUTTON0_0       (0x80)  // Left mouse button
#define ASE_BUTTON0_1       (0x81)  // Right mouse button
#define ASE_BUTTON0_2       (0x82)  // Middle mouse button
#define ASE_BUTTON0_3       (0x83)  // Extra mouse button
#define ASE_BUTTON0_4       (0x84)  // Extra mouse button
#define ASE_BUTTON1_0       (0x88)  // Left mouse button
#define ASE_BUTTON1_1       (0x89)  // Right mouse button
#define ASE_BUTTON1_2       (0x8a)  // Middle mouse button
#define ASE_BUTTON1_3       (0x8b)  // Extra mouse button
#define ASE_BUTTON1_4       (0x8c)  // Extra mouse button
#define ASE_UNUSED_1        (0x8d)  // Button range: start of unused
#define ASE_UNUSED_2        (0x9b)  // Button range: end of unused
#define ASE_JOYSTICK0_UP    (0x9c)  // Joystick up
#define ASE_JOYSTICK0_DOWN  (0x9d)  // Joystick down
#define ASE_JOYSTICK0_LEFT  (0x9e)  // Joystick left
#define ASE_JOYSTICK0_RIGHT (0x9f)  // Joystick right
#define ASE_JOYSTICK1_UP    (0xa0)  // Joystick up
#define ASE_JOYSTICK1_DOWN  (0xa1)  // Joystick down
#define ASE_JOYSTICK1_LEFT  (0xa2)  // Joystick left
#define ASE_JOYSTICK1_RIGHT (0xa3)  // Joystick right

#endif /* _AMIGA_KBD_CODES_H */
