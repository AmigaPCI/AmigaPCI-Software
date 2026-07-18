/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Config area management (non-volatile storage)
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#define DF_RTC              0x00000001  // Real-Time Clock
#define DF_AMIGA_KEYBOARD   0x00000002  // Amiga keyboard
#define DF_AMIGA_MOUSE      0x00000004  // Amiga mouse
#define DF_AMIGA_JOYSTICK   0x00000008  // Amiga joystick
#define DF_USB              0x00000010  // USB configuration (noisy)
#define DF_USB_CONN         0x00000020  // USB connect/disconnect
#define DF_USB_KEYBOARD     0x00000040  // USB keyboard
#define DF_USB_MOUSE        0x00000080  // USB mouse
#define DF_USB_REPORT       0x00000100  // USB HID report descriptor
#define DF_USB_DECODE_MISC  0x00000200  // USB HID decode miscellaneous
#define DF_USB_DECODE_MOUSE 0x00000400  // USB HID decode mouse
#define DF_USB_DECODE_JOY   0x00000800  // USB HID decode joystick
#define DF_USB_DECODE_KBD   0x00001000  // USB HID decode keyboard
#define DF_USB_CONTROL      0x00002000  // USB control messages
#define DF_UNUSED1          0x00004000  // Not yet used
#define DF_UNUSED2          0x00008000  // Not yet used
#define DF_I2C              0x00010000  // I2C operations
#define DF_I2C_LL           0x00020000  // I2C low-level operations
#define DF_HIDEN            0x00040000  // HIDEN enable/disable
#define DF_BECMSG           0x00080000  // BEC messages
#define DF_FAN              0x00100000  // Fan limit changes
#define DF_SPI              0x00200000  // SPI operations

#define CF_MOUSE_INVERT_X   0x00000001  // Invert mouse X axis
#define CF_MOUSE_INVERT_Y   0x00000002  // Invert mouse Y axis
#define CF_MOUSE_INVERT_W   0x00000004  // Invert mouse wheel
#define CF_MOUSE_INVERT_P   0x00000008  // Invert mouse AC Pan
#define CF_MOUSE_SWAP_XY    0x00000010  // Swap mouse X and Y axis
#define CF_MOUSE_SWAP_WP    0x00000020  // Swap mouse Wheel and Pan axis
#define CF_MOUSE_KEYUP_WP   0x00000040  // Send key released for wheel / pan
#define CF_GAMEPAD_MOUSE    0x00000080  // Use joystick on gamepad as mouse
#define CF_HAVE_FAN         0x00000100  // Board has fan attached
#define CF_KEYBOARD_NOSYNC  0x00000200  // Skip sync for Amiga keyboard
#define CF_KEYBOARD_SWAPALT 0x00000400  // Swap Alt and Amiga keys

typedef struct {
    uint32_t    magic;          // Structure magic
    uint32_t    crc;            // Structure CRC
    uint16_t    size;           // Structure size in bytes
    uint8_t     valid;          // Structure record is valid
    uint8_t     version;        // Structure version
    char        name[32];       // Unique name for this board
    uint32_t    keymap[256];    // USB key mappings to Amiga keys
    uint32_t    modkeymap[8];   // DEPRECATED USB modifier key mappings
    uint8_t     led_level;      // Power LED brightness (0 to 100)
    uint8_t     ps_on_mode;     // Power supply on at AC restored
    uint8_t     fan_speed;      // Fan speed percent (bit 7 = auto)
    uint8_t     fan_speed_min;  // Fan (auto) speed at minimum temperature
    uint8_t     fan_temp_max;   // Temperature (C) for maximum fan speed
    uint8_t     fan_temp_min;   // Temperature (C) for minimum fan speed
    uint16_t    fan_rpm_max;    // Fan maximum RPM
    uint32_t    debug_flag;     // Debug flags (see DF_* above)
    int8_t      cpu_temp_bias;  // CPU temperature bias
    uint8_t     board_rev;      // Board revision, for board-specific changes
    uint8_t     board_type;     // Board type
    uint8_t     unused1;        // Unused
    uint32_t    flags;          // Runtime flags
    uint32_t    buttonmap[64];  // Mouse and Joystick button mappings
    uint32_t    jdirectmap[4];  // DEPRECATED Joystick direction mappings (UDLR)
    uint32_t    scrollmap[4];   // DEPRECATED Mouse scroll wheel mappings (UDLR)
    uint32_t    sysctlmap[4];   // DEPRECATED System control button mappings
    uint8_t     mouse_div_x;    // Mouse X speed scaling divisor
    uint8_t     mouse_div_y;    // Mouse Y speed scaling divisor
    uint8_t     mouse_mul_x;    // Mouse X speed scaling factor
    uint8_t     mouse_mul_y;    // Mouse Y speed scaling factor
    uint16_t    i2c_max_speed;  // I2C maximum speed
    uint16_t    i2c_min_speed;  // I2C minimum speed
    uint8_t     unused[616];    // Unused
} config_t;

extern config_t config;

void config_updated(void);
void config_poll(void);
void config_read(void);

void config_name(const char *name);
void config_set_led(uint value);
void config_set_defaults(void);

#define BOARD_TYPE_UNKNOWN  0
#define BOARD_TYPE_AMIGAPCI 1
#define BOARD_TYPE_APCIDEV  2
#define BOARD_TYPE_KEYJAM   3

#endif /* _CONFIG_H */
