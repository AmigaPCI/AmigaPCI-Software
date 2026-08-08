/*
 * becky
 * -----
 * Utility to manipulate AmigaPCI HID keyboard mapping tables stored in
 * BEC (the Board Environment Controller).
 *
 * Copyright 2025 Chris Hooper. This program and source may be used
 * and distributed freely, for any purpose which benefits the Amiga
 * community. Commercial use of the binary, source, or algorithms requires
 * prior written approval from Chris Hooper <amiga@cdh.eebugs.com>.
 * All redistributions must retain this Copyright notice.
 *
 * DISCLAIMER: THE SOFTWARE IS PROVIDED "AS-IS", WITHOUT ANY WARRANTY.
 * THE AUTHOR ASSUMES NO LIABILITY FOR ANY DAMAGE ARISING OUT OF THE USE
 * OR MISUSE OF THIS UTILITY OR INFORMATION REPORTED BY THIS UTILITY.
 */
const char *version = "\0$VER: becky "VERSION" ("BUILD_DATE") \xA9 Chris Hooper";

#include <stdio.h>
#include "becmsg.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <libraries/keymap.h>
#include <libraries/asl.h>
#include <exec/types.h>
#include <exec/memory.h>
#include <clib/dos_protos.h>
#include <inline/timer.h>
#include <inline/exec.h>
#include <inline/dos.h>
#include <inline/asl.h>
#include <inline/diskfont.h>
#include <inline/gadtools.h>
#include <libraries/gadtools.h>
#include "amiga_kbd_codes.h"
#include "hid_kbd_codes.h"
#include "becmsg.h"
#include "bec_cmd.h"

/*
 * Define compile-time assert. This macro relies on gcc's built-in
 * static assert checking which is available in C11.
 */
#define STATIC_ASSERT(test_for_true) \
    static_assert((test_for_true), "(" #test_for_true ") failed")

#define BIT(x) (1U << (x))
#define ARRAY_SIZE(x) ((sizeof (x) / sizeof ((x)[0])))

#define KEY_PLAIN       0  // Regular white keycap
#define KEY_SHADED      1  // Non-alphanumeric keys (ESC, Fn, Shift, Alt, etc)
#define KEY_NOT_MAPPED  2  // Key present, but does not have active mapping
#define KEY_PRESSED     3  // Key is actively pressed
#define KEY_NOT_PRESENT 4  // Key is not present in this keymap (ANSI vs ISO)

/* HID key code bytes from BEC */
#define KEYCAP_DOWN   0x00  // Key pressed
#define KEYCAP_UP     0x01  // Key released
#define KEYCAP_BUTTON 0x02  // Key is actually a mouse or joystick button

#define BUTMAP_MODE_MOUSE    0 // Mouse button mapping mode
#define BUTMAP_MODE_JOYSTICK 1 // Joystick button mapping mode

uint        flag_debug = 0;
static uint8_t is_ansi_layout = 1;   // 1=ANSI (North America), 0=ISO (Europe)
static uint8_t enable_esc_exit = 1;  // ESC key will exit program
static uint8_t edit_key_mapping_mode = 0;  // 1=Hex editing a single mapping
static uint8_t button_mapping_mode = 0;    // 0=Mouse, 1=Joystick
static uint8_t gui_initialized = 0;
static BOOL program_done = FALSE;

static uint amiga_keyboard_left;
static uint amiga_keyboard_top;
static uint hid_keyboard_left;
static uint hid_keyboard_top;

static uint win_width          = 500;
static uint win_height         = 220;
static uint kbd_width          = 470;
static uint kbd_height         = 100;
static uint kbd_keyarea_width  = 450;
static uint kbd_keyarea_height = 90;

static const uint font_pixels_x = 8;
static const uint font_pixels_y = 8;
static SHORT gui_print_line1_y;
static SHORT gui_print_line2_y;

typedef struct {
    uint8_t  shaded;
    uint16_t x;
    uint16_t y;
} keysize_t;

#define NUM_SCANCODES 256
static uint8_t hid_scancode_to_amiga[NUM_SCANCODES][4];
static uint hid_button_scancode_to_capnum(uint8_t scancode);

#define NUM_BUTTON_SCANCODES 64
static uint8_t hid_button_scancode_to_amiga[NUM_BUTTON_SCANCODES][4];

/* 1u 1.25u 1.5u 2u 2.25u 9u */
#define U      200
#define PLAIN  KEY_PLAIN
#define SHADED KEY_SHADED
static const keysize_t amiga_keywidths[] = {
    { PLAIN,  1 * U,     1 * U },  //  0: Standard key ('1' key)
    { SHADED, 1 * U,     1 * U },  //  1: Standard shaded key (ESC key)
    { SHADED, 1.22 * U,  1 * U },  //  2: Function and modifier key (F1 key)
    { SHADED, 1.45 * U,  1 * U },  //  3: Tilde and backtick key
    { SHADED, 1.45 * U,  1 * U },  //  4: Del and Help keys
    { SHADED, 2 * U,     1 * U },  //  5: Tab key
    { SHADED, 1.75 * U,  1 * U },  //  6: Left shift (wider for ANSI layout)
    { SHADED, 2.8  * U,  1 * U },  //  7: Right shift
    { SHADED, 1 * U,   2.1 * U },  //  8: Keypad enter key
    { PLAIN,  9 * U,     1 * U },  //  9: Spacebar
    { PLAIN,  2 * U,     1 * U },  // 10: Keypad 0
    { PLAIN,  1 * U,     1 * U },  // 11: Extra keys (ISO layout)
    { SHADED, 1.5 * U, 2.1 * U },  // 12: Enter key
    { PLAIN,  0.6 * U, 1.2 * U },  // 13: Mouse buttons
};

typedef struct {
    uint8_t  scancode;
    uint8_t  type;  // Type (for dimensions) and color
    uint16_t x;
    uint16_t y;
    char    *name;
} keypos_t;

#define C(x) ((char *)(uintptr_t)(x))

static const keypos_t amiga_keypos[] = {
    { AS_ESC,         1,  5086,  5050, "ESC" },
    { AS_F1,          2,  8190,  5050, "F1" },
    { AS_F2,          2, 10578,  5050, "F2" },
    { AS_F3,          2, 12966,  5050, "F3" },
    { AS_F4,          2, 15354,  5050, "F4" },
    { AS_F5,          2, 17742,  5050, "F5" },
    { AS_F6,          2, 20985,  5050, "F6" },
    { AS_F7,          2, 23373,  5050, "F7" },
    { AS_F8,          2, 25761,  5050, "F8" },
    { AS_F9,          2, 28149,  5050, "F9" },
    { AS_F10,         2, 30537,  5050, "F10" },

    { AS_RESET_BTN,   5, 43100,  5000, "Reset" },
    { AS_POWER_BTN,   5, 46900,  5000, "Power" },

    { AS_BACKTICK,    3,  5553,  7917, C('`') },
    { AS_1,           0,  7931,  7917, C('1') },
    { AS_2,           0,  9836,  7917, C('2') },
    { AS_3,           0, 11741,  7917, C('3') },
    { AS_4,           0, 13646,  7917, C('4') },
    { AS_5,           0, 15551,  7917, C('5') },
    { AS_6,           0, 17456,  7917, C('6') },
    { AS_7,           0, 19361,  7917, C('7') },
    { AS_8,           0, 21266,  7917, C('8') },
    { AS_9,           0, 23171,  7917, C('9') },
    { AS_0,           0, 25076,  7917, C('0') },
    { AS_MINUS,       0, 26981,  7917, C('-') },
    { AS_EQUAL,       0, 28886,  7917, C('=') },
    { AS_BACKSLASH,   0, 30791,  7917, C('\\') },
    { AS_BACKSPACE,   1, 32696,  7917, "<-" },
    { AS_DELETE,      4, 35956,  7917, "Del" },
    { AS_HELP,        4, 38815,  7917, "Help" },
    { AS_KP_LPAREN,   1, 42156,  7917, C('(') },
    { AS_KP_RPAREN,   1, 44061,  7917, C(')') },
    { AS_KP_DIV,      1, 45966,  7917, C('/') },
    { AS_KP_MUL,      1, 47871,  7917, C('*') },

    { AS_TAB,         5,  6026,  9822, "->" },

    { AS_Q,           0,  8870,  9822, C('Q') },
    { AS_W,           0, 10775,  9822, C('W') },
    { AS_E,           0, 12680,  9822, C('E') },
    { AS_R,           0, 14585,  9822, C('R') },
    { AS_T,           0, 16490,  9822, C('T') },
    { AS_Y,           0, 18395,  9822, C('Y') },
    { AS_U,           0, 20300,  9822, C('U') },
    { AS_I,           0, 22205,  9822, C('I') },
    { AS_O,           0, 24110,  9822, C('O') },
    { AS_P,           0, 26015,  9822, C('P') },
    { AS_LBRACKET,    0, 27920,  9822, C('[') },
    { AS_RBRACKET,    0, 29825,  9822, C(']') },
    { AS_ENTER,      12, 32250,  9822, "<_/" },  // Enter key (custom shape)
    { AS_KP_7,        0, 42156,  9822, C('7') },
    { AS_KP_8,        0, 44061,  9822, C('8') },
    { AS_KP_9,        0, 45966,  9822, C('9') },
    { AS_KP_MINUS,    1, 47871,  9822, C('-') },

    { AS_CTRL,        2,  5315, 11727, "Ctrl" },
    { AS_CAPSLOCK,    1,  7448, 11727, "Caps" },
    { AS_A,           0,  9353, 11727, C('A') },
    { AS_S,           0, 11258, 11727, C('S') },
    { AS_D,           0, 13163, 11727, C('D') },
    { AS_F,           0, 15068, 11727, C('F') },
    { AS_G,           0, 16973, 11727, C('G') },
    { AS_H,           0, 18878, 11727, C('H') },
    { AS_J,           0, 20783, 11727, C('J') },
    { AS_K,           0, 22688, 11727, C('K') },
    { AS_L,           0, 24593, 11727, C('L') },
    { AS_SEMICOLON,   0, 26498, 11727, C(';') },
    { AS_APOSTROPHE,  0, 28403, 11727, C('\'') },
    { AS_EXTRA1,     11, 30308, 11727, "E1"  },
    { AS_UP,          0, 37388, 11727, C('^') },
    { AS_KP_4,        0, 42156, 11727, C('4') },
    { AS_KP_5,        0, 44061, 11727, C('5') },
    { AS_KP_6,        0, 45966, 11727, C('6') },
    { AS_KP_PLUS,     1, 47871, 11727, C('+') },

    { AS_LEFTSHIFT,   6,  5798, 13632, "Shift" },
    { AS_EXTRA2,     11,  8413, 13632, "E2" },
    { AS_Z,           0, 10318, 13632, C('Z') },
    { AS_X,           0, 12223, 13632, C('X') },
    { AS_C,           0, 14128, 13632, C('C') },
    { AS_V,           0, 16033, 13632, C('V') },
    { AS_B,           0, 17938, 13632, C('B') },
    { AS_N,           0, 19843, 13632, C('N') },
    { AS_M,           0, 21748, 13632, C('M') },
    { AS_COMMA,       0, 23653, 13632, C(',') },
    { AS_DOT,         0, 25558, 13632, C('.') },
    { AS_SLASH,       0, 27463, 13632, C('/') },
    { AS_RIGHTSHIFT,  7, 31045, 13632, "Shift" },
    { AS_LEFT,        0, 35483, 13632, C('<') },
    { AS_DOWN,        0, 37388, 13632, C('v') },
    { AS_RIGHT,       0, 39293, 13632, C('>') },
    { AS_KP_1,        0, 42156, 13632, C('1') },
    { AS_KP_2,        0, 44061, 13632, C('2') },
    { AS_KP_3,        0, 45966, 13632, C('3') },

    { AS_KP_ENTER,    8, 47871, 13632, "_/" },

    { AS_LEFTALT,     2,  6686, 15537, "Alt" },
    { AS_LEFTAMIGA,   2,  9074, 15537, "A" },
    { AS_SPACE,       9, 18840, 15537, "" },
    { AS_RIGHTAMIGA,  2, 28606, 15537, "A" },
    { AS_RIGHTALT,    2, 30994, 15537, "Alt" },
    { AS_KP_0,       10, 43121, 15537, C('0') },
    { AS_KP_DOT,      0, 45966, 15537, C('.') },
    { 0x80,          13,  5000, 18520, C('1') },  // Mouse button 1
    { 0x81,          13,  6200, 18520, C('2') },  // Mouse button 2
    { 0x82,          13,  7400, 18520, C('3') },  // Mouse button 3
    { AS_BUTTON_4,   13,  8600, 18520, C('4') },  // Mouse button 4
    { AS_BUTTON_5,   13,  9800, 18520, C('5') },  // Mouse button 5
    { AS_WHEEL_UP,    0, 12000, 18520, "WU" },    // Mouse wheel up
    { AS_WHEEL_DOWN,  0, 14000, 18520, "WD" },    // Mouse wheel down
    { AS_WHEEL_LEFT,  0, 16000, 18520, "WL" },    // Mouse wheel left
    { AS_WHEEL_RIGHT, 0, 18000, 18520, "WR" },    // Mouse wheel right
};

typedef struct {
    SHORT x_min;
    SHORT y_min;
    SHORT x_max;
    SHORT y_max;
} key_bbox_t;


#define AKB_MIN_X   5000
#define AKB_MIN_Y   5000
#define AKB_MAX_X  48500
#define AKB_MAX_Y  17000

#define HIDKB_MIN_X  5000
#define HIDKB_MIN_Y  5000
#define HIDKB_MAX_X 47600
#define HIDKB_MAX_Y 17000

// 1u    Alphanumeric keys (A, B, C, etc.), number keys, function keys
// 1.25u Ctrl, Alt, GUI (Windows/Command) keys (bottom row)
// 1.5u  Tab
// 1.75u Caps Lock
// 2u    Backspace, Numpad + and Enter
// 2.25u Left Shift, Enter
// 2.75u Right Shift
// 6.25u Spacebar (some are 7u)
//
// Measured (mm)
//       Top  Base
//       13.8 18.4  Alphanumeric
//       23.8 23.5  Ctrl, Meta, Alt, Fn
//       24.6       Tab, backslash
//       27.9       Capslock
//       33.1 36.3  Backspace, Numpad 0, KP + height, KP enter height
//       38.4 41.1  Left shift
//            42.6  Enter
//       46.4       Right shift
//      113.2       Spacebar

static const keysize_t hid_keywidths[] = {
    { PLAIN,  1 * U,     1 * U },  //  0: Standard key ('1' key)
    { SHADED, 1 * U,     1 * U },  //  1: Standard shaded key (ESC key)
    { SHADED, 1.25 * U,  1 * U },  //  2: Modifier key (Ctrl, Meta, Alt, Fn)
    { SHADED, 2 * U,     1 * U },  //  3: Backspace
    { SHADED, 1.5 * U,   1 * U },  //  4: Del and Help keys
    { SHADED, 1.50 * U,  1 * U },  //  5: Tab key
    { SHADED, 2.30 * U,  1 * U },  //  6: Left shift
    { SHADED, 2.80 * U,  1 * U },  //  7: Right shift
    { SHADED, 1 * U,   2.2 * U },  //  8: Keypad enter key
    { PLAIN,  6.3 * U,   1 * U },  //  9: Spacebar
    { PLAIN,  2 * U,     1 * U },  // 10: Keypad 0
    { SHADED, 1.75 * U,  1 * U },  // 11: Capslock
    { SHADED, 2.25 * U,  1 * U },  // 12: Enter key
};

static const keypos_t hid_keypos[] = {
    { HS_ESC,         1,  5086,  5050, "ESC" },
    { HS_F1,          1,  8895,  5050, "F1" },
    { HS_F2,          1, 10800,  5050, "F2" },
    { HS_F3,          1, 12705,  5050, "F3" },
    { HS_F4,          1, 14610,  5050, "F4" },
    { HS_F5,          1, 17468,  5050, "F5" },
    { HS_F6,          1, 19373,  5050, "F6" },
    { HS_F7,          1, 21278,  5050, "F7" },
    { HS_F8,          1, 23183,  5050, "F8" },
    { HS_F9,          1, 26041,  5050, "F9" },
    { HS_F10,         1, 27946,  5050, "F10" },
    { HS_F11,         1, 29851,  5050, "F11" },
    { HS_F12,         1, 31756,  5050, "F12" },

    { HS_PRTSCN,      1, 34613,  5050, "PrtSc" },
    { HS_SCROLL_LOCK, 1, 36518,  5050, "ScrLk" },
    { HS_PAUSE,       1, 38423,  5050, "Pause" },

    { HS_MEDIA_S_UP,   0, 41000, 5050, "WU" },    // Mouse wheel up
    { HS_MEDIA_S_DOWN, 0, 43000, 5050, "WD" },    // Mouse wheel down
    { HS_MEDIA_BACK,   0, 45000, 5050, "WL" },    // Mouse wheel left
    { HS_MEDIA_FWD,    0, 47000, 5050, "WR" },    // Mouse wheel right

    { HS_BACKTICK,    1,  5086,  7917, C('`') },
    { HS_1,           0,  6991,  7917, C('1') },
    { HS_2,           0,  8896,  7917, C('2') },
    { HS_3,           0, 10801,  7917, C('3') },
    { HS_4,           0, 12706,  7917, C('4') },
    { HS_5,           0, 14611,  7917, C('5') },
    { HS_6,           0, 16516,  7917, C('6') },
    { HS_7,           0, 18421,  7917, C('7') },
    { HS_8,           0, 20326,  7917, C('8') },
    { HS_9,           0, 22231,  7917, C('9') },
    { HS_0,           0, 24136,  7917, C('0') },
    { HS_MINUS,       0, 26041,  7917, C('-') },
    { HS_EQUAL,       0, 27946,  7917, C('=') },
    { HS_BACKSPACE,   3, 30803,  7917, "<-" },

    { HS_INSERT,      1, 34613,  7917, "Ins" },
    { HS_HOME,        1, 36518,  7917, "Home" },
    { HS_PAGEUP,      1, 38423,  7917, "PgUp" },

    { HS_NUMLOCK,     1, 41280,  7917, "NumLk" },
    { HS_KP_DIV,      1, 43185,  7917, C('/') },
    { HS_KP_MUL,      1, 45090,  7917, C('*') },
    { HS_KP_MINUS,    1, 46995,  7917, C('-') },

    { HS_TAB,         5,  5567,  9822, "->" },

    { HS_Q,           0,  7948,  9822, C('Q') },
    { HS_W,           0,  9853,  9822, C('W') },
    { HS_E,           0, 11758,  9822, C('E') },
    { HS_R,           0, 13663,  9822, C('R') },
    { HS_T,           0, 15568,  9822, C('T') },
    { HS_Y,           0, 17473,  9822, C('Y') },
    { HS_U,           0, 19378,  9822, C('U') },
    { HS_I,           0, 21283,  9822, C('I') },
    { HS_O,           0, 23188,  9822, C('O') },
    { HS_P,           0, 25093,  9822, C('P') },
    { HS_LBRACKET,    0, 26998,  9822, C('[') },
    { HS_RBRACKET,    0, 28903,  9822, C(']') },
    { HS_BACKSLASH,   5, 31284,  9822, C('\\') },

    { HS_DELETE,      1, 34613,  9822, "Del" },
    { HS_END,         1, 36518,  9822, "End" },
    { HS_PAGEDOWN,    1, 38423,  9822, "PgDn" },

    { HS_KP_7,        0, 41280,  9822, C('7') },
    { HS_KP_8,        0, 43184,  9822, C('8') },
    { HS_KP_9,        0, 45090,  9822, C('9') },

    { HS_KP_PLUS,     8, 46995, 10775, C('+') },

    { HS_CAPSLOCK,   11,  5800, 11727, "Caps" },
    { HS_A,           0,  8419, 11727, C('A') },
    { HS_S,           0, 10324, 11727, C('S') },
    { HS_D,           0, 12229, 11727, C('D') },
    { HS_F,           0, 14134, 11727, C('F') },
    { HS_G,           0, 16069, 11727, C('G') },
    { HS_H,           0, 17944, 11727, C('H') },
    { HS_J,           0, 19849, 11727, C('J') },
    { HS_K,           0, 21754, 11727, C('K') },
    { HS_L,           0, 23659, 11727, C('L') },
    { HS_SEMICOLON,   0, 25564, 11727, C(';') },
    { HS_APOSTROPHE,  0, 27469, 11727, C('\'') },
    { HS_ENTER,      12, 30517, 11727, "<_/" },  // Enter

    { HS_KP_4,        0, 41280, 11727, C('4') },
    { HS_KP_5,        0, 43184, 11727, C('5') },
    { HS_KP_6,        0, 45090, 11727, C('6') },

    { HS_LSHIFT,      6,  6276, 13632, "Shift" },
    { HS_Z,           0,  9371, 13632, C('Z') },
    { HS_X,           0, 11276, 13632, C('X') },
    { HS_C,           0, 13181, 13632, C('C') },
    { HS_V,           0, 15086, 13632, C('V') },
    { HS_B,           0, 16991, 13632, C('B') },
    { HS_N,           0, 18896, 13632, C('N') },
    { HS_M,           0, 20801, 13632, C('M') },
    { HS_COMMA,       0, 22706, 13632, C(',') },
    { HS_DOT,         0, 24611, 13632, C('.') },
    { HS_SLASH,       0, 26516, 13632, C('/') },
    { HS_RSHIFT,      7, 30087, 13632, "Shift" },
    { HS_UP,          0, 36518, 13632, C('^') },
    { HS_KP_1,        0, 41280, 13632, C('1') },
    { HS_KP_2,        0, 43184, 13632, C('2') },
    { HS_KP_3,        0, 45090, 13632, C('3') },

    { HS_KP_ENTER,    8, 46995, 14577, "_/" },

    { HS_LCTRL,       2,  5324, 15537, "Ctrl" },
    { HS_LMETA,       2,  7705, 15537, "Meta" },
    { HS_LALT,        2, 10086, 15537, "Alt" },
    { HS_SPACE,       9, 17182, 15537, "" },
    { HS_RALT,        2, 24278, 15537, "Alt" },
    { HS_RMETA,       2, 26659, 15537, "Meta" },
    { HS_MENU,        2, 29040, 15537, "Menu" },
    { HS_RCTRL,       2, 31421, 15537, "Ctrl" },
    { HS_LEFT,        0, 34613, 15537, C('<') },
    { HS_DOWN,        0, 36518, 15537, C('v') },
    { HS_RIGHT,       0, 38423, 15537, C('>') },
    { HS_KP_0,       10, 42232, 15537, C('0') },
    { HS_KP_DOT,      0, 45090, 15537, C('.') },
};

static const char *amiga_scancode_to_long_name[] = {
    "Back quote",     // 0x00 "'" aka backtick or grave accent
    "1",              // 0x01 "1"
    "2",              // 0x02 "2"
    "3",              // 0x03 "3"
    "4",              // 0x04 "4"
    "5",              // 0x05 "5"
    "6",              // 0x06 "6"
    "7",              // 0x07 "7"
    "8",              // 0x08 "8"
    "9",              // 0x09 "9"
    "0",              // 0x0a "0"
    "-",              // 0x0b "-"
    "=",              // 0x0c "="
    "Backslash",      // 0x0d Backslash
    "",               // 0x0e (undefined)
    "KP 0",           // 0x0f Keypad "0"
    "Q",              // 0x10 "Q"
    "W",              // 0x11 "W"
    "E",              // 0x12 "E"
    "R",              // 0x13 "R"
    "T",              // 0x14 "T"
    "Y",              // 0x15 "Y"
    "U",              // 0x16 "U"
    "I",              // 0x17 "I"
    "O",              // 0x18 "O"
    "P",              // 0x19 "P"
    "[",              // 0x1a Left bracket "["
    "]",              // 0x1b Right bracket "]"
    "Reset Button",   // 0x1c AmigaPCI reset button, undefined for others
    "KP 1",           // 0x1d Keypad "1"
    "KP 2",           // 0x1e Keypad "2"
    "KP 3",           // 0x1f Keypad "3"
    "A",              // 0x20 "A"
    "S",              // 0x21 "S"
    "D",              // 0x22 "D"
    "F",              // 0x23 "F"
    "G",              // 0x24 "G"
    "H",              // 0x25 "H"
    "J",              // 0x26 "J"
    "K",              // 0x27 "K"
    "L",              // 0x28 "L"
    ";",              // 0x29 Semicolon ";"
    "'",              // 0x2a Apostrophe "'"
    "Extra 1",        // 0x2b ISO Keyboard extra key 1 (next to Enter)
    "Power Button",   // 0x2c AmigaPCI power button, undefined for others
    "KP 4",           // 0x2d Keypad "4"
    "KP 5",           // 0x2e Keypad "5"
    "KP 6",           // 0x2f Keypad "6"
    "Extra 2",        // 0x30 ISO Keyboard extra key 2
    "Z",              // 0x31 "Z"
    "X",              // 0x32 "X"
    "C",              // 0x33 "C"
    "V",              // 0x34 "V"
    "B",              // 0x35 "B"
    "N",              // 0x36 "N"
    "M",              // 0x37 "M"
    ",",              // 0x38 Comma ","
    ".",              // 0x39 Period "."
    "/",              // 0x3a Slash "/"
    "",               // 0x3b (undefined)
    "KP .",           // 0x3c Keypad period "."
    "KP 7",           // 0x3d Keypad "7"
    "KP 8",           // 0x3e Keypad "8"
    "KP 9",           // 0x3f Keypad "9"
    "Space",          // 0x40 Space " "
    "Backspace",      // 0x41 Backspace
    "Tab",            // 0x42 Tab
    "KP Enter",       // 0x43 Keypad Enter
    "Enter",          // 0x44 Enter
    "ESC",            // 0x45 ESC
    "Delete",         // 0x46 DeLete
    "Insert",         // 0x47 Insert       (not on classic keyboards)
    "Page Up",        // 0x48 Page Up      (not on classic keyboards)
    "Page Down",      // 0x49 Page Down    (not on classic keyboards)
    "KP -",           // 0x4a Keypad minus "-"
    "F11",            // 0x4b F11          (not on classic keyboards)
    "Cursor Up",      // 0x4c Cursor Up
    "Cursor Down",    // 0x4d Cursor Down
    "Cursor Right",   // 0x4e Cursor Right
    "Cursor Left",    // 0x4f Cursor Left
    "F1",             // 0x50 F1
    "F2",             // 0x51 F2
    "F3",             // 0x52 F3
    "F4",             // 0x53 F4
    "F5",             // 0x54 F5
    "F6",             // 0x55 F6
    "F7",             // 0x56 F7
    "F8",             // 0x57 F8
    "F9",             // 0x58 F9
    "F10",            // 0x59 F10
    "KP (",           // 0x5a Keypad left paren "("
    "KP )",           // 0x5b Keypad right paren ")"
    "KP /",           // 0x5c Keypad slash "/"
    "KP *",           // 0x5d Keypad multiply "*"
    "KP +",           // 0x5e Keypad plus "+"
    "Help",           // 0x5f Help
    "Left Shift",     // 0x60 Left Shift
    "Right Shift",    // 0x61 Right Shift
    "Caps Lock",      // 0x62 Caps Lock
    "Ctrl",           // 0x63 Control
    "Left Alt",       // 0x64 Left Alt
    "Right Alt",      // 0x65 Right Alt
    "Left Amiga",     // 0x66 Left Amiga
    "Right Amiga",    // 0x67 Right Amiga
    "",               // 0x68 (undefined)
    "",               // 0x69 (undefined)
    "",               // 0x6a (undefined)
    "Menu",           // 0x6b Menu         (not on classic keyboards)
    "",               // 0x6c (undefined)
    "Print Screen",   // 0x6d Print Screen (not on classic keyboards)
    "Break",          // 0x6e Break        (not on classic keyboards)
    "F12",            // 0x6b F12          (not on classic keyboards)
    "Home",           // 0x70 Home         (not on classic keyboards)
    "End",            // 0x71 End          (not on classic keyboards)
    "Stop",           // 0x72 Stop         (CDTV & CD32)
    "Play/Pause",     // 0x73 Play/Pause   (CDTV & CD32)
    "Prev track",     // 0x74 Prev Track   (CDTV & CD32)
    "Next track",     // 0x75 Next Track   (CDTV & CD32)
    "Shuffle",        // 0x76 Shuffle      (CDTV & CD32)
    "Repeat",         // 0x77 Repeat       (CDTV & CD32)
    "Reset warning",  // 0x78 Reset warning
    "",               // 0x79 (undefined)
    "wheel up",       // 0x7a Mouse Wheel Up     (NM_WHEEL_UP)
    "wheel down",     // 0x7b Mouse Wheel Down   (NM_WHEEL_DOWN)
    "wheel left",     // 0x7c Mouse Wheel Left   (NM_WHEEL_LEFT)
    "wheel right",    // 0x7d Mouse Wheel Right  (NM_WHEEL_RIGHT)
    "Button 4",       // 0x7e Mouse button 4     (NM_BUTTON_FOURTH)
    "Button 5",       // 0x7f Mouse button 5     (NM_BUTTON_FIFTH)
    "Button 1",       // 0x80 Mouse button 1     (AmigaPCI)
    "Button 2",       // 0x81 Mouse button 2     (AmigaPCI)
    "Button 3",       // 0x82 Mouse button 3     (AmigaPCI)
    "Button 4",       // 0x83 Mouse button 4     (AmigaPCI)
    "Button 5",       // 0x84 Mouse button 5     (AmigaPCI)
    "Button 6",       // 0x85 Mouse button 6     (AmigaPCI)
    "Button 7",       // 0x86 Mouse button 7     (AmigaPCI)
    "Button 8",       // 0x87 Mouse button 8     (AmigaPCI)
    "Button 9",       // 0x88 Mouse button 9     (AmigaPCI)
    "Button 10",      // 0x89 Mouse button 10    (AmigaPCI)
    "Button 11",      // 0x8a Mouse button 11    (AmigaPCI)
    "Button 12",      // 0x8b Mouse button 12    (AmigaPCI)
    "Button 13",      // 0x8c Mouse button 13    (AmigaPCI)
    "Button 14",      // 0x8d Mouse button 14    (AmigaPCI)
    "Button 15",      // 0x8e Mouse button 15    (AmigaPCI)
    "Button 16",      // 0x8f Mouse button 16    (AmigaPCI)
    "",               // 0x90 Mouse button 17    (AmigaPCI reserved)
    "",               // 0x91 Mouse button 18    (AmigaPCI reserved)
    "",               // 0x92 Mouse button 19    (AmigaPCI reserved)
    "",               // 0x93 Mouse button 20    (AmigaPCI reserved)
    "",               // 0x94 Mouse button 21    (AmigaPCI reserved)
    "",               // 0x95 Mouse button 22    (AmigaPCI reserved)
    "",               // 0x96 Mouse button 23    (AmigaPCI reserved)
    "",               // 0x97 Mouse button 24    (AmigaPCI reserved)
    "",               // 0x98 Mouse button 25    (AmigaPCI reserved)
    "",               // 0x99 Mouse button 26    (AmigaPCI reserved)
    "",               // 0x9a Mouse button 27    (AmigaPCI reserved)
    "",               // 0x9b Mouse button 28    (AmigaPCI reserved)
    "Joystick up",    // 0x9c Mouse button 29    (AmigaPCI reserved)
    "Joystick down",  // 0x9d Mouse button 30    (AmigaPCI reserved)
    "Joystick left",  // 0x9e Mouse button 31    (AmigaPCI reserved)
    "Joystick right", // 0x9f Mouse button 32    (AmigaPCI reserved)
    "JButton 1",      // 0xa0 Joystick button 1  (AmigaPCI)
    "JButton 2",      // 0xa1 Joystick button 2  (AmigaPCI)
    "JButton 3",      // 0xa2 Joystick button 3  (AmigaPCI)
    "JButton 4",      // 0xa3 Joystick button 4  (AmigaPCI)
    "JButton 5",      // 0xa4 Joystick button 5  (AmigaPCI)
    "JButton 6",      // 0xa5 Joystick button 6  (AmigaPCI)
    "JButton 7",      // 0xa6 Joystick button 7  (AmigaPCI)
    "JButton 8",      // 0xa7 Joystick button 8  (AmigaPCI)
    "JButton 9",      // 0xa8 Joystick button 9  (AmigaPCI)
    "JButton 10",     // 0xa9 Joystick button 10 (AmigaPCI)
    "JButton 11",     // 0xaa Joystick button 11 (AmigaPCI)
    "JButton 12",     // 0xab Joystick button 12 (AmigaPCI)
    "JButton 13",     // 0xac Joystick button 13 (AmigaPCI)
    "JButton 14",     // 0xad Joystick button 14 (AmigaPCI)
    "JButton 15",     // 0xae Joystick button 15 (AmigaPCI)
    "JButton 16",     // 0xaf Joystick button 16 (AmigaPCI)
    "",               // 0xb0 Joystick button 17 (AmigaPCI reserved)
    "",               // 0xb1 Joystick button 18 (AmigaPCI reserved)
    "",               // 0xb2 Joystick button 19 (AmigaPCI reserved)
    "",               // 0xb3 Joystick button 20 (AmigaPCI reserved)
    "",               // 0xb4 Joystick button 21 (AmigaPCI reserved)
    "",               // 0xb5 Joystick button 22 (AmigaPCI reserved)
    "",               // 0xb6 Joystick button 23 (AmigaPCI reserved)
    "",               // 0xb7 Joystick button 24 (AmigaPCI reserved)
    "",               // 0xb8 Joystick button 25 (AmigaPCI reserved)
    "",               // 0xb9 Joystick button 26 (AmigaPCI reserved)
    "",               // 0xba Joystick button 27 (AmigaPCI reserved)
    "",               // 0xbb Joystick button 28 (AmigaPCI reserved)
    "",               // 0xbc Joystick button 29 (AmigaPCI reserved)
    "",               // 0xbd Joystick button 30 (AmigaPCI reserved)
    "",               // 0xbe Joystick button 31 (AmigaPCI reserved)
    "",               // 0xbf Joystick button 32 (AmigaPCI reserved)
    "",               // 0xc0 (undefined)
    "",               // 0xc1 (undefined)
    "",               // 0xc2 (undefined)
    "",               // 0xc3 (undefined)
    "",               // 0xc4 (undefined)
    "",               // 0xc5 (undefined)
    "",               // 0xc6 (undefined)
    "",               // 0xc7 (undefined)
    "",               // 0xc8 (undefined)
    "",               // 0xc9 (undefined)
    "",               // 0xca (undefined)
    "",               // 0xcb (undefined)
    "",               // 0xcc (undefined)
    "",               // 0xcd (undefined)
    "",               // 0xce (undefined)
    "",               // 0xcf (undefined)
    "",               // 0xd0 (undefined)
    "",               // 0xd1 (undefined)
    "",               // 0xd2 (undefined)
    "",               // 0xd3 (undefined)
    "",               // 0xd4 (undefined)
    "",               // 0xd5 (undefined)
    "",               // 0xd6 (undefined)
    "",               // 0xd7 (undefined)
    "",               // 0xd8 (undefined)
    "",               // 0xd9 (undefined)
    "",               // 0xda (undefined)
    "",               // 0xdb (undefined)
    "",               // 0xdc (undefined)
    "",               // 0xdd (undefined)
    "",               // 0xde (undefined)
    "",               // 0xdf (undefined)
    "",               // 0xe0 (undefined)
    "",               // 0xe1 (undefined)
    "",               // 0xe2 (undefined)
    "",               // 0xe3 (undefined)
    "",               // 0xe4 (undefined)
    "",               // 0xe5 (undefined)
    "",               // 0xe6 (undefined)
    "",               // 0xe7 (undefined)
    "",               // 0xe8 (undefined)
    "",               // 0xe9 (undefined)
    "",               // 0xea (undefined)
    "",               // 0xeb (undefined)
    "",               // 0xec (undefined)
    "",               // 0xed (undefined)
    "",               // 0xee (undefined)
    "",               // 0xef (undefined)
    "",               // 0xf0 (undefined)
    "",               // 0xf1 (undefined)
    "",               // 0xf2 (undefined)
    "",               // 0xf3 (undefined)
    "",               // 0xf4 (undefined)
    "",               // 0xf5 (undefined)
    "",               // 0xf6 (undefined)
    "",               // 0xf7 (undefined)
    "",               // 0xf8 (undefined)
    "Lost Sync",      // 0xf9 Keyboard lost sync
    "Buf Overflow",   // 0xfa Keyboard output buffer overflow
    "",               // 0xfb (undefined)
    "POST Fail",      // 0xfc Keyboard selftest failed
    "Power Init",     // 0xfd Keyboard powerup start key stream
    "Power Done",     // 0xfe Keyboard powerup done key stream
    "Invalid",        // 0xff Not a valid keycode
};

static const char *hid_scancode_to_long_name[] = {
    "NONE",           // 0x00 No key pressed
    "Rollover",       // 0x01 Keyboard Error Roll Over
    "POST Fail",      // 0x02 Keyboard POST Fail
    "KBD Error",      // 0x03 Keyboard error
    "A",              // 0x04 "A"
    "B",              // 0x05 "B"
    "C",              // 0x06 "C"
    "D",              // 0x07 "D"
    "E",              // 0x08 "E"
    "F",              // 0x09 "F"
    "G",              // 0x0a "G"
    "H",              // 0x0b "H"
    "I",              // 0x0c "I"
    "J",              // 0x0d "J"
    "K",              // 0x0e "K"
    "L",              // 0x0f "L"
    "M",              // 0x10 "M"
    "N",              // 0x11 "N"
    "O",              // 0x12 "O"
    "P",              // 0x13 "P"
    "Q",              // 0x14 "Q"
    "R",              // 0x15 "R"
    "S",              // 0x16 "S"
    "T",              // 0x17 "T"
    "U",              // 0x18 "U"
    "V",              // 0x19 "V"
    "W",              // 0x1a "W"
    "X",              // 0x1b "X"
    "Y",              // 0x1c "Y"
    "Z",              // 0x1d "Z"
    "1",              // 0x1e "1"
    "2",              // 0x1f "2"
    "3",              // 0x20 "3"
    "4",              // 0x21 "4"
    "5",              // 0x22 "5"
    "6",              // 0x23 "6"
    "7",              // 0x24 "7"
    "8",              // 0x25 "8"
    "9",              // 0x26 "9"
    "0",              // 0x27 "0"
    "Enter",          // 0x28 Enter
    "ESC",            // 0x29 ESC
    "Backspace",      // 0x2a Backspace
    "Tab",            // 0x2b Tab
    "Space",          // 0x2c Space
    "-",              // 0x2d Minus "-"
    "=",              // 0x2e Equals "="
    "[",              // 0x2f Left Bracket "["
    "]",              // 0x30 Right Bracket "]"
    "Backslash",      // 0x31 Backslash
    "Number/Tilde",   // 0x32 ISO Number Tilde
    ";",              // 0x33 Semicolon ";"
    "'",              // 0x34 Colon "'"
    "Back quote",     // 0x35 Back quote "`"
    ",",              // 0x36 Comma ","
    ".",              // 0x37 Period "."
    "/",              // 0x38 Forward slash "/"
    "Caps Lock",      // 0x39 Caps Lock
    "F1",             // 0x3a F1
    "F2",             // 0x3b F2
    "F3",             // 0x3c F3
    "F4",             // 0x3d F4
    "F5",             // 0x3e F5
    "F6",             // 0x3f F6
    "F7",             // 0x40 F7
    "F8",             // 0x41 F8
    "F9",             // 0x42 F9
    "F10",            // 0x43 F10
    "F11",            // 0x44 F11
    "F12",            // 0x45 F12
    "Print Screen",   // 0x46 Print Screen
    "Scroll Lock",    // 0x47 Scroll Lock
    "Pause",          // 0x48 Pause
    "Insert",         // 0x49 Insert
    "Home",           // 0x4a Home
    "Page Up",        // 0x4b Page Up
    "Delete",         // 0x4c Delete
    "End",            // 0x4d End
    "Page Down",      // 0x4e Page Down
    "Cursor Right",   // 0x4f Cursor Right
    "Cursor Left",    // 0x50 Cursor Left
    "Cursor Down",    // 0x51 Cursor Down
    "Cursor Up",      // 0x52 Cursor Up
    "Numlock",        // 0x53 Numlock
    "KP /",           // 0x54 Keypad Divide "/"
    "KP *",           // 0x55 Keypad Multiply "*"
    "KP -",           // 0x56 Keypad Minus "-"
    "KP _",           // 0x57 Keypad Plus "+"
    "KP Enter",       // 0x58 Keypad Enter
    "KP 1",           // 0x59 Keypad "1"
    "KP 2",           // 0x5a Keypad "2"
    "KP 3",           // 0x5b Keypad "3"
    "KP 4",           // 0x5c Keypad "4"
    "KP 5",           // 0x5d Keypad "5"
    "KP 6",           // 0x5e Keypad "6"
    "KP 7",           // 0x5f Keypad "7"
    "KP 8",           // 0x60 Keypad "8"
    "KP 9",           // 0x61 Keypad "9"
    "KP 0",           // 0x62 Keypad "0"
    "KP .",           // 0x63 Keypad "."
    "Backslash",      // 0x64 Backslash
    "Menu",           // 0x65 Menu (aka Application / Compose)
    "Power",          // 0x66 Power (AmigaPCI)
    "KP =",           // 0x67 Keypad "="
    "F13",            // 0x68 F13
    "F14",            // 0x69 F14
    "F15",            // 0x6a F15
    "F16",            // 0x6b F16
    "F17",            // 0x6c F17
    "F18",            // 0x6d F18
    "F19",            // 0x6e F19
    "F20",            // 0x6f F20
    "F21",            // 0x70 F21
    "F22",            // 0x71 F22
    "F23",            // 0x72 F23
    "F24",            // 0x73 F24
    "Open",           // 0x74 Open
    "Help",           // 0x75 Help
    "Props",          // 0x76 Props
    "Front",          // 0x77 Front
    "Stop",           // 0x78 Stop
    "Again",          // 0x79 Again
    "Undo",           // 0x7a Undo
    "Cut",            // 0x7b Cut
    "Copy",           // 0x7c Copy
    "Paste",          // 0x7d Paste
    "Find",           // 0x7e Find
    "Mute",           // 0x7f Mute
    "Volume Up",      // 0x80 Volume Up
    "Volume Down",    // 0x81 Volume Down
    "Locking Caps",   // 0x82 Locking Caps Lock
    "Locking Num",    // 0x83 Locking Num Lock
    "Locking Scroll", // 0x84 Locking Scroll Lock
    "KP ,",           // 0x85 Keypad Comma ","
    "KP =",           // 0x86 Keypad Equal "="
    "INTL1",          // 0x87 International1 RO
    "INTL2",          // 0x88 International2 Katakana Hiragana
    "INTL3",          // 0x89 International3 Yen
    "INTL4",          // 0x8a International4 Henkan
    "INTL5",          // 0x8b International5 Muhenkan
    "INTL6",          // 0x8c International6 Keypad JP Comma
    "INTL7",          // 0x8d International7
    "INTL8",          // 0x8e International8
    "INTL9",          // 0x8f International9
    "LANG1",          // 0x90 LANG1 Hangeul
    "LANG2",          // 0x91 LANG2 Hanja
    "LANG3",          // 0x92 LANG3 Katakana
    "LANG4",          // 0x93 LANG4 Hiragana
    "LANG5",          // 0x94 LANG5 Zenkakuhankaku
    "LANG6",          // 0x95 LANG6
    "LANG7",          // 0x96 LANG7
    "LANG8",          // 0x97 LANG8
    "LANG9",          // 0x98 LANG9
    "AltErase",       // 0x99 Alternate Erase
    "SysReq",         // 0x9a SysReq / Attention
    "Cancel",         // 0x9b Cancel
    "Clear",          // 0x9c Clear
    "Prior",          // 0x9d Prior
    "Return",         // 0x9e Return
    "Separator",      // 0x9f Separator
    "Out",            // 0xa0 Out
    "Oper",           // 0xa1 Oper
    "ClearAgain",     // 0xa2 Clear / Again
    "CrSel",          // 0xa3 CrSel / Props
    "ExSel",          // 0xa4 ExSel
    "",               // 0xa5
    "",               // 0xa6
    "",               // 0xa7
    "",               // 0xa8
    "",               // 0xa9
    "",               // 0xaa
    "",               // 0xab
    "",               // 0xac
    "",               // 0xad
    "",               // 0xae
    "",               // 0xaf
    "KP 00",          // 0xb0 Keypad 00
    "KP 000",         // 0xb1 Keypad 000
    "Thousands",      // 0xb2 Thousands Separator
    "Decimal",        // 0xb3 Decimal Separator
    "Currency",       // 0xb4 Currency Unit
    "CurrencySub",    // 0xb5 Currency Sub-unit
    "KP (",           // 0xb6 Keypad Open Paren "("
    "KP )",           // 0xb7 Keypad Close Paren ")"
    "KP {",           // 0xb8 Keypad Open Brace "{"
    "KP }",           // 0xb9 Keypad Close Brace "}"
    "Tab",            // 0xba Tab
    "Backspace",      // 0xbb Backspace
    "KP A",           // 0xbc Keypad A
    "KP B",           // 0xbd Keypad B
    "KP C",           // 0xbe Keypad C
    "KP D",           // 0xbf Keypad D
    "KP E",           // 0xc0 Keypad E
    "KP F",           // 0xc1 Keypad F
    "KP ^",           // 0xc2 Keypad Caret
    "KP XOR",         // 0xc3 Keypad XOR
    "KP %",           // 0xc4 Keypad Percent
    "KP <",           // 0xc5 Keypad Less
    "KP >",           // 0xc6 Keypad Greater
    "KP &",           // 0xc7 Keypad &
    "KP &&",          // 0xc8 Keypad &&
    "KP |",           // 0xc9 Keypad |
    "KP ||",          // 0xca Keypad ||
    "KP :",           // 0xcb Keypad ':'
    "KP #",           // 0xcc Keypad '#'
    "KP Space",       // 0xcd Keypad Space
    "KP @",           // 0xce Keypad '@'
    "KP !",           // 0xcf Keypad '!'
    "KP MStore",      // 0xd0 Keypad Memory Store
    "KP MRecall",     // 0xd1 Keypad Memory Recall
    "KP MClear",      // 0xd2 Keypad Memory Clear
    "KP M+",          // 0xd3 Keypad Memory Add
    "KP M-",          // 0xd4 Keypad Memory Subtract
    "KP M*",          // 0xd5 Keypad Memory Multiply
    "KP M/",          // 0xd6 Keypad Memory Divide
    "KP =-",          // 0xd7 Keypad +/-
    "KP Clear",       // 0xd8 Keypad Clear
    "KP ClearEnt",    // 0xd9 Keypad Clear Entry
    "KP Binary",      // 0xda Keypad Binary
    "KP Octal",       // 0xdb Keypad Octal
    "KP Decimal",     // 0xdc Keypad Decimal
    "KP Hex",         // 0xdd Keypad Hexadecimal
    "",               // 0xde
    "",               // 0xdf
    "Left Ctrl",      // 0xe0 Left Ctrl
    "Left Shift",     // 0xe1 Left Shift
    "Left Alt",       // 0xe2 Left Alt
    "Left Meta",      // 0xe3 Left Meta
    "Right Ctrl",     // 0xe4 Right Ctrl
    "Right Shift",    // 0xe5 Right Shift
    "Right Alt",      // 0xe6 Right Alt
    "Right Meta",     // 0xe7 Right Meta
    "Play/Pause",     // 0xe8 Media Play/Pause
    "Stop",           // 0xe9 Media Stop
    "Prev Track",     // 0xea Media Prev track
    "Next Track",     // 0xeb Media Next track
    "Eject CD",       // 0xec Media Eject CD
    "MVolume Up",     // 0xed Media Volume Up
    "MVolume Down",   // 0xee Media Volume Down
    "MMute",          // 0xef Media Mute
    "WWW",            // 0xf0 Media WWW
    "MBack",          // 0xf1 Media Back
    "MForward",       // 0xf2 Media Forward
    "MStop",          // 0xf3 Media Stop
    "MFind",          // 0xf4 Media Find
    "MScroll Up",     // 0xf5 Media Scroll Up
    "MScroll Down",   // 0xf6 Media Scroll Down
    "MEdit",          // 0xf7 Media Edit
    "MSleep",         // 0xf8 Media Sleep
    "MCoffee",        // 0xf9 Media Coffee
    "MRefresh",       // 0xfa Media Refresh
    "MCalc",          // 0xfb Media Calc
    "",               // 0xfc
    "",               // 0xfd
    "",               // 0xfe
    "",               // 0xff
};

#define NUM_HID_DIRECTIONS 4   // Boxes JU, JD, JL, JR
#define NUM_HID_BUTTONS    16  // Boxes 0..15
#define NUM_HID_BUTTONS_PLUS_DIRECTIONS (NUM_HID_BUTTONS + NUM_HID_DIRECTIONS)

static key_bbox_t amiga_key_bbox[ARRAY_SIZE(amiga_keypos)];
static key_bbox_t hid_key_bbox[ARRAY_SIZE(hid_keypos)];
static key_bbox_t hid_button_bbox[NUM_HID_BUTTONS_PLUS_DIRECTIONS];
static keysize_t  amiga_keysize[ARRAY_SIZE(amiga_keywidths)];
static keysize_t  hid_keysize[ARRAY_SIZE(hid_keywidths)];
static uint8_t    amiga_scancode_to_capnum[256];
static uint8_t    hid_scancode_to_capnum[256];
static uint8_t    amiga_key_mapped[ARRAY_SIZE(amiga_keypos)];
static uint8_t    hid_key_mapped[ARRAY_SIZE(hid_keypos)];
static uint8_t    hid_button_mapped[NUM_BUTTON_SCANCODES];

static const char cmd_options[] =
    "usage: bec <options>\n"
//  "   capamiga     default to capture Amiga scancodes (-C)\n"
    "   caphid       default to capture HID scancodes (-c)\n"
    "   debug        show debug output (-d)\n"
    "   esc          disable ESC key for program exit (-e)\n"
    "   help         display this command help text (-h)\n"
    "   justlive     Just live keys (not mapped keys) (-j)\n"
    "   load <arg>   load key mappings from bec, default, or a filename\n"
    "   save <arg>   save key mappings to bec, or a filename\n"
    "   mapamiga     Amiga key mapping mode (-M)\n"
    "   maphid       HID key mapping mode (-m)\n"
    "   iso          present ISO style keyboard (-i)\n"
    "";

typedef struct {
    const char *const short_name;
    const char *const long_name;
} long_to_short_t;
static const long_to_short_t long_to_short_main[] = {
    { "-C", "capamiga" },
    { "-c", "caphid" },
    { "-d", "debug" },
    { "-e", "esc" },
    { "-h", "help" },
    { "-i", "iso" },
    { "-j", "justlive" },
    { "-l", "load" },
    { "-M", "mapamiga" },
    { "-m", "maphid" },
    { "-p", "mapped" },
    { "-s", "save" },
};

static void
usage(void)
{
    printf("%s\n\n%s", version + 7, cmd_options);
}

const char *
long_to_short(const char *ptr, const long_to_short_t *ltos, uint ltos_count)
{
    uint cur;

    for (cur = 0; cur < ltos_count; cur++)
        if (strcmp(ptr, ltos[cur].long_name) == 0)
            return (ltos[cur].short_name);
    return (ptr);
}

static char *
strcasestr(const char *haystack, const char *needle)
{
    char ch;
    char sc;
    size_t len;

    if ((ch = *needle++) != 0) {
        ch = tolower((unsigned char) ch);
        len = strlen(needle);
        do {
            do {
                if ((sc = *haystack++) == 0)
                    return (NULL);
            } while ((char)tolower((unsigned char) sc) != ch);
        } while (strncasecmp(haystack, needle, len) != 0);
        haystack--;
    }
    return ((char *) haystack);
}

static void __attribute__((format(__printf__, 1, 2)))
err_printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    (void) vsnprintf(buf, sizeof (buf) - 1, fmt, args);
    va_end(args);
    buf[sizeof (buf) - 1] = '\0';
    printf("%s", buf);
}


#include <proto/intuition.h>
#include <proto/graphics.h>
#include <classes/window.h>
#include <graphics/rastport.h>
#include <intuition/intuition.h>

/* Global pointers to libraries */
struct IntuitionBase *IntuitionBase = NULL;
struct GfxBase       *GfxBase       = NULL;
struct Library       *DiskfontBase  = NULL;
struct Library       *GadToolsBase  = NULL;
struct Library       *AslBase       = NULL;
extern struct ExecBase *DOSBase;
extern struct ExecBase *SysBase;
struct Window *window = NULL;
struct RastPort *rp;

/* Function to open libraries */
static BOOL
OpenAmigaLibraries(void)
{
    /* Use version 39 for 3.x compatibility */
    IntuitionBase = (struct IntuitionBase *)
        OpenLibrary("intuition.library", 37L);
    if (IntuitionBase == NULL) {
        err_printf("Failed to open %s\n", "intuition.library");
        return (FALSE);
    }

    GfxBase = (struct GfxBase *) OpenLibrary("graphics.library", 37L);
    if (GfxBase == NULL) {
        err_printf("Failed to open %s\n", "graphics.library");
        return (FALSE);
    }

    GadToolsBase = OpenLibrary("gadtools.library", 37L);
    if (GadToolsBase == NULL) {
        err_printf("Failed to open %s\n", "gadtools.library");
        return (FALSE);
    }

    AslBase = OpenLibrary("asl.library", 37L);
    if (AslBase == NULL) {
        err_printf("Failed to open %s\n", "asl.library");
        return (FALSE);
    }
    return (TRUE);
}

/* Function to close libraries */
static void
CloseAmigaLibraries(void)
{
    if (AslBase) {
        CloseLibrary(AslBase);
        AslBase = NULL;
    }
    if (GadToolsBase) {
        CloseLibrary(GadToolsBase);
        GadToolsBase = NULL;
    }
    if (GfxBase) {
        CloseLibrary((struct Library *)GfxBase);
        GfxBase = NULL;
    }
    if (IntuitionBase != NULL) {
        CloseLibrary((struct Library *)IntuitionBase);
        IntuitionBase = NULL;
    }
}

/* Function to open the window */
static struct Window *
OpenAWindow(void)
{
    struct Screen screen_data;
    uint screen_height = 240;
    uint screen_width = 640;

    /* Get screen size */
    if (GetScreenData(&screen_data, sizeof (struct Screen),
                      WBENCHSCREEN, NULL)) {
        screen_width  = screen_data.Width;
        screen_height = screen_data.Height;
        win_width = screen_width;
        win_height = screen_height;
        if (win_height > 210)
            win_height = 210;  // Cap default height to non-interlaced NTSC
    }
    return (OpenWindowTags(NULL,
//      WA_Left, 0,
#if 1
//      WA_Top, 14 + 200,
        WA_Top, screen_height - win_height - 4,
#endif
        WA_MinWidth, 40,
        WA_MinHeight, 40,
        WA_MaxWidth, screen_width,
        WA_MaxHeight, screen_height,
        WA_Width, win_width,
        WA_Height, win_height,
        WA_IDCMP, IDCMP_RAWKEY | IDCMP_CLOSEWINDOW | IDCMP_REFRESHWINDOW |
                  IDCMP_NEWSIZE | IDCMP_MOUSEMOVE | IDCMP_MOUSEBUTTONS |
                  IDCMP_MENUPICK | IDCMP_INTUITICKS | IDCMP_INACTIVEWINDOW,
        WA_Flags, WFLG_DRAGBAR | WFLG_CLOSEGADGET | WFLG_DEPTHGADGET |
                  WFLG_SIZEGADGET | WFLG_ACTIVATE | WFLG_REPORTMOUSE |
//                WFLG_GIMMEZEROZERO |
                  WFLG_NOCAREREFRESH,
        WA_Title,       (ULONG) "Becky Key Mapping Tool",
        WA_ScreenTitle, (ULONG) "Becky Key Mapping Tool",
        WA_NewLookMenus, TRUE,
        TAG_DONE));
}

static const struct NewMenu becky_menu[] = { // name key flags mutex userdata
    { NM_TITLE, "File",               NULL, 0,  0, NULL },
    {  NM_ITEM, "Load from file",     "L",  0,  0, NULL },
    {  NM_ITEM, "Save to file",       "S",  0,  0, NULL },
    {  NM_ITEM, NM_BARLABEL,          NULL, 0,  0, NULL }, // Separator bar
    {  NM_ITEM, "About...",           "?",  0,  0, NULL },
    {  NM_ITEM, NM_BARLABEL,          NULL, 0,  0, NULL }, // Separator bar
    {  NM_ITEM, "Quit",               "Q",  0,  0, NULL },
    { NM_TITLE, "BEC",                NULL, 0,  0, NULL },
    {  NM_ITEM, "Load from BEC",      NULL, 0,  0, NULL },
    {  NM_ITEM, "Save to BEC",        NULL, 0,  0, NULL },
    {  NM_ITEM, "Load defaults",      NULL, 0,  0, NULL },
    { NM_TITLE, "Mode",               NULL, 0,  0, NULL },
    {  NM_ITEM, "Amiga scancode",     NULL, CHECKIT | MENUTOGGLE,
                                                        0x03 ^ 1, NULL },
    {  NM_ITEM, "HID scancode",       NULL, CHECKIT | MENUTOGGLE,
                                                        0x03 ^ 2, NULL },
    {  NM_ITEM, NM_BARLABEL,          NULL, 0,  0, NULL }, // Separator bar
    {  NM_ITEM, "Map Amiga to HID",   NULL, CHECKIT | MENUTOGGLE,
                                                        0x78 ^ 0x08, NULL },
    {  NM_ITEM, "Map HID to Amiga",   NULL, CHECKIT | MENUTOGGLE,
                                                        0x78 ^ 0x10, NULL },
    {  NM_ITEM, "Live keys",          NULL, CHECKIT | MENUTOGGLE,
                                                        0x78 ^ 0x20, NULL },
    {  NM_ITEM, "Live keys + mapped", NULL, CHECKIT | MENUTOGGLE,
                                                        0x78 ^ 0x40, NULL },
#define MOUSEJOY
#ifdef MOUSEJOY
    {  NM_ITEM, NM_BARLABEL,          NULL, 0,  0, NULL }, // Separator bar
    {  NM_ITEM, "Mouse buttons",      NULL, CHECKIT | MENUTOGGLE,
                                                        0x300 ^ 0x100, NULL },
    {  NM_ITEM, "Joystick buttons",   NULL, CHECKIT | MENUTOGGLE,
                                                        0x300 ^ 0x200, NULL },
#endif
    { NM_TITLE, "Key",                NULL, 0,  0, NULL },
    {  NM_ITEM, "Custom mapping",     NULL, 0,  0, NULL },
    {  NM_ITEM, "Remove mapping",     NULL, 0,  0, NULL },
    {  NM_ITEM, "Redraw keys",        NULL, 0,  0, NULL },
    {  NM_ITEM, "ANSI layout",        NULL, CHECKIT | MENUTOGGLE, 0, NULL },
    {   NM_END, NULL,                 NULL, 0,  0, NULL }  // End of menu
};
static struct NewMenu *menu;
// Remove mappings
//      If Amiga key selected, will remove all HID references to that key
//      If HID key selected, will remove all Amiga keys referenced by that key
// Custom mapping
//      If Amiga key selected, will open requester which specifies the key
//      and asks for space-separated list of HID key codes, where the default
//      is the current list.
//
//      If HID key selected, will open requester which specifies the key
//      and asks for space-separated list of Amiga key codes, where the default
//      is the current list.

#define MENU_NUM_FILE              0
#define MENU_NUM_BEC               1
#define MENU_NUM_MODE              2
#define MENU_NUM_KEY               3

#define MENU_INDEX_MODE            12
#define MENU_INDEX_KEY             (MENU_INDEX_MODE + 11)

#define MENU_FILE_LOAD             0
#define MENU_FILE_SAVE             1
#define MENU_FILE_ABOUT            3
#define MENU_FILE_QUIT             5

#define MENU_BEC_LOAD              0
#define MENU_BEC_SAVE              1
#define MENU_BEC_DEFAULTS          2

#define MENU_MODE_AMIGA_SCANCODE   0
#define MENU_MODE_HID_SCANCODE     1
#define MENU_MODE_AMIGA_TO_HID     3
#define MENU_MODE_HID_TO_AMIGA     4
#define MENU_MODE_LIVE_KEYS        5
#define MENU_MODE_LIVE_KEYS_MAP    6
#define MENU_MODE_MOUSE_BUTTONS    8
#define MENU_MODE_JOY_BUTTONS      9

#define MENU_KEY_CUSTOM_MAPPING    0
#define MENU_KEY_REMOVE_MAPPINGS   1
#define MENU_KEY_REDRAW_KEYS       2
#define MENU_KEY_ANSI_LAYOUT       3

#define ITEMNUM_AMIGA_SCANCODE (SHIFTMENU(MENU_NUM_MODE) | \
                                SHIFTITEM(MENU_MODE_AMIGA_SCANCODE))
#define ITEMNUM_HID_SCANCODE   (SHIFTMENU(MENU_NUM_MODE) | \
                                SHIFTITEM(MENU_MODE_HID_SCANCODE))
#define ITEMNUM_AMIGA_TO_HID   (SHIFTMENU(MENU_NUM_MODE) | \
                                SHIFTITEM(MENU_MODE_AMIGA_TO_HID))
#define ITEMNUM_HID_TO_AMIGA   (SHIFTMENU(MENU_NUM_MODE) | \
                                SHIFTITEM(MENU_MODE_HID_TO_AMIGA))
#define ITEMNUM_LIVEKEYS       (SHIFTMENU(MENU_NUM_MODE) | \
                                SHIFTITEM(MENU_MODE_LIVE_KEYS))
#define ITEMNUM_LIVEKEYS_MAP   (SHIFTMENU(MENU_NUM_MODE) | \
                                SHIFTITEM(MENU_MODE_LIVE_KEYS_MAP))
#define ITEMNUM_MOUSE_BUTTONS  (SHIFTMENU(MENU_NUM_MODE) | \
                                SHIFTITEM(MENU_MODE_MOUSE_BUTTONS))
#define ITEMNUM_JOY_BUTTONS    (SHIFTMENU(MENU_NUM_MODE) | \
                                SHIFTITEM(MENU_MODE_JOY_BUTTONS))

// STATIC_ASSERT(ARRAY_SIZE(becky_menu) == 20);

static struct Menu *menus = NULL;
static APTR        *visual_info;
static uint8_t      capture_hid_scancodes = 0;  // 0=Amiga, 1=HID
static uint8_t      key_mapping_mode;  // 0=Amiga->HID, 1=HID->Amiga, 2=Rawkeys
#define KEY_MAPPING_MODE_AMIGA_TO_HID    0
#define KEY_MAPPING_MODE_HID_TO_AMIGA    1
#define KEY_MAPPING_MODE_LIVEKEYS        2
#define KEY_MAPPING_MODE_LIVEKEYS_MAPPED 3

static void
create_menu(void)
{
    menu = AllocVec(sizeof (becky_menu), MEMF_PUBLIC);
    if (menu == NULL) {
        err_printf("AllocVec failed\n");
        return;
    }
    memcpy(menu, becky_menu, sizeof (becky_menu));
    if (menu[MENU_INDEX_MODE +
             MENU_MODE_HID_TO_AMIGA].nm_MutualExclude == (0x78 ^ 0x10)) {
        /* Add checkmark to "Amiga scancode" or "HID scancode" menu item */
        if (capture_hid_scancodes == 0)
            menu[MENU_INDEX_MODE + MENU_MODE_AMIGA_SCANCODE].nm_Flags |= CHECKED;
        else
            menu[MENU_INDEX_MODE + MENU_MODE_HID_SCANCODE].nm_Flags |= CHECKED;

        /*
         * Add checkmark to "Amiga to HID" or "HID to Amiga" or "Live keys"
         * menu item.
         */
        if (key_mapping_mode == 0)
            menu[MENU_INDEX_MODE + MENU_MODE_AMIGA_TO_HID].nm_Flags |= CHECKED;
        else if (key_mapping_mode == 1)
            menu[MENU_INDEX_MODE + MENU_MODE_HID_TO_AMIGA].nm_Flags |= CHECKED;
        else if (key_mapping_mode == 2)
            menu[MENU_INDEX_MODE + MENU_MODE_LIVE_KEYS].nm_Flags |= CHECKED;
        else
            menu[MENU_INDEX_MODE + MENU_MODE_LIVE_KEYS_MAP].nm_Flags |= CHECKED;

        /*
         * Add checkmark to "Mouse buttons" or "Joystick buttons" menu item.
         */
        if (button_mapping_mode == BUTMAP_MODE_MOUSE)
            menu[MENU_INDEX_MODE + MENU_MODE_MOUSE_BUTTONS].nm_Flags |= CHECKED;
        else
            menu[MENU_INDEX_MODE + MENU_MODE_JOY_BUTTONS].nm_Flags |= CHECKED;

        /* Add checkmark to "ANSI Layout" menu item */
        if (is_ansi_layout)
            menu[MENU_INDEX_KEY + MENU_KEY_ANSI_LAYOUT].nm_Flags |= CHECKED;
    } else {
        err_printf("Bug: becky_menu changed\n");
    }

    visual_info = GetVisualInfoA(window->WScreen, NULL);
    if (visual_info != NULL) {
        menus = CreateMenus(menu,
                            GTMN_FullMenu, TRUE,
                            TAG_END);
        if (menus == NULL) {
            err_printf("CreateMenus failed\n");
        } else {
            if (LayoutMenus(menus, visual_info, GTMN_NewLookMenus,
                            TRUE, TAG_END) == FALSE) {
                err_printf("LayoutMenus failed\n");
                FreeMenus(menus);
                menus = NULL;
                return;
            }
            if (SetMenuStrip(window, menus) == FALSE) {
                err_printf("SetMenuStrip failed\n");
                FreeMenus(menus);
                menus = NULL;
            }
        }
    }
}

static void
close_menu(void)
{
    if (menus != NULL) {
        ClearMenuStrip(window);
        FreeMenus(menus);
        menus = NULL;
    }
    if (visual_info != NULL)
        FreeVisualInfo(visual_info);
    FreeVec(menu);
}

static void
scale_key_dimensions(void)
{
    uint cur;
    uint mul_x = kbd_width;
    uint mul_y = kbd_height;  // Need space for two keyboards
    uint div_x = 24 * U * 2;  // Room for about 24 columns of keys
    uint div_y = 8 * U * 2;   // Room for about 8 rows of keys
    /* 2 above is because the dimensions are +/- pixels from center */

    for (cur = 0; cur < ARRAY_SIZE(amiga_keywidths); cur++) {
        uint16_t key_x = amiga_keywidths[cur].x;
        uint16_t key_y = amiga_keywidths[cur].y;
        amiga_keysize[cur].x = key_x * mul_x / div_x;
        amiga_keysize[cur].y = key_y * mul_y / div_y;
        amiga_keysize[cur].shaded = amiga_keywidths[cur].shaded;
    }

    for (cur = 0; cur < ARRAY_SIZE(hid_keywidths); cur++) {
        hid_keysize[cur].x = hid_keywidths[cur].x * mul_x / div_x;
        hid_keysize[cur].y = hid_keywidths[cur].y * mul_y / div_y;
        hid_keysize[cur].shaded = hid_keywidths[cur].shaded;
    }

    /*
     * For ANSI (North American) keyboards, need to remove Extra1 and Extra2
     * and make left shift key wider. Rendered will need to deal with
     * non-square Return key.
     */
    if (is_ansi_layout) {
        amiga_keysize[11].shaded = KEY_NOT_PRESENT;  // Extra keys = invisible
        amiga_keysize[6].x += 1 * U * mul_x / div_x;  // Left shitt
    }
}

static void
box(SHORT xmin, SHORT ymin, SHORT xmax, SHORT ymax)
{
    Move(rp, xmin, ymin);
    Draw(rp, xmax, ymin);
    Draw(rp, xmax, ymax);
    Draw(rp, xmin, ymax);
    Draw(rp, xmin, ymin);
}

static SHORT amiga_enter_wxlower;
static SHORT amiga_enter_ymid;

static void
box_enterkey_iso(SHORT xpos, SHORT ypos, SHORT wx, SHORT wy)
{
    SHORT ymin  = ypos - wy;
    SHORT ymax  = ypos + wy;
    SHORT xmax  = xpos + wx;
    SHORT xtop    = xpos - wx;
    SHORT xbottom = xpos - amiga_enter_wxlower;

    Move(rp, xtop, ymin);
    Draw(rp, xmax, ymin);
    Draw(rp, xmax, ymax);
    Draw(rp, xbottom, ymax);
    Draw(rp, xbottom, amiga_enter_ymid);
    Draw(rp, xtop, amiga_enter_ymid);
    Draw(rp, xtop, ymin);
}

static void
box_enterkey_ansi(SHORT xpos, SHORT ypos, SHORT wx, SHORT wy)
{
    SHORT ymin  = ypos - wy;
    SHORT ymax  = ypos + wy;
    SHORT xmax  = xpos + wx;
    SHORT xmin  = xpos - amiga_enter_wxlower;

    Move(rp, xpos - wx, ymin);
    Draw(rp, xmax, ymin);
    Draw(rp, xmax, ymax);
    Draw(rp, xmin, ymax);
    Draw(rp, xmin, amiga_enter_ymid);
    Draw(rp, xpos - wx, amiga_enter_ymid);
    Draw(rp, xpos - wx, ymin);
}

static struct TextFont *keycap_font = NULL;

/* Pen colors */
static BYTE pen_cap_white;             // White key cap
static BYTE pen_cap_shaded;            // Shaded key cap
static BYTE pen_cap_pressed;           // Pressed key cap
static BYTE pen_cap_text;              // Keycap text
static BYTE pen_cap_text_pressed;      // Keycap text when pressed
static BYTE pen_cap_outline_lo;        // Normal outline around cap
static BYTE pen_cap_outline_hi;        // Highlighted outline around cap
static BYTE pen_keyboard_case;         // Keyboard case color
static BYTE pen_status_fg;             // Black
static BYTE pen_status_bg;             // White

static void
select_pens(void)
{
    switch (rp->BitMap->Depth) {
        case 1:
            /*
             * Two colors available:
             *    0 = Black
             *    1 = White
             */
            pen_cap_white           = 0;  // White
            pen_cap_shaded          = 0;  // White
            pen_cap_pressed         = 1;  // Black
            pen_cap_text            = 1;  // Black
            pen_cap_text_pressed    = 0;  // White
            pen_cap_outline_lo      = 1;  // White
            pen_cap_outline_hi      = 0;  // Black
            pen_keyboard_case       = 0;  // Background color
            pen_status_fg           = 1;  // Black
            pen_status_bg           = 0;  // White
            break;
        case 0:
        case 2:
            /*
             * Four colors available. Workbench default:
             *    0 = Gray
             *    1 = White
             *    2 = Black
             *    3 = Light Blue
             */
            pen_cap_white           = 2;  // White
            pen_cap_shaded          = 0;  // Background color
            pen_cap_pressed         = 3;  // Blue
            pen_cap_text            = 1;  // Black
            pen_cap_text_pressed    = 1;  // Black
            pen_cap_outline_lo      = 1;  // Black
            pen_cap_outline_hi      = 3;  // Blue
            pen_keyboard_case       = 0;  // Background color
            pen_status_fg           = 1;  // Black
            pen_status_bg           = 2;  // White
            break;
        default:
            /*
             * Eight or more colors available. Workbench default:
             *    0 = Gray
             *    1 = Black
             *    2 = White
             *    3 = Blue
             *    4 = Red
             *    5 = Green
             *    6 = Dark Blue
             *    7 = Orange
             */
            pen_cap_white           = 2;  // White
            pen_cap_shaded          = 0;  // Background color
            pen_cap_pressed         = 3;  // Blue
            pen_cap_text            = 1;  // Black
            pen_cap_text_pressed    = 1;  // Black
            pen_cap_outline_lo      = 1;  // Black
            pen_cap_outline_hi      = 4;  // Red
            pen_keyboard_case       = 6;  // Dark Blue
            pen_status_fg           = 1;  // Black
            pen_status_bg           = 2;  // White
            break;
    }
}

struct TextAttr font_attr = {
    "topaz.font",
    8,
    FS_NORMAL,
    FPF_ROMFONT
};
static void
open_font(void)
{
    DiskfontBase = OpenLibrary("diskfont.library", 0);
    if (DiskfontBase == NULL)
        return;

    keycap_font = OpenDiskFont(&font_attr);
    if (keycap_font) {
        SetFont(rp, keycap_font); // Set the new font
    } else {
        err_printf("Could not open topaz.font 8; using default\n");
    }
    CloseLibrary(DiskfontBase);
    DiskfontBase = NULL;
}

static void
close_font(void)
{
    if (keycap_font != NULL)
        CloseFont(keycap_font);
}

static void
center_text(SHORT pos_x, SHORT pos_y, SHORT max_x, const char *str)
{
    struct TextFont *tf = rp->Font;
    SHORT height = tf->tf_YSize;
    SHORT width = 0;
    uint len;
    char buf[8];

    /* Caution: str might not be a string! */
    if ((uintptr_t) str < 0x100) {
        /* Single character */
        buf[0] = (uintptr_t) str;
        buf[1] = '\0';
        str = buf;
    }
    len = strlen(str);
    width = TextLength(rp, str, len);
    while ((len > 1) && (width >= max_x + 2)) {
        len--;
        width = TextLength(rp, str, len);
    }
    Move(rp, pos_x - width / 2, pos_y + height / 2 - 1);
    Text(rp, str, len);
}

static void __attribute__((format(__printf__, 1, 2)))
gui_printf(const char *fmt, ...)
{
    static SHORT width_last;
    SHORT pos_x = window->BorderLeft + 4;
    SHORT pos_y = gui_print_line1_y;
    SHORT width;
    uint len;
    char buf[80];
    char *ptr;
    va_list args;
    va_start(args, fmt);
    (void) vsnprintf(buf, sizeof (buf) - 1, fmt, args);
    va_end(args);
    buf[sizeof (buf) - 1] = '\0';
    if (gui_initialized == 0) {
        printf("%s\n", buf);
        return;
    }
    for (ptr = buf; *ptr != '\0'; ptr++)
        if (*ptr != ' ')
            break;
    len = strlen(ptr);
    SetAPen(rp, pen_status_fg);
    SetBPen(rp, pen_status_bg);
    if (len > 0) {
        Move(rp, pos_x, pos_y);
        Text(rp, ptr, len);
        width = TextLength(rp, ptr, len);
    } else {
        width = 0;
    }
    if (width_last > width) {
        SetAPen(rp, 0);
        RectFill(rp, pos_x + width, pos_y - font_pixels_y + 1,
                 pos_x + width_last, pos_y + 1);
    }
    width_last = width;
}

static uint8_t set_line2 = 0;
static void __attribute__((format(__printf__, 1, 2)))
gui_printf2(const char *fmt, ...)
{
    static SHORT width_last;
    SHORT pos_x = window->BorderLeft + 4;
    SHORT pos_y = gui_print_line2_y;
    SHORT width;
    uint len;
    char buf[80];
    char *ptr;
    va_list args;
    va_start(args, fmt);
    (void) vsnprintf(buf, sizeof (buf) - 1, fmt, args);
    va_end(args);
    buf[sizeof (buf) - 1] = '\0';
    if (gui_initialized == 0) {
        printf("%s\n", buf);
        return;
    }
    for (ptr = buf; *ptr != '\0'; ptr++)
        if (*ptr != ' ')
            break;
    len = strlen(ptr);
    SetAPen(rp, pen_status_fg);
    SetBPen(rp, pen_status_bg);
    Move(rp, pos_x, pos_y);
    Text(rp, ptr, len);
    width = TextLength(rp, ptr, len);
    if (width_last > width) {
        SetAPen(rp, 0);
        RectFill(rp, pos_x + width, pos_y - font_pixels_y + 1,
                 pos_x + width_last, pos_y + 1);
    }
    width_last = width;
}


static void
user_usage_hint_show(uint from_menu)
{
    switch (key_mapping_mode) {
        case KEY_MAPPING_MODE_AMIGA_TO_HID:
            gui_printf2("Pick Amiga key above and HID key(s) below.");
            break;
        case KEY_MAPPING_MODE_HID_TO_AMIGA:
            gui_printf2("Pick HID key below and up to four Amiga keys above.");
            break;
        case KEY_MAPPING_MODE_LIVEKEYS:
        case KEY_MAPPING_MODE_LIVEKEYS_MAPPED:
            if (from_menu)
                return;  // Not this hint again
            gui_printf2("Select Mode / Map HID to Amiga to remap keys.");
            break;
    }
    set_line2 = 1;
}

static void
handle_esc_key(uint saw_esc)
{
    static uint8_t warned_esc = 0;
    if (saw_esc) {
        if (warned_esc++ == 0) {
            gui_printf2("Press ESC again to exit");
        } else {
            gui_printf2("Exiting");
            program_done = TRUE;
        }
        set_line2 = 1;
    } else if (warned_esc) {
        if (set_line2) {
            set_line2 = 0;
            gui_printf2("");
        }
        warned_esc = 0;
    }
}


static void
user_usage_hint_clear(void)
{
    if (set_line2) {
        gui_printf2("");
        set_line2 = 1;
    }
}

static uint8_t
hid_button_capnum_to_scancode(uint capnum)
{
    uint8_t scancode;

    if (capnum == 0xff)
        return (0xff);

    if (capnum >= NUM_HID_BUTTONS) {
        /* Joystick UDLR is at end of mouse buttons */
        scancode = capnum - NUM_HID_BUTTONS + (32 - 4);
    } else if (button_mapping_mode == BUTMAP_MODE_JOYSTICK) {
        scancode = capnum + 0x20;
    } else {
        scancode = capnum;
    }
    return (scancode);
}


static void
draw_amiga_key(uint cur, uint pressed)
{
    const keypos_t *ke = &amiga_keypos[cur];
    uint8_t scancode = amiga_keypos[cur].scancode;
    uint ktype = amiga_keypos[cur].type;
    uint pos_x;
    uint pos_y;
    uint ke_x = ke->x;
    uint ke_y = ke->y;
    uint wx = amiga_keysize[ktype].x;
    uint wy = amiga_keysize[ktype].y;
    uint shaded = amiga_keysize[amiga_keypos[cur].type].shaded;
    uint keycap_fg_pen;
    uint keycap_bg_pen;

    if (cur > ARRAY_SIZE(amiga_keypos)) {
        err_printf("bug: draw_amiga_key(%u,%u)\n", cur, pressed);
        return;
    }
    if (is_ansi_layout && (scancode == AS_LEFTSHIFT)) {
        ke_x += 1905 / 2;  // Increase width of ANSI left shift
    }
    if (amiga_keywidths[ktype].y > (U + U / 2)) {
        ke_y += 1905 / 2;  // Fixup center for tall keys
    }

    pos_x = (ke_x - AKB_MIN_X) * kbd_keyarea_width /
            (AKB_MAX_X - AKB_MIN_X) + amiga_keyboard_left;
    pos_y = (ke_y - AKB_MIN_Y) * kbd_keyarea_height /
            (AKB_MAX_Y - AKB_MIN_Y) + amiga_keyboard_top;

    if (shaded == KEY_NOT_PRESENT)
        return;  // Key not present in this keymap

    if (pressed) {
        shaded = KEY_PRESSED;
    } else if (!amiga_key_mapped[cur]) {
        shaded = KEY_NOT_MAPPED;
    } else {
        shaded = amiga_keysize[amiga_keypos[cur].type].shaded;
    }

    switch (shaded) {
        default:
        case KEY_PLAIN:  // Normal white key
            keycap_fg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            keycap_bg_pen = pen_cap_white;
            break;
        case KEY_SHADED:  // Normal shaded key
            keycap_fg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            keycap_bg_pen = pen_cap_shaded;
            break;
        case KEY_NOT_MAPPED:  // Key that has not been mapped (Black)
            keycap_fg_pen = pen_cap_white;
            keycap_bg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            break;
        case KEY_PRESSED:  // Key that has been pressed (Blue)
            keycap_fg_pen = pen_cap_text_pressed;
            keycap_bg_pen = pen_cap_pressed;
            break;
    }

    SetAPen(rp, keycap_bg_pen);
    if (scancode == AS_ENTER) {
        /* Special rendering for non-rectangular Enter */
        int text_off = 0;
        if (is_ansi_layout) {
            amiga_enter_ymid = pos_y + wy * 10 / 100;
            amiga_enter_wxlower = wx * 205 / 100;
            RectFill(rp, pos_x - wx, pos_y - wy, pos_x + wx, pos_y + wy);
            RectFill(rp, pos_x - amiga_enter_wxlower, amiga_enter_ymid,
                     pos_x, pos_y + wy);
            SetAPen(rp, pen_cap_outline_lo);
            box_enterkey_ansi(pos_x, pos_y, wx, wy);
        } else {
            amiga_enter_ymid = pos_y - wy * 18 / 100;
            amiga_enter_wxlower = wx * 65 / 100;
            RectFill(rp, pos_x - wx, pos_y - wy, pos_x + wx, amiga_enter_ymid);
            RectFill(rp, pos_x - amiga_enter_wxlower, amiga_enter_ymid,
                     pos_x + wx, pos_y + wy);
            SetAPen(rp, pen_cap_outline_lo);
            box_enterkey_iso(pos_x, pos_y, wx, wy);
            text_off = 2;
        }
        SetAPen(rp, keycap_fg_pen);
        SetBPen(rp, keycap_bg_pen);
        center_text(pos_x + text_off, pos_y,
                    wx * 2 - text_off, amiga_keypos[cur].name);
    } else {
        RectFill(rp, pos_x - wx, pos_y - wy, pos_x + wx, pos_y + wy);
        SetAPen(rp, keycap_fg_pen);
        SetBPen(rp, keycap_bg_pen);
        center_text(pos_x, pos_y, wx * 2, amiga_keypos[cur].name);
        SetAPen(rp, pen_cap_outline_lo);
        box(pos_x - wx, pos_y - wy, pos_x + wx, pos_y + wy);
    }
    amiga_key_bbox[cur].x_min = pos_x - wx;
    amiga_key_bbox[cur].y_min = pos_y - wy;
    amiga_key_bbox[cur].x_max = pos_x + wx;
    amiga_key_bbox[cur].y_max = pos_y + wy;
}

static void
draw_hid_key(uint cur, uint pressed)
{
    const keypos_t *ke = &hid_keypos[cur];
    uint shaded = hid_keysize[hid_keypos[cur].type].shaded;
    uint pos_x;
    uint pos_y;
    uint ktype = hid_keypos[cur].type;
    uint wx = hid_keysize[ktype].x;
    uint wy = hid_keysize[ktype].y;
    uint keycap_fg_pen;
    uint keycap_bg_pen;

    if (cur > ARRAY_SIZE(hid_keypos)) {
        err_printf("bug: draw_hid_key(%u,%u)\n", cur, pressed);
        return;
    }

    if (shaded == KEY_NOT_PRESENT)
        return;  // Key not present in this keymap

    pos_x = (ke->x - HIDKB_MIN_X) * kbd_keyarea_width /
            (HIDKB_MAX_X - HIDKB_MIN_X) + hid_keyboard_left;
    pos_y = (ke->y - HIDKB_MIN_Y) * kbd_keyarea_height /
            (HIDKB_MAX_Y - HIDKB_MIN_Y) +
            hid_keyboard_top + hid_keysize[0].y * 2;

    if (pressed) {
        shaded = KEY_PRESSED;
    } else if (!hid_key_mapped[cur]) {
        shaded = KEY_NOT_MAPPED;
    } else {
        shaded = hid_keysize[hid_keypos[cur].type].shaded;
    }

    switch (shaded) {
        default:
        case KEY_PLAIN:  // Normal white key
            keycap_fg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            keycap_bg_pen = pen_cap_white;
            break;
        case KEY_SHADED:  // Normal shaded key
            keycap_fg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            keycap_bg_pen = pen_cap_shaded;
            break;
        case KEY_NOT_MAPPED:  // Key that has not been mapped (Black)
            keycap_fg_pen = pen_cap_white;
            keycap_bg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            break;
        case KEY_PRESSED:  // Key that has been pressed (Blue)
            keycap_fg_pen = pen_cap_text_pressed;
            keycap_bg_pen = pen_cap_pressed;
            break;
    }

    SetAPen(rp, keycap_bg_pen);
    RectFill(rp, pos_x - wx, pos_y - wy, pos_x + wx, pos_y + wy);
    SetAPen(rp, keycap_fg_pen);
    SetBPen(rp, keycap_bg_pen);
    center_text(pos_x, pos_y, wx * 2, hid_keypos[cur].name);
    SetAPen(rp, pen_cap_outline_lo);
    box(pos_x - wx, pos_y - wy, pos_x + wx, pos_y + wy);
    hid_key_bbox[cur].x_min = pos_x - wx;
    hid_key_bbox[cur].y_min = pos_y - wy;
    hid_key_bbox[cur].x_max = pos_x + wx;
    hid_key_bbox[cur].y_max = pos_y + wy;
}

static void
draw_hid_button(uint cur, uint pressed)
{
    uint pos_x;
    uint pos_y;
    uint wx = amiga_keysize[13].x;
    uint wy = amiga_keysize[13].y;
    uint kbd_buttonarea_width = kbd_keyarea_width * 29 / 40;
    uint shaded = 0;
    uint keycap_fg_pen;
    uint keycap_bg_pen;
    char strbuf[8];
    uint hid_scancode = hid_button_capnum_to_scancode(cur);

    pos_x = cur * kbd_buttonarea_width / NUM_HID_BUTTONS_PLUS_DIRECTIONS +
            hid_keyboard_left;
    pos_y = hid_keyboard_top - hid_keysize[0].y - 2;
    wx    = kbd_buttonarea_width / NUM_HID_BUTTONS_PLUS_DIRECTIONS * 15 / 32;

    if (pressed) {
        shaded = KEY_PRESSED;
    } else if (!hid_button_mapped[hid_scancode]) {
        shaded = KEY_NOT_MAPPED;
    } else {
        shaded = 0;
    }

    switch (shaded) {
        default:
        case KEY_PLAIN:  // Normal white key
            keycap_fg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            keycap_bg_pen = pen_cap_white;
            break;
        case KEY_SHADED:  // Normal shaded key
            keycap_fg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            keycap_bg_pen = pen_cap_shaded;
            break;
        case KEY_NOT_MAPPED:  // Key that has not been mapped (Black)
            keycap_fg_pen = pen_cap_white;
            keycap_bg_pen = pressed ? pen_cap_text_pressed : pen_cap_text;
            break;
        case KEY_PRESSED:  // Key that has been pressed (Blue)
            keycap_fg_pen = pen_cap_text_pressed;
            keycap_bg_pen = pen_cap_pressed;
            break;
    }
    if (cur < NUM_HID_BUTTONS) {
        sprintf(strbuf, "%u", cur + 1);
    } else {
        strbuf[0] = 'J';
        strbuf[1] = "UDLR"[cur - NUM_HID_BUTTONS];
        strbuf[2] = '\0';
        pos_x += 6;
    }

    SetAPen(rp, keycap_bg_pen);
    RectFill(rp, pos_x - wx, pos_y - wy, pos_x + wx, pos_y + wy);
    SetAPen(rp, keycap_fg_pen);
    SetBPen(rp, keycap_bg_pen);
    center_text(pos_x, pos_y, wx * 2, strbuf);
    SetAPen(rp, pen_cap_outline_lo);
    box(pos_x - wx, pos_y - wy, pos_x + wx, pos_y + wy);

    hid_button_bbox[cur].x_min = pos_x - wx;
    hid_button_bbox[cur].y_min = pos_y - wy;
    hid_button_bbox[cur].x_max = pos_x + wx;
    hid_button_bbox[cur].y_max = pos_y + wy;
}

#define NUM_EDITBOX_ROWS 3
static char    editbox[NUM_EDITBOX_ROWS][20];
static uint8_t editbox_max_len[NUM_EDITBOX_ROWS];
static uint8_t editbox_cursor_x;
static uint8_t editbox_cursor_y;
static SHORT   editbox_x_min;
static SHORT   editbox_x_max[NUM_EDITBOX_ROWS];
static SHORT   editbox_y_min[NUM_EDITBOX_ROWS];
static SHORT   editbox_y_max[NUM_EDITBOX_ROWS];
static uint8_t editbox_update_mode;  // 0=Amiga Key, 1=HID Buttons, 2=HID Key
static uint8_t editbox_scancode;     // Either Amiga scancode or HID scancode


static void
draw_amiga_key_box(uint cur, uint do_mark)
{
    uint8_t scancode = amiga_keypos[cur].scancode;
    SetAPen(rp, do_mark ? pen_cap_outline_hi : pen_cap_outline_lo);
    if (scancode == AS_ENTER) {
        /* Special rendering for non-rectangular Enter */
        uint ktype = amiga_keypos[cur].type;
        uint wx = amiga_keysize[ktype].x;
        uint wy = amiga_keysize[ktype].y;

        if (is_ansi_layout) {
            box_enterkey_ansi(amiga_key_bbox[cur].x_min + wx,
                              amiga_key_bbox[cur].y_min + wy, wx, wy);
        } else {
            box_enterkey_iso(amiga_key_bbox[cur].x_min + wx,
                             amiga_key_bbox[cur].y_min + wy, wx, wy);
        }
    } else {
        box(amiga_key_bbox[cur].x_min,
            amiga_key_bbox[cur].y_min,
            amiga_key_bbox[cur].x_max,
            amiga_key_bbox[cur].y_max);
    }
}

static void
draw_hid_key_box(uint cur, uint do_mark)
{
    SetAPen(rp, do_mark ? pen_cap_outline_hi : pen_cap_outline_lo);
    box(hid_key_bbox[cur].x_min,
        hid_key_bbox[cur].y_min,
        hid_key_bbox[cur].x_max,
        hid_key_bbox[cur].y_max);
}

static void
draw_hid_button_box(uint cur, uint do_mark)
{
    SetAPen(rp, do_mark ? pen_cap_outline_hi : pen_cap_outline_lo);
    box(hid_button_bbox[cur].x_min,
        hid_button_bbox[cur].y_min,
        hid_button_bbox[cur].x_max,
        hid_button_bbox[cur].y_max);
}

#define MOUSE_CUR_SELECTION_NONE         0
#define MOUSE_CUR_SELECTION_AMIGA        1
#define MOUSE_CUR_SELECTION_HID          2
#define MOUSE_CUR_SELECTION_EDITBOX      3
#define MOUSE_CUR_SELECTION_HID_BUTTON   4

static uint8_t amiga_last_scancode      = 0xff;
static uint8_t hid_last_scancode        = 0xff;
static uint8_t hid_button_last_scancode = 0xff;
static uint8_t mouse_cur_scancode       = 0xff;
static uint8_t mouse_cur_capnum         = 0xff;
static uint8_t mouse_cur_selection      = MOUSE_CUR_SELECTION_NONE;

static void
editbox_draw_box(uint cursor_y)
{
    const uint gap_pixels = 2;
    uint  cur;
    uint  pos;
    SHORT xmin;
    SHORT xmax;
    SHORT ymin;
    SHORT ymax;
    static SHORT last_len[NUM_EDITBOX_ROWS];
    uint len = editbox_max_len[cursor_y];

    if (len > 12)
        len = 12;
    xmin = editbox_x_min;
    xmax = editbox_x_max[cursor_y];
    ymin = editbox_y_min[cursor_y];
    ymax = ymin + font_pixels_y + 2;
    pos = xmin + 1;

    /* Surrounding box */
    if (len > 0) {
        SetAPen(rp, pen_status_fg);
        box(xmin, ymin, xmax, ymax);
    }

    for (cur = 0; cur < len; cur += 2) {
        /* Fill background above each set of digits */
        SetAPen(rp, pen_status_bg);
        Move(rp, pos, ymin + 1);
        Draw(rp, pos + font_pixels_x * 2 - 1, ymin + 1);

        /* Each set of digits */
        SetAPen(rp, pen_status_fg);
        SetBPen(rp, pen_status_bg);
        Move(rp, pos, ymin + font_pixels_y);
        Text(rp, &editbox[cursor_y][cur], 2);

        pos += font_pixels_x * 2 + gap_pixels;
    }
    if (last_len[cursor_y] > len) {
        /* Cover previous display */
        pos--;
        SetAPen(rp, 0);
        xmax = xmin + font_pixels_x * last_len[cursor_y] +
               ((last_len[cursor_y] / 2) * gap_pixels) + 1;
        RectFill(rp, pos, ymin, xmax, ymax);
    }
    last_len[cursor_y] = len;
}

static void
editbox_update_scancode(uint hid_to_amiga, uint cursor_y,
                        uint8_t *buf, uint len)
{
    uint  cur;
    char *strbuf;

    if (edit_key_mapping_mode)
        return;  // Don't allow updates while editing is taking place

    switch (hid_to_amiga) {
        case 0:
            /* In Amiga to HID mode, there is 1 Amiga box and 6 HID boxes */
            editbox_max_len[cursor_y] = (cursor_y ? 6 : 1) * 2;
            break;
        case 1:
            /* In HID to Amiga mode, there is 1 HID box and 4 Amiga boxes */
            switch (cursor_y) {
                case 0:
                    editbox_max_len[cursor_y] = 4 * 2;
                    break;
                case 1:
                    editbox_max_len[cursor_y] = 1 * 2;
                    break;
                case 2:
                    editbox_max_len[cursor_y] = 0;
                    break;
            }
            break;
        case 2:
            /* In HID to Amiga mode, there is 1 HID box and 4 Amiga boxes */
            switch (cursor_y) {
                case 0:
                    editbox_max_len[cursor_y] = 4 * 2;
                    break;
                case 1:
                    editbox_max_len[cursor_y] = 0;
                    break;
                case 2:
                    editbox_max_len[cursor_y] = 1 * 2;
                    break;
            }
            break;
    }

    for (cur = 0; cur < NUM_EDITBOX_ROWS; cur++) {
        editbox_x_max[cur] = editbox_x_min + 1 +
                             font_pixels_x * editbox_max_len[cur] +
                             ((editbox_max_len[cur] - 1) / 2) * 2;  // gap
    }

    strbuf = editbox[cursor_y];
    for (cur = 0; cur < len; cur++)
        sprintf(strbuf + cur * 2, "%02x", buf[cur]);
    cur *= 2;
    for (; cur < ARRAY_SIZE(editbox[0]); cur++)
        strbuf[cur] = ' ';

    editbox_update_mode = hid_to_amiga;
    editbox_cursor_y = hid_to_amiga;
    editbox_draw_box(cursor_y);
    if (hid_to_amiga == cursor_y) {
        /* The scancode being mapped is always the only entry */
        editbox_scancode = buf[0];
    }
}

static void
editbox_copy_amiga_scancode_mapping(uint8_t amiga_scancode)
{
    uint8_t match_scancode;
    uint    code;
    uint    map;
    uint    hid_buflen = 0;
    uint8_t hid_buf[8];
    uint    button_buflen = 0;
    uint8_t button_buf[8];

    if (amiga_scancode == 0xff)
        return;

    for (code = 0; code < ARRAY_SIZE(hid_scancode_to_amiga); code++) {
        for (map = 0; map < ARRAY_SIZE(hid_scancode_to_amiga[0]); map++) {
            match_scancode = hid_scancode_to_amiga[code][map];
            if (match_scancode == 0xff)
                break;  // End of list
            if (match_scancode == 0) {
                /* Special case for Amiga backtick */
                uint pos = map + 1;
                for (; pos < ARRAY_SIZE(hid_scancode_to_amiga[0]); pos++)
                    if (hid_scancode_to_amiga[code][pos] != 0x00)
                        break;
                if (pos >= ARRAY_SIZE(hid_scancode_to_amiga[0]))
                    continue;
            }
            if (amiga_scancode == match_scancode) {
                if (hid_buflen < sizeof (hid_buf))
                    hid_buf[hid_buflen++] = code;
                break;
            }
        }
    }
    for (code = 0; code < ARRAY_SIZE(hid_button_scancode_to_amiga); code++) {
        for (map = 0; map < ARRAY_SIZE(hid_button_scancode_to_amiga[0]);
             map++) {
            match_scancode = hid_button_scancode_to_amiga[code][map];
            if (match_scancode == 0xff)
                break;  // End of list
            if (match_scancode == 0) {
                /* Special case for Amiga backtick */
                uint pos = map + 1;
                for (; pos < ARRAY_SIZE(hid_button_scancode_to_amiga[0]); pos++)
                    if (hid_button_scancode_to_amiga[code][pos] != 0x00)
                        break;
                if (pos >= ARRAY_SIZE(hid_button_scancode_to_amiga[0]))
                    continue;
            }
            if (amiga_scancode == match_scancode) {
                if (button_buflen < sizeof (button_buf))
                    button_buf[button_buflen++] = code;
                break;
            }
        }
    }

    editbox_update_scancode(0, 0, &amiga_scancode, 1);
    editbox_update_scancode(0, 1, button_buf, button_buflen);
    editbox_update_scancode(0, 2, hid_buf, hid_buflen);
}

static void
editbox_copy_hid_button_scancode_mapping(uint8_t scancode)
{
    editbox_update_scancode(1, 0, hid_button_scancode_to_amiga[scancode],
                            sizeof (hid_button_scancode_to_amiga[0]));
    editbox_update_scancode(1, 1, &scancode, 1);  // No editing
    editbox_update_scancode(1, 2, &scancode, 0);
}

static void
editbox_copy_hid_scancode_mapping(uint8_t hid_scancode)
{
    editbox_update_scancode(2, 0, hid_scancode_to_amiga[hid_scancode],
                            sizeof (hid_scancode_to_amiga[0]));
    editbox_update_scancode(2, 1, &hid_scancode, 0);  // No editing
    editbox_update_scancode(2, 2, &hid_scancode, 1);
}

static void
enter_leave_amiga_scancode(uint8_t amiga_scancode, uint do_mark, uint do_keycap)
{
    uint8_t match_scancode;
    char    printbuf[80];
    char   *printbufptr = printbuf;
    uint    cap;
    uint    code;
    uint    map;
    uint    scanbuflen = 0;
    uint8_t scanbuf[8];
    uint8_t hid_capnum;
    uint    button_count = 0;
    uint    hid_count = 0;

    if (amiga_scancode == 0xff)
        return;  // Not a valid scancode

    cap = amiga_scancode_to_capnum[amiga_scancode];
    if (cap != 0xff) {
        if (do_keycap)
            draw_amiga_key(cap, do_mark);
        else
            draw_amiga_key_box(cap, do_mark);
    }

    for (code = 0; code < ARRAY_SIZE(hid_scancode_to_amiga); code++) {
        for (map = 0; map < ARRAY_SIZE(hid_scancode_to_amiga[0]); map++) {
            match_scancode = hid_scancode_to_amiga[code][map];
            if (match_scancode == 0xff)
                break;  // End of list
            if (match_scancode == 0) {
                /* Special case for Amiga backtick */
                uint pos = map + 1;
                for (; pos < ARRAY_SIZE(hid_scancode_to_amiga[0]); pos++)
                    if (hid_scancode_to_amiga[code][pos] != 0x00)
                        break;
                if (pos >= ARRAY_SIZE(hid_scancode_to_amiga[0]))
                    continue;
            }
            if (amiga_scancode == match_scancode) {
                if (do_mark) {
                    if (scanbuflen < sizeof (scanbuf))
                        scanbuf[scanbuflen++] = code;
                    if (hid_count++ < 6) {
                        sprintf(printbufptr, " %02x", code);
                        printbufptr += strlen(printbufptr);
                    }
                }
                hid_capnum = hid_scancode_to_capnum[code];
                if (hid_capnum != 0xff) {
                    if (do_keycap)
                        draw_hid_key(hid_capnum, do_mark);
                    else
                        draw_hid_key_box(hid_capnum, do_mark);
                }
                break;
            }
        }
    }
    for (code = 0; code < ARRAY_SIZE(hid_button_scancode_to_amiga); code++) {
        for (map = 0; map < ARRAY_SIZE(hid_button_scancode_to_amiga[0]);
             map++) {
            match_scancode = hid_button_scancode_to_amiga[code][map];
            if (match_scancode == 0xff)
                break;  // End of list
            if (match_scancode == 0) {
                /* Special case for Amiga backtick */
                uint pos = map + 1;
                for (; pos < ARRAY_SIZE(hid_button_scancode_to_amiga[0]); pos++)
                    if (hid_button_scancode_to_amiga[code][pos] != 0x00)
                        break;
                if (pos >= ARRAY_SIZE(hid_button_scancode_to_amiga[0]))
                    continue;
            }
            if (amiga_scancode == match_scancode) {
                uint8_t hid_button;
                if (do_mark) {
                    if (scanbuflen < sizeof (scanbuf))
                        scanbuf[scanbuflen++] = code;
                    if (button_count == 0) {
                        sprintf(printbufptr, " Button");
                        printbufptr += strlen(printbufptr);
                    }
                    if (button_count++ < 6) {
                        sprintf(printbufptr, " %02x", code);
                        printbufptr += strlen(printbufptr);
                    }
                }
                hid_button = hid_button_scancode_to_capnum(code);
                if (hid_button != 0xff) {
                    if (do_keycap)
                        draw_hid_button(hid_button, do_mark);
                    else
                        draw_hid_button_box(hid_button, do_mark);
                }
                break;
            }
        }
    }
    if (do_mark) {
        *printbufptr = '\0';
        gui_printf("Amiga %02x <-%s", amiga_scancode, printbuf);
    }
}

static void
enter_leave_hid_scancode(uint8_t hid_scancode, uint do_mark, uint do_keycap)
{
    uint    cap;
    uint    map;
    uint    amigakey;
    char    printbuf[80];
    char   *printbufptr = printbuf;
    uint8_t amiga_scancode;

    if (hid_scancode == 0xff)
        return;  // Not a valid scancode

    cap = hid_scancode_to_capnum[hid_scancode];
    if (cap != 0xff) {
        if (do_keycap)
            draw_hid_key(cap, do_mark);
        else
            draw_hid_key_box(cap, do_mark);
    }

    for (map = 0; map < ARRAY_SIZE(hid_scancode_to_amiga[0]); map++) {
        amiga_scancode = hid_scancode_to_amiga[hid_scancode][map];
        if (amiga_scancode == 0xff)
            break;  // End of list
        if (amiga_scancode == 0x00) {
            /* Special case for Amiga backtick */
            uint pos = map + 1;
            for (; pos < ARRAY_SIZE(hid_scancode_to_amiga[0]); pos++)
                if (hid_scancode_to_amiga[hid_scancode][pos] != 0x00)
                    break;
            if (pos == ARRAY_SIZE(hid_scancode_to_amiga[0]))
                continue;  // End of list
        }
        if (do_mark) {
            sprintf(printbufptr, " %02x", amiga_scancode);
            printbufptr += strlen(printbufptr);
        }
        amigakey = amiga_scancode_to_capnum[amiga_scancode];
        if (amigakey != 0xff) {
            if (do_keycap)
                draw_amiga_key(amigakey, do_mark);
            else
                draw_amiga_key_box(amigakey, do_mark);
        }
    }
    if (do_mark) {
        *printbufptr = '\0';
        gui_printf("HID %02x ->%s", hid_scancode, printbuf);
    }
}

static void
enter_leave_hid_button_scancode(uint8_t scancode, uint do_mark, uint do_keycap)
{
    uint    cap;
    uint    map;
    uint    amigakey;
    char    printbuf[80];
    char   *printbufptr = printbuf;
    uint8_t amiga_scancode;

    if (scancode == 0xff)
        return;  // Not a valid scancode

    cap = hid_button_scancode_to_capnum(scancode);
    if (cap != 0xff) {
        if (do_keycap)
            draw_hid_button(cap, do_mark);
        else
            draw_hid_button_box(cap, do_mark);
    }

    for (map = 0; map < ARRAY_SIZE(hid_button_scancode_to_amiga[0]); map++) {
        amiga_scancode = hid_button_scancode_to_amiga[scancode][map];
        if (amiga_scancode == 0xff)
            break;  // End of list
        if (amiga_scancode == 0x00) {
            /* Special case for Amiga backtick */
            uint pos = map + 1;
            for (; pos < ARRAY_SIZE(hid_scancode_to_amiga[0]); pos++)
                if (hid_button_scancode_to_amiga[scancode][pos] != 0x00)
                    break;
            if (pos == ARRAY_SIZE(hid_scancode_to_amiga[0]))
                continue;  // End of list
        }
        if (do_mark) {
            sprintf(printbufptr, " %02x", amiga_scancode);
            printbufptr += strlen(printbufptr);
        }
        amigakey = amiga_scancode_to_capnum[amiga_scancode];
        if (amigakey != 0xff) {
            if (do_keycap)
                draw_amiga_key(amigakey, do_mark);
            else
                draw_amiga_key_box(amigakey, do_mark);
        }
    }
    if (do_mark) {
        *printbufptr = '\0';
        gui_printf("HID Button %02x ->%s", scancode, printbuf);
    }
}

static uint
hid_button_scancode_to_capnum(uint8_t scancode)
{
    uint capnum = scancode;

    if (scancode == 0xff)
        return (0xff);

    if ((scancode >= 32 - 4) && (scancode < 32)) {
        /* Joystick direction is always visible */
        capnum = scancode - (32 - 4) + NUM_HID_BUTTONS;
    } else if (button_mapping_mode == BUTMAP_MODE_JOYSTICK) {
        if (scancode < 0x20)
            capnum = 0xff;  // Not currently visible
        else
            capnum = scancode - 0x20;
    } else {
        /* Mouse button mapping mode */
        if (scancode >= NUM_HID_BUTTONS)
            capnum = 0xff;  // Not currently visible
        else
            capnum = scancode;
    }
    return (capnum);
}

static void
enter_leave_hid_button(uint8_t scancode, uint do_mark, uint do_keycap)
{
    uint    map;
    uint    amigakey;
    uint    hid_button = hid_button_scancode_to_capnum(scancode);
    char    printbuf[80];
    char   *printbufptr = printbuf;
    uint8_t amiga_scancode;

    if (scancode == 0xff)
        return;

    if (hid_button == 0xff)
        return;  // Not a valid scancode

    if (do_keycap)
        draw_hid_button(hid_button, do_mark);
    else
        draw_hid_button_box(hid_button, do_mark);

    for (map = 0; map < ARRAY_SIZE(hid_button_scancode_to_amiga[0]); map++) {
        amiga_scancode = hid_button_scancode_to_amiga[scancode][map];
        if (amiga_scancode == 0xff)
            break;  // End of list
        if (amiga_scancode == 0x00) {
            /* Special case for Amiga backtick */
            uint pos = map + 1;
            for (; pos < ARRAY_SIZE(hid_scancode_to_amiga[0]); pos++)
                if (hid_button_scancode_to_amiga[scancode][pos] != 0x00)
                    break;
            if (pos == ARRAY_SIZE(hid_button_scancode_to_amiga[0]))
                continue;  // End of list
        }
        if (do_mark) {
            sprintf(printbufptr, " %02x", amiga_scancode);
            printbufptr += strlen(printbufptr);
        }
        amigakey = amiga_scancode_to_capnum[amiga_scancode];
        if (amigakey != 0xff) {
            if (do_keycap)
                draw_amiga_key(amigakey, do_mark);
            else
                draw_amiga_key_box(amigakey, do_mark);
        }
    }

    if (do_mark) {
        uint scancode = hid_button_capnum_to_scancode(hid_button);
        *printbufptr = '\0';
        gui_printf("HID Button %02x ->%s", scancode, printbuf);
    }
}

static void
mouse_move(SHORT x, SHORT y)
{
    uint cur;
    if ((x < 0) || (y < 0))
        return;

    x += window->BorderLeft;
    y += window->BorderTop;

    /* Check Amiga keyboard keys */
    for (cur = 0; cur < ARRAY_SIZE(amiga_key_bbox); cur++) {
        if (amiga_key_bbox[cur].y_max < y)
            continue;
        if (amiga_key_bbox[cur].y_min > y)
            break;  // Not found
        if (amiga_key_bbox[cur].x_max < x)
            continue;
        if (amiga_key_bbox[cur].x_min > x) {
            if ((amiga_keypos[cur].scancode == AS_ENTER) ||
                (amiga_keypos[cur].scancode == AS_KP_ENTER)) {
                continue;  // Key spans multiple rows
            }
            break;  // Not found
        }

        if (amiga_keysize[amiga_keypos[cur].type].shaded == KEY_NOT_PRESENT)
            continue;  // Key not present in this keymap

        /* Inside a box */

        if ((mouse_cur_capnum == cur) &&
            (mouse_cur_selection == MOUSE_CUR_SELECTION_AMIGA))
            return;  // Still in the same box

        if (mouse_cur_capnum != 0xff) {
            /* Redraw original bounding box */
            switch (mouse_cur_selection) {
                case MOUSE_CUR_SELECTION_NONE: // None
                    break;
                case MOUSE_CUR_SELECTION_AMIGA: // Amiga
                    enter_leave_amiga_scancode(mouse_cur_scancode, 0, 0);
                    break;
                case MOUSE_CUR_SELECTION_HID: // HID
                    enter_leave_hid_scancode(mouse_cur_scancode, 0, 0);
                    break;
                case MOUSE_CUR_SELECTION_EDITBOX: // Editbox
                    break;
                case MOUSE_CUR_SELECTION_HID_BUTTON: // HID button
                    enter_leave_hid_button(mouse_cur_scancode, 0, 0);
                    break;
            }
        }

        /* Draw new highlight bounding box */
        mouse_cur_capnum = cur;
        mouse_cur_scancode = amiga_keypos[cur].scancode;
        mouse_cur_selection = MOUSE_CUR_SELECTION_AMIGA;
        enter_leave_amiga_scancode(mouse_cur_scancode, 1, 0);
        return;
    }

    /* Check HID keyboard keys */
    for (cur = 0; cur < ARRAY_SIZE(hid_key_bbox); cur++) {
        if (hid_key_bbox[cur].y_max < y)
            continue;
        if (hid_key_bbox[cur].y_min > y)
            break;  // Not found
        if (hid_key_bbox[cur].x_max < x)
            continue;
        if (hid_key_bbox[cur].x_min > x) {
            if ((hid_keypos[cur].scancode == HS_KP_PLUS) ||
                (hid_keypos[cur].scancode == HS_KP_ENTER))
                continue;  // Key spans multiple rows
            break;  // Not found
        }

        /* Inside a box */
        if ((mouse_cur_capnum == cur) &&
            (mouse_cur_selection == MOUSE_CUR_SELECTION_HID))
            return;  // Still in the same box

        if (mouse_cur_capnum != 0xff) {
            /* Redraw original bounding box */
            switch (mouse_cur_selection) {
                case MOUSE_CUR_SELECTION_NONE: // None
                    break;
                case MOUSE_CUR_SELECTION_AMIGA: // Amiga
                    enter_leave_amiga_scancode(mouse_cur_scancode, 0, 0);
                    break;
                case MOUSE_CUR_SELECTION_HID: // HID
                    enter_leave_hid_scancode(mouse_cur_scancode, 0, 0);
                    break;
                case MOUSE_CUR_SELECTION_EDITBOX: // Editbox
                    break;
                case MOUSE_CUR_SELECTION_HID_BUTTON: // HID button
                    enter_leave_hid_button(mouse_cur_scancode, 0, 0);
                    break;
            }
        }
        /* Draw new highlight bounding box */
        mouse_cur_capnum = cur;
        mouse_cur_scancode = hid_keypos[cur].scancode;
        mouse_cur_selection = MOUSE_CUR_SELECTION_HID;
        enter_leave_hid_scancode(mouse_cur_scancode, 1, 0);
        return;
    }

    /* Check HID buttons */
    for (cur = 0; cur < NUM_HID_BUTTONS_PLUS_DIRECTIONS; cur++) {
        if ((hid_button_bbox[cur].y_max < y) ||
            (hid_button_bbox[cur].y_min > y))
            break;  // Not in range of buttons
        if (hid_button_bbox[cur].x_max < x)
            continue;
        if (hid_button_bbox[cur].x_min > x)
            break;  // Not found

        /* Inside a box */

        if ((mouse_cur_capnum == cur) &&
            (mouse_cur_selection == MOUSE_CUR_SELECTION_HID_BUTTON))
            return;  // Still in the same box

        if (mouse_cur_capnum != 0xff) {
            /* Redraw original bounding box */
            switch (mouse_cur_selection) {
                case MOUSE_CUR_SELECTION_NONE: // None
                    break;
                case MOUSE_CUR_SELECTION_AMIGA: // Amiga
                    enter_leave_amiga_scancode(mouse_cur_scancode, 0, 0);
                    break;
                case MOUSE_CUR_SELECTION_HID: // HID
                    enter_leave_hid_scancode(mouse_cur_scancode, 0, 0);
                    break;
                case MOUSE_CUR_SELECTION_EDITBOX: // Editbox
                    break;
                case MOUSE_CUR_SELECTION_HID_BUTTON: // HID button
                    enter_leave_hid_button(mouse_cur_scancode, 0, 0);
                    break;
            }
        }

        /* Draw new highlight bounding box */
        mouse_cur_capnum = cur;
        mouse_cur_scancode = hid_button_capnum_to_scancode(cur);
        mouse_cur_selection = MOUSE_CUR_SELECTION_HID_BUTTON;
        enter_leave_hid_button(mouse_cur_scancode, 1, 0);
        return;
    }

    /* Check edit area boxes */
    for (cur = 0; cur < ARRAY_SIZE(editbox_y_min); cur++) {
        uint charpos;
        uint capnum;
        if ((y <= editbox_y_min[cur]) || (y >= editbox_y_max[cur]))
            continue;
        if ((x < editbox_x_min) || (x > editbox_x_max[cur]))
            continue;

        /* Inside a box */

        charpos = (x - editbox_x_min) / (font_pixels_x + 1);
        capnum = (cur << 4) | charpos;

        if ((mouse_cur_capnum == capnum) &&
            (mouse_cur_selection == MOUSE_CUR_SELECTION_EDITBOX))
            return;  // Still in the same box

        if (mouse_cur_capnum != 0xff) {
            /* Redraw original bounding box */
            switch (mouse_cur_selection) {
                case MOUSE_CUR_SELECTION_NONE: // None
                    break;
                case MOUSE_CUR_SELECTION_AMIGA: // Amiga
                    enter_leave_amiga_scancode(mouse_cur_scancode, 0, 0);
                    break;
                case MOUSE_CUR_SELECTION_HID: // HID
                    enter_leave_hid_scancode(mouse_cur_scancode, 0, 0);
                    break;
                case MOUSE_CUR_SELECTION_EDITBOX: // Editbox
                    break;
                case MOUSE_CUR_SELECTION_HID_BUTTON: // HID button
                    enter_leave_hid_button(mouse_cur_scancode, 0, 0);
                    break;
            }
        }

        mouse_cur_capnum = capnum;
        gui_printf("%x.%x", mouse_cur_capnum >> 4, mouse_cur_capnum & 0xf);
        mouse_cur_selection = MOUSE_CUR_SELECTION_EDITBOX;
        return;
    }

    gui_printf("");
    /* Mouse is not in a bounding box */
    if (mouse_cur_capnum != 0xff) {
        /* Redraw original bounding box */
        switch (mouse_cur_selection) {
            case MOUSE_CUR_SELECTION_NONE: // None
                break;
            case MOUSE_CUR_SELECTION_AMIGA: // Amiga
                enter_leave_amiga_scancode(mouse_cur_scancode, 0, 0);
                break;
            case MOUSE_CUR_SELECTION_HID: // HID
                enter_leave_hid_scancode(mouse_cur_scancode, 0, 0);
                break;
            case MOUSE_CUR_SELECTION_EDITBOX: // Editbox
                break;
            case MOUSE_CUR_SELECTION_HID_BUTTON: // HID button
                enter_leave_hid_button(mouse_cur_scancode, 0, 0);
                break;
        }
        mouse_cur_capnum = 0xff;
        mouse_cur_scancode = 0xff;
        mouse_cur_selection = MOUSE_CUR_SELECTION_NONE;
    }
}

static void
unhighlight_last_key(void)
{
    /* Find all HID keys which map to the last code */
    if (amiga_last_scancode != 0xff)
        enter_leave_amiga_scancode(amiga_last_scancode, 0, 1);
    if (hid_last_scancode != 0xff)
        enter_leave_hid_scancode(hid_last_scancode, 0, 1);
    if (hid_button_last_scancode != 0xff)
        enter_leave_hid_scancode(hid_button_last_scancode, 0, 1);

    amiga_last_scancode      = 0xff;
    hid_last_scancode        = 0xff;
    hid_button_last_scancode = 0xff;
}

static int
remove_amiga_mapping_from_hid_scancode(uint8_t hid_scancode,
                                       uint8_t amiga_scancode)
{
    uint map;
    uint pos;
    const uint max_map = ARRAY_SIZE(hid_scancode_to_amiga[0]);

    if ((hid_scancode == 0xff) || (amiga_scancode == 0xff))
        return (0);

    /* Check if it's already mapped */
    for (map = 0; map < max_map; map++) {
        /* Check for end of list */
        if (hid_scancode_to_amiga[hid_scancode][map] == 0xff)
            break;  // End of list
        for (pos = map; pos < max_map; pos++)
            if (hid_scancode_to_amiga[hid_scancode][pos] != 0x00)
                break;
        if (pos == max_map)
            break;  // End of list

        if (hid_scancode_to_amiga[hid_scancode][map] != amiga_scancode)
            continue;

        /*
         * Found a mapping; remove it
         *
         * General case:
         *   ww xx yy zz -> ww xx zz 00
         *         ^^
         *
         * Special case:
         *   ww xx 00 ff -> ww xx 00 00
         *         ^^
         * Special case:
         *   ww 00 yy 00 -> ww 00 ff 00
         *         ^^
         */
        uint pos = map;
        for (; pos < max_map - 1; pos++)
            hid_scancode_to_amiga[hid_scancode][pos] =
                hid_scancode_to_amiga[hid_scancode][pos + 1];
        hid_scancode_to_amiga[hid_scancode][max_map - 1] = 0x00;

        uint8_t hid_capnum = hid_scancode_to_capnum[hid_scancode];
        if (hid_capnum != 0xff) {
            if (hid_key_mapped[hid_capnum] > 0)
                hid_key_mapped[hid_capnum]--;
        }

        if ((amiga_scancode == 0x00) &&
            (hid_scancode_to_amiga[hid_scancode][map] == 0xff)) {
            hid_scancode_to_amiga[hid_scancode][map] = 0x00;
        } else if ((map > 0) &&
                   (hid_scancode_to_amiga[hid_scancode][map - 1] == 0x00) &&
                   (hid_scancode_to_amiga[hid_scancode][map] == 0x00)) {
            hid_scancode_to_amiga[hid_scancode][map] = 0xff;
        }

        if ((map > 0) && (pos == map) &&
            (hid_scancode_to_amiga[hid_scancode][map - 1] == 0x00)) {
            /* Special case: tab key precedes removed key */
            hid_scancode_to_amiga[hid_scancode][map] = 0xff;
        }

        /* Check whether mapping is now completely empty */
        for (map = 0; map < max_map; map++) {
            uint8_t scancode = hid_scancode_to_amiga[hid_scancode][map];
            if ((map == 0) && (scancode == 0xff))
                continue;
            if (scancode != 0x00)
                break;
        }
        if (map == max_map) {
            /* Mapping is completely empty */
            uint cur = hid_scancode_to_capnum[hid_scancode];
            if (cur != 0xff) {
                hid_key_mapped[cur] = 0;
                draw_hid_key(cur, 0);
            }
        }
        uint8_t amiga_capnum = amiga_scancode_to_capnum[amiga_scancode];
        if (amiga_capnum != 0xff) {
            if (amiga_key_mapped[amiga_capnum] != 0)
                amiga_key_mapped[amiga_capnum]--;
            draw_amiga_key(amiga_capnum, 0);
        }
        return (1);
    }
    return (0); // Was not found
}

static int
add_amiga_mapping_to_hid_scancode(uint8_t hid_scancode,
                                  uint8_t amiga_scancode)
{
    uint       map;
    uint       pos;
    const uint max_map      = ARRAY_SIZE(hid_scancode_to_amiga[0]);
    uint       hid_capnum   = hid_scancode_to_capnum[hid_scancode];
    uint       amiga_capnum = amiga_scancode_to_capnum[amiga_scancode];

    /* Check if there is space for a new mapping */
    for (map = 0; map < max_map; map++) {
        if (hid_scancode_to_amiga[hid_scancode][map] == 0xff)
            break;  // End of list
        for (pos = map; pos < max_map; pos++)
            if (hid_scancode_to_amiga[hid_scancode][pos] != 0x00)
                break;
        if (pos == max_map)
            break;  // End of list

        if (hid_scancode_to_amiga[hid_scancode][map] == amiga_scancode)
            return (0); // Already in the list; treat it as "added"
    }
    if ((map >= max_map) ||
        ((amiga_scancode == 0x00) && (map >= max_map - 1))) {
        gui_printf("HID %02x: no space for added mapping", hid_scancode);
        return (-1);
    }

    /* Add mapping */
    hid_scancode_to_amiga[hid_scancode][map] = amiga_scancode;

    if (hid_capnum != 0xff)
        hid_key_mapped[hid_capnum]++;
    if (amiga_capnum != 0xff)
        amiga_key_mapped[amiga_capnum]++;

    if (amiga_scancode == 0)
        hid_scancode_to_amiga[hid_scancode][++map] = 0xff;
    return (0);
}

static int
remove_amiga_mapping_from_hid_button_scancode(uint8_t hid_scancode,
                                              uint8_t amiga_scancode)
{
    uint map;
    uint pos;
    const uint max_map = ARRAY_SIZE(hid_button_scancode_to_amiga[0]);

    if ((hid_scancode > NUM_BUTTON_SCANCODES) || (amiga_scancode == 0xff))
        return (0);

    /* Check if it's already mapped */
    for (map = 0; map < max_map; map++) {
        /* Check for end of list */
        if (hid_button_scancode_to_amiga[hid_scancode][map] == 0xff)
            break;  // End of list
        for (pos = map; pos < max_map; pos++)
            if (hid_button_scancode_to_amiga[hid_scancode][pos] != 0x00)
                break;
        if (pos == max_map)
            break;  // End of list

        if (hid_button_scancode_to_amiga[hid_scancode][map] != amiga_scancode)
            continue;

        /*
         * Found a mapping; remove it
         *
         * General case:
         *   ww xx yy zz -> ww xx zz 00
         *         ^^
         *
         * Special case:
         *   ww xx 00 ff -> ww xx 00 00
         *         ^^
         * Special case:
         *   ww 00 yy 00 -> ww 00 ff 00
         *         ^^
         */
        uint pos = map;
        for (; pos < max_map - 1; pos++)
            hid_button_scancode_to_amiga[hid_scancode][pos] =
                hid_button_scancode_to_amiga[hid_scancode][pos + 1];
        hid_button_scancode_to_amiga[hid_scancode][max_map - 1] = 0x00;

        if (hid_scancode != 0xff) {
            if (hid_button_mapped[hid_scancode] > 0)
                hid_button_mapped[hid_scancode]--;
        }

        if ((amiga_scancode == 0x00) &&
            (hid_button_scancode_to_amiga[hid_scancode][map] == 0xff)) {
            hid_button_scancode_to_amiga[hid_scancode][map] = 0x00;
        } else if ((map > 0) &&
                   (hid_button_scancode_to_amiga[hid_scancode][map - 1] == 0x00) &&
                   (hid_button_scancode_to_amiga[hid_scancode][map] == 0x00)) {
            hid_button_scancode_to_amiga[hid_scancode][map] = 0xff;
        }

        if ((map > 0) && (pos == map) &&
            (hid_button_scancode_to_amiga[hid_scancode][map - 1] == 0x00)) {
            /* Special case: tab key precedes removed key */
            hid_button_scancode_to_amiga[hid_scancode][map] = 0xff;
        }

        /* Check whether mapping is now completely empty */
        for (map = 0; map < max_map; map++) {
            uint8_t scancode = hid_button_scancode_to_amiga[hid_scancode][map];
            if ((map == 0) && (scancode == 0xff))
                continue;
            if (scancode != 0x00)
                break;
        }
        if (map == max_map) {
            /* Mapping is completely empty */
            hid_button_mapped[hid_scancode] = 0;
            uint cur = hid_button_scancode_to_capnum(hid_scancode);
            if (cur != 0xff)
                draw_hid_button(cur, 0);
        }
        uint8_t amiga_capnum = amiga_scancode_to_capnum[amiga_scancode];
        if (amiga_capnum != 0xff) {
            if (amiga_key_mapped[amiga_capnum] != 0)
                amiga_key_mapped[amiga_capnum]--;
            draw_amiga_key(amiga_capnum, 0);
        }
        return (1);
    }
    return (0); // Was not found
}

static int
add_amiga_mapping_to_hid_button_scancode(uint8_t hid_scancode,
                                         uint8_t amiga_scancode)
{
    uint       map;
    uint       pos;
    const uint max_map = ARRAY_SIZE(hid_button_scancode_to_amiga[0]);
    uint       amiga_capnum;

    if ((hid_scancode > NUM_BUTTON_SCANCODES) || (amiga_scancode == 0xff))
        return (0);

    amiga_capnum = amiga_scancode_to_capnum[amiga_scancode];

    /* Check if there is space for a new mapping */
    for (map = 0; map < max_map; map++) {
        if (hid_button_scancode_to_amiga[hid_scancode][map] == 0xff)
            break;  // End of list
        for (pos = map; pos < max_map; pos++)
            if (hid_button_scancode_to_amiga[hid_scancode][pos] != 0x00)
                break;
        if (pos == max_map)
            break;  // End of list

        if (hid_button_scancode_to_amiga[hid_scancode][map] == amiga_scancode)
            return (0); // Already in the list; treat it as "added"
    }
    if ((map >= max_map) ||
        ((amiga_scancode == 0x00) && (map >= max_map - 1))) {
        gui_printf("HID button %02x: no space for added mapping", hid_scancode);
        return (-1);
    }

    /* Add mapping */
    hid_button_scancode_to_amiga[hid_scancode][map] = amiga_scancode;
    hid_button_mapped[hid_scancode]++;
    if (amiga_capnum != 0xff)
        amiga_key_mapped[amiga_capnum]++;

    if (amiga_scancode == 0)
        hid_button_scancode_to_amiga[hid_scancode][++map] = 0xff;
    return (0);
}

static int
map_or_unmap_scancode(uint8_t amiga_scancode, uint8_t hid_scancode,
                      uint doing_hid_key)
{
    uint hid_capnum   = hid_scancode_to_capnum[hid_scancode];
    uint amiga_capnum = amiga_scancode_to_capnum[amiga_scancode];
    int  rc;

    if ((amiga_scancode == 0xff) || (hid_scancode == 0xff))
        return (-1);

    if (remove_amiga_mapping_from_hid_scancode(hid_scancode, amiga_scancode)) {
        /*
         * Successfully removed existing mapping
         *
         * Adjustment of hid_key_mapped[] and amiga_key_mapped[] was handled
         * by remove_amiga_mapping_from_hid_scancode();
         */

        if (doing_hid_key) {
            if (amiga_capnum != 0xff)
                draw_amiga_key(amiga_capnum, 0);
            if (hid_scancode != 0xff)
                enter_leave_hid_scancode(hid_scancode, 1, 1);
        } else {
            if (hid_capnum != 0xff)
                draw_hid_key(hid_capnum, 0);
            if (amiga_capnum != 0xff)
                enter_leave_amiga_scancode(amiga_scancode, 1, 1);
        }
        return (1);
    }

    rc = add_amiga_mapping_to_hid_scancode(hid_scancode, amiga_scancode);
    if (rc == 0) {
        if (doing_hid_key) {
            if (amiga_capnum != 0xff)
                draw_amiga_key(amiga_capnum, 1);
            enter_leave_hid_scancode(hid_scancode, 1, 1);
        } else {
            if (hid_capnum != 0xff)
                draw_hid_key(hid_capnum, 1);
            enter_leave_amiga_scancode(amiga_scancode, 1, 1);
        }
    }
    return (rc);
}

static int
map_or_unmap_button_scancode(uint8_t amiga_scancode, uint8_t scancode,
                             uint doing_hid_key)
{
    uint button_capnum = hid_button_scancode_to_capnum(scancode);
    uint amiga_capnum  = amiga_scancode_to_capnum[amiga_scancode];
    int  rc;

    if ((amiga_scancode == 0xff) || (scancode == 0xff))
        return (-1);

    if (remove_amiga_mapping_from_hid_button_scancode(scancode,
                                                      amiga_scancode)) {
        /*
         * Successfully removed existing mapping
         *
         * Adjustment of hid_button_mapped[] and amiga_key_mapped[] was
         * handled by remove_amiga_mapping_from_hid_button_scancode();
         */

        if (doing_hid_key) {
            if (amiga_capnum != 0xff)
                draw_amiga_key(amiga_capnum, 0);
            if (button_capnum != 0xff)
                enter_leave_hid_button_scancode(scancode, 1, 1);
        } else {
            if (button_capnum != 0xff)
                draw_hid_button(button_capnum, 0);
            if (amiga_capnum != 0xff)
                enter_leave_amiga_scancode(amiga_scancode, 1, 1);
        }
        return (1);
    }

    rc = add_amiga_mapping_to_hid_button_scancode(scancode, amiga_scancode);
    if (rc == 0) {
        if (doing_hid_key) {
            if (amiga_capnum != 0xff)
                draw_amiga_key(amiga_capnum, 1);
            enter_leave_hid_button_scancode(scancode, 1, 1);
        } else {
            if (button_capnum != 0xff)
                draw_hid_button(button_capnum, 1);
            enter_leave_amiga_scancode(amiga_scancode, 1, 1);
        }
    }
    return (rc);
}

static const char * const editbox_titles[] = { "Amiga", "Buttons", "HID" };
static void
editbox_draw_titles(void)
{
    uint is_hid;
    SHORT pos_x;
    SHORT pos_y;

    SetAPen(rp, pen_cap_white);
    SetBPen(rp, pen_cap_shaded);
    for (is_hid = 0; is_hid < NUM_EDITBOX_ROWS; is_hid++) {
        const char *str = editbox_titles[is_hid];
        uint        len = strlen(str);
        pos_y = editbox_y_min[is_hid];
        pos_x = editbox_x_min - font_pixels_x * (len + 1);
        Move(rp, pos_x, pos_y + font_pixels_y);
        Text(rp, str, len);
    }
}

static void
editbox_show_single(uint is_at_cursor, uint cursor_x, uint cursor_y)
{
    SHORT ypos = editbox_y_min[cursor_y] + font_pixels_y;
    SHORT xpos = editbox_x_min + cursor_x * font_pixels_x +
                 (cursor_x / 2) * 2 + 1;

    if (xpos >= editbox_x_max[cursor_y])
        return;

    /* Fill background above the digit */
    SetAPen(rp, is_at_cursor ? 3 : pen_status_bg);
    Move(rp, xpos, ypos - font_pixels_y + 1);
    Draw(rp, xpos + font_pixels_x, ypos - font_pixels_y + 1);

    /* Digit */
    SetAPen(rp, pen_status_fg);
    SetBPen(rp, is_at_cursor ? 3 : pen_status_bg);
    Move(rp, xpos, ypos);

    Text(rp, &editbox[cursor_y][cursor_x], 1);
}

static void
editbox_key_display(uint show_keys)
{
    switch (editbox_update_mode) {
        case 0:
            enter_leave_amiga_scancode(editbox_scancode, show_keys, 1);
            break;
        case 1:
            enter_leave_hid_button(editbox_scancode, show_keys, 1);
            break;
        case 2:
            enter_leave_hid_scancode(editbox_scancode, show_keys, 1);
            break;
    }
}

static void
editbox_set_cursor_position(uint edit_capnum)
{
    editbox_show_single(0, editbox_cursor_x, editbox_cursor_y);
    editbox_cursor_x = edit_capnum & 0x0f;
    editbox_cursor_y = edit_capnum >> 4;
    editbox_show_single(1, editbox_cursor_x, editbox_cursor_y);
    edit_key_mapping_mode = 1;

    editbox_key_display(1);
}

static void
editbox_draw(void)
{
    editbox_draw_box(0);
    editbox_draw_box(1);
    editbox_draw_box(2);
}

typedef struct {
    uint8_t scancode;
    uint8_t value;
} key_to_hex_digit_t;
static const key_to_hex_digit_t key_to_hex_digit[] = {
    { AS_0,    '0' },
    { AS_1,    '1' },
    { AS_2,    '2' },
    { AS_3,    '3' },
    { AS_4,    '4' },
    { AS_5,    '5' },
    { AS_6,    '6' },
    { AS_7,    '7' },
    { AS_8,    '8' },
    { AS_9,    '9' },
    { AS_KP_0, '0' },
    { AS_KP_1, '1' },
    { AS_KP_2, '2' },
    { AS_KP_3, '3' },
    { AS_KP_4, '4' },
    { AS_KP_5, '5' },
    { AS_KP_6, '6' },
    { AS_KP_7, '7' },
    { AS_KP_8, '8' },
    { AS_KP_9, '9' },
    { AS_A,    'a' },
    { AS_B,    'b' },
    { AS_C,    'c' },
    { AS_D,    'd' },
    { AS_E,    'e' },
    { AS_F,    'f' },
};

static uint
editbox_okay_to_end(uint *new_x, uint *new_y)
{
    uint cursor_x;
    uint cursor_y;

    /* Check characters */
    for (cursor_y = 0; cursor_y < NUM_EDITBOX_ROWS; cursor_y++) {
        for (cursor_x = 0; cursor_x < editbox_max_len[cursor_y]; cursor_x++) {
            if ((editbox[cursor_y][cursor_x] == ' ') &&
                (editbox[cursor_y][cursor_x ^ 1] != ' ')) {
                /* One digit of two is a space */
                *new_x = cursor_x;
                *new_y = cursor_y;
                gui_printf2("Partial scancode at %u", cursor_x);
                return (0);
            }
        }
    }
    return (1);
}

/*
 * editbox_save_mapping_to_key() will parse the two strings, converting
 * them back to scancodes, and then save the new mappings.
 */
static uint
editbox_save_mapping_to_key(void)
{
    uint    cur;
    uint    pos;
    uint    amiga_scancode;
    uint    hid_scancode;
    uint    scancode;
    char    *ptr;
    uint8_t newlist[8];
    uint8_t oldlist[8];
    uint8_t newlist_2[8];
    uint8_t oldlist_2[8];
    uint    newcount = 0;
    uint    newcount_2 = 0;
    uint    box_for_single   = (editbox_update_mode == 0) ? 0 : 1;
    uint    box_for_multiple = (editbox_update_mode == 0) ? 1 : 0;
    switch (editbox_update_mode) {
        case 0:
            box_for_single   = 0;
            box_for_multiple = 1;
            break;
        case 1:
            box_for_single   = 1;
            box_for_multiple = 0;
            break;
        case 2:
            box_for_single   = 2;
            box_for_multiple = 0;
            break;
    }

    /* Get "from" scancode */
    if (sscanf(editbox[box_for_single], "%02x", &scancode) != 1) {
        gui_printf2("Failed to scan Amiga %.2s", editbox);
        return (1);
    }
    if ((editbox_update_mode == 0) && (scancode == 0xff)) {
        gui_printf2("Amiga scancode %02x is invalid", scancode);
        return (1);
    }
    if ((editbox_update_mode == 1) && ((scancode == 0) || (scancode == 0xff))) {
        gui_printf2("HID scancode %02x is invalid", scancode);
        return (1);
    }

    ptr = editbox[box_for_multiple];
    for (cur = 0; cur < editbox_max_len[box_for_multiple]; cur += 2, ptr += 2) {
        /* Get mapped Amiga or HID button scancodes */
        uint value;
        if ((ptr[0] == ' ') && (ptr[1] == ' '))
            continue;  // Skip blank entry

        if (sscanf(ptr, "%02x", &value) != 1) {
            gui_printf2("Failed to scan %.2s", ptr);
            return (1);
        }
        if (editbox_update_mode == 0) {
            /* Mapping Amiga -> HID */
            if (newcount >= sizeof (newlist)) {
                gui_printf2("Too many HID entries");
                return (1);
            }
            if (value >= 0x40) {
                gui_printf2("HID button scancode %02x is invalid", value);
                return (1);
            }
        } else {
            /* Mapping HID -> Amiga */
            if (newcount >= ARRAY_SIZE(hid_scancode_to_amiga[0])) {
                gui_printf2("Too many Amiga entries");
                return (1);
            }
        }
        if (newcount >= ARRAY_SIZE(newlist))
            break;
        newlist[newcount++] = value;
    }
    if (editbox_update_mode == 0) {
        /* Get mapped HID scancodes */
        ptr = editbox[2];
        for (cur = 0; cur < editbox_max_len[2]; cur += 2, ptr += 2) {
            /* Get mapped scancodes */
            uint value;
            if ((ptr[0] == ' ') && (ptr[1] == ' '))
                continue;  // Skip blank entry

            if (sscanf(ptr, "%02x", &value) != 1) {
                gui_printf2("Failed to scan %.2s", ptr);
                return (1);
            }
            /* Mapping Amiga -> HID */
            if (newcount_2 >= sizeof (newlist_2)) {
                gui_printf2("Too many HID button entries");
                return (1);
            }
            if ((value == 0) || (value == 0xff)) {
                gui_printf2("HID scancode %02x is invalid", value);
                return (1);
            }
            if (newcount_2 >= ARRAY_SIZE(newlist_2))
                break;
            newlist_2[newcount_2++] = value;
        }
    }

    if (editbox_update_mode == 0) {
        uint map;
        uint oldcount = 0;
        uint oldcount_2 = 0;
        const uint max_map = ARRAY_SIZE(hid_scancode_to_amiga[0]);

        /*
         * Single Amiga scancode can map to multiple HID scancodes
         * or HID button scancodes.
         */
        amiga_scancode = scancode;

        /* Generate the list of current HID button -> amiga_scancode mappings */
        for (hid_scancode = 0;
             hid_scancode <= NUM_BUTTON_SCANCODES; hid_scancode++) {
            for (map = 0; map < max_map; map++) {
                if (hid_button_scancode_to_amiga[hid_scancode][map] == 0xff)
                    break;  // End of list
                for (pos = map; pos < max_map; pos++)
                    if (hid_button_scancode_to_amiga[hid_scancode][pos] != 0x00)
                        break;
                if (pos == max_map)
                    break;  // End of list

                if (hid_button_scancode_to_amiga[hid_scancode][map] !=
                    amiga_scancode)
                    continue;

                if (oldcount >= ARRAY_SIZE(oldlist))
                    break;

                /* Found a mapping */
                oldlist[oldcount++] = hid_scancode;
            }
        }
        /* Generate the list of current HID -> amiga_scancode mappings */
        for (hid_scancode = 0; hid_scancode <= 0xff; hid_scancode++) {
            for (map = 0; map < max_map; map++) {
                if (hid_scancode_to_amiga[hid_scancode][map] == 0xff)
                    break;  // End of list
                for (pos = map; pos < max_map; pos++)
                    if (hid_scancode_to_amiga[hid_scancode][pos] != 0x00)
                        break;
                if (pos == max_map)
                    break;  // End of list

                if (hid_scancode_to_amiga[hid_scancode][map] != amiga_scancode)
                    continue;

                if (oldcount_2 >= ARRAY_SIZE(oldlist_2))
                    break;

                /* Found a mapping */
                oldlist_2[oldcount_2++] = hid_scancode;
            }
        }

        /*
         * Now that the lists have been generated, remove all mappings
         * which are not in the newlist.
         */
        for (cur = 0; cur < oldcount; cur++) {
            for (pos = 0; pos < newcount; pos++)
                if (oldlist[cur] == newlist[pos])
                    break;
            if (pos < newcount) {
                /* In both lists */
                newlist[pos] = 0xff;  // Eliminate from the new list
            } else {
                /* Remove old mapping */
                remove_amiga_mapping_from_hid_button_scancode(oldlist[cur],
                                                              amiga_scancode);
            }
        }
        for (cur = 0; cur < oldcount_2; cur++) {
            for (pos = 0; pos < newcount_2; pos++)
                if (oldlist_2[cur] == newlist_2[pos])
                    break;
            if (pos < newcount_2) {
                /* In both lists */
                newlist_2[pos] = 0xff;  // Eliminate from the new list
            } else {
                /* Remove old mapping */
                remove_amiga_mapping_from_hid_scancode(oldlist_2[cur],
                                                       amiga_scancode);
            }
        }

        /* Finally add mappings which were not in the original list */
        for (cur = 0; cur < newcount; cur++) {
            if (newlist[cur] != 0xff) {
                add_amiga_mapping_to_hid_button_scancode(newlist[cur],
                                                         amiga_scancode);
            }
        }
        for (cur = 0; cur < newcount_2; cur++) {
            if (newlist_2[cur] != 0xff) {
                add_amiga_mapping_to_hid_scancode(newlist_2[cur],
                                                  amiga_scancode);
            }
        }
        gui_printf2("");
        gui_printf("Mapping for Amiga %02x updated", amiga_scancode);
    } else {
        /*
         * Single HID scancode or HID button scancode can map to up
         * to 4 Amiga scancodes
         */
        hid_scancode = scancode;

        /*
         * Massage the list of new scancodes.
         * A 0x00 entry is special. It can mean either a backtick (reverse
         * aprostrophe) or just 0x00 padding for unused mapping cells. If
         * a real 0x00 entry is present, it must be terminated by another
         * non-zero entry, such as 0xff.
         * If no entries are present, 0xff should be in the first entry.
         * Other 0xff entries are eliminated unless they terminate a 0x00
         * entry.
         *
         * 00 ff ff 00    00 ff 00 00    00 ff 11 00    11 ff xx xx
         *    ^ remove       ^ keep         ^ remove       ^ remove
         *
         * ff 00 00 00    ff 11 00 00    ff 00 11 00
         * ^ keep         ^ remove       ^ remove
         */

        /* Trim trailing 0x00 */
        for (cur = newcount - 1; cur > 0; cur--) {
            if (newlist[cur] != 0x00)
                break;
            newcount--;
        }

        /* Remove excess 0xff */
        for (cur = 0; cur < newcount; cur++) {
            if (newlist[cur] == 0xff) {
                for (pos = cur + 1; pos < newcount; pos++) {
                    if (newlist[pos] != 0x00)
                        break;
                }
                if ((pos < newcount) || // Something follows
                    ((cur > 0) && (newlist[cur - 1] != 0x00))) {
                    /* Eliminate this entry */
                    newcount--;
                    for (pos = cur; pos < newcount; pos++)
                        newlist[pos] = newlist[pos + 1];
                    cur--;
                }
            }
        }

        /* Terminate list with 0xff, if required */
        if (newcount == 0) {
            newlist[0] = 0xff;
            cur = 1;
        }
        if ((newcount == 1) && (newlist[0] == 0x00)) {
            newlist[1] = 0xff;
            cur = 2;
        }

        /* Fill remainder with 0x00 */
        for (; cur < ARRAY_SIZE(hid_scancode_to_amiga[0]); cur++)
            newlist[cur] = 0x00;

        if (editbox_update_mode == 1) {
            /* HID button scancode to Amiga */
            for (cur = 0; cur < ARRAY_SIZE(hid_button_scancode_to_amiga[0]);
                 cur++) {
                hid_button_scancode_to_amiga[hid_scancode][cur] = newlist[cur];
            }
            gui_printf2("");
            gui_printf("Mapping for HID button %02x updated", hid_scancode);
        } else {
            /* HID scancode to Amiga */
            for (cur = 0; cur < ARRAY_SIZE(hid_scancode_to_amiga[0]); cur++)
                hid_scancode_to_amiga[hid_scancode][cur] = newlist[cur];
            gui_printf2("");
            gui_printf("Mapping for HID %02x updated", hid_scancode);
        }
    }
    return (0);
}

static void
editbox_key_command(uint8_t scancode)
{
    uint pos;
    uint cursor_x = editbox_cursor_x;
    uint cursor_y = editbox_cursor_y;
    uint do_complete_redraw = 0;

    if (scancode & 0x80)
        return;  // Don't care about key up events

    switch (scancode) {
        case AS_SPACE:      // Overwrite cur with space
            if (cursor_x < editbox_max_len[cursor_y] - 1) {
                editbox[cursor_y][cursor_x] = ' ';
                cursor_x++;
            }
            break;
        case AS_BACKSPACE:  // Overwrite prev with space
            if (cursor_x == 0)
                break;
            editbox[cursor_y][--cursor_x] = ' ';
            break;
        case AS_DELETE:     // Delete char and possibly one to the right
            for (pos = cursor_x; pos < editbox_max_len[cursor_y] - 1; pos++)
                editbox[cursor_y][pos] = editbox[cursor_y][pos + 1];
            editbox[cursor_y][pos] = ' ';
            do_complete_redraw = 1;
            break;
        case AS_UP:         // Move to Amiga editing
            if (cursor_y > 0) {
                cursor_y--;
                if ((cursor_y == 1) && (editbox_update_mode == 2)) {
                    /* Middle is not active when HID -> Amiga is active */
                    cursor_y--;
                }
                if (cursor_x > editbox_max_len[cursor_y] - 1)
                    cursor_x = editbox_max_len[cursor_y] - 1;
            }
            break;
        case AS_DOWN:       // Move to HID editing
            if ((cursor_y == 0) ||
                ((cursor_y == 1) && (editbox_update_mode == 0))) {
                cursor_y++;
                if ((cursor_y == 1) && (editbox_update_mode == 2)) {
                    /* Middle is not active when HID -> Amiga is active */
                    cursor_y++;
                }
                if (cursor_x > editbox_max_len[cursor_y] - 1)
                    cursor_x = editbox_max_len[cursor_y] - 1;
            }
            break;
        case AS_RIGHT:      // Move left
            if (cursor_x < editbox_max_len[cursor_y] - 1)
                cursor_x++;
            break;
        case AS_LEFT:       // Move right
            if (cursor_x > 0)
                cursor_x--;
            break;
        case AS_INSERT:     // Insert space, possibly pushing to the right
            for (pos = editbox_max_len[cursor_y] - 1; pos > cursor_x; pos--)
                editbox[cursor_y][pos] = editbox[cursor_y][pos - 1];
            editbox[cursor_y][cursor_x] = ' ';
            do_complete_redraw = 1;
            break;
        case AS_ESC:        // Terminate edit mode
            editbox_show_single(0, editbox_cursor_x, editbox_cursor_y);
            editbox_key_display(0);
            edit_key_mapping_mode = 0;
            return;
        case AS_ENTER:      // Save and terminate edit mode
        case AS_KP_ENTER:   // Save and terminate edit mode
            editbox_show_single(0, editbox_cursor_x, editbox_cursor_y);
            editbox_key_display(0);
            if (editbox_okay_to_end(&cursor_x, &cursor_y) &&
                (editbox_save_mapping_to_key() == 0)) {
                edit_key_mapping_mode = 0;
                return;
            }
            /* Failed; resume editing */
            editbox_key_display(1);
            break;
        default:
            for (pos = 0; pos < ARRAY_SIZE(key_to_hex_digit); pos++)
                if (key_to_hex_digit[pos].scancode == scancode)
                    break;
            if (pos < ARRAY_SIZE(key_to_hex_digit)) {
                /* Digit match; replace current digit and advance cursor */
                editbox[cursor_y][cursor_x] = key_to_hex_digit[pos].value;
                if (cursor_x < editbox_max_len[cursor_y] - 1)
                    cursor_x++;
            }
    }

    if (do_complete_redraw) {
        editbox_cursor_x = cursor_x;
        editbox_cursor_y = cursor_y;
        for (pos = 0; pos < editbox_max_len[cursor_y]; pos++)
            editbox_show_single(pos == cursor_x, pos, editbox_cursor_y);
    } else if ((editbox_cursor_x != cursor_x) ||
               (editbox_cursor_y != cursor_y)) {
        /* Disable cursor, do move, enable cursor */
        editbox_show_single(0, editbox_cursor_x, editbox_cursor_y);
        editbox_cursor_x = cursor_x;
        editbox_cursor_y = cursor_y;
    }
    editbox_show_single(1, editbox_cursor_x, editbox_cursor_y);
}

static void
editbox_key_command_hid(uint8_t scancode)
{
    uint map;
    for (map = 0; map < ARRAY_SIZE(hid_scancode_to_amiga[0]); map++) {
        uint8_t code = hid_scancode_to_amiga[scancode][map];
        if (code == 0xff)
            break;
        if (code == 0x00) {
            uint cur = map + 3;
            for (; cur < ARRAY_SIZE(hid_scancode_to_amiga[0]); cur++)
                if (hid_scancode_to_amiga[scancode][cur] != 0x00)
                    break;
            if (cur == ARRAY_SIZE(hid_scancode_to_amiga[0]))
                break;  // No more mappings
        }
        editbox_key_command(code);
    }
}

static void
draw_amiga_mouse_buttons_title(void)
{
    const char mousestr[] = "Mouse Buttons";
    const uint mouselen = sizeof (mousestr) - 1;
    SHORT which_box = ARRAY_SIZE(amiga_key_bbox) - 1;
    SHORT ycenter = (amiga_key_bbox[which_box].y_min +
                     amiga_key_bbox[which_box].y_max) / 2;
    SHORT ypos = ycenter + font_pixels_y / 2 - 1;
    SHORT xpos = amiga_key_bbox[which_box].x_max + font_pixels_x;
    SetAPen(rp, pen_cap_white);
    SetBPen(rp, pen_cap_shaded);
    Move(rp, xpos, ypos);
    Text(rp, mousestr, mouselen);
}

static void
draw_hid_buttons(void)
{
    uint cur;
    for (cur = 0; cur < NUM_HID_BUTTONS_PLUS_DIRECTIONS; cur++)
        draw_hid_button(cur, 0);
}

static BOOL
draw_win(void)
{
    uint cur;

    win_width  = window->Width - window->BorderLeft - window->BorderRight;
    win_height = window->Height - window->BorderTop - window->BorderBottom;

    kbd_width  = win_width - 2;
    kbd_height = win_height * 24 / 64;

    scale_key_dimensions();

    kbd_keyarea_width  = kbd_width  - amiga_keysize[0].x * 2;
    kbd_keyarea_height = kbd_height - amiga_keysize[0].y * 2;

    /* Draw Amiga keyboard case */
    uint win_left = window->BorderLeft;
    hid_keyboard_top = win_height - kbd_height - 2 +
                       window->BorderTop + window->BorderBottom;

    /* Default drawing mode */
    SetDrMd(rp, JAM2);
    SetBPen(rp, pen_keyboard_case);

    /* Draw Amiga keyboard case */
    SetAPen(rp, pen_keyboard_case);
    RectFill(rp, win_left, window->BorderTop,
             win_left + kbd_width, window->BorderTop + kbd_height);

    /* Empty area between keyboards */
    SetAPen(rp, 0);
    RectFill(rp, win_left, window->BorderTop + kbd_height,
             win_left + kbd_width, hid_keyboard_top);

    /* Draw HID keyboard case */
    SetAPen(rp, pen_keyboard_case);
    RectFill(rp, win_left, hid_keyboard_top,
             win_left + kbd_width, hid_keyboard_top + kbd_height);

    /* Draw HID keyboard outline */
    SetAPen(rp, pen_cap_white);
    box(win_left, hid_keyboard_top,
        win_left + kbd_width, hid_keyboard_top + kbd_height);

    /* Draw Amiga keyboard outline */
    SetAPen(rp, pen_cap_white);
    box(win_left, window->BorderTop,
        win_left + kbd_width, window->BorderTop + kbd_height);

    /* Draw Amiga keyboard */
    amiga_keyboard_left = window->BorderLeft + amiga_keysize[0].x + 4;
    amiga_keyboard_top  = window->BorderTop  + amiga_keysize[0].y + 4;

    for (cur = 0; cur < ARRAY_SIZE(amiga_keypos); cur++)
        draw_amiga_key(cur, 0);

    /* Draw HID keyboard */
    hid_keyboard_left = window->BorderLeft + hid_keysize[0].x + 4;

    for (cur = 0; cur < ARRAY_SIZE(hid_keypos); cur++)
        draw_hid_key(cur, 0);

    /* Draw HID mouse buttons */
    draw_hid_buttons();

    /*
     * Location for two gui_print() lines will be at center of empty area
     * (-/- font_pixels_y).
     */
    uint gap_center_y = (window->BorderTop + kbd_height + hid_keyboard_top) / 2;
    gui_print_line1_y = gap_center_y + font_pixels_y;
    gui_print_line2_y = gap_center_y - 1;

    editbox_x_min = win_width - (font_pixels_x + 2) * 6 - font_pixels_x * 5 - 5;
    editbox_y_min[0] = (amiga_keyboard_top + kbd_height - 6);  // Amiga
    editbox_y_min[1] = (gap_center_y - 1);                     // HID Buttons
    editbox_y_min[2] = (hid_keyboard_top - 4 - font_pixels_y); // HID
    for (cur = 0; cur < NUM_EDITBOX_ROWS; cur++) {
        editbox_x_max[cur] = editbox_x_min + 1 +
                             font_pixels_x * editbox_max_len[cur] +
                             ((editbox_max_len[cur] - 1) / 2) * 2;  // gap
        editbox_y_max[cur] = editbox_y_min[cur] + font_pixels_y + 4;
    }

    draw_amiga_mouse_buttons_title();
    editbox_draw_titles();
    editbox_draw();

    user_usage_hint_show(0);
    return (TRUE);
}

static void
amiga_rawkey(uint8_t scancode, uint released)
{
    uint cur = amiga_scancode_to_capnum[scancode];

//  printf("amiga_rawkey %x\n", scancode | released);
    if (edit_key_mapping_mode) {
        if (!released)
            editbox_key_command(scancode);
        return;
    }

    if (!released) {
        if (enable_esc_exit && !released)
            handle_esc_key(scancode == RAWKEY_ESC);
        if (key_mapping_mode != KEY_MAPPING_MODE_HID_TO_AMIGA)
            editbox_copy_amiga_scancode_mapping(scancode);
    }

    /*
     * key_mapping_mode
     * 0  An Amiga keystroke switches to that key's mappings
     * 1  An Amiga keystroke adds or removes the key from the current
     *    HID -> Amiga mapping
     * 2  Live keystrokes are reported with no regard for mappings.
     * 3  Live keystrokes and their mappings are reported
     */
    switch (key_mapping_mode) {
        case KEY_MAPPING_MODE_AMIGA_TO_HID:
            /* Last key pressed stays highlighted */
            if (released)
                return;
            if (amiga_last_scancode != scancode) {
                /* Exit all keys which map to the last code */
                enter_leave_amiga_scancode(amiga_last_scancode, 0, 1);
            }
            enter_leave_amiga_scancode(scancode, 1, 1);
            break;
        case KEY_MAPPING_MODE_HID_TO_AMIGA:
            if (released)
                return;
            if (hid_last_scancode != 0xff) {
                map_or_unmap_scancode(scancode, hid_last_scancode, 1);
                enter_leave_hid_scancode(hid_last_scancode, 1, 1);
                editbox_copy_hid_scancode_mapping(hid_last_scancode);
            } else if (hid_button_last_scancode != 0xff) {
                uint8_t button_scancode = hid_button_last_scancode;
                map_or_unmap_button_scancode(scancode, button_scancode, 1);
                enter_leave_hid_button(button_scancode, 1, 1);
                editbox_copy_hid_button_scancode_mapping(button_scancode);
            }
            break;
        case KEY_MAPPING_MODE_LIVEKEYS:
            /* Just highlight all keys which are currently pressed */
            if (cur != 0xff)
                draw_amiga_key(cur, !released);
            break;
        case KEY_MAPPING_MODE_LIVEKEYS_MAPPED:
            enter_leave_amiga_scancode(scancode, !released, 1);
            break;
    }
    if (!released) {
        amiga_last_scancode = scancode;
    }
}

static void
hid_rawkey(uint8_t scancode, uint key_state)
{
    uint cur = hid_scancode_to_capnum[scancode];
    uint released = key_state & KEYCAP_UP;

//  printf("hid_rawkey %x %x\n", scancode, released);

    if (edit_key_mapping_mode) {
        if (!released)
            editbox_key_command_hid(scancode);
        return;
    }

    if (!released) {
        if (enable_esc_exit && !released)
            handle_esc_key(scancode == HS_ESC);
        if (key_mapping_mode != KEY_MAPPING_MODE_AMIGA_TO_HID)
            editbox_copy_hid_scancode_mapping(scancode);
    }

    /*
     * key_mapping_mode
     * 0  A HID keystroke adds or removes the key from the current
     *    Amiga <- HID mapping
     * 1  A HID keystroke switches to that key's mappings
     * 2  Live keystrokes are reported with no regard for mappings.
     * 3  Live keystrokes and their mappings are reported
     */
    switch (key_mapping_mode) {
        case KEY_MAPPING_MODE_AMIGA_TO_HID:
            if (released)
                return;
            map_or_unmap_scancode(amiga_last_scancode, scancode, 0);
            editbox_copy_amiga_scancode_mapping(amiga_last_scancode);
            break;
        case KEY_MAPPING_MODE_HID_TO_AMIGA:
            /* Last key pressed stays highlighted */
            if (released)
                return;
            if (hid_button_last_scancode != 0xff) {
                enter_leave_hid_button(hid_button_last_scancode, 0, 1);
                hid_button_last_scancode = 0xff;
            }
            if (hid_last_scancode != scancode) {
                /* Exit all keys which map to the last code */
                enter_leave_hid_scancode(hid_last_scancode, 0, 1);
            }
            enter_leave_hid_scancode(scancode, 1, 1);
            break;
        case KEY_MAPPING_MODE_LIVEKEYS:
            /* Just highlight all keys which are currently pressed */
            if (cur != 0xff)
                draw_hid_key(cur, !released);
            break;
        case KEY_MAPPING_MODE_LIVEKEYS_MAPPED:
            enter_leave_hid_scancode(scancode, !released, 1);
            break;
    }
    if (!released) {
        hid_last_scancode = scancode;
    }
}

static void
hid_rawbutton(uint8_t scancode, uint key_state)
{
    uint cur = hid_button_scancode_to_capnum(scancode);
    uint released = key_state & KEYCAP_UP;

//  printf("hid_rawbutton %x %x\n", scancode, key_state);
    if (cur >= NUM_HID_BUTTONS_PLUS_DIRECTIONS) {
        /* This can happen if displaying mouse and got joystick button */
        return;
    }

    if ((scancode == 0x00) || (scancode == 0x01)) {
        /*
         * Ignore left and right mouse button captures because handling
         * them can interfere with the operation being performed (or
         * even clobber an AmigaOS menu).
         */
        return;
    }

    if (edit_key_mapping_mode)
        return;

    if (!released) {
        if (key_mapping_mode != KEY_MAPPING_MODE_AMIGA_TO_HID)
            editbox_copy_hid_button_scancode_mapping(scancode);
    }

    /*
     * key_mapping_mode
     * 0  A HID button adds or removes the key from the current
     *    Amiga <- HID mapping
     * 1  A HID button switches to that button's mappings
     * 2  Live buttons are reported with no regard for mappings.
     * 3  Live buttons and their mappings are reported
     */
    switch (key_mapping_mode) {
        case KEY_MAPPING_MODE_AMIGA_TO_HID:
            if (released)
                return;
            map_or_unmap_button_scancode(amiga_last_scancode, scancode, 0);
            editbox_copy_amiga_scancode_mapping(amiga_last_scancode);
            break;
        case KEY_MAPPING_MODE_HID_TO_AMIGA:
            /* Last button pressed stays highlighted */
            if (released)
                return;
            if (hid_last_scancode != 0xff) {
                enter_leave_hid_scancode(hid_last_scancode, 0, 1);
                hid_last_scancode = 0xff;
            }
            if (hid_button_last_scancode != scancode) {
                /* Exit all keys which map to the last code */
                enter_leave_hid_button(hid_button_last_scancode, 0, 1);
            }
            enter_leave_hid_button(scancode, 1, 1);
            break;
        case KEY_MAPPING_MODE_LIVEKEYS:
            /* Just highlight all keys which are currently pressed */
            if (cur != 0xff)
                draw_hid_button(cur, !released);
            break;
        case KEY_MAPPING_MODE_LIVEKEYS_MAPPED:
            enter_leave_hid_button(scancode, !released, 1);
            break;
    }
    if (!released) {
        hid_button_last_scancode = scancode;
    }
}

static void
mouse_button_press(uint button_down)
{
    /* Mouse button press occurred */

    /* Let the keystroke handlers deal with button-triggered key presses */
    switch (mouse_cur_selection) {
        case MOUSE_CUR_SELECTION_NONE: // None
            break;
        case MOUSE_CUR_SELECTION_AMIGA: // Amiga
            if (mouse_cur_capnum == 0xff)
                return;  // Not over keyboard button
            amiga_rawkey(amiga_keypos[mouse_cur_capnum].scancode, !button_down);
            break;
        case MOUSE_CUR_SELECTION_HID: // HID
            if (mouse_cur_capnum == 0xff)
                return;  // Not over keyboard button
            hid_rawkey(hid_keypos[mouse_cur_capnum].scancode, !button_down);
            break;
        case MOUSE_CUR_SELECTION_EDITBOX: // Editbox
            if (button_down)
                editbox_set_cursor_position(mouse_cur_capnum);
            break;
        case MOUSE_CUR_SELECTION_HID_BUTTON: // Mouse / Joystick button
            if (mouse_cur_scancode == 0xff)
                return;  // Not over keyboard button
            hid_rawbutton(mouse_cur_scancode, !button_down);
            break;
    }
}

static void
remove_single_key_mappings(void)
{
    uint map;
    uint8_t capnum = 0xff;
    uint8_t scancode = 0xff;
    uint    remove_hid = 0;

    switch (key_mapping_mode) {
        case KEY_MAPPING_MODE_AMIGA_TO_HID:
            scancode = amiga_last_scancode;
            capnum = amiga_scancode_to_capnum[scancode];
            remove_hid = 0;
            break;
        case KEY_MAPPING_MODE_HID_TO_AMIGA:
            scancode = hid_last_scancode;
            capnum = hid_scancode_to_capnum[scancode];
            remove_hid = 1;
            break;
        case KEY_MAPPING_MODE_LIVEKEYS:
        case KEY_MAPPING_MODE_LIVEKEYS_MAPPED:
            switch (mouse_cur_selection) {
                case MOUSE_CUR_SELECTION_NONE: // None
                    break;
                case MOUSE_CUR_SELECTION_AMIGA: // Amiga
                    scancode = amiga_last_scancode;
                    capnum = amiga_scancode_to_capnum[scancode];
                    remove_hid = 0;
                    break;
                case MOUSE_CUR_SELECTION_HID: // HID
                    scancode = hid_last_scancode;
                    capnum = hid_scancode_to_capnum[scancode];
                    remove_hid = 1;
                    break;
                case MOUSE_CUR_SELECTION_EDITBOX: // Editbox
                    break;
                case MOUSE_CUR_SELECTION_HID_BUTTON: // HID buttons
                    scancode = hid_button_last_scancode;
                    capnum = hid_scancode_to_capnum[scancode];
                    remove_hid = 2;
                    break;
            }
            break;
    }
    if (capnum == 0xff) {
        gui_printf("You must first select a key to remove its mappings.");
        return;
    }

    if (remove_hid == 0) {
        /* Remove all HID mappings to this Amiga scancode */
        uint hid_scancode;
        enter_leave_amiga_scancode(scancode, 0, 1);

        for (hid_scancode = 0; hid_scancode <= 0xff; hid_scancode++) {
            remove_amiga_mapping_from_hid_scancode(hid_scancode, scancode);
        }
        amiga_key_mapped[capnum] = 0;
        draw_amiga_key(capnum, 0);
        if ((key_mapping_mode == KEY_MAPPING_MODE_LIVEKEYS) ||
            (key_mapping_mode == KEY_MAPPING_MODE_LIVEKEYS_MAPPED)) {
            enter_leave_amiga_scancode(scancode, 0, 1);
        } else {
            enter_leave_amiga_scancode(scancode, 1, 1);
        }
    } else if (remove_hid == 1) {
        /* Remove all Amiga mappings to this HID scancode */
        enter_leave_hid_scancode(scancode, 0, 1);
        for (map = 0; map < ARRAY_SIZE(hid_scancode_to_amiga[0]); map++) {
            uint8_t amiga_scancode = hid_scancode_to_amiga[scancode][map];
            remove_amiga_mapping_from_hid_scancode(scancode, amiga_scancode);
        }
        hid_scancode_to_amiga[scancode][0] = 0xff;

        hid_key_mapped[capnum] = 0;
        draw_hid_key(capnum, 0);
        if ((key_mapping_mode == KEY_MAPPING_MODE_LIVEKEYS) ||
            (key_mapping_mode == KEY_MAPPING_MODE_LIVEKEYS_MAPPED)) {
            enter_leave_hid_scancode(scancode, 0, 1);
        } else {
            enter_leave_hid_scancode(scancode, 1, 1);
        }
    } else {
        /* Remove all Amiga mappings to this HID button scancode */
        uint8_t amiga_scancode;
        enter_leave_hid_button(scancode, 0, 1);
        for (map = 0; map < ARRAY_SIZE(hid_button_scancode_to_amiga[0]);
             map++) {
            amiga_scancode = hid_button_scancode_to_amiga[scancode][map];
            remove_amiga_mapping_from_hid_scancode(scancode, amiga_scancode);
        }
        hid_button_scancode_to_amiga[scancode][0] = 0xff;

        hid_button_mapped[scancode] = 0;
        draw_hid_button(capnum, 0);
        if ((key_mapping_mode == KEY_MAPPING_MODE_LIVEKEYS) ||
            (key_mapping_mode == KEY_MAPPING_MODE_LIVEKEYS_MAPPED)) {
            enter_leave_hid_button(scancode, 0, 1);
        } else {
            enter_leave_hid_button(scancode, 1, 1);
        }
    }
}

static void
unmap_all_keycaps(void)
{
    uint cap;
    memset(amiga_key_mapped, 0, sizeof (amiga_key_mapped));
    memset(hid_key_mapped, 0, sizeof (hid_key_mapped));
    memset(hid_button_mapped, 0, sizeof (hid_button_mapped));
    if (gui_initialized == 0)
        return;
    for (cap = 0; cap < ARRAY_SIZE(amiga_keypos); cap++)
        draw_amiga_key(cap, 0);
    for (cap = 0; cap < ARRAY_SIZE(hid_keypos); cap++)
        draw_hid_key(cap, 0);
    for (cap = 0; cap < NUM_HID_BUTTONS_PLUS_DIRECTIONS; cap++)
        draw_hid_button(cap, 0);
}

static uint
load_keymap_from_bec(uint which)
{
    bec_keymap_t  req;
    bec_keymap_t *reply;
    uint          rc;
    uint          pos;
    uint          maxpos;
    uint          cur;
    uint          rlen;
    uint          map;
    uint          maxmap;
    uint8_t       replybuf[256];
    uint8_t      *data;
    uint          is_buttons = 0;
    uint          maxcur;

    switch (which) {
        case 0:
            req.bkm_which = BKM_WHICH_KEYMAP;
            is_buttons = 0;
            maxcur = 256;
            break;
        case 1:
            req.bkm_which = BKM_WHICH_BUTTONMAP;
            is_buttons = 1;
            maxcur = NUM_BUTTON_SCANCODES;
            break;
        case 2:
            req.bkm_which = BKM_WHICH_DEF_KEYMAP;
            is_buttons = 0;
            maxcur = 256;
            break;
        case 3:
            req.bkm_which = BKM_WHICH_DEF_BUTTONMAP;
            is_buttons = 1;
            maxcur = NUM_BUTTON_SCANCODES;
            break;
        default:
            err_printf("bug: load_keymap_from_bec(%u)\n", which);
            return (1);
    }
    req.bkm_start = 0;
    req.bkm_len   = 0;
    req.bkm_count = 128;

    for (cur = 0; cur < maxcur; ) {
        req.bkm_start = cur;
        rc = send_cmd(BEC_CMD_GET_MAP, &req, sizeof (req),
                      replybuf, sizeof (replybuf), &rlen);
        if (rc != 0) {
            gui_printf("BEC get map fail: %s", bec_err(rc));
            return (1);
        }
        if (rlen < sizeof (*reply)) {
            gui_printf("Got bad map reply from BEC: %u", rlen);
            return (1);
        }
        rlen -= sizeof (bec_keymap_t *);
        if (rlen == 0) {
            gui_printf("BEC map ended early: missing %u and higher", cur);
            return (1);
        }
        reply = (bec_keymap_t *) replybuf;
        data = (void *) (reply + 1);
        maxmap = reply->bkm_len;
        if (maxmap > ARRAY_SIZE(hid_scancode_to_amiga[0]))
            maxmap = ARRAY_SIZE(hid_scancode_to_amiga[0]);
        maxpos = reply->bkm_count;
        if (maxpos > NUM_SCANCODES - cur)
            maxpos = NUM_SCANCODES - cur;
        gui_printf("  Load %u ents at %02x from BEC",
                   reply->bkm_count, reply->bkm_start);
        for (pos = 0; pos < maxpos; pos++) {
            uint cap;
            uint mapped = 0;
            for (map = 0; map < maxmap; map++, data++) {
                if (is_buttons == 0) {
                    hid_scancode_to_amiga[cur][map] = *data;
                } else {
                    hid_button_scancode_to_amiga[cur][map] = *data;
                }
                if (*data == 0x00) {
                    /*
                     * This scancode is special -- could mean either Amiga
                     * backtick key or could be no key at all. Filter out
                     * data which is not a key.
                     *
                     * The only case where 0x00 is treated as a scancode
                     * is when the following value is non-zero. So it's
                     * not valid for the last slot. The exception is if
                     * the map provided only has room for a single slot.
                     */
                    if ((maxmap > 1) &&
                        ((map >= maxmap - 1) || (data[1] == 0x00))) {
                        continue;
                    }
                }
                cap = amiga_scancode_to_capnum[*data];
                if (cap != 0xff) {
                    if ((amiga_key_mapped[cap]++ == 0) && gui_initialized)
                        draw_amiga_key(cap, 0);
                }
                if (*data != 0xff)
                    mapped++;
            }
            if (is_buttons == 0) {
                if ((map == 1) && (hid_scancode_to_amiga[cur][0] == 0x00)) {
                    /* Special case: terminate backtick when maps are 1-byte */
                    hid_scancode_to_amiga[cur][map++] = 0xff;
                }
                for (; map < ARRAY_SIZE(hid_scancode_to_amiga[0]); map++)
                    hid_scancode_to_amiga[cur][map] = 0;
            } else {
                if ((map == 1) &&
                    (hid_button_scancode_to_amiga[cur][0] == 0x00)) {
                    /* Special case: terminate backtick when maps are 1-byte */
                    hid_button_scancode_to_amiga[cur][map++] = 0xff;
                }
                for (; map < ARRAY_SIZE(hid_button_scancode_to_amiga[0]); map++)
                    hid_button_scancode_to_amiga[cur][map] = 0;
            }

            if (maxmap < reply->bkm_len) {
                /* Local per-key map is smaller than BEC map */
                data += reply->bkm_len - maxmap;
            } else if (maxmap > reply->bkm_len) {
                /* Local per-key map is larger than BEC map */
                if (is_buttons == 0) {
                    memset(&hid_scancode_to_amiga[cur][map], 0,
                           maxmap - reply->bkm_len);
                } else {
                    memset(&hid_button_scancode_to_amiga[cur][map], 0,
                           maxmap - reply->bkm_len);
                }
            }
            if (mapped) {
                if (is_buttons == 0) {
                    cap = hid_scancode_to_capnum[cur];
                    if (cap != 0xff) {
                        if ((hid_key_mapped[cap]++ == 0) && gui_initialized)
                            draw_hid_key(cap, 0);
                    }
                } else {
                    hid_button_mapped[cur]++;
                    cap = hid_button_scancode_to_capnum(cur);
                    if (cap != 0xff) {
                        if ((hid_button_mapped[cur] == 1) && gui_initialized)
                            draw_hid_button(cap, 0);
                    }
                }
            }
            cur++;
        }
    }
    gui_printf("Done loading %s%s from BEC",
                ((which == 0) || (which == 1)) ? "" : "default ",
                ((which == 0) || (which == 2)) ? "keymap" : "buttonmap");
    return (0);
}

static uint
save_keymap_to_bec(uint which)
{
    bec_keymap_t *req;
    uint          rc;
    uint          pos;
    uint          maxpos = 60;
    uint          cur;
    uint          rlen;
    uint          map;
    uint          maxmap = 4;
    uint8_t       sendbuf[256];
    uint8_t      *data;
    uint          sendlen;
    uint          maxcur;
    uint          is_buttons;

    req = (void *)sendbuf;
    if (maxpos > (sizeof (sendbuf) - sizeof (*req) - 10) / 4)
        maxpos = (sizeof (sendbuf) - sizeof (*req) - 10);

    switch (which) {
        case 0:
            req->bkm_which = BKM_WHICH_KEYMAP;
            is_buttons = 0;
            maxcur = 256;
            break;
        case 1:
            req->bkm_which = BKM_WHICH_BUTTONMAP;
            is_buttons = 1;
            maxcur = NUM_BUTTON_SCANCODES;
            break;
        default:
            err_printf("bug: save_keymap_to_bec(%u)\n", which);
            return (1);
    }

    for (cur = 0; cur < maxcur; ) {
        data = (void *) (req + 1);
        if (maxpos > maxcur - cur)
            maxpos = maxcur - cur;
        req->bkm_start = cur;
        req->bkm_len   = maxmap;
        req->bkm_count = maxpos;
        for (pos = 0; pos < maxpos; pos++) {
            for (map = 0; map < maxmap; map++)
                if (is_buttons == 0) {
                    *(data++) = hid_scancode_to_amiga[cur][map];
                } else {
                    *(data++) = hid_button_scancode_to_amiga[cur][map];
                }
            cur++;
        }
        sendlen = (uintptr_t) data - (uintptr_t) sendbuf;
        rc = send_cmd(BEC_CMD_SET_MAP, sendbuf, sendlen, NULL, 0, &rlen);
        if (rc != 0) {
            gui_printf("BEC set map fail: %s", bec_err(rc));
            return (1);
        }
        gui_printf("  Save %u ents at %02x to BEC",
                   req->bkm_count, req->bkm_start);
    }
    gui_printf("Done saving %s to BEC", (which == 0) ? "keymap" : "buttonmap");
    return (0);
}

static void
about_program(void)
{
    static const struct EasyStruct about = {
        sizeof (struct EasyStruct),         // es_StructSize
        0,                                  // es_Flags
        "About Becky",                      // es_Title
        "Becky "VERSION"\n\n"               // es_TextFormat
        "The AmigaPCI BEC key and button\n"
        "mapping tool by Chris Hooper.\n"
        "Built "BUILD_DATE"  "BUILD_TIME"\n",
        "Ok"                                // es_GadgetFormat
    };
    EasyRequest(NULL, &about, NULL);
}

#define MAP_TYPE_UNKNOWN 0
#define MAP_TYPE_KEY     1
#define MAP_TYPE_BUTTON  2

static uint
load_keymap_from_file(const char *filename)
{
    char                 *kptr;
    char                 *ptr;
    FILE                 *fp;
    struct FileRequester *req;
    char   linebuf[300];
    char   full_path[300];
    uint   line;
    uint   err_count = 0;

    if (filename != NULL) {
        strcpy(full_path, filename);
    } else {
        req = AllocAslRequestTags(ASL_FileRequest,
                                  ASLFR_Window,        (ULONG)window,
                                  ASLFR_TitleText,     (ULONG)"Load File...",
                                  ASLFR_PositiveText,  (ULONG)"Load",
                                  ASLFR_InitialFile,   (ULONG)"bec_keymap.txt",
//                                ASLFR_InitialDrawer, (ULONG)"RAM:",
                                  TAG_DONE);
        if (req == NULL)
            return (1);
        if (AslRequestTags(req, TAG_DONE) == 0) {
            /* User Canceled */
            return (1);
        }
        strcpy(full_path, req->fr_Drawer);
        AddPart(full_path, req->rf_File, sizeof (full_path));
        FreeAslRequest(req);
    }

    /* Open file for Load... */
    fp = fopen(full_path, "r");
    if (fp == NULL) {
        gui_printf("Failed to open %s", full_path);
        return (1);
    }
    unmap_all_keycaps();

    line = 0;
    while (fgets(linebuf, sizeof (linebuf) - 1, fp) != NULL) {
        linebuf[sizeof (linebuf) - 1] = '\0';
        line++;
        if ((ptr = strchr(linebuf, '#')) != NULL)
            *ptr = '\0';
        if ((ptr = strchr(linebuf, '\n')) != NULL)
            *ptr = '\0';
        if ((ptr = strcasestr(linebuf, "MAP")) != NULL) {
            uint map_type = MAP_TYPE_UNKNOWN;
            if ((kptr = strcasestr(ptr + 3, "KEY")) != NULL) {
                kptr += 3;
                map_type = MAP_TYPE_KEY;
            } else if ((kptr = strcasestr(ptr + 3, "BUTTON")) != NULL) {
                kptr += 6;
                map_type = MAP_TYPE_BUTTON;
            }
            if (map_type != MAP_TYPE_UNKNOWN) {
                int      pos = 0;
                uint     hid_code;
                uint     amiga_code;
                uint     sc_count = 0;
                uint     hcap;
                uint     acap;
                uint8_t *map_ptr;

                if ((sscanf(kptr, "%x%n", &hid_code, &pos) != 1) ||
                    ((kptr[pos] != ' ') && (kptr[pos] != '\0')) ||
                    (hid_code > 0xff)) {
invalid_hid_code:
                    err_printf("%u: Invalid HID %sscancode \"%.*s\":\n%s\n",
                               (map_type == MAP_TYPE_KEY) ? "" : "button ",
                               line, pos, kptr, linebuf);
load_parse_error:
                    if (err_count++ > 8) {
                        err_printf("Too many errors; giving up\n");
                        break;
                    }
                    continue;
                }
                if ((map_type == MAP_TYPE_BUTTON) && (hid_code >= 0x40))
                    goto invalid_hid_code;
                map_ptr = (map_type == MAP_TYPE_KEY) ?
                                    hid_scancode_to_amiga[hid_code] :
                                    hid_button_scancode_to_amiga[hid_code];

                ptr = strcasestr(kptr + pos, " TO ");
                if (ptr == NULL) {
                    err_printf("%u: missing \"TO\" in MAP %s command:\n:%s\n",
                               (map_type == MAP_TYPE_KEY) ? "KEY" : "BUTTON",
                               line, linebuf);
                    goto load_parse_error;
                }
                kptr = ptr + 4;
                amiga_code = 0xff;
                while (sc_count < 5) {
                    /* Skip whitespace and comma separators */
                    while ((*kptr == ' ') || (*kptr == '\t') || (*kptr == ','))
                        kptr++;

                    if ((sscanf(kptr, "%x%n", &amiga_code, &pos) != 1) ||
                        ((kptr[pos] != ' ') && (kptr[pos] != '\t') &&
                         (kptr[pos] != ',') && (kptr[pos] != '\0')) ||
                        (amiga_code > 0xff)) {
                        ptr = kptr;
                        while (*ptr == ' ')
                            ptr++;
                        if (*ptr == '\0')
                            break;  // End of line

                        err_printf("%u: Invalid Amiga scancode \"%.*s\":\n%s\n",
                                   line, pos, kptr, linebuf);
                        goto load_parse_error;
                    }
                    if (sc_count == 4) {
                        err_printf("%u: too many Amiga scancodes at %x:\n%s\n",
                                   line, amiga_code, linebuf);
                        goto load_parse_error;
                    }
                    map_ptr[sc_count] = amiga_code;
                    acap = amiga_scancode_to_capnum[amiga_code];
                    if (acap != 0xff) {
                        if (amiga_key_mapped[acap]++ == 0)
                            draw_amiga_key(acap, 0);
                    }
                    sc_count++;
                    kptr += pos;
                }

                /* Handle special cases */
                if (sc_count == 0) {
                    /* No mapping provided: terminate this mapping */
                    map_ptr[sc_count++] = 0xff;
                } else if ((sc_count <= 3) && (map_ptr[sc_count] == 0x00)) {
                    /*
                     * Last mapping provided was Amiga backtick scancode:
                     * terminate mapping by adding 0xff.
                     */
                    map_ptr[sc_count++] = 0xff;
                }
                while (sc_count < 4)
                    map_ptr[sc_count++] = 00;

                if (map_type == MAP_TYPE_KEY) {
                    /* HID scancode */
                    hcap = hid_scancode_to_capnum[hid_code];
                    if (hcap != 0xff) {
                        if (hid_key_mapped[hcap]++ == 0)
                            draw_hid_key(hcap, 0);
                    }
                } else if (map_type == MAP_TYPE_BUTTON) {
                    /* HID button scancode */
                    if (hid_button_mapped[hid_code]++ == 0) {
                        hcap = hid_button_scancode_to_capnum(hid_code);
                        if (hcap != 0xff)
                            draw_hid_button(hcap, 0);
                    }
                }
            }
        }
    }
    fclose(fp);
    gui_printf("Done loading keymap from %s", full_path);
    return (0);
}

static void
save_single_keymap(FILE *fp, uint map_type, uint8_t scancode,
                   uint8_t *amiga_scancodes)
{
    uint  map;
    uint  printed = 0;
    char  amiga_buf[128];
    char *amiga_buf_ptr = amiga_buf;

    for (map = 0; map < 4; map++) {
        uint8_t amiga_scancode = amiga_scancodes[map];
        if (amiga_scancode == 0xff)
            break;  // End of list
        if (amiga_scancode == 0x00) {
            /* Special case for Amiga backtick */
            uint pos = map + 1;
            for (; pos < 4; pos++)
                if (amiga_scancodes[pos] != 0x00)
                    break;
            if (pos == 4)
                break;  // End of list
        }
        if (printed++ == 0)
            fprintf(fp, "MAP %s %02x TO",
                    (map_type == 1) ? "KEY" : "BUTTON", scancode);
        fprintf(fp, " %02x", amiga_scancode);

        if (amiga_scancode != 0xff) {
            sprintf(amiga_buf_ptr, " \"%s\"",
                    amiga_scancode_to_long_name[amiga_scancode]);
            amiga_buf_ptr += strlen(amiga_buf_ptr);
        }
    }

    if (printed) {
        fprintf(fp, "  #");
        if (map_type == MAP_TYPE_KEY) {
            /* HID scancode */
            fprintf(fp, " %s", hid_scancode_to_long_name[scancode]);
        } else {
            /* HID button scancode */
            if (scancode < 0x1c) {
                fprintf(fp, " Button %u", scancode + 1);
            } else if (scancode < 0x20) {
                fprintf(fp, " Joystick %s",
                        (scancode == 0x1c) ? "Up" :
                        (scancode == 0x1d) ? "Down" :
                        (scancode == 0x1e) ? "Left" : "Right");
            } else {
                fprintf(fp, " Joystick Button %u", scancode + 1 - 0x20);
            }
        }
        *amiga_buf_ptr = '\0';
        fprintf(fp, " ->%s\n", amiga_buf);
    }
}

static uint
save_keymap_to_file(const char *filename)
{
    FILE                 *fp;
    struct FileRequester *req;
    struct tm            *timeinfo;
    time_t now;
    char   full_path[300];
    uint   cur;

    if (filename != NULL) {
        strcpy(full_path, filename);
    } else {
        req = AllocAslRequestTags(ASL_FileRequest,
                                  ASLFR_Window,        (ULONG)window,
                                  ASLFR_TitleText,     (ULONG)"Save File As...",
                                  ASLFR_PositiveText,  (ULONG)"Save",
                                  ASLFR_InitialFile,   (ULONG)"bec_keymap.txt",
                                  ASLFR_DoSaveMode,    TRUE,
//                                ASLFR_InitialDrawer, (ULONG)"RAM:",
                                  TAG_DONE);
        if (req == NULL)
            return (1);

        if (AslRequestTags(req, TAG_DONE) == 0) {
            /* User Canceled */
            return (1);
        }
        strcpy(full_path, req->fr_Drawer);
        AddPart(full_path, req->rf_File, sizeof (full_path));
        FreeAslRequest(req);
    }

    /* Open file for Save... */
    fp = fopen(full_path, "w");
    if (fp == NULL) {
        gui_printf("Failed to open %s", full_path);
        return (1);
    }
    time(&now);
    timeinfo = localtime(&now);
    fprintf(fp, "#\n"
                "# BEC HID keymap by Becky %s\n"
                "# %s"  // asctime() emits newline
                "#\n"
                "\n"
                "#\n"
                "# Use \"MAP KEY\" to map USB HID key scancode to Amiga "
                    "scancode.\n"
                "# Use \"MAP BUTTON\" to map mouse / joystick button number "
                    "to Amiga scancode.\n"
                "# Up to four Amiga scancodes may be specified from a single "
                    "key or button.\n"
                "# Amiga scancode 00 must always be followed by invalid "
                    "scancode ff when last.\n"
                "#\n"
                "\n",
                VERSION, asctime(timeinfo));
    for (cur = 0; cur < ARRAY_SIZE(hid_scancode_to_amiga); cur++)
        save_single_keymap(fp, 1, cur, hid_scancode_to_amiga[cur]);

    for (cur = 0; cur < ARRAY_SIZE(hid_button_scancode_to_amiga); cur++)
        save_single_keymap(fp, 2, cur, hid_button_scancode_to_amiga[cur]);

    fclose(fp);
    gui_printf("Done saving keymap to %s", full_path);
    return (0);
}

static uint
load_keymap_from_file_or_bec(const char *filename)
{
    uint rc;
    if ((filename == NULL) || (strcasecmp(filename, "BEC") == 0)) {
        rc = load_keymap_from_bec(0) ||
             load_keymap_from_bec(1);
    } else if (strcasecmp(filename, "DEFAULT") == 0) {
        rc = load_keymap_from_bec(2) ||
             load_keymap_from_bec(3);
    } else {
        rc = load_keymap_from_file(filename);
    }
    return (rc);
}

static uint
save_keymap_to_file_or_bec(const char *filename)
{
    uint rc;
    if ((filename == NULL) || (strcasecmp(filename, "BEC") == 0)) {
        rc = save_keymap_to_bec(0) ||
             save_keymap_to_bec(1);
    } else {
        rc = save_keymap_to_file(filename);
    }
    return (rc);
}


static uint8_t polling_for_scancodes;

/*
 * stop_poll_for_hid_scancodes() ends keyboard scancode capture.
 */
static void
stop_poll_for_hid_scancodes(void)
{
    uint8_t    replybuf[48];
    uint       rlen;
    bec_poll_t req;

    if (polling_for_scancodes) {
        polling_for_scancodes = 0;
        req.bkm_source  = BKM_SOURCE_NONE;
        req.bkm_count   = 16;
        req.bkm_timeout = 0;

        /* Flush previous poll */
        (void) send_cmd(BEC_CMD_POLL_INPUT, &req, sizeof (req),
                        replybuf, sizeof (replybuf), &rlen);
    }
}

static void
poll_for_hid_scancodes(void)
{
    uint            rc;
    uint            rlen;
    uint8_t         replybuf[48];
    uint            pos;
    bec_poll_t      req;
    bec_poll_t     *rep = (bec_poll_t *)replybuf;
    uint8_t        *data;
    static uint8_t  err_timeout;
    static uint8_t  err_count;

    if (edit_key_mapping_mode) {
        /* Flush previous poll */
        stop_poll_for_hid_scancodes();
        return;
    }
    polling_for_scancodes = 1;

    if (err_timeout) {
        err_timeout--;
        return;
    }
    req.bkm_source  = BKM_SOURCE_HID_SCANCODE;
    req.bkm_count   = 16;
    req.bkm_timeout = 700;  // msec timeout if not polling_for_scancodes again

    rep->bkm_count = 0;
    rc = send_cmd(BEC_CMD_POLL_INPUT, &req, sizeof (req),
                  replybuf, sizeof (replybuf), &rlen);
    if (rc != 0) {
        if (err_count < 7)
            err_count++;
        err_timeout = (1 << err_count);  // Exponential backoff
        gui_printf("BEC poll fail rc=%d", rc);
        return;
    }
    if (rep->bkm_count == 0)
        return;

    data = (uint8_t *) (rep + 1);
    for (pos = 0; pos < rep->bkm_count * 2; pos += 2) {
        if (data[pos + 1] & KEYCAP_BUTTON) {
            hid_rawbutton(data[pos], data[pos + 1]);
        } else {
            hid_rawkey(data[pos], data[pos + 1]);
            if (((data[pos + 1] & KEYCAP_UP) == 0) &&
                ((data[pos] == HS_MEDIA_S_UP) ||
                 (data[pos] == HS_MEDIA_S_DOWN) ||
                 (data[pos] == HS_MEDIA_BACK) ||
                 (data[pos] == HS_MEDIA_FWD))) {
                /* Mouse wheel scancodes don't issue "key up" */
                hid_rawkey(data[pos], KEYCAP_UP);
            }
        }
    }
}

#if 0
static void
showdatestamp(struct DateStamp *ds, uint usec)
{
    struct DateTime  dtime;
    char             datebuf[32];
    char             timebuf[32];

    dtime.dat_Stamp.ds_Days   = ds->ds_Days;
    dtime.dat_Stamp.ds_Minute = ds->ds_Minute;
    dtime.dat_Stamp.ds_Tick   = ds->ds_Tick;
    dtime.dat_Format          = FORMAT_DOS;
    dtime.dat_Flags           = 0x0;
    dtime.dat_StrDay          = NULL;
    dtime.dat_StrDate         = datebuf;
    dtime.dat_StrTime         = timebuf;
    DateToStr(&dtime);
    printf("%s %s.%06u\n", datebuf, timebuf, usec);
}

static void
showsystime(uint sec, uint usec)
{
    struct DateStamp ds;
    uint min  = sec / 60;
    uint day  = min / (24 * 60);

    ds.ds_Days   = day;
    ds.ds_Minute = min % (24 * 60);
    ds.ds_Tick   = (sec % 60) * TICKS_PER_SECOND;
    showdatestamp(&ds, usec);
}
#endif

static void
handle_win(void)
{
    /* Event loop */
    struct IntuiMessage *msg;
    ULONG class;
    UWORD icode;

    while (!program_done) {
        WaitPort(window->UserPort);
//      while ((msg = (struct IntuiMessage *)GetMsg(window->UserPort))) {
        while ((msg = GT_GetIMsg(window->UserPort))) {
            class = msg->Class;
            icode = msg->Code;

//          printf("class %x\n", class);
            switch (class) {
                case IDCMP_CLOSEWINDOW:
                    program_done = TRUE;
                    break;
                case IDCMP_INTUITICKS:
                    if (capture_hid_scancodes)
                        poll_for_hid_scancodes();
                    break;
                case IDCMP_INACTIVEWINDOW:
                    stop_poll_for_hid_scancodes();
                    break;
                case IDCMP_MENUPICK: {
                    int cnt;
                    stop_poll_for_hid_scancodes();
                    user_usage_hint_clear();
                    USHORT menu_item = icode;
                    for (cnt = 0; (menu_item != MENUNULL) && (cnt < 9); cnt++) {
                        /*
                         *    Menu number is bits 0..4
                         *    Item number is bits 5..10
                         * Subitem number is bits 11..15
                         */
                        ULONG menu_num = MENUNUM(menu_item);
                        ULONG item_num = ITEMNUM(menu_item);
                        // ULONG subitem_num = SUBITEMNUM(menu_item);
                        struct MenuItem *item = ItemAddress(menus, menu_item);
                        uint  checked = 0;
                        ULONG chk_item = 0;
                        if ((item != NULL) && (item->Flags & CHECKED))
                            checked = 1;
#if 0
                        printf("item=%x menu=%x item=%x\n",
                               menu_item, menu_num, item_num);
#endif
                        switch (menu_num) {
                            case MENU_NUM_FILE:
                                switch (item_num) {
                                    case MENU_FILE_ABOUT:
                                        about_program();
                                        break;
                                    case MENU_FILE_LOAD:
                                        load_keymap_from_file(NULL);
                                        break;
                                    case MENU_FILE_SAVE:
                                        save_keymap_to_file(NULL);
                                        break;
                                    case MENU_FILE_QUIT:
                                        program_done = TRUE;
                                        break;
                                }
                                break;
                            case MENU_NUM_BEC:
                                switch (item_num) {
                                    case MENU_BEC_LOAD:
                                        gui_printf("Loading from BEC");
                                        unmap_all_keycaps();
                                        load_keymap_from_bec(0);
                                        load_keymap_from_bec(1);
                                        break;
                                    case MENU_BEC_SAVE:
                                        gui_printf("Saving to BEC");
                                        save_keymap_to_bec(0);
                                        save_keymap_to_bec(1);
                                        break;
                                    case MENU_BEC_DEFAULTS:
                                        gui_printf("Loading defaults");
                                        unmap_all_keycaps();
                                        load_keymap_from_bec(2);
                                        load_keymap_from_bec(3);
                                        break;
                                }
                                break;
                            case MENU_NUM_MODE:
                                unhighlight_last_key();
                                switch (item_num) {
                                    case MENU_MODE_AMIGA_SCANCODE:
                                        capture_hid_scancodes = 0;
                                        if (!checked) {
                                            /* Force it to remain checked */
                                            chk_item = ITEMNUM_AMIGA_SCANCODE;
set_menu_checked:
                                            item = ItemAddress(menus, chk_item);
                                            if (item != NULL) {
                                                ClearMenuStrip(window);
                                                item->Flags |= CHECKED;
                                                ResetMenuStrip(window, menus);
                                            } else {
                                                err_printf("menuitem %x NULL\n",
                                                           chk_item);
                                            }
                                        }
                                        break;
                                    case MENU_MODE_HID_SCANCODE:
                                        capture_hid_scancodes = 1;
                                        if (!checked) {
                                            /* Force it to remain checked */
                                            chk_item = ITEMNUM_HID_SCANCODE;
                                            goto set_menu_checked;
                                        }
                                        break;
                                    case MENU_MODE_AMIGA_TO_HID:
                                        key_mapping_mode =
                                                KEY_MAPPING_MODE_AMIGA_TO_HID;
                                        if (!checked) {
                                            /* Force it to remain checked */
                                            chk_item = ITEMNUM_AMIGA_TO_HID;
                                            goto set_menu_checked;
                                        }
                                        break;
                                    case MENU_MODE_HID_TO_AMIGA:
                                        key_mapping_mode =
                                                KEY_MAPPING_MODE_HID_TO_AMIGA;
                                        if (!checked) {
                                            /* Force it to remain checked */
                                            chk_item = ITEMNUM_HID_TO_AMIGA;
                                            goto set_menu_checked;
                                        }
                                        break;
                                    case MENU_MODE_LIVE_KEYS:
                                        key_mapping_mode =
                                                KEY_MAPPING_MODE_LIVEKEYS;
                                        if (!checked) {
                                            /* Force it to remain checked */
                                            chk_item = ITEMNUM_LIVEKEYS;
                                            goto set_menu_checked;
                                        }
                                        break;
                                    case MENU_MODE_LIVE_KEYS_MAP:
                                        key_mapping_mode =
                                            KEY_MAPPING_MODE_LIVEKEYS_MAPPED;
                                        if (!checked) {
                                            /* Force it to remain checked */
                                            chk_item = ITEMNUM_LIVEKEYS_MAP;
                                            goto set_menu_checked;
                                        }
                                        break;
                                    case MENU_MODE_MOUSE_BUTTONS:
                                        button_mapping_mode = BUTMAP_MODE_MOUSE;
                                        draw_hid_buttons();
                                        if (!checked) {
                                            /* Force it to remain checked */
                                            chk_item = ITEMNUM_MOUSE_BUTTONS;
                                            goto set_menu_checked;
                                        }
                                        break;
                                    case MENU_MODE_JOY_BUTTONS:
                                        button_mapping_mode =
                                                        BUTMAP_MODE_JOYSTICK;
                                        draw_hid_buttons();
                                        if (!checked) {
                                            /* Force it to remain checked */
                                            chk_item = ITEMNUM_JOY_BUTTONS;
                                            goto set_menu_checked;
                                        }
                                        break;
                                }
                                break;
                            case MENU_NUM_KEY:
                                switch (item_num) {
                                    case MENU_KEY_CUSTOM_MAPPING: {
                                        uint pos = 0;
                                        if (edit_key_mapping_mode)
                                            break;  // Already in mapping mode
                                        switch (key_mapping_mode) {
                                            case KEY_MAPPING_MODE_AMIGA_TO_HID:
                                                pos = 0x20;  // Edit HID
                                                break;
                                            case KEY_MAPPING_MODE_HID_TO_AMIGA:
                                                pos = 0x00;  // Edit Amiga
                                                break;
                                            default:
                                                /* Last keypress determines */
                                                switch (editbox_update_mode) {
                                                    case 0:
                                                        pos = 0x20;  // HID
                                                        break;
                                                    case 1:
                                                        pos = 0x10;  // Buttons
                                                        break;
                                                    case 2:
                                                        pos = 0x00;  // Amiga
                                                        break;
                                                }
                                                break;
                                        }
                                        editbox_set_cursor_position(pos);
                                        break;
                                    }
                                    case MENU_KEY_REMOVE_MAPPINGS:
                                        remove_single_key_mappings();
                                        break;
                                    case MENU_KEY_REDRAW_KEYS:
                                        draw_win();
                                        break;
                                    case MENU_KEY_ANSI_LAYOUT:
                                        is_ansi_layout = checked;
                                        draw_win();
                                        break;
                                }
                                break;
                        }
                        /*
                         * Get the next menu item if it was part of a
                         * selection list
                         *
                         * Note: Actual next item is usually in a
                         *       structure passed via message
                         */
                        menu_item = msg->Qualifier;
                        break;
                    }
                    user_usage_hint_show(1);
                    break;
                }
                case IDCMP_MOUSEMOVE:
                    mouse_move(window->GZZMouseX, window->GZZMouseY);
                    break;
                case IDCMP_MOUSEBUTTONS:
                    switch (icode) {
                        case SELECTDOWN:
                            mouse_button_press(1);
                            break;
                        case SELECTUP:
                            mouse_button_press(0);
                            break;
                        case MENUDOWN:
                        case MENUUP:
                            break;
                        case MIDDLEDOWN:
                            amiga_rawkey(ASE_BUTTON_2, KEYCAP_DOWN);
                            break;
                        case MIDDLEUP:
                            amiga_rawkey(ASE_BUTTON_2, KEYCAP_UP);
                            break;
                    }
                    break;
                case IDCMP_NEWSIZE:
//                  printf("newsize %u %u\n", window->Width, window->Height);
//                  BeginRefresh(window);
                    draw_win();
//                  EndRefresh(window, TRUE);
                    break;
                case IDCMP_RAWKEY:
                    amiga_rawkey(icode & ~0x80, icode & 0x80);
                    if ((icode == AS_WHEEL_UP) ||
                        (icode == AS_WHEEL_DOWN) ||
                        (icode == AS_WHEEL_LEFT) ||
                        (icode == AS_WHEEL_RIGHT)) {
                        /* Mouse wheel scancodes don't issue "key up" */
                        amiga_rawkey(icode, 0x80);
                    }
                    break;
                case IDCMP_REFRESHWINDOW:
                    BeginRefresh(window);
                    draw_win();
                    EndRefresh(window, TRUE);
                    break;
            }
            GT_ReplyIMsg(msg);
        }
    }
    stop_poll_for_hid_scancodes();
}

static void
generate_scancode_to_capnum(void)
{
    uint cap;
    memset(amiga_scancode_to_capnum, 0xff, sizeof (amiga_scancode_to_capnum));
    memset(hid_scancode_to_capnum, 0xff, sizeof (hid_scancode_to_capnum));

    for (cap = 0; cap < ARRAY_SIZE(amiga_keypos); cap++)
        amiga_scancode_to_capnum[amiga_keypos[cap].scancode] = cap;

    for (cap = 0; cap < ARRAY_SIZE(hid_keypos); cap++)
        hid_scancode_to_capnum[hid_keypos[cap].scancode] = cap;
}

int
main(int argc, char *argv[])
{
    uint8_t flag_mapamiga = 0;
    uint8_t flag_maphid = 0;
    uint8_t flag_livekeys = 0;
    char *load_filename = NULL;
    char *save_filename = NULL;

    generate_scancode_to_capnum();
    if (argc > 0) {
        /* Started by AmigaOS CLI */
        int arg;
        for (arg = 1; arg < argc; arg++) {
            const char *ptr;
            ptr = long_to_short(argv[arg], long_to_short_main,
                                ARRAY_SIZE(long_to_short_main));
            if (*ptr == '-') {
                for (++ptr; *ptr != '\0'; ptr++) {
                    switch (*ptr) {
                        case 'C':  // capamiga
                            capture_hid_scancodes = 0;
                            break;
                        case 'c':  // caphid
                            capture_hid_scancodes = 1;
                            break;
                        case 'd':  // debug
                            flag_debug++;
                            break;
                        case 'e':  // ESC exit
                            enable_esc_exit = 0;
                            break;
                        case 'h':  // Help
                            usage();
                            exit(0);
                            break;
                        case 'i':  // iso
                            is_ansi_layout = 0;
                            break;
                        case 'j':  // justlive
                            flag_livekeys = 1;
                            break;
                        case 'k':
                            bec_msg_interface++;
                            break;
                        case 'l':  // load
                            if (++arg >= argc) {
                                err_printf("-l option requires an argument, one"
                                           " of bec, default, or a filename\n");
                                exit(1);
                            }
                            load_filename = argv[arg];
                            break;
                        case 'M':  // mapamiga
                            flag_mapamiga = 1;
                            break;
                        case 'm':  // maphid
                            flag_maphid = 1;
                            break;
                        case 's':  // save
                            if (++arg >= argc) {
                                err_printf("-s option requires an argument, one"
                                           " of bec or a filename\n");
                                exit(1);
                            }
                            save_filename = argv[arg];
                            break;
                        default:
                            err_printf("Unknown argument %s\n", ptr);
                            usage();
                            exit(1);
                    }
                }
            } else {
                err_printf("Error: unknown argument %s\n", ptr);
                usage();
                exit(1);
            }
        }
    } else {
        /* Started by Workbench */
        struct WBStartup *wbs = (struct WBStartup *) argv;
        (void) wbs;
    }
    if (save_filename != NULL) {
        /* Not going to start the GUI for this */
        if (load_filename == NULL) {
            err_printf("You must also supply a load filename (-l)\n");
            exit(1);
        }
        if (load_keymap_from_file_or_bec(load_filename) != 0)
            exit(1);

        save_keymap_to_file_or_bec(save_filename);
        exit(0);
    }

    if (flag_maphid)
        key_mapping_mode = KEY_MAPPING_MODE_HID_TO_AMIGA;
    else if (flag_mapamiga)
        key_mapping_mode = KEY_MAPPING_MODE_AMIGA_TO_HID;
    else if (flag_livekeys)
        key_mapping_mode = KEY_MAPPING_MODE_LIVEKEYS;
    else
        key_mapping_mode = KEY_MAPPING_MODE_LIVEKEYS_MAPPED;

    if (OpenAmigaLibraries()) {
        window = OpenAWindow();
        if (window != NULL) {
            rp = window->RPort;
            select_pens();
            open_font();
            create_menu();
            gui_initialized = 1;
            if (draw_win()) {
                load_keymap_from_file_or_bec(load_filename);
                handle_win();
            }
            gui_initialized = 0;
            close_menu();
            close_font();
            CloseWindow(window);
        } else {
            err_printf("Failed to open window.\\n");
        }
    }
    CloseAmigaLibraries();
    exit(EXIT_SUCCESS);
}
