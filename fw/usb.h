/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * USB handling.
 */

#ifndef _USB_H
#define _USB_H

#define USB0_BASE USB_OTG_FS_BASE
#define USB1_BASE USB_OTG_HS_BASE

#define USB_ROOT_PORT_MASK_0    1
#define USB_ROOT_PORT_MASK_1    2
#define USB_ROOT_PORT_MASK_BOTH (USB_ROOT_PORT_MASK_0 | USB_ROOT_PORT_MASK_1)

#define USB_SET_POWER_ON  1
#define USB_SET_POWER_OFF 0

void usb_init(void);
void usb_poll(void);
void usb_ls(uint verbose);
void usb_show_regs(void);
void usb_show_stats(void);
#ifdef USB_TRACE
void usb_trace_show(void);
void usb_trace_cmd(int argc, char * const *argv);
#endif
void usb_set_power(int state, uint portmask);
uint usb_power_port_mask(uint portmask);
void usb_shutdown(uint mode);
int usb_power_cycle(const char *path);

extern uint usb_debug_mask;
extern uint usb_keyboard_terminal;
extern uint8_t usb_is_powered;
extern volatile uint8_t usb_keyboard_count;
extern volatile uint8_t usb_mouse_count;
extern volatile uint8_t usb_joystick_count;

#endif /* _USB_H */
