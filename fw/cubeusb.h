/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * ST-Micro CubeMX USB Host library handling.
 */

#ifndef _CUBEUSB_H
#define _CUBEUSB_H

void cubeusb_init(void);
void cubeusb_shutdown(void);
void cubeusb_poll(void);

struct _USBH_HandleTypeDef;
struct _USBH_HandleTypeDef *cubeusb_msc_get(uint index);
struct _USBH_HandleTypeDef *cubeusb_mtp_get(uint index);

#endif /* _CUBEUSB_H */
