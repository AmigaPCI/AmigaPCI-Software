/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * USB mass storage (MSC) block device access.
 */

#ifndef _USB_MSC_H
#define _USB_MSC_H

#include <stdint.h>
#include "cmdline.h"

/* Return codes of usb_msc_read() / usb_msc_write() / usb_msc_info() */
#define USB_MSC_OK          0
#define USB_MSC_NODEV      -1   // No such device or LUN
#define USB_MSC_NOT_READY  -2   // Device still initializing or recovering
#define USB_MSC_IO_ERROR   -3   // Device reported an error
#define USB_MSC_TIMEOUT    -4   // Transfer did not complete in time
#define USB_MSC_REMOVED    -5   // Device was removed during the transfer
#define USB_MSC_BAD_PARAM  -6

typedef struct {
    uint32_t blocks;            // Number of blocks
    uint16_t block_size;        // Block size in bytes
    uint8_t  ready;             // Unit is ready (media present)
    char     vendor[9];
    char     product[17];
    char     revision[5];
    char     path[32];          // USB device path (as shown by "usb ls")
} usb_msc_info_t;

uint usb_msc_count(void);
uint usb_msc_luns(uint dev);
int  usb_msc_info(uint dev, uint lun, usb_msc_info_t *info);
int  usb_msc_read(uint dev, uint lun, uint32_t lba, void *buf, uint count);
int  usb_msc_write(uint dev, uint lun, uint32_t lba, const void *buf,
                   uint count);
const char *usb_msc_strerror(int rc);
uint32_t usb_msc_generation(void);
void usb_msc_list(void);
rc_t usb_msc_read_cmd(int argc, char * const *argv);

#endif /* _USB_MSC_H */
