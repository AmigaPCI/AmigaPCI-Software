/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * USB MTP (Media Transfer Protocol) device access.
 */

#ifndef _USB_MTP_H
#define _USB_MTP_H

#include <stdint.h>

/* Return codes (as for usb_msc.h) */
#define USB_MTP_OK          0
#define USB_MTP_NODEV      -1   // No such device
#define USB_MTP_NOT_READY  -2   // Device still initializing or recovering
#define USB_MTP_IO_ERROR   -3   // Transfer failed
#define USB_MTP_TIMEOUT    -4   // Transaction did not complete in time
#define USB_MTP_REMOVED    -5   // Device was removed during the transaction
#define USB_MTP_BAD_PARAM  -6

#define USB_MTP_DIR_NONE    0   // No data phase
#define USB_MTP_DIR_IN      1   // Data from the device
#define USB_MTP_DIR_OUT     2   // Data to the device

/* Result of a transaction which completed */
typedef struct {
    uint16_t resp;          // MTP response code (0x2001 is OK)
    uint8_t  nresp;         // Number of response parameters
    uint32_t rparam[5];     // Response parameters
    uint32_t len;           // IN: bytes stored in the buffer
    uint32_t total;         // IN: data size the device sent
} usb_mtp_result_t;

typedef struct {
    uint32_t id;            // Storage ID
    uint16_t access;        // PTP_AC_* (0 is read-write)
    uint64_t capacity;      // Bytes
    uint64_t freespace;     // Bytes
    char     desc[40];      // Storage description (UTF-8)
    char     label[40];     // Volume label (UTF-8)
} usb_mtp_storage_t;

typedef struct {
    uint8_t  ready;         // Session open, storage known
    uint8_t  nstorage;      // Number of storage units
    uint16_t ops;           // MTP_OPS_* the device supports
    char     manufacturer[40];
    char     model[40];
    char     path[32];      // USB device path (as shown by "usb ls")
} usb_mtp_info_t;

struct _USBH_HandleTypeDef;

uint usb_mtp_count(void);
int  usb_mtp_info(uint dev, usb_mtp_info_t *info);
int  usb_mtp_storage(uint dev, uint idx, usb_mtp_storage_t *st);
int  usb_mtp_xact(uint dev, uint16_t code, uint nparam,
                  const uint32_t *params, uint dir, void *buf,
                  uint32_t buflen, usb_mtp_result_t *res);
int  usb_mtp_xact_sink(uint dev, uint16_t code, uint nparam,
                       const uint32_t *params, void *buf, uint32_t buflen,
                       void (*sink)(void *arg, const uint8_t *data,
                                    uint32_t len),
                       void *sink_arg, usb_mtp_result_t *res);
const char *usb_mtp_strerror(int rc);
uint32_t usb_mtp_generation(void);
uint32_t usb_mtp_changes(void);
void usb_mtp_list(void);
void usb_mtp_show(struct _USBH_HandleTypeDef *phost);

#endif /* _USB_MTP_H */
