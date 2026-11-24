/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * USB mass storage (MSC) block device access.
 *
 * Reads and writes are started asynchronously in the MSC class driver
 * and advanced by the normal USB host polling (cubeusb_poll(), from
 * main_poll()). The functions here wait for completion by running
 * main_poll(), so the keyboard, mouse and the rest of the firmware keep
 * running during a transfer. They must not be called from interrupt
 * context or from inside the USB host stack.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "printf.h"
#include "timer.h"
#include "uart.h"
#include "cubeusb.h"
#include "usb_msc.h"
#include <usbh_core.h>
#include <usbh_msc.h>

#define USB_MSC_READY_WAIT_MS   2000   // Wait for device to become idle
#define USB_MSC_XFER_BASE_MS    5000   // Transfer timeout ...
#define USB_MSC_XFER_BLOCK_MS     50   // ... plus this per block

/* Strip leading and trailing spaces (SCSI INQUIRY strings are padded) */
static void
trim(char *str)
{
    char *start = str;
    size_t len;

    while (*start == ' ')
        start++;
    len = strlen(start);
    while ((len > 0) && (start[len - 1] == ' '))
        len--;
    memmove(str, start, len);
    str[len] = '\0';
}

static MSC_HandleTypeDef *
msc_handle(USBH_HandleTypeDef *phost)
{
    return ((MSC_HandleTypeDef *) phost->pActiveClass->pData);
}

uint
usb_msc_count(void)
{
    uint count = 0;
    while (cubeusb_msc_get(count) != NULL)
        count++;
    return (count);
}

uint
usb_msc_luns(uint dev)
{
    USBH_HandleTypeDef *phost = cubeusb_msc_get(dev);
    if (phost == NULL)
        return (0);
    return (msc_handle(phost)->max_lun);
}

int
usb_msc_info(uint dev, uint lun, usb_msc_info_t *info)
{
    USBH_HandleTypeDef *phost = cubeusb_msc_get(dev);
    MSC_HandleTypeDef  *MSC_Handle;
    MSC_LUNTypeDef     *unit;

    if (phost == NULL)
        return (USB_MSC_NODEV);
    MSC_Handle = msc_handle(phost);
    if (lun >= MSC_Handle->max_lun)
        return (USB_MSC_NODEV);
    unit = &MSC_Handle->unit[lun];

    memset(info, 0, sizeof (*info));
    info->blocks     = unit->capacity.block_nbr;
    info->block_size = unit->capacity.block_size;
    info->ready      = (phost->gState == HOST_CLASS) &&
                       (MSC_Handle->state != MSC_INIT) &&
                       (unit->error == MSC_OK);
    memcpy(info->vendor,   unit->inquiry.vendor_id,   sizeof (info->vendor) - 1);
    memcpy(info->product,  unit->inquiry.product_id,  sizeof (info->product) - 1);
    memcpy(info->revision, unit->inquiry.revision_id, sizeof (info->revision) - 1);
    trim(info->vendor);
    trim(info->product);
    trim(info->revision);
    strncpy(info->path, usb_get_path(phost, NULL, -1), sizeof (info->path) - 1);
    return (USB_MSC_OK);
}

/*
 * usb_msc_rw() - read or write blocks, waiting for completion.
 *
 * The device handle may be freed by main_poll() if the device is
 * unplugged, so after every poll the MSC de-init counter is checked
 * before the handle is used again.
 */
static int
usb_msc_rw(uint dev, uint lun, uint32_t lba, void *buf, uint count,
           uint write)
{
    USBH_HandleTypeDef *phost = cubeusb_msc_get(dev);
    USBH_StatusTypeDef  st;
    uint32_t gen = USBH_MSC_DeInitCount;
    uint64_t deadline;

    if (phost == NULL)
        return (USB_MSC_NODEV);
    if ((count == 0) || (count > 0xffff) || (buf == NULL))
        return (USB_MSC_BAD_PARAM);
    if (lun >= msc_handle(phost)->max_lun)
        return (USB_MSC_NODEV);

    /* Wait for the device to finish init, or a previous error recovery */
    deadline = timer_tick_plus_msec(USB_MSC_READY_WAIT_MS);
    for (;;) {
        if (write)
            st = USBH_MSC_StartWrite(phost, lun, lba, buf, count);
        else
            st = USBH_MSC_StartRead(phost, lun, lba, buf, count);
        if (st == USBH_OK)
            break;
        if (st != USBH_BUSY)
            return (USB_MSC_IO_ERROR);
        if (timer_tick_has_elapsed(deadline))
            return (USB_MSC_NOT_READY);
        main_poll();
        if (gen != USBH_MSC_DeInitCount)
            return (USB_MSC_REMOVED);
    }

    deadline = timer_tick_plus_msec(USB_MSC_XFER_BASE_MS +
                                    USB_MSC_XFER_BLOCK_MS * count);
    for (;;) {
        main_poll();
        if (gen != USBH_MSC_DeInitCount)
            return (USB_MSC_REMOVED);
        st = USBH_MSC_RdWrStatus(phost);
        if (st != USBH_BUSY)
            break;
        if (timer_tick_has_elapsed(deadline)) {
            USBH_MSC_RdWrAbort(phost);
            return (USB_MSC_TIMEOUT);
        }
    }
    return ((st == USBH_OK) ? USB_MSC_OK : USB_MSC_IO_ERROR);
}

int
usb_msc_read(uint dev, uint lun, uint32_t lba, void *buf, uint count)
{
    return (usb_msc_rw(dev, lun, lba, buf, count, 0));
}

int
usb_msc_write(uint dev, uint lun, uint32_t lba, const void *buf, uint count)
{
    /* The OUT path only reads from buf */
    return (usb_msc_rw(dev, lun, lba, (void *) (uintptr_t) buf, count, 1));
}

/*
 * usb_msc_generation() - changes whenever any mass storage device is
 *                        removed, so callers can tell that the device
 *                        numbering (or the media) may have changed.
 */
uint32_t
usb_msc_generation(void)
{
    return (USBH_MSC_DeInitCount);
}

const char *
usb_msc_strerror(int rc)
{
    switch (rc) {
        case USB_MSC_OK:        return ("OK");
        case USB_MSC_NODEV:     return ("no such device");
        case USB_MSC_NOT_READY: return ("device not ready");
        case USB_MSC_IO_ERROR:  return ("I/O error");
        case USB_MSC_TIMEOUT:   return ("timeout");
        case USB_MSC_REMOVED:   return ("device removed");
        case USB_MSC_BAD_PARAM: return ("invalid parameter");
        default:                return ("unknown error");
    }
}

void
usb_msc_list(void)
{
    uint dev;
    uint lun;
    usb_msc_info_t info;

    for (dev = 0; cubeusb_msc_get(dev) != NULL; dev++) {
        uint luns = usb_msc_luns(dev);
        if (luns == 0) {
            printf("%u: initializing\n", dev);
            continue;
        }
        for (lun = 0; lun < luns; lun++) {
            if (usb_msc_info(dev, lun, &info) != USB_MSC_OK)
                continue;
            printf("%u.%u: USB%-6s %-8s %-16s %-4s ", dev, lun, info.path,
                   info.vendor, info.product, info.revision);
            if (info.ready) {
                uint64_t bytes = (uint64_t) info.blocks * info.block_size;
                printf("%lu x %u (%lu MB)\n", info.blocks, info.block_size,
                       (uint32_t) (bytes >> 20));
            } else {
                printf("not ready\n");
            }
        }
    }
    if (dev == 0)
        printf("No USB mass storage devices\n");
}

static void
dump_blocks(const uint8_t *buf, uint len, uint32_t lba, uint block_size)
{
    uint pos;
    uint i;

    for (pos = 0; pos < len; pos += 16) {
        if ((pos % block_size) == 0)
            printf("Block %lu (0x%lx)\n", lba + pos / block_size,
                   lba + pos / block_size);
        printf("%04x:", pos % block_size);
        for (i = 0; i < 16; i++)
            printf("%s%02x", (i == 8) ? "  " : " ", buf[pos + i]);
        printf("  ");
        for (i = 0; i < 16; i++) {
            uint8_t ch = buf[pos + i];
            printf("%c", ((ch >= ' ') && (ch < 0x7f)) ? ch : '.');
        }
        printf("\n");
        if (input_break_pending())
            break;
    }
}

/* Parse "dev" or "dev.lun" */
static int
parse_dev(const char *str, uint *dev, uint *lun)
{
    int pos = 0;
    *lun = 0;
    if ((sscanf(str, "%u%n", dev, &pos) != 1))
        return (-1);
    if (str[pos] == '.') {
        int pos2 = 0;
        if (sscanf(str + pos + 1, "%u%n", lun, &pos2) != 1)
            return (-1);
        pos += pos2 + 1;
    }
    return ((str[pos] == '\0') ? 0 : -1);
}

static int
parse_uint(const char *str, uint32_t *value)
{
    int ival;
    if ((scan_int(str, &ival) != RC_SUCCESS) || (ival < 0))
        return (-1);
    *value = (uint32_t) ival;
    return (0);
}

#define USB_MSC_CMD_MAX_BYTES   (16 * 1024)

/*
 * usb_msc_read_cmd() - raw block read command ("fs read")
 *
 * argv[0] is the command name, then
 *     <dev[.lun]> <lba> [<count>] [q]
 * With "q", only the transfer rate is reported.
 */
rc_t
usb_msc_read_cmd(int argc, char * const *argv)
{
    uint     dev;
    uint     lun;
    uint32_t lba;
    uint32_t count = 1;
    uint     quiet = 0;
    uint8_t *buf;
    usb_msc_info_t info;
    uint64_t start;
    uint64_t usec;
    uint32_t bytes;
    int      rc;

    if (argc < 3) {
        printf("%s requires <dev[.lun]> <lba> [<count>] [q]\n", argv[0]);
        return (RC_USER_HELP);
    }
    if (parse_dev(argv[1], &dev, &lun) != 0) {
        printf("Invalid device %s\n", argv[1]);
        return (RC_BAD_PARAM);
    }
    if (parse_uint(argv[2], &lba) != 0) {
        printf("Invalid block %s\n", argv[2]);
        return (RC_BAD_PARAM);
    }
    if ((argc > 3) && (strcmp(argv[3], "q") != 0)) {
        if (parse_uint(argv[3], &count) != 0) {
            printf("Invalid count %s\n", argv[3]);
            return (RC_BAD_PARAM);
        }
    }
    if ((argc > 3) && (strcmp(argv[argc - 1], "q") == 0))
        quiet = 1;

    rc = usb_msc_info(dev, lun, &info);
    if (rc != USB_MSC_OK) {
        printf("USB MSC %u.%u: %s\n", dev, lun, usb_msc_strerror(rc));
        return (RC_BAD_PARAM);
    }
    if (!info.ready || (info.block_size == 0)) {
        printf("USB MSC %u.%u: %s\n", dev, lun,
               usb_msc_strerror(USB_MSC_NOT_READY));
        return (RC_FAILURE);
    }
    if ((count == 0) || (count * info.block_size > USB_MSC_CMD_MAX_BYTES)) {
        printf("Count must be 1 to %u\n",
               USB_MSC_CMD_MAX_BYTES / info.block_size);
        return (RC_BAD_PARAM);
    }
    if ((lba >= info.blocks) || (count > info.blocks - lba)) {
        printf("Block range exceeds device size (%lu blocks)\n", info.blocks);
        return (RC_BAD_PARAM);
    }

    bytes = count * info.block_size;
    buf = malloc(bytes);
    if (buf == NULL) {
        printf("Failed to allocate %lu bytes\n", bytes);
        return (RC_FAILURE);
    }

    start = timer_tick_get();
    rc = usb_msc_read(dev, lun, lba, buf, count);
    usec = timer_tick_to_usec(timer_tick_get() - start);

    if (rc != USB_MSC_OK) {
        printf("Read failed: %s\n", usb_msc_strerror(rc));
        free(buf);
        return (RC_FAILURE);
    }
    if (quiet) {
        printf("Read %lu bytes in %lu us", bytes, (uint32_t) usec);
        if (usec != 0)
            printf(" (%lu KB/s)", (uint32_t) ((uint64_t) bytes * 1000000 /
                                              1024 / usec));
        printf("\n");
    } else {
        dump_blocks(buf, bytes, lba, info.block_size);
    }
    free(buf);
    return (RC_SUCCESS);
}
