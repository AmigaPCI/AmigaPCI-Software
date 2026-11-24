/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * FatFs disk I/O layer: maps FatFs physical drives onto USB mass storage
 * devices. Physical drive N is USB MSC device N, LUN 0 (as numbered by
 * "usb msc").
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"
#include "rtc.h"
#include "usb_msc.h"
#include "fatfs/ff.h"
#include "fatfs/diskio.h"

/*
 * A drive is valid only while no mass storage device has been removed
 * since it was initialized: removing any device can renumber the rest.
 * FatFs re-mounts the volume (calling disk_initialize()) on its next
 * access once disk_status() reports STA_NOINIT.
 */
static uint8_t  drv_init[FF_VOLUMES];
static uint32_t drv_gen[FF_VOLUMES];

DSTATUS
disk_status(BYTE pdrv)
{
    if ((pdrv >= FF_VOLUMES) || !drv_init[pdrv] ||
        (drv_gen[pdrv] != usb_msc_generation()))
        return (STA_NOINIT);
    return (0);
}

DSTATUS
disk_initialize(BYTE pdrv)
{
    usb_msc_info_t info;

    if (pdrv >= FF_VOLUMES)
        return (STA_NOINIT | STA_NODISK);
    drv_init[pdrv] = 0;
    if (usb_msc_info(pdrv, 0, &info) != USB_MSC_OK)
        return (STA_NOINIT | STA_NODISK);
    if (!info.ready || (info.block_size < FF_MIN_SS) ||
        (info.block_size > FF_MAX_SS))
        return (STA_NOINIT);
    drv_gen[pdrv] = usb_msc_generation();
    drv_init[pdrv] = 1;
    return (0);
}

DRESULT
disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    int rc;

    if (disk_status(pdrv) & STA_NOINIT)
        return (RES_NOTRDY);
    if ((sector > 0xffffffff) || (sector + count - 1 > 0xffffffff))
        return (RES_PARERR);  // READ(10)/WRITE(10) block numbers are 32-bit
    rc = usb_msc_read(pdrv, 0, (uint32_t) sector, buff, count);
    if (rc == USB_MSC_OK)
        return (RES_OK);
    if ((rc == USB_MSC_NODEV) || (rc == USB_MSC_REMOVED))
        return (RES_NOTRDY);
    return (RES_ERROR);
}

#if FF_FS_READONLY == 0
DRESULT
disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    int rc;

    if (disk_status(pdrv) & STA_NOINIT)
        return (RES_NOTRDY);
    if ((sector > 0xffffffff) || (sector + count - 1 > 0xffffffff))
        return (RES_PARERR);  // READ(10)/WRITE(10) block numbers are 32-bit
    rc = usb_msc_write(pdrv, 0, (uint32_t) sector, buff, count);
    if (rc == USB_MSC_OK)
        return (RES_OK);
    if ((rc == USB_MSC_NODEV) || (rc == USB_MSC_REMOVED))
        return (RES_NOTRDY);
    return (RES_ERROR);
}
#endif

DRESULT
disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    usb_msc_info_t info;

    if (disk_status(pdrv) & STA_NOINIT)
        return (RES_NOTRDY);

    switch (cmd) {
        case CTRL_SYNC:
            /* Writes complete (CSW received) before disk_write returns */
            return (RES_OK);
        case GET_SECTOR_COUNT:
            if (usb_msc_info(pdrv, 0, &info) != USB_MSC_OK)
                return (RES_NOTRDY);
            *(LBA_t *) buff = info.blocks;
            return (RES_OK);
        case GET_SECTOR_SIZE:
            if (usb_msc_info(pdrv, 0, &info) != USB_MSC_OK)
                return (RES_NOTRDY);
            *(WORD *) buff = info.block_size;
            return (RES_OK);
        case GET_BLOCK_SIZE:
            *(DWORD *) buff = 1;    // Erase block size unknown
            return (RES_OK);
        default:
            return (RES_PARERR);
    }
}

#if FF_FS_NORTC == 0
/*
 * get_fattime() - current time for file timestamps
 *
 * bit 31:25 Year from 1980, 24:21 Month, 20:16 Day,
 *     15:11 Hour, 10:5 Minute, 4:0 Second / 2
 */
DWORD
get_fattime(void)
{
    uint year, mon, day, hour, min, sec, msec;

    rtc_get_time(&year, &mon, &day, &hour, &min, &sec, &msec);
    if (year < 1980)
        year = 1980;
    return (((DWORD) (year - 1980) << 25) | ((DWORD) mon << 21) |
            ((DWORD) day << 16) | ((DWORD) hour << 11) |
            ((DWORD) min << 5) | ((DWORD) sec >> 1));
}
#endif
