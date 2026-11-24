/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * Filesystem (FatFs) commands for USB mass storage.
 *
 * Volume N ("N:") is USB mass storage device N, as listed by "usb msc".
 * FAT12, FAT16, FAT32 and exFAT are supported.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "printf.h"
#include "uart.h"
#include "utils.h"
#include "cmdline.h"
#include "usb_msc.h"
#include "fs.h"
#include "fatfs/ff.h"

const char cmd_fs_help[] =
    "fs devs          - list USB mass storage devices\n"
    "fs info [<vol>]  - show filesystem size and free space\n"
    "fs ls [<path>]   - list directory\n"
    "fs read <dev[.lun]> <lba> [<count>] [q]\n"
    "                 - read and display raw blocks (q: only show rate)\n"
    "Volume <vol> is USB mass storage device <vol> from \"fs devs\". "
    "A path may\nstart with \"<vol>:\" to select it; the default is 0.";

static FATFS *fs_vol[FF_VOLUMES];

static const char * const fresult_str[] = {
    "OK",                               // FR_OK
    "disk I/O error",                   // FR_DISK_ERR
    "internal error",                   // FR_INT_ERR
    "device not ready",                 // FR_NOT_READY
    "file not found",                   // FR_NO_FILE
    "path not found",                   // FR_NO_PATH
    "invalid path name",                // FR_INVALID_NAME
    "access denied",                    // FR_DENIED
    "already exists",                   // FR_EXIST
    "invalid object",                   // FR_INVALID_OBJECT
    "write protected",                  // FR_WRITE_PROTECTED
    "invalid volume",                   // FR_INVALID_DRIVE
    "volume not mounted",               // FR_NOT_ENABLED
    "no FAT or exFAT filesystem found", // FR_NO_FILESYSTEM
    "mkfs aborted",                     // FR_MKFS_ABORTED
    "timeout",                          // FR_TIMEOUT
    "file locked",                      // FR_LOCKED
    "out of memory",                    // FR_NOT_ENOUGH_CORE
    "too many open files",              // FR_TOO_MANY_OPEN_FILES
    "invalid parameter",                // FR_INVALID_PARAMETER
};

static const char *
fs_strerror(FRESULT fr)
{
    if ((int) fr < ARRAY_SIZE(fresult_str))
        return (fresult_str[fr]);
    return ("unknown error");
}

/*
 * fs_register() - make sure a work area is registered for the volume.
 *                 FatFs mounts it on first access (and re-mounts it after
 *                 the device changes; see fs_diskio.c).
 */
static FRESULT
fs_register(uint vol)
{
    char drv[3] = { (char) ('0' + vol), ':', '\0' };
    FRESULT fr;

    if (vol >= FF_VOLUMES)
        return (FR_INVALID_DRIVE);
    if (fs_vol[vol] != NULL)
        return (FR_OK);
    fs_vol[vol] = malloc(sizeof (FATFS));
    if (fs_vol[vol] == NULL)
        return (FR_NOT_ENOUGH_CORE);
    fr = f_mount(fs_vol[vol], drv, 0);
    if (fr != FR_OK) {
        free(fs_vol[vol]);
        fs_vol[vol] = NULL;
    }
    return (fr);
}

/* Volume number from a path ("N:..."), or 0 if the path has none */
static int
fs_path_vol(const char *path)
{
    if ((path[0] >= '0') && (path[0] <= '9') && (path[1] == ':'))
        return (path[0] - '0');
    return (0);
}

/* Print a size given in KB; above 9000 KB it is shown in MB */
static void
print_kb(uint64_t kb)
{
    if (kb > 9000)
        printf("%llu MB", (unsigned long long) (kb >> 10));
    else
        printf("%llu KB", (unsigned long long) kb);
}

static const char *
fs_type_str(BYTE fs_type)
{
    switch (fs_type) {
        case FS_FAT12: return ("FAT12");
        case FS_FAT16: return ("FAT16");
        case FS_FAT32: return ("FAT32");
        case FS_EXFAT: return ("exFAT");
        default:       return ("Unknown");
    }
}

static rc_t
fs_info(uint vol)
{
    char     drv[3] = { (char) ('0' + vol), ':', '\0' };
    char     label[34];     // exFAT labels are up to 11 UTF-16 characters
    DWORD    serial = 0;
    DWORD    free_clust;
    FATFS   *fs;
    FRESULT  fr;
    uint64_t clust_bytes;
    usb_msc_info_t info;

    fr = fs_register(vol);
    if (fr == FR_OK)
        fr = f_getfree(drv, &free_clust, &fs);
    if (fr != FR_OK) {
        printf("Volume %u: %s\n", vol, fs_strerror(fr));
        return (RC_FAILURE);
    }
    if (f_getlabel(drv, label, &serial) != FR_OK)
        label[0] = '\0';

    clust_bytes = (uint64_t) fs->csize * FF_MAX_SS;

    printf("Volume %u:", vol);
    if (usb_msc_info(vol, 0, &info) == USB_MSC_OK)
        printf(" USB%s %s %s", info.path, info.vendor, info.product);
    printf("\n");
    printf("  Filesystem   %s\n", fs_type_str(fs->fs_type));
    printf("  Label        %s\n", (label[0] != '\0') ? label : "(none)");
    printf("  Serial       %04lX-%04lX\n", serial >> 16, serial & 0xffff);
    printf("  Cluster size %lu bytes\n", (uint32_t) clust_bytes);
    if (usb_msc_info(vol, 0, &info) == USB_MSC_OK) {
        printf("  Media size   ");
        print_kb((uint64_t) info.blocks * info.block_size / 1024);
        printf("\n");
    }
    printf("  Volume size  ");
    print_kb((uint64_t) (fs->n_fatent - 2) * clust_bytes / 1024);
    printf("\n  Free space   ");
    print_kb((uint64_t) free_clust * clust_bytes / 1024);
    printf("\n");
    return (RC_SUCCESS);
}

static rc_t
fs_ls(const char *path)
{
    DIR      dir;
    FILINFO  fno;
    FRESULT  fr;
    uint     files = 0;
    uint     dirs = 0;
    uint64_t bytes = 0;
    int      vol = fs_path_vol(path);

    fr = fs_register(vol);
    if (fr == FR_OK)
        fr = f_opendir(&dir, path);
    if (fr != FR_OK) {
        printf("%s: %s\n", path, fs_strerror(fr));
        return (RC_FAILURE);
    }
    for (;;) {
        fr = f_readdir(&dir, &fno);
        if ((fr != FR_OK) || (fno.fname[0] == '\0'))
            break;
        printf("%04u-%02u-%02u %02u:%02u  ",
               ((fno.fdate >> 9) & 0x7f) + 1980, (fno.fdate >> 5) & 0xf,
               fno.fdate & 0x1f, (fno.ftime >> 11) & 0x1f,
               (fno.ftime >> 5) & 0x3f);
        printf("%c%c%c%c ",
               (fno.fattrib & AM_RDO) ? 'r' : '-',
               (fno.fattrib & AM_HID) ? 'h' : '-',
               (fno.fattrib & AM_SYS) ? 's' : '-',
               (fno.fattrib & AM_ARC) ? 'a' : '-');
        if (fno.fattrib & AM_DIR) {
            printf("%12s  %s/\n", "<DIR>", fno.fname);
            dirs++;
        } else {
            printf("%12llu  %s\n", (unsigned long long) fno.fsize, fno.fname);
            files++;
            bytes += fno.fsize;
        }
        if (input_break_pending()) {
            (void) f_closedir(&dir);
            return (RC_USR_ABORT);
        }
    }
    (void) f_closedir(&dir);
    if (fr != FR_OK) {
        printf("%s: %s\n", path, fs_strerror(fr));
        return (RC_FAILURE);
    }
    printf("%u file%s, %u director%s, ", files, (files == 1) ? "" : "s",
           dirs, (dirs == 1) ? "y" : "ies");
    print_kb((bytes + 1023) / 1024);
    printf("\n");
    return (RC_SUCCESS);
}

rc_t
cmd_fs(int argc, char * const *argv)
{
    if (argc < 2)
        return (RC_USER_HELP);

    if ((strncmp(argv[1], "devs", 3) == 0) ||
        (strncmp(argv[1], "devices", 3) == 0)) {
        usb_msc_list();
        return (RC_SUCCESS);
    }
    if (strcmp(argv[1], "read") == 0)
        return (usb_msc_read_cmd(argc - 1, argv + 1));
    if (strncmp(argv[1], "info", 1) == 0) {
        int vol = 0;
        if (argc > 2) {
            if ((scan_int(argv[2], &vol) != RC_SUCCESS) ||
                (vol < 0) || (vol >= FF_VOLUMES)) {
                printf("Invalid volume %s\n", argv[2]);
                return (RC_BAD_PARAM);
            }
        }
        return (fs_info(vol));
    }
    if ((strcmp(argv[1], "ls") == 0) || (strcmp(argv[1], "dir") == 0)) {
        rc_t rc = RC_SUCCESS;
        int  arg;
        if (argc < 3)
            return (fs_ls("0:/"));
        for (arg = 2; arg < argc; arg++) {
            if (argc > 3)
                printf("%s:\n", argv[arg]);
            rc = fs_ls(argv[arg]);
            if (rc == RC_USR_ABORT)
                break;
        }
        return (rc);
    }
    printf("Unknown argument %s\n", argv[1]);
    return (RC_USER_HELP);
}
