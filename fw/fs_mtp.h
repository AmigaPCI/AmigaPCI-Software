/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * Filesystem access to USB MTP device storage, with an interface which
 * follows FatFs (results are FRESULT, entries are FILINFO), so that the
 * BEC file service can serve MTP storage as additional volumes.
 *
 * Paths are UTF-8, relative to the storage root ("" is the root), with
 * '/' separators and no "." or ".." elements.
 */

#ifndef _FS_MTP_H
#define _FS_MTP_H

#include <stdint.h>
#include "fatfs/ff.h"

#define FS_MTP_VOLS     4       // Maximum MTP storage volumes

typedef struct mtpfile mtpfile_t;
typedef struct mtpdir  mtpdir_t;

/* Volume information */
typedef struct {
    uint32_t serial;            // Identifies the volume
    uint8_t  readonly;          // Storage or device can't be written
    char     name[36];          // Suggested volume name (UTF-8)
} fs_mtp_vol_t;

uint32_t fs_mtp_refresh(void);
uint     fs_mtp_volume(uint mvol, fs_mtp_vol_t *vol);

FRESULT  fs_mtp_stat(uint mvol, const char *path, FILINFO *fno);
FRESULT  fs_mtp_opendir(mtpdir_t **dp, uint mvol, const char *path);
FRESULT  fs_mtp_readdir(mtpdir_t *dp, FILINFO *fno);
void     fs_mtp_rewinddir(mtpdir_t *dp);
void     fs_mtp_closedir(mtpdir_t *dp);

FRESULT  fs_mtp_open(mtpfile_t **fp, uint mvol, const char *path, BYTE mode);
FRESULT  fs_mtp_read(mtpfile_t *fp, void *buf, UINT len, UINT *got);
FRESULT  fs_mtp_write(mtpfile_t *fp, const void *buf, UINT len, UINT *wrote);
FRESULT  fs_mtp_lseek(mtpfile_t *fp, FSIZE_t pos);
FSIZE_t  fs_mtp_tell(mtpfile_t *fp);
FSIZE_t  fs_mtp_size(mtpfile_t *fp);
FRESULT  fs_mtp_truncate(mtpfile_t *fp);
FRESULT  fs_mtp_sync(mtpfile_t *fp);
FRESULT  fs_mtp_close(mtpfile_t *fp);

FRESULT  fs_mtp_mkdir(uint mvol, const char *path);
FRESULT  fs_mtp_unlink(uint mvol, const char *path);
FRESULT  fs_mtp_rename(uint mvol, const char *opath, const char *npath);
FRESULT  fs_mtp_utime(uint mvol, const char *path, const FILINFO *fno);
FRESULT  fs_mtp_getfree(uint mvol, uint64_t *total, uint64_t *freeb);

extern uint8_t fs_mtp_debug;

#endif /* _FS_MTP_H */
