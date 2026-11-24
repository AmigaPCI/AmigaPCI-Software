/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * BEC file service for the Amiga (BEC_CMD_F* messages).
 *
 * This is the BEC equivalent of the KickSmash hostsmash file service
 * (hostsmash_fs.c), serving files from USB mass storage through FatFs
 * instead of from a host filesystem. The protocol is described in
 * bec_cmd.h. Differences from hostsmash, due to FAT / exFAT:
 *   - There are no links, owners, device nodes or access / change times.
 *     FSETOWN is accepted and ignored. Access and change times read as
 *     the modify time; setting the access time is ignored.
 *   - Amiga permissions map to the FAT read-only attribute (write and
 *     delete protection) and archive attribute (inverted); the rest read
 *     as allowed.
 *   - Names are converted between Amiga Latin-1 and FatFs UTF-8.
 *
 * Each USB mass storage device is one volume (FatFs volume N is "fs devs"
 * device N). Volumes are identified by their FAT serial number, so that a
 * handle never refers to a different device after USB devices come and
 * go. Removing any USB storage device re-mounts the others, which makes
 * their open files and directories stale.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "printf.h"
#include "bec_cmd.h"
#include "crc32.h"
#include "rtc.h"
#include "timer.h"
#include "utils.h"
#include "usb_msc.h"
#include "fs.h"
#include "fs_server.h"
#include "fatfs/ff.h"

#define SWAP16(x)   __builtin_bswap16(x)
#define SWAP32(x)   __builtin_bswap32(x)

/* AmigaOS FileInfoBlock protection bits */
#define FIBF_ARCHIVE    0x00000010  // File has been archived
#define FIBF_READ       0x00000008  // Owner: file is not readable
#define FIBF_WRITE      0x00000004  // Owner: file is not writable
#define FIBF_EXECUTE    0x00000002  // Owner: file is not executable
#define FIBF_DELETE     0x00000001  // Owner: file may not be deleted

/* UNIX mode bits reported in bfd_mode */
#define UMODE_IFDIR     0040000
#define UMODE_IFREG     0100000

#define SEEK_OFFSET_BEGINNING  (-1)
#define SEEK_OFFSET_CURRENT      (0)
#define SEEK_OFFSET_END          (1)

#define HANDLE_VOLDIR   0x00000000  // Volume Directory
#define HANDLE_DEFAULT  0xffffffff  // Root of first volume

#define FS_NAME_MAX     256         // Maximum Latin-1 path element + NIL
#define DIRENT_MAX      (sizeof (bf_fdirent_t) + FS_NAME_MAX + 2)

uint8_t fsrv_debug;

#define fsprintf(...) do { if (fsrv_debug) printf(__VA_ARGS__); } while (0)

typedef struct fhandle fhandle_t;
struct fhandle {
    fhandle_t *next;
    uint32_t   handle;      // Handle number given to the Amiga
    int8_t     vol;         // FatFs volume, or -1 for Volume Directory
    uint8_t    type;        // BF_TYPE_*
    uint16_t   mode;        // BF_MODE_* (without lock bits)
    uint16_t   lock;        // 0, BF_MODE_LOCK, or BF_MODE_LOCK_EX
    uint16_t   entnum;      // Directory entry position (Volume Directory
                            // and stat handles)
    uint32_t   serial;      // Volume serial number (identifies the volume)
    char      *name;        // Latin-1 path from volume root ("" = root)
    FIL       *fil;         // Open file
    DIR       *dir;         // Open directory
};

typedef struct {
    uint8_t  mounted;
    uint32_t serial;
    char     name[36];      // Latin-1 volume name (no colon)
} fsvol_t;

static fhandle_t *fh_head;
static uint32_t   fh_next;
static uint32_t   fh_epoch;

static fsvol_t    fsvol[FF_VOLUMES];
static uint32_t   fsvol_gen;      // usb_msc_generation() at last refresh
static uint       fsvol_count;    // usb_msc_count() at last refresh
static uint8_t    fsvol_valid;

/* Requests are copied here: aligned, and NIL-padded after the payload */
static uint32_t   fsrv_reqbuf[(BEC_MSG_MAX_PAYLOAD + 8) / 4];
static uint32_t   fsrv_replybuf[(BEC_MSG_MAX_PAYLOAD + 3) / 4];

/* ------------------------------------------------------------------ */
/* Name conversion                                                     */

/* Latin-1 to UTF-8; returns 0 on success, -1 if dst is too small */
static int
latin1_to_utf8(char *dst, size_t dstlen, const char *src)
{
    const uint8_t *s = (const uint8_t *) src;
    size_t pos = 0;

    for (; *s != '\0'; s++) {
        if (*s < 0x80) {
            if (pos + 1 >= dstlen)
                return (-1);
            dst[pos++] = *s;
        } else {
            if (pos + 2 >= dstlen)
                return (-1);
            dst[pos++] = 0xc0 | (*s >> 6);
            dst[pos++] = 0x80 | (*s & 0x3f);
        }
    }
    if (pos >= dstlen)
        return (-1);
    dst[pos] = '\0';
    return (0);
}

/* UTF-8 to Latin-1 in place; characters beyond Latin-1 become '?' */
static void
utf8_to_latin1(char *str)
{
    const uint8_t *s = (const uint8_t *) str;
    uint8_t *d = (uint8_t *) str;

    while (*s != '\0') {
        uint32_t ch = *s++;
        uint     extra = 0;

        if (ch >= 0xf0) {
            ch &= 0x07;
            extra = 3;
        } else if (ch >= 0xe0) {
            ch &= 0x0f;
            extra = 2;
        } else if (ch >= 0xc0) {
            ch &= 0x1f;
            extra = 1;
        } else if (ch >= 0x80) {
            ch = '?';       // Stray continuation byte
        }
        while ((extra > 0) && ((*s & 0xc0) == 0x80)) {
            ch = (ch << 6) | (*s++ & 0x3f);
            extra--;
        }
        if ((extra != 0) || (ch > 0xff))
            ch = '?';
        *d++ = (uint8_t) ch;
    }
    *d = '\0';
}

/* ASCII case-insensitive compare of len bytes (FAT is case-insensitive) */
static int
name_ncaseeq(const char *a, const char *b, size_t len)
{
    for (; len > 0; len--, a++, b++) {
        char ca = *a;
        char cb = *b;
        if ((ca >= 'a') && (ca <= 'z'))
            ca -= 'a' - 'A';
        if ((cb >= 'a') && (cb <= 'z'))
            cb -= 'a' - 'A';
        if (ca != cb)
            return (0);
        if (ca == '\0')
            break;
    }
    return (1);
}

static int
name_caseeq(const char *a, const char *b)
{
    size_t len = strlen(a);
    return ((strlen(b) == len) && name_ncaseeq(a, b, len));
}

/* ------------------------------------------------------------------ */
/* Time conversion (FAT and Amiga times are both local time)          */

static uint32_t
fat_to_secs(WORD fdate, WORD ftime)
{
    uint mon = (fdate >> 5) & 0xf;
    uint day = fdate & 0x1f;

    if ((mon < 1) || (mon > 12) || (day < 1))
        return (rtc_to_utc(1980, 1, 1, 0, 0, 0));
    return (rtc_to_utc((fdate >> 9) + 1980, mon, day, (ftime >> 11) & 0x1f,
                       (ftime >> 5) & 0x3f, (ftime & 0x1f) * 2));
}

static void
secs_to_fat(uint32_t secs, WORD *fdate, WORD *ftime)
{
    uint year, mon, day, hour, min, sec;

    utc_to_rtc(secs, &year, &mon, &day, &hour, &min, &sec);
    if (year < 1980) {
        year = 1980;
        mon  = 1;
        day  = 1;
        hour = 0;
        min  = 0;
        sec  = 0;
    } else if (year > 2107) {
        year = 2107;
    }
    *fdate = ((year - 1980) << 9) | (mon << 5) | day;
    *ftime = (hour << 11) | (min << 5) | (sec / 2);
}

static uint32_t
now_secs(void)
{
    uint year, mon, day, hour, min, sec, msec;
    rtc_get_time(&year, &mon, &day, &hour, &min, &sec, &msec);
    return (rtc_to_utc(year, mon, day, hour, min, sec));
}

/* ------------------------------------------------------------------ */
/* Status conversion                                                   */

static uint
fr_to_status(FRESULT fr)
{
    switch (fr) {
        case FR_OK:
            return (BEC_STATUS_OK);
        case FR_NO_FILE:
        case FR_NO_PATH:
            return (BEC_STATUS_NOEXIST);
        case FR_INVALID_NAME:
        case FR_INVALID_PARAMETER:
            return (BEC_STATUS_INVALID);
        case FR_DENIED:
        case FR_WRITE_PROTECTED:
            return (BEC_STATUS_PERM);
        case FR_EXIST:
            return (BEC_STATUS_EXIST);
        case FR_INVALID_OBJECT:
            return (BEC_STATUS_STALE);
        case FR_LOCKED:
            return (BEC_STATUS_INUSE);
        case FR_NOT_READY:
        case FR_INVALID_DRIVE:
        case FR_NOT_ENABLED:
        case FR_NO_FILESYSTEM:
            return (BEC_STATUS_UNAVAIL);
        default:
            return (BEC_STATUS_FAIL);
    }
}

/* ------------------------------------------------------------------ */
/* Volumes                                                             */

/*
 * fsvol_refresh() - mount new volumes and forget removed ones. Volumes
 *                   are only re-examined when the set of USB storage
 *                   devices changes.
 */
static void
fsvol_refresh(void)
{
    uint32_t gen   = usb_msc_generation();
    uint     count = usb_msc_count();
    uint     vol;

    if (fsvol_valid && (gen == fsvol_gen) && (count == fsvol_count))
        return;

    for (vol = 0; vol < FF_VOLUMES; vol++) {
        char  drv[3] = { (char) ('0' + vol), ':', '\0' };
        char  label[36];
        DWORD serial = 0;
        uint  other;

        fsvol[vol].mounted = 0;
        if ((vol >= count) || (fs_volume_register(vol) != FR_OK) ||
            (f_getlabel(drv, label, &serial) != FR_OK))
            continue;
        utf8_to_latin1(label);
        /* Use the label, unless empty, unusable, or already in use */
        for (other = 0; other < vol; other++)
            if (fsvol[other].mounted && name_caseeq(fsvol[other].name, label))
                break;
        if ((label[0] == '\0') || (other < vol) ||
            (strpbrk(label, ":/") != NULL))
            sprintf(label, "USB%u", vol);
        strcpy(fsvol[vol].name, label);
        fsvol[vol].serial  = serial;
        fsvol[vol].mounted = 1;
        fsprintf("FS vol %u: %s serial %08lx\n", vol, label, serial);
    }
    fsvol_gen   = gen;
    fsvol_count = count;
    fsvol_valid = 1;
}

/* Find a mounted volume by name (with no colon); returns -1 if none */
static int
fsvol_lookup(const char *name, size_t len)
{
    uint vol;

    for (vol = 0; vol < FF_VOLUMES; vol++) {
        char usbname[8];
        if (!fsvol[vol].mounted)
            continue;
        sprintf(usbname, "USB%u", vol);
        if (((strlen(fsvol[vol].name) == len) &&
             name_ncaseeq(fsvol[vol].name, name, len)) ||
            ((strlen(usbname) == len) && name_ncaseeq(usbname, name, len)))
            return (vol);
    }
    return (-1);
}

/* Is the volume a handle refers to still the same, mounted volume? */
static int
fsvol_same(int vol, uint32_t serial)
{
    return ((vol < 0) ||
            (fsvol[vol].mounted && (fsvol[vol].serial == serial)));
}

/* ------------------------------------------------------------------ */
/* Handles                                                             */

static fhandle_t *
fh_get(uint32_t handle)
{
    fhandle_t *fh;
    for (fh = fh_head; fh != NULL; fh = fh->next)
        if (fh->handle == handle)
            return (fh);
    return (NULL);
}

static fhandle_t *
fh_new(int vol, const char *name, uint type, uint mode)
{
    fhandle_t *fh = malloc(sizeof (*fh));
    uint32_t   handle;

    if (fh == NULL)
        return (NULL);
    memset(fh, 0, sizeof (*fh));
    fh->name = malloc(strlen(name) + 1);
    if (fh->name == NULL) {
        free(fh);
        return (NULL);
    }
    strcpy(fh->name, name);

    if (fh_epoch == 0) {
        /*
         * Handle numbers start from a value which differs for every BEC
         * run, so that handles from a previous run are (with near
         * certainty) reported as stale rather than finding another file.
         */
        uint32_t seed = crc32(0, cpu_serial_str, strlen(cpu_serial_str));
        seed ^= (uint32_t) timer_tick_get();
        seed = crc32(seed, &seed, sizeof (seed));
        fh_epoch = (seed & 0x7fffffff) | 0x00010000;
        fh_next  = fh_epoch;
    }
    do {
        handle = ++fh_next;
        if ((handle == HANDLE_VOLDIR) || (handle == HANDLE_DEFAULT))
            handle = fh_next = fh_epoch;
    } while (fh_get(handle) != NULL);

    fh->handle = handle;
    fh->vol    = vol;
    fh->type   = type;
    fh->mode   = mode;
    fh->serial = (vol >= 0) ? fsvol[vol].serial : 0;
    fh->next   = fh_head;
    fh_head    = fh;
    return (fh);
}

static void
fh_free(fhandle_t *fh)
{
    fhandle_t **prev;

    for (prev = &fh_head; *prev != NULL; prev = &(*prev)->next) {
        if (*prev == fh) {
            *prev = fh->next;
            break;
        }
    }
    free(fh->fil);
    free(fh->dir);
    free(fh->name);
    free(fh);
}

/*
 * fh_find_open() - find a handle on the specified object
 *
 * what: 0 = any handle, 1 = a lock (exclusive_only selects exclusive),
 *       2 = a file open for write
 */
static fhandle_t *
fh_find_open(int vol, uint32_t serial, const char *name, uint what,
             uint exclusive_only)
{
    fhandle_t *fh;

    for (fh = fh_head; fh != NULL; fh = fh->next) {
        if ((fh->vol != vol) || (fh->serial != serial) ||
            !name_caseeq(fh->name, name))
            continue;
        if ((what == 1) &&
            ((fh->lock == 0) ||
             (exclusive_only && (fh->lock != BF_MODE_LOCK_EX))))
            continue;
        if ((what == 2) &&
            ((fh->fil == NULL) || !(fh->mode & BF_MODE_WRITE)))
            continue;
        return (fh);
    }
    return (NULL);
}

/* ------------------------------------------------------------------ */
/* Paths                                                               */

/* Remove the last element of a path (stays at the root) */
static void
path_up(char *path)
{
    char *slash = strrchr(path, '/');
    if (slash != NULL)
        *slash = '\0';
    else
        *path = '\0';
}

/*
 * fs_resolve() - convert an Amiga path relative to a handle to a
 *                volume and a Latin-1 path relative to that volume's root
 *
 * Handles "::" (Volume Directory), ":" (root of the current volume),
 * "VOLUME:" prefixes, "/" (parent, AmigaDOS style), "." and "..". At the
 * Volume Directory, the first path element names a volume.
 *
 * On success, *pathp is a malloc()ed path, and *volp is the volume or -1
 * for the Volume Directory. Returns a BEC_STATUS_* value.
 */
static uint
fs_resolve(uint32_t phandle, const char *name, int *volp, char **pathp)
{
    const char *base = "";
    const char *p = name;
    const char *colon;
    const char *slash;
    fhandle_t  *parent;
    int         vol;
    char       *buf;
    size_t      len;

    fsvol_refresh();
    *pathp = NULL;
    if (phandle == HANDLE_VOLDIR) {
        vol = -1;
    } else if (phandle == HANDLE_DEFAULT) {
        vol = 0;
        if (!fsvol[0].mounted)
            return (BEC_STATUS_UNAVAIL);
    } else {
        parent = fh_get(phandle);
        if ((parent == NULL) || !fsvol_same(parent->vol, parent->serial))
            return (BEC_STATUS_STALE);
        vol = parent->vol;
        base = parent->name;
    }

    if ((p[0] == ':') && (p[1] == ':')) {
        vol = -1;
        base = "";
        p += 2;
    } else if (p[0] == ':') {
        base = "";
        p++;
    } else {
        colon = strchr(p, ':');
        slash = strchr(p, '/');
        if ((colon != NULL) && ((slash == NULL) || (colon < slash))) {
            vol = fsvol_lookup(p, colon - p);
            if (vol < 0)
                return (BEC_STATUS_NOEXIST);
            base = "";
            p = colon + 1;
        }
    }

    buf = malloc(strlen(base) + strlen(p) + 2);
    if (buf == NULL)
        return (BEC_STATUS_FAIL);
    strcpy(buf, (vol < 0) ? "" : base);

    /* Each leading slash is the parent directory */
    for (; *p == '/'; p++)
        path_up(buf);

    while (*p != '\0') {
        const char *elem = p;
        size_t      elen;

        while ((*p != '\0') && (*p != '/'))
            p++;
        elen = p - elem;
        if ((elen == 1) && (elem[0] == '.')) {
            /* Same directory */
        } else if ((elen == 2) && (elem[0] == '.') && (elem[1] == '.')) {
            path_up(buf);
        } else if (vol < 0) {
            /* Element at the Volume Directory names a volume */
            if ((elen > 0) && (elem[elen - 1] == ':'))
                elen--;
            vol = fsvol_lookup(elem, elen);
            if (vol < 0) {
                free(buf);
                return (BEC_STATUS_NOEXIST);
            }
            buf[0] = '\0';
        } else {
            len = strlen(buf);
            if (len > 0)
                buf[len++] = '/';
            memcpy(buf + len, elem, elen);
            buf[len + elen] = '\0';
        }
        if (*p == '/') {
            /* Separator; each additional slash is the parent directory */
            for (p++; *p == '/'; p++)
                path_up(buf);
        }
    }
    if (vol < 0)
        buf[0] = '\0';
    *volp = vol;
    *pathp = buf;
    return (BEC_STATUS_OK);
}

/* Make a FatFs path ("N:/path" in UTF-8); returns NULL if too long */
static char *
fs_fatpath(int vol, const char *path)
{
    size_t len = strlen(path) * 2 + 5;
    char  *fpath = malloc(len);

    if (fpath == NULL)
        return (NULL);
    fpath[0] = '0' + vol;
    fpath[1] = ':';
    fpath[2] = '/';
    if (latin1_to_utf8(fpath + 3, len - 3, path) != 0) {
        free(fpath);
        return (NULL);
    }
    return (fpath);
}

/* Stat an object; the volume root is reported as a directory */
static FRESULT
fs_stat(int vol, const char *path, FILINFO *fno)
{
    FRESULT fr;
    char   *fpath;

    if (path[0] == '\0') {
        memset(fno, 0, sizeof (*fno));
        fno->fattrib = AM_DIR;
        return (FR_OK);
    }
    fpath = fs_fatpath(vol, path);
    if (fpath == NULL)
        return (FR_INVALID_NAME);
    fr = f_stat(fpath, fno);
    free(fpath);
    return (fr);
}

/* ------------------------------------------------------------------ */
/* Permissions                                                         */

static uint32_t
aperms_from_fat(BYTE fattrib)
{
    uint32_t perms = 0;
    if (fattrib & AM_RDO)
        perms |= FIBF_WRITE | FIBF_DELETE;
    if ((fattrib & AM_ARC) == 0)
        perms |= FIBF_ARCHIVE;  // DOS archive flag means "not archived"
    return (perms);
}

static BYTE
fat_from_aperms(uint32_t aperms)
{
    BYTE attr = 0;
    if (aperms & (FIBF_WRITE | FIBF_DELETE))
        attr |= AM_RDO;
    if ((aperms & FIBF_ARCHIVE) == 0)
        attr |= AM_ARC;
    return (attr);
}

/* ------------------------------------------------------------------ */
/* Directory entries                                                   */

/*
 * dirent_put() - format a directory entry at buf, followed by its name
 *                and an empty comment. Returns the entry length.
 */
static uint
dirent_put(uint8_t *buf, const char *name, uint type, uint64_t size,
           uint32_t blksize, uint32_t blks, uint32_t mtime, uint32_t aperms,
           uint32_t umode, uint32_t ino, uint32_t nlink, uint32_t rdev)
{
    bf_fdirent_t de;
    uint nlen = strlen(name) + 1;
    char *nptr = (char *) buf + sizeof (de);

    if (nlen > FS_NAME_MAX)
        nlen = FS_NAME_MAX;
    memcpy(nptr, name, nlen - 1);
    nptr[nlen - 1] = '\0';
    nptr[nlen++] = '\0';            // Empty comment
    if (nlen & 1)
        nptr[nlen++] = '\0';        // Pad to even length

    memset(&de, 0, sizeof (de));
    de.bfd_type    = SWAP16(type);
    de.bfd_elen    = SWAP16(nlen);
    de.bfd_size_hi = SWAP32((uint32_t) (size >> 32));
    de.bfd_size_lo = SWAP32((uint32_t) size);
    de.bfd_blksize = SWAP32(blksize);
    de.bfd_blks    = SWAP32(blks);
    de.bfd_atime   = SWAP32(mtime);
    de.bfd_ctime   = SWAP32(mtime);
    de.bfd_mtime   = SWAP32(mtime);
    de.bfd_aperms  = SWAP32(aperms);
    de.bfd_ino     = SWAP32(ino);
    de.bfd_mode    = SWAP32(umode);
    de.bfd_nlink   = SWAP32(nlink);
    de.bfd_rdev    = SWAP32(rdev);
    memcpy(buf, &de, sizeof (de));
    return (sizeof (de) + nlen);
}

/* Volume entry: size is in blocks of blksize (the cluster size) */
static uint
dirent_put_volume(uint8_t *buf, int vol, const char *name)
{
    char     drv[3] = { (char) ('0' + vol), ':', '\0' };
    DWORD    free_clust = 0;
    FATFS   *fs = NULL;
    uint32_t total = 0;
    uint32_t used = 0;
    uint32_t blksize = 0;

    if ((vol >= 0) && (f_getfree(drv, &free_clust, &fs) == FR_OK)) {
        total   = fs->n_fatent - 2;
        used    = total - free_clust;
        blksize = (uint32_t) fs->csize * FF_MAX_SS;
    }
    return (dirent_put(buf, name, BF_TYPE_VOLUME, total, blksize, used,
                       now_secs(), FIBF_WRITE | FIBF_DELETE,
                       UMODE_IFDIR | 0700, 0, 0, fh_epoch));
}

/* File or directory entry from FatFs FILINFO (fname is UTF-8) */
static uint
dirent_put_file(uint8_t *buf, fhandle_t *fh, const char *dirpath,
                FILINFO *fno)
{
    FATFS   *fs = NULL;
    DWORD    free_clust;
    char     drv[3] = { (char) ('0' + fh->vol), ':', '\0' };
    uint32_t blksize = 512;
    uint32_t ino;
    uint     isdir = (fno->fattrib & AM_DIR) != 0;
    uint64_t size = isdir ? 0 : fno->fsize;
    uint32_t umode;

    utf8_to_latin1(fno->fname);

    /* FAT has no inode numbers: use a hash of the path */
    ino = crc32(fh->serial, dirpath, strlen(dirpath));
    ino = crc32(ino, fno->fname, strlen(fno->fname));

    /* Cluster size; f_getfree() is cheap once the free count is known */
    if (f_getfree(drv, &free_clust, &fs) == FR_OK)
        blksize = (uint32_t) fs->csize * FF_MAX_SS;

    umode = isdir ? (UMODE_IFDIR | 0755) : (UMODE_IFREG | 0644);
    if (fno->fattrib & AM_RDO)
        umode &= ~0222;
    return (dirent_put(buf, fno->fname, isdir ? BF_TYPE_DIR : BF_TYPE_FILE,
                       size, blksize, (uint32_t) ((size + 511) / 512),
                       fat_to_secs(fno->fdate, fno->ftime),
                       aperms_from_fat(fno->fattrib), umode, ino, 1, 0));
}

/* ------------------------------------------------------------------ */
/* Request handlers                                                    */

/* Common reply for requests which return their request structure */
#define REPLY_REQ(req, size) \
    do { memcpy(reply, (req), (size)); *replylen = (size); } while (0)

static uint
fsrv_fopen(bf_fopenhandle_t *req, uint8_t *reply, uint *replylen)
{
    const char *aname   = (const char *) (req + 1);
    uint        mode    = SWAP16(req->bf_mode);
    uint        lockreq = mode & BF_MODE_LOCK_EX;
    uint        type;
    uint        status;
    int         vol;
    char       *path    = NULL;
    char       *fpath   = NULL;
    fhandle_t  *fh      = NULL;
    FILINFO     fno;
    FRESULT     fr;
    int         exists;

    fsprintf("fopen(%s %x) in %lx\n", aname, mode, req->bf_handle);
    mode &= ~BF_MODE_LOCK_EX;
    if (lockreq != 0)
        lockreq = (lockreq & ~BF_MODE_LOCK) ? BF_MODE_LOCK_EX : BF_MODE_LOCK;
    if ((aname[0] == '\0') && (mode == 0))
        mode = BF_MODE_READ;  // Re-open of the handle's own object

    status = fs_resolve(req->bf_handle, aname, &vol, &path);
    if (status != BEC_STATUS_OK)
        goto fail;

    if (vol < 0) {
        /* Volume Directory */
        if ((mode & BF_MODE_READ) == 0) {
            status = BEC_STATUS_INVALID;
            goto fail;
        }
        fh = fh_new(-1, "", BF_TYPE_VOLDIR, mode);
        type = BF_TYPE_DIR;
        goto success;
    }
    if (mode & BF_MODE_LINK) {
        /* FAT has no links */
        status = BEC_STATUS_INVALID;
        goto fail;
    }

    fr = fs_stat(vol, path, &fno);
    exists = (fr == FR_OK);
    if (!exists &&
        (((fr != FR_NO_FILE) || !(mode & BF_MODE_CREATE)) ||
         (mode & BF_MODE_DIR))) {
        status = fr_to_status(fr);
        goto fail;
    }
    type = (exists && (fno.fattrib & AM_DIR)) ? BF_TYPE_DIR : BF_TYPE_FILE;

    if ((lockreq != 0) && exists &&
        (fh_find_open(vol, fsvol[vol].serial, path, 1,
                      lockreq == BF_MODE_LOCK) != NULL)) {
        fsprintf("fopen(%s) lock refused: object in use\n", path);
        status = BEC_STATUS_INUSE;
        goto fail;
    }

    if (mode & BF_MODE_DIR) {
        /* Stat handle: reads the object's own directory entry */
        if ((mode & BF_MODE_RDWR) != BF_MODE_READ) {
            status = BEC_STATUS_INVALID;
            goto fail;
        }
        fh = fh_new(vol, path, type, mode);
        goto success;
    }

    fpath = fs_fatpath(vol, path);
    if (fpath == NULL) {
        status = BEC_STATUS_INVALID;
        goto fail;
    }

    if (type == BF_TYPE_DIR) {
        DIR *dir;
        if ((mode & ~BF_MODE_DIR) != BF_MODE_READ) {
            status = BEC_STATUS_INVALID;
            goto fail;
        }
        dir = malloc(sizeof (*dir));
        if (dir == NULL) {
            status = BEC_STATUS_FAIL;
            goto fail;
        }
        fr = f_opendir(dir, fpath);
        if (fr != FR_OK) {
            free(dir);
            status = fr_to_status(fr);
            goto fail;
        }
        fh = fh_new(vol, path, type, mode);
        if (fh == NULL) {
            f_closedir(dir);
            free(dir);
        } else {
            fh->dir = dir;
        }
    } else {
        FIL  *fil;
        BYTE  fmode = 0;

        if (mode & BF_MODE_READ)
            fmode |= FA_READ;
        if (mode & BF_MODE_WRITE)
            fmode |= FA_WRITE;
        if ((fmode == 0) ||
            (((mode & (BF_MODE_CREATE | BF_MODE_TRUNC)) != 0) &&
             ((mode & BF_MODE_WRITE) == 0))) {
            status = BEC_STATUS_INVALID;
            goto fail;
        }
        if ((mode & BF_MODE_WRITE) &&
            (fh_find_open(vol, fsvol[vol].serial, path, 2, 0) != NULL)) {
            /* FatFs can't share a file between two writers */
            fsprintf("fopen(%s) already open for write\n", path);
            status = BEC_STATUS_INUSE;
            goto fail;
        }
        if (mode & BF_MODE_CREATE)
            fmode |= (mode & BF_MODE_TRUNC) ? FA_CREATE_ALWAYS : FA_OPEN_ALWAYS;
        else
            fmode |= FA_OPEN_EXISTING;

        fil = malloc(sizeof (*fil));
        if (fil == NULL) {
            status = BEC_STATUS_FAIL;
            goto fail;
        }
        fr = f_open(fil, fpath, fmode);
        if ((fr == FR_OK) && (mode & BF_MODE_TRUNC) &&
            !(mode & BF_MODE_CREATE)) {
            fr = f_truncate(fil);  // At position 0 after open
            if (fr != FR_OK)
                f_close(fil);
        }
        if (fr != FR_OK) {
            free(fil);
            status = fr_to_status(fr);
            goto fail;
        }
        fh = fh_new(vol, path, BF_TYPE_FILE, mode);
        if (fh == NULL) {
            f_close(fil);
            free(fil);
        } else {
            fh->fil = fil;
        }
    }

success:
    if (fh == NULL) {
        status = BEC_STATUS_FAIL;
        goto fail;
    }
    fh->lock = lockreq;
    free(path);
    free(fpath);
    req->bf_handle = fh->handle;
    req->bf_type   = SWAP16(type);
    req->bf_mode   = SWAP16(BF_MODE_LOCK);  // Advertise lock support
    fsprintf("  handle=%lx\n", fh->handle);
    REPLY_REQ(req, sizeof (*req));
    return (BEC_STATUS_OK);

fail:
    fsprintf("fopen(%s) fail %x\n", aname, status);
    free(path);
    free(fpath);
    req->bf_handle = 0;
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

static uint
fsrv_fclose(bf_fopenhandle_t *req, uint8_t *reply, uint *replylen)
{
    fhandle_t *fh = fh_get(req->bf_handle);
    uint       status = BEC_STATUS_OK;

    fsprintf("fclose(%lx)\n", req->bf_handle);
    REPLY_REQ(req, sizeof (*req));
    if (fh == NULL)
        return (BEC_STATUS_STALE);
    if (fh->fil != NULL) {
        FRESULT fr = f_close(fh->fil);
        if ((fr != FR_OK) && (fr != FR_INVALID_OBJECT))
            status = fr_to_status(fr);  // Data may not have been written
    } else if (fh->dir != NULL) {
        (void) f_closedir(fh->dir);
    }
    fh_free(fh);
    return (status);
}

static uint
fsrv_fread(bf_freadwrite_t *req, uint8_t *reply, uint *replylen)
{
    fhandle_t       *fh     = fh_get(req->bf_handle);
    bf_freadwrite_t *rep    = (bf_freadwrite_t *) reply;
    uint8_t         *data   = (uint8_t *) (rep + 1);
    uint             length = SWAP32(req->bf_length);
    uint             flag   = SWAP16(req->bf_flag);
    uint             status = BEC_STATUS_OK;
    uint             pos    = 0;
    FRESULT          fr;

    memcpy(rep, req, sizeof (*rep));
    rep->bf_flag = 0;
    rep->bf_unused = 0;
    *replylen = sizeof (*rep);

    if (length > BF_RW_MAX)
        length = BF_RW_MAX;
    if (fh == NULL) {
        status = BEC_STATUS_STALE;
        goto done;
    }
    if ((fh->mode & BF_MODE_READ) == 0) {
        status = BEC_STATUS_INVALID;
        goto done;
    }
    fsvol_refresh();
    if (!fsvol_same(fh->vol, fh->serial)) {
        status = BEC_STATUS_STALE;
        goto done;
    }

    if (fh->fil != NULL) {
        UINT got = 0;
        if (flag & BF_FLAG_SEEK0) {
            fr = f_lseek(fh->fil, 0);
            if (fr != FR_OK) {
                status = fr_to_status(fr);
                goto done;
            }
        }
        fr = f_read(fh->fil, data, length, &got);
        pos = got;
        if (fr != FR_OK)
            status = fr_to_status(fr);
        else if (got < length)
            status = BEC_STATUS_EOF;
        goto done;
    }

    /* Directory listing, Volume Directory, or stat of one object */
    if (flag & BF_FLAG_SEEK0) {
        fh->entnum = 0;
        if (fh->dir != NULL)
            (void) f_rewinddir(fh->dir);
    }
    while (1) {
        FILINFO fno;

        /*
         * Stop if the next entry might not fit. At least one entry is
         * always sent, even if longer than requested (as hostsmash does:
         * the Amiga receives into an oversized buffer).
         */
        if ((pos > 0) && (pos + DIRENT_MAX > length))
            break;

        if (fh->type == BF_TYPE_VOLDIR) {
            if (fh->mode & BF_MODE_DIR) {
                /* Stat of the Volume Directory itself */
                if (fh->entnum++ != 0) {
                    status = BEC_STATUS_EOF;
                    break;
                }
                pos += dirent_put_volume(data + pos, -1, "Volume Directory");
                continue;
            }
            while ((fh->entnum < FF_VOLUMES) && !fsvol[fh->entnum].mounted)
                fh->entnum++;
            if (fh->entnum >= FF_VOLUMES) {
                status = BEC_STATUS_EOF;
                break;
            }
            {
                char vname[sizeof (fsvol[0].name) + 1];
                sprintf(vname, "%s:", fsvol[fh->entnum].name);
                pos += dirent_put_volume(data + pos, fh->entnum, vname);
            }
            fh->entnum++;
            continue;
        }

        if (fh->mode & BF_MODE_DIR) {
            /* Stat of a single object */
            const char *slash;
            char       *dirpath;
            if (fh->entnum++ != 0) {
                status = BEC_STATUS_EOF;
                break;
            }
            if (fh->name[0] == '\0') {
                char vname[sizeof (fsvol[0].name) + 1];
                sprintf(vname, "%s:", fsvol[fh->vol].name);
                pos += dirent_put_volume(data + pos, fh->vol, vname);
                continue;
            }
            fr = fs_stat(fh->vol, fh->name, &fno);
            if (fr != FR_OK) {
                status = fr_to_status(fr);
                break;
            }
            slash = strrchr(fh->name, '/');
            dirpath = malloc(strlen(fh->name) + 1);
            if (dirpath == NULL) {
                status = BEC_STATUS_FAIL;
                break;
            }
            strcpy(dirpath, fh->name);
            if (slash != NULL)
                dirpath[slash - fh->name] = '\0';
            else
                dirpath[0] = '\0';
            pos += dirent_put_file(data + pos, fh, dirpath, &fno);
            free(dirpath);
            continue;
        }

        if (fh->dir == NULL) {
            status = BEC_STATUS_INVALID;
            break;
        }
        fr = f_readdir(fh->dir, &fno);
        if (fr != FR_OK) {
            status = fr_to_status(fr);
            break;
        }
        if (fno.fname[0] == '\0') {
            status = BEC_STATUS_EOF;
            break;
        }
        pos += dirent_put_file(data + pos, fh, fh->name, &fno);
        fh->entnum++;
    }

done:
    rep->bf_length = SWAP32(pos);
    *replylen = sizeof (*rep) + pos;
    if ((status != BEC_STATUS_OK) && (status != BEC_STATUS_EOF))
        fsprintf("fread(%lx) fail %x\n", req->bf_handle, status);
    return (status);
}

static uint
fsrv_fwrite(bf_freadwrite_t *req, uint msglen, uint8_t *reply,
            uint *replylen)
{
    fhandle_t *fh     = fh_get(req->bf_handle);
    uint       length = SWAP32(req->bf_length);
    uint       flag   = SWAP16(req->bf_flag);
    UINT       wrote  = 0;
    uint       status = BEC_STATUS_OK;
    FRESULT    fr     = FR_OK;

    if ((length > BF_RW_MAX) || (length > msglen - sizeof (*req))) {
        status = BEC_STATUS_BADLEN;
        goto done;
    }
    if (fh == NULL) {
        status = BEC_STATUS_STALE;
        goto done;
    }
    if ((fh->fil == NULL) || ((fh->mode & BF_MODE_WRITE) == 0)) {
        status = BEC_STATUS_INVALID;
        goto done;
    }
    if (flag & BF_FLAG_SEEK0)
        fr = f_lseek(fh->fil, 0);
    else if (fh->mode & BF_MODE_APPEND)
        fr = f_lseek(fh->fil, f_size(fh->fil));
    if (fr == FR_OK)
        fr = f_write(fh->fil, req + 1, length, &wrote);
    if (fr != FR_OK)
        status = fr_to_status(fr);
    else if (wrote < length)
        status = BEC_STATUS_FAIL;   // Volume full
done:
    fsprintf("fwrite(%lx l=%x) wrote %x status %x\n", req->bf_handle,
             length, wrote, status);
    req->bf_length = SWAP32(wrote);
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

static uint
fsrv_fseek(bf_fseek_t *req, uint8_t *reply, uint *replylen)
{
    fhandle_t *fh = fh_get(req->bf_handle);
    uint       status = BEC_STATUS_OK;

    fsprintf("fseek(%lx %d)\n", req->bf_handle, req->bf_seek);
    if (fh == NULL) {
        status = BEC_STATUS_STALE;
    } else if (fh->fil == NULL) {
        /* Directories and stat handles can only be rewound */
        req->bf_old_hi = 0;
        req->bf_old_lo = SWAP32(fh->entnum);
        fh->entnum = 0;
        if (fh->dir != NULL)
            (void) f_rewinddir(fh->dir);
    } else {
        int64_t  offset = (int64_t) (((uint64_t) SWAP32(req->bf_off_hi) << 32) |
                                     SWAP32(req->bf_off_lo));
        int64_t  oldpos = f_tell(fh->fil);
        int64_t  eof    = f_size(fh->fil);
        int64_t  target;
        FRESULT  fr;

        switch (req->bf_seek) {
            case SEEK_OFFSET_BEGINNING:
                target = offset;
                break;
            case SEEK_OFFSET_CURRENT:
                target = oldpos + offset;
                break;
            case SEEK_OFFSET_END:
                target = eof + offset;
                break;
            default:
                status = BEC_STATUS_INVALID;
                goto done;
        }
        if ((target < 0) ||
            ((req->bf_flags & FT_SEEK_FLAG_NOEXTEND) && (target > eof))) {
            /* AmigaDOS: seek outside the file fails; position unchanged */
            status = BEC_STATUS_INVALID;
            goto done;
        }
        /* Past EOF extends a file open for write (as POSIX after write) */
        fr = f_lseek(fh->fil, (FSIZE_t) target);
        if (fr != FR_OK) {
            status = fr_to_status(fr);
            goto done;
        }
        target = f_tell(fh->fil);
        req->bf_off_hi = SWAP32((uint32_t) ((uint64_t) target >> 32));
        req->bf_off_lo = SWAP32((uint32_t) target);
        req->bf_old_hi = SWAP32((uint32_t) ((uint64_t) oldpos >> 32));
        req->bf_old_lo = SWAP32((uint32_t) oldpos);
    }
done:
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

static uint
fsrv_fcreate(bf_fopenhandle_t *req, uint8_t *reply, uint *replylen)
{
    const char *aname  = (const char *) (req + 1);
    uint        type   = SWAP16(req->bf_type);
    uint32_t    aperms = SWAP32(req->bf_aperms);
    uint        status;
    int         vol;
    char       *path   = NULL;
    char       *fpath  = NULL;
    FRESULT     fr;

    fsprintf("fcreate(%s) type=%x perms=%lx in %lx\n",
             aname, type, aperms, req->bf_handle);
    status = fs_resolve(req->bf_handle, aname, &vol, &path);
    if (status != BEC_STATUS_OK)
        goto done;
    if ((vol < 0) || (path[0] == '\0')) {
        /* Can't create the Volume Directory or a volume */
        status = BEC_STATUS_INVALID;
        goto done;
    }
    fpath = fs_fatpath(vol, path);
    if (fpath == NULL) {
        status = BEC_STATUS_INVALID;
        goto done;
    }
    switch (type) {
        case BF_TYPE_FILE: {
            FIL *fil = malloc(sizeof (*fil));
            if (fil == NULL) {
                status = BEC_STATUS_FAIL;
                goto done;
            }
            fr = f_open(fil, fpath, FA_WRITE | FA_CREATE_NEW);
            if (fr == FR_OK)
                fr = f_close(fil);
            free(fil);
            break;
        }
        case BF_TYPE_DIR:
            fr = f_mkdir(fpath);
            break;
        default:
            /* FAT has no links, devices, etc */
            status = BEC_STATUS_INVALID;
            goto done;
    }
    if ((fr == FR_OK) && (fat_from_aperms(aperms) != AM_ARC))
        fr = f_chmod(fpath, fat_from_aperms(aperms), AM_RDO | AM_ARC);
    status = fr_to_status(fr);
done:
    free(path);
    free(fpath);
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

static uint
fsrv_fdelete(bf_fhandle_t *req, uint8_t *reply, uint *replylen)
{
    const char *aname = (const char *) (req + 1);
    uint        status;
    int         vol;
    char       *path  = NULL;
    char       *fpath = NULL;
    FILINFO     fno;
    FRESULT     fr;

    fsprintf("fdelete(%s) in %lx\n", aname, req->bf_handle);
    status = fs_resolve(req->bf_handle, aname, &vol, &path);
    if (status != BEC_STATUS_OK)
        goto done;
    if (vol < 0) {
        status = BEC_STATUS_INVALID;
        goto done;
    }
    if (path[0] == '\0') {
        status = BEC_STATUS_PERM;   // Can't delete a volume
        goto done;
    }
    fr = fs_stat(vol, path, &fno);
    if (fr != FR_OK) {
        status = fr_to_status(fr);
        goto done;
    }
    if (fh_find_open(vol, fsvol[vol].serial, path, 0, 0) != NULL) {
        /* Locked or open (FatFs must not delete an open object) */
        fsprintf("fdelete(%s) refused: object in use\n", path);
        status = BEC_STATUS_INUSE;
        goto done;
    }
    fpath = fs_fatpath(vol, path);
    if (fpath == NULL) {
        status = BEC_STATUS_INVALID;
        goto done;
    }
    if (fno.fattrib & AM_DIR) {
        DIR dir;
        fr = f_opendir(&dir, fpath);
        if (fr == FR_OK) {
            fr = f_readdir(&dir, &fno);
            (void) f_closedir(&dir);
            if ((fr == FR_OK) && (fno.fname[0] != '\0')) {
                status = BEC_STATUS_NOTEMPTY;
                goto done;
            }
        }
    }
    status = fr_to_status(f_unlink(fpath));
done:
    free(path);
    free(fpath);
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

/* Update the paths of handles below a renamed object */
static void
handles_renamed(int vol, uint32_t serial, const char *oldpath,
                const char *newpath)
{
    size_t     olen = strlen(oldpath);
    fhandle_t *fh;

    for (fh = fh_head; fh != NULL; fh = fh->next) {
        char *nname;
        if ((fh->vol != vol) || (fh->serial != serial) ||
            !name_ncaseeq(fh->name, oldpath, olen) ||
            ((fh->name[olen] != '\0') && (fh->name[olen] != '/')))
            continue;
        nname = malloc(strlen(newpath) + strlen(fh->name + olen) + 1);
        if (nname == NULL)
            continue;
        strcpy(nname, newpath);
        strcat(nname, fh->name + olen);
        free(fh->name);
        fh->name = nname;
    }
}

static uint
fsrv_frename(bf_frename_t *req, uint8_t *reply, uint *replylen)
{
    const char *oname  = (const char *) (req + 1);
    const char *nname  = oname + strlen(oname) + 1;
    uint        status;
    int         ovol;
    int         nvol;
    char       *opath  = NULL;
    char       *npath  = NULL;
    char       *ofpath = NULL;
    char       *nfpath = NULL;
    FILINFO     fno;
    FRESULT     fr;

    fsprintf("frename(%s to %s) in %lx to %lx\n",
             oname, nname, req->bf_shandle, req->bf_dhandle);
    status = fs_resolve(req->bf_shandle, oname, &ovol, &opath);
    if (status != BEC_STATUS_OK)
        goto done;
    status = fs_resolve(req->bf_dhandle, nname, &nvol, &npath);
    if (status != BEC_STATUS_OK)
        goto done;
    if ((ovol < 0) || (nvol < 0) || (ovol != nvol)) {
        /* Not the Volume Directory, and not across volumes */
        status = BEC_STATUS_INVALID;
        goto done;
    }
    if ((opath[0] == '\0') || (npath[0] == '\0')) {
        status = BEC_STATUS_PERM;   // Can't rename a volume root
        goto done;
    }
    fr = fs_stat(ovol, opath, &fno);
    if (fr != FR_OK) {
        status = fr_to_status(fr);
        goto done;
    }
    if (fh_find_open(ovol, fsvol[ovol].serial, opath, 2, 0) != NULL) {
        /* FatFs updates an open file's entry where it was opened */
        status = BEC_STATUS_INUSE;
        goto done;
    }
    ofpath = fs_fatpath(ovol, opath);
    nfpath = fs_fatpath(nvol, npath);
    if ((ofpath == NULL) || (nfpath == NULL)) {
        status = BEC_STATUS_INVALID;
        goto done;
    }
    /*
     * As with AmigaOS, an existing object is never replaced. FatFs
     * refuses with FR_EXIST, but allows renaming an object onto itself
     * (changing only the case of the name).
     */
    fr = f_rename(ofpath, nfpath);
    status = fr_to_status(fr);
    if (fr == FR_OK)
        handles_renamed(ovol, fsvol[ovol].serial, opath, npath);
done:
    free(opath);
    free(npath);
    free(ofpath);
    free(nfpath);
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

static uint
fsrv_fpath(bf_fhandle_t *req, uint8_t *reply, uint *replylen)
{
    fhandle_t *fh = fh_get(req->bf_handle);
    char      *path = (char *) reply + sizeof (*req);
    uint       maxlen = BEC_MSG_MAX_PAYLOAD - sizeof (*req) - 1;

    fsprintf("fpath(%lx)\n", req->bf_handle);
    memcpy(reply, req, sizeof (*req));
    *replylen = sizeof (*req);
    fsvol_refresh();
    if (req->bf_handle == HANDLE_VOLDIR) {
        strcpy(path, "::");
    } else if (req->bf_handle == HANDLE_DEFAULT) {
        if (!fsvol[0].mounted)
            return (BEC_STATUS_UNAVAIL);
        sprintf(path, "%s:", fsvol[0].name);
    } else if ((fh == NULL) || !fsvol_same(fh->vol, fh->serial)) {
        return (BEC_STATUS_STALE);
    } else if (fh->vol < 0) {
        strcpy(path, "::");
    } else {
        size_t vlen = strlen(fsvol[fh->vol].name) + 1;
        if (vlen + strlen(fh->name) > maxlen)
            return (BEC_STATUS_FAIL);
        sprintf(path, "%s:%s", fsvol[fh->vol].name, fh->name);
    }
    *replylen += strlen(path) + 1;
    return (BEC_STATUS_OK);
}

/*
 * Resolve a request's name relative to its handle to an existing object
 * on a volume (not the Volume Directory, nor a volume root). On success,
 * *fpathp is the malloc()ed FatFs path.
 */
static uint
fs_resolve_object(uint32_t phandle, const char *aname, char **fpathp,
                  FILINFO *fno)
{
    uint    status;
    int     vol;
    char   *path = NULL;
    FRESULT fr;

    *fpathp = NULL;
    status = fs_resolve(phandle, aname, &vol, &path);
    if (status != BEC_STATUS_OK)
        return (status);
    if (vol < 0) {
        status = BEC_STATUS_INVALID;
    } else if (path[0] == '\0') {
        status = BEC_STATUS_PERM;
    } else if ((fr = fs_stat(vol, path, fno)) != FR_OK) {
        status = fr_to_status(fr);
    } else if ((*fpathp = fs_fatpath(vol, path)) == NULL) {
        status = BEC_STATUS_INVALID;
    }
    free(path);
    return (status);
}

static uint
fsrv_fsetperms(bf_fopenhandle_t *req, uint8_t *reply, uint *replylen)
{
    const char *aname  = (const char *) (req + 1);
    uint32_t    aperms = SWAP32(req->bf_aperms);
    char       *fpath;
    FILINFO     fno;
    uint        status;

    fsprintf("fsetperms(%s %lx)\n", aname, aperms);
    status = fs_resolve_object(req->bf_handle, aname, &fpath, &fno);
    if (status == BEC_STATUS_OK)
        status = fr_to_status(f_chmod(fpath, fat_from_aperms(aperms),
                                      AM_RDO | AM_ARC));
    free(fpath);
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

static uint
fsrv_fsetown(bf_fsetown_t *req, uint8_t *reply, uint *replylen)
{
    const char *aname = (const char *) (req + 1);
    char       *fpath;
    FILINFO     fno;
    uint        status;

    /* FAT has no owner or group: accept if the object exists */
    fsprintf("fsetown(%s)\n", aname);
    status = fs_resolve_object(req->bf_handle, aname, &fpath, &fno);
    free(fpath);
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

static uint
fsrv_fsetdate(bf_fsetdate_t *req, uint8_t *reply, uint *replylen)
{
    const char *aname = (const char *) (req + 1);
    char       *fpath;
    FILINFO     fno;
    uint        status;

    fsprintf("fsetdate(%s %u %lu)\n", aname, req->bf_which,
             SWAP32(req->bf_time));
    status = fs_resolve_object(req->bf_handle, aname, &fpath, &fno);
    if (status != BEC_STATUS_OK)
        goto done;
    switch (req->bf_which) {
        case 0:  // Set the modify date/time
            secs_to_fat(SWAP32(req->bf_time), &fno.fdate, &fno.ftime);
            status = fr_to_status(f_utime(fpath, &fno));
            break;
        case 1:  // Get the modify date/time
        case 3:  // Get the change date/time (FAT doesn't have it)
        case 5:  // Get the access date/time (FAT doesn't have it)
            req->bf_time    = SWAP32(fat_to_secs(fno.fdate, fno.ftime));
            req->bf_time_ns = 0;
            break;
        case 4:  // Set the access date/time (FAT doesn't have it)
            break;
        case 2:  // Set the change date/time
        default:
            status = BEC_STATUS_INVALID;
            break;
    }
done:
    free(fpath);
    REPLY_REQ(req, sizeof (*req));
    return (status);
}

/* ------------------------------------------------------------------ */

uint
fsrv_cmd(uint cmd, const uint8_t *msg, uint msglen, const void **reply,
         uint *replylen)
{
    uint8_t *req = (uint8_t *) fsrv_reqbuf;
    uint8_t *rep = (uint8_t *) fsrv_replybuf;
    uint     minlen;

    *reply = rep;
    *replylen = 0;

    switch (cmd) {
        case BEC_CMD_FOPEN:
        case BEC_CMD_FCLOSE:
        case BEC_CMD_FCREATE:
        case BEC_CMD_FSETPERMS:
            minlen = sizeof (bf_fopenhandle_t);
            break;
        case BEC_CMD_FREAD:
        case BEC_CMD_FWRITE:
            minlen = sizeof (bf_freadwrite_t);
            break;
        case BEC_CMD_FSEEK:
            minlen = sizeof (bf_fseek_t);
            break;
        case BEC_CMD_FDELETE:
        case BEC_CMD_FPATH:
            minlen = sizeof (bf_fhandle_t);
            break;
        case BEC_CMD_FRENAME:
            minlen = sizeof (bf_frename_t);
            break;
        case BEC_CMD_FSETOWN:
            minlen = sizeof (bf_fsetown_t);
            break;
        case BEC_CMD_FSETDATE:
            minlen = sizeof (bf_fsetdate_t);
            break;
        default:
            return (BEC_STATUS_UNKCMD);
    }
    if ((msglen < minlen) || (msglen > BEC_MSG_MAX_PAYLOAD))
        return (BEC_STATUS_BADLEN);

    /*
     * Copy the request: the message buffer is not aligned, and may be
     * overwritten by a new message while USB storage I/O is in progress.
     * NIL padding terminates any names which the Amiga didn't terminate
     * (a rename's second name may begin just past the payload).
     */
    memcpy(req, msg, msglen);
    memset(req + msglen, 0, sizeof (fsrv_reqbuf) - msglen);

    switch (cmd) {
        case BEC_CMD_FOPEN:
            return (fsrv_fopen((bf_fopenhandle_t *) req, rep, replylen));
        case BEC_CMD_FCLOSE:
            return (fsrv_fclose((bf_fopenhandle_t *) req, rep, replylen));
        case BEC_CMD_FREAD:
            return (fsrv_fread((bf_freadwrite_t *) req, rep, replylen));
        case BEC_CMD_FWRITE:
            return (fsrv_fwrite((bf_freadwrite_t *) req, msglen, rep,
                                replylen));
        case BEC_CMD_FSEEK:
            return (fsrv_fseek((bf_fseek_t *) req, rep, replylen));
        case BEC_CMD_FCREATE:
            return (fsrv_fcreate((bf_fopenhandle_t *) req, rep, replylen));
        case BEC_CMD_FDELETE:
            return (fsrv_fdelete((bf_fhandle_t *) req, rep, replylen));
        case BEC_CMD_FRENAME:
            return (fsrv_frename((bf_frename_t *) req, rep, replylen));
        case BEC_CMD_FPATH:
            return (fsrv_fpath((bf_fhandle_t *) req, rep, replylen));
        case BEC_CMD_FSETPERMS:
            return (fsrv_fsetperms((bf_fopenhandle_t *) req, rep, replylen));
        case BEC_CMD_FSETOWN:
            return (fsrv_fsetown((bf_fsetown_t *) req, rep, replylen));
        case BEC_CMD_FSETDATE:
        default:
            return (fsrv_fsetdate((bf_fsetdate_t *) req, rep, replylen));
    }
}

void
fsrv_sync_all(void)
{
    fhandle_t *fh;

    for (fh = fh_head; fh != NULL; fh = fh->next)
        if ((fh->fil != NULL) && (fh->mode & BF_MODE_WRITE))
            (void) f_sync(fh->fil);
}

void
fsrv_show_handles(void)
{
    static const char * const type_str[] = {
        "?", "File", "Dir", "Link", "HLink", "?", "?", "?", "?", "?",
        "Vol", "VolDir"
    };
    fhandle_t *fh;
    uint vol;

    fsvol_refresh();
    for (vol = 0; vol < FF_VOLUMES; vol++)
        if (fsvol[vol].mounted)
            printf("Volume %u: %s: serial %08lx\n",
                   vol, fsvol[vol].name, fsvol[vol].serial);
    if (fh_head == NULL) {
        printf("No open handles\n");
        return;
    }
    printf("Handle   Type   Mode Lock Vol Path\n");
    for (fh = fh_head; fh != NULL; fh = fh->next) {
        printf("%08lx %-6s %04x %-4s ", fh->handle,
               (fh->type < ARRAY_SIZE(type_str)) ? type_str[fh->type] : "?",
               fh->mode,
               (fh->lock == BF_MODE_LOCK_EX) ? "Excl" :
               (fh->lock == BF_MODE_LOCK) ? "Shrd" : "");
        if (fh->vol < 0)
            printf("  - ::\n");
        else
            printf("%3d %s:%s%s\n", fh->vol,
                   fsvol[fh->vol].mounted ? fsvol[fh->vol].name : "?",
                   fh->name, fsvol_same(fh->vol, fh->serial) ? "" :
                   " (stale)");
    }
}

/* ------------------------------------------------------------------ */
/* Self test ("fs test"): sends requests through fsrv_cmd() as the     */
/* Amiga would, and checks the replies.                                */

static uint8_t  fst_msg[BEC_MSG_MAX_PAYLOAD];
static uint     fst_fail;

static uint
fst_send(uint cmd, const void *req, uint reqlen, const char *name,
         const char *name2, const void *data, uint datalen,
         const uint8_t **reply, uint *rlen)
{
    uint len = reqlen;
    uint rc;
    const void *rep;

    memcpy(fst_msg, req, reqlen);
    if (name != NULL) {
        strcpy((char *) fst_msg + len, name);
        len += strlen(name) + 1;
    }
    if (name2 != NULL) {
        strcpy((char *) fst_msg + len, name2);
        len += strlen(name2) + 1;
    }
    if (data != NULL) {
        memcpy(fst_msg + len, data, datalen);
        len += datalen;
    }
    rc = fsrv_cmd(cmd, fst_msg, len, &rep, rlen);
    *reply = rep;
    return (rc);
}

static void
fst_check(const char *what, uint rc, uint expect)
{
    printf("  %-34s status %02x %s\n", what, rc,
           (rc == expect) ? "ok" : "FAIL");
    if (rc != expect) {
        printf("    expected %02x\n", expect);
        fst_fail++;
    }
}

static uint32_t
fst_open(uint32_t parent, const char *name, uint mode, uint expect,
         const char *what)
{
    bf_fopenhandle_t req;
    const uint8_t   *rep;
    uint             rlen;
    uint             rc;
    uint32_t         handle;

    memset(&req, 0, sizeof (req));
    req.bf_handle = parent;
    req.bf_mode   = SWAP16(mode);
    rc = fst_send(BEC_CMD_FOPEN, &req, sizeof (req), name, NULL, NULL, 0,
                  &rep, &rlen);
    fst_check(what, rc, expect);
    memcpy(&handle, rep, sizeof (handle));
    return ((rc == BEC_STATUS_OK) ? handle : 0);
}

static void
fst_close(uint32_t handle)
{
    bf_fopenhandle_t req;
    const uint8_t   *rep;
    uint             rlen;

    memset(&req, 0, sizeof (req));
    req.bf_handle = handle;
    fst_check("close", fst_send(BEC_CMD_FCLOSE, &req, sizeof (req), NULL,
                                NULL, NULL, 0, &rep, &rlen), BEC_STATUS_OK);
}

/* Read a directory, listing up to maxshow entries; returns entry count */
static uint
fst_list(uint32_t handle, uint maxshow, uint length)
{
    bf_freadwrite_t req;
    const uint8_t  *rep;
    uint            rlen;
    uint            rc;
    uint            count = 0;

    memset(&req, 0, sizeof (req));
    req.bf_handle = handle;
    req.bf_length = SWAP32(length);
    do {
        uint pos;
        uint dlen;
        rc = fst_send(BEC_CMD_FREAD, &req, sizeof (req), NULL, NULL,
                      NULL, 0, &rep, &rlen);
        if ((rc != BEC_STATUS_OK) && (rc != BEC_STATUS_EOF)) {
            fst_check("read directory", rc, BEC_STATUS_OK);
            break;
        }
        memcpy(&dlen, rep + 4, 4);
        dlen = SWAP32(dlen);
        if (dlen + sizeof (req) != rlen) {
            printf("    reply length %u != %u + %u\n",
                   rlen, sizeof (req), dlen);
            fst_fail++;
            break;
        }
        for (pos = 0; pos < dlen; ) {
            bf_fdirent_t de;
            const char  *name = (const char *) rep + sizeof (req) +
                                pos + sizeof (de);
            memcpy(&de, rep + sizeof (req) + pos, sizeof (de));
            if (count < maxshow) {
                printf("    %-2s %10lu %08lx %s\n",
                       (SWAP16(de.bfd_type) == BF_TYPE_DIR) ? "D" :
                       (SWAP16(de.bfd_type) == BF_TYPE_FILE) ? "F" :
                       (SWAP16(de.bfd_type) == BF_TYPE_VOLUME) ? "V" : "?",
                       SWAP32(de.bfd_size_lo), SWAP32(de.bfd_aperms), name);
            }
            count++;
            pos += sizeof (de) + SWAP16(de.bfd_elen);
        }
        req.bf_flag = 0;
    } while (rc == BEC_STATUS_OK);
    if (count > maxshow)
        printf("    ... %u entries\n", count);
    return (count);
}

static uint
fst_read(uint32_t handle, uint length, uint flag, uint8_t *buf, uint *got)
{
    bf_freadwrite_t req;
    const uint8_t  *rep;
    uint            rlen;
    uint            rc;

    memset(&req, 0, sizeof (req));
    req.bf_handle = handle;
    req.bf_length = SWAP32(length);
    req.bf_flag   = SWAP16(flag);
    rc = fst_send(BEC_CMD_FREAD, &req, sizeof (req), NULL, NULL, NULL, 0,
                  &rep, &rlen);
    *got = rlen - sizeof (req);
    if (buf != NULL)
        memcpy(buf, rep + sizeof (req), *got);
    return (rc);
}

static uint
fst_simple(uint cmd, uint32_t handle, const char *name, const char *name2,
           uint32_t arg)
{
    const uint8_t *rep;
    uint           rlen;

    switch (cmd) {
        case BEC_CMD_FRENAME: {
            bf_frename_t req = { handle, handle };
            return (fst_send(cmd, &req, sizeof (req), name, name2, NULL, 0,
                             &rep, &rlen));
        }
        case BEC_CMD_FSETDATE: {
            bf_fsetdate_t req;
            memset(&req, 0, sizeof (req));
            req.bf_handle = handle;
            req.bf_which  = 0;
            req.bf_time   = SWAP32(arg);
            return (fst_send(cmd, &req, sizeof (req), name, NULL, NULL, 0,
                             &rep, &rlen));
        }
        case BEC_CMD_FCREATE:
        case BEC_CMD_FSETPERMS: {
            bf_fopenhandle_t req;
            memset(&req, 0, sizeof (req));
            req.bf_handle = handle;
            req.bf_type   = SWAP16(BF_TYPE_FILE);
            req.bf_aperms = SWAP32(arg);
            if ((cmd == BEC_CMD_FCREATE) && (name2 != NULL))
                req.bf_type = SWAP16(BF_TYPE_DIR);
            return (fst_send(cmd, &req, sizeof (req), name, NULL, NULL, 0,
                             &rep, &rlen));
        }
        default: {
            bf_fhandle_t req = { handle };
            return (fst_send(cmd, &req, sizeof (req), name, NULL, NULL, 0,
                             &rep, &rlen));
        }
    }
}

/*
 * fsrv_test() - exercise the file service. Without write, only reads the
 * first volume. With write, creates, writes, renames and deletes
 * BECTEST.TMP and BECTEST.DIR in the volume root.
 */
void
fsrv_test(uint write)
{
    static uint8_t buf[BF_RW_MAX];
    const uint8_t *rep;
    uint32_t       h;
    uint32_t       hroot;
    uint           rlen;
    uint           got;
    uint           rc;
    char           vname[40];
    bf_fhandle_t   fhreq;

    fst_fail = 0;
    printf("Volume Directory:\n");
    h = fst_open(HANDLE_VOLDIR, "", BF_MODE_READ, BEC_STATUS_OK,
                 "open ::");
    if (h != 0) {
        fst_list(h, 8, BF_RW_MAX);
        fst_close(h);
    }
    fsvol_refresh();
    if (!fsvol[0].mounted) {
        printf("No volume 0\n");
        return;
    }
    sprintf(vname, "%s:", fsvol[0].name);

    printf("Volume root %s\n", vname);
    hroot = fst_open(HANDLE_VOLDIR, vname, BF_MODE_READ | BF_MODE_LOCK,
                     BEC_STATUS_OK, "open volume root (shared lock)");
    if (hroot == 0)
        return;
    fst_open(hroot, "", BF_MODE_READ | BF_MODE_LOCK_EX, BEC_STATUS_INUSE,
             "exclusive lock while shared");
    h = fst_open(hroot, "", BF_MODE_DIR | BF_MODE_READ, BEC_STATUS_OK,
                 "stat volume root");
    if (h != 0) {
        fst_list(h, 1, 64);
        fst_close(h);
    }
    h = fst_open(HANDLE_DEFAULT, "", BF_MODE_READ, BEC_STATUS_OK,
                 "open default volume");
    if (h != 0) {
        printf("    %u entries (requested 400 bytes per read)\n",
               fst_list(h, 4, 400));
        fst_close(h);
    }
    fhreq.bf_handle = hroot;
    rc = fst_send(BEC_CMD_FPATH, &fhreq, sizeof (fhreq), NULL, NULL, NULL, 0,
                  &rep, &rlen);
    fst_check("fpath root", rc, BEC_STATUS_OK);
    printf("    path \"%s\"\n", (const char *) rep + sizeof (fhreq));
    fst_open(hroot, "no_such_file", BF_MODE_READ, BEC_STATUS_NOEXIST,
             "open missing file");
    fst_open(0x12345678, "x", BF_MODE_READ, BEC_STATUS_STALE,
             "open with stale handle");

    /* First file in the root */
    {
        DIR     dir;
        FILINFO fno;
        char    drv[4] = "0:/";
        fno.fname[0] = '\0';
        if (f_opendir(&dir, drv) == FR_OK) {
            do {
                if (f_readdir(&dir, &fno) != FR_OK)
                    fno.fname[0] = '\0';
            } while ((fno.fname[0] != '\0') && (fno.fattrib & AM_DIR));
            f_closedir(&dir);
        }
        utf8_to_latin1(fno.fname);
        if (fno.fname[0] != '\0') {
            uint32_t sum1 = 0;
            uint32_t sum2;
            uint     pos;
            printf("File %s (%lu bytes)\n", fno.fname, (uint32_t) fno.fsize);
            h = fst_open(hroot, fno.fname, BF_MODE_READ, BEC_STATUS_OK,
                         "open file");
            if (h != 0) {
                rc = fst_read(h, sizeof (buf), 0, buf, &got);
                fst_check("read first block", rc, BEC_STATUS_OK);
                for (pos = 0; pos < got; pos++)
                    sum1 += buf[pos];
                printf("    got %u bytes, sum %08lx\n", got, sum1);
                rc = fst_read(h, sizeof (buf), BF_FLAG_SEEK0, buf, &got);
                for (sum2 = 0, pos = 0; pos < got; pos++)
                    sum2 += buf[pos];
                fst_check("re-read with SEEK0 matches", rc,
                          (sum2 == sum1) ? BEC_STATUS_OK : 0xff);
                {
                    bf_fseek_t sreq;
                    memset(&sreq, 0, sizeof (sreq));
                    sreq.bf_handle = h;
                    sreq.bf_seek   = SEEK_OFFSET_END;
                    sreq.bf_off_lo = SWAP32(-16);
                    sreq.bf_off_hi = SWAP32(-1);
                    rc = fst_send(BEC_CMD_FSEEK, &sreq, sizeof (sreq), NULL,
                                  NULL, NULL, 0, &rep, &rlen);
                    fst_check("seek 16 before end", rc, BEC_STATUS_OK);
                    rc = fst_read(h, 64, 0, buf, &got);
                    fst_check("read at end gives EOF", rc, BEC_STATUS_EOF);
                    printf("    got %u bytes\n", got);
                    sreq.bf_seek   = SEEK_OFFSET_END;
                    sreq.bf_off_lo = SWAP32(1);
                    sreq.bf_off_hi = 0;
                    sreq.bf_flags  = FT_SEEK_FLAG_NOEXTEND;
                    rc = fst_send(BEC_CMD_FSEEK, &sreq, sizeof (sreq), NULL,
                                  NULL, NULL, 0, &rep, &rlen);
                    fst_check("seek past end (NOEXTEND)", rc,
                              BEC_STATUS_INVALID);
                }
                fst_close(h);
                h = fst_open(hroot, fno.fname, BF_MODE_DIR | BF_MODE_READ,
                             BEC_STATUS_OK, "stat file");
                if (h != 0) {
                    fst_list(h, 1, 64);
                    fst_close(h);
                }
            }
        }
    }

    if (write) {
        static const char text[] = "Hello from the BEC file service\n";
        printf("Write tests\n");
        rc = fst_simple(BEC_CMD_FCREATE, hroot, "BECTEST.DIR", "", 0);
        fst_check("create directory BECTEST.DIR", rc, BEC_STATUS_OK);
        h = fst_open(hroot, "BECTEST.DIR/test file.txt",
                     BF_MODE_WRITE | BF_MODE_CREATE | BF_MODE_TRUNC,
                     BEC_STATUS_OK, "open new file for write");
        if (h != 0) {
            bf_freadwrite_t wreq;
            uint i;
            fst_open(hroot, "BECTEST.DIR/test file.txt", BF_MODE_WRITE,
                     BEC_STATUS_INUSE, "second writer refused");
            memset(&wreq, 0, sizeof (wreq));
            wreq.bf_handle = h;
            for (i = 0; i < sizeof (buf); i++)
                buf[i] = text[i % (sizeof (text) - 1)];
            wreq.bf_length = SWAP32(sizeof (buf));
            for (i = 0; i < 3; i++) {
                rc = fst_send(BEC_CMD_FWRITE, &wreq, sizeof (wreq), NULL,
                              NULL, buf, sizeof (buf), &rep, &rlen);
                fst_check("write block", rc, BEC_STATUS_OK);
            }
            fst_close(h);
        }
        rc = fst_simple(BEC_CMD_FDELETE, hroot, "BECTEST.DIR", NULL, 0);
        fst_check("delete non-empty dir refused", rc, BEC_STATUS_NOTEMPTY);
        rc = fst_simple(BEC_CMD_FRENAME, hroot, "BECTEST.DIR/test file.txt",
                        "BECTEST.DIR/Renamed \xe9t\xe9.txt", 0);
        fst_check("rename (Latin-1 name)", rc, BEC_STATUS_OK);
        rc = fst_simple(BEC_CMD_FSETDATE, hroot,
                        "BECTEST.DIR/Renamed \xe9t\xe9.txt", NULL,
                        rtc_to_utc(2001, 2, 3, 4, 5, 6));
        fst_check("set date 2001-02-03 04:05:06", rc, BEC_STATUS_OK);
        rc = fst_simple(BEC_CMD_FSETPERMS, hroot,
                        "BECTEST.DIR/Renamed \xe9t\xe9.txt", NULL,
                        FIBF_WRITE | FIBF_DELETE);
        fst_check("set write protect", rc, BEC_STATUS_OK);
        h = fst_open(hroot, "BECTEST.DIR", BF_MODE_READ, BEC_STATUS_OK,
                     "open BECTEST.DIR");
        if (h != 0) {
            fst_list(h, 4, BF_RW_MAX);
            fst_close(h);
        }
        h = fst_open(hroot, "BECTEST.DIR/Renamed \xe9t\xe9.txt",
                     BF_MODE_READ, BEC_STATUS_OK, "open renamed file");
        if (h != 0) {
            uint total = 0;
            uint bad = 0;
            do {
                uint i;
                rc = fst_read(h, sizeof (buf), 0, buf, &got);
                /* Each block written restarted the text at its start */
                for (i = 0; i < got; i++)
                    if (buf[i] != (uint8_t)
                        text[((total + i) % sizeof (buf)) %
                             (sizeof (text) - 1)])
                        bad++;
                total += got;
            } while (rc == BEC_STATUS_OK);
            fst_check("read back to EOF", rc, BEC_STATUS_EOF);
            fst_check("data matches", (bad == 0) && (total == 3 * sizeof (buf)) ?
                      BEC_STATUS_OK : BEC_STATUS_FAIL, BEC_STATUS_OK);
            printf("    read %u bytes, %u mismatches\n", total, bad);
            rc = fst_simple(BEC_CMD_FDELETE, hroot,
                            "BECTEST.DIR/Renamed \xe9t\xe9.txt", NULL, 0);
            fst_check("delete open file refused", rc, BEC_STATUS_INUSE);
            fst_close(h);
        }
        rc = fst_simple(BEC_CMD_FDELETE, hroot,
                        "BECTEST.DIR/Renamed \xe9t\xe9.txt", NULL, 0);
        fst_check("delete write-protected refused", rc, BEC_STATUS_PERM);
        rc = fst_simple(BEC_CMD_FSETPERMS, hroot,
                        "BECTEST.DIR/Renamed \xe9t\xe9.txt", NULL, 0);
        fst_check("clear write protect", rc, BEC_STATUS_OK);
        rc = fst_simple(BEC_CMD_FDELETE, hroot,
                        "BECTEST.DIR/Renamed \xe9t\xe9.txt", NULL, 0);
        fst_check("delete file", rc, BEC_STATUS_OK);
        rc = fst_simple(BEC_CMD_FDELETE, hroot, "BECTEST.DIR", NULL, 0);
        fst_check("delete directory", rc, BEC_STATUS_OK);
    }
    fst_close(hroot);
    printf("%s: %u failure%s\n", fst_fail ? "FAILED" : "Passed", fst_fail,
           (fst_fail == 1) ? "" : "s");
}
