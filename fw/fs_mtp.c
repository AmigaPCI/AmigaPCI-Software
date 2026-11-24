/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * Filesystem access to USB MTP device storage (see fs_mtp.h).
 *
 * MTP has no paths: objects are numbered (handles), and each knows its
 * parent. Paths are resolved one element at a time, by finding the
 * child of a directory with a matching name (case-insensitive, as on the
 * Amiga). To keep that from costing a transaction per directory entry,
 * a directory's names are hashed into a cache, filled with a single
 * GetObjectPropList (filename) transaction where the device supports it.
 * Resolved paths are also cached. The caches are dropped whenever a
 * device reports that objects changed.
 *
 * Writing uses the Android MTP edit extensions (BeginEditObject,
 * SendPartialObject, TruncateObject, EndEditObject), which allow a file
 * to be written in pieces. Storage on devices without them is read-only
 * (except for creating directories and deleting objects).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "printf.h"
#include "crc32.h"
#include "utils.h"
#include "usb_mtp.h"
#include "fs_mtp.h"
#include <usbh_core.h>
#include <usbh_mtp.h>

#define MTP_ROOT        0xffffffff  // Parent handle of storage root objects
#define MTP_DIR_MAX     4096        // Maximum directory entries handled
#define MTP_DCACHES     4           // Directory name caches
#define MTP_DCACHE_ENTS 4096        // Total directory cache entries
#define MTP_PCACHES     8           // Resolved path cache entries
#define MTP_RBUF_SIZE   4096        // File read-ahead buffer
#define MTP_WBUF_SIZE   4096        // File write-behind buffer
#define MTP_OI_SIZE     1024        // ObjectInfo dataset buffer

#define PTP_FMT_UNDEFINED           0x3000
#define PTP_FMT_ASSOCIATION         0x3001

uint8_t fs_mtp_debug;

#define mprintf(...) do { if (fs_mtp_debug) printf(__VA_ARGS__); } while (0)

typedef struct {
    uint8_t  used;
    uint8_t  dev;           // usb_mtp device number
    uint8_t  readonly;
    uint8_t  no_proplist;   // GetObjectPropList failed: don't use it
    uint16_t ops;           // MTP_OPS_*
    uint32_t storage;       // Storage ID
    uint32_t serial;
    char     name[36];
} mvol_t;

typedef struct {
    uint32_t handle;
    uint32_t hash;          // Hash of the case-folded name
} dent_t;

typedef struct {
    uint8_t  used;
    uint8_t  mvol;
    uint8_t  complete;      // All of the directory's entries are present
    uint32_t parent;
    uint32_t change;        // usb_mtp_changes() when filled
    uint32_t lru;
    uint32_t n;
    uint32_t max;
    dent_t  *ent;
} dcache_t;

typedef struct {
    uint8_t  used;
    uint8_t  mvol;
    uint32_t hash;          // Hash of the case-folded path
    uint32_t handle;
    uint32_t change;
    uint32_t lru;
} pcache_t;

/* Decoded ObjectInfo */
typedef struct {
    uint32_t storage;
    uint32_t parent;        // MTP_ROOT for storage root objects
    uint16_t format;
    uint16_t prot;
    uint64_t size;
    char     date[20];      // ModificationDate (or CaptureDate)
    char     name[FF_LFN_BUF + 1];
} oinfo_t;

struct mtpdir {
    uint8_t   mvol;
    uint32_t  n;            // Number of entries
    uint32_t  pos;          // Next entry to read
    uint32_t *handles;
};

struct mtpfile {
    uint8_t   mvol;
    uint8_t   mode;         // FA_READ | FA_WRITE
    uint8_t   editing;      // BeginEditObject sent
    uint32_t  handle;
    FSIZE_t   size;
    FSIZE_t   pos;
    uint8_t  *rbuf;         // Read-ahead data
    FSIZE_t   rbuf_off;
    uint32_t  rbuf_len;
    uint8_t  *wbuf;         // Data not yet sent
    FSIZE_t   wbuf_off;
    uint32_t  wbuf_len;
};

static mvol_t   mvols[FS_MTP_VOLS];
static uint32_t mvol_sig;
static uint8_t  mvol_valid;
static dcache_t dcache[MTP_DCACHES];
static uint32_t dcache_ents;    // Entries allocated over all caches
static pcache_t pcache[MTP_PCACHES];
static uint32_t cache_lru;
static uint8_t  oi_buf[MTP_OI_SIZE];
static uint8_t  stage_buf[1024];  // Staging for streamed data phases

/* ------------------------------------------------------------------ */
/* Helpers                                                             */

static uint32_t
name_hash(uint32_t seed, const char *name, size_t len)
{
    uint32_t crc = seed;
    size_t   i;

    for (i = 0; i < len; i++) {
        char ch = name[i];
        if ((ch >= 'a') && (ch <= 'z'))
            ch -= 'a' - 'A';
        crc = crc32(crc, &ch, 1);
    }
    return (crc);
}

/* ASCII case-insensitive compare of a name with len bytes of another */
static int
name_eq(const char *a, const char *b, size_t blen)
{
    size_t i;

    for (i = 0; i < blen; i++) {
        char ca = a[i];
        char cb = b[i];
        if (ca == '\0')
            return (0);
        if ((ca >= 'a') && (ca <= 'z'))
            ca -= 'a' - 'A';
        if ((cb >= 'a') && (cb <= 'z'))
            cb -= 'a' - 'A';
        if (ca != cb)
            return (0);
    }
    return (a[blen] == '\0');
}

static uint32_t
get_le32(const uint8_t *p)
{
    return (p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24));
}

static uint8_t *
put_le16(uint8_t *p, uint16_t val)
{
    p[0] = (uint8_t) val;
    p[1] = (uint8_t) (val >> 8);
    return (p + 2);
}

static uint8_t *
put_le32(uint8_t *p, uint32_t val)
{
    p[0] = (uint8_t) val;
    p[1] = (uint8_t) (val >> 8);
    p[2] = (uint8_t) (val >> 16);
    p[3] = (uint8_t) (val >> 24);
    return (p + 4);
}

/* Encode a UTF-8 string as a PTP string (at most 254 characters) */
static uint8_t *
put_str(uint8_t *p, const char *str)
{
    const uint8_t *s = (const uint8_t *) str;
    uint8_t *countp = p++;
    uint     count = 0;

    if (*s == '\0') {
        *countp = 0;    // Empty string: no characters at all
        return (p);
    }
    while ((*s != '\0') && (count < 252)) {
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
        }
        while ((extra > 0) && ((*s & 0xc0) == 0x80)) {
            ch = (ch << 6) | (*s++ & 0x3f);
            extra--;
        }
        if (ch >= 0x10000) {
            ch -= 0x10000;
            p = put_le16(p, 0xd800 | (ch >> 10));
            p = put_le16(p, 0xdc00 | (ch & 0x3ff));
            count += 2;
        } else {
            p = put_le16(p, ch);
            count++;
        }
    }
    p = put_le16(p, 0);   // NUL
    *countp = count + 1;
    return (p);
}

/* Split a path into its parent directory (in place) and last element */
static const char *
path_split(char *path)
{
    char *slash = strrchr(path, '/');
    if (slash == NULL) {
        /* Parent is the root: return the name, path becomes "" */
        memmove(path + 1, path, strlen(path) + 1);
        path[0] = '\0';
        return (path + 1);
    }
    *slash = '\0';
    return (slash + 1);
}

static FRESULT
usb_to_fr(int rc)
{
    switch (rc) {
        case USB_MTP_OK:
            return (FR_OK);
        case USB_MTP_NODEV:
        case USB_MTP_REMOVED:
        case USB_MTP_NOT_READY:
            return (FR_NOT_READY);
        default:
            return (FR_DISK_ERR);
    }
}

static FRESULT
resp_to_fr(uint resp)
{
    switch (resp) {
        case PTP_RC_OK:
            return (FR_OK);
        case PTP_RC_InvalidObjectHandle:
            return (FR_NO_FILE);
        case PTP_RC_ObjectWriteProtected:
        case PTP_RC_StoreReadOnly:
            return (FR_WRITE_PROTECTED);
        case PTP_RC_StoreFull:
        case PTP_RC_AccessDenied:
        case PTP_RC_OperationNotSupported:
            return (FR_DENIED);
        default:
            return (FR_DISK_ERR);
    }
}

/*
 * mx() - run a transaction on the volume's device. Returns FR_OK if the
 *        device responded OK.
 */
static FRESULT
mx(mvol_t *v, uint16_t code, uint nparam, const uint32_t *params, uint dir,
   void *buf, uint32_t buflen, usb_mtp_result_t *res)
{
    usb_mtp_result_t lres;
    int rc;

    if (res == NULL)
        res = &lres;
    rc = usb_mtp_xact(v->dev, code, nparam, params, dir, buf, buflen, res);
    if (rc != USB_MTP_OK) {
        mprintf("MTP op %04x: %s\n", code, usb_mtp_strerror(rc));
        return (usb_to_fr(rc));
    }
    if (res->resp != PTP_RC_OK) {
        mprintf("MTP op %04x: response %04x\n", code, res->resp);
        return (resp_to_fr(res->resp));
    }
    return (FR_OK);
}

static mvol_t *
mvol_get(uint mvol)
{
    if ((mvol >= FS_MTP_VOLS) || !mvols[mvol].used)
        return (NULL);
    return (&mvols[mvol]);
}

/* ------------------------------------------------------------------ */
/* Caches                                                              */

static void
dcache_drop(dcache_t *dc)
{
    if (dc->used) {
        dcache_ents -= dc->max;
        free(dc->ent);
    }
    memset(dc, 0, sizeof (*dc));
}

static void
cache_flush(void)
{
    uint i;
    for (i = 0; i < MTP_DCACHES; i++)
        dcache_drop(&dcache[i]);
    memset(pcache, 0, sizeof (pcache));
}

/* Drop cached information about a directory's entries */
static void
cache_dir_changed(uint mvol, uint32_t parent)
{
    uint i;
    for (i = 0; i < MTP_DCACHES; i++)
        if (dcache[i].used && (dcache[i].mvol == mvol) &&
            (dcache[i].parent == parent))
            dcache_drop(&dcache[i]);
    memset(pcache, 0, sizeof (pcache));
}

static dcache_t *
dcache_find(uint mvol, uint32_t parent)
{
    uint32_t change = usb_mtp_changes();
    uint i;

    for (i = 0; i < MTP_DCACHES; i++) {
        dcache_t *dc = &dcache[i];
        if (!dc->used || (dc->mvol != mvol) || (dc->parent != parent))
            continue;
        if (dc->change != change) {
            dcache_drop(dc);
            return (NULL);
        }
        dc->lru = ++cache_lru;
        return (dc);
    }
    return (NULL);
}

/* Allocate an empty cache for a directory, evicting the oldest */
static dcache_t *
dcache_new(uint mvol, uint32_t parent)
{
    dcache_t *dc = &dcache[0];
    uint i;

    for (i = 0; i < MTP_DCACHES; i++) {
        if (!dcache[i].used) {
            dc = &dcache[i];
            break;
        }
        if (dcache[i].lru < dc->lru)
            dc = &dcache[i];
    }
    dcache_drop(dc);
    dc->used   = 1;
    dc->mvol   = mvol;
    dc->parent = parent;
    dc->change = usb_mtp_changes();
    dc->lru    = ++cache_lru;
    return (dc);
}

/* Add an entry; returns 0 if there's no room (the cache is incomplete) */
static int
dcache_add(dcache_t *dc, uint32_t handle, uint32_t hash)
{
    if (dc->n >= dc->max) {
        uint32_t nmax = (dc->max == 0) ? 64 : dc->max * 2;
        dent_t  *nent;
        if (nmax > MTP_DIR_MAX)
            nmax = MTP_DIR_MAX;
        if ((nmax <= dc->max) ||
            (dcache_ents - dc->max + nmax > MTP_DCACHE_ENTS))
            return (0);
        nent = realloc(dc->ent, nmax * sizeof (dent_t));
        if (nent == NULL)
            return (0);
        dcache_ents += nmax - dc->max;
        dc->ent = nent;
        dc->max = nmax;
    }
    dc->ent[dc->n].handle = handle;
    dc->ent[dc->n].hash   = hash;
    dc->n++;
    return (1);
}

static pcache_t *
pcache_find(uint mvol, uint32_t hash)
{
    uint32_t change = usb_mtp_changes();
    uint i;

    for (i = 0; i < MTP_PCACHES; i++) {
        pcache_t *pc = &pcache[i];
        if (pc->used && (pc->mvol == mvol) && (pc->hash == hash)) {
            if (pc->change != change) {
                pc->used = 0;
                return (NULL);
            }
            pc->lru = ++cache_lru;
            return (pc);
        }
    }
    return (NULL);
}

static void
pcache_add(uint mvol, uint32_t hash, uint32_t handle)
{
    pcache_t *pc = &pcache[0];
    uint i;

    for (i = 0; i < MTP_PCACHES; i++) {
        if (!pcache[i].used) {
            pc = &pcache[i];
            break;
        }
        if (pcache[i].lru < pc->lru)
            pc = &pcache[i];
    }
    pc->used   = 1;
    pc->mvol   = mvol;
    pc->hash   = hash;
    pc->handle = handle;
    pc->change = usb_mtp_changes();
    pc->lru    = ++cache_lru;
}

/* ------------------------------------------------------------------ */
/* Volumes                                                             */

/*
 * fs_mtp_refresh() - re-scan MTP devices if they changed. Returns a
 *                    value which changes whenever the set of MTP volumes
 *                    may have changed.
 */
uint32_t
fs_mtp_refresh(void)
{
    uint32_t sig = usb_mtp_generation();
    uint     ndev = usb_mtp_count();
    uint     dev;
    uint     idx;
    uint     nvol = 0;

    sig = crc32(sig, &ndev, sizeof (ndev));
    for (dev = 0; dev < ndev; dev++) {
        usb_mtp_info_t info;
        usb_mtp_storage_t st;
        if (usb_mtp_info(dev, &info) != USB_MTP_OK)
            continue;
        sig = crc32(sig, &info.ready, sizeof (info.ready));
        for (idx = 0; idx < info.nstorage; idx++)
            if (usb_mtp_storage(dev, idx, &st) == USB_MTP_OK)
                sig = crc32(sig, &st.id, sizeof (st.id));
    }
    if (mvol_valid && (sig == mvol_sig))
        return (mvol_sig);

    /* Rebuild the volume list */
    cache_flush();
    memset(mvols, 0, sizeof (mvols));
    for (dev = 0; dev < ndev; dev++) {
        usb_mtp_info_t info;
        if ((usb_mtp_info(dev, &info) != USB_MTP_OK) || !info.ready)
            continue;
        for (idx = 0; (idx < info.nstorage) && (nvol < FS_MTP_VOLS); idx++) {
            usb_mtp_storage_t st;
            mvol_t *v = &mvols[nvol];
            char   *name;
            size_t  len;

            if (usb_mtp_storage(dev, idx, &st) != USB_MTP_OK)
                continue;
            v->used     = 1;
            v->dev      = dev;
            v->ops      = info.ops;
            v->storage  = st.id;
            v->readonly = (st.access != PTP_AC_ReadWrite) ||
                          !(info.ops & MTP_OPS_EDIT);
            v->serial   = crc32(st.id, info.path, strlen(info.path));
            /* Description: some devices put a serial number in the label */
            name = (st.desc[0] != '\0') ? st.desc : st.label;
            strncpy(v->name, name, sizeof (v->name) - 1);
            len = strlen(v->name);
            while ((len > 0) && (v->name[len - 1] == ' '))
                v->name[--len] = '\0';
            if (len == 0)
                sprintf(v->name, "MTP%u", nvol);
            mprintf("MTP vol %u: dev %u storage %08lx %s ops %04x\n", nvol,
                    dev, st.id, v->name, v->ops);
            nvol++;
        }
    }
    mvol_sig   = sig;
    mvol_valid = 1;
    return (mvol_sig);
}

/* Returns 1 and fills *vol if MTP volume mvol exists */
uint
fs_mtp_volume(uint mvol, fs_mtp_vol_t *vol)
{
    mvol_t *v = mvol_get(mvol);
    if (v == NULL)
        return (0);
    vol->serial   = v->serial;
    vol->readonly = v->readonly;
    strcpy(vol->name, v->name);
    return (1);
}

/* ------------------------------------------------------------------ */
/* Objects                                                             */

/* Convert an MTP date "YYYYMMDDThhmmss..." to FAT date and time */
static void
date_to_fat(const char *date, WORD *fdate, WORD *ftime)
{
    uint val[6];
    uint pos[6] = { 0, 4, 6, 9, 11, 13 };
    uint len[6] = { 4, 2, 2, 2, 2, 2 };
    uint i;
    uint j;

    *fdate = (0 << 9) | (1 << 5) | 1;   // 1980-01-01
    *ftime = 0;
    if (strlen(date) < 15)
        return;
    for (i = 0; i < 6; i++) {
        val[i] = 0;
        for (j = 0; j < len[i]; j++) {
            char ch = date[pos[i] + j];
            if ((ch < '0') || (ch > '9'))
                return;
            val[i] = val[i] * 10 + (ch - '0');
        }
    }
    if ((val[0] < 1980) || (val[0] > 2107) || (val[1] < 1) ||
        (val[1] > 12) || (val[2] < 1) || (val[2] > 31))
        return;
    *fdate = ((val[0] - 1980) << 9) | (val[1] << 5) | val[2];
    *ftime = (val[3] << 11) | (val[4] << 5) | (val[5] / 2);
}

/* Read and decode an object's ObjectInfo */
static FRESULT
get_info(mvol_t *v, uint32_t handle, oinfo_t *oi)
{
    usb_mtp_result_t res;
    uint32_t off;
    uint32_t len;
    char     capture[20];
    FRESULT  fr;

    fr = mx(v, PTP_OC_GetObjectInfo, 1, &handle, USB_MTP_DIR_IN, oi_buf,
            sizeof (oi_buf), &res);
    if (fr != FR_OK)
        return (fr);
    len = res.len;
    if (len < 53) {
        mprintf("MTP ObjectInfo %08lx: short (%lu of %lu)\n", handle, len,
                res.total);
        return (FR_DISK_ERR);
    }
    memset(oi, 0, sizeof (*oi));
    oi->storage = get_le32(oi_buf + PTP_oi_StorageID);
    oi->format  = oi_buf[PTP_oi_ObjectFormat] |
                  (oi_buf[PTP_oi_ObjectFormat + 1] << 8);
    oi->prot    = oi_buf[PTP_oi_ProtectionStatus] |
                  (oi_buf[PTP_oi_ProtectionStatus + 1] << 8);
    oi->size    = get_le32(oi_buf + PTP_oi_ObjectCompressedSize);
    oi->parent  = get_le32(oi_buf + PTP_oi_ParentObject);
    if (oi->parent == 0)
        oi->parent = MTP_ROOT;
    off = PTP_oi_filenamelen;
    (void) MTP_GetString(oi_buf, len, &off, oi->name, sizeof (oi->name));
    (void) MTP_GetString(oi_buf, len, &off, capture, sizeof (capture));
    (void) MTP_GetString(oi_buf, len, &off, oi->date, sizeof (oi->date));
    if (oi->date[0] == '\0')
        strcpy(oi->date, capture);

    if ((oi->size == 0xffffffff) && (v->ops & MTP_OPS_GETPROP)) {
        /* 4 GB or larger: get the 64-bit ObjectSize property */
        uint32_t params[2] = { handle, PTP_OPC_ObjectSize };
        uint8_t  sbuf[8];
        if ((mx(v, PTP_OC_GetObjectPropValue, 2, params, USB_MTP_DIR_IN,
                sbuf, sizeof (sbuf), &res) == FR_OK) && (res.len == 8))
            oi->size = get_le32(sbuf) | ((uint64_t) get_le32(sbuf + 4) << 32);
    }
    return (FR_OK);
}

static void
oinfo_to_filinfo(mvol_t *v, const oinfo_t *oi, FILINFO *fno)
{
    memset(fno, 0, sizeof (*fno));
    if (oi->format == PTP_FMT_ASSOCIATION) {
        fno->fattrib = AM_DIR;
    } else {
        fno->fattrib = AM_ARC;
        fno->fsize = oi->size;
    }
    if ((oi->prot != 0) || v->readonly)
        fno->fattrib |= AM_RDO;
    date_to_fat(oi->date, &fno->fdate, &fno->ftime);
    strncpy(fno->fname, oi->name, sizeof (fno->fname) - 1);
}

/* ------------------------------------------------------------------ */
/* Directory scanning                                                  */

/* Streamed array of 32-bit values (GetObjectHandles) */
typedef struct {
    uint32_t *arr;
    uint32_t  n;
    uint32_t  max;
    uint32_t  count;        // Count from the dataset
    uint8_t   got_count;
    uint8_t   cn;
    uint8_t   carry[4];
} u32sink_t;

static void
u32_sink(void *arg, const uint8_t *data, uint32_t len)
{
    u32sink_t *s = arg;

    while (len > 0) {
        uint32_t val;
        while ((s->cn < 4) && (len > 0)) {
            s->carry[s->cn++] = *data++;
            len--;
        }
        if (s->cn < 4)
            break;
        s->cn = 0;
        val = get_le32(s->carry);
        if (!s->got_count) {
            s->got_count = 1;
            s->count = val;
            s->max = (val > MTP_DIR_MAX) ? MTP_DIR_MAX : val;
            if (s->max > 0) {
                s->arr = malloc(s->max * sizeof (uint32_t));
                if (s->arr == NULL)
                    s->max = 0;
            }
            continue;
        }
        if (s->n < s->max)
            s->arr[s->n++] = val;
    }
}

/* Get the handles of a directory's entries (malloc()ed array) */
static FRESULT
get_handles(mvol_t *v, uint32_t parent, uint32_t **handles, uint32_t *count,
            uint8_t *complete)
{
    usb_mtp_result_t res;
    u32sink_t sink;
    uint32_t  params[3] = { v->storage, 0, parent };
    int       rc;

    memset(&sink, 0, sizeof (sink));
    rc = usb_mtp_xact_sink(v->dev, PTP_OC_GetObjectHandles, 3, params,
                           stage_buf, sizeof (stage_buf), u32_sink, &sink,
                           &res);
    if ((rc != USB_MTP_OK) || (res.resp != PTP_RC_OK)) {
        mprintf("MTP handles %08lx: rc=%d resp=%04x\n", parent, rc, res.resp);
        free(sink.arr);
        if (rc != USB_MTP_OK)
            return (usb_to_fr(rc));
        return (resp_to_fr(res.resp));
    }
    if (sink.count > sink.n)
        mprintf("MTP dir %08lx: %lu of %lu entries\n", parent, sink.n,
                sink.count);
    *handles = sink.arr;
    *count = sink.n;
    if (complete != NULL)
        *complete = (sink.n == sink.count);
    return (FR_OK);
}

/* Streamed GetObjectPropList (ObjectFileName) parser */
typedef struct {
    dcache_t *dc;
    uint32_t  nelem;        // Elements remaining (from the dataset)
    uint8_t   got_count;
    uint8_t   bad;
    uint8_t   full;         // Cache had no room for an entry
    uint16_t  cn;
    uint8_t   carry[8 + 1 + 2 * 255 + 4];
} plsink_t;

static void
pl_parse(plsink_t *s)
{
    uint32_t pos = 0;

    if (!s->got_count) {
        if (s->cn < 4)
            return;
        s->nelem = get_le32(s->carry);
        s->got_count = 1;
        pos = 4;
    }
    while (!s->bad && (s->nelem > 0)) {
        uint32_t avail = s->cn - pos;
        uint8_t *el = s->carry + pos;
        uint32_t elen;
        uint32_t off;
        char     name[FF_LFN_BUF + 1];

        if (avail < 9)
            break;
        /* Handle (4), property code (2), data type (2), string */
        if ((el[4] | (el[5] << 8)) != PTP_OPC_ObjectFileName ||
            (el[6] | (el[7] << 8)) != PTP_DTC_STR) {
            s->bad = 1;
            break;
        }
        elen = 9 + 2 * el[8];
        if (avail < elen)
            break;
        off = 8;
        (void) MTP_GetString(el, elen, &off, name, sizeof (name));
        if (!dcache_add(s->dc, get_le32(el), name_hash(0, name, strlen(name))))
            s->full = 1;
        pos += elen;
        s->nelem--;
    }
    if (pos > 0) {
        memmove(s->carry, s->carry + pos, s->cn - pos);
        s->cn -= pos;
    }
}

static void
pl_sink(void *arg, const uint8_t *data, uint32_t len)
{
    plsink_t *s = arg;

    while ((len > 0) && !s->bad) {
        uint32_t n = sizeof (s->carry) - s->cn;
        if (n > len)
            n = len;
        memcpy(s->carry + s->cn, data, n);
        s->cn += n;
        data  += n;
        len   -= n;
        pl_parse(s);
        if (s->cn == sizeof (s->carry))
            s->bad = 1;   // An element larger than possible
    }
}

/* Fill a directory cache with the hashed names of its entries */
static FRESULT
dcache_fill(uint mvol, mvol_t *v, uint32_t parent, dcache_t **dcp)
{
    dcache_t *dc = dcache_new(mvol, parent);
    uint32_t *handles = NULL;
    uint32_t  count = 0;
    uint32_t  i;
    uint8_t   complete = 0;
    FRESULT   fr;

    /*
     * GetObjectPropList for the root may list the root of every storage
     * unit, so it's only used below the root.
     */
    if ((parent != MTP_ROOT) && (v->ops & MTP_OPS_PROPLIST) &&
        !v->no_proplist) {
        usb_mtp_result_t res;
        plsink_t *s = malloc(sizeof (*s));
        uint32_t params[5] = { parent, 0, PTP_OPC_ObjectFileName, 0, 1 };
        int rc;

        if (s == NULL) {
            dcache_drop(dc);
            return (FR_NOT_ENOUGH_CORE);
        }
        memset(s, 0, sizeof (*s));
        s->dc = dc;
        rc = usb_mtp_xact_sink(v->dev, PTP_OC_GetObjPropList, 5, params,
                               stage_buf, sizeof (stage_buf), pl_sink, s,
                               &res);
        if ((rc == USB_MTP_OK) && (res.resp == PTP_RC_OK) && !s->bad &&
            s->got_count && (s->nelem == 0)) {
            dc->complete = !s->full;
            free(s);
            *dcp = dc;
            return (FR_OK);
        }
        mprintf("MTP proplist %08lx failed: rc=%d resp=%04x bad=%u\n",
                parent, rc, res.resp, s->bad);
        free(s);
        if (rc != USB_MTP_OK) {
            dcache_drop(dc);
            return (usb_to_fr(rc));
        }
        v->no_proplist = 1;   // Use ObjectInfo from now on
        dc->n = 0;
    }

    /* Get the name of each entry */
    fr = get_handles(v, parent, &handles, &count, &complete);
    if (fr != FR_OK) {
        dcache_drop(dc);
        return (fr);
    }
    dc->complete = complete;
    for (i = 0; i < count; i++) {
        oinfo_t *oi = malloc(sizeof (*oi));
        if (oi == NULL) {
            dc->complete = 0;
            break;
        }
        fr = get_info(v, handles[i], oi);
        if (fr == FR_OK) {
            if (!dcache_add(dc, handles[i],
                            name_hash(0, oi->name, strlen(oi->name))))
                dc->complete = 0;
        } else if (fr != FR_NO_FILE) {
            free(oi);
            free(handles);
            dcache_drop(dc);
            return (fr);
        }
        free(oi);
    }
    free(handles);
    *dcp = dc;
    return (FR_OK);
}

/*
 * lookup_child() - find the entry of a directory with the specified name.
 *                  On success, *handle and *oi describe it.
 */
static FRESULT
lookup_child(uint mvol, mvol_t *v, uint32_t parent, const char *name,
             size_t nlen, uint32_t *handle, oinfo_t *oi)
{
    dcache_t *dc;
    uint32_t  hash = name_hash(0, name, nlen);
    uint32_t  i;
    uint      tries;
    FRESULT   fr;

    for (tries = 0; tries < 2; tries++) {
        dc = dcache_find(mvol, parent);
        if (dc == NULL) {
            fr = dcache_fill(mvol, v, parent, &dc);
            if (fr != FR_OK)
                return (fr);
        }
        for (i = 0; i < dc->n; i++) {
            if (dc->ent[i].hash != hash)
                continue;
            fr = get_info(v, dc->ent[i].handle, oi);
            if ((fr == FR_OK) && (oi->parent == parent)) {
                if (name_eq(oi->name, name, nlen)) {
                    *handle = dc->ent[i].handle;
                    return (FR_OK);
                }
                if (name_hash(0, oi->name, strlen(oi->name)) == hash)
                    continue;   // A different name with the same hash
            }
            if ((fr != FR_OK) && (fr != FR_NO_FILE))
                return (fr);
            /* Stale entry: refill the cache and try again */
            dcache_drop(dc);
            dc = NULL;
            break;
        }
        if (dc != NULL) {
            if (dc->complete)
                return (FR_NO_FILE);
            break;
        }
    }
    if (tries >= 2)
        return (FR_NO_FILE);

    /* Directory too large for the cache: search all entries */
    {
        uint32_t *handles = NULL;
        uint32_t  count = 0;
        fr = get_handles(v, parent, &handles, &count, NULL);
        if (fr != FR_OK)
            return (fr);
        for (i = 0; i < count; i++) {
            if ((get_info(v, handles[i], oi) == FR_OK) &&
                name_eq(oi->name, name, nlen)) {
                *handle = handles[i];
                free(handles);
                return (FR_OK);
            }
        }
        free(handles);
    }
    return (FR_NO_FILE);
}

/*
 * resolve() - find the object at a path. The root has handle MTP_ROOT,
 *             and its *oi is a directory with no name.
 */
static FRESULT
resolve(uint mvol, mvol_t *v, const char *path, uint32_t *handle,
        oinfo_t *oi)
{
    uint32_t    phash;
    pcache_t   *pc;
    const char *slash;
    const char *name;
    uint32_t    parent;
    FRESULT     fr;

    if (path[0] == '\0') {
        memset(oi, 0, sizeof (*oi));
        oi->format  = PTP_FMT_ASSOCIATION;
        oi->storage = v->storage;
        oi->parent  = MTP_ROOT;
        *handle = MTP_ROOT;
        return (FR_OK);
    }
    phash = name_hash(0, path, strlen(path));
    slash = strrchr(path, '/');
    name  = (slash == NULL) ? path : slash + 1;

    pc = pcache_find(mvol, phash);
    if (pc != NULL) {
        fr = get_info(v, pc->handle, oi);
        if ((fr == FR_OK) && name_eq(oi->name, name, strlen(name))) {
            *handle = pc->handle;
            return (FR_OK);
        }
        pc->used = 0;
        if ((fr != FR_OK) && (fr != FR_NO_FILE))
            return (fr);
    }

    /* Resolve the parent directory, then look up the name in it */
    if (slash == NULL) {
        parent = MTP_ROOT;
    } else {
        char *ppath = malloc(slash - path + 1);
        if (ppath == NULL)
            return (FR_NOT_ENOUGH_CORE);
        memcpy(ppath, path, slash - path);
        ppath[slash - path] = '\0';
        fr = resolve(mvol, v, ppath, &parent, oi);
        free(ppath);
        if (fr == FR_NO_FILE)
            return (FR_NO_PATH);
        if (fr != FR_OK)
            return (fr);
        if (oi->format != PTP_FMT_ASSOCIATION)
            return (FR_NO_PATH);
    }
    fr = lookup_child(mvol, v, parent, name, strlen(name), handle, oi);
    if (fr == FR_OK)
        pcache_add(mvol, phash, *handle);
    return (fr);
}

/* Resolve a path to a directory */
static FRESULT
resolve_dir(uint mvol, mvol_t *v, const char *path, uint32_t *handle)
{
    oinfo_t *oi = malloc(sizeof (*oi));
    FRESULT  fr;

    if (oi == NULL)
        return (FR_NOT_ENOUGH_CORE);
    fr = resolve(mvol, v, path, handle, oi);
    if (fr == FR_NO_FILE)
        fr = FR_NO_PATH;
    else if ((fr == FR_OK) && (oi->format != PTP_FMT_ASSOCIATION))
        fr = FR_NO_PATH;
    free(oi);
    return (fr);
}

/* ------------------------------------------------------------------ */
/* Public interface                                                    */

FRESULT
fs_mtp_stat(uint mvol, const char *path, FILINFO *fno)
{
    mvol_t  *v = mvol_get(mvol);
    oinfo_t *oi;
    uint32_t handle;
    FRESULT  fr;

    if (v == NULL)
        return (FR_NOT_READY);
    oi = malloc(sizeof (*oi));
    if (oi == NULL)
        return (FR_NOT_ENOUGH_CORE);
    fr = resolve(mvol, v, path, &handle, oi);
    if (fr == FR_OK)
        oinfo_to_filinfo(v, oi, fno);
    free(oi);
    return (fr);
}

FRESULT
fs_mtp_opendir(mtpdir_t **dp, uint mvol, const char *path)
{
    mvol_t   *v = mvol_get(mvol);
    mtpdir_t *d;
    uint32_t  handle;
    FRESULT   fr;

    *dp = NULL;
    if (v == NULL)
        return (FR_NOT_READY);
    fr = resolve_dir(mvol, v, path, &handle);
    if (fr != FR_OK)
        return (fr);
    d = malloc(sizeof (*d));
    if (d == NULL)
        return (FR_NOT_ENOUGH_CORE);
    memset(d, 0, sizeof (*d));
    d->mvol = mvol;
    fr = get_handles(v, handle, &d->handles, &d->n, NULL);
    if (fr != FR_OK) {
        free(d);
        return (fr);
    }
    *dp = d;
    return (FR_OK);
}

/* Read the next entry; fno->fname is "" at the end of the directory */
FRESULT
fs_mtp_readdir(mtpdir_t *d, FILINFO *fno)
{
    mvol_t  *v = mvol_get(d->mvol);
    oinfo_t *oi;
    FRESULT  fr = FR_OK;

    memset(fno, 0, sizeof (*fno));
    if (v == NULL)
        return (FR_NOT_READY);
    oi = malloc(sizeof (*oi));
    if (oi == NULL)
        return (FR_NOT_ENOUGH_CORE);
    while (d->pos < d->n) {
        fr = get_info(v, d->handles[d->pos++], oi);
        if (fr == FR_NO_FILE) {
            fr = FR_OK;     // Deleted since the directory was opened
            continue;
        }
        if (fr == FR_OK)
            oinfo_to_filinfo(v, oi, fno);
        break;
    }
    free(oi);
    return (fr);
}

void
fs_mtp_rewinddir(mtpdir_t *d)
{
    d->pos = 0;
}

void
fs_mtp_closedir(mtpdir_t *d)
{
    if (d != NULL) {
        free(d->handles);
        free(d);
    }
}

/* Create an object (file or directory) in a directory */
static FRESULT
create_object(uint mvol, mvol_t *v, uint32_t parent, const char *name,
              uint dir, uint32_t *handle)
{
    usb_mtp_result_t res;
    uint32_t params[2] = { v->storage, parent };
    uint8_t *buf;
    uint8_t *p;
    FRESULT  fr;

    if (!(v->ops & MTP_OPS_SENDINFO))
        return (FR_WRITE_PROTECTED);
    buf = malloc(64 + 4 * 256);
    if (buf == NULL)
        return (FR_NOT_ENOUGH_CORE);
    p = put_le32(buf, v->storage);                        // StorageID
    p = put_le16(p, dir ? PTP_FMT_ASSOCIATION : PTP_FMT_UNDEFINED);
    p = put_le16(p, 0);                                   // Protection
    p = put_le32(p, 0);                                   // Size
    p = put_le16(p, 0);                                   // ThumbFormat
    p = put_le32(p, 0);                                   // ThumbSize
    p = put_le32(p, 0);                                   // ThumbWidth
    p = put_le32(p, 0);                                   // ThumbHeight
    p = put_le32(p, 0);                                   // ImageWidth
    p = put_le32(p, 0);                                   // ImageHeight
    p = put_le32(p, 0);                                   // ImageBitDepth
    p = put_le32(p, parent);                              // ParentObject
    p = put_le16(p, dir ? PTP_AT_GenericFolder : 0);      // AssociationType
    p = put_le32(p, 0);                                   // AssociationDesc
    p = put_le32(p, 0);                                   // SequenceNumber
    p = put_str(p, name);                                 // Filename
    p = put_str(p, "");                                   // CaptureDate
    p = put_str(p, "");                                   // ModificationDate
    p = put_str(p, "");                                   // Keywords

    fr = mx(v, PTP_OC_SendObjectInfo, 2, params, USB_MTP_DIR_OUT, buf,
            p - buf, &res);
    free(buf);
    if (fr != FR_OK)
        return (fr);
    if (res.nresp < 3)
        return (FR_DISK_ERR);
    *handle = res.rparam[2];
    if (!dir) {
        /* The object is created by SendObject (here with no data) */
        uint8_t dummy = 0;
        fr = mx(v, PTP_OC_SendObject, 0, NULL, USB_MTP_DIR_OUT, &dummy, 0,
                NULL);
    }
    cache_dir_changed(mvol, parent);
    return (fr);
}

FRESULT
fs_mtp_open(mtpfile_t **fpp, uint mvol, const char *path, BYTE mode)
{
    mvol_t    *v = mvol_get(mvol);
    mtpfile_t *fp;
    oinfo_t   *oi;
    uint32_t   handle;
    FRESULT    fr;

    *fpp = NULL;
    if (v == NULL)
        return (FR_NOT_READY);
    if ((mode & (FA_WRITE | FA_CREATE_NEW | FA_CREATE_ALWAYS |
                 FA_OPEN_ALWAYS)) && v->readonly)
        return (FR_WRITE_PROTECTED);
    oi = malloc(sizeof (*oi));
    if (oi == NULL)
        return (FR_NOT_ENOUGH_CORE);
    fr = resolve(mvol, v, path, &handle, oi);
    if (fr == FR_OK) {
        if (oi->format == PTP_FMT_ASSOCIATION)
            fr = FR_DENIED;           // A directory
        else if (mode & FA_CREATE_NEW)
            fr = FR_EXIST;
        else if ((oi->prot != 0) && (mode & FA_WRITE))
            fr = FR_DENIED;
    } else if ((fr == FR_NO_FILE) &&
               (mode & (FA_CREATE_NEW | FA_CREATE_ALWAYS | FA_OPEN_ALWAYS))) {
        char       *ppath = malloc(strlen(path) + 2);
        const char *name;
        uint32_t    parent;
        if (ppath == NULL) {
            fr = FR_NOT_ENOUGH_CORE;
        } else {
            strcpy(ppath, path);
            name = path_split(ppath);
            fr = resolve_dir(mvol, v, ppath, &parent);
            if (fr == FR_OK)
                fr = create_object(mvol, v, parent, name, 0, &handle);
            free(ppath);
            memset(oi, 0, sizeof (*oi));
        }
    }
    if (fr != FR_OK) {
        free(oi);
        return (fr);
    }

    fp = malloc(sizeof (*fp));
    if (fp == NULL) {
        free(oi);
        return (FR_NOT_ENOUGH_CORE);
    }
    memset(fp, 0, sizeof (*fp));
    fp->mvol   = mvol;
    fp->mode   = mode & (FA_READ | FA_WRITE);
    fp->handle = handle;
    fp->size   = oi->size;
    free(oi);
    *fpp = fp;

    if ((mode & FA_CREATE_ALWAYS) && (fp->size != 0)) {
        fr = fs_mtp_truncate(fp);
        if (fr != FR_OK) {
            (void) fs_mtp_close(fp);
            *fpp = NULL;
        }
    }
    return (fr);
}

/* Send buffered write data */
static FRESULT
wbuf_flush(mtpfile_t *fp)
{
    mvol_t  *v = mvol_get(fp->mvol);
    uint32_t params[4];
    FRESULT  fr;

    if (fp->wbuf_len == 0)
        return (FR_OK);
    if (v == NULL)
        return (FR_NOT_READY);
    if (!fp->editing) {
        fr = mx(v, MTP_OC_ANDROID_BeginEditObject, 1, &fp->handle,
                USB_MTP_DIR_NONE, NULL, 0, NULL);
        if (fr != FR_OK)
            return (fr);
        fp->editing = 1;
    }
    params[0] = fp->handle;
    params[1] = (uint32_t) fp->wbuf_off;
    params[2] = (uint32_t) ((uint64_t) fp->wbuf_off >> 32);
    params[3] = fp->wbuf_len;
    fr = mx(v, MTP_OC_ANDROID_SendPartialObject, 4, params, USB_MTP_DIR_OUT,
            fp->wbuf, fp->wbuf_len, NULL);
    if (fr == FR_OK)
        fp->wbuf_len = 0;
    return (fr);
}

/* Finish an edit, so the device makes the changes visible */
static FRESULT
edit_end(mtpfile_t *fp)
{
    mvol_t *v = mvol_get(fp->mvol);
    FRESULT fr = wbuf_flush(fp);

    if ((fr == FR_OK) && fp->editing) {
        if (v == NULL)
            return (FR_NOT_READY);
        fr = mx(v, MTP_OC_ANDROID_EndEditObject, 1, &fp->handle,
                USB_MTP_DIR_NONE, NULL, 0, NULL);
        fp->editing = 0;
    }
    return (fr);
}

FRESULT
fs_mtp_read(mtpfile_t *fp, void *buf, UINT len, UINT *got)
{
    mvol_t  *v = mvol_get(fp->mvol);
    uint8_t *dst = buf;
    FRESULT  fr;

    *got = 0;
    if (v == NULL)
        return (FR_NOT_READY);
    if (!(fp->mode & FA_READ))
        return (FR_DENIED);
    fr = edit_end(fp);      // Read back what was written
    if (fr != FR_OK)
        return (fr);

    while ((len > 0) && (fp->pos < fp->size)) {
        uint32_t n;
        if ((fp->rbuf != NULL) && (fp->pos >= fp->rbuf_off) &&
            (fp->pos < fp->rbuf_off + fp->rbuf_len)) {
            uint32_t boff = (uint32_t) (fp->pos - fp->rbuf_off);
            n = fp->rbuf_len - boff;
            if (n > len)
                n = len;
            memcpy(dst, fp->rbuf + boff, n);
            dst += n;
            len -= n;
            *got += n;
            fp->pos += n;
            continue;
        }

        /* Fetch more */
        if (fp->rbuf == NULL) {
            fp->rbuf = malloc(MTP_RBUF_SIZE);
            if (fp->rbuf == NULL)
                return (FR_NOT_ENOUGH_CORE);
        }
        fp->rbuf_off = fp->pos;
        fp->rbuf_len = 0;
        n = MTP_RBUF_SIZE;
        if (fp->size - fp->pos < n)
            n = (uint32_t) (fp->size - fp->pos);
        {
            usb_mtp_result_t res;
            uint32_t params[4];
            if ((fp->pos >> 32) || !(v->ops & MTP_OPS_PARTIAL)) {
                if (!(v->ops & MTP_OPS_PARTIAL64))
                    return (FR_DENIED);
                params[0] = fp->handle;
                params[1] = (uint32_t) fp->pos;
                params[2] = (uint32_t) ((uint64_t) fp->pos >> 32);
                params[3] = n;
                fr = mx(v, MTP_OC_ANDROID_GetPartialObject64, 4, params,
                        USB_MTP_DIR_IN, fp->rbuf, n, &res);
            } else {
                params[0] = fp->handle;
                params[1] = (uint32_t) fp->pos;
                params[2] = n;
                fr = mx(v, PTP_OC_GetPartialObject, 3, params,
                        USB_MTP_DIR_IN, fp->rbuf, n, &res);
            }
            if (fr != FR_OK)
                return (fr);
            fp->rbuf_len = res.len;
            if (res.len == 0)
                break;      // File shorter than its reported size
        }
    }
    return (FR_OK);
}

FRESULT
fs_mtp_write(mtpfile_t *fp, const void *buf, UINT len, UINT *wrote)
{
    const uint8_t *src = buf;
    FRESULT fr;

    *wrote = 0;
    if (!(fp->mode & FA_WRITE))
        return (FR_DENIED);
    if (fp->wbuf == NULL) {
        fp->wbuf = malloc(MTP_WBUF_SIZE);
        if (fp->wbuf == NULL)
            return (FR_NOT_ENOUGH_CORE);
    }
    fp->rbuf_len = 0;   // Read-ahead data may now be stale
    while (len > 0) {
        uint32_t n;
        if ((fp->wbuf_len > 0) &&
            ((fp->wbuf_off + fp->wbuf_len != fp->pos) ||
             (fp->wbuf_len == MTP_WBUF_SIZE))) {
            fr = wbuf_flush(fp);
            if (fr != FR_OK)
                return (fr);
        }
        if (fp->wbuf_len == 0)
            fp->wbuf_off = fp->pos;
        n = MTP_WBUF_SIZE - fp->wbuf_len;
        if (n > len)
            n = len;
        memcpy(fp->wbuf + fp->wbuf_len, src, n);
        fp->wbuf_len += n;
        fp->pos += n;
        src += n;
        len -= n;
        *wrote += n;
        if (fp->pos > fp->size)
            fp->size = fp->pos;
    }
    return (FR_OK);
}

FRESULT
fs_mtp_lseek(mtpfile_t *fp, FSIZE_t pos)
{
    fp->pos = pos;
    return (FR_OK);
}

FSIZE_t
fs_mtp_tell(mtpfile_t *fp)
{
    return (fp->pos);
}

FSIZE_t
fs_mtp_size(mtpfile_t *fp)
{
    return (fp->size);
}

/* Truncate at the current position */
FRESULT
fs_mtp_truncate(mtpfile_t *fp)
{
    mvol_t  *v = mvol_get(fp->mvol);
    uint32_t params[3];
    FRESULT  fr;

    if (v == NULL)
        return (FR_NOT_READY);
    if (!(fp->mode & FA_WRITE))
        return (FR_DENIED);
    if (!(v->ops & MTP_OPS_EDIT))
        return (FR_WRITE_PROTECTED);
    fr = wbuf_flush(fp);
    if (fr != FR_OK)
        return (fr);
    if (!fp->editing) {
        fr = mx(v, MTP_OC_ANDROID_BeginEditObject, 1, &fp->handle,
                USB_MTP_DIR_NONE, NULL, 0, NULL);
        if (fr != FR_OK)
            return (fr);
        fp->editing = 1;
    }
    params[0] = fp->handle;
    params[1] = (uint32_t) fp->pos;
    params[2] = (uint32_t) ((uint64_t) fp->pos >> 32);
    fr = mx(v, MTP_OC_ANDROID_TruncateObject, 3, params, USB_MTP_DIR_NONE,
            NULL, 0, NULL);
    if (fr == FR_OK) {
        fp->size = fp->pos;
        fp->rbuf_len = 0;
    }
    return (fr);
}

FRESULT
fs_mtp_sync(mtpfile_t *fp)
{
    return (edit_end(fp));
}

FRESULT
fs_mtp_close(mtpfile_t *fp)
{
    FRESULT fr = edit_end(fp);

    free(fp->rbuf);
    free(fp->wbuf);
    free(fp);
    return (fr);
}

FRESULT
fs_mtp_mkdir(uint mvol, const char *path)
{
    mvol_t  *v = mvol_get(mvol);
    char    *ppath;
    const char *name;
    oinfo_t *oi;
    uint32_t parent;
    uint32_t handle;
    FRESULT  fr;

    if (v == NULL)
        return (FR_NOT_READY);
    if (v->readonly && !(v->ops & MTP_OPS_SENDINFO))
        return (FR_WRITE_PROTECTED);
    ppath = malloc(strlen(path) + 2);
    oi = malloc(sizeof (*oi));
    if ((ppath == NULL) || (oi == NULL)) {
        free(ppath);
        free(oi);
        return (FR_NOT_ENOUGH_CORE);
    }
    strcpy(ppath, path);
    name = path_split(ppath);
    fr = resolve_dir(mvol, v, ppath, &parent);
    if (fr == FR_OK) {
        fr = lookup_child(mvol, v, parent, name, strlen(name), &handle, oi);
        if (fr == FR_OK)
            fr = FR_EXIST;
        else if (fr == FR_NO_FILE)
            fr = create_object(mvol, v, parent, name, 1, &handle);
    }
    free(ppath);
    free(oi);
    return (fr);
}

FRESULT
fs_mtp_unlink(uint mvol, const char *path)
{
    mvol_t  *v = mvol_get(mvol);
    oinfo_t *oi;
    uint32_t handle;
    FRESULT  fr;

    if (v == NULL)
        return (FR_NOT_READY);
    if (!(v->ops & MTP_OPS_DELETE))
        return (FR_WRITE_PROTECTED);
    oi = malloc(sizeof (*oi));
    if (oi == NULL)
        return (FR_NOT_ENOUGH_CORE);
    fr = resolve(mvol, v, path, &handle, oi);
    if ((fr == FR_OK) && (handle == MTP_ROOT))
        fr = FR_DENIED;
    if (fr == FR_OK) {
        uint32_t params[2] = { handle, 0 };
        fr = mx(v, PTP_OC_DeleteObject, 2, params, USB_MTP_DIR_NONE, NULL, 0,
                NULL);
        cache_dir_changed(mvol, oi->parent);
    }
    free(oi);
    return (fr);
}

FRESULT
fs_mtp_rename(uint mvol, const char *opath, const char *npath)
{
    mvol_t  *v = mvol_get(mvol);
    oinfo_t *oi;
    char    *ppath = NULL;
    const char *nname;
    uint32_t handle;
    uint32_t oparent;
    uint32_t nparent;
    uint32_t other;
    FRESULT  fr;

    if (v == NULL)
        return (FR_NOT_READY);
    oi = malloc(sizeof (*oi));
    ppath = malloc(strlen(npath) + 2);
    if ((oi == NULL) || (ppath == NULL)) {
        fr = FR_NOT_ENOUGH_CORE;
        goto done;
    }
    fr = resolve(mvol, v, opath, &handle, oi);
    if (fr != FR_OK)
        goto done;
    if (handle == MTP_ROOT) {
        fr = FR_DENIED;
        goto done;
    }
    oparent = oi->parent;
    strcpy(ppath, npath);
    nname = path_split(ppath);
    fr = resolve_dir(mvol, v, ppath, &nparent);
    if (fr != FR_OK)
        goto done;

    /* An existing object is never replaced (as AmigaOS) */
    fr = lookup_child(mvol, v, nparent, nname, strlen(nname), &other, oi);
    if ((fr == FR_OK) && (other != handle)) {
        fr = FR_EXIST;
        goto done;
    }
    if ((fr != FR_OK) && (fr != FR_NO_FILE))
        goto done;
    fr = get_info(v, handle, oi);
    if (fr != FR_OK)
        goto done;

    if (nparent != oparent) {
        uint32_t params[3] = { handle, v->storage,
                               (nparent == MTP_ROOT) ? 0 : nparent };
        if (!(v->ops & MTP_OPS_MOVE)) {
            fr = FR_DENIED;
            goto done;
        }
        fr = mx(v, PTP_OC_MoveObject, 3, params, USB_MTP_DIR_NONE, NULL, 0,
                NULL);
        cache_dir_changed(mvol, oparent);
        cache_dir_changed(mvol, nparent);
        if (fr != FR_OK)
            goto done;
    }
    if (strcmp(oi->name, nname) != 0) {
        uint32_t params[2] = { handle, PTP_OPC_ObjectFileName };
        uint8_t  sbuf[1 + 2 * 256];
        uint8_t *end;
        if (!(v->ops & MTP_OPS_SETPROP)) {
            fr = FR_DENIED;
            goto done;
        }
        end = put_str(sbuf, nname);
        fr = mx(v, PTP_OC_SetObjectPropValue, 2, params, USB_MTP_DIR_OUT,
                sbuf, end - sbuf, NULL);
        cache_dir_changed(mvol, nparent);
    }
done:
    free(oi);
    free(ppath);
    return (fr);
}

/* Set the modify date (devices often don't allow it: errors are ignored) */
FRESULT
fs_mtp_utime(uint mvol, const char *path, const FILINFO *fno)
{
    mvol_t  *v = mvol_get(mvol);
    oinfo_t *oi;
    uint32_t handle;
    FRESULT  fr;

    if (v == NULL)
        return (FR_NOT_READY);
    oi = malloc(sizeof (*oi));
    if (oi == NULL)
        return (FR_NOT_ENOUGH_CORE);
    fr = resolve(mvol, v, path, &handle, oi);
    free(oi);
    if ((fr == FR_OK) && (handle != MTP_ROOT) && (v->ops & MTP_OPS_SETPROP)) {
        uint32_t params[2] = { handle, PTP_OPC_DateModified };
        char     date[20];
        uint8_t  sbuf[1 + 2 * 20];
        uint8_t *end;
        sprintf(date, "%04u%02u%02uT%02u%02u%02u",
                (fno->fdate >> 9) + 1980, (fno->fdate >> 5) & 0xf,
                fno->fdate & 0x1f, fno->ftime >> 11, (fno->ftime >> 5) & 0x3f,
                (fno->ftime & 0x1f) * 2);
        end = put_str(sbuf, date);
        (void) mx(v, PTP_OC_SetObjectPropValue, 2, params, USB_MTP_DIR_OUT,
                  sbuf, end - sbuf, NULL);
    }
    return (fr);
}

FRESULT
fs_mtp_getfree(uint mvol, uint64_t *total, uint64_t *freeb)
{
    mvol_t *v = mvol_get(mvol);
    usb_mtp_result_t res;
    FRESULT fr;

    *total = 0;
    *freeb = 0;
    if (v == NULL)
        return (FR_NOT_READY);
    fr = mx(v, PTP_OC_GetStorageInfo, 1, &v->storage, USB_MTP_DIR_IN, oi_buf,
            sizeof (oi_buf), &res);
    if (fr != FR_OK)
        return (fr);
    if (res.len < PTP_si_FreeSpaceInBytes + 8)
        return (FR_DISK_ERR);
    *total = get_le32(oi_buf + PTP_si_MaxCapability) |
             ((uint64_t) get_le32(oi_buf + PTP_si_MaxCapability + 4) << 32);
    *freeb = get_le32(oi_buf + PTP_si_FreeSpaceInBytes) |
             ((uint64_t) get_le32(oi_buf + PTP_si_FreeSpaceInBytes + 4) << 32);
    return (FR_OK);
}
