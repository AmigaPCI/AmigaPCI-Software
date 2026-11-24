/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * AmigaPCI BEC file service client: files on USB storage attached to the
 * BEC. This is the BEC equivalent of KickSmash sm_file.c, with the same
 * functions (bf_* instead of sm_*). Requests and replies use the
 * structures and BF_* values in bec_cmd.h, which mirror the KickSmash
 * hm_* structures and HM_* values. All values are big-endian, which is
 * native for the Amiga.
 *
 * Each request is a single BEC message. Requests are re-sent if the reply
 * is lost; the BEC_CMD_FSEQ bit lets the BEC recognize a re-sent request
 * and return its saved reply rather than performing it a second time.
 */

#include <stdint.h>
#include <string.h>
#include <exec/types.h>
#include <exec/memory.h>
#include <proto/exec.h>
#include "printf.h"
#include "becmsg.h"
#include "bec_file.h"

#define BF_SEND_TRIES   8   // Attempts for a request whose reply was lost
#define BF_NAME_MAX     (BEC_MSG_MAX_PAYLOAD - 32)  // Request names total

uint8_t bf_file_active = 0;  // File service is available
uint8_t bf_host_locks  = 0;  // BEC enforces BF_MODE_LOCK

static uint8_t  bf_seq;      // BEC_CMD_FSEQ value of the last request

/*
 * Message buffers, allocated by bf_fservice() and kept until
 * bf_file_exit(). (Static buffers would take ROM space in becfsrom, whose
 * BSS is part of its data image.) AllocVec() memory is long-aligned, as
 * the reply structures need.
 */
#define BF_REQBUF_SIZE   BEC_MSG_MAX_PAYLOAD
#define BF_REPLYBUF_SIZE (BEC_MSG_MAX_PAYLOAD + 8)
static uint8_t *bf_reqbuf;
static uint8_t *bf_replybuf;

/*
 * bf_transient
 * ------------
 * Returns non-zero if a request status means the request or its reply was
 * lost or damaged in transfer, so it may be sent again.
 */
static int
bf_transient(uint rc)
{
    switch (rc) {
        case BEC_STATUS_CRC:       // BEC got a damaged request
        case BEC_STATUS_TIMEOUT:   // No reply
        case BEC_STATUS_BADMAGIC:  // Damaged reply
        case BEC_STATUS_REPLYCRC:
        case BEC_STATUS_NODATA:    // Keyboard interface: reply incomplete
            return (1);
        default:
            return (0);
    }
}

/*
 * bf_rw_max
 * ---------
 * Largest file data per read or write message: BF_RW_MAX, or less on the
 * keyboard interface (see bec_msg_data_max()). Callers handle short reads
 * and bf_fwrite() splits writes, so the limit is not seen above here.
 */
static uint
bf_rw_max(void)
{
    uint max = bec_msg_data_max() - sizeof (bf_freadwrite_t);
    return ((max < BF_RW_MAX) ? max : BF_RW_MAX);
}

/* Is the status a failure to communicate with the BEC at all? */
static int
bf_link_error(uint rc)
{
    return (bf_transient(rc) || (rc == BEC_STATUS_REPLYLEN));
}

/*
 * bf_msg
 * ------
 * Sends a file request (in bf_reqbuf) and receives its reply into
 * bf_replybuf. *rlen is the reply length.
 */
static uint
bf_msg(uint cmd, uint reqlen, uint *rlen)
{
    uint tries = BF_SEND_TRIES;
    uint alen = 0;
    uint rc;

    bf_seq ^= BEC_CMD_FSEQ;   // New request (re-sends keep the same value)
    do {
        alen = 0;
        rc = send_cmd(cmd | bf_seq, bf_reqbuf, reqlen, bf_replybuf,
                      BF_REPLYBUF_SIZE, &alen);
    } while (bf_transient(rc) && (--tries > 0));

    if (alen > BF_REPLYBUF_SIZE)
        alen = BF_REPLYBUF_SIZE;
    *rlen = bf_link_error(rc) ? 0 : alen;
    if (bf_link_error(rc)) {
        printf("BEC file request %02x failed: %s\n", cmd, bec_err(rc));
        (void) bf_fservice();  // Is the BEC still there?
    }
    return (rc);
}

/*
 * bf_fservice
 * -----------
 * Returns non-zero if the BEC provides file service. This is checked
 * with a path request for the Volume Directory, which needs no storage.
 * Firmware without file service rejects it.
 */
uint
bf_fservice(void)
{
    bf_fhandle_t *req;
    uint tries = BF_SEND_TRIES;
    uint alen = 0;
    uint rc;

    if (bf_reqbuf == NULL) {
        bf_reqbuf = AllocVec(BF_REQBUF_SIZE + BF_REPLYBUF_SIZE, MEMF_PUBLIC);
        if (bf_reqbuf == NULL) {
            bf_file_active = 0;
            return (0);
        }
        bf_replybuf = bf_reqbuf + BF_REQBUF_SIZE;
    }
    req = (bf_fhandle_t *) bf_reqbuf;
    req->bf_handle = 0;
    bf_seq ^= BEC_CMD_FSEQ;
    do {
        rc = send_cmd(BEC_CMD_FPATH | bf_seq, req, sizeof (*req),
                      bf_replybuf, BF_REPLYBUF_SIZE, &alen);
    } while (bf_transient(rc) && (--tries > 0));
    bf_file_active = (rc == BEC_STATUS_OK) && (alen > sizeof (*req));
    return (bf_file_active);
}

/* Copies a name (and an optional second name) after a request structure */
static uint
bf_req_names(uint structlen, const char *name1, const char *name2,
             uint *reqlen)
{
    uint len1 = strlen(name1) + 1;
    uint len2 = (name2 == NULL) ? 0 : strlen(name2) + 1;
    char *ptr = (char *) bf_reqbuf + structlen;

    if (len1 + len2 > BF_NAME_MAX) {
        printf("Path \"%s\" too long\n", name1);
        return (BF_CSTATUS_BAD_LENGTH);
    }
    memcpy(ptr, name1, len1);
    if (name2 != NULL)
        memcpy(ptr + len1, name2, len2);
    *reqlen = structlen + len1 + len2;
    return (BEC_STATUS_OK);
}

/* Note: functions must not use bf_reqbuf or bf_replybuf before this */
#define BF_SERVICE_CHECK() \
    if ((bf_file_active == 0) && (bf_fservice() == 0)) \
        return (BEC_STATUS_UNAVAIL)

/*
 * bf_fopen
 * --------
 * Opens the named object relative to parent_handle, returning a handle.
 * Parent handle 0 is the Volume Directory, and 0xffffffff is the root of
 * the first USB storage volume. See bec_cmd.h for path forms and modes.
 */
uint
bf_fopen(handle_t parent_handle, const char *name, uint mode, uint *bf_type,
         uint create_perms, handle_t *handle)
{
    bf_fopenhandle_t *req;
    bf_fopenhandle_t *rep;
    uint reqlen;
    uint rlen;
    uint rc;

    *handle = 0;
    BF_SERVICE_CHECK();
    req = (bf_fopenhandle_t *) bf_reqbuf;
    rep = (bf_fopenhandle_t *) bf_replybuf;
    req->bf_handle = parent_handle;
    req->bf_type   = 0;
    req->bf_mode   = mode;
    req->bf_aperms = create_perms;
    rc = bf_req_names(sizeof (*req), name, NULL, &reqlen);
    if (rc != BEC_STATUS_OK)
        return (rc);

    rc = bf_msg(BEC_CMD_FOPEN, reqlen, &rlen);
    if ((rc == BEC_STATUS_OK) && (rlen < sizeof (*rep))) {
        printf("fopen: short reply %u\n", rlen);
        rc = BF_CSTATUS_BAD_DATA;
    }
    if (rc == BEC_STATUS_OK) {
        *handle = rep->bf_handle;
        bf_host_locks = (rep->bf_mode & BF_MODE_LOCK) ? 1 : 0;
        if (bf_type != NULL)
            *bf_type = rep->bf_type;
    }
    return (rc);
}

uint
bf_fclose(handle_t handle)
{
    bf_fopenhandle_t *req;
    uint rlen;

    BF_SERVICE_CHECK();
    req = (bf_fopenhandle_t *) bf_reqbuf;
    memset(req, 0, sizeof (*req));
    req->bf_handle = handle;
    return (bf_msg(BEC_CMD_FCLOSE, sizeof (*req), &rlen));
}

/*
 * bf_fread
 * --------
 * Reads file data or directory entries. *data is set to point to the data
 * in a static buffer, valid until the next request. At most bf_rw_max()
 * bytes are returned per call. A directory read always returns at least
 * one complete entry, which may be longer than readsize.
 */
uint
bf_fread(handle_t handle, uint readsize, void **data, uint *rlen, uint flags)
{
    bf_freadwrite_t *req;
    bf_freadwrite_t *rep;
    uint replylen;
    uint rc;

    *rlen = 0;
    BF_SERVICE_CHECK();
    req = (bf_freadwrite_t *) bf_reqbuf;
    rep = (bf_freadwrite_t *) bf_replybuf;
    if (readsize > bf_rw_max())
        readsize = bf_rw_max();
    req->bf_handle = handle;
    req->bf_length = readsize;
    req->bf_flag   = flags;
    req->bf_unused = 0;

    rc = bf_msg(BEC_CMD_FREAD, sizeof (*req), &replylen);
    if ((rc != BEC_STATUS_OK) && (rc != BEC_STATUS_EOF))
        return (rc);
    if ((replylen < sizeof (*rep)) ||
        (rep->bf_length != replylen - sizeof (*rep))) {
        printf("fread: bad reply %u (%u)\n", replylen,
               (replylen < sizeof (*rep)) ? 0 : rep->bf_length);
        return (BF_CSTATUS_BAD_DATA);
    }
    *data = (void *) (rep + 1);
    *rlen = rep->bf_length;
    return (rc);
}

/*
 * bf_fwrite
 * ---------
 * Writes data at the file's current position. If padded_header is set,
 * buf begins with space for a bf_freadwrite_t, which is followed by the
 * data (as for KickSmash sm_fwrite()). The data is sent in messages of up
 * to BF_RW_MAX bytes.
 */
uint
bf_fwrite(handle_t handle, void *buf, uint writelen, uint padded_header,
          uint flags)
{
    bf_freadwrite_t *req;
    const uint8_t   *data = (const uint8_t *) buf;
    uint             rlen;
    uint             rc = BEC_STATUS_OK;

    BF_SERVICE_CHECK();
    req = (bf_freadwrite_t *) bf_reqbuf;
    if (padded_header)
        data += sizeof (bf_freadwrite_t);
    while (writelen > 0) {
        uint chunk = (writelen > bf_rw_max()) ? bf_rw_max() : writelen;
        req->bf_handle = handle;
        req->bf_length = chunk;
        req->bf_flag   = flags;
        req->bf_unused = 0;
        memcpy(req + 1, data, chunk);
        rc = bf_msg(BEC_CMD_FWRITE, sizeof (*req) + chunk, &rlen);
        if (rc != BEC_STATUS_OK)
            break;
        data     += chunk;
        writelen -= chunk;
        flags    &= ~BF_FLAG_SEEK0;  // Only before the first chunk
    }
    return (rc);
}

/*
 * bf_fpath
 * --------
 * Gets the full path of a handle. *name points to a static buffer, valid
 * until the next request.
 */
uint
bf_fpath(handle_t handle, char **name)
{
    bf_fhandle_t *req;
    uint rlen;
    uint rc;

    BF_SERVICE_CHECK();
    req = (bf_fhandle_t *) bf_reqbuf;
    req->bf_handle = handle;
    rc = bf_msg(BEC_CMD_FPATH, sizeof (*req), &rlen);
    if (rc != BEC_STATUS_OK)
        return (rc);
    if ((rlen <= sizeof (*req)) ||
        (memchr((char *) bf_replybuf + sizeof (*req), '\0',
                rlen - sizeof (*req)) == NULL)) {
        printf("fpath: bad reply %u\n", rlen);
        return (BF_CSTATUS_BAD_DATA);
    }
    *name = (char *) bf_replybuf + sizeof (*req);
    return (BEC_STATUS_OK);
}

uint
bf_fdelete(handle_t handle, const char *name)
{
    bf_fhandle_t *req;
    uint reqlen;
    uint rlen;
    uint rc;

    BF_SERVICE_CHECK();
    req = (bf_fhandle_t *) bf_reqbuf;
    req->bf_handle = handle;
    rc = bf_req_names(sizeof (*req), name, NULL, &reqlen);
    if (rc == BEC_STATUS_OK)
        rc = bf_msg(BEC_CMD_FDELETE, reqlen, &rlen);
    if (rc != BEC_STATUS_OK)
        printf("Failed to delete %s: %s\n", name, bf_err(rc));
    return (rc);
}

uint
bf_frename(handle_t shandle, const char *name_old,
           handle_t dhandle, const char *name_new)
{
    bf_frename_t *req;
    uint reqlen;
    uint rlen;
    uint rc;

    BF_SERVICE_CHECK();
    req = (bf_frename_t *) bf_reqbuf;
    req->bf_shandle = shandle;
    req->bf_dhandle = dhandle;
    rc = bf_req_names(sizeof (*req), name_old, name_new, &reqlen);
    if (rc == BEC_STATUS_OK)
        rc = bf_msg(BEC_CMD_FRENAME, reqlen, &rlen);
    if (rc != BEC_STATUS_OK) {
        printf("Failed to rename %s to %s: %s\n",
               name_old, name_new, bf_err(rc));
    }
    return (rc);
}

/*
 * bf_fcreate
 * ----------
 * Creates a file or directory (BF_TYPE_FILE or BF_TYPE_DIR). FAT has no
 * links or special files, so tgt_name is only sent for compatibility.
 */
uint
bf_fcreate(handle_t parent_handle, const char *name, const char *tgt_name,
           uint bf_type, uint create_perms)
{
    bf_fopenhandle_t *req;
    uint reqlen;
    uint rlen;
    uint rc;

    BF_SERVICE_CHECK();
    req = (bf_fopenhandle_t *) bf_reqbuf;
    req->bf_handle = parent_handle;
    req->bf_type   = bf_type;
    req->bf_mode   = 0;
    req->bf_aperms = create_perms;
    rc = bf_req_names(sizeof (*req), name, tgt_name, &reqlen);
    if (rc == BEC_STATUS_OK)
        rc = bf_msg(BEC_CMD_FCREATE, reqlen, &rlen);
    if (rc != BEC_STATUS_OK)
        printf("Failed to create %s: %s\n", name, bf_err(rc));
    return (rc);
}

uint
bf_fseek(handle_t handle, int seek_mode, uint64_t offset,
         uint64_t *new_pos, uint64_t *prev_pos, uint flags)
{
    bf_fseek_t *req;
    bf_fseek_t *rep;
    uint rlen;
    uint rc;

    if (new_pos != NULL)
        *new_pos = 0;
    if (prev_pos != NULL)
        *prev_pos = 0;
    BF_SERVICE_CHECK();
    req = (bf_fseek_t *) bf_reqbuf;
    rep = (bf_fseek_t *) bf_replybuf;
    req->bf_handle  = handle;
    req->bf_off_hi  = offset >> 32;
    req->bf_off_lo  = offset;
    req->bf_old_hi  = 0;
    req->bf_old_lo  = 0;
    req->bf_seek    = seek_mode;
    req->bf_flags   = flags;
    req->bf_unused2 = 0;
    rc = bf_msg(BEC_CMD_FSEEK, sizeof (*req), &rlen);
    if ((rc == BEC_STATUS_OK) && (rlen < sizeof (*rep)))
        rc = BF_CSTATUS_BAD_DATA;
    if (rc != BEC_STATUS_OK) {
        printf("Failed to seek: %s\n", bf_err(rc));
        return (rc);
    }
    if (new_pos != NULL)
        *new_pos = ((uint64_t) rep->bf_off_hi << 32) | rep->bf_off_lo;
    if (prev_pos != NULL)
        *prev_pos = ((uint64_t) rep->bf_old_hi << 32) | rep->bf_old_lo;
    return (rc);
}

/*
 * bf_fsetdate
 * -----------
 * which: 0=set modify, 1=get modify, 2=set change, 3=get change,
 *        4=set access, 5=get access. Times are seconds since 1970.
 */
uint
bf_fsetdate(handle_t parent_handle, const char *name, uint which,
            uint *sec, uint *nsec)
{
    bf_fsetdate_t *req;
    bf_fsetdate_t *rep;
    uint reqlen;
    uint rlen;
    uint rc;

    BF_SERVICE_CHECK();
    req = (bf_fsetdate_t *) bf_reqbuf;
    rep = (bf_fsetdate_t *) bf_replybuf;
    req->bf_handle  = parent_handle;
    req->bf_which   = which;
    req->bf_unused0 = 0;
    req->bf_unused1 = 0;
    req->bf_time    = *sec;
    req->bf_time_ns = *nsec;
    rc = bf_req_names(sizeof (*req), name, NULL, &reqlen);
    if (rc == BEC_STATUS_OK)
        rc = bf_msg(BEC_CMD_FSETDATE, reqlen, &rlen);
    if (rc != BEC_STATUS_OK) {
        printf("Failed to set date %u.%u %s: %s\n",
               *sec, *nsec, name, bf_err(rc));
    } else if (rlen >= sizeof (*rep)) {
        /* For the "get" operations, the reply carries the time */
        *sec  = rep->bf_time;
        *nsec = rep->bf_time_ns;
    }
    return (rc);
}

uint
bf_fsetown(handle_t parent_handle, const char *name, uint oid, uint gid)
{
    bf_fsetown_t *req;
    uint reqlen;
    uint rlen;
    uint rc;

    BF_SERVICE_CHECK();
    req = (bf_fsetown_t *) bf_reqbuf;
    req->bf_handle = parent_handle;
    req->bf_oid    = oid;
    req->bf_gid    = gid;
    rc = bf_req_names(sizeof (*req), name, NULL, &reqlen);
    if (rc == BEC_STATUS_OK)
        rc = bf_msg(BEC_CMD_FSETOWN, reqlen, &rlen);
    if (rc != BEC_STATUS_OK) {
        printf("Failed to set owner %u.%u for %s: %s\n",
               oid, gid, name, bf_err(rc));
    }
    return (rc);
}

/*
 * bf_fsetprotect
 * --------------
 * Sets Amiga protection bits. On FAT, write or delete protection sets the
 * read-only attribute, and the archive bit maps to the archive attribute.
 */
uint
bf_fsetprotect(handle_t parent_handle, const char *name, uint perms)
{
    bf_fopenhandle_t *req;
    uint reqlen;
    uint rlen;
    uint rc;

    BF_SERVICE_CHECK();
    req = (bf_fopenhandle_t *) bf_reqbuf;
    req->bf_handle = parent_handle;
    req->bf_type   = 0;
    req->bf_mode   = 0;
    req->bf_aperms = perms;
    rc = bf_req_names(sizeof (*req), name, NULL, &reqlen);
    if (rc == BEC_STATUS_OK)
        rc = bf_msg(BEC_CMD_FSETPERMS, reqlen, &rlen);
    if (rc != BEC_STATUS_OK) {
        printf("Failed to set perms 0x%x for %s: %s\n",
               perms, name, bf_err(rc));
    }
    return (rc);
}

const char *
bf_err(uint status)
{
    switch (status) {
        case BF_CSTATUS_NO_MEM:
            return ("Out of memory");
        case BF_CSTATUS_BAD_LENGTH:
            return ("Request too long");
        case BF_CSTATUS_BAD_DATA:
            return ("Malformed reply");
        default:
            return (bec_err(status));
    }
}

/* Frees the message buffers, at program exit */
void
bf_file_exit(void)
{
    FreeVec(bf_reqbuf);
    bf_reqbuf      = NULL;
    bf_replybuf    = NULL;
    bf_file_active = 0;
}
