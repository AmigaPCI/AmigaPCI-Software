/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * AmigaPCI BEC file service client (USB storage attached to the BEC).
 * This is the BEC equivalent of KickSmash sm_file.c.
 */

#ifndef _BEC_FILE_H
#define _BEC_FILE_H

#include <stdint.h>
#include "util.h"
#include "../fw/bec_cmd.h"

typedef bf_handle_t handle_t;

/*
 * Client-side status values, beyond the BEC_STATUS_* values which come
 * from the BEC (or from becmsg.c for transport errors).
 */
#define BF_CSTATUS_NO_MEM       0x100  // Out of memory
#define BF_CSTATUS_BAD_LENGTH   0x101  // Request (name) too long
#define BF_CSTATUS_BAD_DATA     0x102  // Reply malformed

uint bf_fservice(void);
void bf_file_exit(void);
uint bf_fopen(handle_t parent_handle, const char *name, uint mode,
              uint *bf_type, uint create_perms, handle_t *handle);
uint bf_fclose(handle_t handle);
uint bf_fread(handle_t handle, uint readsize, void **data, uint *rlen,
              uint flags);
uint bf_fwrite(handle_t handle, void *buf, uint writelen, uint padded_header,
               uint flags);
uint bf_fpath(handle_t handle, char **name);
uint bf_frename(handle_t shandle, const char *name_old,
                handle_t dhandle, const char *name_new);
uint bf_fcreate(handle_t parent_handle, const char *name, const char *tgt_name,
                uint bf_type, uint create_perms);
uint bf_fdelete(handle_t handle, const char *name);
uint bf_fseek(handle_t handle, int seek_mode, uint64_t offset,
              uint64_t *new_pos, uint64_t *prev_pos, uint flags);
uint bf_fsetdate(handle_t parent_handle, const char *name, uint which,
                 uint *sec, uint *nsec);
uint bf_fsetown(handle_t parent_handle, const char *name, uint oid, uint gid);
uint bf_fsetprotect(handle_t parent_handle, const char *name, uint perms);
const char *bf_err(uint status);

extern uint8_t bf_file_active;
extern uint8_t bf_host_locks;

#define SEEK_OFFSET_BEGINNING (-1)
#define SEEK_OFFSET_CURRENT   (0)
#define SEEK_OFFSET_END       (1)

#endif /* _BEC_FILE_H */
