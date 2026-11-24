/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * BEC file service for the Amiga (BEC_CMD_F* messages), serving files
 * from USB mass storage.
 */

#ifndef _FS_SERVER_H
#define _FS_SERVER_H

#include <stdint.h>

/*
 * fsrv_cmd() - process a BEC_CMD_F* request
 *
 * msg and msglen are the request payload. Returns the BEC_STATUS_* reply
 * status; *reply and *replylen are set to the reply payload.
 */
uint fsrv_cmd(uint cmd, const uint8_t *msg, uint msglen,
              const void **reply, uint *replylen);

/* Flush data of all files open for write to the USB storage devices */
void fsrv_sync_all(void);

/* Show open handles ("fs handles") */
void fsrv_show_handles(void);

/* Self test ("fs test [write] [<volume>]") */
void fsrv_test(uint write, const char *volname);

extern uint8_t fsrv_debug;  // Non-zero to log file service requests

#endif /* _FS_SERVER_H */
