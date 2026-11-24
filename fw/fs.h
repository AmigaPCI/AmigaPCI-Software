/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * Filesystem (FatFs) commands for USB mass storage.
 */

#ifndef _FS_H
#define _FS_H

#include "cmdline.h"

extern const char cmd_fs_help[];
rc_t cmd_fs(int argc, char * const *argv);

#endif /* _FS_H */
