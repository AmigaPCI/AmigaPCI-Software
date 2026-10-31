/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * main prototypes.
 */

#ifndef _MAIN_H
#define _MAIN_H

#include <stdint.h>

typedef unsigned int uint;

void main_poll(void);

/*
 * Firmware identification structure, embedded in the firmware image so that
 * an external program (such as the updater) can locate it by scanning flash
 * for fi_magic. A match is only valid if fi_self points to the structure
 * itself, which rejects the magic value appearing elsewhere in the image.
 */
#define FW_INFO_MAGIC 0x42454346  // "BECF"

typedef struct fw_info {
    uint32_t                fi_magic;       // FW_INFO_MAGIC
    uint32_t                fi_struct_size; // sizeof (fw_info_t)
    const struct fw_info   *fi_self;        // Address of this structure
    const char *const      *fi_version_str; // Pointer to version_str
    uint32_t                fi_fw_size;     // Firmware image size in bytes
} fw_info_t;

extern const fw_info_t fw_info;

#endif /* _MAIN_H */
