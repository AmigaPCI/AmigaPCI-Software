/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * Variables which survive a reset or update.
 */

#include <stdint.h>
#include <string.h>
#include "persist.h"

SRAM_PERSIST sram_persist_t persist;

void
persist_init(void)
{
    if ((persist.sp_magic != SP_MAGIC) ||
        (persist.sp_addr != (uintptr_t) &persist)) {
        memset(&persist, 0, sizeof (persist));
        persist.sp_magic = SP_MAGIC;
        persist.sp_addr = (uintptr_t) &persist;
    }
}
