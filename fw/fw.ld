MEMORY {
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 384K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 127K
    persist (rwx) : ORIGIN = 0x2001fc00, LENGTH = 1K
}

/* Keep the firmware identification structure (see main.h) */
EXTERN(fw_info)

SECTIONS
{
    .persist : {
        *(.persist*)        /* Read-write zero initialized data */
    } >persist
}

INCLUDE cortex-m-generic.ld

/* Size in bytes of the firmware image in flash (.text through .data) */
_fw_size = ABSOLUTE(LOADADDR(.data) + SIZEOF(.data) - ORIGIN(rom));
