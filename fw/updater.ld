MEMORY
{
    rom (rx) : ORIGIN = 0x20000000, LENGTH = 16K
    ram (rwx) : ORIGIN = 0x20004000, LENGTH = 8K
    persist (rwx) : ORIGIN = 0x2000fc00, LENGTH = 1K
}

SECTIONS
{
    .persist : {
        *(.persist*)        /* Read-write zero initialized data */
    } >persist
}

INCLUDE cortex-m-generic.ld
