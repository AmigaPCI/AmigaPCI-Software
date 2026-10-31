/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Erase and write STM32 internal flash memory
 */

#include "printf.h"
#include "board.h"
#include "main.h"
#include "stm32flash.h"
#include "cmdline.h"
#include "utils.h"
#include <string.h>
#include <stdbool.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/common/flash_common_idcache.h>

/*
 * STM32F205 / STM32F215 / STM32F207 / STM32F217
 *
 *            |  Sector 0  0x08000000 - 0x08003fff  16 KB
 *            |  Sector 1  0x08004000 - 0x08007fff  16 KB
 *            |  Sector 2  0x08008000 - 0x0800bfff  16 KB
 *            |  Sector 3  0x0800c000 - 0x0800ffff  16 KB
 * User Flash |  Sector 4  0x08010000 - 0x0801ffff  64 KB
 *            |  Sector 5  0x08020000 - 0x0803ffff  128 KB
 *            |  Sector 6  0x08040000 - 0x0808ffff  128 KB
 * ...        |
 *            |  Sector 11 0x080e0000 - 0x080fffff  128 KB
 *
 * System Memory (DFU)     0x1fff0000 - 0x1fff77ff  30 KB
 * OTP area                0x1fff7800 - 0x1fff7a0f  528 bytes
 * Option bytes            0x1fffc000 - 0x1fffc00f  16 bytes
 *
 * Option byte 0
 *      bit 1:0    Not used (11)
 *      bit 2:3    BOR level (reset voltage threshold)
 *                   00 = 2.70V to 3.60V   01 = 2.40V to 2.70V
 *                   10 = 2.10V to 2.40V   11 = 1.8V to 2.10V
 *      bit 4      Not used (1)
 *      bit 5      0=HW Watchdog, 1=SW Watchdog
 *      bit 6      0=Reset at Stop enter, 1=No reset
 *      bit 7      0=Reset at Standby enter, 1=No reset
 * Option byte 8
 *      bit 0:7    1=Write protect not active on sector (0-7)
 * Option byte 9
 *      bit 0:3    1=Write protect not active on sector (8-11)
 *      bit 4:7    Unused (1)
 */

/*
 * flash_sector_size
 * -----------------
 * Returns the size in bytes of the specified flash sector
 */
static uint32_t
flash_sector_size(uint32_t sector)
{
    /*
     * Sectors 0-3  16 KB
     * Sector  4    64 KB
     * Sectors 5-11 128 KB
     */
    if (sector < 4)
        return (0x4000);
    else if (sector == 4)
        return (0x10000);
    else if (sector < 12)
        return (0x20000);
    else
        return (0);
}

/*
 * flash_sector_start
 * ------------------
 * Returns the offset from the start of flash of the specified flash sector
 */
static uint32_t
flash_sector_start(uint32_t sector)
{
    if (sector < 4)
        return (sector << 14);
    else if (sector == 4)
        return (0x10000);
    else
        return ((sector - 4) << 17);
}

/*
 * flash_addr_to_sector
 * --------------------
 * Returns the flash sector number corresponding to the specified address
 */
static uint32_t
flash_addr_to_sector(uint32_t addr)
{
    if (addr < 0x10000)
        return (addr >> 14);
    else
        return (4 + (addr >> 17));
}

/*
 * flash_is_erased
 * ---------------
 * Returns true if the specified flash area has already been erased
 */
static bool_t
flash_is_erased(uint32_t addr, uint len)
{
    /*
     * Assume that addr is 4-byte aligned because flash_write() already verified
     * alignment.
     */
    addr += FLASH_BASE;

    if ((addr & 3) == 0) {
        while (len >= sizeof (uint32_t)) {
            if (*ADDR32(addr) != 0xffffffff)
                return (false);

            addr += sizeof (uint32_t);
            len  -= sizeof (uint32_t);
        }
    }

    while (len >= sizeof (uint8_t)) {
        if (*ADDR8(addr) != 0xff)
            return (false);

        addr += sizeof (uint8_t);
        len  -= sizeof (uint8_t);
    }

    return (true);
}

typedef enum {
    FLASH_MODE_NORMAL    = 0,
    FLASH_MODE_AUTOERASE = 1,
    FLASH_MODE_ERASE     = 2,
    FLASH_MODE_PROTECT   = 3
} flash_write_mode_t;

static rc_t
flash_write(uint32_t addr, uint nbytes, const void *datap, uint mode)
{
    uintptr_t data = (uintptr_t) datap;
    uint32_t sector = flash_addr_to_sector(addr);
    uint len = nbytes;

    if (addr + nbytes > STM32FLASH_SIZE)
        return (RC_BAD_PARAM);

    if (FLASH_CR & FLASH_CR_LOCK)
        flash_unlock();

    switch (mode) {
        case FLASH_MODE_AUTOERASE:
        case FLASH_MODE_ERASE: {
            uint32_t last;

            if ((mode == FLASH_MODE_ERASE) &&
                (addr != flash_sector_start(sector)))
                return (RC_FAILURE);
            if (nbytes == 0)
                break;

            /* Erase each sector which begins within the area to be written */
            last = flash_addr_to_sector(addr + nbytes - 1);
            for (; sector <= last; sector++)
                if (flash_sector_start(sector) >= addr)
                    flash_erase_sector(sector, FLASH_CR_PROGRAM_X32);
            break;
        }
        case FLASH_MODE_PROTECT:
            if (!flash_is_erased(addr, nbytes))
                return (RC_PROTECT);
            break;
        default:
            break;
    }

    addr += FLASH_BASE;
    while (nbytes > 0) {
        uintptr_t align = data | addr;
        if ((nbytes >= sizeof (uint32_t)) &&
            ((align & (sizeof (uint32_t) - 1)) == 0)) {
            /* Program in 32-bit writes where possible */
            flash_program_word(addr, *ADDR32(data));
            addr   += sizeof (uint32_t);
            data   += sizeof (uint32_t);
            nbytes -= sizeof (uint32_t);
        } else if ((nbytes >= sizeof (uint16_t)) &&
                   ((align & (sizeof (uint16_t) - 1)) == 0)) {
            /* Program in 16-bit writes where possible */
            flash_program_half_word(addr, *ADDR16(data));
            addr   += sizeof (uint16_t);
            data   += sizeof (uint16_t);
            nbytes -= sizeof (uint16_t);
        } else {
            /* Otherwise, program in 8-bit writes */
            flash_program_byte(addr, *ADDR8(data));
            addr   += sizeof (uint8_t);
            data   += sizeof (uint8_t);
            nbytes -= sizeof (uint8_t);
        }
    }

    /* Flush the data cache */
    flash_dcache_disable();
    flash_dcache_reset();
    flash_dcache_enable();

    /* Verify that flash now contains the written data */
    if (memcmp((const void *) (addr - len), datap, len) != 0)
        return (RC_FAILURE);

    return (RC_SUCCESS);
}

int
stm32flash_erase(uint32_t addr, uint len)
{
    if (addr + len > STM32FLASH_SIZE)
        return (RC_BAD_PARAM);
    while (len > 0) {
        uint32_t sector = flash_addr_to_sector(addr);
        uint32_t size   = flash_sector_size(sector);

        flash_erase_sector(sector, FLASH_CR_PROGRAM_X32);

        addr += size;
        if (size > len)
            len = 0;
        else
            len -= size;
    }

    /* Flush the data cache */
    flash_dcache_disable();
    flash_dcache_reset();
    flash_dcache_enable();
    return (RC_SUCCESS);
}

int
stm32flash_write(uint32_t addr, uint len, void *buf, uint flags)
{
    uint mode = (flags & STM32FLASH_FLAG_AUTOERASE) ? FLASH_MODE_AUTOERASE :
                                                      FLASH_MODE_NORMAL;
    return (flash_write(addr, len, buf, mode));
}

int
stm32flash_read(uint32_t addr, uint len, void *buf)
{
    if (addr + len > STM32FLASH_SIZE)
        return (RC_BAD_PARAM);
    memcpy(buf, (void *)((uint)addr | FLASH_BASE), len);
    return (0);
}
