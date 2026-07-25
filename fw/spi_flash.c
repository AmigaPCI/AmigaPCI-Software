/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * SPI flash support.
 */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "printf.h"
#include "cmdline.h"
#include "spi.h"
#include "spi_flash.h"
#include "timer.h"
#include "config.h"
#include "led.h"
#include "crc32.h"
#include "uart.h"

#define SPI_FLASH_MFG_EON          0x1c  // EON Silicon Solution
#define SPI_FLASH_MFG_ATMEL        0x1f  // Atmel JEDEC Vendor ID
#define SPI_FLASH_MFG_MICRON       0x20  // Micron JEDEC Vendor ID
#define SPI_FLASH_MFG_ONSEMI       0x62  // On Semi JEDEC Vendor ID
#define SPI_FLASH_MFG_EVERSPIN     0x6b  // Everspin Vendor ID
#define SPI_FLASH_MFG_MICROCHIP    0xbf  // Microchip (SST) Vendor ID
#define SPI_FLASH_MFG_WINBOND      0xef  // Winbond JEDEC Vendor ID
#define SPI_FLASH_MFG_NONE         0xff  // Invalid

#define MICROCHIP_25AA512    0x25aa512f  // Microchip 25AA512 64KB SPI EEPROM
#define EVERSPIN_MR25H40     0x25840cdc  // Everspin MR25H40CDC 512KB SPI EEPROM
#define ONSEMI_CAT25256      0x25256f62  // On Semi CAT25256 32KB SPI EEPROM

#define SPI_FLASH_BLOCK_SIZE_4KB    (4 * 1024) // Smallest erase unit size
#define SPI_FLASH_BLOCK_SIZE_32KB   (32 * 1024)
#define SPI_FLASH_BLOCK_SIZE_64KB   (64 * 1024)
#define SPI_FLASH_BLOCK_SIZE        SPI_FLASH_BLOCK_SIZE_4KB

/*
 * SPI Flash command set. The prefix refers to the Atmel AT25 series SPI
 * flash chips. Most of these commands also apply to the Micron N25Q and
 * those of several other vendors.
 */
#define AT25_WRITE_STATUS          0x01  // Write status register
#define AT25_WRITE_PAGE_BUFFER     0x02  // Write data bytes
#define AT25_READ_DATA             0x03  // Up to 50 MHz SPI clock
#define AT25_READ_DATA_85MHZ       0x0b  // Faster
#define AT25_READ_DATA_100MHZ      0x1b  // Fastest
#define AT25_WRITE_DISABLE         0x04  // Disable write
#define AT25_READ_STATUS           0x05  // Read status register
#define AT25_WRITE_ENABLE          0x06  // Enable write
#define AT25_IDENTIFY              0x9f  // Identify vendor/device
#define AT25_BLOCK_ERASE_4KB       0x20  // Erase aligned 4KB block
#define AT25_BLOCK_ERASE_32KB      0x52  // Erase aligned 32KB block
#define AT25_BLOCK_ERASE_64KB      0xd8  // Erase aligned 64KB block
#define AT25_CHIP_ERASE            0x60  // Also 0xc7
#define AT25_PROTECT_SECTOR        0x36  // Protect sector from writes
#define AT25_UNPROTECT_SECTOR      0x39  // Unprotect sector from writes
#define AT25_READ_SECTOR_PROT      0x3c  // Read current sector protection
#define AT25_DEEP_POWERDOWN        0xb9  // Low power mode (sleep)
#define AT25_RESUME_FROM_SLEEP     0xab  // Normal operation

/* AT25_READ_STATUS result bits */
#define AT25_STATUS_BUSY           0x01 // Ready/busy (1=busy)
#define AT25_STATUS_WEL            0x02 // Write enable latch (1=write enabled)
#define AT25_STATUS_SWP            0x0c // Software prot status (00=unprotected)
#define AT25_STATUS_WPP            0x10 // Write prot pin stat (1=unprotected)
#define AT25_STATUS_EPE            0x20 // Erase/Program Error (1=error)
#define AT25_STATUS_SPRL           0x80 // Sector protect registers (0=unlocked)

#define AT25_PAGE_BUFFER_SIZE     0x100 // Writes should not cross page boundary

/* Micron M25PE, N25Q, and MT25QU */
/* Micron specific commands */
#define MICRON_READ_FLAG_STATUS    0x70  // Read flag status register command
#define MICRON_CLEAR_FLAG_STATUS   0x50  // Clear flag status register command
#define MICRON_READ_NONVOL_CONFIG  0xb5  // Read nonvolatile config register
#define MICRON_WRITE_NONVOL_CONFIG 0xb1  // Write nonvolatile config register
#define MICRON_ENTER_4BYTE_ADDR    0xb7  // Enter 4 byte addressing mode
#define MICRON_4B_READ_DATA        0x13  // Read command with a 4-byte address
#define MICRON_4B_WRITE_PAGE       0x12  // Write command with a 4-byte address

/* Micron READ_FLAG_STATUS register bits */
#define MICRON_FLAG_STATUS_ADDR    0x01  // Addressing mode (0=3-byte, 1=4-byte)
#define MICRON_FLAG_STATUS_PROT    0x02  // P/E failed due to write protect
#define MICRON_FLAG_STATUS_PSUSP   0x04  // A program is or will be suspended
#define MICRON_FLAG_STATUS_VPP     0x08  // Invalid Vpp during P/E (1=Disabled)
#define MICRON_FLAG_STATUS_PROG    0x10  // Program failed status
#define MICRON_FLAG_STATUS_ERASE   0x20  // Erase failed status
#define MICRON_FLAG_STATUS_ESUSP   0x40  // An erase is or will be suspended
#define MICRON_FLAG_STATUS_READY   0x80  // P/E/W command status (0=busy)

/* Micron N25Q and MT25QU READ_STATUS register bits */
#define MICRON_STATUS_BUSY      0x01  // Ready/busy (1=busy)
#define MICRON_STATUS_WEL       0x02  // Write enable latch (1=write enabled)
#define MICRON_STATUS_SWP       0x1c  // Block prot status (000=unprotected)
#define MICRON_STATUS_TOB       0x20  // Prot memory start (0=Top, 1=Bottom)
#define MICRON_STATUS_WPP       0x80  // Write protect status (1=protected)

/* Micron M25PE READ_STATUS register bits */
#define M25PE_STATUS_WIP        0x01  // Write in progress
#define M25PE_STATUS_WEL        0x02  // Write enable latch (1=write enabled)
#define M25PE_STATUS_SWP        0x0c  // Block protect status (00=unprotected)
#define M25PE_STATUS_SRWD       0x80  // Status register write protect (1=prot)

/* EON EN25Q READ_STATUS register bits */
#define EN25Q_STATUS_WIP        0x01  // Write in progress
#define EN25Q_STATUS_WEL        0x02  // Write enable latch (1=write enabled)
#define EN25Q_STATUS_SWP        0x3c  // Block protect status (0000=unprotected)
#define EN25Q_STATUS_WPDIS      0x40  // Write protect disable
#define EN25Q_STATUS_SRP        0x80  // Status Register Protect

/* Winbond Status Register-1 bits */
#define W25Q_STATUS_BUSY        0x01  // Ready/busy (1=busy)
#define W25Q_STATUS_WEL         0x02  // Write enable latch (1=write enabled)
#define W25Q_STATUS_BP0         0x04  // Block Protect 0
#define W25Q_STATUS_BP1         0x08  // Block Protect 1
#define W25Q_STATUS_BP2         0x10  // Block Protect 2
#define W25Q_STATUS_SWP         0x1c  // Block protect status (000=unprotected)
#define W25Q_STATUS_TB          0x20  // Top/Bottom Protect
#define W25Q_STATUS_SEC         0x40  // Sector protect registers (0=unlocked)
#define W25Q_STATUS_SRP0        0x80  // Status Register Protect 0


#define W25Q_READ_STATUS_2      0x35  // Read status register 2
#define W25Q_WRITE_STATUS_2     0x31  // Write status register 2
#define W25Q_WRITE_EN_VOLATILE  0x50  // Write enable volatile

/* Winbond Status Register-2 bits */
#define W25Q_STATUS_SRP1        0x01  // Status Register Protect 1
#define W25Q_STATUS_QE          0x02  // Quad Enable
#define W25Q_STATUS_LB0         0x04  // Security Register Lock 0
#define W25Q_STATUS_LB1         0x08  // Security Register Lock 1
#define W25Q_STATUS_LB2         0x10  // Security Register Lock 2
#define W25Q_STATUS_LB3         0x20  // Security Register Lock 3
#define W25Q_STATUS_CMP         0x40  // Complement Protect
#define W25Q_STATUS_SUS         0x80  // Suspend Status

/* Microchip 25AA512 Status register bits */
#define MC25AA_STATUS_WIP       0x01  // Write in progress
#define MC25AA_STATUS_WEL       0x02  // Write Enable Latch
#define MC25AA_STATUS_BP0       0x04  // Block Protect 0
#define MC25AA_STATUS_BP1       0x08  // Block Protect 1
#define MC25AA_STATUS_SWP       0x0c  // Block protect status (00=unprotected)

#define MC25AA_PAGE_BUFFER_SIZE 0x80 // Writes should not cross a page boundary

/* On Semi CAT25256 Status register bits */
#define CAT25_STATUS_WIP        0x01  // Write in progress
#define CAT25_STATUS_WEL        0x02  // Write Enable Latch
#define CAT25_STATUS_BP0        0x04  // Block Protect 0
#define CAT25_STATUS_BP1        0x08  // Block Protect 1
#define CAT25_STATUS_SWP        0x0c  // Block protect status (00=unprotected)

#define CAT25_PAGE_BUFFER_SIZE  0x40 // Writes should not cross a page boundary

/*
 * Atmel AT25DF321 access times     Typical  Max
 * ---------------------------      ------- ----
 * 256-byte Page Program                1.0  3.0 ms
 * Byte Program                           7      us
 * 4KB Block Erase                       50  200 ms
 * 32KB Block Erase                     250  600 ms
 * 64KB Block Erase                     400  950 ms
 * Chip Erase                            25   40 sec
 * OTP Register Program                 200  500 us
 * Write Status Register                     200 ns
 *
 * Micron N25Q32                Min Typical  Max
 * ---------------------------  --- ------- ----
 * 256-byte Page Program                0.5  5.0 ms
 * 4KB Subsector Erase                 0.25  0.8 sec
 * 64KB Sector Erase                    0.6  3.0 sec
 * Chip Erase                            30   60 sec
 * OTP Program                          0.2      ms
 * Write Status Register                 40      ns
 * Write Protect Setup           20              us
 * Write Protect Hold           100              us
 *
 * Micron MT25QU01              Min  Typical  Max
 * ---------------------------  ---  ------- ----
 * 256-byte Page Program                0.12  2.8 ms
 * 4KB Subsector Erase                  0.05  0.4 sec
 * 64KB Sector Erase                    0.15  1.0 sec
 * 512Mb Die Erase                      153   460 sec
 * OTP Program                          0.12  0.8 ms
 * Write Status Register                1.3   8.0 ms
 * Write Protect Setup           20               ns
 * Write Protect Hold           100               ns
 *
 * Winbond W25Q32               Min  Typical  Max
 * ---------------------------  ---  ------- ----
 * 256-byte Page Program                0.7   3.0 ms
 * 4KB Sector Erase                      30   200 ms
 * 32KB Block Erase                     120   800 ms
 * 64KB Block Erase                     150  1000 ms
 * Chip Erase                           7.5    30 sec
 * Write Status Register                 10    15 ms
 * Write Protect Setup           20               ns
 * Write Protect Hold           100               ns
 *
 * Winbond W25X20               Min  Typical  Max
 * ---------------------------  ---  ------- ----
 * 256-byte Page Program                 0.4  0.8 ms
 * 4KB Sector Erase                       30  300 ms
 * 32KB Block Erase                      120  800 ms
 * 64KB Block Erase                      150 1000 ms
 * Chip Erase                            0.5    2 sec
 * Write Status Register                  10   15 ms
 * Write Protect Setup           20               ns
 * Write Protect Hold           100               ns
 */
#define FLASH_TIMEOUT          3000  // milliseconds for 64KB erase
#define FLASH_TIMEOUT_SHORT     300  // milliseconds for 4KB erase

#define FLASH_MAX_CHIPSEL      7     // Maximum chip select for flash

#define DATA_CRC_INTERVAL      256   // Buffer size to check CRC

#define P2END(x, a)             (-(~(x) & -(__typeof__(x))(a)))

/* Flash info globals */
static uint32_t flash_ids[FLASH_MAX_CHIPSEL + 1];
static uint32_t flash_sizes[FLASH_MAX_CHIPSEL + 1];

static rc_t spi_flash_identify(uint chip, uint verbose);

static uint8_t
spi_flash_mfg(uint chipsel)
{
    return ((uint8_t)flash_ids[(uint8_t)chipsel]);
}

/*
 * spi_flash_identify_no_probe() returns the flash id of devices which can
 *                               not be probed.
 *
 * @param [in]  chipsel - SPI chip select number.
 *
 * @return      The device id, if it can not be dynamically probed.
 * @return      0 if the device id should be dynamically probed.
 */
static uint32_t
spi_flash_identify_no_probe(uint chipsel)
{
    /*
     * If there is a device on a specific chip select which does not
     * respond to the CFI identify command, this function should return
     * a value to identify that device.
     */
    return (0);
}

#undef SPI_FLASH_QUAD_SUPPORT
#ifdef SPI_FLASH_QUAD_SUPPORT
/*
 * spi_flash_quad_enable() sets the Quad Enable bit (QE) bit in status
 *                         register-2. For Winbond W25 devices the default
 *                         setting for QE is set as a factory default for
 *                         ordering option IQ. For other parts the default is
 *                         0. In our case there is no downside to always
 *                         setting QE. It will often be necessary because
 *                         the device reading from the SPI flash may expect
 *                         QE to be set by default.
 *
 * @param [in]  chip - SPI chip select number.
 *
 * @return      RC_SUCCESS   - The QE bit was successfully set.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_quad_enable(uint chip)
{
    rc_t rc;
    uint8_t status_2 = 0;

    /* Write Enable */
    rc = spi_flash_write_enable(chip);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Read status-2 register */
    rc = spi_flash_read_status_2(chip, &status_2);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Set QE bit in status-2 register */
    status_2 |= W25Q_STATUS_QE;
    rc = spi_chip_write(chipsel, W25Q_WRITE_STATUS_2 << 8 | status_2, 2, 0, NULL);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Wait for status write to complete */
    rc = spi_flash_wait_not_busy(chip, 100);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Read status-2 register */
    rc = spi_flash_read_status_2(chip, &status_2);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Verify QE bit is set */
    if ((status_2 & W25Q_STATUS_QE) == 0)
        return (RC_FAILURE);

    return (RC_SUCCESS);
}
#endif

/*
 * spi_flash_read_status() reads the SPI flash status register.
 *
 * @param [in]  chip   - SPI flash chip select number.
 * @param [out] status - The status value which was read.
 *
 * @return      RC_SUCCESS   - Successful operation.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_read_status(uint chip, uint16_t *status)
{
    uint chipsel = (uint8_t) chip;
    return (spi_chip_read(chipsel, AT25_READ_STATUS, 1, 2, status));
}

#ifdef SPI_FLASH_QUAD
/*
 * spi_flash_read_status_2() reads the SPI flash status register-2.
 *
 * @param [in]  chip   - SPI flash chip select number.
 * @param [out] status - The status value which was read.
 *
 * @return      RC_SUCCESS   - Successful operation.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_read_status_2(uint chip, uint8_t *status)
{
    uint chipsel = (uint8_t) chip;
    return (spi_chip_read(chipsel, W25Q_READ_STATUS_2, 1, 1, status));
}
#endif

/*
 * spi_flash_read_flag_status() reads the SPI flash flag status register.
 *
 * @param [in]  chip   - SPI flash chip select number.
 * @param [out] status - The status value which was read.
 *
 * @return      RC_SUCCESS   - Successful operation.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_read_flag_status(uint chip, uint16_t *status)
{
    uint chipsel = (uint8_t) chip;
    return (spi_chip_read(chipsel, MICRON_READ_FLAG_STATUS, 1, 1, status));
}

/*
 * spi_flash_clear_flag_status() clears the SPI flash flag status register.
 *
 * @param [in]  chip   - SPI flash chip select number.
 *
 * @return      RC_SUCCESS   - Successful operation.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_clear_flag_status(uint chip)
{
    uint chipsel = (uint8_t) chip;
    return (spi_chip_write(chipsel, MICRON_CLEAR_FLAG_STATUS, 1, 0, NULL));
}

/*
 * spi_flash_write_enable() sets the write enable latch in the flash chip.
 *                          The enables the next write or erase operation
 *                          to be performed.
 *
 * @param [in]  chip - SPI flash chip select number.
 *
 * @return      RC_SUCCESS   - Successful operation.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_write_enable(uint chip)
{
    uint chipsel = (uint8_t) chip;
    return (spi_chip_write(chipsel, AT25_WRITE_ENABLE, 1, 0, NULL));
}

/*
 * spi_flash_unprotect() turns off sector write and erase protection in
 *                       the specified SPI flash chip.
 *
 * @param [in]  chip   - SPI flash chip select number.
 *
 * @return      RC_SUCCESS   - Successful operation.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_unprotect(uint chip)
{
    rc_t     rc;
    uint16_t status;
    uint16_t swp;
    uint     mfg;
    uint     chipsel = (uint8_t) chip;

    /* Read status register */
    rc = spi_flash_read_status(chip, &status);
    if (rc != RC_SUCCESS)
        return (rc);

    mfg = spi_flash_mfg(chip);
    switch (mfg) {
        case SPI_FLASH_MFG_EON:
            swp = EN25Q_STATUS_SWP;
            break;
        case SPI_FLASH_MFG_ATMEL:
            swp = AT25_STATUS_SWP;
            if ((status & AT25_STATUS_WPP) == 0) {
                warnx("Flash write failed - WP pin asserted");
                return (RC_PROTECT);
            }
            break;
        case SPI_FLASH_MFG_MICRON:
            swp = MICRON_STATUS_SWP;
            if (status & MICRON_STATUS_WPP) {
                warnx("Flash write failed - WP pin asserted");
                return (RC_PROTECT);
            }
            break;
        case SPI_FLASH_MFG_MICROCHIP:
            swp = MC25AA_STATUS_SWP;
            break;
        case SPI_FLASH_MFG_ONSEMI:
            swp = CAT25_STATUS_SWP;
            break;
        case SPI_FLASH_MFG_EVERSPIN:
            return (RC_SUCCESS);
        case SPI_FLASH_MFG_WINBOND: {
            swp = W25Q_STATUS_SWP;
            /*
             * There does not appear to be a way to read WP, and it
             * doesn't seem to affect writes on the W25Q32DW part.
             */
            break;
        }
        default:
            warnx("Unsupported SPI mfg %x", mfg);
            return (RC_NO_DATA);
    }

    if (status & swp) {
        /* Write Enable */
        rc = spi_flash_write_enable(chip);
        if (rc != RC_SUCCESS)
            return (rc);

        /* Global unprotect by writing status register */
        rc = spi_chip_write(chipsel, AT25_WRITE_STATUS << 8, 2, 0, NULL);
        if (rc != RC_SUCCESS)
            return (rc);

        /* Read status register */
        rc = spi_flash_read_status(chip, &status);
        if (rc != RC_SUCCESS)
            return (rc);

        /* Verify Global unprotect worked */
        if (status & swp) {
            warnx("Flash write failed - could not unprotect");
            return (RC_PROTECT);
        }
    }

    if ((status & AT25_STATUS_WEL) == 0) {
        /* Write Enable */
        rc = spi_flash_write_enable(chip);
        if (rc != RC_SUCCESS)
            return (rc);
    }

    return (RC_SUCCESS);
}

/*
 * spi_flash_wait_not_busy() waits until the SPI flash status no longer
 *                           indicates the device is busy.
 *
 * @param [in]  chip       - SPI flash chip select number.
 * @param [in]  timeout_ms - Timeout in milliseconds.
 *
 * @return      RC_SUCCESS - Successful operation.
 * @return      RC_TIMEOUT - The device took too long to provide status
 *                           that it is not busy. The timeout is driven
 *                           by the timeout_ms
 * @return      RC_FAILURE - A flash programming error was detected.
 */
static rc_t
spi_flash_wait_not_busy(uint chip, uint timeout_ms)
{
    uint16_t status;
    uint64_t timeout;
    rc_t     rc;
    uint8_t  mfg = spi_flash_mfg(chip);

    /* Wait until status is no longer busy */
    timeout = timer_tick_plus_msec(timeout_ms);
    do {
        rc = spi_flash_read_status(chip, &status);
        if (rc != RC_SUCCESS)
            return (rc);
        if (timer_tick_has_elapsed(timeout)) {
            /* Check one more time in case SPI debug code is active */
            rc = spi_flash_read_status(chip, &status);
            if (rc != RC_SUCCESS)
                return (rc);
            break;
        }
    } while (status & AT25_STATUS_BUSY);

    if (status & AT25_STATUS_BUSY) {
        printf("SPI%x Flash busy timeout\n", (uint8_t) chip);
        return (RC_TIMEOUT);
    }

    if (mfg == SPI_FLASH_MFG_ATMEL) {
        if (status & AT25_STATUS_EPE) {
            printf("SPI%x Flash programming error: %02x\n",
                   (uint8_t) chip, status);
            return (RC_FAILURE);
        }
    }
    if (mfg == SPI_FLASH_MFG_MICRON) {
        uint chipsel = (uint8_t) chip;
        uint8_t memtype = (uint8_t) (flash_ids[chipsel] >> 8);

        if (memtype == 0x80) {
            /* M25PE devices only have simple status register */
            return (RC_SUCCESS);
        }

        /* Wait until status is no longer busy */
        timeout = timer_tick_plus_msec(timeout_ms);

        do {
            rc = spi_flash_read_flag_status(chip, &status);
            if (rc != RC_SUCCESS)
                return (rc);
            if (timer_tick_has_elapsed(timeout))
                break;
        } while ((status & MICRON_FLAG_STATUS_READY) == 0);

        if ((status & MICRON_FLAG_STATUS_READY) == 0) {
            printf("SPI%x Flash Write timeout\n", chip);
            return (RC_TIMEOUT);
        }

        if (status & (MICRON_FLAG_STATUS_PROG | MICRON_FLAG_STATUS_ERASE)) {
            printf("SPI%x Flash programming error: %02x\n",
                   (uint8_t) chip, status);
            (void) spi_flash_clear_flag_status(chip);
            (void) spi_flash_read_flag_status(chip, &status);
            if (status & (MICRON_FLAG_STATUS_PROG | MICRON_FLAG_STATUS_ERASE))
                printf("SPI%x Flash failed to clear status: %02x\n",
                       (uint8_t) chip, status);
            return (RC_FAILURE);
        }
    }
    return (RC_SUCCESS);
}

static uint
get_chip_address_mode(uint chip, uint32_t addr)
{
    uint chipsel = (uint8_t) chip;
    uint32_t chip_id;
    uint8_t mfg;

    if ((chip >> 8) != 0)
        return (chip);  // Already has address width specified

    chip_id = spi_flash_identify_no_probe(chipsel);
    switch (chip_id) {
        case MICROCHIP_25AA512: // 25AA512 EEPROM has 2-byte addressing
        case ONSEMI_CAT25256:   // CAT25256 EEPROM has 2-byte addressing
            chip |= (2 << 8);
            break;
        default:
            mfg = spi_flash_mfg(chipsel);
            if (mfg == 0) {
                /* Probe for Mfg ID */
//              (void) spi_flash_identify(chip, 0);
                mfg = spi_flash_mfg(chipsel);
            }
            if (mfg == SPI_FLASH_MFG_MICRON &&
                ((addr >> 24) != 0)) {
                /* Micron SPI Flash > 128Mb uses 4-byte addressing */
                chip |= (4 << 8);
            } else
                chip |= (3 << 8);   // Default to 3-byte addressing
            break;
    }
    return (chip);
}

/*
 * spi_flash_identify() probes to determine the flash id and capacity.
 *                      This function updates the flash_id and flash_size
 *                      globals.
 *
 * @param [in]  chip          - SPI flash chip select number.
 * @param [in]  verbose       - Probe and report ID each time (0=Silent probe)
 *
 * @return      RC_SUCCESS    - The SPI flash chip has been identified.
 * @return      RC_NO_DATA    - The specified chip is not supported.
 * @return      RC_TIMEOUT    - A hardware timeout occurred.
 * @return      RC_BAD_PARAM  - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_identify(uint chip, uint verbose)
{
    const char    *mfgname;
    const char    *part;
    const char    *subpart;
    rc_t           rc;
    uint           mfg;
    uint           chipsel = (uint8_t) chip;
    uint32_t       flash_id   = 0;
    uint           flash_size = 0;
    uint           flash_erase_size = SPI_FLASH_BLOCK_SIZE_4KB;

    if (config.board_type == BOARD_TYPE_UNKNOWN)
        return (RC_NO_DATA);

    if (chipsel >= ARRAY_SIZE(flash_ids))
        return (RC_BAD_PARAM);

    if ((verbose == 0) && (flash_ids[chipsel] != 0))
        return (RC_SUCCESS);

    if ((flash_id = spi_flash_identify_no_probe(chipsel)) == 0) {
        /* Take flash out of powerdown mode 0xa5 */
        rc = spi_chip_write(chipsel, AT25_RESUME_FROM_SLEEP, 0, 0, NULL);
        if (rc != RC_SUCCESS) {
            warnx("SPI%x access failed identify (wake): %d", chipsel, rc);
            goto id_failure;
        }

        /* Wait in case last operation has not completed erase */
        rc = spi_flash_wait_not_busy(chip, FLASH_TIMEOUT_SHORT);
        if (rc != RC_SUCCESS) {
            warnx("SPI%x access failed identify (busy): %d", chipsel, rc);
            goto id_failure;
        }

        /* Acquire the flash ID from the target chip 0x9f */
        rc = spi_chip_read(chipsel, AT25_IDENTIFY, 1, 4, &flash_id);
        if (rc != RC_SUCCESS) {
            warnx("SPI%x access failed identify: %d", chipsel, rc);
            goto id_failure;
        }

        if ((flash_id == 0) || (flash_id == 0xffffffff)) {
            printf("SPI%x Flash ID: %08x", chipsel, (uint) flash_id);
            printf(" failed to identify");
            if (rc != RC_SUCCESS)
                printf(": rc=%d", rc);
            else
                rc = RC_FAILURE;
            printf("\n");
            flash_id = 0;
            goto id_failure;
        }
    }

    flash_ids[chipsel] = flash_id;
    mfg = spi_flash_mfg(chipsel);

    if (verbose)
        printf("SPI%x Flash ID: %08x", chipsel, (uint) flash_id);
    switch (mfg) {
        case SPI_FLASH_MFG_EON: {
            uint8_t family   = (flash_id >> 8) & 0xff;
            uint8_t capacity = (flash_id >> 16) & 0xff;

            if ((family == 0x30) || (family == 0x33)) {
                part = "EN25Q";
                switch (capacity) {
                    case 0x16:  // EN25Q32 4MB 32Mb
                        subpart = "32";
                        flash_size = 4 << 20;
                        break;
                    case 0x17:  // EN25Q64 8MB 64Mb
                        subpart = "64";
                        flash_size = 8 << 20;
                        break;
                    case 0x18:  // EN25Q128 16MB 128Mb
                        subpart = "128";
                        flash_size = 16 << 20;
                        break;
                    default:
                        subpart = "";
                        flash_size = 4 << (20 + (capacity - 0x16));  // Guess
                        break;
                }
                flash_erase_size = SPI_FLASH_BLOCK_SIZE_64KB;
            } else {
                part = "?";
                subpart = "";
            }
            if (verbose) {
                mfgname = "EON";
                printf(" %s %s%s %uKB (%uKB erase)", mfgname, part, subpart,
                       flash_size >> 10, flash_erase_size >> 10);
            }
            break;
        }
        case SPI_FLASH_MFG_ATMEL: {
            /* Atmel */
            uint8_t family  = (flash_id >> 13) & 0x3;
            uint8_t density = (flash_id >> 8) & 0x1f;

            if (family == 0x2) {
                part = "AT25DF"; // Also AT26DF (legacy part)
                switch (density) {
                    case 3:  // AT25DF021 256KB 2Mb
                        subpart = "021";
                        break;
                    case 4:  // AT25DF041 512KB 4Mb
                        subpart = "041";
                        break;
                    case 5:  // AT25DF081 1MB 8Mb
                        subpart = "081";
                        break;
                    case 6:  // AT25DF161 2MB 16Mb
                        subpart = "161";
                        break;
                    case 7:  // AT25DF321 4MB 32Mb
                        subpart = "321";
                        break;
                    case 8:  // AT25DF641 8MB 64Mb
                        subpart = "641";
                        break;
                    default:
                        subpart = "";
                        break;
                }
                flash_size = 1 << (density + 15);
                flash_erase_size = SPI_FLASH_BLOCK_SIZE_64KB;
            } else {
                part = "?";
                subpart = "";
            }
            if (verbose) {
                mfgname = "Atmel";
                printf(" %s %s%s %uKB (%uKB erase)", mfgname, part, subpart,
                       flash_size >> 10, flash_erase_size >> 10);
            }
            break;
        }
        case SPI_FLASH_MFG_EVERSPIN:
            if (flash_id == EVERSPIN_MR25H40) {
                flash_size = 512 << 10;  // 512 KB
                if (verbose) {
                    printf("Everspin MR25H40CDC %uKB (%uKB erase)",
                           flash_size >> 10, flash_erase_size >> 10);
                }
            }
            break;
        case SPI_FLASH_MFG_MICROCHIP:
            if (flash_id == MICROCHIP_25AA512) {
                flash_size = 64 << 10;  // 64 KB
                flash_erase_size = 0;   // No erase
                if (verbose) {
                    printf("Microchip 25AA512 %uKB", flash_size >> 10);
                }
            }
            break;
        case SPI_FLASH_MFG_MICRON: {
            /* Micron */
            uint8_t family   = (uint8_t) (flash_id >> 8);
            uint8_t capacity = (uint8_t) (flash_id >> 16);

            if ((family == 0xba) || (family == 0xbb)) {
                part = "N25Q";
                switch (capacity) {
                    case 0x16:  // N25Q32 4MB 32Mb
                        subpart = "32";
                        flash_size = 4 << 20;
                        break;
                    case 0x17:  // N25Q64 8MB 64Mb
                        subpart = "64";
                        flash_size = 8 << 20;
                        break;
                    case 0x18:  // N25Q128 16MB 128Mb
                        subpart = "128";
                        flash_size = 16 << 20;
                        break;
                    case 0x19:  // N25Q256 32MB 256Mb
                        subpart = "256";
                        flash_size = 32 << 20;
                        break;
                    case 0x20:  // N25Q512 64MB 512Mb
                        subpart = "512";
                        flash_size = 64 << 20;
                        break;
                    case 0x21:  // N25Q1G 128MB 1Gb
                        subpart = "1G";
                        flash_size = 128 << 20;
                        break;
                    default:
                        subpart = "";
                        break;
                }
                flash_erase_size = SPI_FLASH_BLOCK_SIZE_64KB;
            } else if (family == 0x80) {
                part = "M25PE";
                switch (capacity) {
                    case 0x11:
                        subpart = "10";
                        flash_size = 128 << 10;
                        break;
                    case 0x12:
                        subpart = "20";
                        flash_size = 256 << 10;
                        break;
                    default:
                        subpart = "";
                        break;
                }
                flash_erase_size = SPI_FLASH_BLOCK_SIZE_64KB;
            } else {
                part = "?";
                subpart = "";
            }
            if (verbose) {
                mfgname = "Micron";
                printf(" %s %s%s %uKB (%uKB erase)", mfgname, part, subpart,
                       flash_size >> 10, flash_erase_size >> 10);
            }
            break;
        }
        case SPI_FLASH_MFG_ONSEMI:
            if (flash_id == ONSEMI_CAT25256) {
                flash_size = 32 << 10;  // 32 KB
                flash_erase_size = 0;   // No erase
                if (verbose) {
                    printf("On Semiconductor %uKB", flash_size >> 10);
                }
            }
            break;
        case SPI_FLASH_MFG_WINBOND: {
            /* Winbond - typical Flash ID: 001660ef */
            uint8_t mem_type = (uint8_t) (flash_id >> 8);
            uint8_t capacity = (uint8_t) (flash_id >> 16);

            if (mem_type == 0x30) {
                part = "W25X";
                switch (capacity) {
                    case 0x12:  // W25X20 256KB 2Mb
                        subpart = "20";
                        flash_size = 256 << 10;
                        break;
                    case 0x13:  // W25X40 512KB 4Mb
                        subpart = "40";
                        flash_size = 512 << 10;
                        break;
                    default:
                        subpart = "?";
                        break;
                }
            } else if ((mem_type == 0x60) || (mem_type == 0x40)) {
                part = "W25Q";
                switch (capacity) {
                    case 0x16:  // W25Q32DW 4MB 32Mb
                        subpart = "32";
                        flash_size = 4 << 20;
                        break;
                    case 0x17:  // W25Q64DW 8MB 64Mb
                        subpart = "64";
                        flash_size = 8 << 20;
                        break;
                    case 0x18:  // W25Q128BV 16MB 128Mb
                        subpart = "128";
                        flash_size = 16 << 20;
                        break;
                    case 0x19:  // W25Q256 32MB 256Mb
                        subpart = "256";
                        flash_size = 32 << 20;
                        break;
                    default:
                        subpart = "?";
                        break;
                }
                flash_erase_size = SPI_FLASH_BLOCK_SIZE_64KB;
            } else {
                part = "?";
                subpart = "";
            }
            if (verbose) {
                mfgname = "Winbond";
                printf(" %s %s%s %uKB (%uKB erase)", mfgname, part, subpart,
                       flash_size >> 10, flash_erase_size >> 10);
            }
            break;
        }
        default:
            printf(" Unknown flash id: %08x", (uint) flash_id);
            break;
        case SPI_FLASH_MFG_NONE:
            printf(" Flash did not respond");
            break;
    }
    if (verbose)
        printf("\n");

    if (flash_size == 0) {
        flash_ids[chipsel] = 0;
        rc = RC_NO_DATA;
    }

    flash_sizes[chipsel] = flash_size;

id_failure:
    return (rc);
}

/*
 * spi_flash_erase_size() determines the erase size of the SPI chip at a
 *                        particular address. The maximum erase size is
 *                        determined by spi_flash_identify() based on the
 *                        specific chip. However, specific regions of
 *                        certain chips may need smaller erase sizes due
 *                        to the application.
 *
 * @param [in]  chip - SPI flash chip select number.
 * @param [in]  addr - The address to test.
 *
 * @return      The erase block size at the address to test.
 */
static uint32_t
spi_flash_erase_size(uint chip, uint32_t addr)
{
    (void) chip;
    (void) addr;
    return (SPI_FLASH_BLOCK_SIZE);
}

/*
 * spi_flash_is_at_block_boundary() determines if the specified address is at
 *                                  an SPI flash block boundary.
 *
 * @param [in]  chip - SPI flash chip select number.
 * @param [in]  addr - The address to test.
 *
 * @return      TRUE  - The specified address is at the start of an SPI flash
 *                      erase block.
 * @return      FALSE - The specified address is not at the start of an SPI
 *                      flash erase block.
 */
static bool_t
spi_flash_is_at_block_boundary(uint chip, uint32_t addr)
{
    return ((addr & (spi_flash_erase_size(chip, addr) - 1)) == 0);
}

/*
 * spi_flash_block_erase() sends the erase command to the SPI flash block at the
 *                         specified base address.
 *
 * @param [in]  chip - SPI flash chip select number.
 * @param [in]  addr - The base address to erase.
 *
 * @return      RC_SUCCESS   - Successful operation.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_block_erase(uint chip, uint32_t addr)
{
    rc_t rc;
    uint abytes = (uint8_t) (chip >> 8);
    uint chipsel = (uint8_t) chip;
    switch (spi_flash_erase_size(chip, addr)) {
        case SPI_FLASH_BLOCK_SIZE_64KB:
            addr |= (AT25_BLOCK_ERASE_64KB << (abytes * 8));
            break;
        case SPI_FLASH_BLOCK_SIZE_32KB:
            addr |= (AT25_BLOCK_ERASE_32KB << (abytes * 8));
            break;
        case SPI_FLASH_BLOCK_SIZE_4KB:
            addr |= (AT25_BLOCK_ERASE_4KB << (abytes * 8));
            break;
        default:
            return (RC_SUCCESS);
    }
    rc = spi_chip_write(chipsel, addr, abytes + 1, 0, NULL);
    return (rc);
}

/*
 * spi_flash_erase_block() erases the SPI flash block at the specified base
 *                         address. This function is safe to call with any
 *                         arbitrary flash address. It will only erase if the
 *                         specified address is at the start of an erase block.
 *
 * @param [in]  chip - SPI flash chip select number.
 * @param [in]  addr - The base address to erase.
 *
 * @return      RC_SUCCESS   - Successful operation.
 * @return      RC_TIMEOUT   - A hardware timeout occurred.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
static rc_t
spi_flash_erase_block(uint chip, uint32_t addr)
{
    rc_t rc;
    uint chipsel = (uint8_t) chip;

    if (flash_ids[chipsel] == MICROCHIP_25AA512 ||
        flash_ids[chipsel] == ONSEMI_CAT25256)
        return (RC_SUCCESS);

    if (spi_flash_is_at_block_boundary(chip, addr) == FALSE)
        return (RC_SUCCESS);

    /* First check if flash is still busy (from a previous command) */
    rc = spi_flash_wait_not_busy(chip, FLASH_TIMEOUT);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Write Enable */
    rc = spi_flash_write_enable(chip);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Block erase */
    rc = spi_flash_block_erase(chip, addr);
    if (rc != RC_SUCCESS)
        return (rc);

    return (RC_SUCCESS);
}

/*
 * spi_flash_write_page() writes up to one page of bytes to the specified
 *                        SPI Flash chip. The caller is responsible for
 *                        ensuring the address and length do not cross a
 *                        page boundary. The caller is also responsible for
 *                        ensuring data may be written at the specified address
 *                        (bytes to be written were previously erased).
 *
 * @param [in]  chip - SPI flash chip select number.
 * @param [in]  addr - The address to write.
 * @param [in]  len  - The number of bytes to write.
 * @param [in]  data - The data to write.
 *
 * @return      RC_SUCCESS - Successful operation.
 * @return      RC_TIMEOUT - The erase took too long to complete.
 */
static rc_t
spi_flash_write_page(uint chip, uint32_t addr, uint len, const void *data)
{
    rc_t rc;
    uint abytes = (uint8_t) (chip >> 8);
    uint chipsel = (uint8_t) chip;
    uint64_t addr_op = addr;
    uint8_t command = AT25_WRITE_PAGE_BUFFER;

    /* First check if flash is still busy (from a previous command) */
    rc = spi_flash_wait_not_busy(chip, FLASH_TIMEOUT);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Write Enable */
    rc = spi_flash_write_enable(chip);
    if (rc != RC_SUCCESS)
        return (rc);

    /* Micron SPI Flash uses a different write command with 4-byte addressing */
    if (abytes == 4 && spi_flash_mfg(chipsel) == SPI_FLASH_MFG_MICRON)
        command = MICRON_4B_WRITE_PAGE;

    /* Write operation -- Load page buffer and program */
    addr_op |= ((uint64_t)command << (abytes * 8));
    rc = spi_chip_write(chipsel, addr_op, abytes + 1, len, data);
    if (rc != RC_SUCCESS)
        return (rc);

    return (RC_SUCCESS);
}

/*
 * spi_flash_erase() erases blocks of data from an SPI Flash device.
 *
 * @param [in]  chip - SPI chip select number.
 * @param [in]  addr - The address to erase.
 * @param [in]  mode - The number of bytes to erase.
 *
 * @return      RC_SUCCESS   - The erase succeeded.
 * @return      RC_TIMEOUT   - The SPI master took too long to send data.
 * @return      RC_BAD_PARAM - Invalid address or SPI chip select specified.
 */
rc_t
spi_flash_erase(uint chip, uint32_t addr, uint mode)
{
    rc_t rc;
    uint32_t erase_size;
    uint chipsel = (uint8_t) chip;
    uint retry = 0;

    /* Acquire ownership of the SPI device */
    if ((rc = spi_chip_own(chipsel, TRUE)) != RC_SUCCESS)
        return (rc);

    chip = get_chip_address_mode(chip, addr);

    rc = spi_flash_identify(chip, 0);
    if (rc != RC_SUCCESS)
        goto release;
    if (addr + mode > flash_sizes[chipsel]) {
        rc = RC_BAD_PARAM;
        goto release;
    }

    rc = spi_flash_unprotect(chip);
    if (rc != RC_SUCCESS)
        goto release;

    if (mode == 0)
        mode = flash_sizes[chipsel] - addr;
    erase_size = spi_flash_erase_size(chip, addr);
    while (mode > 0) {
        uint32_t next = P2END(addr, erase_size);
        uint32_t len = next - addr;
        /*
         * Re-lock SPI access between block erasures to allow USB interrupts to
         * be serviced periodically during long-duration operations.
         */
        (void) spi_chip_own(chipsel, FALSE);
        if ((rc = spi_chip_own(chipsel, TRUE)) != RC_SUCCESS)
            return (rc);
        rc = spi_flash_erase_block(chip, addr);
        /* Retry up to three times */
        if (rc == RC_TIMEOUT && retry++ < 3)
            continue;
        if (rc != RC_SUCCESS || len > mode)
            break;
        retry = 0;
        mode -= len;
        addr  = next;
    }

release:
    /* Release ownership of the specified device */
    (void) spi_chip_own(chipsel, FALSE);

    return (rc);
}

/*
 * spi_flash_write() writes a block of data to an SPI Flash device.
 *
 * @param [in]  chip      - SPI chip select number.
 * @param [in]  addr      - The address to write.
 * @param [in]  mode      - The number of bytes to write.
 * @param [in]  data      - The data to write to the device.
 * @param [in]  autoerase - Flag to automatically erase the target block when
 *                          writing to the beginning of its first page.
 *
 * @return      RC_SUCCESS   - The values were written.
 * @return      RC_TIMEOUT   - The SPI master took too long to send data.
 * @return      RC_BAD_PARAM - Invalid address or SPI chip select specified.
 */
rc_t
spi_flash_write(uint chip, uint32_t addr, uint mode, const uint8_t *data,
                bool_t autoerase)
{
    rc_t rc;
    uint len;
    uint page_size;
    uint chipsel = (uint8_t) chip;
    const uint32_t chip_id = spi_flash_identify_no_probe(chipsel);

    /* Acquire ownership of the SPI device */
    if ((rc = spi_chip_own(chipsel, TRUE)) != RC_SUCCESS)
        return (rc);

    chip = get_chip_address_mode(chip, addr);

    switch (chip_id) {
        case MICROCHIP_25AA512:
            page_size = MC25AA_PAGE_BUFFER_SIZE;
            break;
        case ONSEMI_CAT25256:
            page_size = CAT25_PAGE_BUFFER_SIZE;
            break;
        default:
            page_size = AT25_PAGE_BUFFER_SIZE;
            break;
    }

    rc = spi_flash_identify(chip, 0);
    if (rc != RC_SUCCESS)
        goto release;
    if (addr + mode > flash_sizes[chipsel]) {
        rc = RC_BAD_PARAM;
        goto release;
    }

    rc = spi_flash_unprotect(chip);
    if (rc != RC_SUCCESS)
        goto release;

    while (mode > 0) {
        /* Write must be segmented at page buffer boundaries */
        uint32_t next_page    = (addr + page_size) & ~(page_size - 1);
        uint32_t to_next_page = next_page - addr;

        if (mode > to_next_page)
            len = to_next_page;
        else
            len = mode;

        if (autoerase) {
            /* spi_flash_erase_block() only erases at block boundaries */
            rc = spi_flash_erase_block(chip, addr);
            if (rc != RC_SUCCESS)
                goto release;
        }

        rc = spi_flash_write_page(chip, addr, len, data);
        if (rc != RC_SUCCESS)
            goto release;

        mode -= len;
        addr += len;
        data += len;
    }

release:
    /* Release ownership of the specified device */
    (void) spi_chip_own(chipsel, FALSE);

    return (rc);
}

/*
 * spi_flash_read() reads a block of data from an SPI Flash device.
 *
 * @param [in]  chip - SPI chip select number plus other information.
 *                     Bits 0-7:  chip select.
 *                     Bits 8-15: bytes per dev address (0=unknown)
 * @param [in]  addr - The address to read.
 * @param [in]  mode - The number of bytes to read.
 * @param [in]  data - The data receive buffer.
 *
 * @return      RC_SUCCESS   - The values were read.
 * @return      RC_TIMEOUT   - The SPI master took too long to receive data.
 * @return      RC_BAD_PARAM - Invalid address or SPI chip select specified.
 */
rc_t
spi_flash_read(uint chip, uint32_t addr, uint mode, uint8_t *data)
{
    rc_t rc;
    uint chipsel = (uint8_t) chip;
    uint abytes;
    uint64_t addr_op = addr;
    uint8_t command = AT25_READ_DATA;

    /* Acquire ownership of the SPI device */
    if ((rc = spi_chip_own(chipsel, TRUE)) != RC_SUCCESS)
        return (rc);

    chip = get_chip_address_mode(chip, addr);

    abytes = (uint8_t) (chip >> 8);

    /* Micron SPI Flash uses a different read command with 4-byte addressing */
    if (abytes == 4 && spi_flash_mfg(chipsel) == SPI_FLASH_MFG_MICRON)
        command = MICRON_4B_READ_DATA;

    rc = spi_flash_identify(chip, 0);
    if (rc != RC_SUCCESS)
        goto release;
    if ((addr + mode > flash_sizes[chipsel]) && (flash_sizes[chipsel] > 0))  {
        rc = RC_BAD_PARAM;
        goto release;
    }

    /* Wait in case last operation was a write or erase */
    rc = spi_flash_wait_not_busy(chip, FLASH_TIMEOUT);
    if (rc != RC_SUCCESS)
        goto release;

    /* Read operation */
    addr_op |= ((uint64_t)command << (abytes * 8));
    rc = spi_chip_read(chipsel, addr_op, abytes + 1, mode, data);

release:
    /* Release ownership of the specified device */
    (void) spi_chip_own(chipsel, FALSE);

    return (rc);
}

/*
 * spi_flash_id() probes and reports the flash id and capacity.
 *
 * @param [in]  chip - SPI flash chip select number.
 *
 * @return      RC_SUCCESS   - The values were reported.
 * @return      RC_TIMEOUT   - The SPI master took too long to receive data.
 * @return      RC_BAD_PARAM - Invalid address or SPI chip select specified.
 */
rc_t
spi_flash_id(uint chip)
{
    rc_t rc;
    uint chipsel = (uint8_t) chip;

    if (chip == 0xff) {
        for (chip = 0; chip < FLASH_MAX_CHIPSEL; chip++) {
            rc = spi_flash_id(chip);
            if (rc != RC_SUCCESS)
                break;
        }
        return (rc);
    }

    /* Acquire ownership of the SPI device */
    if ((rc = spi_chip_own(chipsel, TRUE)) != RC_SUCCESS)
        return (rc);

    rc = spi_flash_identify(chip, 1);

    /* Release ownership of the specified device */
    (void) spi_chip_own(chipsel, FALSE);

    return (rc);
}

static int
getchar_wait(uint pos)
{
    int      ch;
    uint64_t timeout = timer_tick_plus_msec(200);

    while ((ch = getchar()) == -1)
        if (timer_tick_has_elapsed(timeout))
            break;

    return (ch);
}

static int
check_crc(uint32_t crc, uint spos, uint epos, bool send_rc)
{
    int      ch;
    size_t   pos;
    uint32_t compcrc;

    for (pos = 0; pos < sizeof (compcrc); pos++) {
        ch = getchar_wait(200);
        if (ch == -1) {
            printf("Receive timeout waiting for CRC %08lx at 0x%x\n",
                   crc, epos);
            return (RC_TIMEOUT);
        }
        ((uint8_t *)&compcrc)[pos] = ch;
    }
    if (crc != compcrc) {
        printf("Received CRC %08lx doesn't match %08lx at 0x%x-0x%x\n",
               compcrc, crc, spos, epos);
        return (1);
    }
    return (0);
}

static int
check_rc(uint pos)
{
    int ch = getchar_wait(200);
    if (ch == -1) {
        printf("Receive timeout waiting for rc at 0x%x\n", pos);
        return (RC_TIMEOUT);
    }
    if (ch != 0) {
        printf("Remote sent error %d at 0x%x\n", ch, pos);
        return (RC_FAILURE);
    }
    return (RC_SUCCESS);
}

/*
 * spi_flash_read_binary() reads data from an EEPROM and writes it to the host.
 *
 * Every 256 bytes, a rolling CRC value is expected back from the host.
 */
rc_t
spi_flash_read_binary(uint chip, uint32_t addr, uint32_t len)
{
    rc_t     rc;
    __attribute__((aligned(16)))
    uint8_t  buf[256];
    uint32_t crc = 0;
    uint     crc_next = DATA_CRC_INTERVAL;
    uint32_t cap_pos[4];
    uint     cap_count = 0;
    uint     cap_prod  = 0;  // producer
    uint     cap_cons  = 0;  // consumer
    uint     pos = 0;

    while (len > 0) {
        uint32_t tlen = sizeof (buf);
        if (tlen > len)
            tlen = len;
        if (tlen > crc_next)
            tlen = crc_next;
        rc = spi_flash_read(chip, addr, tlen, buf);
        if (puts_binary(&rc, 1)) {
            printf("Status send timeout at %lx\n", addr + pos);
            return (RC_TIMEOUT);
        }
        if (rc != RC_SUCCESS)
            return (rc);
        if (puts_binary(buf, tlen)) {
            printf("Data send timeout at %lx\n", addr + pos);
            return (RC_TIMEOUT);
        }

        crc = crc32(crc, buf, tlen);
        crc_next -= tlen;
        addr     += tlen;
        len      -= tlen;
        pos      += tlen;

        if (cap_count >= ARRAY_SIZE(cap_pos)) {
            /* Verify received RC */
            cap_count--;
            if (check_rc(cap_pos[cap_cons]))
                return (RC_FAILURE);
            if (++cap_cons >= ARRAY_SIZE(cap_pos))
                cap_cons = 0;
        }

        if (crc_next == 0) {
            /* Send and record the current CRC value */
            if (puts_binary(&crc, sizeof (crc))) {
                printf("Data send CRC timeout at %lx\n", addr + pos);
                return (RC_TIMEOUT);
            }
            cap_pos[cap_prod] = pos;
            if (++cap_prod >= ARRAY_SIZE(cap_pos))
                cap_prod = 0;
            cap_count++;
            crc_next = DATA_CRC_INTERVAL;
        }
        led_poll();  // Blink power LED if it needs to be blinked
    }
    if (crc_next != DATA_CRC_INTERVAL) {
        /* Send CRC for last partial segment */
        if (puts_binary(&crc, sizeof (crc)))
            return (RC_TIMEOUT);
    }

    /* Verify trailing CRC packets */
    cap_prod += ARRAY_SIZE(cap_pos) - cap_count;
    if (cap_prod >= ARRAY_SIZE(cap_pos))
        cap_prod -= ARRAY_SIZE(cap_pos);

    while (cap_count-- > 0) {
        if (check_rc(cap_pos[cap_cons]))
            return (1);
        if (++cap_cons >= ARRAY_SIZE(cap_pos))
            cap_cons = 0;
    }

    if (crc_next != DATA_CRC_INTERVAL) {
        /* Verify CRC for last partial segment */
        if (check_rc(pos))
            return (RC_FAILURE);
    }
    return (RC_SUCCESS);
}

/*
 * spi_flash_write_binary() takes binary input from an application via the
 *                     serial console and writes that to the EEPROM.
 *
 *  Every 256 bytes, a rolling 8-bit CRC value is sent back to the host.
 *  This is so the host knows that the data was received correctly.
 *  Incorrectly received data will still be written to the EEPROM.
 */
rc_t
spi_flash_write_binary(uint chip, uint32_t addr, uint32_t len)
{
    uint8_t  buf[128];
    int      ch;
    rc_t     rc;
    uint32_t crc = 0;
    uint32_t saddr = addr;
    uint     crc_next = DATA_CRC_INTERVAL;

    while (len > 0) {
        uint32_t tlen    = len;
        uint32_t rem     = addr & (sizeof (buf) - 1);
        uint64_t timeout = timer_tick_plus_msec(1000);
        uint32_t pos;
        uint8_t *ptr = buf;

        if (tlen > sizeof (buf) - rem)
            tlen = sizeof (buf) - rem;

        for (pos = 0; pos < tlen; pos++) {
            while ((ch = getchar()) == -1)
                if (timer_tick_has_elapsed(timeout)) {
                    printf("Data receive timeout at %lx\n", addr + pos);
                    rc = RC_TIMEOUT;
                    goto fail;
                }
            timeout = timer_tick_plus_msec(1000);
            *(ptr++) = ch;
            crc = crc32(crc, ptr - 1, 1);
            if (--crc_next == 0) {
                if (check_crc(crc, saddr, addr + pos + 1, false)) {
                    rc = RC_FAILURE;
                    goto fail;
                }
                rc = RC_SUCCESS;
                if (puts_binary(&rc, 1)) {
                    rc = RC_TIMEOUT;
                    goto fail;
                }
                crc_next = DATA_CRC_INTERVAL;
                saddr = addr + pos + 1;
            }
        }
        rc = spi_flash_write(chip, addr, tlen, buf, 1);
        if (rc != RC_SUCCESS) {
fail:
            (void) puts_binary(&rc, 1);  // Inform remote side
            timeout = timer_tick_plus_msec(2000);
            while (!timer_tick_has_elapsed(timeout))
                (void) getchar();  // Discard input
            return (rc);
        }
        addr += tlen;
        len  -= tlen;
        led_poll();  // Blink power LED if it needs to be blinked
    }
    if (crc_next != DATA_CRC_INTERVAL) {
        if (check_crc(crc, saddr, addr, false)) {
            rc = RC_FAILURE;
            goto fail;
        }
    }
    rc = RC_SUCCESS;
    if (puts_binary(&rc, 1)) {
        rc = RC_TIMEOUT;
        goto fail;
    }
    return (RC_SUCCESS);
}
