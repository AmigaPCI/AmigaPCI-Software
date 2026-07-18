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

#ifndef _SPI_FLASH_H
#define _SPI_FLASH_H

rc_t spi_flash_id(uint chip);
rc_t spi_flash_read(uint chip, uint32_t addr, uint mode, uint8_t *data);
rc_t spi_flash_write(uint chip, uint32_t addr, uint mode,
                     const uint8_t *data, bool_t autoerase);
rc_t spi_flash_erase(uint chip, uint32_t addr, uint mode);
rc_t spi_flash_read_binary(uint chip, uint32_t addr, uint32_t len);
rc_t spi_flash_write_binary(uint chip, uint32_t addr, uint32_t len);

#endif /* _SPI_FLASH_H */
