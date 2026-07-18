/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * SPI support.
 */

#ifndef _SPI_H
#define _SPI_H

rc_t spi_chip_own(uint chip, bool_t lock);
rc_t spi_chip_read(uint chip, uint64_t addr, uint alen, uint dlen, void *data);
rc_t spi_chip_write(uint chip, uint64_t addr, uint alen, uint dlen, const void *data);
rc_t spi_chip_rw(uint chip, uint width, uint send_len, uint recv_len, uint recv_start, void *send_data, void *recv_data);
void spi_list_chipsel(void);
void spi_init(void);

#endif /* _SPI_H */
