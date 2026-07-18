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

#include <stdint.h>
#include <stdlib.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <string.h>
#include "main.h"
#include "config.h"
#include "cmdline.h"
#include "gpio.h"
#include "spi.h"
#include "main.h"
#include "printf.h"
#include "timer.h"
#include "config.h"
#include "utils.h"

#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define SPI_READ_TIMEOUT  1     // timeout in milliseconds
#define SPI_WRITE_TIMEOUT 1     // timeout in milliseconds

/* STM32F2 + STM32F4 */
#define IO_BASE              0x40000000
#define BND_IO_BASE          0x42000000
#define GPIO_IDR_OFFSET      0x10  // Input Data Register offset
#define GPIO_ODR_OFFSET      0x14  // Output Data Register offset
#define BND_IO(byte, bit)    (BND_IO_BASE + ((byte) - IO_BASE) * 32 + (bit) * 4)
#define BND_ODR_TO_IDR(addr) ((addr) + (GPIO_IDR_OFFSET - GPIO_ODR_OFFSET) * 32)

#define SPI_HW_CORE SPI1

#define _DMABUF static __attribute__((section(".dmabuf,\"aw\",%nobits@")))
/* SPI DMA buffer */
__attribute__((aligned(4))) _DMABUF
uint8_t spi_dma_buf[256];
static const uint8_t cfg_spi_dma = 0;

/* Bit bang bit band register addresses */
static uint               spi_sw_bitbang;  // 0=HW SPI, 1=SW SPI
static volatile uint32_t *spi_cs;
static volatile uint32_t *spi_clk;
static volatile uint32_t *spi_mosi;
static volatile uint32_t *spi_miso;

/* SPI pin GPIOs */
static uint32_t spi_cs_gpio;
static uint32_t spi_clk_gpio;
static uint32_t spi_mosi_gpio;
static uint32_t spi_miso_gpio;
static uint16_t spi_cs_pin;
static uint16_t spi_clk_pin;
static uint16_t spi_mosi_pin;
static uint16_t spi_miso_pin;

/*
 * spi_get_pins() set up the GPIOs which will be used for the specified
 *                chip select.
 *
 * @param [in]  chip         - SPI chip select number.
 *
 * @return      RC_SUCCESS   - SPI channel is available.
 * @return      RC_BAD_PARAM - SPI channel does not exist.
 */
static rc_t
spi_get_pins(uint chip)
{
    switch (config.board_type) {
        case BOARD_TYPE_AMIGAPCI:
            switch (chip) {
                case 0:
                    spi_clk_gpio  = GPIOE; spi_clk_pin  = GPIO0; // PE0  _SCLKA
                    spi_mosi_gpio = GPIOE; spi_mosi_pin = GPIO1; // PE1  _SDIA
                    spi_miso_gpio = GPIOE; spi_miso_pin = GPIO2; // PE2  _SDOA
                    spi_cs_gpio   = GPIOE; spi_cs_pin   = GPIO3; // PE3 _U110EN
                    break;
                case 1:
                    spi_clk_gpio  = GPIOE; spi_clk_pin  = GPIO0; // PE0  _SCLKA
                    spi_mosi_gpio = GPIOE; spi_mosi_pin = GPIO1; // PE1  _SDIA
                    spi_miso_gpio = GPIOE; spi_miso_pin = GPIO2; // PE2  _SDOA
                    spi_cs_gpio   = GPIOE; spi_cs_pin   = GPIO4; // PE4 _U109EN
                    break;
                case 2:
                    spi_clk_gpio  = GPIOE; spi_clk_pin  = GPIO0; // PE0  _SCLKA
                    spi_mosi_gpio = GPIOE; spi_mosi_pin = GPIO1; // PE1  _SDIA
                    spi_miso_gpio = GPIOE; spi_miso_pin = GPIO2; // PE2  _SDOA
                    spi_cs_gpio   = GPIOE; spi_cs_pin   = GPIO5; // PE5 _U712EN
                    break;
                case 3:
                    spi_clk_gpio  = GPIOE; spi_clk_pin  = GPIO0; // PE0  _SCLKA
                    spi_mosi_gpio = GPIOE; spi_mosi_pin = GPIO1; // PE1  _SDIA
                    spi_miso_gpio = GPIOE; spi_miso_pin = GPIO2; // PE2  _SDOA
                    spi_cs_gpio   = GPIOE; spi_cs_pin   = GPIO6; // PE6 _U409EN
                    break;
                case 4:
                    spi_clk_gpio  = GPIOE; spi_clk_pin  = GPIO0; // PE0  _SCLKB
                    spi_mosi_gpio = GPIOE; spi_mosi_pin = GPIO1;  // PE1  _SDIB
                    spi_miso_gpio = GPIOE; spi_miso_pin = GPIO12; // PE12 _SDOB
                    spi_cs_gpio   = GPIOE; spi_cs_pin  = GPIO15; // PE15 _SPIAEN
                    break;
                case 5:
                    spi_clk_gpio  = GPIOE; spi_clk_pin  = GPIO0;  // PE0  _SCLKB
                    spi_mosi_gpio = GPIOE; spi_mosi_pin = GPIO1;  // PE1  _SDIB
                    spi_miso_gpio = GPIOE; spi_miso_pin = GPIO12; // PE12 _SDOB
                    spi_cs_gpio   = GPIOE; spi_cs_pin  = GPIO14; // PE14 _SPIBEN
                    break;
                case 6:
                    spi_clk_gpio  = GPIOE; spi_clk_pin  = GPIO0;  // PE0  _SCLKB
                    spi_mosi_gpio = GPIOE; spi_mosi_pin = GPIO1;  // PE1  _SDIB
                    spi_miso_gpio = GPIOE; spi_miso_pin = GPIO12; // PE12 _SDOB
                    spi_cs_gpio   = GPIOE; spi_cs_pin  = GPIO13; // PE13 _SPICEN
                    break;
                default:
                    return (RC_BAD_PARAM);
            }
            break;
        case BOARD_TYPE_APCIDEV:
            /* SPI not supported */
            return (RC_BAD_PARAM);
        case BOARD_TYPE_KEYJAM:
            switch (chip) {
                case 0:
                    spi_clk_gpio  = GPIOA; spi_clk_pin  = GPIO5; // PA5 SPI_CLK
                    spi_mosi_gpio = GPIOA; spi_mosi_pin = GPIO7; // PA7 SPI_MOSI
                    spi_miso_gpio = GPIOA; spi_miso_pin = GPIO6; // PA6 SPI_MISO
                    spi_cs_gpio   = GPIOA; spi_cs_pin   = GPIO4; // PA4 SPI_CS1
                    break;
                case 1:
                    spi_clk_gpio  = GPIOA; spi_clk_pin  = GPIO5; // PA5 SPI_CLK
                    spi_mosi_gpio = GPIOA; spi_mosi_pin = GPIO7; // PA7 SPI_MOSI
                    spi_miso_gpio = GPIOA; spi_miso_pin = GPIO6; // PA6 SPI_MISO
                    spi_cs_gpio   = GPIOA; spi_cs_pin   = GPIO3; // PA3 SPI_CS2
                    break;
                default:
                    return (RC_BAD_PARAM);
            }
            break;
    }

    spi_cs  = VADDR32(BND_IO(spi_cs_gpio  + GPIO_ODR_OFFSET,
                             low_bit(spi_cs_pin)));
    spi_clk = VADDR32(BND_IO(spi_clk_gpio + GPIO_ODR_OFFSET,
                             low_bit(spi_clk_pin)));
    spi_mosi = VADDR32(BND_IO(spi_mosi_gpio + GPIO_ODR_OFFSET,
                             low_bit(spi_mosi_pin)));
    spi_miso = VADDR32(BND_IO(spi_miso_gpio + GPIO_IDR_OFFSET,
                             low_bit(spi_miso_pin)));
    return (RC_SUCCESS);
}

/*
 * spi_chipsel() asserts or de-asserts the chip select for the specified
 *               SPI chip number.
 *
 * @param [in]  chip      - SPI chip select number.
 * @param [in]  do_select - TRUE = assert and FALSE = deassert the chip select.
 */
static void
spi_chipsel(uint chip, bool_t do_select)
{
    /* If the chip number is 0xff, chip select is controlled manually */
    if (chip == 0xff)
        return;

    if (do_select == TRUE)
        *spi_cs = 0;
    else
        *spi_cs = 1;
}

/*
 * spi_clear_mode_fault() clears an SPI mode fault if one is asserted.
 *
 * @param [in]  spi - The SPI master base address.
 */
static void
spi_clear_mode_fault(uint32_t spi)
{
    if (SPI_SR(spi) & SPI_SR_MODF)
        SPI_CR1(spi) = SPI_CR1(spi);
}

/*
 * spi_dma_init() initializes SPI DMA.
 *
 * @param [in]  spi - The SPI master base address.
 */
static void
spi_dma_init(uint32_t spi)
{
    uint32_t dma;
    uint8_t  channel;
    uint8_t  stream;

    /*
     * Note:
     *   SPI1_TX uses DMA2 stream 5 (ch 3).
     *   SPI1_RX uses DMA2 stream 2 (ch 3).
     *
     * Refer to ST-Micro datasheet Table 43. DMA2 request mapping for
     * valid mappings of DMA streams and channels to external peripherals.
     */
    dma = DMA2;
    stream = 5;
    channel = 3;

    dma_stream_reset(dma, stream);
    dma_set_peripheral_address(dma, stream, (uintptr_t) &SPI_DR(spi));
    dma_set_transfer_mode(dma, stream, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_channel_select(dma, stream, channel);
    dma_disable_peripheral_increment_mode(dma, stream);
    dma_enable_memory_increment_mode(dma, stream);
    dma_set_peripheral_size(dma, stream, DMA_SxCR_PSIZE_8BIT);
//  dma_disable_circular_mode(dma, stream);
    dma_set_priority(dma, stream, DMA_SxCR_PL_HIGH);
    dma_enable_direct_mode(dma, stream);
    dma_set_fifo_threshold(dma, stream, DMA_SxFCR_FTH_2_4_FULL);
    dma_set_memory_burst(dma, stream, DMA_SxCR_MBURST_SINGLE);
    dma_set_peripheral_burst(dma, stream, DMA_SxCR_PBURST_SINGLE);

    /* SPI1_RX */
    dma = DMA2;
    stream = 2;
    channel = 3;

    dma_stream_reset(dma, stream);
    dma_set_peripheral_address(dma, stream, (uintptr_t) &SPI_DR(spi));
    dma_set_transfer_mode(dma, stream, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_channel_select(dma, stream, channel);
    dma_disable_peripheral_increment_mode(dma, stream);
    dma_enable_memory_increment_mode(dma, stream);
    dma_set_peripheral_size(dma, stream, DMA_SxCR_PSIZE_8BIT);
//  dma_disable_circular_mode(dma, stream);
    dma_set_priority(dma, stream, DMA_SxCR_PL_HIGH);
    dma_enable_direct_mode(dma, stream);
    dma_set_fifo_threshold(dma, stream, DMA_SxFCR_FTH_2_4_FULL);
    dma_set_memory_burst(dma, stream, DMA_SxCR_MBURST_SINGLE);
    dma_set_peripheral_burst(dma, stream, DMA_SxCR_PBURST_SINGLE);
}

/*
 * spi_dma_rw() executes a SPI transaction using DMA.
 *
 * @param [in]  spi        - The SPI master base address.
 * @param [in]  width      - The byte width to use for transactions.
 *                           Currently only width of 1 is supported.
 * @param [in]  cmd_len    - Length of the command bytes to prefix on
 *                           the transaction.
 * @param [in]  send_len   - Length of the bytes to send.
 * @param [in]  recv_len   - Length of the bytes to receive.
 * @param [in]  recv_start - Offset from the start of the transaction
 *                           where we expect receive data to start.
 *                           Usually this would naturally be equal to cmd_len +
 *                           send_len, but we allow recv data to start earlier
 *                           or later than that.
 * @param [in]  cmd_data   - Command data buffer. May be NULL if there is no
 *                           command. This data is sent in big endian format.
 * @param [in]  send_data  - Send data buffer. May be NULL if there is no data.
 * @param [out] recv_data  - Receive data buffer. May be NULL if there is no
 *                           data to receive.
 *
 * @return      RC_SUCCESS   - SPI transaction was successful.
 * @return      RC_BAD_PARAM - Bad length parameters. Note that the total
 *                             transaction length cannot exceed the size
 *                             of the SPI DMA buffer.
 * @return      RC_BUSY      - SPI transaction was blocked by lack of space
 *                             in the TX direction.
 * @return      RC_TIMEOUT   - SPI transaction timed out.
 */
static rc_t
spi_dma_rw(uint32_t spi, uint width, uint16_t cmd_len, uint16_t send_len,
           uint16_t recv_len, uint16_t recv_start, const void *cmd_data,
           const void *send_data, void *recv_data)
{
    rc_t     rc = RC_SUCCESS;
    uint32_t dma;
    uint8_t  stream;
    uint16_t len = MAX(cmd_len + send_len + recv_len, recv_start + recv_len);
    uint8_t *buf = spi_dma_buf;
    uint64_t timeout;

    /*
     *       |-------------- SPI TRANSACTION -------------|
     *        <-cmd_len----> <-send_len---> <-recv_len--->
     * MOSI  |  cmd_data... | send_data... |              |
     * MISO  |                             | recv_data... |
     *                                     ^
     *                                     |
     *                                 recv_start
     *
     * Send the cmd_data and send_data to the slave, and get recv_data
     * back. In the common case cmd_len + send_len = recv_start, but one
     * can also handle where the recv data starts before or after the
     * last send_data byte.
     *
     * The same DMA buffer is used for both TX and RX data. This should
     * be safe because the TX byte will always be read before the RX byte
     * is written. The original TX data in the DMA buffer will be
     * overwritten by RX data.
     */

    if (len > sizeof (spi_dma_buf))
        return (RC_BAD_PARAM);

    /*
     * Check if TXE is set. This should normally not happen, unless the
     * previous SPI transaction was prematurely aborted.
     */
    if ((SPI_SR(spi) & SPI_SR_TXE) == 0) {
        dprintf(DF_SPI, "SPI DMA TXE is not set");
        return (RC_BUSY);
    }

    if (config.debug_flag & DF_SPI) {
        if (cmd_data) {
            for (int i = cmd_len - 1; i >= 0; i--) {
                printf(" c=%02x", ((uint8_t *)cmd_data)[i]);
            }
        }
        if (send_data) {
            for (uint i = 0; i < send_len; i++) {
                printf(" w=%02x", ((uint8_t *)send_data)[i]);
            }
        }
    }

    /* Copy cmd_data into DMA buffer (reverse byte order) */
    while (cmd_len && cmd_data) {
        *buf++ = ((uint8_t *)cmd_data)[--cmd_len];
    }

    /* Copy send_data into DMA buffer */
    if (send_len && send_data) {
        memcpy(buf, send_data, send_len);
        buf += send_len;
    }

    /* Zero-fill the remainder of the DMA buffer */
    while (buf < (spi_dma_buf + len))
        *buf++ = 0;

    spi_dma_init(spi);

    /* Set up DMA parameters for SPI1_TX */
    dma = DMA2;
    stream = 5;
    dma_disable_stream(dma, stream);
    dma_clear_interrupt_flags(dma, stream, DMA_LIFCR_CTEIF0 |
                              DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0);
    dma_set_memory_address(dma, stream, (uintptr_t) spi_dma_buf);
    dma_set_number_of_data(dma, stream, len);
    if (width == 1)
        dma_set_memory_size(dma, stream, DMA_SxCR_MSIZE_8BIT);
    else
        dma_set_memory_size(dma, stream, DMA_SxCR_MSIZE_16BIT);

    /* Enable SPI1_TX DMA */
    dma_enable_stream(dma, stream);

    /* Set up DMA parameters for SPI1_RX */
    dma = DMA2;
    stream = 2;

    dma_disable_stream(dma, stream);
    dma_clear_interrupt_flags(dma, stream, DMA_LIFCR_CTEIF0 |
                              DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0);

    dma_set_memory_address(dma, stream, (uintptr_t) spi_dma_buf);
    dma_set_number_of_data(dma, stream, len);
    if (width == 1)
        dma_set_memory_size(dma, stream, DMA_SxCR_MSIZE_8BIT);
    else
        dma_set_memory_size(dma, stream, DMA_SxCR_MSIZE_16BIT);

    /* Enable SPI1_RX DMA */
    dma_enable_stream(dma, stream);

    /* Enable DMA to begin the SPI transaction */
    spi_enable_rx_dma(spi);
    spi_enable_tx_dma(spi);

    /* Timeout is proportional to length (100 usec / byte) */
    timeout = timer_tick_plus_usec(100 * len);

    /* Wait for transfer finished by checking TCIF, TXE, and BSY in order */
    while ((DMA_LISR(dma) & DMA_LISR_TCIF2) == 0)
        if (timer_tick_has_elapsed(timeout)) {
            dprintf(DF_SPI, "SPI DMA completion timeout");
            rc = RC_TIMEOUT;
            break;
        }
    while ((SPI_SR(spi) & SPI_SR_TXE) == 0)
        if (timer_tick_has_elapsed(timeout)) {
            dprintf(DF_SPI, "SPI DMA TXE timeout");
            rc = RC_TIMEOUT;
            break;
        }
    while ((SPI_SR(spi) & SPI_SR_BSY) != 0)
        if (timer_tick_has_elapsed(timeout)) {
            dprintf(DF_SPI, "SPI DMA BSY timeout");
            rc = RC_TIMEOUT;
            break;
        }

    /* SPI transaction is complete, disable DMA */
    spi_disable_tx_dma(spi);
    spi_disable_rx_dma(spi);

    if (config.debug_flag & DF_SPI) {
        for (uint i = recv_start; i < recv_start + recv_len; i++) {
            printf(" r=%0*x", width * 2, spi_dma_buf[i]);
        }
    }

    /* Copy received data into recv buffer */
    if (recv_len && recv_data)
        memcpy(recv_data, spi_dma_buf + recv_start, recv_len);

    return (rc);
}

/*
 * spi_setup() configures the SPI master at the specified byte width.
 *             GPIOs for all SPI chip selects will also be configured.
 *
 * @param [in]  chip - SPI chip select number.
 * @param [in]  width - The byte width to use for transactions.
 *                      Only a value of 1 or 2 is supported by the STM32F4
 *                      hardware master. Note that most devices use a 8-bit
 *                      width. Some devices use a 16-bit width. Some devices
 *                      (some audio codes) require a 12-bit width, which is
 *                      not supported by the STM32F4 SPI hardware master.
 */
static rc_t
spi_setup(uint chip, uint width)
{
    if (width == 0)
        width = 1;

    if (config.board_type == BOARD_TYPE_UNKNOWN)
        return (RC_NO_DATA);

#if 0
    uint32_t gpio;
    uint16_t pin;
    rc_t rc;

    /* If the chip number is 0xff, the chip select is controlled by caller */
    if (chip != 0xff) {
        /* Configure chip select as de-selected */
        if ((rc = spi_get_chipsel(chip, &gpio, &pin)) != RC_SUCCESS)
            return (rc);
        gpio_set(spi_cs_gpio, spi_cs_pin);  // De-select
        gpio_setmode(spi_cs_gpio, spi_cs_pin, GPIO_SETMODE_OUTPUT_50);
        gpio_mode_setup(spi_cs_gpio, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
        gpio_set_output_options(spi_cs_gpio, GPIO_OTYPE_PP,
                                GPIO_OSPEED_50MHZ, pin);
    }
#endif

    switch (config.board_type) {
        case BOARD_TYPE_AMIGAPCI:
            break;
        case BOARD_TYPE_APCIDEV:
            /* SPI not supported */
            break;
        case BOARD_TYPE_KEYJAM:
            break;
    }

    if (spi_sw_bitbang == 0) {  // HW
        uint32_t spi = SPI_HW_CORE;
        uint32_t dformat = (width == 2) ? SPI_CR1_DFF_16BIT : SPI_CR1_DFF_8BIT;
        uint32_t spi_clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_8;  // 8 MHz

        spi_disable(spi);
        (void) spi_init_master(spi, spi_clkdiv,
                        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,  // idle clock polarity
                        SPI_CR1_CPHA_CLK_TRANSITION_1,    // read on first edge
                        dformat,                          // data frame format
                        SPI_CR1_MSBFIRST);                // bit order
        spi_disable_crc(spi);
        spi_enable_software_slave_management(spi);
        spi_set_nss_high(spi);
        spi_clear_mode_fault(spi);
        spi_enable(spi);
    }

    return (RC_SUCCESS);
}

/*
 * spi_word_recv() receives one or two bytes (depending on the configured
 *                 width) from the currently selected SPI chip select.
 *
 * @param [in]  spi   - The SPI master base address.
 * @param [out] data  - The data value received by the SPI master.
 * @param [in]  width - The SPI transfer byte width (1 byte or 2).
 *
 * @return      RC_SUCCESS - A value was retrieved.
 * @return      RC_TIMEOUT - The SPI master took too long to receive data.
 */
static rc_t
spi_word_recv(uint32_t spi, uint16_t *data, uint width)
{
    if (spi_sw_bitbang == 0) {
        /* Hardware SPI */
        uint64_t timeout = timer_tick_plus_msec(SPI_READ_TIMEOUT);

        /* Wait for transfer finished */
        while ((SPI_SR(spi) & SPI_SR_RXNE) == 0)
            if (timer_tick_has_elapsed(timeout))
                break;

        if ((SPI_SR(spi) & SPI_SR_RXNE) == 0)
            return (RC_TIMEOUT);

        /* Read the data (8 or 16 bits, depending on DFF bit) from DR */
        *data = (uint16_t) SPI_DR(spi);
    } else {
        /* Software SPI */
        uint dataval = 0;
        if (spi_dma_buf[0] != 0) {
            /* Have previous data received during send */
            spi_dma_buf[0] = 0;
            if (width == 2)
                dataval = spi_dma_buf[1] | (spi_dma_buf[2] << 1);
            else
                dataval = spi_dma_buf[1];
        } else {
            uint bits = (width == 2) ? 16 : 8;
            uint bit;
            for (bit = bits; bit > 0; bit--) {
                *spi_clk = 1;
                __sync_synchronize();
                dataval = (dataval << 1) | *spi_miso;
                *spi_clk = 0;
                __sync_synchronize();
            }
        }
        *data = dataval;
    }
    return (RC_SUCCESS);
}

/*
 * spi_word_send() sends one or two bytes (depending on the configured
 *                 width) to the currently selected SPI chip select.
 *
 * @param [in]  spi   - The SPI master base address.
 * @param [in]  data  - The data value for the SPI master to send.
 * @param [in]  width - The SPI transfer byte width (1 byte or 2).
 *
 * @return      RC_SUCCESS - The value was sent.
 * @return      RC_TIMEOUT - The SPI master took too long to send data.
 */
static rc_t
spi_word_send(uint32_t spi, uint16_t data, uint width)
{
    if (spi_sw_bitbang == 0) {
        /* Hardware SPI */
        uint64_t timeout = timer_tick_plus_msec(SPI_WRITE_TIMEOUT);

        /* Wait for transfer finished */
        while ((SPI_SR(spi) & SPI_SR_TXE) == 0)
            if (timer_tick_has_elapsed(timeout))
                break;

        if ((SPI_SR(spi) & SPI_SR_TXE) == 0)
            return (RC_TIMEOUT);

        /* Write data (8 or 16 bits, depending on DFF) into DR */
        SPI_DR(spi) = data;
    } else {
        /* Software SPI */
        uint bits = (width == 2) ? 16 : 8;
        uint bit;
        uint dataval = 0;
        for (bit = bits; bit > 0; bit--) {
            *spi_mosi = data >> (bit - 1);
            __sync_synchronize();
            *spi_clk = 1;
            __sync_synchronize();
            dataval = (dataval << 1) | *spi_miso;
            *spi_clk = 0;
        }
        spi_dma_buf[0] = width;
        spi_dma_buf[1] = dataval;
        if (width == 2)
            spi_dma_buf[2] = dataval >> 8;
    }
    return (RC_SUCCESS);
}

/*
 * spi_block_send_r() sends a sequence of bytes on the SPI bus in
 *                    reverse order (last byte first).
 *
 * @param [in]  spi   - The SPI master base address.
 * @param [in]  data  - The data value for the SPI master to send.
 * @param [in]  len   - The number of bytes to send.
 * @param [in]  width - The SPI transfer byte width (1 byte or 2).
 *
 * @return      RC_SUCCESS - The values were sent.
 * @return      RC_TIMEOUT - The SPI master took too long to send data.
 */
static rc_t
spi_block_send_r(uint32_t spi, const uint8_t *data, uint len, uint width)
{
    uint16_t value;
    rc_t     rc;

    while (len > 0) {
        if (width > len)
            width = len;
        len -= width;
        if (width == 2)
            value = data[len] | (data[len + 1] << 8);
        else
            value = data[len];
        rc = spi_word_send(spi, value, width);
        if (rc != RC_SUCCESS)
            return (rc);

        dprintf(DF_SPI, " w=%0*x", width * 2, value);

        rc = spi_word_recv(spi, &value, width);
        if (rc != RC_SUCCESS)
            return (rc);
    }
    return (RC_SUCCESS);
}

/*
 * spi_block_send() sends a sequence of bytes on the SPI bus.
 *
 * @param [in]  spi   - The SPI master base address.
 * @param [in]  data  - The data value for the SPI master to send.
 * @param [in]  len   - The number of bytes to send.
 * @param [in]  width - The SPI transfer byte width (1 byte or 2).
 *
 * @return      RC_SUCCESS - The values were sent.
 * @return      RC_TIMEOUT - The SPI master took too long to send data.
 */
static rc_t
spi_block_send(uint32_t spi, const uint8_t *data, uint len, uint width)
{
    rc_t     rc;
    uint16_t value;

    while (len > 0 && data) {
        if (width > len)
            width = len;

        value = data[0];
        if (width == 2)
            value |= (data[1] << 8);

        rc = spi_word_send(spi, value, width);
        if (rc != RC_SUCCESS)
            return (rc);

        dprintf(DF_SPI, " w=%0*x", width * 2, value);

        rc = spi_word_recv(spi, &value, width);
        if (rc != RC_SUCCESS)
            return (rc);

        len  -= width;
        data += width;
    }
    return (RC_SUCCESS);
}

/*
 * spi_block_recv() receives a sequence of bytes on the SPI bus.
 *
 * @param [in]  spi   - The SPI master base address.
 * @param [in]  data  - The data values received by the SPI master.
 * @param [in]  len   - The number of bytes to send.
 * @param [in]  width - The SPI transfer byte width (1 byte or 2).
 *
 * @return      RC_SUCCESS - The values were sent.
 * @return      RC_TIMEOUT - The SPI master took too long to send data.
 */
static rc_t
spi_block_recv(uint32_t spi, uint8_t *data, uint len, uint width)
{
    rc_t     rc;
    uint16_t value;

    while (len > 0) {
        if (width > len)
            width = len;

        rc = spi_word_send(spi, 0, width);
        if (rc != RC_SUCCESS)
            return (rc);
        rc = spi_word_recv(spi, &value, width);
        if (rc != RC_SUCCESS)
            return (rc);

        data[0] = (uint8_t) value;
        if (width == 2)
            data[1] = (uint8_t) (value >> 8);

        dprintf(DF_SPI, " R=%0*x", width * 2, value);

        len  -= width;
        data += width;
    }
    return (RC_SUCCESS);
}

/**
 * spi_chip_own() acquires or releases exclusive access to the specified
 *                SPI chip select. It is the caller's responsibility to
 *                call this function with lock = TRUE before starting a
 *                sequence of SPI accesses and to always call this function
 *                a second time with lock = FALSE if the first call
 *                returned RC_SUCCESS. Failure to do so will leave USB
 *                interrupts disabled.
 *
 * @param [in]  chip - SPI chip select number.
 * @param [in]  lock - TRUE  - Lock (acquire) the specified chip.
 *                     FALSE - Unlock (release) the specified chip.
 *
 * @return      RC_SUCCESS - Chip lock acquired or released.
 * @return      RC_FAILURE - Chip lock acquire or release failed.
 */
rc_t
spi_chip_own(uint chip, bool_t lock)
{
    rc_t rc = spi_get_pins(chip);
    if (rc != RC_SUCCESS)
        return (rc);

    if (lock == TRUE) {
        dprintf(DF_SPI, "CS%x <\n", chip);
    } else {
        dprintf(DF_SPI, "> CS%x\n", chip);
    }

    return (RC_SUCCESS);
}

/**
 * spi_chip_read() reads a block of data from an SPI bus device.
 *
 * @param [in]  chip  - SPI chip select number.
 * @param [in]  addr  - The address / command to initiate the read.
 *                      Note this data is sent in big endian format on
 *                      the wire so that the MSB of addr is sent first.
 *                      Up to 8 bytes of address may be sent.
 * @param [in]  alen  - The number of bytes in the address to send.
 * @param [in]  dlen  - The number of bytes of data to receive.
 * @param [in]  data  - The data values received by the SPI master. These
 *                      bytes are not endian formatted -- they are received
 *                      in wire byte order.
 *
 * @return      RC_SUCCESS   - The values were sent.
 * @return      RC_TIMEOUT   - The SPI master took too long to receive data.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
rc_t
spi_chip_read(uint chip, uint64_t addr, uint alen, uint dlen, void *data)
{
    uint     abytes = (uint8_t) alen;
    uint     width  = (uint8_t) (alen >> 8);
    uint32_t spi    = SPI_HW_CORE;
    uint     len;
    rc_t     rc;

    if (abytes == 0xff)
        abytes = 0;
    else if ((abytes == 0) || (abytes > 8))
        abytes = 1;
    if (width == 0)
        width = 1;

    rc = spi_setup(chip, width);
    if (rc != RC_SUCCESS)
        goto spi_failure;
    spi_chipsel(chip, TRUE);

    len = abytes + dlen;

    /* Use DMA if allowed by config and requirements are met */
    if (cfg_spi_dma && len <= sizeof (spi_dma_buf) && width == 1) {
        rc = spi_dma_rw(spi, width, (uint16_t) abytes, 0, (uint16_t) dlen,
                        (uint16_t) abytes, &addr, NULL, data);
    } else {
        /* Send the command and address bytes */
        rc = spi_block_send_r(spi, (void *) &addr, abytes, width);
        if (rc != RC_SUCCESS)
            goto spi_failure;

        rc = spi_block_recv(spi, data, dlen, width);
    }

spi_failure:
    spi_chipsel(chip, FALSE);
    dprintf(DF_SPI, "\n");

    return (rc);
}

/**
 * spi_chip_write() writes a block of data to an SPI bus device.
 *
 * @param [in]  chip  - SPI chip select number.
 * @param [in]  addr  - The address / command to initiate the write.
 *                      Note this data is sent in big endian format on
 *                      the wire so that the MSB of addr is sent first.
 *                      Up to 8 bytes of address may be sent.
 * @param [in]  alen  - The number of bytes in the address to send.
 * @param [in]  dlen  - The number of bytes of data to send.
 * @param [in]  data  - The data values sent by the SPI master. These
 *                      bytes are not endian formatted and are sent in
 *                      wire byte order.
 *
 * @return      RC_SUCCESS   - The values were received.
 * @return      RC_TIMEOUT   - The SPI master took too long to send data.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
rc_t
spi_chip_write(uint chip, uint64_t addr, uint alen, uint dlen, const void *data)
{
    uint     abytes = (uint8_t) alen;
    uint     width  = (uint8_t) (alen >> 8);
    uint32_t spi    = SPI_HW_CORE;
    uint     len;
    rc_t     rc;

    if (abytes == 0xff)
        abytes = 0;
    else if ((abytes == 0) || (abytes > 8))
        abytes = 1;
    if (width == 0)
        width = 1;

    rc = spi_setup(chip, width);
    if (rc != RC_SUCCESS)
        goto spi_failure;
    spi_chipsel(chip, TRUE);

    len = abytes + dlen;

    /* Use DMA if allowed by config and requirements are met */
    if (cfg_spi_dma && len <= sizeof (spi_dma_buf) && width == 1) {
        rc = spi_dma_rw(spi, width, (uint16_t) abytes, (uint16_t) dlen,
                        0, 0, &addr, data, NULL);
    } else {
        /* Send the command and address bytes */
        rc = spi_block_send_r(spi, (void *) &addr, abytes, width);
        if (rc != RC_SUCCESS)
            goto spi_failure;

        rc = spi_block_send(spi, data, dlen, width);
    }

spi_failure:
    spi_chipsel(chip, FALSE);
    dprintf(DF_SPI, "\n");

    return (rc);
}

/**
 * spi_chip_rw() simultaneously reads and writes block of data with an SPI bus
 *               device.
 *
 * @param [in]  chip       - SPI chip select number.
 * @param [in]  width      - The SPI bus width to use (1 = 8-bit; 2 = 16-bit).
 * @param [in]  send_len   - Number of bytes to send.
 * @param [in]  recv_len   - Number of bytes to receive.
 * @param [in]  recv_start - Number of bytes to skip before receive starts.
 * @param [in]  send_data  - Send data buffer.
 * @param [out] recv_data  - Receive data buffer.
 *
 * @return      RC_SUCCESS   - The values were sent.
 * @return      RC_TIMEOUT   - The SPI master took too long to receive data.
 * @return      RC_BAD_PARAM - Invalid SPI chip select specified.
 */
rc_t
spi_chip_rw(uint chip, uint width,  uint send_len, uint recv_len,
            uint recv_start, void *send_data, void *recv_data)
{
    uint32_t spi = SPI_HW_CORE;
    uint     len;
    rc_t     rc;

    rc = spi_setup(chip, width);
    if (rc != RC_SUCCESS)
        goto spi_failure;
    spi_chipsel(chip, TRUE);

    len = MAX(send_len + recv_len, recv_start + recv_len);

    /* Use DMA if allowed by config and requirements are met */
    if (cfg_spi_dma && len <= sizeof (spi_dma_buf) && width == 1) {
        rc = spi_dma_rw(SPI_HW_CORE, width, 0, (uint16_t) send_len,
                        (uint16_t) recv_len, (uint16_t) recv_start,
                        NULL, send_data, recv_data);
    } else {
        uint8_t *s_data = send_data;
        uint8_t *r_data = recv_data;
        uint16_t value;

        while (len > 0) {
            bool_t did_write = FALSE;

            if (width > len)
                width = len;

            /* Send next byte or word */
            if (send_len > 0) {
                value = s_data[0];
                if (width == 2)
                    value |= (s_data[1] << 8);
                send_len -= width;
                s_data   += width;
                dprintf(DF_SPI, " w=%0*x", width * 2, value);
                did_write = TRUE;
            } else {
                value = 0;
            }
            rc = spi_word_send(spi, value, width);
            if (rc != RC_SUCCESS)
                goto spi_failure;

            /* Receive next byte or word */
            rc = spi_word_recv(spi, &value, width);
            if (rc != RC_SUCCESS)
                goto spi_failure;
            if (recv_start >= width) {
                recv_start -= width;
            } else if (recv_len > 0) {
                r_data[0] = (uint8_t) value;
                if (width == 2)
                    r_data[1] = (uint8_t) (value >> 8);
                recv_len -= width;
                r_data   += width;
                dprintf(DF_SPI, "%cr=%0*x", (did_write == TRUE) ? ';' : ' ',
                        width * 2, value);
            }

            len -= width;
        }
    }

spi_failure:
    spi_chipsel(chip, FALSE);
    dprintf(DF_SPI, "\n");

    return (rc);
}

void
spi_list_chipsel(void)
{
    switch (config.board_type) {
        case BOARD_TYPE_AMIGAPCI:
            printf("0 U110\n"
                   "1 U109\n"
                   "2 U712\n"
                   "3 U409\n"
                   "4 SPI-A U111\n"
                   "5 SPI-B U400\n"
                   "6 SPI-C\n");
            break;
        case BOARD_TYPE_APCIDEV:
            printf("No SPI devices\n");
            break;
        case BOARD_TYPE_KEYJAM:
            printf("0 SPI_CS1\n"
                   "1 SPI_CS2\n");
            break;
    }
}

void
spi_init(void)
{
    switch (config.board_type) {
        case BOARD_TYPE_AMIGAPCI:
            spi_sw_bitbang = 1;
            gpio_set(GPIOE, GPIO0 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 |
                            GPIO12 | GPIO13 | GPIO14 | GPIO15);
            gpio_setmode(GPIOE, GPIO0 | GPIO1 | GPIO3 | GPIO4 | GPIO5 |
                                GPIO6 | GPIO13 | GPIO14 | GPIO15,
                         GPIO_SETMODE_OUTPUT_50);
            gpio_setmode(GPIOE, GPIO2 | GPIO12, GPIO_SETMODE_INPUT);
            break;
        case BOARD_TYPE_APCIDEV:
            /* SPI not supported */
            break;
        case BOARD_TYPE_KEYJAM:
            /*
             * PA3 CS2
             * PA4 CS1
             * PA5 CLK
             * PA6 MISO
             * PA7 MOSI
             */
            spi_sw_bitbang = 1;  // Debug
            gpio_set(GPIOA, GPIO3 | GPIO4 | GPIO5 | GPIO7);
            gpio_setmode(GPIOA, GPIO3 | GPIO4 | GPIO5 | GPIO7,
                         GPIO_SETMODE_OUTPUT_50);
            gpio_setmode(GPIOA, GPIO6, GPIO_SETMODE_INPUT);
            if (spi_sw_bitbang) {
                /* Software SPI */
                gpio_clear(GPIOA, GPIO5);  // CLK=0
            } else {
                /* Hardware SPI */
                gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
                gpio_set_af(GPIOA, GPIO_AF5, GPIO5);
                gpio_set_af(GPIOA, GPIO_AF5, GPIO6 | GPIO7);
                gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                                GPIO6 | GPIO7);
                RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;
            }
            break;
    }
    if (spi_sw_bitbang)
        spi_dma_buf[0] = 0;
}
