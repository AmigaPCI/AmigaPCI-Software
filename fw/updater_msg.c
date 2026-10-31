/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2026.
 *
 * ---------------------------------------------------------------------
 *
 * BEC message interface handling.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bec_cmd.h"
#include "main.h"
#include "msg.h"
#include "amigartc.h"
#include "keyboard.h"
#include "mouse.h"
#include "crc32.h"
#include "keyboard.h"
#include "persist.h"
#include "printf.h"
#include "timer.h"
#include "uart.h"
#include "utils.h"
#include "version.h"
#include "stdlib.h"
#include "cmdline.h"
#include "stm32flash.h"
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/scb.h>

#define SWAP16(x)   __builtin_bswap16(x)
#define SWAP32(x)   __builtin_bswap32(x)
#define SWAP64(x)   __builtin_bswap64(x)

#define FW_READ_MAX 256  // Maximum bytes per BEC_CMD_FW_READ

uint8_t msg_source;  // 0 = RTC, 1 = Keyboard

/*
 * fw_info_find
 * ------------
 * Locate the firmware identification structure (see main.h) in flash.
 * Returns NULL if no valid structure is found.
 */
static const fw_info_t *
fw_info_find(void)
{
    /* Flash size register is in Kbytes */
    uint32_t flash_size = *(volatile uint16_t *) DESIG_FLASH_SIZE_BASE * 1024;
    uintptr_t addr;

    for (addr = FLASH_BASE; addr <= FLASH_BASE + flash_size - sizeof (fw_info_t);
         addr += 4) {
        const fw_info_t *fi = (const fw_info_t *) addr;
        if ((fi->fi_magic == FW_INFO_MAGIC) &&
            (fi->fi_self == fi) &&
            (fi->fi_struct_size >= sizeof (fw_info_t)) &&
            (fi->fi_fw_size <= flash_size) &&
            (addr - FLASH_BASE + fi->fi_struct_size <= fi->fi_fw_size)) {
            return (fi);
        }
    }
    return (NULL);
}

int
atoi(const char *str)
{
    int value;

    for (value = 0; (*str >= '0') && (*str <= '9'); str++)
        value = value * 10 + (*str - '0');

    return (value);
}

static void
msg_reply(uint rstatus, uint rlen1, const void *data1,
                        uint rlen2, const void *data2)
{
    uint32_t crc;
    uint rlen = rlen1 + rlen2;

    if (rlen + BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN > sizeof (bec_msg_outbuf)) {
        uart_puts("msg len ");
        uart_puthex(rlen);
        uart_puts(" too long to send\n");
        return;
    }
    bec_msg_out = 0;
    bec_msg_outbuf[0] = 0xcd;  // bec_magic
    bec_msg_outbuf[1] = 0x68;  // bec_magic
    bec_msg_outbuf[2] = rstatus;
    bec_msg_outbuf[3] = (uint8_t) (rlen >> 8);
    bec_msg_outbuf[4] = (uint8_t) rlen;
    if (rlen1 > 0)
        memcpy(&bec_msg_outbuf[5], data1, rlen1);
    if (rlen2 > 0)
        memcpy(&bec_msg_outbuf[5] + rlen1, data2, rlen2);

    /* CRC includes cmd + length + data */
    crc = crc32(0, bec_msg_outbuf + 2, rlen + BEC_MSG_HDR_LEN - 2);
    crc = SWAP32(crc);
    memcpy(&bec_msg_outbuf[BEC_MSG_HDR_LEN + rlen], &crc, BEC_MSG_CRC_LEN);

    if (msg_source == 1) {
        /* Reply is via keyboard byte sequence */
        bec_msg_out_max = (rlen + BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN);
        keyboard_reply_msg();
        return;
    }

    bec_msg_out_max = (rlen + BEC_MSG_HDR_LEN + BEC_MSG_CRC_LEN) * 2;
    bec_msg_out_timeout = timer_tick_plus_msec(1000);

    /* Kick off reply by pre-loading first response byte */
    amigartc_reply_pending();
}

int
msg_process_fast(void)
{
    uint cmd    = bec_msg_inbuf[2];
    uint msglen = (bec_msg_inbuf[3] << 8) | bec_msg_inbuf[4];
    uint32_t crc_expect;
    uint32_t crc_calc;

    memcpy(&crc_expect, bec_msg_inbuf + BEC_MSG_HDR_LEN + msglen,
           sizeof (crc_expect));
    crc_calc = crc32(0, &bec_msg_inbuf[2], BEC_MSG_HDR_LEN - 2 + msglen);
    crc_calc = SWAP32(crc_calc);
    if (crc_expect != crc_calc) {
        msg_reply(BEC_STATUS_CRC, 0, NULL, 0, NULL);
        uart_puts("cmd=");
        uart_puthex(cmd);
        uart_puts(" l=");
        uart_puthex(msglen);
        uart_puts(" CRC ");
        uart_puthex(crc_expect);
        uart_puts(" != ");
        uart_puthex(crc_calc);
        uart_putchar('\n');
        return (1);
    }
    switch (cmd) {
        case BEC_CMD_NULL:
            /* No reply */
            break;
        case BEC_CMD_NOP:
            msg_reply(BEC_STATUS_OK, 0, NULL, 0, NULL);
            break;
        default:
            return (0);  // Push message to processing in the slow path
    }

    /* The message has been processed */
    return (1);
#if 0
    uint pos;
    printf("cmd=%02x len=%04x", cmd, msglen);
    for (pos = 0; pos < msglen; pos++)
        printf(" %02x", bec_msg_inbuf[4 + pos]);
    printf("\n");
#endif
}

void
msg_process_slow(void)
{
    /* The message was already CRC-checked in the fast path */
    uint cmd    = bec_msg_inbuf[2];
    uint msglen = (bec_msg_inbuf[3] << 8) | bec_msg_inbuf[4];
    uint pos;

    switch (cmd) {
        case BEC_CMD_ID: {
            bec_id_t reply;
            const fw_info_t *fi;
            uint temp[3];
            memset(&reply, 0, sizeof (reply));
            temp[0] = version_str[8 + 0] - '0';
            temp[1] = version_str[8 + 2] - '0';
            reply.bid_version[0] = SWAP16(temp[0]);
            reply.bid_version[1] = SWAP16(temp[1]);
            if (pos == 0)
                pos = 18;
            else
                pos += 8 + 7;
            /* Year-Month-Day */
            temp[0] = atoi(&version_str[19]);
            temp[1] = atoi(&version_str[24]);
            temp[2] = atoi(&version_str[27]);
            reply.bid_date[0] = temp[0] / 100;
            reply.bid_date[1] = temp[0] % 100;
            reply.bid_date[2] = temp[1];
            reply.bid_date[3] = temp[2];
            pos += 11;
            /* Hour:Minute:Second */
            temp[0] = atoi(&version_str[30]);
            temp[1] = atoi(&version_str[33]);
            temp[2] = atoi(&version_str[36]);
            reply.bid_time[0] = temp[0];
            reply.bid_time[1] = temp[1];
            reply.bid_time[2] = temp[2];
            reply.bid_time[3] = 0;
            strcpy(reply.bid_serial, "Updater");
            reply.bid_rev        = SWAP16(0x0002);     // Protocol version 0.2
            reply.bid_features   = SWAP16(0x0003);     // Features
            reply.bid_board_type = persist.sp_board_type;
            reply.bid_board_rev  = persist.sp_board_rev;
            reply.bid_name[0] = '\0';
            fi = fw_info_find();
            reply.bid_size       = (fi == NULL) ? 0 : SWAP32(fi->fi_fw_size);
            msg_reply(BEC_STATUS_OK, sizeof (reply), &reply, 0, NULL);
            break;
        }
        case BEC_CMD_UPTIME: {
            uint64_t now = timer_tick_get();
            uint64_t usec = timer_tick_to_usec(now);
            usec = SWAP64(usec);  // Big endian format
            msg_reply(BEC_STATUS_OK, sizeof (usec), &usec, 0, NULL);
            break;
        }
        case BEC_CMD_LOOPBACK:
            msg_reply(cmd, msglen, bec_msg_inbuf + BEC_MSG_HDR_LEN, 0, NULL);
            break;
        case BEC_CMD_FW_READ: {
            bec_fw_rw_t req;
            const fw_info_t *fi = fw_info_find();
            uint32_t offset;
            uint32_t len;

            if (msglen < sizeof (req)) {
                msg_reply(BEC_STATUS_BADLEN, 0, NULL, 0, NULL);
                break;
            }
            /* Request is not 32-bit aligned in the input buffer */
            memcpy(&req, &bec_msg_inbuf[BEC_MSG_HDR_LEN], sizeof (req));
            offset = SWAP32(req.bfrw_offset);
            len    = SWAP16(req.bfrw_size);
            if ((fi == NULL) || (offset >= fi->fi_fw_size)) {
                msg_reply(BEC_STATUS_FAIL, 0, NULL, 0, NULL);
                break;
            }
            if (len > FW_READ_MAX)
                len = FW_READ_MAX;
            if (len > fi->fi_fw_size - offset)
                len = fi->fi_fw_size - offset;
            req.bfrw_size = SWAP16(len);
            msg_reply(BEC_STATUS_OK, sizeof (req), &req,
                      len, (const void *) (FLASH_BASE + offset));
            break;
        }
        case BEC_CMD_FW_WRITE: {
            bec_fw_rw_t req;
            uint32_t offset;
            uint32_t len;
            int rc;

            if (msglen < sizeof (req)) {
                msg_reply(BEC_STATUS_BADLEN, 0, NULL, 0, NULL);
                break;
            }
            /* Request is not 32-bit aligned in the input buffer */
            memcpy(&req, &bec_msg_inbuf[BEC_MSG_HDR_LEN], sizeof (req));
            offset = SWAP32(req.bfrw_offset);
            len    = SWAP16(req.bfrw_size);
            if (msglen < sizeof (req) + len) {
                /* Not enough data following request */
                msg_reply(BEC_STATUS_BADLEN, 0, NULL, 0, NULL);
                break;
            }
            if ((offset > STM32FLASH_SIZE) || (len > STM32FLASH_SIZE - offset)) {
                msg_reply(BEC_STATUS_BADARG, 0, NULL, 0, NULL);
                break;
            }
            /* Sector is erased first when offset is at the start of one */
            rc = stm32flash_write(offset, len,
                                  &bec_msg_inbuf[BEC_MSG_HDR_LEN + sizeof (req)],
                                  STM32FLASH_FLAG_AUTOERASE);
            if (rc != RC_SUCCESS) {
                msg_reply(BEC_STATUS_FAIL, 0, NULL, 0, NULL);
                break;
            }
            msg_reply(BEC_STATUS_OK, sizeof (req), &req, 0, NULL);
            if (offset == 0)
                uart_puts("Firmware update: ");
            else if ((offset & 0x7ff) == 0)
                uart_putchar('.');  // Every 4 KB
            break;
        }
        case BEC_CMD_FW_UNLOCK:
            msg_reply(BEC_STATUS_BADARG, 0, NULL, 0, NULL);  // Already unlocked
            break;
        case BEC_CMD_FW_LOCK:
            msg_reply(BEC_STATUS_OK, 0, NULL, 0, NULL);
            uart_puts("\nUpdater rebooting\n");
            timer_delay_msec(10);
            scb_reset_system();
            break;
        default:
            msg_reply(BEC_STATUS_UNKCMD, 0, NULL, 0, NULL);
            break;
    }
}

void
msg_init(void)
{
#if 0
    /* Map PA4 to EXTI4 */
    exti_select_source(EXTI4, GPIOA);
    exti_set_trigger(EXTI4, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI4);
    exti_reset_request(EXTI4);
    nvic_set_priority(NVIC_EXTI4_IRQ, 0x10);
    nvic_enable_irq(NVIC_EXTI4_IRQ);
#endif
#if 0
    exti_select_source(EXTI5, GPIOA);
    exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI5);
    exti_reset_request(EXTI5);
    nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x10);
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
#endif
}
