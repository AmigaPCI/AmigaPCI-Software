/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * Functions for AmigaPCI BEC messaging.
 */

#ifndef _BECMSG_H
#define _BECMSG_H

uint send_cmd(uint8_t cmd, void *arg, uint16_t arglen,
              void *reply, uint replymax, uint *replyalen);

uint send_cmd_retry(uint8_t cmd, void *arg, uint16_t arglen,
                    void *reply, uint replymax, uint *replyalen);

const char *bec_err(uint status);

/*
 * Preferred maximum payload for bulk data (such as file reads and writes)
 * on the current message interface. The keyboard interface transfers
 * replies at about 2.5 KB/s with interrupts disabled, so smaller messages
 * keep the Amiga responsive. Messages may still be up to
 * BEC_MSG_MAX_PAYLOAD.
 */
uint bec_msg_data_max(void);

void cia_spin(unsigned int ticks);
extern uint8_t bec_msg_interface;
extern uint16_t bec_rx_timeout;  // Message timeout in milliseconds
uint cia_ticks(void);

#endif  /* _BECMSG_H */
