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

void cia_spin(unsigned int ticks);
extern uint8_t bec_msg_interface;
extern uint16_t bec_rx_timeout;  // Message timeout in milliseconds
uint cia_ticks(void);

#endif  /* _BECMSG_H */
