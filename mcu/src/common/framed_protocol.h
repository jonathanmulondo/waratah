/* framed_protocol.h -- payload-computer link framing (EDDI-MCU)
 *
 * Portable C port of the framing/CRC layer from fpga/rtl/payload_if.v:
 * [SOF=0xA5][CMD][LEN][PAYLOAD x LEN][CRC-8], CRC-8 poly 0x07 init 0x00
 * over CMD+LEN+PAYLOAD. Same wire format as the FPGA design, so ground
 * software written against one target works against the other.
 *
 * This module only does framing + CRC -- byte-stream in, complete
 * frames out, and frames-to-bytes for replies. Command *dispatch* (what
 * PING/GET_STATUS/READ_PSD/etc. actually do) is application logic that
 * belongs in the firmware, not here, since it needs to reach into the
 * QTN/debris state this module doesn't know about. No HAL/UART
 * dependency -- caller feeds bytes in and drains bytes out.
 */
#ifndef EDDI_FRAMED_PROTOCOL_H
#define EDDI_FRAMED_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define FRAME_SOF          0xA5u
#define FRAME_MAX_PAYLOAD  32u   /* covers every command in the RTL's set */

#define CMD_PING     0x01u
#define CMD_STATUS   0x02u
#define CMD_RDPSD    0x03u
#define CMD_RDBUF    0x04u
#define CMD_ACKEV    0x05u
#define CMD_CFG      0x06u
#define CMD_NACK     0xEEu
#define CMD_ACK_BIT  0x80u   /* success responses echo CMD | this bit */

#define ERR_BAD_CRC     1u
#define ERR_UNKNOWN_CMD 2u
#define ERR_BAD_ARGS    3u

typedef enum {
    FRAME_R_SOF = 0,
    FRAME_R_CMD,
    FRAME_R_LEN,
    FRAME_R_PAY,
    FRAME_R_CRC
} frame_rx_state_t;

typedef struct {
    frame_rx_state_t state;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  payload[FRAME_MAX_PAYLOAD];
    uint8_t  pcnt;
    uint8_t  crc_running;
    bool     frame_ready;   /* a complete frame (good or bad CRC) landed */
    bool     crc_ok;
} frame_rx_t;

uint8_t crc8(uint8_t crc, uint8_t data);

void frame_rx_init(frame_rx_t *rx);

/* Feed one received byte. Sets rx->frame_ready when a full frame has
 * arrived -- caller should check rx->crc_ok, dispatch on rx->cmd, then
 * call frame_rx_init() (or a dedicated reset) before the next byte. */
void frame_rx_byte(frame_rx_t *rx, uint8_t byte);

/* Build a complete response frame into out_buf (must be at least
 * len+4 bytes). Returns the total frame length written. */
uint8_t frame_build(uint8_t cmd, const uint8_t *payload, uint8_t len,
                     uint8_t *out_buf);

#endif
