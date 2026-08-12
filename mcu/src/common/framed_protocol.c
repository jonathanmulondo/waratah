/* framed_protocol.c -- see framed_protocol.h.
 *
 * Note on timeouts: unlike the RTL (which has its own TIMEOUT_CLKS
 * counter to abandon a stalled frame), this module has no notion of
 * time -- it only reacts to bytes. The caller is responsible for
 * calling frame_rx_init() to resync if too long passes between bytes
 * (e.g. driven off a UART idle-line interrupt or a periodic timer).
 */
#include "framed_protocol.h"

uint8_t crc8(uint8_t crc, uint8_t data)
{
    uint8_t x = crc ^ data;
    for (int k = 0; k < 8; k++)
        x = (x & 0x80) ? (uint8_t)((x << 1) ^ 0x07) : (uint8_t)(x << 1);
    return x;
}

void frame_rx_init(frame_rx_t *rx)
{
    rx->state = FRAME_R_SOF;
    rx->pcnt = 0;
    rx->crc_running = 0;
    rx->frame_ready = false;
    rx->crc_ok = false;
}

void frame_rx_byte(frame_rx_t *rx, uint8_t byte)
{
    switch (rx->state) {
    case FRAME_R_SOF:
        if (byte == FRAME_SOF)
            rx->state = FRAME_R_CMD;
        break;

    case FRAME_R_CMD:
        rx->cmd = byte;
        rx->crc_running = crc8(0x00, byte);
        rx->state = FRAME_R_LEN;
        break;

    case FRAME_R_LEN:
        rx->len = byte;
        rx->pcnt = 0;
        rx->crc_running = crc8(rx->crc_running, byte);
        rx->state = (byte == 0) ? FRAME_R_CRC : FRAME_R_PAY;
        break;

    case FRAME_R_PAY:
        if (rx->pcnt < FRAME_MAX_PAYLOAD)
            rx->payload[rx->pcnt] = byte;
        rx->crc_running = crc8(rx->crc_running, byte);
        if (rx->pcnt == rx->len - 1)
            rx->state = FRAME_R_CRC;
        else
            rx->pcnt++;
        break;

    case FRAME_R_CRC:
        rx->crc_ok = (byte == rx->crc_running);
        rx->frame_ready = true;
        rx->state = FRAME_R_SOF;   /* ready to resync for the next frame */
        break;
    }
}

uint8_t frame_build(uint8_t cmd, const uint8_t *payload, uint8_t len,
                     uint8_t *out_buf)
{
    uint8_t crc = crc8(0x00, cmd);
    crc = crc8(crc, len);

    uint8_t i = 0;
    out_buf[i++] = FRAME_SOF;
    out_buf[i++] = cmd;
    out_buf[i++] = len;
    for (uint8_t k = 0; k < len; k++) {
        out_buf[i++] = payload[k];
        crc = crc8(crc, payload[k]);
    }
    out_buf[i++] = crc;
    return i;
}
