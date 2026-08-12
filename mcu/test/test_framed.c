/* test_framed.c -- self-checking test for framed_protocol.c
 *
 * Round-trips a few frames through build->parse, plus a deliberately
 * corrupted CRC, checking the parser recovers correctly.
 */
#include "../src/common/framed_protocol.h"
#include <stdio.h>
#include <string.h>

static int feed_and_check(const char *name, uint8_t cmd, const uint8_t *payload,
                           uint8_t len, bool corrupt_crc, bool expect_ok,
                           uint8_t expect_cmd, uint8_t expect_len)
{
    uint8_t buf[64];
    uint8_t n = frame_build(cmd, payload, len, buf);
    if (corrupt_crc) buf[n - 1] ^= 0xFF;

    frame_rx_t rx;
    frame_rx_init(&rx);
    for (uint8_t i = 0; i < n; i++)
        frame_rx_byte(&rx, buf[i]);

    if (!rx.frame_ready) {
        printf("FAIL [%s]: frame never completed\n", name);
        return 1;
    }
    if (rx.crc_ok != expect_ok) {
        printf("FAIL [%s]: crc_ok=%d, expected %d\n", name, rx.crc_ok, expect_ok);
        return 1;
    }
    if (expect_ok && (rx.cmd != expect_cmd || rx.len != expect_len)) {
        printf("FAIL [%s]: cmd=%02x len=%d, expected %02x/%d\n",
               name, rx.cmd, rx.len, expect_cmd, expect_len);
        return 1;
    }
    if (expect_ok && len > 0 && memcmp(rx.payload, payload, len) != 0) {
        printf("FAIL [%s]: payload mismatch\n", name);
        return 1;
    }
    return 0;
}

int main(void)
{
    int errors = 0;
    uint8_t p3[3] = {0x00, 0x05, 0x02};

    errors += feed_and_check("PING (no payload)", CMD_PING, NULL, 0,
                              false, true, CMD_PING, 0);
    errors += feed_and_check("READ_PSD (3-byte payload)", CMD_RDPSD, p3, 3,
                              false, true, CMD_RDPSD, 3);
    errors += feed_and_check("corrupted CRC", CMD_PING, NULL, 0,
                              true, false, 0, 0);

    if (errors == 0) {
        printf("TB_PASS: test_framed\n");
        return 0;
    } else {
        printf("TB_FAIL: test_framed (%d errors)\n", errors);
        return 1;
    }
}
