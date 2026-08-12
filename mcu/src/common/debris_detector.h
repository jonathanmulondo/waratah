/* debris_detector.h -- time-domain debris impact detector (EDDI-MCU)
 *
 * Direct C port of fpga/rtl/debris_detector.v's algorithm, for the
 * STM32H745/755 M4 core. Same physics, same math, same state machine --
 * only the substrate changed (per-clock RTL update -> per-sample C call).
 * See fpga/docs/eddi_walkthrough.pdf for the "why" behind each check
 * (4-sigma without a square root, zero-crossing frequency qualification,
 * degenerate-background bypass, etc.) -- that reasoning is unchanged.
 *
 * Hardware-independent: no HAL, no peripheral access. The caller feeds
 * one ADC sample per call via debris_detector_step().
 */
#ifndef EDDI_DEBRIS_DETECTOR_H
#define EDDI_DEBRIS_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

#define DEBRIS_CAL_LEN      2048u   /* LOG2CAL=11 in the RTL */
#define DEBRIS_PRE_LEN      512u
#define DEBRIS_POST_LEN     1024u
#define DEBRIS_BUF_LEN      (DEBRIS_PRE_LEN + DEBRIS_POST_LEN)  /* 1536 */
#define DEBRIS_QUAL_LEN     128u
#define DEBRIS_VAR_FLOOR    4u
#define DEBRIS_QUAL_MARGIN  8u
#define DEBRIS_RECAL_INTERVAL   3000000u  /* ~60 s at 50 ksps */
#define DEBRIS_READY_TIMEOUT    250000u   /* ~5 s at 50 ksps */

typedef enum {
    DEBRIS_S_CAL_MEAN = 0,
    DEBRIS_S_CAL_VAR,
    DEBRIS_S_ARMED,
    DEBRIS_S_POST,
    DEBRIS_S_READY
} debris_state_t;

typedef struct {
    debris_state_t state;

    /* capture buffer: circular, always recording except while frozen */
    int16_t buf_ram[DEBRIS_BUF_LEN];
    uint16_t wptr;
    uint16_t trig_wptr;

    /* calibration accumulators */
    uint32_t cal_cnt;
    int32_t  sum;
    uint64_t sumsq;

    /* calibration results (also housekeeping/telemetry) */
    int16_t  cal_mean;
    uint32_t cal_var;
    uint16_t cal_zc;

    /* detection bookkeeping */
    uint64_t abs_idx;
    uint16_t armed_fill;
    uint16_t post_cnt;
    uint16_t event_zc;
    uint32_t recal_cnt;
    uint32_t ready_cnt;
    bool     sign_d;

    /* outputs */
    bool     event_ready;
    uint16_t event_count;
    uint8_t  events_dropped;
    uint64_t trig_index;
} debris_detector_t;

void debris_detector_init(debris_detector_t *d);

/* Feed one ADC sample. Call at the debris sample rate (~50 ksps). */
void debris_detector_step(debris_detector_t *d, int16_t sample);

/* Release a frozen event and re-arm (mirrors the RTL's `ack` port). */
void debris_detector_ack(debris_detector_t *d);

/* Read one sample from the frozen event capture (logical address
 * 0..DEBRIS_BUF_LEN-1, trigger sample sits at DEBRIS_PRE_LEN). Only
 * meaningful while d->event_ready is true. */
int16_t debris_detector_read_buf(const debris_detector_t *d, uint16_t addr);

#endif
