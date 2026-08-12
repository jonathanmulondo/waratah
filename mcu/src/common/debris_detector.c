/* debris_detector.c -- see debris_detector.h. Ported line-for-line in
 * spirit from fpga/rtl/debris_detector.v -- read that file's comments
 * for the physics/math reasoning; this keeps the same variable names
 * and structure so the two stay easy to cross-check.
 */
#include "debris_detector.h"

#define ZC_SHIFT (11 - 7)  /* LOG2CAL - log2(QUAL_LEN); QUAL_LEN=128=2^7 */

void debris_detector_init(debris_detector_t *d)
{
    *d = (debris_detector_t){0};
    d->state = DEBRIS_S_CAL_MEAN;
}

static uint16_t wptr_next(uint16_t wptr)
{
    return (wptr == DEBRIS_BUF_LEN - 1) ? 0 : (uint16_t)(wptr + 1);
}

void debris_detector_step(debris_detector_t *d, int16_t sample)
{
    if (d->state == DEBRIS_S_READY) {
        /* frozen: buffer stops, only the ack-timeout clock still runs */
        if (d->ready_cnt + 1 >= DEBRIS_READY_TIMEOUT) {
            /* never freeze forever waiting on the payload computer */
            d->event_ready = false;
            d->events_dropped++;
            d->armed_fill = 0;
            d->state = DEBRIS_S_ARMED;
        } else {
            d->ready_cnt++;
        }
        return;
    }

    /* things that happen on every sample outside S_READY */
    d->buf_ram[d->wptr] = sample;
    d->wptr = wptr_next(d->wptr);
    d->abs_idx++;

    int32_t diff = (int32_t)sample - (int32_t)d->cal_mean;
    bool sign_now = diff < 0;
    bool zc = (sign_now != d->sign_d);
    d->sign_d = sign_now;

    uint64_t diff_sq = (uint64_t)((int64_t)diff * (int64_t)diff);

    switch (d->state) {

    case DEBRIS_S_CAL_MEAN: {
        d->sum += sample;
        if (d->cal_cnt == DEBRIS_CAL_LEN - 1) {
            /* arithmetic (sign-extending) right shift on ARM GCC --
             * matches the RTL's `sum_nxt >>> LOG2CAL`, not truncating
             * division, so negative means round the same way */
            d->cal_mean = (int16_t)(d->sum >> 11);
            d->cal_cnt  = 0;
            d->sumsq    = 0;
            d->cal_zc   = 0;
            d->state    = DEBRIS_S_CAL_VAR;
        } else {
            d->cal_cnt++;
        }
        break;
    }

    case DEBRIS_S_CAL_VAR: {
        d->sumsq += diff_sq;
        d->cal_zc = (uint16_t)(d->cal_zc + (zc ? 1 : 0));
        if (d->cal_cnt == DEBRIS_CAL_LEN - 1) {
            uint64_t var_nxt = d->sumsq >> 11;
            /* saturate: a pathological input swing must not wrap the
             * variance to a tiny value and cause a trigger storm */
            d->cal_var    = (var_nxt > 0xFFFFFFFFu) ? 0xFFFFFFFFu
                                                     : (uint32_t)var_nxt;
            d->cal_cnt    = 0;
            d->armed_fill = 0;
            d->recal_cnt  = 0;
            d->state      = DEBRIS_S_ARMED;
        } else {
            d->cal_cnt++;
        }
        break;
    }

    case DEBRIS_S_ARMED: {
        d->recal_cnt++;
        if (d->recal_cnt >= DEBRIS_RECAL_INTERVAL) {
            /* periodic recalibration -- tracks thermal/orbital drift of
             * the DC operating point, same as the flight firmware did
             * every minute */
            d->sum       = 0;
            d->cal_cnt   = 0;
            d->recal_cnt = 0;
            d->state     = DEBRIS_S_CAL_MEAN;
        } else if (d->armed_fill < DEBRIS_PRE_LEN) {
            /* don't trigger until the circular buffer holds a full
             * PRE_LEN of genuine pre-trigger samples */
            d->armed_fill++;
        } else {
            uint32_t var_eff = (d->cal_var < DEBRIS_VAR_FLOOR)
                                    ? DEBRIS_VAR_FLOOR : d->cal_var;
            uint64_t thresh  = (uint64_t)var_eff << 4;  /* 16*var = (4sigma)^2 */
            if (diff_sq > thresh) {
                d->trig_wptr  = (d->wptr == 0) ? (uint16_t)(DEBRIS_BUF_LEN - 1)
                                               : (uint16_t)(d->wptr - 1);
                /* wptr already advanced past this sample above, so back
                 * up one to point at the sample that just triggered */
                d->trig_index = d->abs_idx - 1;
                d->post_cnt   = 0;
                d->event_zc   = 0;
                d->state      = DEBRIS_S_POST;
            }
        }
        break;
    }

    case DEBRIS_S_POST: {
        d->recal_cnt++;
        if (d->post_cnt < DEBRIS_QUAL_LEN)
            d->event_zc = (uint16_t)(d->event_zc + (zc ? 1 : 0));
        /* the trigger sample already occupies one POST_LEN slot, so
         * capture POST_LEN-1 further samples -- one later and the
         * oldest pre-trigger sample would be overwritten */
        if (d->post_cnt == DEBRIS_POST_LEN - 2) {
            uint32_t bg_zc_scaled = (uint32_t)d->cal_zc >> ZC_SHIFT;
            uint32_t event_zc_x3  = (uint32_t)d->event_zc * 3;
            bool bg_degen = bg_zc_scaled < DEBRIS_QUAL_MARGIN;
            bool qualify  = bg_degen || (event_zc_x3 < bg_zc_scaled);

            if (qualify) {
                d->event_ready = true;
                d->event_count++;
                d->ready_cnt = 0;
                d->state = DEBRIS_S_READY;
            } else {
                /* loud but fast = interference, not debris: discard
                 * silently and refill the pre-buffer */
                d->armed_fill = 0;
                d->state = DEBRIS_S_ARMED;
            }
        }
        d->post_cnt++;
        break;
    }

    case DEBRIS_S_READY:
        /* handled above before the per-sample block */
        break;
    }
}

void debris_detector_ack(debris_detector_t *d)
{
    if (d->state == DEBRIS_S_READY) {
        d->event_ready = false;
        d->armed_fill  = 0;
        d->state       = DEBRIS_S_ARMED;
    }
}

int16_t debris_detector_read_buf(const debris_detector_t *d, uint16_t addr)
{
    /* logical addr 0 maps to trig_wptr - PRE_LEN, wrapped mod BUF_LEN
     * (BUF_LEN=1536 is not a power of two, so wrap explicitly) */
    uint32_t base_raw = (uint32_t)d->trig_wptr + DEBRIS_BUF_LEN - DEBRIS_PRE_LEN;
    uint32_t base      = (base_raw >= DEBRIS_BUF_LEN) ? base_raw - DEBRIS_BUF_LEN
                                                       : base_raw;
    uint32_t rd_raw    = base + addr;
    uint32_t rd_phys   = (rd_raw >= DEBRIS_BUF_LEN) ? rd_raw - DEBRIS_BUF_LEN
                                                     : rd_raw;
    return d->buf_ram[rd_phys];
}
