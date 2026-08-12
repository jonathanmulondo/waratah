/* qtn_pipeline.c -- see qtn_pipeline.h. */
#include "qtn_pipeline.h"
#include "fft_backend.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void qtn_pipeline_init(qtn_pipeline_t *q)
{
    *q = (qtn_pipeline_t){0};
    /* same formula as gen_vectors.py's hann ROM, so results line up
     * bin-for-bin with the RTL's golden vectors */
    for (int n = 0; n < QTN_N; n++) {
        double w = 0.5 * (1.0 - cos(2.0 * M_PI * (double)n / (double)QTN_N));
        double v = w * 32767.0;
        q->hann[n] = (int16_t)(v < 0 ? v - 0.5 : v + 0.5);
    }
}

void qtn_pipeline_restart(qtn_pipeline_t *q)
{
    if (q->done) {
        q->frame_idx = 0;
        q->done = false;
    }
}

void qtn_pipeline_push_sample(qtn_pipeline_t *q, int16_t sample)
{
    if (q->done)
        return;   /* mirrors sample_ready gating -- won't accept until restart */

    /* Q1.15 window multiply, truncating shift (matches the RTL's
     * `prod >>> 15`, replicated bit-exactly by gen_vectors.py too) */
    int32_t prod = (int32_t)sample * (int32_t)q->hann[q->frame_pos];
    q->frame_buf[q->frame_pos] = (int16_t)(prod >> 15);
    q->frame_pos++;

    if (q->frame_pos == QTN_N) {
        q->frame_pos = 0;

        int16_t fft_re[QTN_NBINS], fft_im[QTN_NBINS];
        fft_backend_transform(q->frame_buf, fft_re, fft_im, QTN_N);

        bool first = (q->frame_idx == 0);
        for (int b = 0; b < QTN_NBINS; b++) {
            uint64_t p = (uint64_t)((int64_t)fft_re[b] * fft_re[b])
                       + (uint64_t)((int64_t)fft_im[b] * fft_im[b]);
            /* frame 0 seeds the accumulator instead of adding to it --
             * this is what makes restart cheap, no need to spend
             * QTN_NBINS cycles zeroing the array first */
            q->psd_acc[b] = first ? p : q->psd_acc[b] + p;
        }

        q->frame_idx++;
        if (q->frame_idx == QTN_NAVG)
            q->done = true;
    }
}

uint32_t qtn_pipeline_read_psd(const qtn_pipeline_t *q, int bin)
{
    uint64_t mean = q->psd_acc[bin] >> QTN_LOG2AVG;
    return (mean > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)mean;
}
