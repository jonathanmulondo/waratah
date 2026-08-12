/* test_qtn.c -- self-checking test for qtn_pipeline.c
 *
 * Same shape as fpga/tb/tb_qtn.v: a tone at a known bin plus noise,
 * averaged over NAVG frames, check the PSD peak lands on that bin and
 * a distant bin stays quiet.
 */
#include "../src/common/qtn_pipeline.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static uint32_t rng_state = 42;
static int32_t rng_uniform(int32_t lo, int32_t hi)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    uint32_t r = rng_state >> 8;
    return lo + (int32_t)(r % (uint32_t)(hi - lo + 1));
}

int main(void)
{
    qtn_pipeline_t q;
    qtn_pipeline_init(&q);

    const int TONE_BIN = 77;
    const double TONE_AMP = 0.45 * 32767.0;

    for (int f = 0; f < QTN_NAVG; f++) {
        for (int n = 0; n < QTN_N; n++) {
            double x = TONE_AMP * sin(2.0 * M_PI * TONE_BIN * n / QTN_N
                                       + 0.7 * f);
            x += (double)rng_uniform(-64, 64);
            if (x > 32767) x = 32767;
            if (x < -32768) x = -32768;
            qtn_pipeline_push_sample(&q, (int16_t)x);
        }
    }

    int errors = 0;
    if (!q.done) {
        printf("FAIL: pipeline never reached done after %d frames\n", QTN_NAVG);
        return 1;
    }

    uint32_t peak_val = 0;
    int peak_bin = -1;
    for (int b = 1; b <= QTN_N / 2; b++) {
        uint32_t v = qtn_pipeline_read_psd(&q, b);
        if (v > peak_val) { peak_val = v; peak_bin = b; }
    }

    if (peak_bin != TONE_BIN) {
        printf("FAIL: PSD peak at bin %d, expected %d\n", peak_bin, TONE_BIN);
        errors++;
    }

    uint32_t quiet = qtn_pipeline_read_psd(&q, 400);
    if (quiet > peak_val / 20) {
        printf("FAIL: quiet bin 400 power %u too close to peak %u\n",
               quiet, peak_val);
        errors++;
    }

    if (errors == 0) {
        printf("TB_PASS: test_qtn (peak bin %d, power %u, quiet %u)\n",
               peak_bin, peak_val, quiet);
        return 0;
    } else {
        printf("TB_FAIL: test_qtn (%d errors)\n", errors);
        return 1;
    }
}
