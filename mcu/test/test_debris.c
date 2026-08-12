/* test_debris.c -- self-checking test for debris_detector.c
 *
 * Same shape as fpga/tb/tb_debris.v: quiet noise background, one
 * injected impact transient, check the detector triggers on it and
 * doesn't false-trigger on the noise.
 */
#include "../src/common/debris_detector.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* small deterministic PRNG -- repeatability matters more than
 * statistical quality for this test */
static uint32_t rng_state = 12345;
static int32_t rng_uniform(int32_t lo, int32_t hi)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    uint32_t r = rng_state >> 8;
    return lo + (int32_t)(r % (uint32_t)(hi - lo + 1));
}

int main(void)
{
    debris_detector_t d;
    debris_detector_init(&d);

    int errors = 0;
    const int32_t baseline = 1000;
    const int32_t SPIKE_AT = 20000;
    const double  SPIKE_AMP = 2000.0;
    const double  SPIKE_TAU_SAMPLES = 200.0;   /* 4ms @ 50ksps */
    const double  SPIKE_FREQ_HZ = 1500.0;
    const double  FS = 50000.0;

    bool triggered_early = false;
    int  trigger_sample = -1;

    for (int i = 0; i < 45000; i++) {
        int32_t noise = rng_uniform(-100, 100);
        double sample = (double)baseline + (double)noise;

        if (i >= SPIKE_AT && i < SPIKE_AT + 4000) {
            double t = (double)(i - SPIKE_AT) / FS;
            sample += SPIKE_AMP * exp(-t * FS / SPIKE_TAU_SAMPLES)
                      * sin(2.0 * M_PI * SPIKE_FREQ_HZ * t);
        }

        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;

        debris_detector_step(&d, (int16_t)sample);

        if (d.event_ready && trigger_sample < 0) {
            trigger_sample = i;
            if (i < SPIKE_AT) triggered_early = true;
        }
    }

    if (triggered_early) {
        printf("FAIL: false trigger before the injected impact\n");
        errors++;
    }
    if (trigger_sample < 0) {
        printf("FAIL: never triggered on the injected impact\n");
        errors++;
    } else if (trigger_sample < SPIKE_AT || trigger_sample > SPIKE_AT + 2000) {
        printf("FAIL: trigger at sample %d, expected shortly after %d\n",
               trigger_sample, SPIKE_AT);
        errors++;
    }
    if (d.events_dropped != 0) {
        printf("FAIL: %u events dropped (unexpected in this test)\n",
               d.events_dropped);
        errors++;
    }

    /* the capture buffer should contain the impact's peak amplitude
     * somewhere around the trigger point (logical addr PRE_LEN) */
    if (trigger_sample >= 0) {
        int16_t peak = 0;
        for (uint16_t a = 0; a < DEBRIS_BUF_LEN; a++) {
            int16_t v = debris_detector_read_buf(&d, a);
            if (v > peak) peak = v;
        }
        if (peak < 1000) {
            printf("FAIL: captured buffer peak %d, expected a large impact amplitude\n",
                   peak);
            errors++;
        } else {
            printf("note: captured buffer peak = %d LSB\n", peak);
        }
    }

    if (errors == 0) {
        printf("TB_PASS: test_debris (trigger at sample %d, event_count=%u)\n",
               trigger_sample, d.event_count);
        return 0;
    } else {
        printf("TB_FAIL: test_debris (%d errors)\n", errors);
        return 1;
    }
}
