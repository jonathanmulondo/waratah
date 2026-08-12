/* fft_backend_ref.c -- portable double-precision FFT, for host-side
 * testing only (see fft_backend.h). Correctness bar here is "matches
 * the golden model within tolerance" -- same standard the RTL
 * testbenches used -- not bit-exact fixed-point reproduction, since
 * that concern (avoiding synthesis surprises) doesn't apply off-target.
 * Standard iterative radix-2 Cooley-Tukey, no cleverness.
 */
#include "fft_backend.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846  /* not standard C; MinGW gates it behind _USE_MATH_DEFINES */
#endif

static int log2_int(int n)
{
    int l = 0;
    while ((1 << l) < n) l++;
    return l;
}

static unsigned bit_reverse(unsigned v, int bits)
{
    unsigned r = 0;
    for (int i = 0; i < bits; i++) {
        r = (r << 1) | (v & 1u);
        v >>= 1;
    }
    return r;
}

static int16_t quantize_q15(double v)
{
    double r = v < 0 ? v - 0.5 : v + 0.5;   /* round half away from zero */
    if (r > 32767.0)  return 32767;
    if (r < -32768.0) return -32768;
    return (int16_t)r;
}

void fft_backend_transform(const int16_t *windowed_in,
                            int16_t *out_re, int16_t *out_im, int n)
{
    int bits = log2_int(n);
    double *re = (double *)malloc((size_t)n * sizeof(double));
    double *im = (double *)malloc((size_t)n * sizeof(double));

    for (int i = 0; i < n; i++) {
        unsigned j = bit_reverse((unsigned)i, bits);
        re[j] = (double)windowed_in[i];
        im[j] = 0.0;
    }

    for (int stage = 1; stage <= bits; stage++) {
        int m = 1 << stage;
        int half = m / 2;
        double theta = -2.0 * M_PI / (double)m;
        for (int base = 0; base < n; base += m) {
            for (int k = 0; k < half; k++) {
                double c = cos(theta * k);
                double s = sin(theta * k);
                int a = base + k;
                int b = base + k + half;
                double tr = re[b] * c - im[b] * s;
                double ti = re[b] * s + im[b] * c;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] = re[a] + tr;
                im[a] = im[a] + ti;
            }
        }
    }

    /* output = FFT(x)/N, matching the RTL convention */
    for (int k = 0; k <= n / 2; k++) {
        out_re[k] = quantize_q15(re[k] / (double)n);
        out_im[k] = quantize_q15(im[k] / (double)n);
    }

    free(re);
    free(im);
}
