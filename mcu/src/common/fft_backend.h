/* fft_backend.h -- swappable FFT implementation for qtn_pipeline
 *
 * qtn_pipeline.c doesn't call an FFT library directly -- it calls this
 * interface, so the same pipeline code runs against a portable
 * reference FFT on a PC (for host-side testing, see fft_backend_ref.c)
 * and against CMSIS-DSP's hardware-accelerated FFT on the real M7 core
 * (fft_backend_cmsis.c -- NOT YET WRITTEN, see mcu/README.md; I'm not
 * writing untested CMSIS-DSP calling code against a library I can't
 * currently build against, so that's next once real hardware/toolchain
 * is in hand rather than a guess presented as working).
 */
#ifndef EDDI_FFT_BACKEND_H
#define EDDI_FFT_BACKEND_H

#include <stdint.h>

/* N-point FFT of real Q1.15 input. Outputs bins 0..n/2 (the RTL's
 * "real input -> conjugate-symmetric spectrum" convention -- only the
 * non-redundant half is produced). out_re/out_im must each hold
 * n/2 + 1 entries. Scaling: output = FFT(x)/n, same convention as
 * fft_r2.v, so downstream PSD/averaging code doesn't need to care
 * which backend produced it. */
void fft_backend_transform(const int16_t *windowed_in,
                            int16_t *out_re, int16_t *out_im, int n);

#endif
