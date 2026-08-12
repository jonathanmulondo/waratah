/* qtn_pipeline.h -- ionosphere QTN pipeline (EDDI-MCU)
 *
 * Port of fpga/rtl/qtn_core.v + psd_avg.v: Hann window -> FFT -> |X|^2
 * -> mean over NAVG frames. Same Q1.15 fixed-point conventions as the
 * RTL (and gen_vectors.py's golden model) so results are comparable
 * bin-for-bin. FFT itself goes through fft_backend.h, not called here
 * directly -- see that header for why.
 */
#ifndef EDDI_QTN_PIPELINE_H
#define EDDI_QTN_PIPELINE_H

#include <stdint.h>
#include <stdbool.h>

#define QTN_LOG2N   10
#define QTN_N       (1 << QTN_LOG2N)      /* 1024 */
#define QTN_NBINS   (QTN_N / 2 + 1)       /* 513 */
#define QTN_LOG2AVG 2
#define QTN_NAVG    (1 << QTN_LOG2AVG)    /* 4 frames averaged */

typedef struct {
    int16_t  hann[QTN_N];
    int16_t  frame_buf[QTN_N];
    int      frame_pos;
    uint64_t psd_acc[QTN_NBINS];
    int      frame_idx;      /* 0..QTN_NAVG-1, which frame of the average */
    bool     done;           /* NAVG frames accumulated, ready to read */
} qtn_pipeline_t;

void qtn_pipeline_init(qtn_pipeline_t *q);

/* Feed one raw ADC sample. Call at the QTN sample rate. Internally
 * batches into QTN_N-sample frames and runs the window+FFT+accumulate
 * step once a frame is full. Ignored (matches sample_ready gating in
 * the RTL) once q->done is true -- caller must qtn_pipeline_restart()
 * after reading the spectrum. */
void qtn_pipeline_push_sample(qtn_pipeline_t *q, int16_t sample);

/* Start a new averaging period. Only meaningful once q->done is true
 * (mirrors the RTL's restart_eff -- clearing mid-average would mix old
 * and new data). */
void qtn_pipeline_restart(qtn_pipeline_t *q);

/* Mean PSD for one bin (0..QTN_N/2), valid while q->done is true. */
uint32_t qtn_pipeline_read_psd(const qtn_pipeline_t *q, int bin);

#endif
