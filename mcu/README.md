# EDDI-MCU — C port for STM32H745/755

C port of the EDDI-FPGA algorithms (`fpga/rtl/`) for the dual-core
STM32H745/755 (Cortex-M7 @480MHz + Cortex-M4 @240MHz), replacing the
PolarFire FPGA target after the 2026-07-20 hardware pivot. See
`CLAUDE.md` and the project memory for why.

## Layout

```
src/common/   hardware-independent algorithm modules (no HAL, no board deps)
test/         host-side (PC/gcc) self-checking tests for src/common/
```

## Status (v0.1, 2026-07-20)

| Module | Ports from | Status |
|---|---|---|
| `debris_detector.c/h` | `fpga/rtl/debris_detector.v` | Tested, passing |
| `framed_protocol.c/h` | `fpga/rtl/payload_if.v` (framing/CRC only) | Tested, passing |
| `qtn_pipeline.c/h` | `fpga/rtl/qtn_core.v` + `psd_avg.v` | Tested, passing |
| `fft_backend_ref.c` | -- (host-test-only reference FFT) | Tested, passing |
| `fft_backend_cmsis.c` | -- | **Not written** |

Run tests: `cd test && mingw32-make test` (or `make test` if `make` is
on PATH instead of `mingw32-make`). Builds against gcc for x86 to
verify the algorithms on a PC — same role `fpga/sim/run_all.sh` plays
for the RTL. Not an embedded build.

`test_qtn`'s measured peak power (13,591,814) landed within 0.015% of
the RTL's own `tb_qtn` result (13,589,813) on an independent stimulus
seed — a real cross-check that the fixed-point math ported faithfully,
not just a code-reads-ok claim.

## What's deliberately not done yet

- **`fft_backend_cmsis.c`** — the real target FFT backend (ARM
  CMSIS-DSP, `arm_rfft_fast_q15` or similar). Not written because I
  don't have the CMSIS-DSP headers/library available to verify calling
  conventions against — writing it blind would be a guess presented as
  working code. Do this once a real toolchain/board is in hand.
- **HAL / board bring-up** — clock config, IWDG watchdog, UART
  peripheral, GPIO. Needs a dev board decision first (Nucleo-H745ZI-Q?
  custom carrier?) since pin mapping is board-specific.
- **Inter-core communication** (M7 <-> M4 handoff — shared memory +
  HSEM hardware semaphores is ST's recommended approach for H7 dual-core,
  per `AN5557`). Not started.
- **The load-bearing open risk: ADC data ingestion.** Getting the
  LTC2165's 40 Msps x 16-bit stream into the MCU has no confirmed
  answer yet — STM32H7's DCMI/FMC peripherals are built for
  camera-style pixel clocks with blanking gaps, not a continuously
  saturated 40 MHz stream. This should be resolved (parallel-CMOS-mode
  DMA capture feasibility check, or an alternate approach) before
  investing further in application-layer C code, since it's the one
  thing that could make this whole pivot infeasible regardless of which
  STM32H7 variant or board gets picked.
- **Radiation mitigation** — cross-core checksum verification, memory
  scrubbing of critical state, ECC-RAM (if available on the chosen
  variant), flash integrity CRC. Real techniques identified (see
  project memory, citing arXiv:1903.08781) but not yet implemented.
- **`burst_ctrl` equivalent** (QTN amp power-gating duty cycle) and
  **`watchdog.v` equivalent** (maps to STM32's IWDG peripheral, trivial
  once board bring-up exists) — not started, both are thin/mechanical
  once the board is chosen.

## Fixed-point conventions (unchanged from the RTL)

Same as `fpga/README.md`: samples signed 16-bit, Hann/twiddle-style
coefficients Q1.15, FFT output = FFT(x)/N, PSD = re^2+im^2 accumulated
over a power-of-two frame count. `gen_vectors.py`'s golden-model
methodology is the reference for "is this bit-exact where it needs to
be" — the QTN pipeline's windowing math replicates it exactly (`(sample
* coeff) >>> 15`, arithmetic right shift, not truncating division).
