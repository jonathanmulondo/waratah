# EDDI-FPGA — Verilog RTL

FPGA implementation of the EDDI instrument's digital signal processing,
replacing the LPC4370 microcontroller of the flight design. Target:
**Microchip PolarFire MPF300T-1FCG1152I**. ADC: LTC2165 or TI ADS5560
(16-bit, undecided). All RTL is vendor-neutral Verilog-2001 and simulates
identically in Questa, Quartus, and Libero/ModelSim.

## Layout

```
rtl/   synthesizable modules
tb/    self-checking testbenches (print TB_PASS / TB_FAIL)
sim/   gen_vectors.py (numpy golden vectors + ROM init), run script, mem/
```

## Modules (v0.2)

| Module              | Function | Status |
|---------------------|----------|--------|
| `fft_r2.v`          | Radix-2 DIT iterative FFT, param LOG2N (512/1024-pt), in-place, 1/N scaled, 5 clk/butterfly | tested |
| `psd_avg.v`         | \|X\|² + mean over 2^LOG2AVG frames, bins 0..N/2, 32-bit readout | tested |
| `qtn_core.v`        | **Ionosphere module**: Hann window → FFT → averaged PSD (flight QTN mode) | tested |
| `debris_detector.v` | **Debris module**: 2-phase calibration, 4σ + crossing-rate trigger, 512-pre/1024-post capture (flight debris mode) | tested |
| `watchdog.v`        | Timeout reset with sticky fired count (gap flagged in the thesis) | tested |
| `burst_ctrl.v`      | QTN burst/cadence controller: amp power-gating, settle, one spectrum per collection (v0.2) | tested (`tb_burst`) |
| `cic_decim.v`       | 3-stage CIC decimator, 800:1 (40 Msps → 50 ksps) for the debris path off a shared ADC (v0.2) | tested (`tb_cic`) |
| `payload_if.v`      | Framed command/telemetry link (SOF/CMD/LEN/payload/CRC-8, ACK/NACK, heartbeat) over UART (v0.2) | tested (`tb_payload`) |
| `uart.v`            | 8N1 `uart_rx` + `uart_tx` used by `payload_if` (v0.2) | via `tb_payload` / `tb_top` |
| `eddi_top.v`        | Top level: QTN and debris paths concurrent off one ADC bus, payload link, watchdog (v0.2) | tested (`tb_top`) |

## Running the simulations (Questa CLI)

```sh
cd fpga/sim
sh run_all.sh            # all testbenches
sh run_all.sh tb_fft     # a single one
```

`run_all.sh` regenerates the golden vectors (`python gen_vectors.py`),
compiles with `vlog`, runs each bench with `vsim -c`, and greps for
TB_PASS. Logs land in `sim/<tb>.log`. Always run from `fpga/sim` — the
`$readmemh` paths (`mem/*.mem`) are relative to it.

Verification results (v0.1): FFT matches numpy bin-for-bin (513 bins,
tol ≈ ±24 LSB + 1.5%); QTN mean-PSD peak power within 0.01% of the float
golden; debris trigger fires 1 sample after the injected impact with a
correctly frozen pre/post capture buffer.

## Fixed-point conventions

- Samples: signed 16-bit (ADC LSB). Hann coeffs and twiddles: Q1.15.
- FFT halves each stage (round-half-up, saturating) → output = FFT/N.
- PSD: re²+im² in LSB², 33-bit, averaged over a power-of-two frame count.
- gen_vectors.py replicates the RTL bit-exactly for the window step and
  uses float FFT/N for the golden spectrum (tolerance-compared).

## Code review (v0.1.1)

An independent review pass (2026-07-08) confirmed the QTN/FFT pipeline
math and timing, and found four robustness defects, all fixed and
re-verified in sim:

1. Degenerate calibration (quiet/DC input → variance and crossing rate
   of 0) could permanently disable debris detection → variance floor
   (`VAR_FLOOR`) + degenerate-safe qualification (`bg_degen` bypass).
2. No recalibration path (flight FW recalibrated every minute) →
   periodic re-cal (`RECAL_INTERVAL`, default ~60 s at 50 ksps).
3. A missed payload-computer `ack` froze the detector forever →
   `READY_TIMEOUT` auto-drop with an `events_dropped` telemetry counter.
4. One-sample handshake leak at the end of a QTN averaging run (masked
   by hann[0]=0) → explicit `all_loaded` gate on `sample_ready`.

Plus smaller hardening: 48-bit `trig_index` (32-bit wrapped in <24 h at
50 ksps), saturating `cal_var`, signed-safe mean arithmetic, watchdog
kick edge-detection + `>=` timeout compare, corrected read-latency docs
(PSD readout is 2 cycles), and a testbench read race in tb_debris.

**Known open item:** `initial $readmemh` ROM init must be validated in
the Libero synthesis trial — PolarFire LSRAM does not power up
initialized like SRAM-FPGA block RAM; if Synplify doesn't map the
Hann/twiddle ROMs with their init data (sNVM/uPROM flow or logic ROMs),
hardware would read zeros while sims stay green.

## Roadmap (v0.2+)

- Per-bin standard deviation in `psd_avg` (flight FW parity) — second
  sum-of-squares RAM.
- Runtime-selectable 512/1024 FFT (currently a synthesis parameter).
- ADC interface module: LVDS DDR deserialiser for LTC2165/ADS5560;
  (The debris-path decimation tap is done: `cic_decim.v`, v0.2.)
  Constraints from the analog front-end design (LMH6629 chain, QTN path,
  per its designer 2026-07-10):
  - **Effective sample rate locked at 40 Msps.** The chain's anti-alias
    LPF is designed for a 20 MHz Nyquist; sampling slower would alias
    the 17.5–20+ MHz region into band. Faster is safe only as
    oversample-then-decimate back to 40 Msps effective.
  - **Select two's-complement output mode** on the ADC (LTC2165 is
    pin-selectable offset-binary/two's-complement); the whole pipeline
    assumes signed 16-bit. Forgetting this produces garbage spectra
    that still simulate green.
  - Front end re-biased to the LTC2165's 0.9 V VCM output; residual DC
    offset lands in FFT bins 0–1, below the 100 kHz science band — no
    RTL impact, but expect a bin-0 spike in raw spectra.
  - Passband is flat 230 Hz–17.5 MHz (~72 dB gain, ~300 mW, single 5 V);
    bins above 17.5 MHz see the filter roll-off — ground-calibration
    fact, not an RTL one.
  - Gain rebalance pending: LTC2165 LSB ≈ 30 µV (2 Vpp / 16 bit) vs
    ~200 µV on the LPC4370, so some of the 72 dB analog gain can likely
    be shed. Bin widths to quote for SNR sizing: ~39 kHz (1024-pt) /
    ~78 kHz (512-pt) at 40 Msps, 4 frames averaged per spectrum.
- Payload-computer interface: the UART framed link with ACK/NACK +
  heartbeat is done (`payload_if.v`, v0.2); an I2C transport, if the bus
  needs one, is not.
- Storage module (SD/SPI or external flash, raw 512-B block writes).
- Libero synthesis trial of `eddi_top.v` (done in sim, v0.2) on the
  MPF300T (math-block/LSRAM mapping, timing closure at target clock).
