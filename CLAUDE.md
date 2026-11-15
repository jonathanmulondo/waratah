# LPC4370 — CubeSat FPGA Instrument Project

**This project is fully independent from all of Jonathan's other projects** (Hardwire, content
work, etc.). Do not pull in context from other folders on this machine.

## The Vision

Build the next generation of a CubeSat-borne ionospheric plasma + space debris instrument,
moving the signal processing from the current LPC4370 microcontroller onto an **FPGA
(programmed in Verilog)**. The reference design is **EDDI (Electron Density and Debris
Instrument)**, developed at the University of Sydney / CUAVA and flown on the CUAVA-2 and
Waratah Seed-1 (WS-1) CubeSats in August 2024. The three PDFs in this folder are the full
background:

1. **Quinn Musulin's MPhil thesis (2026, USYD)** — the deep technical reference. Design,
   build, and flight results of EDDI. This is the document this project builds on.
2. **SmallSat 2024 paper (SSC24-S4-04)** — WS-1 satellite bus, payloads, ground segment.
3. **IAC-25 paper (IAC-25,B4,6B,3,x97520)** — WS-1 first-year in-orbit results and lessons.

## What EDDI Does (the science)

A **passive, low-power electric-field sensor** on a CubeSat in LEO (~500 km, F-region of the
ionosphere, sun-synchronous orbit). Two measurement techniques, two goals:

- **Quasi-Thermal Noise (QTN) spectroscopy** (frequency domain): thermal Langmuir waves in
  the plasma produce a voltage spectrum on an antenna. The spectral peak sits near the local
  plasma frequency → gives **electron density**; peak shape/width → **electron temperature**.
  In a magnetised ionosphere the peak moves to the upper-hybrid frequency
  (f_uh = √(f_pe² + f_ce²)) with Bernstein-wave maxima between gyro-harmonic minima (Dazzi
  et al. 2025 model). Target band: **0.1–20 MHz** (ionospheric plasma frequencies ~1–10 MHz).
- **Debris impact detection** (time domain): sub-mm particles hitting the spacecraft at up to
  ~10 km/s vaporise into a plasma plume; charge separation (fast electrons, slow ions)
  induces a characteristic transient voltage spike (~1–25 kHz, sharp peak + slow opposite-
  polarity decay — same physics as STEREO/WAVES and Parker Solar Probe FIELDS dust
  detections). Goal: fill the sub-cm data gap in debris models (NASA ORDEM, ESA MASTER),
  which ground radar/optical can't see below ~1 cm.

Why it matters: real-time ionosphere data feeds over-the-horizon radar (e.g. Australia's
JORN needs the critical frequency), space-weather models (NOAA WAM-IPE), and HF comms;
debris population data feeds collision-risk / Kessler-syndrome modelling.

## EDDI Architecture As Flown (baseline to improve on)

Two hardware variants, both ¼U-class payloads:

- **Main variant** (flown on CUAVA-2): dipole antenna (two 150 mm arms — repurposed eyeglass
  arms, spring hinges, fishing-line + burn-resistor deployment, contact-switch deploy
  detection), fully-differential amp chain OPA859×2 → THS4541 → LMH6881 VGA (GPIO-set gain
  6–26 dB) → OPA837×2. ~71–75 dB gain, pass-band 100 kHz–20 MHz, RC anti-alias at ~20 MHz.
  CUAVA-2 died from a suspected radiation event before EDDI was commissioned — **no data**.
- **Debris variant** (flown on WS-1, called **SPaDeS** on that mission): monopole antenna +
  CubeSat chassis as the second arm (ground plane tied to chassis via mounting holes),
  single-ended chain OPA859×2 → RC → OPA837, ~60–68 dB gain, pass-band 10–100 kHz.
  **Still operating and returning data.** Note: the antenna never deployed (burn circuit or
  seized spring), but the chassis-as-antenna still yields usable spectra.

**Processing (what the FPGA would replace):** LPC-Link2 dev board, NXP **LPC4370**
(ARM Cortex-M4 @ 204 MHz + 2× Cortex-M0, 264 kB SRAM, built-in **12-bit 80 Msps ADC** —
the chip was chosen for that ADC; input range 100–900 mV, ~0.2 mV resolution). Code runs
from SRAM (linker-script relocation, dummy-main trick), stored in 1 MB W25Q80 SPI flash.
1 MB flash / Link2 board is discontinued; BGA-100 package is hard to hand-mount.

**Software modes** (single command byte: bits 0–2 runtime ×18 min, bits 3–4 VGA gain,
bits 5–7 mode):
- 4 QTN/FFT modes: 512 or 1024-point FFT, normal or low power. ADC bursts at **40 Msps**
  into a 64 kB DMA circular buffer (819.2 µs per burst); 4 random 1024-sample chunks per
  burst, Hann window, CMSIS FFT, |·|², recursive mean + SD → one averaged PSD + SD per
  second (2–4 kB/s). Op-amps power-gated to only be on during bursts (1.35 W peak,
  ~0.58 W average). Bin widths 40/80 kHz.
- 1 debris mode: continuous 50 ksps sampling with concurrent processing; calibrates
  quiescent mean/SD/zero-crossing rate each minute; saves events with amplitude > 4σ AND
  frequency < ⅓ of background, storing the 512-sample buffer plus neighbours.
- Storage: SD card over SPI, raw 512-byte writes at fixed addresses (FAT32 libs too big
  for SRAM). Comms: I2C slave to a BeagleBone Black payload computer (115200 baud,
  8.192 kB/s transfer — the bottleneck), SET/GET command protocol.

## Flight Results So Far (WS-1 debris variant, run in QTN mode)

- Detected Sydney AM stations on the ground (validates chain + 204 MHz/40 MHz clocks).
- Plasma chamber tests: detected electron-cyclotron-related peaks (~500 kHz) and harmonics.
- In orbit: day/night spectra over Sydney; full-orbit spectrograms (15 s bursts/minute).
  Fixed-frequency peaks (10 MHz, 8, 6, 4.5 MHz) are suspected spacecraft electrical noise;
  but broad peaks **above 10 MHz drift predictably with latitude** (double-peak structure
  near the south magnetic pole) — likely plasma/upper-hybrid features. Broadband vertical
  spikes may be debris impacts (debris mode tests planned). Ram-side density enhancement is
  another hypothesis being tested via attitude maneuvers.

## Known Weaknesses → Where This Project Comes In

The thesis's future-work chapter is effectively this project's starting requirements list:

1. **QTN and debris modes can't run simultaneously** — one ADC, two incompatible chains.
   The thesis explicitly names an **FPGA as the natural fit for the repetitive QTN/FFT
   pipeline**, with time-domain anomaly detection handled separately (M0 cores / ML / GPU
   were other options floated). This is the motivation for the Verilog/FPGA direction.
2. Single PCB integration needed: antenna + burn circuit + both amp chains + processor
   (a V2 EM exists but has a flash-programming fault).
3. External ADC(s) desirable: 16-bit (e.g. TI ADS5560, 0–3.3 V) → ~50 µV resolution vs
   0.2 mV now; frees processor choice from the LPC4370's built-in ADC.
4. Split-dipole idea: one dipole arm feeds both the differential QTN chain and the
   single-ended debris chain (chassis-grounded), Fig 4.2 of thesis.
5. Software gaps: no watchdog, no sleep modes, fragile 1-byte I2C command protocol (needs
   framed packets + ACK/NACK/heartbeat), fixed 1 Hz cadence (needs configurable duty cycle
   for UHF-only downlink budgets of ~5 kB/pass).
6. Antenna deployment reliability (WS-1's never deployed; needs redesign + stowage tests).
7. Drop the LMH6881 VGA (noisy, 0.5 W) or replace with a simpler 2–3 step gain switch.

## Satellite/Bus Context (from the WS-1 papers)

WS-1: 6U CubeSat, ISIS iOBC + TrxVU UHF/VHF + Satlab SRS-4 S-band, CubeSpace ADCS
(1–3° pointing), 45 Wh EPS, ~7.44 Wh/orbit solar. Payload computer = BeagleBone Black
(custom CUAVA board), SLIP-over-UART to OBC; EDDI/SPaDeS connects to the OBC side.
Cube-OS (open-source KubOS fork, Rust HAL, UDP commanding, digital-twin ground CLI).
Operational realities that shape instrument design: S-band overheating outages (weeks-long),
X-axis reaction-wheel failure, UHF downlink only ~5 kB/pass, payloads must run one at a
time, everything must survive -10 to +40 °C thermal-vac and 5.58 Grms vibration; SD/flash
vulnerable to single-event upsets (CUAVA-2 was killed by a suspected radiation event —
radiation tolerance matters).

## Target Hardware (decided 2026-07-07)

- **FPGA: Microchip PolarFire MPF300T-1FCG1152I** — 300k logic elements, FCG1152 package,
  speed grade -1, industrial temp. Flash-based (SONOS) configuration → immune to
  configuration-memory SEUs, low static power; there is a radiation-tolerant sibling
  family (RT PolarFire) sharing the same fabric, which gives a flight-upgrade path.
  Toolchain: Microchip Libero SoC (SmartDesign, Synplify Pro, ModelSim/QuestaSim).
  Language: **Verilog**.
- **ADC: LTC2165 (Analog Devices/Linear, 16-bit, 125 Msps) or TI ADS5560 (16-bit, 40 Msps)**
  — final choice pending. Both are 16-bit (≈50 µV-class resolution vs the LPC4370's
  0.2 mV) with DDR LVDS data outputs (both also have parallel CMOS modes). ADS5560 was
  the thesis's suggestion; LTC2165 gives 3× the sample rate and headroom above the
  40 Msps the QTN mode needs (20 MHz band, Nyquist ×2).

## Planned RTL Module Architecture

Top-level Verilog design decomposed into independent modules (one per instrument
function), replacing the LPC4370 firmware's monolithic mode loop:

- **ADC interface module** — LVDS DDR deserialisation, sample framing, clocking (from the
  chosen ADC), feeding a common sample bus/FIFO.
- **Ionosphere (QTN) module** — windowing (Hann) → streaming FFT (512/1024 pt) → |·|² →
  recursive average PSD + SD, one spectrum/second cadence, configurable FFT size and
  averaging count (maps to the four legacy FFT modes).
- **Debris detection module** — time-domain path: quiescent mean/SD/zero-crossing
  calibration, 4σ-amplitude + low-frequency trigger criteria, event capture with pre/post
  buffers. Runs **concurrently** with the QTN module — the whole point of the FPGA move.
- **Payload-computer interface module** — command/telemetry link to the CubeSat payload
  computer (I2C slave and/or UART, per bus); implement the framed packet protocol with
  ACK/NACK + heartbeat recommended in the thesis, replacing the fragile 1-byte commands.
- **Storage module** — SD/SPI (or external flash) writer for PSD/SD arrays and debris
  events, address-based raw writes as in the flight firmware.
- Supporting: housekeeping/power-gating control (op-amp enables, VGA gain pins, burn
  circuit + deploy-switch GPIO), watchdog/reset, clock/reset infrastructure.

## Repo State

- **`fpga/`** — the Verilog RTL project (see `fpga/README.md`). v0.1 (2026-07): fft_r2,
  psd_avg, qtn_core (ionosphere), debris_detector, watchdog — all with self-checking
  testbenches passing in Questa. Golden vectors generated by `fpga/sim/gen_vectors.py`
  (numpy). Run sims from `fpga/sim` with `sh run_all.sh` (Questa `vlog`/`vsim -c` are on
  PATH from the Quartus 25.1 install at C:\altera_lite). Jonathan also runs tests in
  Quartus himself — keep RTL vendor-neutral Verilog-2001, no vendor primitives.
  v0.2 (2026-07-10) added burst_ctrl, cic_decim, payload_if (framed link) + uart, and
  eddi_top, each with a testbench in `fpga/tb/` that `run_all.sh` runs.
- **`mcu/`** — EDDI-MCU v0.1 (2026-08, see `mcu/README.md`): C port of the debris, framed
  protocol and QTN algorithms for a dual-core **STM32H745/755** (M7 QTN/FFT, M4 debris),
  from the pivot away from the PolarFire FPGA target (reasons in the commit message of
  `6784276`: native industrial temperature grade, still two cores running the two
  science paths in parallel). `mcu/src/common/` is hardware-independent; run the host
  tests with `cd mcu/test && make test` (gcc, x86; not an embedded build). Not written
  yet: CMSIS-DSP FFT backend, HAL/board bring-up, inter-core comms.

## Working Conventions

- Extracted plain-text copies of the three PDFs live in the session scratchpad; re-extract
  with `pypdf` if needed (system Python 3.13 has it installed).
- Jonathan's stated near-term focus: FPGA programming in **Verilog** for the instrument's
  DSP (the "Variable" in voice notes = Verilog). Ionosphere/QTN work and debris detection
  are the two application drivers.
- EDDI's existing C firmware: https://github.com/Quinn-Musulin/EDDI-Software
- Cube-OS: https://github.com/Cube-OS · LPC4370 datasheet: nxp.com/docs/en/data-sheet/LPC4370.pdf
- Link2 schematic (for PCB integration reference): Embedded Artists LPC-Link2_v3_RevA1.
