---
title: "EDDI-FPGA: A Code Walkthrough"
subtitle: "How a CubeSat instrument's brain works, module by module"
author: Jonathan Mulondo
date: July 2026
---

# EDDI-FPGA: A Code Walkthrough

EDDI-FPGA is the digital signal processor for a small space instrument with two jobs: *listen to the ionosphere* — the shell of electrified gas around Earth — and *feel space debris* hitting the spacecraft. The original instrument flew on a CubeSat in 2024 on a microcontroller that could only do one job at a time. This document walks through the Verilog that moves both jobs into an FPGA (a Microchip PolarFire MPF300T), in the order the signal experiences them.

## Step 0 — The big picture

One antenna signal, two completely different questions asked of it:

```
                          ┌─────────────────────────────────────────────┐
                          │                FPGA (MPF300T)               │
                          │                                             │
antenna → amp → ADC ──┬──→│  qtn_core ──→ fft_r2 ──→ psd_avg   ──→ mean │
                      │   │  (window)     (FFT)      (average)  spectrum│
                      │   │  ·············· "what is the ionosphere     │
                      │   │      doing?"  (frequency domain)            │
                      │   │                                             │
                      └──→│  debris_detector ──→ frozen 1536-sample     │
                          │  ·············· "did something hit us?"     │
                          │      (time domain)            snapshot      │
                          │                                             │
                          │  watchdog ── supervises everything          │
                          └─────────────────────────────────────────────┘
```

The whole reason this project exists: the flight microcontroller could only run **one** of those two paths at a time. On the FPGA they are separate pieces of physical hardware, so they run simultaneously.

Keep one thing in mind throughout: everything below is *hardware being described*, not a program executing line by line. Every `always @(posedge clk)` block in every module fires on the same clock tick, all at once, forever. That is the whole trick.

### Why a satellite listens to static

A wire antenna floating in the ionosphere picks up a faint hiss: the thermal jiggling of free electrons, called **quasi-thermal noise** (QTN). That hiss has a shape. Its spectrum peaks at the **plasma frequency**, which depends directly on how many electrons are around — so measuring where the peak sits measures the electron density, from inside the ionosphere, with an instrument that transmits nothing and uses milliwatts. Over-the-horizon radar and GPS corrections live and die by exactly this number.

The same antenna does a second job. When a dust-grain of space debris hits the spacecraft at ~10 km/s it vaporises into a tiny cloud of charged plasma, and that cloud sweeping past the antenna makes a sharp voltage spike. Count and characterise the spikes, and you are measuring the sub-millimetre debris population that no radar on Earth can see.

## Step 1 — The number system everything depends on

There is no floating point anywhere. Every value is an integer that we *agree* to interpret as a fraction. The convention is **Q1.15**: a 16-bit signed integer where we pretend the value is divided by 2^15 = 32768.

```
Q1.15:   0100 0000 0000 0000  =  16384  → means 16384/32768 = 0.5
         0111 1111 1111 1111  =  32767  → means ~0.99997  (max)
         1000 0000 0000 0000  = -32768  → means -1.0      (min)
```

Multiplying two Q1.15 numbers gives a Q2.30 result (32 bits), so after every multiply you will see `>>> 15` — an arithmetic right shift that divides by 2^15 and puts the decimal point back where it belongs:

```
0.5 × 0.5:   16384 × 16384 = 268,435,456          (Q2.30)
             268,435,456 >>> 15 = 8192  =  0.25 in Q1.15  ✓
```

That is the entire trick. Whenever a shift follows a multiply in this code, it is not magic — it is bookkeeping.

## Step 2 — `qtn_core.v`: the front door of the ionosphere path

### Why a window at all?

The FFT is going to chop the ADC stream into 1024-sample frames, and it mathematically assumes each frame repeats forever, end-to-start. If a frame ends at a different level than it began, that seam looks like a violent step edge — and a step edge contains *every* frequency, smearing garbage across the whole spectrum. The fix: multiply each frame by a bell curve (the **Hann window**) that fades both ends to zero. No ends, no seam.

```
raw frame:        window:              windowed frame:
/\/\/\/\/\/\      _.-'''-._            _./\/\/\/\._
(edges cut        (fades edges         (starts and ends
 mid-swing)        to zero)             at zero — no seam)
```

### The two-stage pipeline

```verilog
always @(posedge clk) begin
    coeff_q <= hann_rom[n];     // stage 1: look up window coefficient
    samp_d  <= sample_in;       //          and hold the sample beside it
end

wire signed [31:0] prod = samp_d * coeff_q;
win_q <= prod >>> 15;           // stage 2: multiply, rescale to Q1.15
```

Why two stages? Reading a ROM takes one clock. Multiplying takes another. Doing both in one tick would limit how fast the whole chip can clock. So it is an assembly line:

```
clock tick:     1        2        3        4
              ┌──────┬────────┬────────┬────────┐
sample #1 →   │lookup│multiply│  done  │        │
sample #2 →   │      │ lookup │multiply│  done  │
sample #3 →   │      │        │ lookup │multiply│
              └──────┴────────┴────────┴────────┘
     one sample enters AND one exits every tick —
     latency is 2 ticks, but throughput is full speed
```

### The handshake — and the bug the review caught here

`qtn_core` talks to the outside world with a **ready/valid handshake**: the sender raises `valid` when it has a sample, the receiver raises `ready` when it can take one, and a transfer happens only on a clock tick where *both* are high. Simple — but the edges are where bugs live. The gate that decides `ready`:

```verilog
assign sample_ready = fft_ready && !frame_wait && !all_loaded && !spectrum_ready;
```

That `all_loaded` term is a code-review fix. Originally the gate relied on `spectrum_ready`, which arrives two clock ticks after the last sample was actually needed. For those two ticks the door stood open and exactly one stray sample slipped in — invisible only because `hann[0] = 0` multiplied it by zero. Now a latch slams the door the instant the final frame's final sample is accepted.

## Step 3 — `fft_r2.v`: the heart of the machine

This module turns 1024 time samples into 1024 frequency bins, in three phases: **load**, **compute**, **unload**.

### Load — the bit-reversal shuffle

The radix-2 FFT needs its input in a scrambled order to produce output in natural order. The scramble is beautifully simple: write each sample to the address you get by **mirroring the bits of its index**:

```
sample #1  =  index 0000000001  →  stored at 1000000000  =  address 512
sample #2  =  index 0000000010  →  stored at 0100000000  =  address 256
sample #3  =  index 0000000011  →  stored at 1100000000  =  address 768
```

```verilog
function [LOG2N-1:0] bitrev;
    input [LOG2N-1:0] v;
    integer i;
    for (i = 0; i < LOG2N; i = i + 1)
        bitrev[i] = v[LOG2N-1-i];    // bit 0 ↔ bit 9, bit 1 ↔ bit 8, ...
endfunction
```

In software this shuffle costs a loop. In hardware it costs *nothing* — it is just wires crossed on the way to the RAM's address port.

### Compute — the butterfly

The entire FFT is one small operation repeated 5,120 times. Take two values A and B out of RAM, multiply B by a **twiddle factor** W (a point on the unit circle — the rotating ruler the FFT measures against), and put back their sum and difference:

```
            A ────────────●──────→  A' = (A + W·B) / 2
                           \   /
                            \ /      ← the crossing shape is why
                            / \        it's called a "butterfly"
            B ──×W ────────●──────→  B' = (A − W·B) / 2
```

Ten rounds for 1024 points. In round 1 the butterflies pair neighbors; each round the pairs stretch twice as far apart:

```
round 1:   [0↔1] [2↔3] [4↔5] ...          span 1
round 2:   [0↔2] [1↔3] [4↔6] ...          span 2
round 3:   [0↔4] [1↔5] [2↔6] ...          span 4
  ...                                        ...
round 10:  [0↔512] [1↔513] ...            span 512
```

512 butterflies × 10 rounds = 5,120 operations per spectrum.

### The 5-tick sequencer

RAM gives you one read per clock, and a read takes a tick to come back. So each butterfly is choreographed by a small state machine:

```
┌────────┐   ┌────────┐   ┌─────────┐   ┌─────────────┐   ┌─────────────┐
│ S_RD_A │──→│ S_RD_B │──→│  S_MUL  │──→│    S_WRA    │──→│    S_WRB    │─┐
│ask for │   │ask for │   │catch A; │   │catch B; do  │   │write back   │ │
│A and W │   │   B    │   │W arrives│   │W·B; write A'│   │B'; advance  │ │
└────────┘   └────────┘   └─────────┘   └─────────────┘   └──────┬──────┘ │
     ↑                                                           │        │
     └────────────────────── next butterfly ─────────────────────┘        │
                                              after round 10: unload bins
```

Five ticks × 5,120 butterflies ≈ 26,000 clocks per FFT. At 100 MHz that is 0.26 ms — and the mission needs only 20 spectra per second, so the engine idles ~99% of the time. Loads of headroom.

### Two safety details inside the math

**Scaling.** Adding two numbers can double the magnitude; do that ten rounds in a row and you overflow 16 bits by a factor of 1024. So every butterfly divides its results by 2 — after ten rounds the output equals FFT/1024, which conveniently matches NumPy's `fft(x)/N`, making verification a direct comparison.

**Saturation.** If a result *still* will not fit (a full-scale input right on the edge), `halve_sat` clamps it to the maximum instead of letting it wrap:

```
wrapping (disaster):    32767 + 1  →  -32768    loudest signal becomes
                                                 its own opposite
saturating (graceful):  32767 + 1  →   32767    distorted, but sane
```

```verilog
S_WRA: begin
    // rdata holds B; t = W*B is valid combinationally
    ram_re[idxA] <= halve_sat(a_re_x + {{2{t_re_nxt[17]}}, t_re_nxt});
    state <= S_WRB;
end
S_WRB: begin
    ram_re[idxB] <= halve_sat(a_re_x - {{2{t_re[17]}}, t_re});
    // ... then advance to the next butterfly, group, or round
end
```

The odd `{{2{...}}}` syntax is sign extension — padding a negative number's sign bit so 18-bit and 20-bit values add correctly.

## Step 4 — `psd_avg.v`: from spectrum to measurement

The FFT streams out 513 complex bins (for real input the top half of the spectrum is a mirror image, so bins 0…512 carry everything). Each becomes a **power**: real² + imaginary². Phase is thrown away — for this measurement only energy matters.

One spectrum of thermal hiss is too jittery to read. But noise is random and the plasma peak is not, so averaging several frames cancels the jitter and leaves the peak:

```
1 frame:     ~~/\~^~~v~/\~~█~~^~/\~~     (peak barely visible)
4 averaged:  ______________█________     (peak unmistakable)
                            ↑
              bin 77 = plasma frequency = electron density
```

```verilog
// power in this frequency bin
wire [32:0] p = re*re + im*im;

// first frame writes fresh; later frames add on top
wire [ACCW-1:0] acc_new = first_q ? p_q : acc_rd + p_q;
wire [ACCW-1:0] mean    = acc_rd >> LOG2AVG;
```

Two things worth pointing at: the `first_q` flag means there is no separate "clear the RAM" pass — the first frame's write *is* the clear. And the average is a bit-shift because the frame count is forced to be a power of two: division costs real hardware; dropping bits costs nothing.

## Step 5 — `debris_detector.v`: statistics with a hair trigger

Different philosophy entirely. The QTN path processes everything identically, forever. The debris path must *judge* — almost everything it sees is boring, and it has to catch the one-in-a-million exception without crying wolf. Its life is a five-state loop:

```
      power-on
         │
         ▼
 ┌────────────┐  2048 samples   ┌────────────┐  2048 samples
 │ CAL_MEAN   │────────────────→│  CAL_VAR   │──────────────┐
 │ learn the  │                 │ learn σ and│              │
 │ DC level   │                 │ wobble rate│              ▼
 └────────────┘                 └────────────┘      ┌────────────┐
         ▲                                          │   ARMED    │
         │  every ~60 s (recalibrate)               │  watching  │
         └──────────────────────────────────────────└─────┬──────┘
                                     spike > 4σ           │
                        ┌─────────────────────────────────┘
                        ▼
                 ┌────────────┐  real impact  ┌────────────┐
                 │   POST     │──────────────→│   READY    │── ack or
                 │ capture +  │               │ event      │   timeout
                 │ interrogate│──────┐        │ frozen     │─────┐
                 └────────────┘      │        └────────────┘     │
                            false alarm:                        back
                            discard, re-arm ───────────────→  to ARMED
```

### Calibration: what does "normal" look like?

Phase one averages 2048 samples to find the DC level. Phase two measures two things *relative to that mean*: the variance (how loud the noise is) and the **mean-crossing rate** (how *fast* the noise wobbles — white noise crosses its average roughly every other sample).

### The trigger: 4σ without a square root

A sample is suspicious if it sits more than 4 standard deviations from the mean — a one-in-16,000 fluke for noise. But computing σ needs a square root, which is genuinely painful hardware. So the code squares both sides of the inequality instead:

```
|x − mean| > 4σ      ⇔      (x − mean)² > 16 · variance

one subtract, one multiply, one shift-by-4, one compare.
No square root anywhere in the machine.
```

```verilog
wire signed [16:0] diff    = in_sample - cal_mean;
wire        [33:0] diff_sq = diff * diff;
wire        [31:0] var_eff = (cal_var < VAR_FLOOR) ? VAR_FLOOR : cal_var;
wire        [35:0] thresh  = {var_eff, 4'b0};   // 16*var = (4σ)²
wire trigger = (diff_sq > thresh);
```

That `VAR_FLOOR` line is the most important review fix in the design. If calibration happens during a dead-quiet stretch, variance is zero — and *zero times sixteen is zero*, so every single 1-bit wiggle would trigger, each burning a ~1500-sample blind window, while (worse) the qualification step below would reject every one of them. A real impact would be thrown away, silently, forever. The floor makes a degenerate calibration merely conservative instead of fatal.

### Qualification: the fingerprint check

Loudness alone is not proof — interference is loud too. But an impact has a signature: the vaporised dust grain's plasma cloud rings at *kilohertz*, far slower than the fizzy background noise. So during the first 128 samples after a trigger, the detector counts mean-crossings and demands the event wobble at less than a third of the background rate:

```
background noise:   /\/\/\/\/\/\/\/\    crosses mean ~every 2 samples
impact ringdown:    ___/‾‾\___/‾\__     crosses mean ~every 17 samples
                        slow decay ← what a vaporised dust
                                     grain "sounds" like
```

```verilog
wire qualify = bg_degen || (event_zc_x3 < bg_zc_scaled);
```

The ×3 is computed as `x + (x << 1)` — dodging real arithmetic again — and `bg_degen` is the other half of the review fix: if the background is so quiet that its crossing rate carries no information, the frequency test is skipped rather than letting it veto everything.

### The capture buffer: recording the past

The best part. The buffer is **circular** — 1536 slots, always recording, new samples overwriting the oldest:

```
      write pointer keeps looping forever →
    ┌──────────────────────────────────────────┐
    │ ... older ← ← ← ← ● newest               │
    └────────────────────│─────────────────────┘
                      TRIGGER!
                         │
     freeze after 1023 more samples:
    ┌──────────────┬─●──────────────────────────┐
    │  512 samples │ │      1023 samples        │
    │  BEFORE the  │ │      AFTER               │
    │  impact      │ │                          │
    └──────────────┴──────────────────────────── ┘
     the instrument recorded the lead-up to an
     event before it knew the event would happen
```

By the time the trigger fires, the 512 samples of "before" already exist in the buffer — the detector just stops overwriting at the right moment. The snapshot then holds in `READY` until the spacecraft computer collects it, or, per another review fix, until a timeout auto-releases it and bumps an `events_dropped` counter — because a detector frozen waiting on a dead comms link would otherwise miss every subsequent impact for the rest of the mission.

## Step 6 — `watchdog.v`: the dead-man's switch

The simplest module, and conceptually the most space-flavored. A counter climbs every clock. The healthy system must "kick" it back to zero regularly. If the kicks ever stop — software wedged, state machine stuck, radiation flipped a bit — the counter tops out and the watchdog resets everything:

```
cnt │        /      /      /                 / ← TIMEOUT reached:
    │       /      /      /                 /     pull reset, log it
    │      /      /      /      ...        /
    └─────┴──────┴──────┴─────────────────┴──────→ time
          kick   kick   kick    (kicks stop)
```

```verilog
// a stuck-high kick must not silence the watchdog: only edges count
wire kick_edge = kick && !kick_d;

end else if (kick_edge) begin
    cnt <= {WIDTH{1'b0}};
end else if (cnt >= TIMEOUT-1) begin
    wdt_rst     <= 1'b1;
    fired_count <= fired_count + 1;
end
```

Three paranoid details, each a lesson in space-grade thinking:

1. **Only a change counts.** If a fault freezes the kick line high, a level-sensitive check would be fooled forever; an edge detector is not.
2. **`>=` not `==`.** Even if radiation bumps the counter *past* the target, the timeout still fires. This is space.
3. **The fired-counter survives the watchdog's own reset**, so ground control can ask "how many times did you have to restart yourself?" — silent recovery hides exactly the evidence you need.

The flight firmware never enabled its watchdog at all; the thesis calls that out as a gap, which is why this exists as a first-class module here.

## Step 7 — How we know any of this is true

Nothing has touched silicon. The proof chain is:

```
gen_vectors.py (NumPy)           Questa simulator
┌──────────────────────┐        ┌──────────────────────┐
│ synth test signals   │──.mem─→│ the actual Verilog,  │
│ + compute the exact  │        │ clock tick by tick   │
│ float answers        │──.mem─→│ vs. golden answers   │──→ TB_PASS / TB_FAIL
└──────────────────────┘        └──────────────────────┘
```

The Python even replicates the RTL's fixed-point windowing *bit-exactly*, so the only difference left between "golden" and "hardware" is the FFT's rounding — which is the thing being measured. Results:

| Measure | Result |
|---|---|
| Pipeline spectrum peak vs. NumPy floating-point truth | within **0.007%** |
| Frequency bins individually checked against the golden FFT | **513 / 513** |
| Debris trigger delay after an injected impact | **+1 sample** |
| False alarms across 45,000 samples of pure noise | **0** |

### What the adversarial review changed

Passing tests only proves the code handles what the tests imagined. An independent review pass hunted for what they didn't — degenerate inputs, stuck links, orbital drift — and reproduced two real bugs in simulation. All confirmed findings are fixed and re-verified:

| Finding | Fix | Status |
|---|---|---|
| Degenerate calibration bricked debris detection | `VAR_FLOOR` + `bg_degen` bypass (Step 5) | fixed |
| No recalibration over an orbit's thermal drift | ~60 s recalibration loop (Step 5) | fixed |
| Frozen forever on a missed acknowledgment | READY timeout + `events_dropped` (Step 5) | fixed |
| One-sample handshake leak per averaging run | `all_loaded` gate (Step 2) | fixed |
| ROM contents may not survive PolarFire synthesis | to validate in the first Libero trial | open |

The meta-lesson worth retelling: the original instrument's sister satellite was killed a month into its mission by a radiation event. For space hardware, the happy path is the easy 80% — robustness against the input you never imagined *is* the design problem.

---

**Repo map:** `fpga/rtl/` — five synthesizable Verilog modules (`fft_r2`, `psd_avg`, `qtn_core`, `debris_detector`, `watchdog`) · `fpga/tb/` — self-checking testbenches · `fpga/sim/` — NumPy golden-vector generator + Questa run script (`sh run_all.sh`).

**Heritage:** the EDDI instrument (University of Sydney / CUAVA), flown on the Waratah Seed-1 CubeSat, launched August 2024 and still returning data. **Target:** Microchip PolarFire MPF300T-1FCG1152I · **ADC:** LTC2165 or ADS5560 (16-bit).
