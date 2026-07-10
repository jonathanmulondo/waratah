//-----------------------------------------------------------------------------
// debris_detector.v -- time-domain debris impact detector, EDDI-FPGA
//
// WHAT THIS IS
//   FPGA port of the LPC4370 flight firmware's debris mode. A sub-mm
//   particle hitting the spacecraft at ~10 km/s vaporises into a plasma
//   plume; the charge separation (fast electrons, slow ions) induces a
//   characteristic voltage transient on the antenna: a sharp spike with
//   a slow opposite-polarity decay, ringing at ~1-25 kHz -- far below
//   the broadband background. This module watches the low-rate
//   (~50 ksps) sample stream from the debris amplifier chain for exactly
//   that signature. On the FPGA it runs CONCURRENTLY with qtn_core --
//   the whole point of moving off the single-ADC-mode LPC4370.
//
// DETECTION ALGORITHM (same as flight firmware)
//   1. S_CAL_MEAN : average CAL_LEN samples -> quiescent mean (DC level)
//   2. S_CAL_VAR  : accumulate (x-mean)^2 -> variance, and count
//                   mean-crossings -> background "how fast does the
//                   noise wiggle" rate
//   3. S_ARMED    : trigger when (x-mean)^2 > 16*var, i.e. |x-mean| > 4
//                   standard deviations. Squared comparison: identical
//                   decision to the firmware's 4-sigma test but needs no
//                   square root in hardware.
//   4. S_POST     : capture POST_LEN more samples; count mean-crossings
//                   over the first QUAL_LEN of them. A real impact rings
//                   SLOWLY, so it must cross the mean far less often
//                   than the background: qualify when
//                   3*event_zc < bg_zc scaled to the same window length.
//                   RF interference is loud but fast -> rejected here.
//   5. S_READY    : a qualified event is frozen in the buffer until the
//                   payload computer `ack`s (or a timeout drops it);
//                   unqualified events re-arm automatically.
//
//   The capture buffer is circular and always recording, so an event
//   record contains PRE_LEN samples from BEFORE the trigger plus the
//   trigger and POST_LEN-1 after it (flight design: 512 + 1024 = 1536).
//
// ROBUSTNESS ADDITIONS BEYOND THE FLIGHT FIRMWARE
//   - variance floor      : a dead-quiet calibration can't arm a hair trigger
//   - periodic recal      : tracks thermal/orbital drift of the DC point
//   - degenerate-bg bypass: a zero-crossing-free background can't veto
//                           every event forever
//   - ready timeout       : an unresponsive payload computer can't freeze
//                           the detector permanently (drops are counted)
//-----------------------------------------------------------------------------
module debris_detector #(
    parameter LOG2CAL  = 11,                // CAL_LEN = 2048 (~41 ms at 50 ksps)
    parameter PRE_LEN  = 512,               // pre-trigger context samples
    parameter POST_LEN = 1024,              // trigger + post-trigger samples
    parameter QUAL_LEN = 128,               // qualification window; must be a
                                            // power of 2 dividing CAL_LEN
    parameter BUF_LEN  = PRE_LEN + POST_LEN,
    parameter BUF_AW   = 11,                // >= clog2(BUF_LEN)
    // robustness floors and timeouts (in samples)
    parameter VAR_FLOOR      = 32'd4,       // min variance -> min trigger |x-mean| = 4*2 LSB
    parameter QUAL_MARGIN    = 8,           // background zc rate below this is
                                            // "degenerate" -> skip freq test
    parameter RECAL_INTERVAL = 32'd3_000_000, // ~60 s at 50 ksps: periodic recal
    parameter READY_TIMEOUT  = 32'd250_000  // ~5 s at 50 ksps: auto-drop unclaimed event
) (
    input  wire               clk,
    input  wire               rst,

    input  wire               in_valid,     // one pulse per ADC sample
    input  wire signed [15:0] in_sample,

    input  wire               ack,          // release frozen event, re-arm

    output reg                event_ready,  // a valid event is frozen
    output reg  [15:0]        event_count,
    output reg  [7:0]         events_dropped, // unclaimed events auto-released
    output reg  [47:0]        trig_index,   // absolute sample index of trigger
                                            // (48 bits: no wrap for >100 years
                                            // at 50 ksps -- 32 bits wrapped in
                                            // under a day)

    // frozen event readout: logical 0 .. BUF_LEN-1, trigger sits at PRE_LEN
    input  wire [BUF_AW-1:0]  buf_rd_addr,
    output reg  signed [15:0] buf_rd_data,  // valid 1 cycle after addr

    // calibration results (housekeeping/telemetry)
    output reg signed [15:0]  cal_mean,
    output reg [31:0]         cal_var,
    output reg [15:0]         cal_zc
);

    localparam CAL_LEN = 1 << LOG2CAL;

    localparam S_CAL_MEAN = 3'd0;
    localparam S_CAL_VAR  = 3'd1;
    localparam S_ARMED    = 3'd2;
    localparam S_POST     = 3'd3;
    localparam S_READY    = 3'd4;

    reg [2:0] state;

    // ------------------------------------------------------------- capture RAM
    // Circular buffer, written on every sample in every state except
    // S_READY (frozen). Because it never stops, the PRE_LEN samples
    // before any trigger are already in it -- pre-trigger capture for free.
    reg signed [15:0] buf_ram [0:BUF_LEN-1];
    reg [BUF_AW-1:0]  wptr;
    reg [BUF_AW-1:0]  trig_wptr;            // where wptr stood at trigger time

    // BUF_LEN (1536) is not a power of two, so wraparound is explicit
    wire [BUF_AW-1:0] wptr_nxt = (wptr == BUF_LEN-1) ? {BUF_AW{1'b0}} : wptr + 1'b1;

    // Frozen-event readout address math: the reader asks for logical
    // address 0..BUF_LEN-1 (0 = oldest pre-trigger sample, PRE_LEN =
    // trigger); physically that region starts at trig_wptr - PRE_LEN and
    // wraps around the circular buffer. Both adds carry an extra bit and
    // subtract BUF_LEN on overflow (mod-1536 arithmetic).
    wire [BUF_AW+0:0] base_raw = {1'b0, trig_wptr} + BUF_LEN - PRE_LEN;
    wire [BUF_AW-1:0] base     = (base_raw >= BUF_LEN) ? base_raw - BUF_LEN
                                                       : base_raw[BUF_AW-1:0];
    wire [BUF_AW+0:0] rd_raw   = {1'b0, base} + {1'b0, buf_rd_addr};
    wire [BUF_AW-1:0] rd_phys  = (rd_raw >= BUF_LEN) ? rd_raw - BUF_LEN
                                                     : rd_raw[BUF_AW-1:0];
    always @(posedge clk)
        buf_rd_data <= buf_ram[rd_phys];

    // ------------------------------------------------------------- statistics
    reg  [LOG2CAL-1:0] cal_cnt;
    reg  signed [27:0] sum;                          // running sum of samples
    reg  [47:0]        sumsq;                        // running sum of (x-mean)^2

    // deviation from the calibrated DC level, and its square
    wire signed [16:0] diff    = {in_sample[15], in_sample} - {cal_mean[15], cal_mean};
    wire        [33:0] diff_sq = diff * diff;
    // Variance floor: a degenerate (dead-quiet or pure-DC) calibration
    // would otherwise set a threshold so low that 1-LSB noise wiggles
    // fire the trigger continuously.
    wire        [31:0] var_eff = (cal_var < VAR_FLOOR) ? VAR_FLOOR : cal_var;
    // The 4-sigma test, squared to avoid a square root:
    //   |x-mean| > 4*sigma  <=>  (x-mean)^2 > 16*var
    // and {var,4'b0} is var*16 by bit-shift.
    wire        [35:0] thresh  = {var_eff, 4'b0};

    wire trigger = ({2'b0, diff_sq} > {2'b0, thresh});

    // Calibration arithmetic. Kept fully signed on purpose: in Verilog a
    // {concatenation} operand is always unsigned and silently turns >>>
    // into a logical shift -- a real bug the code review caught here.
    wire signed [27:0] sum_nxt  = sum + in_sample;
    wire signed [27:0] mean_nxt = sum_nxt >>> LOG2CAL;   // mean = sum / CAL_LEN
    wire        [47:0] var_nxt  = (sumsq + {14'b0, diff_sq}) >> LOG2CAL;

    // Mean-crossing detection: a "crossing" is the sign of (x - mean)
    // flipping between consecutive samples. Cheap 1-bit proxy for
    // dominant frequency: fast noise crosses often, a slow impact
    // transient crosses rarely.
    wire sign_now = diff[16];
    reg  sign_d;
    wire zc = (sign_now ^ sign_d);

    // ------------------------------------------------------------- detection bookkeeping
    reg [47:0] abs_idx;                              // absolute sample counter
    reg [15:0] armed_fill;                           // samples since (re)arming --
                                                     // no trigger until PRE_LEN
                                                     // real pre-samples exist
    reg [10:0] post_cnt;
    reg [15:0] event_zc;
    reg [31:0] recal_cnt;                            // samples since calibration
    reg [31:0] ready_cnt;                            // samples spent frozen in S_READY

    // Frequency qualification. Background rate cal_zc was counted over
    // CAL_LEN samples but the event rate over QUAL_LEN, so scale the
    // background down by CAL_LEN/QUAL_LEN (a shift -- both are powers of
    // two). Then require the event to cross at less than 1/3 of the
    // background rate. If the background itself is degenerate (near-zero
    // crossings: quiet/DC input), the crossing rate carries no
    // information and would veto every event forever -- so the frequency
    // criterion only applies when the background rate is meaningful.
    localparam ZC_SHIFT = LOG2CAL - $clog2(QUAL_LEN);
    wire [15:0] bg_zc_scaled = cal_zc >> ZC_SHIFT;
    wire [17:0] event_zc_x3  = {2'b0, event_zc} + ({2'b0, event_zc} << 1);
    wire        bg_degen     = (bg_zc_scaled < QUAL_MARGIN);
    wire        qualify      = bg_degen
                             || (event_zc_x3 < {2'b0, bg_zc_scaled});

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_CAL_MEAN;
            cal_cnt     <= 0;
            sum         <= 0;
            sumsq       <= 0;
            cal_mean    <= 0;
            cal_var     <= 0;
            cal_zc      <= 0;
            wptr        <= 0;
            abs_idx     <= 0;
            event_ready <= 1'b0;
            event_count <= 0;
            events_dropped <= 0;
            armed_fill  <= 0;
            sign_d      <= 1'b0;
            recal_cnt   <= 0;
            ready_cnt   <= 0;
        end else begin
            if (in_valid && state != S_READY) begin
                // things that happen on EVERY sample: record it, advance
                // the circular buffer and absolute index, remember the
                // sign for crossing detection
                buf_ram[wptr] <= in_sample;
                wptr          <= wptr_nxt;
                abs_idx       <= abs_idx + 1;
                sign_d        <= sign_now;

                case (state)
                // ------------------------------------ phase 1: DC level
                S_CAL_MEAN: begin
                    sum <= sum_nxt;
                    if (cal_cnt == CAL_LEN-1) begin
                        cal_mean <= mean_nxt[15:0];
                        cal_cnt  <= 0;
                        sumsq    <= 0;
                        cal_zc   <= 0;
                        state    <= S_CAL_VAR;
                    end else
                        cal_cnt <= cal_cnt + 1;
                end
                // ------------------------------------ phase 2: noise stats
                // Needs a second pass because (x-mean)^2 requires the
                // mean phase 1 just produced.
                S_CAL_VAR: begin
                    sumsq  <= sumsq + {14'b0, diff_sq};
                    cal_zc <= cal_zc + {15'b0, zc};
                    if (cal_cnt == CAL_LEN-1) begin
                        // saturate: pathological input swings must not
                        // wrap the variance to a tiny value and cause a
                        // trigger storm
                        cal_var    <= (|var_nxt[47:32]) ? 32'hFFFF_FFFF
                                                        : var_nxt[31:0];
                        cal_cnt    <= 0;
                        armed_fill <= 0;
                        recal_cnt  <= 0;
                        state      <= S_ARMED;
                    end else
                        cal_cnt <= cal_cnt + 1;
                end
                // ------------------------------------ watch for impacts
                S_ARMED: begin
                    recal_cnt <= recal_cnt + 1;
                    if (recal_cnt >= RECAL_INTERVAL) begin
                        // periodic recalibration (flight FW did this every
                        // minute) -- tracks thermal/orbital drift of the
                        // DC operating point
                        sum       <= 0;
                        cal_cnt   <= 0;
                        recal_cnt <= 0;
                        state     <= S_CAL_MEAN;
                    end else if (armed_fill < PRE_LEN)
                        // don't trigger until the circular buffer holds
                        // PRE_LEN genuine pre-trigger samples
                        armed_fill <= armed_fill + 1;
                    else if (trigger) begin
                        trig_wptr  <= wptr;
                        trig_index <= abs_idx;
                        post_cnt   <= 0;
                        event_zc   <= 0;
                        state      <= S_POST;
                    end
                end
                // ------------------------------------ capture + qualify
                S_POST: begin
                    recal_cnt <= recal_cnt + 1;
                    if (post_cnt < QUAL_LEN)
                        event_zc <= event_zc + {15'b0, zc};
                    // the trigger sample itself already occupies one of
                    // the POST_LEN slots, so capture POST_LEN-1 further
                    // samples (stopping one later would overwrite the
                    // oldest pre-trigger sample -- an off-by-one found in
                    // simulation)
                    if (post_cnt == POST_LEN-2) begin
                        if (qualify) begin
                            event_ready <= 1'b1;
                            event_count <= event_count + 1;
                            ready_cnt   <= 0;
                            state       <= S_READY;
                        end else begin
                            // loud but fast => interference, not debris:
                            // discard silently and refill the pre-buffer
                            armed_fill  <= 0;
                            state       <= S_ARMED;
                        end
                    end
                    post_cnt <= post_cnt + 1;
                end
                default: ;
                endcase
            end

            // S_READY sits outside the in_valid gate so an ack is honoured
            // immediately, not only on sample edges
            if (state == S_READY) begin
                if (ack) begin
                    event_ready <= 1'b0;
                    armed_fill  <= 0;
                    state       <= S_ARMED;
                end else if (in_valid) begin
                    // never freeze forever waiting on the payload
                    // computer: auto-drop the event after READY_TIMEOUT
                    // samples and count the drop for telemetry
                    ready_cnt <= ready_cnt + 1;
                    if (ready_cnt >= READY_TIMEOUT-1) begin
                        event_ready    <= 1'b0;
                        events_dropped <= events_dropped + 1;
                        armed_fill     <= 0;
                        state          <= S_ARMED;
                    end
                end
            end
        end
    end

endmodule
