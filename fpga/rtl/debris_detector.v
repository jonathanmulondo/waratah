//-----------------------------------------------------------------------------
// debris_detector.v -- time-domain debris impact detector, EDDI-FPGA
//
// FPGA port of the LPC4370 flight firmware's debris mode. Runs on the
// low-rate (~50 ksps) sample stream from the debris amplifier chain.
//
// Operation:
//   1. CAL_MEAN : average CAL_LEN samples -> quiescent mean (DC level)
//   2. CAL_VAR  : accumulate (x-mean)^2 -> variance, and count
//                 mean-crossings -> background crossing rate
//   3. ARMED    : trigger when (x-mean)^2 > 16*var  (i.e. |x-mean| > 4 sigma)
//   4. POST     : capture POST_LEN more samples; count mean-crossings in
//                 the first QUAL_LEN samples. Event is valid when
//                 3*event_zc < bg_zc/(CAL_LEN/QUAL_LEN)  (impact transients
//                 ring at ~kHz, far below the background crossing rate)
//   5. READY    : valid event frozen in buffer until `ack`; invalid
//                 events re-arm automatically.
//
// The capture buffer holds PRE_LEN pre-trigger + POST_LEN post-trigger
// samples (flight design: 512 + 1024 = 1536).
//-----------------------------------------------------------------------------
module debris_detector #(
    parameter LOG2CAL  = 11,                // CAL_LEN = 2048
    parameter PRE_LEN  = 512,
    parameter POST_LEN = 1024,
    parameter QUAL_LEN = 128,               // must divide CAL_LEN, power of 2
    parameter BUF_LEN  = PRE_LEN + POST_LEN,
    parameter BUF_AW   = 11,                // >= clog2(BUF_LEN)
    // robustness floors and timeouts (in samples)
    parameter VAR_FLOOR      = 32'd4,       // min variance -> min trigger |x-mean| = 4*2 LSB
    parameter QUAL_MARGIN    = 8,           // crossing-rate margin (degenerate-safe qualify)
    parameter RECAL_INTERVAL = 32'd3_000_000, // ~60 s at 50 ksps: periodic recalibration
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
                                            // (48 bits: no wrap for >100 years at 50 ksps)

    // frozen event readout: 0 .. BUF_LEN-1 (trigger at PRE_LEN)
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
    reg signed [15:0] buf_ram [0:BUF_LEN-1];
    reg [BUF_AW-1:0]  wptr;
    reg [BUF_AW-1:0]  trig_wptr;

    wire [BUF_AW-1:0] wptr_nxt = (wptr == BUF_LEN-1) ? {BUF_AW{1'b0}} : wptr + 1'b1;

    // frozen-event readout: logical addr 0 maps to trig_wptr - PRE_LEN
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
    reg  signed [27:0] sum;                          // sum of samples
    reg  [47:0]        sumsq;                        // sum of (x-mean)^2

    wire signed [16:0] diff    = {in_sample[15], in_sample} - {cal_mean[15], cal_mean};
    wire        [33:0] diff_sq = diff * diff;
    // variance floor: a degenerate (quiet/DC) calibration must not arm the
    // trigger on every 1-LSB wiggle
    wire        [31:0] var_eff = (cal_var < VAR_FLOOR) ? VAR_FLOOR : cal_var;
    wire        [35:0] thresh  = {var_eff, 4'b0};    // 16 * var  (4-sigma squared)

    wire trigger = ({2'b0, diff_sq} > {2'b0, thresh});

    // calibration arithmetic (kept fully signed: a {concat} operand would
    // silently turn >>> into a logical shift)
    wire signed [27:0] sum_nxt  = sum + in_sample;
    wire signed [27:0] mean_nxt = sum_nxt >>> LOG2CAL;
    wire        [47:0] var_nxt  = (sumsq + {14'b0, diff_sq}) >> LOG2CAL;

    // mean-crossing detection
    wire sign_now = diff[16];
    reg  sign_d;
    wire zc = (sign_now ^ sign_d);

    // ------------------------------------------------------------- detection bookkeeping
    reg [47:0] abs_idx;                              // absolute sample counter
    reg [15:0] armed_fill;                           // samples since armed
    reg [10:0] post_cnt;
    reg [15:0] event_zc;
    reg [31:0] recal_cnt;                            // samples since calibration
    reg [31:0] ready_cnt;                            // samples spent frozen in S_READY

    // qualification: 3*event_zc < bg_zc/(CAL_LEN/QUAL_LEN). With a quiet/DC
    // background the crossing rate carries no information (bg_zc ~ 0 would
    // veto every event), so the frequency criterion only applies when the
    // background rate is meaningful.
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
                // capture buffer runs continuously except when frozen
                buf_ram[wptr] <= in_sample;
                wptr          <= wptr_nxt;
                abs_idx       <= abs_idx + 1;
                sign_d        <= sign_now;

                case (state)
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
                S_CAL_VAR: begin
                    sumsq  <= sumsq + {14'b0, diff_sq};
                    cal_zc <= cal_zc + {15'b0, zc};
                    if (cal_cnt == CAL_LEN-1) begin
                        // saturate: pathological swings must not wrap to a
                        // tiny variance and cause a trigger storm
                        cal_var    <= (|var_nxt[47:32]) ? 32'hFFFF_FFFF
                                                        : var_nxt[31:0];
                        cal_cnt    <= 0;
                        armed_fill <= 0;
                        recal_cnt  <= 0;
                        state      <= S_ARMED;
                    end else
                        cal_cnt <= cal_cnt + 1;
                end
                S_ARMED: begin
                    recal_cnt <= recal_cnt + 1;
                    if (recal_cnt >= RECAL_INTERVAL) begin
                        // periodic recalibration (flight FW did this every
                        // minute) -- tracks thermal/orbital drift of the DC
                        // operating point
                        sum       <= 0;
                        cal_cnt   <= 0;
                        recal_cnt <= 0;
                        state     <= S_CAL_MEAN;
                    end else if (armed_fill < PRE_LEN)
                        armed_fill <= armed_fill + 1;
                    else if (trigger) begin
                        trig_wptr  <= wptr;
                        trig_index <= abs_idx;
                        post_cnt   <= 0;
                        event_zc   <= 0;
                        state      <= S_POST;
                    end
                end
                S_POST: begin
                    recal_cnt <= recal_cnt + 1;
                    if (post_cnt < QUAL_LEN)
                        event_zc <= event_zc + {15'b0, zc};
                    // trigger sample already occupies one POST_LEN slot,
                    // so capture POST_LEN-1 further samples
                    if (post_cnt == POST_LEN-2) begin
                        if (qualify) begin
                            event_ready <= 1'b1;
                            event_count <= event_count + 1;
                            ready_cnt   <= 0;
                            state       <= S_READY;
                        end else begin
                            armed_fill  <= 0;     // discard, refill pre-buffer
                            state       <= S_ARMED;
                        end
                    end
                    post_cnt <= post_cnt + 1;
                end
                default: ;
                endcase
            end

            if (state == S_READY) begin
                if (ack) begin
                    event_ready <= 1'b0;
                    armed_fill  <= 0;
                    state       <= S_ARMED;
                end else if (in_valid) begin
                    // never freeze forever waiting on the payload computer:
                    // auto-drop the event after READY_TIMEOUT samples
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
