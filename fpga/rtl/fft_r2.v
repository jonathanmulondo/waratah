//-----------------------------------------------------------------------------
// fft_r2.v -- Radix-2 DIT iterative FFT engine (EDDI-FPGA QTN path)
//
// WHAT THIS IS
//   An N-point complex FFT of a real input stream, built the classic
//   textbook way: decimation-in-time (DIT), radix-2, computed in place in
//   a single dual-role RAM. It replaces the CMSIS-DSP FFT call in the
//   LPC4370 flight firmware's QTN modes.
//
// HOW A DIT FFT WORKS (30-second version)
//   An N-point DFT can be split into two N/2-point DFTs (even samples,
//   odd samples), each of those into two N/4-point DFTs, and so on down
//   to trivial 1-point DFTs. Recombining is done with "butterflies":
//
//        A ----------(+)----> A' = A + W*B
//                 \  /
//                  \/                W = twiddle factor e^(-j*2*pi*k/N),
//                  /\                a point on the unit circle
//                 /  \
//        B --[*W]-(-)-----> B' = A - W*B
//
//   Doing the recursion iteratively means LOG2N passes ("stages") over the
//   RAM, N/2 butterflies per stage. The only price is that the input must
//   be loaded in BIT-REVERSED address order (for N=8, sample 3 = 011 goes
//   to address 110 = 6), which the load state does for free with the
//   bitrev() function below.
//
// DATAPATH / SCHEDULE
//   load (N cycles, ready/valid)  ->  LOG2N stages x N/2 butterflies
//   (5 clocks each: the RAM has one read port, so A and B are fetched
//   sequentially -- see the S_RD_A..S_WRB walkthrough at the FSM)  ->
//   unload bins 0..N/2 (real input => conjugate-symmetric spectrum, the
//   upper half carries no new information).
//
//   Total ~ N + 5*(N/2)*LOG2N + N/2 clocks (~27k for N=1024): trivially
//   fast next to the 819.2 us ADC burst it processes.
//
// FIXED-POINT SCALING
//   Samples and twiddles are Q1.15 (16-bit signed, 15 fractional bits).
//   A butterfly sum can double in magnitude, so every stage halves its
//   result: after LOG2N stages the output equals FFT(x)/N exactly, and
//   nothing can overflow 16 bits (the halving rounds-to-nearest and
//   saturates as a belt-and-braces measure -- see halve_sat).
//
// PORTABILITY
//   Vendor-neutral Verilog-2001, no primitives. ROMs are initialised with
//   $readmemh, which Questa/Quartus/Libero all accept for inference --
//   BUT PolarFire LSRAM does not power up initialised, so whether the
//   synthesised ROM actually contains the data must be validated in the
//   first Libero trial (top open item in the README).
//-----------------------------------------------------------------------------
module fft_r2 #(
    parameter LOG2N      = 10,                 // 10 -> 1024-pt, 9 -> 512-pt
    parameter N          = 1 << LOG2N,
    parameter TW_RE_FILE = "mem/tw_re_1024.mem",
    parameter TW_IM_FILE = "mem/tw_im_1024.mem"
) (
    input  wire               clk,
    input  wire               rst,

    // load interface: N real samples, sequential order, ready/valid
    input  wire               in_valid,
    input  wire signed [15:0] in_re,
    output wire               in_ready,

    // unload interface: bins 0..N/2 streamed in order, one per clock
    output reg                out_valid,
    output reg  [LOG2N-1:0]   out_bin,
    output reg  signed [15:0] out_re,
    output reg  signed [15:0] out_im,
    output reg                out_last,        // high with bin N/2

    output wire               busy
);

    // ------------------------------------------------------------- FSM states
    // One pass through S_RD_A -> S_RD_B -> S_MUL -> S_WRA -> S_WRB computes
    // one butterfly (5 clocks). The sequencing exists because the data RAM
    // has a single registered read port: A and B must be fetched on
    // consecutive cycles, and the read data lags the address by one clock.
    localparam S_LOAD  = 3'd0;  // accept N samples (bit-reversed store)
    localparam S_RD_A  = 3'd1;  // issue read of A, and of the twiddle ROM
    localparam S_RD_B  = 3'd2;  // issue read of B (A lands in rdata next edge)
    localparam S_MUL   = 3'd3;  // capture A (B lands in rdata next edge)
    localparam S_WRA   = 3'd4;  // B on rdata; write A' = (A + W*B)/2
    localparam S_WRB   = 3'd5;  // write B' = (A - W*B)/2; advance indices
    localparam S_PRIME = 3'd6;  // prime the read pipeline before unload
    localparam S_UNL   = 3'd7;  // stream bins 0..N/2

    reg [2:0] state;

    // ------------------------------------------------------------- data RAM
    // In-place working memory: samples go in, butterflies read-modify-write
    // it LOG2N times, the spectrum comes out. Separate re/im arrays so the
    // tools infer two simple block RAMs. The read is REGISTERED (rdata_*
    // updates one clock after raddr changes) -- that latency is what the
    // 5-state butterfly and the S_PRIME state are choreographed around.
    reg signed [15:0] ram_re [0:N-1];
    reg signed [15:0] ram_im [0:N-1];
    reg [LOG2N-1:0]   raddr;
    reg signed [15:0] rdata_re, rdata_im;

    always @(posedge clk) begin
        rdata_re <= ram_re[raddr];
        rdata_im <= ram_im[raddr];
    end

    // ------------------------------------------------------------- twiddle ROM
    // W_N^k = e^(-j*2*pi*k/N) for k = 0..N/2-1, stored as Q1.15 pairs.
    // Only half the unit circle is needed: a radix-2 FFT never indexes
    // twiddles beyond k = N/2-1 (the "other half" is just negation, which
    // the butterfly's subtract provides for free). Files generated by
    // fpga/sim/gen_vectors.py. Registered read, same 1-clock latency as
    // the data RAM, so twiddle and data arrive together in the schedule.
    reg signed [15:0] tw_re_rom [0:N/2-1];
    reg signed [15:0] tw_im_rom [0:N/2-1];
    initial begin
        $readmemh(TW_RE_FILE, tw_re_rom);
        $readmemh(TW_IM_FILE, tw_im_rom);
    end
    reg [LOG2N-2:0]   tw_addr;
    reg signed [15:0] tw_re_q, tw_im_q;
    always @(posedge clk) begin
        tw_re_q <= tw_re_rom[tw_addr];
        tw_im_q <= tw_im_rom[tw_addr];
    end

    // ------------------------------------------------------------- counters
    // Stage geometry, for stage s = 1..LOG2N:
    //   m    = 2^s          distance between group starts
    //   half = m/2          butterflies per group, and the A-to-B span
    //   grp  = 0, m, 2m...  base address of the current group
    //   j    = 0..half-1    butterfly index within the group
    // Example, N=8, stage 2: m=4, half=2, groups start at 0 and 4;
    //   group 0 pairs (0,2) and (1,3); group 4 pairs (4,6) and (5,7).
    reg [LOG2N:0]   load_cnt;              // 0..N-1
    reg [3:0]       stage;                 // 1..LOG2N
    reg [LOG2N:0]   grp;                   // group base (steps of m)
    reg [LOG2N-1:0] j;                     // butterfly index within group
    reg [LOG2N:0]   unl_cnt;               // 0..N/2

    wire [LOG2N:0]   m    = 1 << stage;
    wire [LOG2N-1:0] half = 1 << (stage-1);

    wire [LOG2N-1:0] idxA = grp[LOG2N-1:0] + j;          // upper butterfly leg
    wire [LOG2N-1:0] idxB = grp[LOG2N-1:0] + j + half;   // lower butterfly leg

    // Reverse the bit order of an address. Sample n is stored at bitrev(n)
    // so that after LOG2N in-place DIT stages the spectrum comes out in
    // natural bin order. Pure wiring -- costs nothing in hardware.
    function [LOG2N-1:0] bitrev;
        input [LOG2N-1:0] v;
        integer i;
        begin
            for (i = 0; i < LOG2N; i = i + 1)
                bitrev[i] = v[LOG2N-1-i];
        end
    endfunction

    // ------------------------------------------------------------- butterfly math
    reg signed [15:0] a_re, a_im;          // A leg, captured in S_MUL
    reg signed [17:0] t_re, t_im;          // t = W*B, registered for S_WRB

    // Complex multiply t = W * B, computed combinationally from the live
    // RAM output while it holds B (i.e. during S_WRA):
    //   Re(t) = Br*Wr - Bi*Wi        Im(t) = Br*Wi + Bi*Wr
    // Each 16x16 product is 32 bits; the sums are sign-extended to 33 bits
    // so the add/sub cannot overflow.
    wire signed [31:0] p_rr = rdata_re * tw_re_q;
    wire signed [31:0] p_ii = rdata_im * tw_im_q;
    wire signed [31:0] p_ri = rdata_re * tw_im_q;
    wire signed [31:0] p_ir = rdata_im * tw_re_q;
    wire signed [32:0] mul_re = {p_rr[31], p_rr} - {p_ii[31], p_ii};
    wire signed [32:0] mul_im = {p_ri[31], p_ri} + {p_ir[31], p_ir};
    // Q1.15 * Q1.15 = Q2.30; adding 2^14 before >>>15 rounds to nearest
    // instead of truncating toward -inf (removes a systematic DC bias
    // that would otherwise build up over 10 stages). Fits 18 bits signed.
    wire signed [17:0] t_re_nxt = (mul_re + 33'sd16384) >>> 15;
    wire signed [17:0] t_im_nxt = (mul_im + 33'sd16384) >>> 15;

    // (u +/- t + 1)/2 with saturation back to 16 bits: the per-stage
    // halving that makes the output FFT(x)/N. The +1 rounds half-up.
    // Saturation should never fire given the halving, but a silent wrap
    // here would corrupt a spectrum undetectably, so it is clamped anyway.
    function signed [15:0] halve_sat;
        input signed [19:0] v;
        reg   signed [19:0] h;
        begin
            h = (v + 20'sd1) >>> 1;
            if (h > 20'sd32767)       halve_sat = 16'sd32767;
            else if (h < -20'sd32768) halve_sat = -16'sd32768;
            else                      halve_sat = h[15:0];
        end
    endfunction

    // A sign-extended to the 20-bit width halve_sat's adder works in
    wire signed [19:0] a_re_x = {{4{a_re[15]}}, a_re};
    wire signed [19:0] a_im_x = {{4{a_im[15]}}, a_im};

    assign in_ready = (state == S_LOAD);
    assign busy     = (state != S_LOAD);

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_LOAD;
            load_cnt  <= 0;
            stage     <= 4'd1;
            grp       <= 0;
            j         <= 0;
            unl_cnt   <= 0;
            out_valid <= 1'b0;
            out_last  <= 1'b0;
            raddr     <= 0;
            tw_addr   <= 0;
        end else begin
            // default low; only S_UNL pulses them
            out_valid <= 1'b0;
            out_last  <= 1'b0;

            case (state)
            // ---------------------------------------------------- load
            // Store each incoming sample at its bit-reversed address with
            // zero imaginary part (real input). After sample N-1 the
            // butterfly stages start immediately.
            S_LOAD: begin
                if (in_valid) begin
                    ram_re[bitrev(load_cnt[LOG2N-1:0])] <= in_re;
                    ram_im[bitrev(load_cnt[LOG2N-1:0])] <= 16'sd0;
                    if (load_cnt == N-1) begin
                        load_cnt <= 0;
                        stage    <= 4'd1;
                        grp      <= 0;
                        j        <= 0;
                        state    <= S_RD_A;
                    end else
                        load_cnt <= load_cnt + 1;
                end
            end
            // ---------------------------------------------------- butterfly
            // 5-clock schedule, dictated by the 1-clock RAM/ROM read latency:
            //
            //   state   | raddr set to | rdata holds | action
            //   --------+--------------+-------------+-------------------------
            //   S_RD_A  | idxA         | (stale)     | also issue twiddle read
            //   S_RD_B  | idxB         | (stale)     |
            //   S_MUL   |              | A           | capture A into a_re/a_im
            //   S_WRA   |              | B           | t = W*B (comb); write A'
            //   S_WRB   |              |             | write B'; step j/grp/stage
            S_RD_A: begin
                raddr   <= idxA;
                // twiddle index = j * N/m: stage 1 uses only W^0, the final
                // stage sweeps W^0..W^(N/2-1) in steps of 1
                tw_addr <= j << (LOG2N - stage);
                state   <= S_RD_B;
            end
            S_RD_B: begin
                raddr <= idxB;                     // rdata <= A at this edge
                state <= S_MUL;                    // tw_*_q <= ROM at this edge
            end
            S_MUL: begin
                a_re  <= rdata_re;                 // capture A
                a_im  <= rdata_im;                 // rdata <= B at this edge
                state <= S_WRA;
            end
            S_WRA: begin
                // rdata holds B, so t = W*B is valid combinationally right
                // now: write the A' leg and keep t for the B' leg next cycle
                t_re <= t_re_nxt;
                t_im <= t_im_nxt;
                ram_re[idxA] <= halve_sat(a_re_x + {{2{t_re_nxt[17]}}, t_re_nxt});
                ram_im[idxA] <= halve_sat(a_im_x + {{2{t_im_nxt[17]}}, t_im_nxt});
                state <= S_WRB;
            end
            S_WRB: begin
                ram_re[idxB] <= halve_sat(a_re_x - {{2{t_re[17]}}, t_re});
                ram_im[idxB] <= halve_sat(a_im_x - {{2{t_im[17]}}, t_im});
                // advance the nested loops: j within group, then grp across
                // the stage, then stage; after stage LOG2N go to unload
                if (j == half - 1) begin
                    j <= 0;
                    if (grp + m >= N) begin
                        grp <= 0;
                        if (stage == LOG2N) begin
                            unl_cnt <= 0;
                            raddr   <= 0;          // pre-issue read of bin 0
                            state   <= S_PRIME;
                        end else begin
                            stage <= stage + 1;
                            state <= S_RD_A;
                        end
                    end else begin
                        grp   <= grp + m;
                        state <= S_RD_A;
                    end
                end else begin
                    j     <= j + 1;
                    state <= S_RD_A;
                end
            end
            // ---------------------------------------------------- unload
            // The registered read means rdata always trails raddr by one
            // clock, so the address counter must run ahead of the output.
            // S_PRIME does the initial read-ahead (bin 0 was issued in
            // S_WRB, bin 1 here); then in S_UNL, while bin k sits on rdata
            // being emitted, the read of bin k+2 is issued.
            S_PRIME: begin
                raddr <= 1;                        // rdata <= ram[0] at this edge
                state <= S_UNL;
            end
            S_UNL: begin
                // rdata holds ram[unl_cnt]
                out_valid <= 1'b1;
                out_bin   <= unl_cnt[LOG2N-1:0];
                out_re    <= rdata_re;
                out_im    <= rdata_im;
                out_last  <= (unl_cnt == N/2);
                raddr     <= unl_cnt[LOG2N-1:0] + 2'd2;   // stay 2 ahead
                if (unl_cnt == N/2) begin
                    state    <= S_LOAD;            // ready for the next frame
                    load_cnt <= 0;
                end
                unl_cnt <= unl_cnt + 1;
            end
            default: state <= S_LOAD;
            endcase
        end
    end

endmodule
