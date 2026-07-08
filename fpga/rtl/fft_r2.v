//-----------------------------------------------------------------------------
// fft_r2.v -- Radix-2 DIT iterative FFT engine (EDDI-FPGA QTN path)
//
// N-point complex FFT of a real input stream. Input samples are loaded
// sequentially (0..N-1) and written to bit-reversed addresses; LOG2N
// in-place butterfly stages follow (5 clocks per butterfly); bins 0..N/2
// are then streamed out in order.
//
// Scaling: each stage divides by 2, so the output equals FFT(x)/N. With
// Q1.15 twiddles the output stays within 16 bits (saturating).
//
// Vendor-neutral Verilog-2001. ROMs initialised with $readmemh (supported
// for inference by Libero, Quartus, and Questa).
//-----------------------------------------------------------------------------
module fft_r2 #(
    parameter LOG2N      = 10,
    parameter N          = 1 << LOG2N,
    parameter TW_RE_FILE = "mem/tw_re_1024.mem",
    parameter TW_IM_FILE = "mem/tw_im_1024.mem"
) (
    input  wire               clk,
    input  wire               rst,

    // load interface: N real samples, sequential order
    input  wire               in_valid,
    input  wire signed [15:0] in_re,
    output wire               in_ready,

    // unload interface: bins 0..N/2 streamed in order
    output reg                out_valid,
    output reg  [LOG2N-1:0]   out_bin,
    output reg  signed [15:0] out_re,
    output reg  signed [15:0] out_im,
    output reg                out_last,

    output wire               busy
);

    // ------------------------------------------------------------- FSM states
    localparam S_LOAD  = 3'd0;  // accept N samples
    localparam S_RD_A  = 3'd1;  // issue read of A, twiddle
    localparam S_RD_B  = 3'd2;  // issue read of B
    localparam S_MUL   = 3'd3;  // capture A
    localparam S_WRA   = 3'd4;  // B on rdata; write A = (a + W*B)/2
    localparam S_WRB   = 3'd5;  // write B = (a - W*B)/2; advance indices
    localparam S_PRIME = 3'd6;  // prime read pipeline for unload
    localparam S_UNL   = 3'd7;  // stream bins 0..N/2

    reg [2:0] state;

    // ------------------------------------------------------------- data RAM
    reg signed [15:0] ram_re [0:N-1];
    reg signed [15:0] ram_im [0:N-1];
    reg [LOG2N-1:0]   raddr;
    reg signed [15:0] rdata_re, rdata_im;

    always @(posedge clk) begin
        rdata_re <= ram_re[raddr];
        rdata_im <= ram_im[raddr];
    end

    // ------------------------------------------------------------- twiddle ROM
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
    reg [LOG2N:0]   load_cnt;              // 0..N-1
    reg [3:0]       stage;                 // 1..LOG2N
    reg [LOG2N:0]   grp;                   // group base (steps of m)
    reg [LOG2N-1:0] j;                     // butterfly index within group
    reg [LOG2N:0]   unl_cnt;               // 0..N/2

    wire [LOG2N:0]   m    = 1 << stage;
    wire [LOG2N-1:0] half = 1 << (stage-1);

    wire [LOG2N-1:0] idxA = grp[LOG2N-1:0] + j;
    wire [LOG2N-1:0] idxB = grp[LOG2N-1:0] + j + half;

    function [LOG2N-1:0] bitrev;
        input [LOG2N-1:0] v;
        integer i;
        begin
            for (i = 0; i < LOG2N; i = i + 1)
                bitrev[i] = v[LOG2N-1-i];
        end
    endfunction

    // ------------------------------------------------------------- butterfly math
    reg signed [15:0] a_re, a_im;          // u = A[idxA]
    reg signed [17:0] t_re, t_im;          // t = W*B (registered for S_WRB)

    // W*B computed from the live RAM output (B) during S_WRA
    wire signed [31:0] p_rr = rdata_re * tw_re_q;
    wire signed [31:0] p_ii = rdata_im * tw_im_q;
    wire signed [31:0] p_ri = rdata_re * tw_im_q;
    wire signed [31:0] p_ir = rdata_im * tw_re_q;
    wire signed [32:0] mul_re = {p_rr[31], p_rr} - {p_ii[31], p_ii};
    wire signed [32:0] mul_im = {p_ri[31], p_ri} + {p_ir[31], p_ir};
    // round-to-nearest, Q2.30 -> Q1.15 (result fits 18 bits signed)
    wire signed [17:0] t_re_nxt = (mul_re + 33'sd16384) >>> 15;
    wire signed [17:0] t_im_nxt = (mul_im + 33'sd16384) >>> 15;

    // (u +/- t + 1)/2 with saturation to 16 bits
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
            out_valid <= 1'b0;
            out_last  <= 1'b0;

            case (state)
            // ---------------------------------------------------- load
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
            S_RD_A: begin
                raddr   <= idxA;
                tw_addr <= j << (LOG2N - stage);   // twiddle index j*N/m
                state   <= S_RD_B;
            end
            S_RD_B: begin
                raddr <= idxB;                     // rdata <= A at this edge
                state <= S_MUL;                    // tw_*_q  <= ROM at this edge
            end
            S_MUL: begin
                a_re  <= rdata_re;                 // capture A
                a_im  <= rdata_im;                 // rdata <= B at this edge
                state <= S_WRA;
            end
            S_WRA: begin
                // rdata holds B; t = W*B valid combinationally
                t_re <= t_re_nxt;
                t_im <= t_im_nxt;
                ram_re[idxA] <= halve_sat(a_re_x + {{2{t_re_nxt[17]}}, t_re_nxt});
                ram_im[idxA] <= halve_sat(a_im_x + {{2{t_im_nxt[17]}}, t_im_nxt});
                state <= S_WRB;
            end
            S_WRB: begin
                ram_re[idxB] <= halve_sat(a_re_x - {{2{t_re[17]}}, t_re});
                ram_im[idxB] <= halve_sat(a_im_x - {{2{t_im[17]}}, t_im});
                // advance j -> grp -> stage
                if (j == half - 1) begin
                    j <= 0;
                    if (grp + m >= N) begin
                        grp <= 0;
                        if (stage == LOG2N) begin
                            unl_cnt <= 0;
                            raddr   <= 0;
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
                raddr     <= unl_cnt[LOG2N-1:0] + 2'd2;
                if (unl_cnt == N/2) begin
                    state    <= S_LOAD;
                    load_cnt <= 0;
                end
                unl_cnt <= unl_cnt + 1;
            end
            default: state <= S_LOAD;
            endcase
        end
    end

endmodule
