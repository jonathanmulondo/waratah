//-----------------------------------------------------------------------------
// tb_fft.v -- self-checking testbench for fft_r2
//
// Loads a pre-windowed 1024-sample tone-plus-noise frame (gen_vectors.py),
// runs the FFT and compares every output bin (0..N/2) against the numpy
// golden reference with a fixed-point tolerance. Also verifies the power
// peak lands on the expected tone bin.
//
// Run from fpga/sim:  vsim -c work.tb_fft -do "run -all; quit -f"
//-----------------------------------------------------------------------------
`timescale 1ns/1ps
`include "tb_params.vh"

module tb_fft;

    localparam LOG2N = `TB_LOG2N;
    localparam N     = `TB_N;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg                in_valid = 0;
    reg  signed [15:0] in_re = 0;
    wire               in_ready;
    wire               out_valid, out_last;
    wire [LOG2N-1:0]   out_bin;
    wire signed [15:0] out_re, out_im;
    wire               busy;

    fft_r2 #(.LOG2N(LOG2N)) dut (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_re(in_re), .in_ready(in_ready),
        .out_valid(out_valid), .out_bin(out_bin),
        .out_re(out_re), .out_im(out_im), .out_last(out_last),
        .busy(busy)
    );

    reg signed [15:0] stim    [0:N-1];
    reg signed [15:0] gold_re [0:N-1];
    reg signed [15:0] gold_im [0:N-1];

    integer i, errors;
    integer got_bins;
    reg signed [31:0] dre, dim, gre, gim, err_re, err_im, tol;
    reg [63:0] pow, peak_pow;
    integer peak_bin;
    reg done;

    initial begin
        $readmemh("mem/fft_in.mem",      stim);
        $readmemh("mem/fft_gold_re.mem", gold_re);
        $readmemh("mem/fft_gold_im.mem", gold_im);

        errors   = 0;
        got_bins = 0;
        peak_pow = 0;
        peak_bin = -1;
        done     = 0;

        repeat (4) @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // load N samples
        for (i = 0; i < N; i = i + 1) begin
            while (!in_ready) @(posedge clk);
            in_valid <= 1'b1;
            in_re    <= stim[i];
            @(posedge clk);
        end
        in_valid <= 1'b0;

        // wait for completion (checker below collects outputs)
        wait (done);

        if (got_bins !== N/2 + 1) begin
            $display("FAIL: expected %0d bins, got %0d", N/2+1, got_bins);
            errors = errors + 1;
        end
        if (peak_bin !== `TB_TONE_BIN) begin
            $display("FAIL: power peak at bin %0d, expected %0d", peak_bin, `TB_TONE_BIN);
            errors = errors + 1;
        end

        if (errors == 0) $display("TB_PASS: tb_fft (%0d bins checked, peak at bin %0d)",
                                  got_bins, peak_bin);
        else             $display("TB_FAIL: tb_fft (%0d errors)", errors);
        $finish;
    end

    // output checker
    always @(posedge clk) begin
        if (!rst && out_valid) begin
            dre = out_re;  dim = out_im;
            gre = gold_re[out_bin];  gim = gold_im[out_bin];

            err_re = (dre > gre) ? dre - gre : gre - dre;
            err_im = (dim > gim) ? dim - gim : gim - dim;
            tol    = ((gre < 0 ? -gre : gre) + (gim < 0 ? -gim : gim)) / 64 + 24;

            if (err_re > tol || err_im > tol) begin
                if (errors < 10)
                    $display("FAIL bin %0d: dut (%0d,%0d) gold (%0d,%0d) tol %0d",
                             out_bin, dre, dim, gre, gim, tol);
                errors = errors + 1;
            end

            pow = dre*dre + dim*dim;
            if (out_bin != 0 && pow > peak_pow) begin
                peak_pow = pow;
                peak_bin = out_bin;
            end

            got_bins = got_bins + 1;
            if (out_last) done = 1;
        end
    end

    // global timeout
    initial begin
        #2_000_000;
        $display("TB_FAIL: tb_fft timeout");
        $finish;
    end

endmodule
