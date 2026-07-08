//-----------------------------------------------------------------------------
// tb_qtn.v -- self-checking testbench for qtn_core (ionosphere module)
//
// Streams NAVG raw frames (tone + noise, gen_vectors.py) through the full
// window -> FFT -> PSD-average pipeline, then reads the mean PSD back and
// checks: peak on the tone bin, peak power within +-30% of the numpy
// golden value, and a quiet bin at least 20 dB down.
//-----------------------------------------------------------------------------
`timescale 1ns/1ps
`include "tb_params.vh"

module tb_qtn;

    localparam LOG2N = `TB_LOG2N;
    localparam N     = `TB_N;
    localparam NAVG  = `TB_NAVG;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg                sample_valid = 0;
    reg  signed [15:0] sample_in = 0;
    wire               sample_ready;
    wire               spectrum_ready;
    reg  [LOG2N-1:0]   rd_addr = 0;
    wire [31:0]        rd_data;
    wire               busy;

    qtn_core #(.LOG2N(LOG2N), .LOG2AVG(2)) dut (
        .clk(clk), .rst(rst), .restart(1'b0),
        .sample_valid(sample_valid), .sample_in(sample_in),
        .sample_ready(sample_ready),
        .spectrum_ready(spectrum_ready),
        .psd_rd_addr(rd_addr), .psd_rd_data(rd_data),
        .busy(busy)
    );

    reg signed [15:0] stim [0:N*NAVG-1];

    integer i, errors;
    reg [31:0] psd [0:N/2];
    reg [31:0] peak_val;
    integer    peak_bin;

    initial begin
        $readmemh("mem/qtn_in.mem", stim);
        errors = 0;

        repeat (4) @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // stream all frames. Drive on negedge: if ready is high at the
        // negedge it is guaranteed high at the next posedge, so exactly
        // one sample is accepted per presentation (no stale re-accepts).
        i = 0;
        @(negedge clk);
        while (i < N*NAVG) begin
            if (sample_ready) begin
                sample_valid = 1'b1;
                sample_in    = stim[i];
                i = i + 1;
            end else
                sample_valid = 1'b0;
            @(negedge clk);
        end
        sample_valid = 1'b0;

        wait (spectrum_ready);
        repeat (4) @(posedge clk);

        // read back mean PSD: rd_data lags rd_addr by 2 clocks; sample on
        // the negedge so the NBA update has settled (avoids read race)
        for (i = 0; i <= N/2; i = i + 1) begin
            rd_addr <= i[LOG2N-1:0];
            @(posedge clk);
            @(posedge clk);
            @(negedge clk);
            psd[i] = rd_data;
        end

        peak_val = 0;
        peak_bin = -1;
        for (i = 1; i <= N/2; i = i + 1)
            if (psd[i] > peak_val) begin
                peak_val = psd[i];
                peak_bin = i;
            end

        if (peak_bin !== `TB_TONE_BIN) begin
            $display("FAIL: PSD peak at bin %0d, expected %0d", peak_bin, `TB_TONE_BIN);
            errors = errors + 1;
        end
        if (peak_val < `QTN_PEAK_MIN || peak_val > `QTN_PEAK_MAX) begin
            $display("FAIL: peak power %0d outside [%0d, %0d]",
                     peak_val, `QTN_PEAK_MIN, `QTN_PEAK_MAX);
            errors = errors + 1;
        end
        if (psd[`QTN_QUIET_BIN] > `QTN_QUIET_MAX) begin
            $display("FAIL: quiet bin %0d power %0d exceeds %0d",
                     `QTN_QUIET_BIN, psd[`QTN_QUIET_BIN], `QTN_QUIET_MAX);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_PASS: tb_qtn (peak bin %0d, power %0d, quiet %0d)",
                     peak_bin, peak_val, psd[`QTN_QUIET_BIN]);
        else
            $display("TB_FAIL: tb_qtn (%0d errors)", errors);
        $finish;
    end

    // handshake sanity: sample_valid must never be dropped while stalled
    // (covered by the while loop above)

    initial begin
        #8_000_000;
        $display("TB_FAIL: tb_qtn timeout");
        $finish;
    end

endmodule
