//-----------------------------------------------------------------------------
// tb_cic.v -- self-checking testbench for cic_decim
//
// Runs two instances in parallel: the TB-friendly R=16 config and the
// flight R=800 config. Checks:
//   1. DC gain is exact: R=16/SHIFT=12 has unity gain (1000 -> 1000);
//      R=800/SHIFT=29 has gain 800^3/2^29 (1000 -> 953, floor).
//   2. A slow square wave's plateaus pass through exactly (the CIC step
//      response is monotone -- no overshoot), so min/max of the output
//      equal the plateau levels once settled.
//   3. Output count = input count / R.
//-----------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_cic;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg                in_valid = 0;
    reg  signed [15:0] in_sample = 0;

    wire               ov16, ov800;
    wire signed [15:0] os16, os800;

    cic_decim #(.R(16),  .W(48), .SHIFT(12)) dut16 (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_sample(in_sample),
        .out_valid(ov16), .out_sample(os16)
    );
    cic_decim #(.R(800), .W(48), .SHIFT(29)) dut800 (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_sample(in_sample),
        .out_valid(ov800), .out_sample(os800)
    );

    integer n16, n800;
    reg signed [15:0] last16, last800;
    reg signed [15:0] min16, max16, min800, max800;
    integer track;                 // 1: record min/max

    always @(posedge clk) begin
        if (ov16) begin
            last16 <= os16;
            n16    <= n16 + 1;
            if (track) begin
                if (os16 < min16) min16 <= os16;
                if (os16 > max16) max16 <= os16;
            end
        end
        if (ov800) begin
            last800 <= os800;
            n800    <= n800 + 1;
            if (track) begin
                if (os800 < min800) min800 <= os800;
                if (os800 > max800) max800 <= os800;
            end
        end
    end

    integer i, errors;
    reg signed [15:0] dc16, dc800;

    initial begin
        errors = 0;
        n16 = 0; n800 = 0;
        track = 0;
        min16 = 16'sd32767;  max16 = -16'sd32768;
        min800 = 16'sd32767; max800 = -16'sd32768;

        repeat (4) @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // ---------------------------------------------------- phase 1: DC
        // 8000 samples of DC 1000: both filters fully settled (worst case
        // 3 output periods = 2400 inputs for R=800)
        for (i = 0; i < 8000; i = i + 1) begin
            @(negedge clk);
            in_valid  = 1'b1;
            in_sample = 16'sd1000;
        end
        @(negedge clk);
        in_valid = 1'b0;
        repeat (4) @(posedge clk);

        dc16  = last16;
        dc800 = last800;
        if (last16 !== 16'sd1000) begin
            $display("FAIL: R=16 DC out %0d, expected 1000 (exact)", last16);
            errors = errors + 1;
        end
        // 1000 * 800^3 >> 29 = 953 (floor of 953.67)
        if (last800 !== 16'sd953) begin
            $display("FAIL: R=800 DC out %0d, expected 953", last800);
            errors = errors + 1;
        end

        // ---------------------------------------------------- phase 2: square
        // period 8192 (plateaus of 4096 > 3*R for both configs), 3 periods.
        // settle one period before recording extremes.
        for (i = 0; i < 8192; i = i + 1) begin
            @(negedge clk);
            in_valid  = 1'b1;
            in_sample = (i[12]) ? -16'sd8000 : 16'sd8000;
        end
        track = 1;
        for (i = 8192; i < 32768; i = i + 1) begin
            @(negedge clk);
            in_valid  = 1'b1;
            in_sample = (i[12]) ? -16'sd8000 : 16'sd8000;
        end
        @(negedge clk);
        in_valid = 1'b0;
        track = 0;
        repeat (4) @(posedge clk);

        // R=16 unity gain: plateaus exact
        if (max16 !== 16'sd8000 || min16 !== -16'sd8000) begin
            $display("FAIL: R=16 square extremes %0d/%0d, expected +/-8000",
                     min16, max16);
            errors = errors + 1;
        end
        // R=800 gain 0.9537: plateaus ~7629 (allow rounding slack)
        if (max800 < 16'sd7620 || max800 > 16'sd7640 ||
            min800 > -16'sd7620 || min800 < -16'sd7640) begin
            $display("FAIL: R=800 square extremes %0d/%0d, expected ~+/-7629",
                     min800, max800);
            errors = errors + 1;
        end

        // ---------------------------------------------------- output counts
        // 8001 + 24577 inputs driven = 40770... count exactly: phase1 8000,
        // phase2 32768 -> 40768 inputs total
        if (n16 !== 40768/16) begin
            $display("FAIL: R=16 produced %0d outputs, expected %0d",
                     n16, 40768/16);
            errors = errors + 1;
        end
        if (n800 !== 40768/800) begin
            $display("FAIL: R=800 produced %0d outputs, expected %0d",
                     n800, 40768/800);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_PASS: tb_cic (DC 1000->%0d/%0d, square %0d/%0d)",
                     dc16, dc800, max16, max800);
        else
            $display("TB_FAIL: tb_cic (%0d errors)", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("TB_FAIL: tb_cic timeout");
        $finish;
    end

endmodule
