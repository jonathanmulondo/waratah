//-----------------------------------------------------------------------------
// tb_watchdog.v -- self-checking testbench for the watchdog timer
//
// Phase 1: kick regularly at half the timeout -> must never fire.
// Phase 2: stop kicking -> must fire once at TIMEOUT, pulse RST_PULSE
//          clocks, then run again and fire a second time.
//-----------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_watchdog;

    localparam TIMEOUT   = 32'd1000;
    localparam RST_PULSE = 8;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg  enable = 0;
    reg  kick   = 0;
    wire wdt_rst, fired;
    wire [7:0] fired_count;

    watchdog #(.TIMEOUT(TIMEOUT), .RST_PULSE(RST_PULSE)) dut (
        .clk(clk), .rst(rst), .enable(enable), .kick(kick),
        .wdt_rst(wdt_rst), .fired(fired), .fired_count(fired_count)
    );

    integer errors = 0;
    integer i, pulse_len;

    initial begin
        repeat (4) @(posedge clk);
        rst    <= 0;
        enable <= 1;

        // phase 1: 5 kicks at TIMEOUT/2 spacing, must not fire
        for (i = 0; i < 5; i = i + 1) begin
            repeat (TIMEOUT/2) @(posedge clk);
            kick <= 1'b1;
            @(posedge clk);
            kick <= 1'b0;
        end
        if (fired) begin
            $display("FAIL: fired while being kicked");
            errors = errors + 1;
        end

        // phase 2: stop kicking, expect fire at ~TIMEOUT
        i = 0;
        while (!wdt_rst && i < TIMEOUT*2) begin
            @(posedge clk);
            i = i + 1;
        end
        if (!wdt_rst) begin
            $display("FAIL: never fired after kicks stopped");
            errors = errors + 1;
        end else if (i < TIMEOUT - 4 || i > TIMEOUT + 4) begin
            $display("FAIL: fired after %0d clocks, expected ~%0d", i, TIMEOUT);
            errors = errors + 1;
        end

        // measure reset pulse width
        pulse_len = 0;
        while (wdt_rst) begin
            @(posedge clk);
            pulse_len = pulse_len + 1;
        end
        if (pulse_len !== RST_PULSE) begin
            $display("FAIL: reset pulse %0d clocks, expected %0d", pulse_len, RST_PULSE);
            errors = errors + 1;
        end
        if (fired_count !== 8'd1) begin
            $display("FAIL: fired_count=%0d, expected 1", fired_count);
            errors = errors + 1;
        end

        // second timeout with no kicks
        i = 0;
        while (!wdt_rst && i < TIMEOUT*2) begin
            @(posedge clk);
            i = i + 1;
        end
        while (wdt_rst) @(posedge clk);
        if (fired_count !== 8'd2) begin
            $display("FAIL: fired_count=%0d after 2nd timeout, expected 2", fired_count);
            errors = errors + 1;
        end
        if (!fired) begin
            $display("FAIL: sticky fired flag not set");
            errors = errors + 1;
        end

        if (errors == 0) $display("TB_PASS: tb_watchdog");
        else             $display("TB_FAIL: tb_watchdog (%0d errors)", errors);
        $finish;
    end

    initial begin
        #200_000;
        $display("TB_FAIL: tb_watchdog timeout");
        $finish;
    end

endmodule
