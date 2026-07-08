//-----------------------------------------------------------------------------
// tb_debris.v -- self-checking testbench for debris_detector
//
// Streams the gen_vectors.py stimulus: bounded uniform noise around a
// 1000-LSB DC offset with one damped-oscillation "impact" injected at
// DEB_SPIKE_AT. Checks that exactly one qualified event is detected, at
// the right sample index, and that the frozen capture buffer contains
// the spike with the trigger at the expected position.
//-----------------------------------------------------------------------------
`timescale 1ns/1ps
`include "tb_params.vh"

module tb_debris;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg                in_valid = 0;
    reg  signed [15:0] in_sample = 0;
    reg                ack = 0;
    wire               event_ready;
    wire [15:0]        event_count;
    wire [7:0]         events_dropped;
    wire [47:0]        trig_index;
    reg  [10:0]        buf_rd_addr = 0;
    wire signed [15:0] buf_rd_data;
    wire signed [15:0] cal_mean;
    wire [31:0]        cal_var;
    wire [15:0]        cal_zc;

    debris_detector dut (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_sample(in_sample),
        .ack(ack),
        .event_ready(event_ready), .event_count(event_count),
        .events_dropped(events_dropped), .trig_index(trig_index),
        .buf_rd_addr(buf_rd_addr), .buf_rd_data(buf_rd_data),
        .cal_mean(cal_mean), .cal_var(cal_var), .cal_zc(cal_zc)
    );

    reg signed [15:0] stim [0:`DEB_LEN-1];

    integer i, errors;
    integer max_val, max_pos;

    initial begin
        $readmemh("mem/debris_stim.mem", stim);
        errors = 0;

        repeat (4) @(posedge clk);
        rst <= 0;

        // stream samples, 1 valid per 4 clocks (models 50 ksps pacing)
        for (i = 0; i < `DEB_LEN; i = i + 1) begin
            @(posedge clk);
            in_valid  <= 1'b1;
            in_sample <= stim[i];
            @(posedge clk);
            in_valid <= 1'b0;
            @(posedge clk);
            @(posedge clk);
            // stop early once the event has frozen
            if (event_ready && i > `DEB_SPIKE_AT + 2000)
                i = `DEB_LEN;
        end
        in_valid <= 1'b0;
        repeat (8) @(posedge clk);

        $display("info: cal_mean=%0d cal_var=%0d cal_zc=%0d trig_index=%0d",
                 cal_mean, cal_var, cal_zc, trig_index);

        // ---- checks -------------------------------------------------
        if (event_count !== 16'd1) begin
            $display("FAIL: event_count=%0d, expected 1", event_count);
            errors = errors + 1;
        end
        if (!event_ready) begin
            $display("FAIL: event_ready not asserted");
            errors = errors + 1;
        end
        if (events_dropped !== 8'd0) begin
            $display("FAIL: events_dropped=%0d, expected 0", events_dropped);
            errors = errors + 1;
        end
        if (trig_index < `DEB_SPIKE_AT - 2 || trig_index > `DEB_SPIKE_AT + 80) begin
            $display("FAIL: trig_index=%0d not near spike at %0d",
                     trig_index, `DEB_SPIKE_AT);
            errors = errors + 1;
        end
        // calibration sanity: mean ~1000 LSB, uniform(-100,100) var ~3333
        if (cal_mean < 980 || cal_mean > 1020) begin
            $display("FAIL: cal_mean=%0d, expected ~1000", cal_mean);
            errors = errors + 1;
        end
        if (cal_var < 2500 || cal_var > 4200) begin
            $display("FAIL: cal_var=%0d, expected ~3333", cal_var);
            errors = errors + 1;
        end

        // read the frozen buffer, find the peak (relative to mean)
        max_val = 0;
        max_pos = -1;
        for (i = 0; i < 1536; i = i + 1) begin
            buf_rd_addr <= i[10:0];
            @(posedge clk);
            @(posedge clk);
            @(negedge clk);   // let the NBA update settle before sampling
            if ((buf_rd_data - cal_mean) > max_val) begin
                max_val = buf_rd_data - cal_mean;
                max_pos = i;
            end
        end
        if (max_val < 1500) begin
            $display("FAIL: buffer peak %0d LSB, expected > 1500", max_val);
            errors = errors + 1;
        end
        // trigger sample sits at logical address PRE_LEN=512; the first
        // oscillation peak follows within ~10 samples of it
        if (max_pos < 500 || max_pos > 600) begin
            $display("FAIL: buffer peak at %0d, expected near 512", max_pos);
            errors = errors + 1;
        end

        // ack releases the event
        ack <= 1'b1;
        @(posedge clk);
        ack <= 1'b0;
        @(posedge clk);
        if (event_ready) begin
            $display("FAIL: event_ready still set after ack");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_PASS: tb_debris (trig at %0d, buffer peak %0d LSB at %0d)",
                     trig_index, max_val, max_pos);
        else
            $display("TB_FAIL: tb_debris (%0d errors)", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("TB_FAIL: tb_debris timeout");
        $finish;
    end

endmodule
