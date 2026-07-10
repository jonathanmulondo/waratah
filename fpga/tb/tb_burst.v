//-----------------------------------------------------------------------------
// tb_burst.v -- self-checking testbench for burst_ctrl
//
// A mock qtn_core asserts spectrum_ready after accepting 50 gated
// samples and clears it on restart (mirroring restart_eff). Checks:
//   1. the gate stays shut in S_WAIT and during amplifier settle
//      (no qtn_valid until SETTLE_CLKS after amp_en rises),
//   2. amp_en drops when the spectrum completes,
//   3. two full collect cycles happen: spectra_count == 2, and restart
//      pulses exactly twice,
//   4. cadence: collections are separated by the idle gap.
//-----------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_burst;

    localparam CAD = 300;
    localparam SET = 20;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg                in_valid = 0;
    reg  signed [15:0] in_sample = 16'sd123;
    wire               qtn_valid;
    wire signed [15:0] qtn_sample;
    reg                spectrum_ready;
    wire               restart, amp_en, collecting;
    wire [15:0]        spectra_count;

    burst_ctrl #(.CADENCE_CLKS(CAD), .SETTLE_CLKS(SET)) dut (
        .clk(clk), .rst(rst), .enable(1'b1),
        .in_valid(in_valid), .in_sample(in_sample),
        .qtn_valid(qtn_valid), .qtn_sample(qtn_sample),
        .spectrum_ready(spectrum_ready), .restart(restart),
        .amp_en(amp_en), .spectra_count(spectra_count),
        .collecting(collecting)
    );

    // ------------------------------------------------- mock qtn_core
    integer acc;
    always @(posedge clk) begin
        if (rst) begin
            acc            <= 0;
            spectrum_ready <= 1'b0;
        end else begin
            if (restart && spectrum_ready) begin   // restart_eff behaviour
                spectrum_ready <= 1'b0;
                acc            <= 0;
            end
            if (qtn_valid && !spectrum_ready) begin
                if (acc == 49) begin
                    spectrum_ready <= 1'b1;
                    acc            <= 0;
                end else
                    acc <= acc + 1;
            end
        end
    end

    // ------------------------------------------------- monitors
    integer errors;
    integer restarts;
    integer amp_age;              // clocks since amp_en rose (big when off)

    always @(posedge clk) begin
        if (rst) begin
            restarts <= 0;
            amp_age  <= 100000;
        end else begin
            if (restart)
                restarts <= restarts + 1;
            if (amp_en)
                amp_age <= amp_age + 1;
            else
                amp_age <= 0;

            // gate must be shut while idle or while the amps settle
            if (qtn_valid && !amp_en) begin
                $display("FAIL: qtn_valid while amps off (t=%0t)", $time);
                errors = errors + 1;
            end
            if (qtn_valid && amp_en && amp_age < SET-1) begin
                $display("FAIL: qtn_valid during settle, amp_age=%0d", amp_age);
                errors = errors + 1;
            end
        end
    end

    integer t_spec1, t_amp2;

    initial begin
        errors = 0;

        repeat (4) @(posedge clk);
        rst <= 0;
        @(negedge clk);
        in_valid = 1'b1;               // continuous sample stream

        // gate must be shut during the initial cadence wait
        repeat (CAD/2) @(posedge clk);
        if (qtn_valid !== 1'b0 || amp_en !== 1'b0) begin
            $display("FAIL: active during initial S_WAIT");
            errors = errors + 1;
        end

        // first collection completes
        wait (spectrum_ready);
        t_spec1 = $time;
        repeat (3) @(posedge clk);
        if (amp_en !== 1'b0) begin
            $display("FAIL: amp_en still high after spectrum_ready");
            errors = errors + 1;
        end
        if (spectra_count !== 16'd1) begin
            $display("FAIL: spectra_count %0d after first cycle", spectra_count);
            errors = errors + 1;
        end

        // second collection: amp_en must rise no sooner than the cadence gap
        wait (amp_en);
        t_amp2 = $time;
        if (t_amp2 - t_spec1 < CAD*10 - 50) begin
            $display("FAIL: cadence gap %0d ns, expected >= %0d",
                     t_amp2 - t_spec1, CAD*10 - 50);
            errors = errors + 1;
        end

        wait (spectra_count == 16'd2);
        repeat (3) @(posedge clk);

        if (restarts !== 2) begin
            $display("FAIL: %0d restart pulses, expected 2", restarts);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_PASS: tb_burst (2 collections, cadence gap %0d ns)",
                     t_amp2 - t_spec1);
        else
            $display("TB_FAIL: tb_burst (%0d errors)", errors);
        $finish;
    end

    initial begin
        #200_000;
        $display("TB_FAIL: tb_burst timeout");
        $finish;
    end

endmodule
