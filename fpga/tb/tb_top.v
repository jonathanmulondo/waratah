//-----------------------------------------------------------------------------
// tb_top.v -- integration smoke test for eddi_top
//
// The headline claim, demonstrated end-to-end: QTN spectra and debris
// detection running CONCURRENTLY off one shared sample bus, commanded
// over the framed UART link, supervised by the watchdog.
//
// Scaled-down configuration for simulation speed: CIC 16:1 (not 800:1),
// cadence 50k clocks (not 1 s), UART at CLKS_PER_BIT=16, watchdog 1M.
//
// Stimulus: a 0.45 FS tone at bin 64 (mem/top_in.mem) -- exactly one
// cycle per decimated sample, i.e. on a CIC null, so the debris path
// sees a silent background and only the injected impact. (An off-null
// tone folds through the CIC into the debris band and causes trigger
// storms -- demonstrated with the bin-77 vector and recorded as a
// design constraint in the README.)
//
// Timeline:
//   - burst_ctrl collects and completes an averaged spectrum
//   - READ_PSD over UART: tone bin is hot, distant bin is quiet
//   - at input sample 300000 an impact transient is injected (step
//     +12000, exponential decay): the debris path must trigger on it
//     (concurrently with the second QTN collection)
//   - GET_STATUS over UART: alive, event_ready set, event_count == 1
//   - READ_BUF around the trigger point: captured amplitude is large
//   - ACK_EVENT: detector re-arms, event_ready falls
//   - stimulus and comms stop: the watchdog must fire; GET_STATUS after
//     recovery reports fired_count >= 1 (evidence survives the reset)
//-----------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_top;

    localparam CPB = 16;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg                adc_valid = 0;
    reg  signed [15:0] adc_sample = 0;
    reg                rxd = 1;
    wire               txd;
    wire               qtn_amp_en;
    wire [1:0]         gain_sel;
    wire               wdt_rst_out, spectrum_ready, event_ready;

    eddi_top #(
        .LOG2N(10), .LOG2AVG(2),
        .CIC_R(16), .CIC_W(48), .CIC_SHIFT(12),
        .CADENCE_CLKS(32'd50_000), .SETTLE_CLKS(32'd50),
        .CLKS_PER_BIT(CPB),
        .WDT_TIMEOUT(32'd1_000_000)
    ) dut (
        .clk(clk), .rst_ext(rst),
        .adc_valid(adc_valid), .adc_sample(adc_sample),
        .rxd(rxd), .txd(txd),
        .qtn_amp_en(qtn_amp_en), .gain_sel(gain_sel),
        .wdt_rst_out(wdt_rst_out),
        .spectrum_ready(spectrum_ready), .event_ready(event_ready)
    );

    // ---------------------------------------------- stimulus
    // bin-64 tone from gen_vectors.py (CIC null -> silent debris
    // background), replayed cyclically, plus an injected impact:
    // +12000 step decaying with tau ~2048 samples (slow and unipolar
    // -> must qualify as debris)
    reg signed [15:0] stim [0:4095];

    integer icnt;
    reg [15:0] imp;
    reg        drive_en;

    always @(posedge clk) begin
        if (rst) begin
            icnt <= 0;
            imp  <= 0;
        end else if (drive_en) begin
            adc_valid  <= 1'b1;
            adc_sample <= stim[icnt[11:0]] + $signed({1'b0, imp});
            icnt       <= icnt + 1;
            if (icnt == 300_000)
                imp <= 16'd12000;
            else if (imp != 0)
                // decay with a floor so the offset reaches exactly 0
                imp <= (imp > (imp >> 11) + 1) ? imp - ((imp >> 11) + 1)
                                               : 16'd0;
        end else
            adc_valid <= 1'b0;
    end

    // ---------------------------------------------- monitors
    reg saw_spectrum, saw_wdt;
    always @(posedge clk) begin
        if (rst) begin
            saw_spectrum <= 1'b0;
            saw_wdt      <= 1'b0;
        end else begin
            if (spectrum_ready) saw_spectrum <= 1'b1;
            if (wdt_rst_out)    saw_wdt      <= 1'b1;
        end
    end

    // ---------------------------------------------- CRC-8 + UART tasks
    function [7:0] crc8;
        input [7:0] c;
        input [7:0] d;
        integer k;
        reg [7:0] x;
        begin
            x = c ^ d;
            for (k = 0; k < 8; k = k + 1)
                x = x[7] ? ((x << 1) ^ 8'h07) : (x << 1);
            crc8 = x;
        end
    endfunction

    task send_byte;
        input [7:0] b;
        integer k;
        begin
            rxd = 0;
            repeat (CPB) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin
                rxd = b[k];
                repeat (CPB) @(posedge clk);
            end
            rxd = 1;
            repeat (CPB) @(posedge clk);
        end
    endtask

    task recv_byte;
        output [7:0] b;
        integer k;
        begin
            @(negedge txd);
            repeat (CPB/2) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin
                repeat (CPB) @(posedge clk);
                b[k] = txd;
            end
            repeat (CPB) @(posedge clk);
        end
    endtask

    reg [7:0] p0, p1, p2;
    task send_frame;
        input [7:0] c;
        input [7:0] l;
        reg [7:0] crc;
        begin
            crc = crc8(8'h00, c);
            crc = crc8(crc, l);
            send_byte(8'hA5);
            send_byte(c);
            send_byte(l);
            if (l >= 1) begin send_byte(p0); crc = crc8(crc, p0); end
            if (l >= 2) begin send_byte(p1); crc = crc8(crc, p1); end
            if (l >= 3) begin send_byte(p2); crc = crc8(crc, p2); end
            send_byte(crc);
        end
    endtask

    reg [7:0] r_cmd, r_len;
    reg [7:0] r_pay [0:127];
    integer errors;
    task recv_frame;
        reg [7:0] b, crc;
        integer k;
        begin
            recv_byte(b);
            if (b !== 8'hA5) begin
                $display("FAIL: response SOF %02x", b);
                errors = errors + 1;
            end
            recv_byte(r_cmd);
            recv_byte(r_len);
            crc = crc8(8'h00, r_cmd);
            crc = crc8(crc, r_len);
            for (k = 0; k < r_len; k = k + 1) begin
                recv_byte(b);
                r_pay[k] = b;
                crc = crc8(crc, b);
            end
            recv_byte(b);
            if (b !== crc) begin
                $display("FAIL: response CRC %02x, expected %02x", b, crc);
                errors = errors + 1;
            end
        end
    endtask

    integer i;
    reg [31:0] w;
    reg signed [15:0] s, smax;

    initial begin
        $readmemh("mem/top_in.mem", stim);
        errors   = 0;
        drive_en = 1;

        repeat (8) @(posedge clk);
        rst <= 0;

        // ---------------------------------------- 1. a spectrum completes
        // first collection: 50k wait + settle + 4 frames x ~28k clocks
        wait (spectrum_ready === 1'b1);
        $display("note: first spectrum at %0t", $time);

        // ---------------------------------------- 1b. READ_PSD spot check
        // read during the burst controller's idle gap (~50k clocks), well
        // before the next collection clears the average. Tone bin 64
        // carries ~13.6e6; bin 32 only leakage.
        p0 = 8'h00; p1 = 8'd64; p2 = 8'd1;
        send_frame(8'h03, 8'd3);
        recv_frame;
        if (r_cmd !== 8'h83 || r_len !== 8'd4) begin
            $display("FAIL: READ_PSD hdr %02x/%0d", r_cmd, r_len);
            errors = errors + 1;
        end
        w = {r_pay[0], r_pay[1], r_pay[2], r_pay[3]};
        if (w < 32'd1_000_000) begin
            $display("FAIL: PSD tone bin = %0d, expected > 1e6", w);
            errors = errors + 1;
        end
        p0 = 8'h00; p1 = 8'd32; p2 = 8'd1;
        send_frame(8'h03, 8'd3);
        recv_frame;
        w = {r_pay[0], r_pay[1], r_pay[2], r_pay[3]};
        if (w > 32'd100_000) begin
            $display("FAIL: PSD quiet bin = %0d, expected < 1e5", w);
            errors = errors + 1;
        end

        // ---------------------------------------- 2. debris event triggers
        // (impact injected at input 300k, during the second QTN
        // collection -- both paths active at once; capture takes 1024
        // decimated samples = ~16k clocks)
        wait (event_ready === 1'b1);
        $display("note: debris event at %0t", $time);

        // ---------------------------------------- 3. GET_STATUS
        send_frame(8'h02, 8'd0);
        recv_frame;
        if (r_cmd !== 8'h82 || r_len !== 8'd12) begin
            $display("FAIL: GET_STATUS hdr %02x/%0d", r_cmd, r_len);
            errors = errors + 1;
        end
        if (r_pay[0][0] !== 1'b1) begin
            $display("FAIL: alive flag clear");
            errors = errors + 1;
        end
        if (r_pay[0][2] !== 1'b1) begin
            $display("FAIL: event_ready flag clear in status");
            errors = errors + 1;
        end
        if (r_pay[0][4] !== 1'b0) begin
            $display("FAIL: wdt_fired flag set prematurely");
            errors = errors + 1;
        end
        if ({r_pay[1], r_pay[2]} !== 16'd1) begin
            $display("FAIL: event_count %0d, expected 1", {r_pay[1], r_pay[2]});
            errors = errors + 1;
        end
        if (r_pay[3] !== 8'd0) begin
            $display("FAIL: events_dropped %0d", r_pay[3]);
            errors = errors + 1;
        end

        // ---------------------------------------- 4. READ_BUF at the trigger
        // logical addr 512 = the trigger sample; the impact rise spans a
        // couple of decimated samples, so check the max of 4
        p0 = 8'h02; p1 = 8'h00; p2 = 8'd4;     // start 0x0200 = 512
        send_frame(8'h04, 8'd3);
        recv_frame;
        if (r_cmd !== 8'h84 || r_len !== 8'd8) begin
            $display("FAIL: READ_BUF hdr %02x/%0d", r_cmd, r_len);
            errors = errors + 1;
        end
        smax = -16'sd32768;
        for (i = 0; i < 4; i = i + 1) begin
            s = {r_pay[2*i], r_pay[2*i+1]};
            if (s > smax) smax = s;
        end
        if (smax < 16'sd4000) begin
            $display("FAIL: captured impact peak %0d, expected > 4000", smax);
            errors = errors + 1;
        end

        // ---------------------------------------- 5. ACK_EVENT re-arms
        send_frame(8'h05, 8'd0);
        recv_frame;
        repeat (8) @(posedge clk);
        if (event_ready !== 1'b0) begin
            $display("FAIL: event_ready still set after ACK_EVENT");
            errors = errors + 1;
        end

        // ---------------------------------------- 6. watchdog fires on silence
        drive_en = 0;                           // no samples, no comms
        wait (saw_wdt === 1'b1);
        $display("note: watchdog fired at %0t", $time);
        repeat (100) @(posedge clk);            // ride out the reset pulse

        send_frame(8'h02, 8'd0);                // link must work post-reset
        recv_frame;
        if (r_cmd !== 8'h82) begin
            $display("FAIL: GET_STATUS after wdt reset: cmd %02x", r_cmd);
            errors = errors + 1;
        end
        if (r_pay[0][4] !== 1'b1) begin
            $display("FAIL: wdt_fired flag clear after fire");
            errors = errors + 1;
        end
        if (r_pay[4] < 8'd1) begin
            $display("FAIL: fired_count %0d, expected >= 1", r_pay[4]);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_PASS: tb_top (spectrum + debris concurrent, UART ok, wdt recovery ok)");
        else
            $display("TB_FAIL: tb_top (%0d errors)", errors);
        $finish;
    end

    initial begin
        #40_000_000;
        $display("TB_FAIL: tb_top timeout");
        $finish;
    end

endmodule
