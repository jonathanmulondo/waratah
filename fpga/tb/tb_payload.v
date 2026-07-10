//-----------------------------------------------------------------------------
// tb_payload.v -- self-checking testbench for payload_if (framed UART)
//
// Drives real UART waveforms (CLKS_PER_BIT=16 for speed) against stub
// PSD / debris-buffer memories with the correct read latencies, and
// exercises the whole command set plus the error paths:
//   PING, GET_STATUS, READ_PSD, READ_BUF, ACK_EVENT, SET_CONFIG,
//   corrupted CRC -> NACK(1), unknown command -> NACK(2),
//   heartbeat pulse count == number of intact frames.
//-----------------------------------------------------------------------------
`timescale 1ns/1ps

module tb_payload;

    localparam CPB = 16;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg  rxd = 1;
    wire txd;

    reg  [95:0] status;
    wire [9:0]  psd_addr;
    reg  [31:0] psd_data;
    wire [10:0] buf_addr;
    reg  signed [15:0] buf_data;
    wire        ack_event;
    wire [7:0]  cfg;
    wire        heartbeat;

    payload_if #(.CLKS_PER_BIT(CPB), .PSD_AW(10), .BUF_AW(11),
                 .TIMEOUT_CLKS(32'd100_000)) dut (
        .clk(clk), .rst(rst), .rxd(rxd), .txd(txd),
        .status(status),
        .psd_addr(psd_addr), .psd_data(psd_data),
        .buf_addr(buf_addr), .buf_data(buf_data),
        .ack_event(ack_event), .cfg(cfg), .heartbeat(heartbeat)
    );

    // ---------------------------------------------- stub memories
    // PSD port: 2-cycle latency, value = addr*3 + 7
    reg [31:0] psd_p;
    always @(posedge clk) begin
        psd_p    <= {22'd0, psd_addr} * 32'd3 + 32'd7;
        psd_data <= psd_p;
    end
    // debris buffer port: 1-cycle latency, value = addr*5 - 100
    always @(posedge clk)
        buf_data <= $signed({5'd0, buf_addr}) * 16'sd5 - 16'sd100;

    // ---------------------------------------------- monitors
    integer hb_cnt, ack_cnt;
    always @(posedge clk) begin
        if (rst) begin
            hb_cnt  <= 0;
            ack_cnt <= 0;
        end else begin
            if (heartbeat) hb_cnt  <= hb_cnt + 1;
            if (ack_event) ack_cnt <= ack_cnt + 1;
        end
    end

    // ---------------------------------------------- CRC-8 (mirror of RTL)
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

    // ---------------------------------------------- UART bit-banging
    task send_byte;
        input [7:0] b;
        integer k;
        begin
            rxd = 0;                                  // start
            repeat (CPB) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin       // data, LSB first
                rxd = b[k];
                repeat (CPB) @(posedge clk);
            end
            rxd = 1;                                  // stop
            repeat (CPB) @(posedge clk);
        end
    endtask

    task recv_byte;
        output [7:0] b;
        integer k;
        begin
            @(negedge txd);
            repeat (CPB/2) @(posedge clk);            // centre of start bit
            for (k = 0; k < 8; k = k + 1) begin
                repeat (CPB) @(posedge clk);          // centre of bit k
                b[k] = txd;
            end
            repeat (CPB) @(posedge clk);              // ride out stop bit
        end
    endtask

    // send a frame; pay0..2 used according to len; crc_offset corrupts
    reg [7:0] p0, p1, p2;
    task send_frame;
        input [7:0] c;
        input [7:0] l;
        input [7:0] crc_offset;
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
            send_byte(crc + crc_offset);
        end
    endtask

    // receive a full response frame into r_cmd/r_len/r_pay, check CRC
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

    task expect_hdr;
        input [7:0] ec;
        input [7:0] el;
        input [127:0] what;
        begin
            if (r_cmd !== ec || r_len !== el) begin
                $display("FAIL: %0s -> cmd %02x len %0d, expected %02x/%0d",
                         what, r_cmd, r_len, ec, el);
                errors = errors + 1;
            end
        end
    endtask

    integer i;
    reg [31:0] w;
    reg signed [15:0] s;

    initial begin
        errors = 0;
        status = 96'h01_0203_04_05_0607_08090A0B_0C;   // bytes 01..0C

        repeat (8) @(posedge clk);
        rst <= 0;
        repeat (8) @(posedge clk);

        // ------------------------------------------------ 1. PING
        send_frame(8'h01, 8'd0, 8'd0);
        recv_frame;
        expect_hdr(8'h81, 8'd0, "PING");

        // ------------------------------------------------ 2. GET_STATUS
        send_frame(8'h02, 8'd0, 8'd0);
        recv_frame;
        expect_hdr(8'h82, 8'd12, "GET_STATUS");
        for (i = 0; i < 12; i = i + 1)
            if (r_pay[i] !== status[95-8*i -: 8]) begin
                $display("FAIL: status byte %0d = %02x, expected %02x",
                         i, r_pay[i], status[95-8*i -: 8]);
                errors = errors + 1;
            end

        // ------------------------------------------------ 3. READ_PSD
        p0 = 8'h00; p1 = 8'd5; p2 = 8'd2;      // bins 5..6
        send_frame(8'h03, 8'd3, 8'd0);
        recv_frame;
        expect_hdr(8'h83, 8'd8, "READ_PSD");
        for (i = 0; i < 2; i = i + 1) begin
            w = {r_pay[4*i], r_pay[4*i+1], r_pay[4*i+2], r_pay[4*i+3]};
            if (w !== (5+i)*3 + 7) begin
                $display("FAIL: PSD word %0d = %0d, expected %0d",
                         i, w, (5+i)*3 + 7);
                errors = errors + 1;
            end
        end

        // ------------------------------------------------ 4. READ_BUF
        p0 = 8'h00; p1 = 8'd3; p2 = 8'd2;      // samples 3..4
        send_frame(8'h04, 8'd3, 8'd0);
        recv_frame;
        expect_hdr(8'h84, 8'd4, "READ_BUF");
        for (i = 0; i < 2; i = i + 1) begin
            s = {r_pay[2*i], r_pay[2*i+1]};
            if (s !== (3+i)*5 - 100) begin
                $display("FAIL: BUF sample %0d = %0d, expected %0d",
                         i, s, (3+i)*5 - 100);
                errors = errors + 1;
            end
        end

        // ------------------------------------------------ 5. bad CRC -> NACK(1)
        send_frame(8'h01, 8'd0, 8'd1);         // corrupted CRC
        recv_frame;
        expect_hdr(8'hEE, 8'd1, "bad-CRC NACK");
        if (r_pay[0] !== 8'd1) begin
            $display("FAIL: bad-CRC err code %0d", r_pay[0]);
            errors = errors + 1;
        end

        // ------------------------------------------------ 6. unknown cmd -> NACK(2)
        send_frame(8'h7A, 8'd0, 8'd0);
        recv_frame;
        expect_hdr(8'hEE, 8'd1, "unknown-cmd NACK");
        if (r_pay[0] !== 8'd2) begin
            $display("FAIL: unknown-cmd err code %0d", r_pay[0]);
            errors = errors + 1;
        end

        // ------------------------------------------------ 7. ACK_EVENT
        send_frame(8'h05, 8'd0, 8'd0);
        recv_frame;
        expect_hdr(8'h85, 8'd0, "ACK_EVENT");
        if (ack_cnt !== 1) begin
            $display("FAIL: ack_event pulsed %0d times, expected 1", ack_cnt);
            errors = errors + 1;
        end

        // ------------------------------------------------ 8. SET_CONFIG
        p0 = 8'h0B;                            // gain=2, debris on, qtn on
        send_frame(8'h06, 8'd1, 8'd0);
        recv_frame;
        expect_hdr(8'h86, 8'd0, "SET_CONFIG");
        if (cfg !== 8'h0B) begin
            $display("FAIL: cfg = %02x, expected 0B", cfg);
            errors = errors + 1;
        end

        // ------------------------------------------------ heartbeat count
        // every intact frame pulses it (including unknown-cmd), the
        // corrupted one must not: 7 total
        repeat (8) @(posedge clk);
        if (hb_cnt !== 7) begin
            $display("FAIL: %0d heartbeats, expected 7", hb_cnt);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_PASS: tb_payload (8 exchanges, %0d heartbeats)", hb_cnt);
        else
            $display("TB_FAIL: tb_payload (%0d errors)", errors);
        $finish;
    end

    initial begin
        #3_000_000;
        $display("TB_FAIL: tb_payload timeout");
        $finish;
    end

endmodule
