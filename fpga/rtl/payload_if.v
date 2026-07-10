//-----------------------------------------------------------------------------
// payload_if.v -- payload-computer command/telemetry interface (EDDI-FPGA)
//
// WHY IT EXISTS
//   The flight instrument spoke to the BeagleBone payload computer with
//   single unframed command bytes over I2C -- no CRC, no ACK, no way to
//   tell a corrupted command from a real one. The thesis lists a framed
//   packet protocol with ACK/NACK and a heartbeat as a key fix. This
//   module implements exactly that, over UART (the physical link -- I2C
//   slave or UART -- is bus-dependent; UART is the SLIP-over-UART-style
//   link the WS-1 stack favours, and the framing layer here is
//   transport-agnostic).
//
// FRAME FORMAT (both directions)
//   [SOF=0xA5] [CMD] [LEN] [PAYLOAD x LEN] [CRC8]
//   CRC-8 (poly 0x07, init 0x00) over CMD, LEN and PAYLOAD -- not SOF.
//   Responses echo CMD with bit 7 set (= ACK). Errors return
//   [0xA5][0xEE][0x01][err][CRC]: err 1 = bad CRC, 2 = unknown command,
//   3 = bad arguments. A partial frame that stalls longer than
//   TIMEOUT_CLKS is silently abandoned (parser resynchronises on the
//   next 0xA5) -- corruption can cost a frame, never the link.
//
// COMMAND SET
//   0x01 PING        len 0            -> ACK (comms heartbeat)
//   0x02 GET_STATUS  len 0            -> 12-byte status block (see top)
//   0x03 READ_PSD    [hi][lo][n]      -> n 32-bit bins, big-endian,
//                                        starting at bin {hi,lo} (n<=32)
//   0x04 READ_BUF    [hi][lo][n]      -> n 16-bit debris-buffer samples,
//                                        big-endian (n<=64)
//   0x05 ACK_EVENT   len 0            -> ACK; releases the frozen debris
//                                        event (the detector re-arms)
//   0x06 SET_CONFIG  [cfg]            -> ACK; cfg = {gain[3:2],
//                                        debris_en[1], qtn_en[0]}
//
//   Every frame with a good CRC pulses `heartbeat` -- eddi_top uses it
//   (with spectrum completions) to kick the watchdog, so a live payload
//   computer keeps the instrument out of reset even when idle.
//
//   Large transfers are split into many small READ_ frames by the
//   payload computer; each is individually CRC-protected and
//   retransmittable, which is the point -- the flight link's raw
//   8 kB/s dumps had no way to recover a corrupted stretch.
//-----------------------------------------------------------------------------
module payload_if #(
    parameter CLKS_PER_BIT  = 868,           // 100 MHz / 115200 baud
    parameter PSD_AW        = 10,
    parameter BUF_AW        = 11,
    parameter TIMEOUT_CLKS  = 32'd2_000_000  // ~20 ms: inter-byte timeout
) (
    input  wire               clk,
    input  wire               rst,

    input  wire               rxd,
    output wire               txd,

    // telemetry snapshot, packed by eddi_top (byte 0 = status[95:88])
    input  wire [95:0]        status,

    // QTN mean-PSD read port (data valid 2 cycles after addr)
    output reg  [PSD_AW-1:0]  psd_addr,
    input  wire [31:0]        psd_data,

    // debris capture-buffer read port (data valid 1 cycle after addr)
    output reg  [BUF_AW-1:0]  buf_addr,
    input  wire signed [15:0] buf_data,

    // control strobes / registers
    output reg                ack_event,     // 1-clk pulse
    output reg  [7:0]         cfg,           // reset: QTN + debris enabled
    output reg                heartbeat      // 1-clk pulse per good frame
);

    localparam SOF = 8'hA5;

    localparam CMD_PING   = 8'h01;
    localparam CMD_STATUS = 8'h02;
    localparam CMD_RDPSD  = 8'h03;
    localparam CMD_RDBUF  = 8'h04;
    localparam CMD_ACKEV  = 8'h05;
    localparam CMD_CFG    = 8'h06;
    localparam CMD_NACK   = 8'hEE;

    // ------------------------------------------------------------- UART
    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data;
    reg        tx_valid;
    wire       tx_ready;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst), .rxd(rxd),
        .data(rx_data), .valid(rx_valid)
    );
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst(rst),
        .data(tx_data), .valid(tx_valid), .ready(tx_ready), .txd(txd)
    );

    // ------------------------------------------------------------- CRC-8
    // poly 0x07 (x^8+x^2+x+1), init 0x00, MSB first. Same function is
    // replicated in the testbench and must be in the payload computer's
    // driver.
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

    // ------------------------------------------------------------- FSM
    localparam R_SOF   = 4'd0;   // hunt for 0xA5
    localparam R_CMD   = 4'd1;
    localparam R_LEN   = 4'd2;
    localparam R_PAY   = 4'd3;
    localparam R_CRC   = 4'd4;
    localparam X_DEC   = 4'd5;   // decode + validate, choose the response
    localparam T_SOF   = 4'd6;
    localparam T_CMD   = 4'd7;
    localparam T_LEN   = 4'd8;
    localparam T_PAY   = 4'd9;
    localparam T_FETCH = 4'd10;  // read one PSD word / buffer sample
    localparam T_CRC   = 4'd11;

    reg [3:0]  state;

    // request side
    reg [7:0]  cmd, len, pcnt;
    reg [7:0]  paybuf [0:7];               // only first 8 payload bytes kept
    reg [7:0]  crc_rx;
    reg        crc_ok;
    reg [31:0] to_cnt;                     // inter-byte timeout

    // response side
    localparam K_NONE = 2'd0;              // header-only ACK
    localparam K_SREG = 2'd1;              // from the 96-bit shift register
    localparam K_PSD  = 2'd2;              // fetched from the PSD port
    localparam K_BUF  = 2'd3;              // fetched from the debris buffer

    reg [1:0]  kind;
    reg [7:0]  resp_cmd, resp_len;
    reg [95:0] sreg;                       // status / NACK-code shifter
    reg [31:0] word32;                     // current PSD word / buf sample
    reg [15:0] rd_start;
    reg [7:0]  byte_idx;
    reg        fetched;
    reg [1:0]  fcnt;
    reg [7:0]  crc_tx;
    reg [15:0] gap;                        // turnaround delay before replying

    // next payload byte to transmit: sources shift MSB-first so the
    // wire format is big-endian throughout
    wire [7:0] cur_byte = (kind == K_SREG) ? sreg[95:88] : word32[31:24];

    always @(posedge clk) begin
        if (rst) begin
            state     <= R_SOF;
            tx_valid  <= 1'b0;
            ack_event <= 1'b0;
            heartbeat <= 1'b0;
            cfg       <= 8'h03;            // both science paths enabled
            psd_addr  <= 0;
            buf_addr  <= 0;
            to_cnt    <= 0;
            pcnt      <= 0;
            crc_ok    <= 1'b0;
        end else begin
            // strobes are single-clock pulses
            ack_event <= 1'b0;
            heartbeat <= 1'b0;

            // abandon a frame that stalls mid-reception: the parser must
            // never get stuck waiting for bytes that will never come
            if (state == R_CMD || state == R_LEN ||
                state == R_PAY || state == R_CRC) begin
                if (rx_valid)
                    to_cnt <= 0;
                else if (to_cnt >= TIMEOUT_CLKS-1)
                    state <= R_SOF;
                else
                    to_cnt <= to_cnt + 1;
            end else
                to_cnt <= 0;

            case (state)
            // ------------------------------------------------ receive
            R_SOF:
                if (rx_valid && rx_data == SOF)
                    state <= R_CMD;
            R_CMD:
                if (rx_valid) begin
                    cmd    <= rx_data;
                    crc_rx <= crc8(8'h00, rx_data);
                    state  <= R_LEN;
                end
            R_LEN:
                if (rx_valid) begin
                    len    <= rx_data;
                    pcnt   <= 0;
                    crc_rx <= crc8(crc_rx, rx_data);
                    state  <= (rx_data == 0) ? R_CRC : R_PAY;
                end
            R_PAY:
                if (rx_valid) begin
                    if (pcnt < 8)
                        paybuf[pcnt[2:0]] <= rx_data;
                    crc_rx <= crc8(crc_rx, rx_data);
                    if (pcnt == len-1)
                        state <= R_CRC;
                    else
                        pcnt <= pcnt + 1;
                end
            R_CRC:
                if (rx_valid) begin
                    crc_ok <= (rx_data == crc_rx);
                    state  <= X_DEC;
                end
            // ------------------------------------------------ decode
            X_DEC: begin
                // defaults: NACK with "bad arguments"; each valid branch
                // overwrites them
                resp_cmd <= CMD_NACK;
                resp_len <= 8'd1;
                kind     <= K_SREG;
                sreg     <= {8'd3, 88'd0};

                if (!crc_ok)
                    sreg <= {8'd1, 88'd0};
                else begin
                    heartbeat <= 1'b1;     // intact frame = live payload PC
                    case (cmd)
                    CMD_PING: begin
                        resp_cmd <= CMD_PING | 8'h80;
                        resp_len <= 0;
                        kind     <= K_NONE;
                    end
                    CMD_STATUS: begin
                        resp_cmd <= CMD_STATUS | 8'h80;
                        resp_len <= 8'd12;
                        kind     <= K_SREG;
                        sreg     <= status;
                    end
                    CMD_RDPSD:
                        if (len == 3 && paybuf[2] != 0 && paybuf[2] <= 8'd32) begin
                            resp_cmd <= CMD_RDPSD | 8'h80;
                            resp_len <= {paybuf[2][5:0], 2'b00};   // n*4 bytes
                            kind     <= K_PSD;
                            rd_start <= {paybuf[0], paybuf[1]};
                        end
                    CMD_RDBUF:
                        if (len == 3 && paybuf[2] != 0 && paybuf[2] <= 8'd64) begin
                            resp_cmd <= CMD_RDBUF | 8'h80;
                            resp_len <= {paybuf[2][6:0], 1'b0};    // n*2 bytes
                            kind     <= K_BUF;
                            rd_start <= {paybuf[0], paybuf[1]};
                        end
                    CMD_ACKEV: begin
                        resp_cmd  <= CMD_ACKEV | 8'h80;
                        resp_len  <= 0;
                        kind      <= K_NONE;
                        ack_event <= 1'b1;
                    end
                    CMD_CFG:
                        if (len == 1) begin
                            resp_cmd <= CMD_CFG | 8'h80;
                            resp_len <= 0;
                            kind     <= K_NONE;
                            cfg      <= paybuf[0];
                        end
                    default:
                        sreg <= {8'd2, 88'd0};   // unknown command
                    endcase
                end
                // half-duplex turnaround: the request's CRC is decoded at
                // the CENTRE of its stop bit, so replying immediately puts
                // our start bit inside the commander's stop bit -- a
                // half-duplex master still switching to receive would miss
                // it. Two bit-times of silence before the response SOF.
                gap   <= 2*CLKS_PER_BIT;
                state <= T_SOF;
            end
            // ------------------------------------------------ respond
            // Each T_ state uses the same two-beat pattern: present the
            // byte to uart_tx for exactly one clock when it is ready,
            // then advance (uart_tx latched it on that clock's edge).
            T_SOF:
                if (gap != 0)
                    gap <= gap - 1;
                else if (tx_valid) begin
                    tx_valid <= 1'b0;
                    state    <= T_CMD;
                end else if (tx_ready) begin
                    tx_data  <= SOF;
                    tx_valid <= 1'b1;
                end
            T_CMD:
                if (tx_valid) begin
                    tx_valid <= 1'b0;
                    crc_tx   <= crc8(8'h00, resp_cmd);
                    state    <= T_LEN;
                end else if (tx_ready) begin
                    tx_data  <= resp_cmd;
                    tx_valid <= 1'b1;
                end
            T_LEN:
                if (tx_valid) begin
                    tx_valid <= 1'b0;
                    crc_tx   <= crc8(crc_tx, resp_len);
                    byte_idx <= 0;
                    fetched  <= 1'b0;
                    state    <= (resp_len == 0) ? T_CRC : T_PAY;
                end else if (tx_ready) begin
                    tx_data  <= resp_len;
                    tx_valid <= 1'b1;
                end
            T_PAY:
                if (tx_valid) begin
                    // byte accepted: fold it into the CRC, shift the
                    // source, decide whether the next word needs a fetch
                    tx_valid <= 1'b0;
                    crc_tx   <= crc8(crc_tx, cur_byte);
                    if (kind == K_SREG)
                        sreg <= {sreg[87:0], 8'h00};
                    else
                        word32 <= {word32[23:0], 8'h00};
                    if (kind == K_PSD && byte_idx[1:0] == 2'b11)
                        fetched <= 1'b0;
                    if (kind == K_BUF && byte_idx[0])
                        fetched <= 1'b0;
                    if (byte_idx == resp_len-1)
                        state <= T_CRC;
                    byte_idx <= byte_idx + 1;
                end else if ((kind == K_PSD || kind == K_BUF) && !fetched) begin
                    fcnt  <= 0;
                    state <= T_FETCH;
                end else if (tx_ready) begin
                    tx_data  <= cur_byte;
                    tx_valid <= 1'b1;
                end
            // fetch one item from the addressed RAM port. Four clocks
            // covers the slowest port (PSD: addr -> RAM read -> output
            // register -> latch here); both ports hold their data while
            // the address is held, so over-waiting is safe.
            T_FETCH: begin
                fcnt <= fcnt + 1;
                if (fcnt == 2'd0) begin
                    if (kind == K_PSD)
                        psd_addr <= rd_start[PSD_AW-1:0] + byte_idx[7:2];
                    else
                        buf_addr <= rd_start[BUF_AW-1:0] + byte_idx[7:1];
                end else if (fcnt == 2'd3) begin
                    word32  <= (kind == K_PSD) ? psd_data
                                               : {buf_data, 16'h0000};
                    fetched <= 1'b1;
                    state   <= T_PAY;
                end
            end
            T_CRC:
                if (tx_valid) begin
                    tx_valid <= 1'b0;
                    state    <= R_SOF;
                end else if (tx_ready) begin
                    tx_data  <= crc_tx;
                    tx_valid <= 1'b1;
                end
            default: state <= R_SOF;
            endcase
        end
    end

endmodule
