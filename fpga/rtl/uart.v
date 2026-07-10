//-----------------------------------------------------------------------------
// uart.v -- minimal UART receiver + transmitter (EDDI-FPGA payload link)
//
// 8N1, no flow control. CLKS_PER_BIT = clk frequency / baud rate
// (e.g. 100 MHz / 115200 = 868). Two small modules in one file:
//
//   uart_rx : oversamples rxd, resynchronises on every start bit,
//             samples each data bit at its centre. `valid` pulses one
//             clock per received byte (only if the stop bit is high --
//             a framing error silently drops the byte, and the framed
//             packet protocol above recovers via CRC/timeout).
//
//   uart_tx : ready/valid input; shifts out start + 8 data (LSB first)
//             + stop. `ready` is high exactly when idle.
//-----------------------------------------------------------------------------

module uart_rx #(
    parameter CLKS_PER_BIT = 868
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       rxd,
    output reg  [7:0] data,
    output reg        valid          // 1-clk pulse per good byte
);

    localparam R_IDLE  = 2'd0;
    localparam R_START = 2'd1;
    localparam R_DATA  = 2'd2;
    localparam R_STOP  = 2'd3;

    // 2-FF synchroniser: rxd is asynchronous to clk
    reg r1, r2;
    always @(posedge clk) begin
        r1 <= rxd;
        r2 <= r1;
    end

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bidx;

    always @(posedge clk) begin
        if (rst) begin
            state <= R_IDLE;
            cnt   <= 0;
            bidx  <= 0;
            valid <= 1'b0;
        end else begin
            valid <= 1'b0;
            case (state)
            R_IDLE:
                if (!r2) begin             // falling edge = start bit
                    cnt   <= 0;
                    state <= R_START;
                end
            R_START:
                // wait to the middle of the start bit, confirm it is
                // still low (glitch filter), then sample every bit
                // CLKS_PER_BIT later -- i.e. at each bit's centre
                if (cnt == CLKS_PER_BIT/2 - 1) begin
                    cnt <= 0;
                    if (!r2) begin
                        bidx  <= 0;
                        state <= R_DATA;
                    end else
                        state <= R_IDLE;   // glitch, not a start bit
                end else
                    cnt <= cnt + 1;
            R_DATA:
                if (cnt == CLKS_PER_BIT-1) begin
                    cnt  <= 0;
                    data <= {r2, data[7:1]};    // LSB first
                    if (bidx == 3'd7)
                        state <= R_STOP;
                    else
                        bidx <= bidx + 1;
                end else
                    cnt <= cnt + 1;
            R_STOP:
                if (cnt == CLKS_PER_BIT-1) begin
                    cnt   <= 0;
                    valid <= r2;           // stop bit must be high
                    state <= R_IDLE;
                end else
                    cnt <= cnt + 1;
            endcase
        end
    end

endmodule


module uart_tx #(
    parameter CLKS_PER_BIT = 868
) (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       valid,
    output wire       ready,         // high when a byte can be accepted
    output wire       txd
);

    localparam T_IDLE = 1'b0;
    localparam T_SEND = 1'b1;

    reg        state;
    reg [9:0]  sh;                   // {stop, data[7:0], start}
    reg [3:0]  nbits;
    reg [15:0] cnt;

    assign ready = (state == T_IDLE);
    assign txd   = (state == T_SEND) ? sh[0] : 1'b1;   // idle line is high

    always @(posedge clk) begin
        if (rst) begin
            state <= T_IDLE;
            sh    <= 10'h3FF;
            nbits <= 0;
            cnt   <= 0;
        end else begin
            case (state)
            T_IDLE:
                if (valid) begin
                    sh    <= {1'b1, data, 1'b0};
                    nbits <= 4'd10;
                    cnt   <= 0;
                    state <= T_SEND;
                end
            T_SEND:
                if (cnt == CLKS_PER_BIT-1) begin
                    cnt <= 0;
                    sh  <= {1'b1, sh[9:1]};
                    if (nbits == 4'd1)
                        state <= T_IDLE;
                    else
                        nbits <= nbits - 1;
                end else
                    cnt <= cnt + 1;
            endcase
        end
    end

endmodule
