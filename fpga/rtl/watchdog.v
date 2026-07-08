//-----------------------------------------------------------------------------
// watchdog.v -- watchdog timer, EDDI-FPGA
//
// The flight LPC4370 firmware never enabled its watchdog -- listed as a
// key reliability gap in the thesis. This module supervises the design:
// if `kick` is not pulsed within TIMEOUT clocks, `wdt_rst` is asserted
// for RST_PULSE clocks and the fired counter increments. The fired count
// and a sticky flag survive the watchdog reset (they are only cleared by
// the external hardware reset), so the payload computer can read out how
// many recoveries have happened -- the error-reporting scheme recommended
// in the thesis.
//-----------------------------------------------------------------------------
module watchdog #(
    parameter WIDTH     = 32,
    parameter TIMEOUT   = 32'd100_000_000,   // clocks (1 s at 100 MHz)
    parameter RST_PULSE = 8
) (
    input  wire        clk,
    input  wire        rst,          // external hardware reset
    input  wire        enable,
    input  wire        kick,

    output reg         wdt_rst,      // reset request to the rest of the design
    output reg         fired,        // sticky: at least one timeout occurred
    output reg [7:0]   fired_count
);

    reg [WIDTH-1:0] cnt;
    reg [7:0]       pulse_cnt;

    // a stuck-high kick must not silence the watchdog: only edges count
    reg  kick_d;
    wire kick_edge = kick && !kick_d;

    // parameter sanity: TIMEOUT must be reachable by the counter
    initial begin
        if (TIMEOUT > (1 << WIDTH))
            $display("ERROR: watchdog TIMEOUT %0d exceeds counter range (WIDTH=%0d)",
                     TIMEOUT, WIDTH);
    end

    always @(posedge clk) begin
        if (rst) begin
            cnt         <= {WIDTH{1'b0}};
            wdt_rst     <= 1'b0;
            fired       <= 1'b0;
            fired_count <= 8'd0;
            pulse_cnt   <= 8'd0;
            kick_d      <= 1'b0;
        end else begin
            kick_d <= kick;
            if (wdt_rst) begin
                // hold reset for RST_PULSE clocks
                if (pulse_cnt == RST_PULSE-1) begin
                    wdt_rst <= 1'b0;
                    cnt     <= {WIDTH{1'b0}};
                end else
                    pulse_cnt <= pulse_cnt + 1;
            end else if (!enable) begin
                cnt <= {WIDTH{1'b0}};
            end else if (kick_edge) begin
                cnt <= {WIDTH{1'b0}};
            end else if (cnt >= TIMEOUT-1) begin
                wdt_rst     <= 1'b1;
                pulse_cnt   <= 8'd0;
                fired       <= 1'b1;
                fired_count <= fired_count + 1;
            end else
                cnt <= cnt + 1;
        end
    end

endmodule
