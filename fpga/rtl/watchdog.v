//-----------------------------------------------------------------------------
// watchdog.v -- watchdog timer, EDDI-FPGA
//
// WHY IT EXISTS
//   The flight LPC4370 firmware never enabled its watchdog -- listed as a
//   key reliability gap in the thesis. On orbit there is no one to press
//   reset: if the design hangs (SEU-flipped state machine, stuck
//   handshake, latched-up peripheral), only an autonomous timeout can
//   recover it. CUAVA-2 was lost to a suspected radiation event, so this
//   is not a theoretical concern.
//
// HOW IT WORKS
//   The healthy design must pulse `kick` at least once every TIMEOUT
//   clocks (e.g. once per completed spectrum). If it doesn't, `wdt_rst`
//   asserts for RST_PULSE clocks -- wire it into the reset tree of
//   everything except this module.
//
//   Two details matter:
//   - Only kick EDGES count. If the supervised logic dies with kick stuck
//     high, a level-sensitive check would never time out -- exactly the
//     failure the watchdog exists to catch.
//   - `fired` and `fired_count` deliberately survive the watchdog's own
//     reset (only the external hardware reset clears them), so the
//     payload computer can read how many silent recoveries have happened
//     -- the error-reporting scheme the thesis recommends. A watchdog
//     that resets its own evidence hides the very problem it fixed.
//-----------------------------------------------------------------------------
module watchdog #(
    parameter WIDTH     = 32,
    parameter TIMEOUT   = 32'd100_000_000,   // clocks (1 s at 100 MHz)
    parameter RST_PULSE = 8                  // wdt_rst width in clocks
) (
    input  wire        clk,
    input  wire        rst,          // external hardware reset
    input  wire        enable,
    input  wire        kick,         // "I'm alive" pulse from the design

    output reg         wdt_rst,      // reset request to the rest of the design
    output reg         fired,        // sticky: at least one timeout occurred
    output reg [7:0]   fired_count
);

    reg [WIDTH-1:0] cnt;
    reg [7:0]       pulse_cnt;

    // edge detect: a stuck-high kick must not silence the watchdog
    reg  kick_d;
    wire kick_edge = kick && !kick_d;

    // parameter sanity (simulation-time only): TIMEOUT must be reachable
    // by a WIDTH-bit counter, or the watchdog would never fire.
    // 64-bit compare: (1 << WIDTH) in 32-bit math wraps to 0 at WIDTH=32
    // and the check would cry wolf on every legal configuration.
    initial begin
        if ({32'b0, TIMEOUT} > (64'd1 << WIDTH))
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
                // stretch the reset to RST_PULSE clocks so every clock
                // domain / deeply-piped consumer sees it
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
                // >= not ==: even if an SEU bumps cnt past the compare
                // value, the timeout still fires
                wdt_rst     <= 1'b1;
                pulse_cnt   <= 8'd0;
                fired       <= 1'b1;              // sticky until hardware reset
                fired_count <= fired_count + 1;
            end else
                cnt <= cnt + 1;
        end
    end

endmodule
