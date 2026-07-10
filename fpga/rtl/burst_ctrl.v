//-----------------------------------------------------------------------------
// burst_ctrl.v -- QTN burst / cadence controller (EDDI-FPGA)
//
// WHY IT EXISTS
//   The flight firmware did not sample the QTN path continuously: it
//   woke the (power-hungry) amplifier chain, grabbed a short burst,
//   produced one averaged spectrum, and shut the amps down again --
//   1.35 W peak but only ~0.58 W average. This module reproduces that
//   duty-cycled behaviour in hardware:
//
//     S_WAIT ---(cadence timer)---> S_SETTLE ---(amps stable)---> S_RUN
//        ^                            amp_en=1                 gate open
//        |                                                         |
//        +--------------(spectrum_ready from qtn_core)------------+
//                          amp_en=0, spectra_count++
//
// HOW IT CONNECTS
//   Sits between the ADC sample stream and qtn_core. In S_RUN the
//   stream is passed through (qtn_valid); in every other state the
//   gate is closed and the amplifiers are off. On leaving S_WAIT it
//   pulses `restart`, which qtn_core honours only while a finished
//   spectrum is pending (restart_eff) -- so the previous average is
//   cleared exactly when a new collection begins, never mid-stream.
//
//   Samples that arrive while qtn_core is mid-FFT (sample_ready low)
//   are simply not accepted -- frames need not be contiguous in time,
//   which matches the flight firmware's "4 random chunks per burst"
//   behaviour. The payload computer should read the spectrum during
//   S_WAIT; an unread spectrum is overwritten by the next collection.
//
//   CADENCE_CLKS sets the gap between collections (the thesis asks for
//   a CONFIGURABLE duty cycle to fit ~5 kB/pass UHF downlink budgets --
//   making this register-writable via payload_if is the v0.3 hook).
//-----------------------------------------------------------------------------
module burst_ctrl #(
    parameter CADENCE_CLKS = 32'd100_000_000,  // idle gap between collections
    parameter SETTLE_CLKS  = 32'd1_000         // op-amp power-up settle time
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               enable,          // 0: hold in S_WAIT, amps off

    // raw sample stream (from ADC interface)
    input  wire               in_valid,
    input  wire signed [15:0] in_sample,

    // gated stream to qtn_core
    output wire               qtn_valid,
    output wire signed [15:0] qtn_sample,

    // qtn_core handshake
    input  wire               spectrum_ready,
    output reg                restart,         // 1-clk pulse at collection start

    // GPIO: op-amp chain power gate (flight: amps only on during bursts)
    output reg                amp_en,

    // housekeeping
    output reg  [15:0]        spectra_count,
    output wire               collecting       // high while S_SETTLE/S_RUN
);

    localparam S_WAIT   = 2'd0;
    localparam S_SETTLE = 2'd1;
    localparam S_RUN    = 2'd2;

    reg [1:0]  state;
    reg [31:0] timer;

    // gate: samples reach qtn_core only during the collection window
    assign qtn_valid  = in_valid && (state == S_RUN);
    assign qtn_sample = in_sample;
    assign collecting = (state != S_WAIT);

    always @(posedge clk) begin
        if (rst) begin
            state         <= S_WAIT;
            timer         <= 0;
            restart       <= 1'b0;
            amp_en        <= 1'b0;
            spectra_count <= 0;
        end else begin
            restart <= 1'b0;

            case (state)
            S_WAIT: begin
                amp_en <= 1'b0;
                if (!enable)
                    timer <= 0;
                else if (timer >= CADENCE_CLKS-1) begin
                    // start a collection: power the amps, clear the old
                    // average (restart is ignored by qtn_core unless a
                    // spectrum is actually pending)
                    timer   <= 0;
                    restart <= 1'b1;
                    amp_en  <= 1'b1;
                    state   <= S_SETTLE;
                end else
                    timer <= timer + 1;
            end
            S_SETTLE: begin
                // amps powered but not yet linear: keep the gate shut
                if (timer >= SETTLE_CLKS-1) begin
                    timer <= 0;
                    state <= S_RUN;
                end else
                    timer <= timer + 1;
            end
            S_RUN: begin
                // gate open; qtn_core accepts frames at its own pace.
                // spectrum_ready marks the averaged PSD complete: shut
                // the amps down and go idle until the next cadence tick.
                if (spectrum_ready) begin
                    amp_en        <= 1'b0;
                    spectra_count <= spectra_count + 1;
                    timer         <= 0;
                    state         <= S_WAIT;
                end
            end
            default: state <= S_WAIT;
            endcase
        end
    end

endmodule
