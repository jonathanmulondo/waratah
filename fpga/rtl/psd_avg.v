//-----------------------------------------------------------------------------
// psd_avg.v -- power spectral density accumulator (EDDI-FPGA QTN path)
//
// Consumes the FFT unload stream (bins 0..N/2), computes p = re^2 + im^2
// and accumulates it over NAVG frames (NAVG must be a power of two).
// After the last frame, `done` is asserted and the mean PSD can be read
// out through the rd port (1-cycle registered read).
//
// Flight-firmware equivalent: the averaged PSD half of the QTN mode.
// The per-bin standard deviation of the flight design is a v2 feature
// (needs a second sum-of-squares RAM).
//-----------------------------------------------------------------------------
module psd_avg #(
    parameter LOG2N   = 10,
    parameter N       = 1 << LOG2N,
    parameter NBINS   = N/2 + 1,
    parameter LOG2AVG = 2,                  // NAVG = 1 << LOG2AVG
    parameter ACCW    = 33 + LOG2AVG        // accumulator width
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               clear,        // restart accumulation

    input  wire               in_valid,
    input  wire [LOG2N-1:0]   in_bin,
    input  wire signed [15:0] in_re,
    input  wire signed [15:0] in_im,
    input  wire               in_last,      // last bin of a frame

    output reg                done,         // NAVG frames accumulated

    input  wire [LOG2N-1:0]   rd_addr,
    output reg  [31:0]        rd_data       // mean PSD, valid 2 cycles after
                                            // rd_addr (RAM read + output reg)
);

    localparam NAVG = 1 << LOG2AVG;

    reg [ACCW-1:0] acc [0:NBINS-1];

    reg [LOG2AVG:0] frame_idx;

    // stage 1: compute power, look up current accumulator value
    wire signed [31:0] sq_re = in_re * in_re;
    wire signed [31:0] sq_im = in_im * in_im;
    wire [32:0] p = {1'b0, sq_re} + {1'b0, sq_im};

    reg  [32:0]      p_q;
    reg  [LOG2N-1:0] bin_q;
    reg              v_q, last_q, first_q;
    reg  [ACCW-1:0]  acc_rd;

    wire [LOG2N-1:0] acc_addr = done ? rd_addr : in_bin;

    always @(posedge clk) begin
        acc_rd <= acc[acc_addr];
    end

    wire [ACCW-1:0] acc_new = first_q ? {{(ACCW-33){1'b0}}, p_q}
                                      : acc_rd + {{(ACCW-33){1'b0}}, p_q};
    wire [ACCW-1:0] mean    = acc_rd >> LOG2AVG;

    always @(posedge clk) begin
        if (rst || clear) begin
            frame_idx <= 0;
            done      <= 1'b0;
            v_q       <= 1'b0;
            last_q    <= 1'b0;
        end else begin
            // stage 1 -> stage 2 pipeline
            v_q     <= in_valid && !done;
            p_q     <= p;
            bin_q   <= in_bin;
            last_q  <= in_last;
            first_q <= (frame_idx == 0);

            // stage 2: read-modify-write (bins arrive in order, no hazard)
            if (v_q)
                acc[bin_q] <= acc_new;

            if (v_q && last_q) begin
                if (frame_idx == NAVG-1)
                    done <= 1'b1;
                frame_idx <= frame_idx + 1;
            end

            // registered mean readout (valid when done), saturated to 32 bits
            rd_data <= (|mean[ACCW-1:32]) ? 32'hFFFF_FFFF : mean[31:0];
        end
    end

endmodule
