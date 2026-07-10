//-----------------------------------------------------------------------------
// psd_avg.v -- power spectral density accumulator (EDDI-FPGA QTN path)
//
// WHAT THIS IS
//   The last step of the QTN chain: turn complex FFT bins into an
//   AVERAGED power spectrum. A single FFT of a noisy plasma signal is
//   itself noisy; the QTN spectral peak (which sits at the local plasma /
//   upper-hybrid frequency and gives electron density) only emerges after
//   averaging several spectra. The flight firmware averaged 4 FFTs per
//   second -- this module does the same job in hardware.
//
// WHAT IT DOES
//   For each incoming bin:  p = re^2 + im^2   (power, magnitude-squared;
//   the phase of thermal noise carries no information, so it is dropped).
//   p is accumulated per-bin over NAVG frames. NAVG must be a power of
//   two so the final divide is a plain right-shift -- no divider needed.
//   After the last frame, `done` goes high and the mean PSD is read out
//   through rd_addr/rd_data.
//
// SIZING
//   p fits in 33 bits (two 31-bit squares summed), so the accumulator is
//   ACCW = 33 + LOG2AVG bits: it provably cannot overflow, whatever the
//   input. The mean after >>LOG2AVG is back to <= 33 bits and is
//   saturated into the 32-bit readout word (the top bit can only be set
//   by a full-scale input, but a silent wrap here would be undetectable
//   downstream, so it is clamped).
//
// FLIGHT PARITY
//   This covers the mean-PSD half of the LPC4370 QTN mode. The per-bin
//   standard deviation the flight firmware also kept is a v2 feature
//   (needs a second sum-of-squares RAM) -- see the README roadmap.
//-----------------------------------------------------------------------------
module psd_avg #(
    parameter LOG2N   = 10,
    parameter N       = 1 << LOG2N,
    parameter NBINS   = N/2 + 1,            // real input: bins 0..N/2
    parameter LOG2AVG = 2,                  // NAVG = 1 << LOG2AVG
    parameter ACCW    = 33 + LOG2AVG        // accumulator width (no overflow)
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               clear,        // restart accumulation

    // bin stream from fft_r2's unload port
    input  wire               in_valid,
    input  wire [LOG2N-1:0]   in_bin,
    input  wire signed [15:0] in_re,
    input  wire signed [15:0] in_im,
    input  wire               in_last,      // last bin of a frame

    output reg                done,         // NAVG frames accumulated

    // mean-PSD readout (use while done is high)
    input  wire [LOG2N-1:0]   rd_addr,
    output reg  [31:0]        rd_data       // mean PSD, valid 2 cycles after
                                            // rd_addr (RAM read + output reg)
);

    localparam NAVG = 1 << LOG2AVG;

    // Per-bin accumulator RAM. One entry per output bin; wide enough that
    // NAVG maximal powers cannot overflow (see SIZING above).
    reg [ACCW-1:0] acc [0:NBINS-1];

    reg [LOG2AVG:0] frame_idx;              // which frame of the average

    // --------------------------------------------------------------- stage 1
    // Compute power and issue the accumulator read. Two 16x16 squares are
    // each <= 2^30; their sum fits 33 bits unsigned. (The products are
    // declared signed because the inputs are, but a square is always
    // non-negative, so the {1'b0, ...} zero-extends safely.)
    wire signed [31:0] sq_re = in_re * in_re;
    wire signed [31:0] sq_im = in_im * in_im;
    wire [32:0] p = {1'b0, sq_re} + {1'b0, sq_im};

    // pipeline registers between stage 1 (read) and stage 2 (write-back)
    reg  [32:0]      p_q;
    reg  [LOG2N-1:0] bin_q;
    reg              v_q, last_q, first_q;
    reg  [ACCW-1:0]  acc_rd;

    // The accumulator RAM's single read port is shared: during
    // accumulation it reads the bin being updated; once done it serves
    // the external readout.
    wire [LOG2N-1:0] acc_addr = done ? rd_addr : in_bin;

    always @(posedge clk) begin
        acc_rd <= acc[acc_addr];
    end

    // Frame 0 seeds the accumulator instead of adding to it -- that is
    // what makes `clear` cheap: no need to spend NBINS cycles zeroing the
    // RAM, the first frame simply overwrites whatever was there.
    wire [ACCW-1:0] acc_new = first_q ? {{(ACCW-33){1'b0}}, p_q}
                                      : acc_rd + {{(ACCW-33){1'b0}}, p_q};
    // divide by NAVG = right shift (why NAVG must be a power of two)
    wire [ACCW-1:0] mean    = acc_rd >> LOG2AVG;

    always @(posedge clk) begin
        if (rst || clear) begin
            frame_idx <= 0;
            done      <= 1'b0;
            v_q       <= 1'b0;
            last_q    <= 1'b0;
        end else begin
            // stage 1 -> stage 2 handoff. Gating v_q on !done freezes the
            // accumulator once the average is complete, even if the FFT
            // keeps streaming.
            v_q     <= in_valid && !done;
            p_q     <= p;
            bin_q   <= in_bin;
            last_q  <= in_last;
            first_q <= (frame_idx == 0);

            // ----------------------------------------------------- stage 2
            // Read-modify-write of acc[bin]. Safe without forwarding
            // because bins arrive strictly in order 0,1,2,... -- the same
            // bin is never touched twice within the pipeline depth.
            if (v_q)
                acc[bin_q] <= acc_new;

            // count frames on each frame's last bin; NAVG-th frame => done
            if (v_q && last_q) begin
                if (frame_idx == NAVG-1)
                    done <= 1'b1;
                frame_idx <= frame_idx + 1;
            end

            // registered mean readout (meaningful while done is high),
            // saturated to the 32-bit telemetry word
            rd_data <= (|mean[ACCW-1:32]) ? 32'hFFFF_FFFF : mean[31:0];
        end
    end

endmodule
