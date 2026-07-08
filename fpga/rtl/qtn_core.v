//-----------------------------------------------------------------------------
// qtn_core.v -- Ionosphere (QTN spectroscopy) module, EDDI-FPGA
//
// Sample stream in  ->  Hann window  ->  radix-2 FFT  ->  averaged PSD.
//
// Accepts NAVG frames of N real samples (ready/valid handshake, e.g. from
// the ADC interface burst buffer), applies a Q1.15 Hann window, runs the
// FFT and accumulates |X|^2 over the frames. When done, `spectrum_ready`
// is asserted and the mean PSD (bins 0..N/2, 32-bit) can be read out.
// Assert `restart` to begin a new averaging period.
//
// This is the FPGA replacement for the LPC4370 flight firmware's FFT
// modes (512/1024-pt, one averaged PSD per second).
//-----------------------------------------------------------------------------
module qtn_core #(
    parameter LOG2N     = 10,
    parameter N         = 1 << LOG2N,
    parameter LOG2AVG   = 2,               // frames averaged = 1 << LOG2AVG
    parameter HANN_FILE  = "mem/hann_1024.mem",
    parameter TW_RE_FILE = "mem/tw_re_1024.mem",
    parameter TW_IM_FILE = "mem/tw_im_1024.mem"
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               restart,       // clear PSD, start new average

    input  wire               sample_valid,
    input  wire signed [15:0] sample_in,
    output wire               sample_ready,

    output wire               spectrum_ready,
    input  wire [LOG2N-1:0]   psd_rd_addr,
    output wire [31:0]        psd_rd_data,   // valid 2 cycles after addr

    output wire               busy
);

    localparam NAVG = 1 << LOG2AVG;

    // ------------------------------------------------------------- Hann ROM
    reg signed [15:0] hann_rom [0:N-1];
    initial $readmemh(HANN_FILE, hann_rom);

    // ------------------------------------------------------------- frame control
    reg [LOG2N-1:0]   n;            // sample index within frame
    reg               frame_wait;   // frame fully accepted, FFT running
    reg [LOG2AVG:0]   fcnt;         // frames accepted this averaging run
    reg               all_loaded;   // NAVG-th frame accepted: no more samples
                                    // (spectrum_ready lags by the PSD pipeline,
                                    //  so gating on it alone leaks one sample)

    wire fft_ready;
    wire fft_out_last;

    // restart is only honoured when idle -- clearing the accumulator while
    // FFT bins are still streaming would mix old and new averages
    wire restart_eff = restart && spectrum_ready;

    assign sample_ready = fft_ready && !frame_wait && !all_loaded
                          && !spectrum_ready;
    wire   accept       = sample_valid && sample_ready;

    // ------------------------------------------------------------- window pipeline
    // stage 1: register sample, issue ROM read
    reg signed [15:0] samp_d;
    reg               v1;
    reg signed [15:0] coeff_q;
    always @(posedge clk) begin
        coeff_q <= hann_rom[n];
        samp_d  <= sample_in;
    end
    // stage 2: multiply, Q1.15 scale
    wire signed [31:0] prod = samp_d * coeff_q;
    reg signed [15:0]  win_q;
    reg                v2;

    always @(posedge clk) begin
        if (rst) begin
            n          <= 0;
            frame_wait <= 1'b0;
            fcnt       <= 0;
            all_loaded <= 1'b0;
            v1         <= 1'b0;
            v2         <= 1'b0;
        end else begin
            v1 <= accept;
            v2 <= v1;
            win_q <= prod >>> 15;

            if (accept) begin
                if (n == N-1) begin
                    n          <= 0;
                    frame_wait <= 1'b1;    // hold input until FFT frame done
                    fcnt       <= fcnt + 1;
                    if (fcnt == NAVG-1)
                        all_loaded <= 1'b1;
                end else
                    n <= n + 1;
            end

            if (fft_out_last)
                frame_wait <= 1'b0;

            if (restart_eff) begin
                fcnt       <= 0;
                all_loaded <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------- FFT
    wire               fft_out_valid;
    wire [LOG2N-1:0]   fft_out_bin;
    wire signed [15:0] fft_out_re, fft_out_im;
    wire               fft_busy;

    fft_r2 #(
        .LOG2N(LOG2N),
        .TW_RE_FILE(TW_RE_FILE),
        .TW_IM_FILE(TW_IM_FILE)
    ) u_fft (
        .clk      (clk),
        .rst      (rst),
        .in_valid (v2),
        .in_re    (win_q),
        .in_ready (fft_ready),
        .out_valid(fft_out_valid),
        .out_bin  (fft_out_bin),
        .out_re   (fft_out_re),
        .out_im   (fft_out_im),
        .out_last (fft_out_last),
        .busy     (fft_busy)
    );

    // ------------------------------------------------------------- PSD averaging
    psd_avg #(
        .LOG2N  (LOG2N),
        .LOG2AVG(LOG2AVG)
    ) u_psd (
        .clk     (clk),
        .rst     (rst),
        .clear   (restart_eff),
        .in_valid(fft_out_valid),
        .in_bin  (fft_out_bin),
        .in_re   (fft_out_re),
        .in_im   (fft_out_im),
        .in_last (fft_out_last),
        .done    (spectrum_ready),
        .rd_addr (psd_rd_addr),
        .rd_data (psd_rd_data)
    );

    assign busy = fft_busy || frame_wait;

endmodule
