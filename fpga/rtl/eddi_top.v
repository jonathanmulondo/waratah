//-----------------------------------------------------------------------------
// eddi_top.v -- EDDI-FPGA instrument top level
//
// WHAT THIS IS
//   The whole instrument, wired: both science paths running CONCURRENTLY
//   off one shared ADC sample bus -- the capability the LPC4370 flight
//   design could not provide (one ADC, one mode at a time) and the
//   reason for the FPGA port.
//
//                          +-------------+     +-----------+
//              +---------->| burst_ctrl  |---->| qtn_core  |--- mean PSD
//              |  (gated,  | cadence +   |     | window/FFT|      |
//   ADC bus ---+  bursts)  | amp power   |     | /PSD avg  |      |
//   (40 Msps)  |           +-------------+     +-----------+      |
//              |                                                  v
//              |           +-------------+     +-----------+  +----------+
//              +---------->| cic_decim   |---->| debris_   |->| payload_ |<-> UART
//                (always   | 800:1       |     | detector  |  | if       |  to OBC
//                 on)      | -> 50 ksps  |     | 4-sigma   |  +----------+
//                          +-------------+     +-----------+       |
//                                                             heartbeat
//                          +-------------+                         |
//                          | watchdog    |<------ kick ------------+
//                          +-------------+   (+ spectrum completions)
//
// RESET STRATEGY
//   The watchdog observes heartbeats from the payload computer and
//   spectrum completions from the QTN path. If both go silent for
//   WDT_TIMEOUT clocks, wdt_rst resets EVERYTHING except the watchdog
//   itself; its fired count survives, readable via GET_STATUS -- so the
//   ground can tell how many silent recoveries have happened.
//
// STATUS BLOCK (GET_STATUS response, 12 bytes, big-endian)
//   byte 0     flags: {3'b0, wdt_fired, collecting, event_ready,
//                      spectrum_ready, 1'b1 (alive)}
//   bytes 1-2  debris event_count
//   byte 3     debris events_dropped
//   byte 4     watchdog fired_count
//   bytes 5-6  debris cal_mean (signed)
//   bytes 7-10 debris cal_var
//   byte 11    cfg echo ({gain[3:2], debris_en, qtn_en})
//
// The ADC interface module (LVDS deserialiser, two's-complement
// selection, clock-domain crossing) is the piece that will drive
// adc_valid/adc_sample -- see the README roadmap for its constraints.
//-----------------------------------------------------------------------------
module eddi_top #(
    parameter LOG2N        = 10,
    parameter LOG2AVG      = 2,
    parameter HANN_FILE    = "mem/hann_1024.mem",
    parameter TW_RE_FILE   = "mem/tw_re_1024.mem",
    parameter TW_IM_FILE   = "mem/tw_im_1024.mem",
    parameter CIC_R        = 800,               // 40 Msps -> 50 ksps
    parameter CIC_W        = 48,
    parameter CIC_SHIFT    = 29,
    parameter CADENCE_CLKS = 32'd100_000_000,   // gap between QTN bursts
    parameter SETTLE_CLKS  = 32'd1_000,         // op-amp settle
    parameter CLKS_PER_BIT = 868,               // 100 MHz / 115200
    parameter WDT_TIMEOUT  = 32'd300_000_000    // 3 s: > one burst period
) (
    input  wire               clk,
    input  wire               rst_ext,          // external hardware reset

    // shared ADC sample bus (from the future ADC interface module)
    input  wire               adc_valid,
    input  wire signed [15:0] adc_sample,

    // payload-computer UART
    input  wire               rxd,
    output wire               txd,

    // GPIO
    output wire               qtn_amp_en,       // QTN op-amp power gate
    output wire [1:0]         gain_sel,         // amplifier gain-step pins

    // observability (also in the status block)
    output wire               wdt_rst_out,
    output wire               spectrum_ready,
    output wire               event_ready
);

    // ------------------------------------------------------------- reset tree
    // everything runs on rst_i; only the watchdog keeps rst_ext, so its
    // own firing cannot erase the evidence that it fired
    wire wdt_rst_w;
    wire rst_i = rst_ext | wdt_rst_w;
    assign wdt_rst_out = wdt_rst_w;

    // ------------------------------------------------------------- config
    wire [7:0] cfg;                         // from payload_if (SET_CONFIG)
    assign gain_sel = cfg[3:2];

    // ------------------------------------------------------------- QTN path
    wire               qtn_valid;
    wire signed [15:0] qtn_sample;
    wire               restart_w;
    wire [15:0]        spectra_count;
    wire               collecting;

    burst_ctrl #(
        .CADENCE_CLKS(CADENCE_CLKS),
        .SETTLE_CLKS (SETTLE_CLKS)
    ) u_burst (
        .clk(clk), .rst(rst_i),
        .enable        (cfg[0]),
        .in_valid      (adc_valid),
        .in_sample     (adc_sample),
        .qtn_valid     (qtn_valid),
        .qtn_sample    (qtn_sample),
        .spectrum_ready(spectrum_ready),
        .restart       (restart_w),
        .amp_en        (qtn_amp_en),
        .spectra_count (spectra_count),
        .collecting    (collecting)
    );

    wire [LOG2N-1:0] psd_addr_w;
    wire [31:0]      psd_data_w;

    qtn_core #(
        .LOG2N(LOG2N), .LOG2AVG(LOG2AVG),
        .HANN_FILE(HANN_FILE),
        .TW_RE_FILE(TW_RE_FILE), .TW_IM_FILE(TW_IM_FILE)
    ) u_qtn (
        .clk(clk), .rst(rst_i),
        .restart       (restart_w),
        .sample_valid  (qtn_valid),
        .sample_in     (qtn_sample),
        .sample_ready  (),
        .spectrum_ready(spectrum_ready),
        .psd_rd_addr   (psd_addr_w),
        .psd_rd_data   (psd_data_w),
        .busy          ()
    );

    // ------------------------------------------------------------- debris path
    // always on (subject to cfg[1]) -- debris impacts don't wait for a
    // burst window; this concurrency is the instrument's headline feature
    wire               dec_valid;
    wire signed [15:0] dec_sample;

    cic_decim #(
        .R(CIC_R), .W(CIC_W), .SHIFT(CIC_SHIFT)
    ) u_cic (
        .clk(clk), .rst(rst_i),
        .in_valid  (adc_valid && cfg[1]),
        .in_sample (adc_sample),
        .out_valid (dec_valid),
        .out_sample(dec_sample)
    );

    wire               ack_event_w;
    wire [15:0]        event_count;
    wire [7:0]         events_dropped;
    wire [10:0]        buf_addr_w;
    wire signed [15:0] buf_data_w;
    wire signed [15:0] cal_mean;
    wire [31:0]        cal_var;

    debris_detector u_debris (
        .clk(clk), .rst(rst_i),
        .in_valid      (dec_valid),
        .in_sample     (dec_sample),
        .ack           (ack_event_w),
        .event_ready   (event_ready),
        .event_count   (event_count),
        .events_dropped(events_dropped),
        .trig_index    (),
        .buf_rd_addr   (buf_addr_w),
        .buf_rd_data   (buf_data_w),
        .cal_mean      (cal_mean),
        .cal_var       (cal_var),
        .cal_zc        ()
    );

    // ------------------------------------------------------------- watchdog
    wire hb_w;
    wire wdt_fired;
    wire [7:0] fired_count;

    // kick on proof of life from either direction: an intact frame from
    // the payload computer, or a completed spectrum from the QTN path
    reg spec_d;
    always @(posedge clk) begin
        if (rst_i) spec_d <= 1'b0;
        else       spec_d <= spectrum_ready;
    end
    wire kick_w = hb_w | (spectrum_ready & ~spec_d);

    watchdog #(
        .TIMEOUT(WDT_TIMEOUT)
    ) u_wdt (
        .clk(clk), .rst(rst_ext),
        .enable     (1'b1),
        .kick       (kick_w),
        .wdt_rst    (wdt_rst_w),
        .fired      (wdt_fired),
        .fired_count(fired_count)
    );

    // ------------------------------------------------------------- payload link
    wire [7:0] flags = {3'b000, wdt_fired, collecting,
                        event_ready, spectrum_ready, 1'b1};
    wire [95:0] status = {flags, event_count, events_dropped,
                          fired_count, cal_mean, cal_var, cfg};

    payload_if #(
        .CLKS_PER_BIT(CLKS_PER_BIT),
        .PSD_AW(LOG2N), .BUF_AW(11)
    ) u_pl (
        .clk(clk), .rst(rst_i),
        .rxd(rxd), .txd(txd),
        .status   (status),
        .psd_addr (psd_addr_w),
        .psd_data (psd_data_w),
        .buf_addr (buf_addr_w),
        .buf_data (buf_data_w),
        .ack_event(ack_event_w),
        .cfg      (cfg),
        .heartbeat(hb_w)
    );

endmodule
