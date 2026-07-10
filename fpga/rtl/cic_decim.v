//-----------------------------------------------------------------------------
// cic_decim.v -- 3-stage CIC decimator (EDDI-FPGA debris path)
//
// WHY IT EXISTS
//   The debris detector wants ~50 ksps (impact transients ring at
//   1-25 kHz), but the shared ADC runs at 40 Msps for the QTN path.
//   If the final PCB uses one ADC for both paths, something must turn
//   40 Msps into 50 ksps: an 800:1 decimation. A CIC (cascaded
//   integrator-comb, Hogenauer) filter is the standard hardware answer:
//   it needs NO multipliers -- only adders and delay registers -- and
//   its passband droop is negligible in the 1-10 kHz band where debris
//   impact energy lives.
//
// HOW A CIC WORKS
//   Three integrators run at the INPUT rate, three combs (differencers)
//   at the OUTPUT rate, with the R:1 decimation between them:
//
//     in -->[+]-->[+]-->[+]--( keep 1 in R )-->[-z]-->[-z]-->[-z]--> out
//          integrators (40 Msps)              combs (50 ksps)
//
//   The pair (integrator, comb) is exactly a moving-average of R
//   samples; three in cascade give a sinc^3 low-pass response with
//   nulls at every multiple of the output rate -- precisely where
//   aliases would fold into the band.
//
//   The integrators wrap on overflow BY DESIGN: two's-complement
//   modular arithmetic guarantees the comb differences recover the
//   correct value as long as the register width W covers the total
//   gain, W >= 16 + ceil(3*log2(R)). For R=800 that is 16+29 = 45 bits
//   (48 used, with margin).
//
// SCALING
//   DC gain is R^3. The output is >> SHIFT with 2^SHIFT >= R^3 so
//   full-scale cannot overflow; for R=800, SHIFT=29 gives an overall
//   gain of 800^3 / 2^29 = 0.954. Passband droop (sinc^3): -0.4 dB at
//   5 kHz, -7.3 dB at 20 kHz -- fine for detection (the trigger
//   calibrates against the same filtered background); a compensation
//   FIR is a v0.3 option if flat spectra of events are ever needed.
//-----------------------------------------------------------------------------
module cic_decim #(
    parameter R     = 800,          // decimation ratio (any integer >= 2)
    parameter W     = 48,           // internal width >= 16 + ceil(3*log2(R))
    parameter SHIFT = 29            // output scale; need 2^SHIFT >= R^3
) (
    input  wire               clk,
    input  wire               rst,

    input  wire               in_valid,     // one pulse per fast sample
    input  wire signed [15:0] in_sample,

    output reg                out_valid,    // one pulse per R inputs
    output reg  signed [15:0] out_sample
);

    // integrator section (input rate). Wrap-around is intentional --
    // see header. i1/i2/i3 never "settle"; the combs difference them.
    reg signed [W-1:0] i1, i2, i3;

    // comb section state (output rate)
    reg signed [W-1:0] d1, d2, d3;

    reg [$clog2(R)-1:0] cnt;

    // comb differences, combinational: computed once per output sample
    // from the integrator snapshot and the previous snapshots
    wire signed [W-1:0] c1 = i3 - d1;
    wire signed [W-1:0] c2 = c1 - d2;
    wire signed [W-1:0] c3 = c2 - d3;

    wire signed [W-1:0] scaled = c3 >>> SHIFT;

    always @(posedge clk) begin
        if (rst) begin
            i1 <= 0;  i2 <= 0;  i3 <= 0;
            d1 <= 0;  d2 <= 0;  d3 <= 0;
            cnt        <= 0;
            out_valid  <= 1'b0;
            out_sample <= 16'sd0;
        end else begin
            out_valid <= 1'b0;
            if (in_valid) begin
                // integrators cascade every input sample
                i1 <= i1 + in_sample;
                i2 <= i2 + i1;
                i3 <= i3 + i2;

                if (cnt == R-1) begin
                    cnt <= 0;
                    // output-rate tick: run the combs on the current
                    // integrator value and advance their delay line
                    d1 <= i3;
                    d2 <= c1;
                    d3 <= c2;
                    // scale and saturate (cannot overflow if SHIFT
                    // covers R^3, but clamp against parameter mistakes)
                    if (scaled > 48'sd32767)
                        out_sample <= 16'sd32767;
                    else if (scaled < -48'sd32768)
                        out_sample <= -16'sd32768;
                    else
                        out_sample <= scaled[15:0];
                    out_valid <= 1'b1;
                end else
                    cnt <= cnt + 1;
            end
        end
    end

endmodule
