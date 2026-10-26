// ============================================================================
// miso_phase_select.sv -- pick the 16 data bits out of a 4x oversampled MISO.
// ----------------------------------------------------------------------------
// This is Intan's own mechanism, from MISO_phase_selector.v in the Rhythm
// reference design, rewritten as a loop instead of hand-written cases.
//
// Why it works this way rather than with IDDR + IODELAY:
//
//   The master generates SCLK, so instead of trying to sample MISO at exactly
//   the right instant, it records MISO on EVERY fast clock -- four samples per
//   SCLK bit -- and then picks which of the four to keep. `phase` shifts that
//   choice in quarter-bit steps, which is how cable delay is compensated. It is
//   a runtime register, so the host can sweep it and pick the setting with zero
//   bit errors: an eye scan, for free.
//
//   The second output is the same trick offset by half an SCLK period, which is
//   the RHD2164's falling-edge half (module B, channels 32..63). So one shift
//   register and two muxes de-interleave the double-data-rate stream; there is
//   no separate de-interleaver, and no vendor primitive, which is also why this
//   simulates in plain Icarus Verilog.
//
//   The window runs 78 taps, fourteen past the last SCLK bit, because the
//   RHD2164 puts module B's least significant bit out on the CS rising edge.
//   Sampling past the end of the burst is what catches it; drop those taps and
//   every module-B channel is silently wrong in its LSB. 78 is exactly what the
//   largest phase needs: module B's LSB at phase 15 is tap 15 + 2 + 4*15 = 77,
//   so all sixteen values of the 4-bit phase register read a whole word.
// ============================================================================

`default_nettype none

module miso_phase_select #(
    parameter int BITS  = 16,
    parameter int OS    = 4,                 // oversample factor
    parameter int TAPS  = 78                 // 15 + OS/2 + OS*(BITS-1) + 1
) (
    input  wire [TAPS-1:0]   in4x,
    input  wire [3:0]        phase,          // 0..15, quarter-SCLK steps
    output wire [BITS-1:0]   word_sdr,       // module A: rising-edge half
    output wire [BITS-1:0]   word_ddr        // module B: falling-edge half
);

    // The largest phase whose module-B LSB is still inside the window: 15 at
    // the standard 78 taps, so with a 4-bit phase the clamp below is free.
    localparam int PMAX = TAPS - 1 - OS/2 - OS*(BITS-1);

    // Saturate rather than wrap: an out-of-range phase would index past the end
    // of the window and read X, which in simulation looks like a data bug and in
    // hardware silently reads the wrong tap. ctrl_regs also clamps, so this is
    // belt and braces on a value that arrives from a UART.
    wire [3:0] p = (PMAX >= 15 || phase <= 4'(PMAX)) ? phase : 4'(PMAX);

    genvar k;
    generate
        for (k = 0; k < BITS; k = k + 1) begin : g_taps
            // Bit BITS-1-k is the k'th bit on the wire: the first bit out of the
            // chip is the most significant, as the datasheet specifies.
            assign word_sdr[BITS-1-k] = in4x[p + OS*k];
            assign word_ddr[BITS-1-k] = in4x[p + (OS/2) + OS*k];
        end
    endgenerate

endmodule

`default_nettype wire
