// ============================================================================
// miso_phase_select.sv -- pick the 16 data bits out of a 4x oversampled MISO.
// ----------------------------------------------------------------------------
// This is Intan's own mechanism, from MISO_phase_selector.v in the Rhythm
// reference design, rewritten as a loop instead of twelve hand-written cases.
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
//   The window runs 74 taps, ten past the last SCLK bit, because the RHD2164
//   puts module B's least significant bit out on the CS rising edge. Sampling
//   past the end of the burst is what catches it; drop those taps and every
//   module-B channel is silently wrong in its LSB.
// ============================================================================

`default_nettype none

module miso_phase_select #(
    parameter int BITS  = 16,
    parameter int OS    = 4,                 // oversample factor
    parameter int TAPS  = 74                 // BITS*OS + OS*2 + slack
) (
    input  wire [TAPS-1:0]   in4x,
    input  wire [3:0]        phase,          // 0..11, quarter-SCLK steps
    output wire [BITS-1:0]   word_sdr,       // module A: rising-edge half
    output wire [BITS-1:0]   word_ddr        // module B: falling-edge half
);

    // Saturate rather than wrap: an out-of-range phase would index past the end
    // of the window and read X, which in simulation looks like a data bug and in
    // hardware silently reads the wrong tap. ctrl_regs also clamps, so this is
    // belt and braces on a value that arrives from a UART.
    wire [3:0] p = (phase > 4'd11) ? 4'd11 : phase;

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
