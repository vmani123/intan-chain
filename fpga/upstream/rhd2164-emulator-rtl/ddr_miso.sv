// ============================================================================
// ddr_miso.sv
// ----------------------------------------------------------------------------
// RHD2164 emulator — double-data-rate MISO bit generator (one per chip).
//
// Merges the module-A and module-B 16-bit results onto a single serial MISO
// line using the RHD2164 DDR scheme (datasheet p.10). Generated in the fast
// oversampling-clock domain; the differential output buffer (and optional
// ODDR/IOB placement) lives in the synthesis top.
//
// Bit schedule within one CS cycle (16 SCLK pulses; R = rising, F = falling):
//   * CS falling           : load A = result_a, B = result_b
//   * R_k (k=1..16)         : drive MISO = A[16-k]  (A[15] on R1 ... A[0] on R16)
//                             -> master samples it at the following F_k
//   * F_k (k=1..16)         : drive MISO = B[16-k]  (B[15] on F1 ... B[0] on F16)
//                             -> master samples B[15..1] at R2..R16 and B[0] at CS rising
//   * The master IGNORES the level sampled at the first rising edge R1
//     (it is leftover data from before the load).
//
// Net effect for the master:
//   A[15:0] sampled on the 16 SCLK falling edges,
//   B[15:1] sampled on SCLK rising edges 2..16,
//   B[0]    sampled on the CS rising edge.
//
// MISO is always driven (the 128-channel RHD2164 topology gives each chip its
// own MISO pair, so no bus sharing / weak-MISO HiZ is required — see SPEC §5).
// ============================================================================

`default_nettype none

module ddr_miso (
    input  wire        clk,          // fast oversampling clock
    input  wire        rst_n,

    // Edge strobes from spi_frontend (1 clk pulse each, only while CS low)
    input  wire        cs_falling,
    input  wire        sclk_rising,
    input  wire        sclk_falling,

    // Pipelined results for the CURRENT CS cycle
    input  wire [15:0] result_a,
    input  wire [15:0] result_b,

    output reg         miso_out      // single-ended serial output bit
);

    reg [15:0] a_sr;   // module-A shift register (drained on rising edges)
    reg [15:0] b_sr;   // module-B shift register (drained on falling edges)

    always @(posedge clk) begin
        if (!rst_n) begin
            a_sr     <= 16'h0000;
            b_sr     <= 16'h0000;
            miso_out <= 1'b0;
        end else begin
            if (cs_falling) begin
                // Latch this cycle's results at the start of the transfer.
                a_sr <= result_a;
                b_sr <= result_b;
                // miso_out holds its previous value through the R1 "ignored"
                // sample; it is overwritten on R1 below for the F1 sample.
            end

            if (sclk_rising) begin
                // Present the next A bit (MSB first) for the upcoming falling edge.
                miso_out <= a_sr[15];
                a_sr     <= {a_sr[14:0], 1'b0};
            end else if (sclk_falling) begin
                // Present the next B bit (MSB first) for the upcoming rising edge.
                miso_out <= b_sr[15];
                b_sr     <= {b_sr[14:0], 1'b0};
            end
        end
    end

endmodule

`default_nettype wire
