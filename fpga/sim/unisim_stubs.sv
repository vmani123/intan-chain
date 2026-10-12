// ============================================================================
// unisim_stubs.sv -- just enough of IBUF, BUFG and MMCME2_BASE for Icarus.
// Simulation only; never read by Vivado, which has the real primitives.
// ----------------------------------------------------------------------------
// The board tops are the only files that instantiate vendor primitives. These
// stand-ins let a testbench run a whole board top -- pads, clocking, resets and
// all -- instead of trusting that the wrapper is wired the way its comments
// say.
//
// The MMCM does not track its input: it derives each output period from its
// parameters, exactly as the silicon would after lock, and asserts LOCKED a
// fixed time after RST falls. That checks the multiply/divide arithmetic and
// the reset sequencing. Jitter, phase and lock behaviour are not modelled.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module IBUF (input wire I, output wire O);
    assign O = I;
endmodule

module BUFG (input wire I, output wire O);
    assign O = I;
endmodule

module MMCME2_BASE #(
    parameter real    CLKIN1_PERIOD    = 10.0,
    parameter real    CLKFBOUT_MULT_F  = 5.0,
    parameter integer DIVCLK_DIVIDE    = 1,
    parameter real    CLKOUT0_DIVIDE_F = 1.0,
    parameter integer CLKOUT1_DIVIDE   = 1,
    parameter integer CLKOUT2_DIVIDE   = 1,
    parameter integer CLKOUT3_DIVIDE   = 1,
    parameter integer CLKOUT4_DIVIDE   = 1,
    parameter integer CLKOUT5_DIVIDE   = 1,
    parameter integer CLKOUT6_DIVIDE   = 1,
    parameter real    LOCK_NS          = 1000.0
) (
    input  wire CLKIN1, CLKFBIN, RST, PWRDWN,
    output reg  CLKOUT0, CLKOUT1, CLKOUT2, CLKOUT3, CLKOUT4, CLKOUT5, CLKOUT6,
    output wire CLKOUT0B, CLKOUT1B, CLKOUT2B, CLKOUT3B,
    output wire CLKFBOUT, CLKFBOUTB,
    output reg  LOCKED
);
    localparam real VCO_NS = CLKIN1_PERIOD * DIVCLK_DIVIDE / CLKFBOUT_MULT_F;

    localparam real H0 = VCO_NS * CLKOUT0_DIVIDE_F / 2.0;
    localparam real H1 = VCO_NS * CLKOUT1_DIVIDE   / 2.0;
    localparam real H2 = VCO_NS * CLKOUT2_DIVIDE   / 2.0;
    localparam real H3 = VCO_NS * CLKOUT3_DIVIDE   / 2.0;
    localparam real H4 = VCO_NS * CLKOUT4_DIVIDE   / 2.0;
    localparam real H5 = VCO_NS * CLKOUT5_DIVIDE   / 2.0;
    localparam real H6 = VCO_NS * CLKOUT6_DIVIDE   / 2.0;

    assign CLKOUT0B = ~CLKOUT0;
    assign CLKOUT1B = ~CLKOUT1;
    assign CLKOUT2B = ~CLKOUT2;
    assign CLKOUT3B = ~CLKOUT3;
    assign CLKFBOUT  = 1'b0;
    assign CLKFBOUTB = 1'b1;

    wire run = !RST && !PWRDWN;

    initial begin
        {CLKOUT0, CLKOUT1, CLKOUT2, CLKOUT3, CLKOUT4, CLKOUT5, CLKOUT6} = 7'd0;
        LOCKED = 1'b0;
    end

    always begin #(H0); CLKOUT0 = run ? ~CLKOUT0 : 1'b0; end
    always begin #(H1); CLKOUT1 = run ? ~CLKOUT1 : 1'b0; end
    always begin #(H2); CLKOUT2 = run ? ~CLKOUT2 : 1'b0; end
    always begin #(H3); CLKOUT3 = run ? ~CLKOUT3 : 1'b0; end
    always begin #(H4); CLKOUT4 = run ? ~CLKOUT4 : 1'b0; end
    always begin #(H5); CLKOUT5 = run ? ~CLKOUT5 : 1'b0; end
    always begin #(H6); CLKOUT6 = run ? ~CLKOUT6 : 1'b0; end

    always @(RST or PWRDWN) begin
        LOCKED = 1'b0;
        if (run) begin
            #(LOCK_NS);
            if (run) LOCKED = 1'b1;
        end
    end
endmodule

`default_nettype wire
