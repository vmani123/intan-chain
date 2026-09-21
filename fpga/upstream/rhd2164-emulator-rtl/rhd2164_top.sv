// ============================================================================
// rhd2164_top.sv  (SYNTHESIS TOP — Xilinx 7-series / Spartan-7 XC7S25)
// ----------------------------------------------------------------------------
// Emulates TWO RHD2164 chips sharing one CS/SCLK/MOSI LVDS bus, each driving
// its own MISO LVDS pair (the 128-channel headstage topology).
//
// This file instantiates vendor primitives (IBUFDS / OBUFDS / ODDR / MMCME2 /
// BUFG) and is therefore built only in Vivado, not in the iverilog simulation
// (the testbench drives the rhd2164_emulator core directly, single-ended).
//
// Clocking: 100 MHz board oscillator -> MMCM -> 400 MHz fast oversampling clock.
// All four channel BRAMs are initialised from the mem/ .mem files; set the
// search path with a Vivado constraint or by adding the mem/ files to the
// project so $readmemh resolves them.
// ============================================================================

`default_nettype none

module rhd2164_top (
    input  wire clk_100mhz,   // 100 MHz single-ended board oscillator
    input  wire rst_btn,      // active-high external reset (e.g. push button)

    // Shared SPI inputs (LVDS pairs) — driven by the controller to both chips
    input  wire cs_p,   input wire cs_n,
    input  wire sclk_p, input wire sclk_n,
    input  wire mosi_p, input wire mosi_n,

    // Dedicated MISO outputs (LVDS pairs) — one per emulated chip
    output wire miso0_p, output wire miso0_n,   // chip 0
    output wire miso1_p, output wire miso1_n    // chip 1
);

    // ------------------------------------------------------------------
    // Clock generation: 100 MHz -> 400 MHz (VCO = 800 MHz).
    // ------------------------------------------------------------------
    wire clk_ibuf, clk_fb, clk_fast_unbuf, clk_fast, mmcm_locked;

    IBUF u_clk_ibuf (.I(clk_100mhz), .O(clk_ibuf));

    MMCME2_BASE #(
        .CLKIN1_PERIOD   (10.000),   // 100 MHz
        .CLKFBOUT_MULT_F (8.000),    // VCO = 800 MHz
        .DIVCLK_DIVIDE   (1),
        .CLKOUT0_DIVIDE_F(2.000)     // 400 MHz
    ) u_mmcm (
        .CLKIN1   (clk_ibuf),
        .CLKFBIN  (clk_fb),
        .CLKFBOUT (clk_fb),
        .CLKOUT0  (clk_fast_unbuf),
        .CLKOUT1  (), .CLKOUT2(), .CLKOUT3(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2B(), .CLKOUT3B(),
        .CLKFBOUTB(),
        .LOCKED   (mmcm_locked),
        .PWRDWN   (1'b0),
        .RST      (rst_btn)
    );

    BUFG u_clk_bufg (.I(clk_fast_unbuf), .O(clk_fast));

    // Reset synchronizer: held low until MMCM locks, released on clk_fast.
    reg [3:0] rst_sync = 4'h0;
    wire rst_n;
    always @(posedge clk_fast or negedge mmcm_locked) begin
        if (!mmcm_locked) rst_sync <= 4'h0;
        else              rst_sync <= {rst_sync[2:0], 1'b1};
    end
    assign rst_n = rst_sync[3];

    // ------------------------------------------------------------------
    // Differential input receivers (shared by both chips).
    // ------------------------------------------------------------------
    wire cs_se, sclk_se, mosi_se;
    IBUFDS u_cs_ibufds   (.I(cs_p),   .IB(cs_n),   .O(cs_se));
    IBUFDS u_sclk_ibufds (.I(sclk_p), .IB(sclk_n), .O(sclk_se));
    IBUFDS u_mosi_ibufds (.I(mosi_p), .IB(mosi_n), .O(mosi_se));

    // ------------------------------------------------------------------
    // Two emulator cores.
    // ------------------------------------------------------------------
    wire miso0_core, miso1_core;

    rhd2164_emulator #(
        .MEM_A_FILE ("chip0_A.mem"),
        .MEM_B_FILE ("chip0_B.mem")
    ) u_chip0 (
        .clk (clk_fast), .rst_n (rst_n),
        .cs (cs_se), .sclk (sclk_se), .mosi (mosi_se), .miso (miso0_core)
    );

    rhd2164_emulator #(
        .MEM_A_FILE ("chip1_A.mem"),
        .MEM_B_FILE ("chip1_B.mem")
    ) u_chip1 (
        .clk (clk_fast), .rst_n (rst_n),
        .cs (cs_se), .sclk (sclk_se), .mosi (mosi_se), .miso (miso1_core)
    );

    // ------------------------------------------------------------------
    // MISO output: register in the IOB via ODDR (both phases identical, so the
    // bit is simply forwarded) for deterministic, low output delay, then drive
    // the LVDS pair through OBUFDS.
    // ------------------------------------------------------------------
    wire miso0_oddr, miso1_oddr;

    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("ASYNC")) u_oddr0 (
        .Q(miso0_oddr), .C(clk_fast), .CE(1'b1),
        .D1(miso0_core), .D2(miso0_core), .R(~rst_n), .S(1'b0)
    );
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("ASYNC")) u_oddr1 (
        .Q(miso1_oddr), .C(clk_fast), .CE(1'b1),
        .D1(miso1_core), .D2(miso1_core), .R(~rst_n), .S(1'b0)
    );

    OBUFDS u_miso0_obufds (.I(miso0_oddr), .O(miso0_p), .OB(miso0_n));
    OBUFDS u_miso1_obufds (.I(miso1_oddr), .O(miso1_p), .OB(miso1_n));

endmodule

`default_nettype wire
