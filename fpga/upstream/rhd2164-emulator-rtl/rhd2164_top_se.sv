// ============================================================================
// rhd2164_top_se.sv   (SYNTHESIS TOP -- single-ended I/O, Arty S7-25 / S7-50)
// ----------------------------------------------------------------------------
// Single-ended sibling of rhd2164_top.sv. Drop this file into rtl/ and select
// it as the Vivado top instead of rhd2164_top.sv. Nothing else changes.
//
// WHY THIS FILE EXISTS
//   rhd2164_top.sv presents the SPI bus as LVDS (IBUFDS / OBUFDS, LVDS_25 in
//   the .xdc). That is correct for the final PCB, because a real RHD2164
//   headstage cable is LVDS. It cannot be built for an Arty S7: every user I/O
//   bank on that board is hard-wired to VCCO = 3.3 V, and a 7-series HR bank
//   can only do LVDS_25, which needs VCCO 2.375-2.625 V. Vivado rejects it at
//   DRC before it ever reaches the bitstream.
//
//   This top keeps the emulator cores byte-identical and swaps only the pad
//   buffers: IBUFDS -> IBUF, OBUFDS -> OBUF, LVCMOS33 pins on Pmod JA. It is
//   the same separation Intan use in Rhythm's main.v, where the differential
//   buffers occupy 30 lines at the top and everything below them is
//   single-ended.
//
// WHAT IS UNCHANGED
//   rhd2164_emulator.sv, spi_frontend.sv, command_decoder.sv, register_file.sv,
//   ddr_miso.sv, and mem/*.mem. The bit-level protocol on the wire is
//   identical; only its electrical representation differs. The iverilog
//   testbench already drives the core single-ended, so tb_rhd2164.sv keeps
//   passing untouched.
//
// PAIRING WITH THE RECEIVER
//   Board A (this bitstream) and board B (the receiver) each use Pmod JA in
//   the same pin positions, with directions mirrored:
//
//       JA pin   this emulator      receiver board
//       ------   ----------------   ----------------
//       JA1      cs_in      (in)    cs_out    (out)
//       JA2      sclk_in    (in)    sclk_out  (out)
//       JA3      mosi_in    (in)    mosi_out  (out)
//       JA4      miso0_out  (out)   miso0_in  (in)
//       JA7      miso1_out  (out)   miso1_in  (in)
//       JA5/11   GND                GND
//
//   >>> DO NOT join JA pin 6 or 12 (VCC) between two separately powered
//   boards. Use five signal jumpers plus at least one ground, or a 12-pin
//   Pmod ribbon with the two VCC conductors removed.
// ============================================================================

`default_nettype none

module rhd2164_top_se #(
    // Fast oversampling clock = 100 MHz * 8 / FAST_CLK_DIV.
    //   2.000 -> 400 MHz (matches rhd2164_top.sv; ~16x oversample at 24 MHz SCLK)
    //   4.000 -> 200 MHz (~8x oversample; use this if 400 MHz fails timing --
    //                     Arty S7 boards are speed grade -1)
    parameter real FAST_CLK_DIV = 2.000,

    // Channel-data memories. The defaults are bare filenames because Vivado
    // resolves $readmemh against the project's source set. sim/tb_top_se.sv
    // overrides them with "mem/..." paths so the testbench can run from the
    // repository root.
    parameter MEM_C0A = "chip0_A.mem",
    parameter MEM_C0B = "chip0_B.mem",
    parameter MEM_C1A = "chip1_A.mem",
    parameter MEM_C1B = "chip1_B.mem",

    // LED timing, as bits of a counter running at the fast clock. The defaults
    // are human-visible; the testbench shrinks them so both LEDs can be seen to
    // assert AND decay inside a short simulation.
    parameter int HB_BITS  = 26,   // ~1.5 Hz at 400 MHz
    parameter int ACT_BITS = 23    // ~21 ms activity stretch at 400 MHz
) (
    input  wire clk_100mhz,   // Arty S7: pin R2, IOSTANDARD SSTL135 (shared with DDR3)
    input  wire rst_btn,      // Arty S7: BTN0 (G15), active high

    // Single-ended SPI inputs -- shared bus driven by the receiver board
    input  wire cs_in,        // active-low chip select
    input  wire sclk_in,      // serial clock, CPOL = 0
    input  wire mosi_in,      // command in, MSB first

    // Single-ended MISO outputs -- one per emulated chip
    output wire miso0_out,    // chip 0
    output wire miso1_out,    // chip 1

    // Bench sanity LEDs (optional -- delete the ports and the .xdc lines if unwanted)
    output wire led_locked,   // MMCM locked
    output wire led_heartbeat,// ~1 Hz blink: the fast clock is really running
    output wire led_activity  // stretched CS activity: the receiver is really talking
);

    // ------------------------------------------------------------------
    // Clock generation: 100 MHz -> 800 MHz VCO -> fast oversampling clock.
    // ------------------------------------------------------------------
    wire clk_ibuf, clk_fb, clk_fast_unbuf, clk_fast, mmcm_locked;

    IBUF u_clk_ibuf (.I(clk_100mhz), .O(clk_ibuf));

    MMCME2_BASE #(
        .CLKIN1_PERIOD   (10.000),        // 100 MHz
        .CLKFBOUT_MULT_F (8.000),         // VCO = 800 MHz
        .DIVCLK_DIVIDE   (1),
        .CLKOUT0_DIVIDE_F(FAST_CLK_DIV)
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

    // Reset synchronizer: held low until the MMCM locks, released on clk_fast.
    reg [3:0] rst_sync = 4'h0;
    wire rst_n;
    always @(posedge clk_fast or negedge mmcm_locked) begin
        if (!mmcm_locked) rst_sync <= 4'h0;
        else              rst_sync <= {rst_sync[2:0], 1'b1};
    end
    assign rst_n = rst_sync[3];

    // ------------------------------------------------------------------
    // Single-ended input buffers (shared by both emulated chips).
    // This is the ONLY functional difference from rhd2164_top.sv.
    // ------------------------------------------------------------------
    wire cs_se, sclk_se, mosi_se;
    IBUF u_cs_ibuf   (.I(cs_in),   .O(cs_se));
    IBUF u_sclk_ibuf (.I(sclk_in), .O(sclk_se));
    IBUF u_mosi_ibuf (.I(mosi_in), .O(mosi_se));

    // ------------------------------------------------------------------
    // Two emulator cores -- identical instantiation to the LVDS top.
    // ------------------------------------------------------------------
    wire miso0_core, miso1_core;

    rhd2164_emulator #(
        .MEM_A_FILE (MEM_C0A),
        .MEM_B_FILE (MEM_C0B)
    ) u_chip0 (
        .clk (clk_fast), .rst_n (rst_n),
        .cs (cs_se), .sclk (sclk_se), .mosi (mosi_se), .miso (miso0_core)
    );

    rhd2164_emulator #(
        .MEM_A_FILE (MEM_C1A),
        .MEM_B_FILE (MEM_C1B)
    ) u_chip1 (
        .clk (clk_fast), .rst_n (rst_n),
        .cs (cs_se), .sclk (sclk_se), .mosi (mosi_se), .miso (miso1_core)
    );

    // ------------------------------------------------------------------
    // MISO output: register in the IOB via ODDR (both phases identical, so the
    // bit is simply forwarded) for deterministic, low output delay, then drive
    // the single-ended pad through OBUF.
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

    OBUF u_miso0_obuf (.I(miso0_oddr), .O(miso0_out));
    OBUF u_miso1_obuf (.I(miso1_oddr), .O(miso1_out));

    // ------------------------------------------------------------------
    // Bench sanity LEDs.
    //
    // These answer the two questions you always ask first at the bench and
    // cannot otherwise see: "is this board alive?" and "is the other board
    // actually driving CS?" Without led_activity, a dead link and a wrong
    // pinout look identical.
    // ------------------------------------------------------------------
    reg [HB_BITS-1:0] hb_cnt = '0;
    always @(posedge clk_fast) hb_cnt <= hb_cnt + 1'b1;

    // Stretch any CS falling edge so the eye can see it.
    reg                cs_d1 = 1'b1, cs_d2 = 1'b1;
    reg [ACT_BITS-1:0] act_cnt = '0;
    always @(posedge clk_fast) begin
        cs_d1 <= cs_se;
        cs_d2 <= cs_d1;
        if (cs_d2 && !cs_d1)        act_cnt <= '1;      // CS falling edge
        else if (act_cnt != 0)      act_cnt <= act_cnt - 1'b1;
    end

    assign led_locked    = mmcm_locked;
    assign led_heartbeat = hb_cnt[HB_BITS-1];
    assign led_activity  = (act_cnt != 0);

endmodule

`default_nettype wire
