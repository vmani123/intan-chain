// ============================================================================
// chain_arty_s7_25_top.sv -- the whole bench on one Digilent Arty S7-25:
// two emulated RHD2164s, the receiver, and the QSPI egress to the ESP32-S3.
// ----------------------------------------------------------------------------
// The emulator and the receiver are separate designs that happen to share a
// die. They do not talk inside the FPGA: the receiver's SPI master leaves on
// Pmod JA, five jumper wires carry it to Pmod JB, and the emulator answers
// from there. So the bus crosses real pads, real wires and real input
// buffers, and PHASE_SELECT has a genuine cable delay to absorb -- the same
// job it will have against a real headstage.
//
//   JA1 -> JB1   cs_n          JA4 <- JB4   miso0 (chip 0)
//   JA2 -> JB2   sclk          JA7 <- JB7   miso1 (chip 1)
//   JA3 -> JB3   mosi
//
// The QSPI link to the ESP leaves on Pmod JC, in the same pin positions the
// two-board setup used on its receiver's Pmod: JC1 cs_n, JC2 sclk, JC3/4/7/8
// dq0..3, JC9 esp_ready in. Both halves share one ground, so the SPI loop
// needs no ground jumper; the ESP does.
//
// CLOCKING: 100 MHz -> one MMCM, VCO 960 MHz (100 / 5 * 48).
//   CLKOUT0 / 10 =  96 MHz  receiver. SCLK = 96 / OVERSAMPLE 4 = 24 MHz, the
//                           RHD2164 maximum.
//   CLKOUT1 /  5 = 192 MHz  emulator oversampling clock: 8 samples per SCLK
//                           period. The emulator met timing alone at 200 MHz
//                           on this part; 192 MHz is slightly easier.
// 9.6 is not on the MMCM's 0.125 multiplier grid (DRC AVAL-139), hence 48 / 5.
// No signal crosses between the two clock domains inside the FPGA, so they
// need no synchronisers and no clock-group constraints.
//
// LEDs: LD2 receiver run, LD3 receiver heartbeat, LD4 QSPI activity,
//       LD5 emulator activity (chip select seen on JB -- if this stays dark
//       while LD2 is on, a jumper is missing).
// ============================================================================

`default_nettype none

module chain_arty_s7_25_top #(
    parameter int QSPI_DIV = 2,                 // QSPI SCLK = 96 / (2 * QSPI_DIV) MHz
    parameter int BAUD     = 115_200,           // control UART; the testbench speeds it up
    // Bare names resolve against Vivado's source set; build.tcl passes
    // absolute paths, and the testbench passes repository paths.
    parameter MEM_C0A = "chip0_A.mem",
    parameter MEM_C0B = "chip0_B.mem",
    parameter MEM_C1A = "chip1_A.mem",
    parameter MEM_C1B = "chip1_B.mem",
    parameter int ACT_BITS = 23                 // ~44 ms activity stretch at 192 MHz
) (
    input  wire       clk_100mhz,   // R2, SSTL135 -- sits in the DDR3 bank
    input  wire       rst_btn,      // BTN0, active high

    // Receiver: RHD2164 SPI master, Pmod JA
    output wire       spi_cs_n,
    output wire       spi_sclk,
    output wire       spi_mosi,
    input  wire       spi_miso0,
    input  wire       spi_miso1,

    // Emulator: two RHD2164 slaves sharing one bus, Pmod JB
    input  wire       emu_cs_n,
    input  wire       emu_sclk,
    input  wire       emu_mosi,
    output wire       emu_miso0,
    output wire       emu_miso1,

    // QSPI to the ESP32-S3, Pmod JC
    output wire       qspi_cs_n,
    output wire       qspi_sclk,
    output wire [3:0] qspi_dq,
    input  wire       esp_ready,

    // Control interface, onboard USB-UART (named from the bridge's view)
    input  wire       uart_txd_in,  // FPGA receives
    output wire       uart_rxd_out, // FPGA transmits

    output wire [3:0] led
);

    // ------------------------------------------------------------------
    // 100 MHz -> 96 MHz (receiver) and 192 MHz (emulator)
    // ------------------------------------------------------------------
    wire clk_ibuf, clk_fb, clk96_unbuf, clk192_unbuf, clk96, clk192, mmcm_locked;

    IBUF u_clk_ibuf (.I(clk_100mhz), .O(clk_ibuf));

    MMCME2_BASE #(
        .CLKIN1_PERIOD   (10.000),   // 100 MHz
        .CLKFBOUT_MULT_F (48.000),   // VCO = 100 / 5 * 48 = 960 MHz
        .DIVCLK_DIVIDE   (5),
        .CLKOUT0_DIVIDE_F(10.000),   // 96 MHz
        .CLKOUT1_DIVIDE  (5)         // 192 MHz
    ) u_mmcm (
        .CLKIN1   (clk_ibuf),
        .CLKFBIN  (clk_fb),
        .CLKFBOUT (clk_fb),
        .CLKOUT0  (clk96_unbuf),
        .CLKOUT1  (clk192_unbuf),
        .CLKOUT2  (), .CLKOUT3(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2B(), .CLKOUT3B(),
        .CLKFBOUTB(),
        .LOCKED   (mmcm_locked),
        .PWRDWN   (1'b0),
        .RST      (rst_btn)
    );

    BUFG u_bufg96  (.I(clk96_unbuf),  .O(clk96));
    BUFG u_bufg192 (.I(clk192_unbuf), .O(clk192));

    // Each domain is held in reset until the MMCM locks, then released
    // synchronously to its own clock.
    reg [3:0] rst96_sync  = 4'h0;
    reg [3:0] rst192_sync = 4'h0;
    always @(posedge clk96 or negedge mmcm_locked) begin
        if (!mmcm_locked) rst96_sync <= 4'h0;
        else              rst96_sync <= {rst96_sync[2:0], 1'b1};
    end
    always @(posedge clk192 or negedge mmcm_locked) begin
        if (!mmcm_locked) rst192_sync <= 4'h0;
        else              rst192_sync <= {rst192_sync[2:0], 1'b1};
    end
    wire rst96_n  = rst96_sync[3];
    wire rst192_n = rst192_sync[3];

    // ------------------------------------------------------------------
    // The receiver, 96 MHz
    // ------------------------------------------------------------------
    wire       qspi_oe;      // one-way link: always 1
    wire [2:0] rx_led;

    rx_top #(.BAUD(BAUD), .QSPI_DIV(QSPI_DIV)) u_rx (
        .clk        (clk96),
        .rst_n      (rst96_n),

        .uart_rx_i  (uart_txd_in),
        .uart_tx_o  (uart_rxd_out),

        .spi_cs_n   (spi_cs_n),
        .spi_sclk   (spi_sclk),
        .spi_mosi   (spi_mosi),
        .miso0      (spi_miso0),
        .miso1      (spi_miso1),

        .qspi_cs_n  (qspi_cs_n),
        .qspi_sclk  (qspi_sclk),
        .qspi_dq_out(qspi_dq),
        .qspi_dq_oe (qspi_oe),
        .esp_ready  (esp_ready),

        .led        (rx_led),
        .mode_o     (),
        .link_good  ()
    );

    // ------------------------------------------------------------------
    // The emulator, 192 MHz: two cores on one bus, as in rhd2164_top_se
    // ------------------------------------------------------------------
    wire miso0_core, miso1_core;

    rhd2164_emulator #(
        .MEM_A_FILE (MEM_C0A),
        .MEM_B_FILE (MEM_C0B)
    ) u_chip0 (
        .clk (clk192), .rst_n (rst192_n),
        .cs (emu_cs_n), .sclk (emu_sclk), .mosi (emu_mosi), .miso (miso0_core)
    );

    rhd2164_emulator #(
        .MEM_A_FILE (MEM_C1A),
        .MEM_B_FILE (MEM_C1B)
    ) u_chip1 (
        .clk (clk192), .rst_n (rst192_n),
        .cs (emu_cs_n), .sclk (emu_sclk), .mosi (emu_mosi), .miso (miso1_core)
    );

    // MISO is registered in the output flop of the pad, so its delay from the
    // emulator's clock is fixed and short whatever placement does. This is
    // what rhd2164_top_se's ODDR achieves, without a vendor primitive.
    (* IOB = "TRUE" *) reg miso0_q = 1'b0;
    (* IOB = "TRUE" *) reg miso1_q = 1'b0;
    always @(posedge clk192) begin
        miso0_q <= miso0_core;
        miso1_q <= miso1_core;
    end
    assign emu_miso0 = miso0_q;
    assign emu_miso1 = miso1_q;

    // Stretch every chip-select falling edge the emulator sees, so the eye
    // can tell a live SPI loop from a missing jumper.
    reg                cs_d1 = 1'b1, cs_d2 = 1'b1;
    reg [ACT_BITS-1:0] act_cnt = '0;
    always @(posedge clk192) begin
        cs_d1 <= emu_cs_n;
        cs_d2 <= cs_d1;
        if (cs_d2 && !cs_d1)   act_cnt <= '1;
        else if (act_cnt != 0) act_cnt <= act_cnt - 1'b1;
    end

    assign led = {(act_cnt != 0), rx_led};

endmodule

`default_nettype wire
