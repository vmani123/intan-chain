// ============================================================================
// headstage_arty_s7_25_top.sv -- the receiver on a Digilent Arty S7-25, wired
// to two REAL RHD2164 chips through an LVDS <-> single-ended converter board.
// ----------------------------------------------------------------------------
// Pmod JA carries the SPI bus to the converter board, the same pins the
// single-board bench used for its loop:
//
//   JA1 -> cs_n          JA4 <- miso0 (chip 0: channels   0..63)
//   JA2 -> sclk          JA7 <- miso1 (chip 1: channels  64..127)
//   JA3 -> mosi          JA5 / JA11 GND, JA6 3V3 to the converter board
//
// Both chips share CS, SCLK and MOSI and each drives its own MISO, as in
// Intan's 128-channel headstage. The QSPI link to the ESP is unchanged on
// Pmod JC.
//
// THE ON-DIE EMULATOR, AND WHY IT IS STILL HERE
//   Two emulated RHD2164s sit inside the FPGA with no pins of their own. MODE
//   bit3 routes the receiver's SPI to them instead of to JA, so the whole
//   receiver -- sequencer, phase select, diagnostic list, link check -- can be
//   proven on the bench without touching a wire. If the self-test passes and
//   the real chips do not answer, the fault is outside the FPGA. While looped
//   back, JA is held idle (CS high), so the real chips see no traffic.
//
// PAD REGISTERS
//   CS, SCLK and MOSI leave from output flops in the pads, and each MISO is
//   caught by an input flop in its pad, so every capture tap in spi_master
//   sees MISO at one fixed instant whatever placement does. That costs one
//   clock each way, which PHASE_SELECT absorbs like any cable delay: expect the
//   clean window a tick or two later than on the single-board bench. The
//   loopback path has its own fabric flops, because a pad flop can only drive
//   its pad.
//
// CLOCKING: as chain_arty_s7_25_top. 100 MHz -> one MMCM, VCO 960 MHz
//   CLKOUT0 / 10 =  96 MHz  receiver (SCLK = 24 MHz / (SPI_DIV + 1))
//   CLKOUT1 /  5 = 192 MHz  emulator oversampling clock
// The loopback signals cross between the two domains; the emulator
// synchronises its inputs and the receiver oversamples MISO, so the XDC only
// bounds those paths instead of timing them.
//
// LEDs: LD2 receiver run, LD3 heartbeat, LD4 QSPI activity,
//       LD5 link good -- the last sweep's READ(59) came back right from both
//       chips, both modules. LD2 lit with LD5 dark means the chips are not
//       answering; `python3 -m chain.bringup` says which one and why.
// ============================================================================

`default_nettype none

module headstage_arty_s7_25_top #(
    parameter int QSPI_DIV = 2,                 // QSPI SCLK = 96 / (2 * QSPI_DIV) MHz
    parameter int BAUD     = 115_200,           // control UART; the testbench speeds it up
    // The emulator's playback memories. Bare names resolve against Vivado's
    // source set; build.tcl passes absolute paths, the testbench repo paths.
    parameter MEM_C0A = "chip0_A.mem",
    parameter MEM_C0B = "chip0_B.mem",
    parameter MEM_C1A = "chip1_A.mem",
    parameter MEM_C1B = "chip1_B.mem"
) (
    input  wire       clk_100mhz,   // R2, SSTL135 -- sits in the DDR3 bank
    input  wire       rst_btn,      // BTN0, active high

    // RHD2164 SPI bus to the converter board, Pmod JA
    output wire       spi_cs_n,
    output wire       spi_sclk,
    output wire       spi_mosi,
    input  wire       spi_miso0,
    input  wire       spi_miso1,

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
    wire        rx_cs_n, rx_sclk, rx_mosi;
    wire        rx_miso0, rx_miso1;
    wire        qspi_oe;      // one-way link: always 1
    wire [2:0]  rx_led;
    wire [31:0] rx_mode;
    wire        link_good;

    rx_top #(.BAUD(BAUD), .QSPI_DIV(QSPI_DIV), .HAS_EMU(1'b1)) u_rx (
        .clk        (clk96),
        .rst_n      (rst96_n),

        .uart_rx_i  (uart_txd_in),
        .uart_tx_o  (uart_rxd_out),

        .spi_cs_n   (rx_cs_n),
        .spi_sclk   (rx_sclk),
        .spi_mosi   (rx_mosi),
        .miso0      (rx_miso0),
        .miso1      (rx_miso1),

        .qspi_cs_n  (qspi_cs_n),
        .qspi_sclk  (qspi_sclk),
        .qspi_dq_out(qspi_dq),
        .qspi_dq_oe (qspi_oe),
        .esp_ready  (esp_ready),

        .led        (rx_led),
        .mode_o     (rx_mode),
        .link_good  (link_good)
    );

    wire loop = rx_mode[3];

    // ------------------------------------------------------------------
    // Pmod JA: pad flops both ways. Idle while looped back.
    // ------------------------------------------------------------------
    (* IOB = "TRUE" *) reg pad_cs_n  = 1'b1;
    (* IOB = "TRUE" *) reg pad_sclk  = 1'b0;
    (* IOB = "TRUE" *) reg pad_mosi  = 1'b0;
    (* IOB = "TRUE" *) reg pad_miso0 = 1'b0;
    (* IOB = "TRUE" *) reg pad_miso1 = 1'b0;

    always @(posedge clk96) begin
        pad_cs_n  <= loop ? 1'b1 : rx_cs_n;
        pad_sclk  <= loop ? 1'b0 : rx_sclk;
        pad_mosi  <= loop ? 1'b0 : rx_mosi;
        pad_miso0 <= spi_miso0;
        pad_miso1 <= spi_miso1;
    end

    assign spi_cs_n = pad_cs_n;
    assign spi_sclk = pad_sclk;
    assign spi_mosi = pad_mosi;

    // ------------------------------------------------------------------
    // Loopback: the same one-clock register, into the on-die emulator
    // ------------------------------------------------------------------
    reg emu_cs_n = 1'b1, emu_sclk = 1'b0, emu_mosi = 1'b0;
    always @(posedge clk96) begin
        emu_cs_n <= loop ? rx_cs_n : 1'b1;
        emu_sclk <= loop ? rx_sclk : 1'b0;
        emu_mosi <= loop ? rx_mosi : 1'b0;
    end

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

    reg emu_miso0 = 1'b0, emu_miso1 = 1'b0;
    always @(posedge clk192) begin
        emu_miso0 <= miso0_core;
        emu_miso1 <= miso1_core;
    end

    assign rx_miso0 = loop ? emu_miso0 : pad_miso0;
    assign rx_miso1 = loop ? emu_miso1 : pad_miso1;

    assign led = {link_good, rx_led};

endmodule

`default_nettype wire
