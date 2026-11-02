// ============================================================================
// tap_arty_s7_25_top.sv -- the receiver on a Digilent Arty S7-25 as a passive
// tap on an Intan controller's SPI cable (CONTRACTS.md section 7,
// docs/intan-tap-check.md).
// ----------------------------------------------------------------------------
// The Intan RHD recording controller stays the headstage's SPI master. Five
// LVDS pairs are teed off a pass-through between two Intan SPI cables and land
// on the Arty's high-speed Pmod pairs, receive only:
//
//   JA1/JA2  CS    (L17/L18)       JA9/JA10 MISO1, chip 0 (M18/N18)
//   JA3/JA4  SCLK  (M14/N14)       JB1/JB2  MISO2, chip 1 (P17/P18)
//   JA7/JA8  MOSI  (M16/M17)       JA5 / JA11 GND to the cable ground
//
// Each pair goes through an IBUFDS with internal termination off: the cable is
// already terminated (CS, SCLK and MOSI at the headstage, MISO inside the
// controller), so the tap only loads the lines and never drives them. Port
// names are pair halves: tap_cs_p / tap_cs_n are the + and - legs of CS
// (which is itself active low). The QSPI link to the ESP is unchanged on
// Pmod JC. Nothing drives an SPI bus off the board in this build.
//
// THE ON-DIE EMULATOR, AND THE SELF-TEST
//   Two emulated RHD2164s sit inside the FPGA. MODE bit3 connects the
//   receiver's own SPI master to them, exactly as in the headstage build, so
//   the receiver's self-test still works here. MODE bit3 together with bit4
//   also points the tap at that internal bus instead of the pins: with `run`
//   set, spi_master sweeps the emulators and the frames come from the tap, so
//   the whole tap path -- decode, sweep assembly, frame builder, QSPI -- can
//   be proven on the bench with nothing plugged in. The emulator answers
//   about three clocks after an SCLK edge, so the self-test needs a slow SCLK:
//
//     W 04 0000000f   SPI_DIV 15: SCLK 1.5 MHz, half period 32 clocks
//     W 03 00000018   MODE: loopback + tap
//     W 00 00000001   run (SAMPLE_PERIOD's reset value 48000 = 2 kS/s fits)
//
//   With bit3 clear the receiver's MISO inputs read 0 (MISO_STAT says
//   "stuck low") because its bus goes nowhere, and the emulators see CS high.
//
// PAD REGISTERS
//   Each external tap line is caught by an input flop in its pad, so the five
//   lines reach the tap's synchronisers with equal, placement-independent
//   delay. That is one clock on all five alike, which leaves TAP_DELAY's
//   meaning unchanged. The internal (self-test) bus skips them: it comes from
//   fabric flops already.
//
// CLOCKING: as headstage_arty_s7_25_top. 100 MHz -> one MMCM, VCO 960 MHz
//   CLKOUT0 / 10 =  96 MHz  receiver and tap
//   CLKOUT1 /  5 = 192 MHz  emulator oversampling clock
//
// LEDs: LD2 receiver run, LD3 heartbeat, LD4 QSPI activity,
//       LD5 tap hearing clean traffic -- a CS cycle with exactly 16 + 16 SCLK
//       edges within the last 44 ms, and no framing or known-answer error in
//       the last 44 ms. Dark with the controller streaming: nothing heard
//       (wiring, ground, a swapped pair) or the wrong TAP_DELAY;
//       `python3 -m chain.tapcheck status` reads TAP_STAT and says which.
// ============================================================================

`default_nettype none

module tap_arty_s7_25_top #(
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

    // The overheard SPI bus, LVDS_25 pairs on Pmod JA / JB, receive only
    input  wire       tap_cs_p,    tap_cs_n,
    input  wire       tap_sclk_p,  tap_sclk_n,
    input  wire       tap_mosi_p,  tap_mosi_n,
    input  wire       tap_miso0_p, tap_miso0_n,   // cable MISO1: chip 0
    input  wire       tap_miso1_p, tap_miso1_n,   // cable MISO2: chip 1

    // QSPI to the ESP, Pmod JC
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
    // 100 MHz -> 96 MHz (receiver, tap) and 192 MHz (emulator)
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
    // The tap pins: LVDS receivers, then one pad flop each
    // ------------------------------------------------------------------
    wire ib_cs_n, ib_sclk, ib_mosi, ib_miso0, ib_miso1;

    IBUFDS #(.DIFF_TERM("FALSE")) u_ib_cs    (.I(tap_cs_p),    .IB(tap_cs_n),    .O(ib_cs_n));
    IBUFDS #(.DIFF_TERM("FALSE")) u_ib_sclk  (.I(tap_sclk_p),  .IB(tap_sclk_n),  .O(ib_sclk));
    IBUFDS #(.DIFF_TERM("FALSE")) u_ib_mosi  (.I(tap_mosi_p),  .IB(tap_mosi_n),  .O(ib_mosi));
    IBUFDS #(.DIFF_TERM("FALSE")) u_ib_miso0 (.I(tap_miso0_p), .IB(tap_miso0_n), .O(ib_miso0));
    IBUFDS #(.DIFF_TERM("FALSE")) u_ib_miso1 (.I(tap_miso1_p), .IB(tap_miso1_n), .O(ib_miso1));

    (* IOB = "TRUE" *) reg pad_cs_n  = 1'b1;
    (* IOB = "TRUE" *) reg pad_sclk  = 1'b0;
    (* IOB = "TRUE" *) reg pad_mosi  = 1'b0;
    (* IOB = "TRUE" *) reg pad_miso0 = 1'b0;
    (* IOB = "TRUE" *) reg pad_miso1 = 1'b0;

    always @(posedge clk96) begin
        pad_cs_n  <= ib_cs_n;
        pad_sclk  <= ib_sclk;
        pad_mosi  <= ib_mosi;
        pad_miso0 <= ib_miso0;
        pad_miso1 <= ib_miso1;
    end

    // ------------------------------------------------------------------
    // The receiver, 96 MHz
    // ------------------------------------------------------------------
    wire        rx_cs_n, rx_sclk, rx_mosi;
    wire        rx_miso0, rx_miso1;
    wire        t_cs_n, t_sclk, t_mosi, t_miso0, t_miso1;
    wire        qspi_oe;      // one-way link: always 1
    wire [2:0]  rx_led;
    wire [31:0] rx_mode;
    wire        tap_good;

    rx_top #(.BAUD(BAUD), .QSPI_DIV(QSPI_DIV), .HAS_EMU(1'b1), .HAS_TAP(1'b1)) u_rx (
        .clk        (clk96),
        .rst_n      (rst96_n),

        .uart_rx_i  (uart_txd_in),
        .uart_tx_o  (uart_rxd_out),

        .spi_cs_n   (rx_cs_n),
        .spi_sclk   (rx_sclk),
        .spi_mosi   (rx_mosi),
        .miso0      (rx_miso0),
        .miso1      (rx_miso1),

        .tap_cs_n   (t_cs_n),
        .tap_sclk   (t_sclk),
        .tap_mosi   (t_mosi),
        .tap_miso0  (t_miso0),
        .tap_miso1  (t_miso1),

        .qspi_cs_n  (qspi_cs_n),
        .qspi_sclk  (qspi_sclk),
        .qspi_dq_out(qspi_dq),
        .qspi_dq_oe (qspi_oe),
        .esp_ready  (esp_ready),

        .led        (rx_led),
        .mode_o     (rx_mode),
        .link_good  (),
        .tap_good   (tap_good)
    );

    wire loop    = rx_mode[3];                  // receiver <-> on-die emulator
    wire tap_int = rx_mode[3] && rx_mode[4];    // tap listens to that bus

    // ------------------------------------------------------------------
    // Loopback: the receiver's bus, through one fabric register, into the
    // on-die emulator. This register's output is also what the tap hears in
    // the self-test, so tap and emulator see the same edges.
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

    // No SPI pins of its own: without the loopback the receiver hears nothing.
    assign rx_miso0 = loop ? emu_miso0 : 1'b0;
    assign rx_miso1 = loop ? emu_miso1 : 1'b0;

    // ------------------------------------------------------------------
    // What the tap listens to: the pins, or the internal bus (self-test)
    // ------------------------------------------------------------------
    assign t_cs_n  = tap_int ? emu_cs_n  : pad_cs_n;
    assign t_sclk  = tap_int ? emu_sclk  : pad_sclk;
    assign t_mosi  = tap_int ? emu_mosi  : pad_mosi;
    assign t_miso0 = tap_int ? emu_miso0 : pad_miso0;
    assign t_miso1 = tap_int ? emu_miso1 : pad_miso1;

    assign led = {tap_good, rx_led};

endmodule

`default_nettype wire
