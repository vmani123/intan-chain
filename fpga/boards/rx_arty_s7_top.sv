// ============================================================================
// rx_arty_s7_top.sv -- board wrapper for rx_top on a Digilent Arty S7-50.
// ----------------------------------------------------------------------------
// rx_top is board-independent and takes an already-buffered 96 MHz clock. This
// file is the only place that knows about the Arty: the MMCM, the reset button,
// and the pads. Keeping the split means the whole receiver simulates in Icarus
// with no vendor primitives, and only this ~100-line file is untested until it
// reaches hardware.
//
// CLOCKING: 100 MHz oscillator -> MMCM -> 96 MHz. Not 100 MHz, because SCLK is
// the system clock divided by OVERSAMPLE = 4, and 100/4 = 25 MHz is over the
// RHD2164's 24 MHz limit. 96/4 = 24.0 MHz exactly. VCO is 960 MHz, comfortably
// inside the Spartan-7 -1 range (600-1200 MHz). It is reached as 100/5 * 48:
// the obvious 100 * 9.6 is not on the MMCM's 0.125 multiplier grid, and
// write_bitstream rejects it (DRC AVAL-139). PFD = 20 MHz, inside 10-450 MHz.
//
// SINGLE-ENDED I/O on purpose: every user bank on this board is hard-wired to
// VCCO = 3.3 V and a 7-series HR bank only does LVDS_25, so native LVDS is not
// available here. The custom PCB powers the Intan-facing bank at 2.5 V and uses
// LVDS_25; on the bench the emulator board runs its own single-ended top and
// the two meet over a Pmod ribbon.
// ============================================================================

`default_nettype none

module rx_arty_s7_top #(
    parameter int QSPI_DIV = 2      // QSPI SCLK = 96 / (2 * QSPI_DIV) MHz
) (
    input  wire clk_100mhz,     // R2, SSTL135 -- sits in the DDR3 bank, not LVCMOS33
    input  wire rst_btn,        // BTN0, active high

    // Intan SPI, Pmod JA -- same pin positions as the emulator board, directions reversed
    output wire spi_cs_n,
    output wire spi_sclk,
    output wire spi_mosi,
    input  wire spi_miso0,
    input  wire spi_miso1,

    // QSPI to the ESP32-S3, Pmod JB
    output wire qspi_cs_n,
    output wire qspi_sclk,
    output wire [3:0] qspi_dq,
    input  wire esp_ready,

    // Control interface, onboard USB-UART
    input  wire uart_txd_in,    // FPGA receives on this pin (named from the bridge's view)
    output wire uart_rxd_out,   // FPGA transmits on this pin

    output wire [2:0] led
);

    // ------------------------------------------------------------------
    // 100 MHz -> 96 MHz
    // ------------------------------------------------------------------
    wire clk_ibuf, clk_fb, clk96_unbuf, clk96, mmcm_locked;

    IBUF u_clk_ibuf (.I(clk_100mhz), .O(clk_ibuf));

    MMCME2_BASE #(
        .CLKIN1_PERIOD   (10.000),   // 100 MHz
        .CLKFBOUT_MULT_F (48.000),   // VCO = 100 / 5 * 48 = 960 MHz
        .DIVCLK_DIVIDE   (5),
        .CLKOUT0_DIVIDE_F(10.000)    // 96 MHz
    ) u_mmcm (
        .CLKIN1   (clk_ibuf),
        .CLKFBIN  (clk_fb),
        .CLKFBOUT (clk_fb),
        .CLKOUT0  (clk96_unbuf),
        .CLKOUT1  (), .CLKOUT2(), .CLKOUT3(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2B(), .CLKOUT3B(),
        .CLKFBOUTB(),
        .LOCKED   (mmcm_locked),
        .PWRDWN   (1'b0),
        .RST      (rst_btn)
    );

    BUFG u_bufg (.I(clk96_unbuf), .O(clk96));

    // Held in reset until the MMCM locks, then released synchronously.
    reg [3:0] rst_sync = 4'h0;
    always @(posedge clk96 or negedge mmcm_locked) begin
        if (!mmcm_locked) rst_sync <= 4'h0;
        else              rst_sync <= {rst_sync[2:0], 1'b1};
    end
    wire rst_n = rst_sync[3];

    // ------------------------------------------------------------------
    // The receiver
    // ------------------------------------------------------------------
    wire qspi_oe;   // the link is one-way, so this is always 1; brought out so a
                    // bidirectional variant needs a change here and nowhere else

    rx_top #(.QSPI_DIV(QSPI_DIV)) u_rx (
        .clk        (clk96),
        .rst_n      (rst_n),

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

        .led        (led)
    );

endmodule

`default_nettype wire
