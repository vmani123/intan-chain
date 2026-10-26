# ============================================================================
# headstage_arty_s7_25.xdc -- headstage_arty_s7_25_top on a Digilent Arty S7-25
# (xc7s25csga324-1): receiver to two real RHD2164s via an LVDS converter
# board on Pmod JA, QSPI to the ESP32-S3 on Pmod JC.
#
# Pin numbers from Digilent's Arty-S7-25-Master.xdc, the same as
# chain_arty_s7_25.xdc. JA is a high-speed Pmod (0 R, routed as pairs); JC is
# standard (200 R series).
#
# CONVERTER BOARD (single-ended side, 3.3 V logic), from Pmod JA:
#
#   JA1  cs_n   ->  CS   (both chips)      JA4  miso0  <-  MISO, chip 0 (ch 0..63)
#   JA2  sclk   ->  SCLK (both chips)      JA7  miso1  <-  MISO, chip 1 (ch 64..127)
#   JA3  mosi   ->  MOSI (both chips)
#   JA5 / JA11  GND  -- converter GND (required: a separate board)
#   JA6         3V3  -- converter board supply (the RHD2164s and LVDS parts
#                       draw tens of mA; the Arty's 3.3 V rail has room)
#
#   Nothing on Pmod JB: the single-board bench's JA -> JB jumpers come off.
#
# ESP32-S3 (pins_s3.h), from Pmod JC, unchanged:
#
#   JC1  qspi_cs_n   -> GPIO10        JC7  qspi_dq[2]  -> GPIO14
#   JC2  qspi_sclk   -> GPIO12        JC8  qspi_dq[3]  -> GPIO9
#   JC3  qspi_dq[0]  -> GPIO11        JC9  esp_ready   <- GPIO21
#   JC4  qspi_dq[1]  -> GPIO13        JC5 or JC11 GND  -- ESP GND (required)
#
#   Not JC6/JC12 (3V3): the ESP is powered from its own USB cable.
# ============================================================================

# ----------------------------------------------------------------------------
# Clock. The 100 MHz oscillator lands on R2, inside the DDR3 bank, so its
# IOSTANDARD is SSTL135 and NOT LVCMOS33.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN R2  IOSTANDARD SSTL135 } [get_ports { clk_100mhz }]
create_clock -name clk_100mhz -period 10.000 [get_ports { clk_100mhz }]
create_generated_clock -name clk96  [get_pins u_mmcm/CLKOUT0]
create_generated_clock -name clk192 [get_pins u_mmcm/CLKOUT1]

set_property -dict { PACKAGE_PIN G15 IOSTANDARD LVCMOS33 } [get_ports { rst_btn }] ;# BTN0

# ----------------------------------------------------------------------------
# RHD2164 SPI to the converter board -- Pmod JA (high-speed)
# MISO inputs are pulled down, so a MISO wire that is off reads "stuck low",
# distinct from a converter whose LVDS receiver fails safe high because the
# chip is not driving it ("stuck high"). The bring-up script tells them apart.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports { spi_cs_n  }] ;# JA1
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { spi_sclk  }] ;# JA2
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports { spi_mosi  }] ;# JA3
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 PULLDOWN TRUE } [get_ports { spi_miso0 }] ;# JA4
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 PULLDOWN TRUE } [get_ports { spi_miso1 }] ;# JA7

set_property DRIVE 8    [get_ports { spi_cs_n spi_sclk spi_mosi }]
set_property SLEW  FAST [get_ports { spi_sclk }]

# ----------------------------------------------------------------------------
# QSPI to the ESP32-S3 -- Pmod JC (standard, 200 R series)
# esp_ready is pulled down so an unplugged ESP reads "not ready": STALLS then
# climbs and no datagram leaves, instead of datagrams going nowhere.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports { qspi_cs_n  }] ;# JC1
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports { qspi_sclk  }] ;# JC2
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[0] }] ;# JC3
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[1] }] ;# JC4
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[2] }] ;# JC7
set_property -dict { PACKAGE_PIN P13 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[3] }] ;# JC8
set_property -dict { PACKAGE_PIN R13 IOSTANDARD LVCMOS33 PULLDOWN TRUE } [get_ports { esp_ready }] ;# JC9

set_property DRIVE 8    [get_ports { qspi_cs_n qspi_sclk qspi_dq[*] }]
set_property SLEW  FAST [get_ports { qspi_sclk qspi_dq[*] }]

# ----------------------------------------------------------------------------
# Control interface -- onboard USB-UART, same cable as JTAG.
# Digilent names these from the USB bridge's point of view, so uart_rxd_out is
# the FPGA's TRANSMIT and uart_txd_in is the FPGA's RECEIVE.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { uart_txd_in  }] ;# PC  -> FPGA
set_property -dict { PACKAGE_PIN R12 IOSTANDARD LVCMOS33 } [get_ports { uart_rxd_out }] ;# FPGA -> PC

# ----------------------------------------------------------------------------
# LEDs: LD2 receiver run, LD3 heartbeat, LD4 QSPI activity, LD5 link good.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { led[0] }] ;# LD2
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports { led[1] }] ;# LD3
set_property -dict { PACKAGE_PIN E13 IOSTANDARD LVCMOS33 } [get_ports { led[2] }] ;# LD4
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33 } [get_ports { led[3] }] ;# LD5

# ----------------------------------------------------------------------------
# Timing.
# MISO is caught by a pad flop and then oversampled with a runtime phase
# select, so there is no setup/hold relationship to declare against SCLK:
# PHASE_SELECT absorbs converter, cable and pad delay. esp_ready goes into a
# two-flop synchroniser.
#
# The loopback crosses between clk96 and clk192. The emulator oversamples its
# inputs through synchronisers and the receiver oversamples MISO, so neither
# needs a timed relationship -- only a bound, so the self-test's phase window
# does not depend on where the router happened to put a net.
# ----------------------------------------------------------------------------
set_false_path -from [get_ports { spi_miso0 spi_miso1 esp_ready rst_btn uart_txd_in }]
set_false_path -to   [get_ports { led[*] }]

set_max_delay -datapath_only -from [get_clocks clk96]  -to [get_clocks clk192] 5.000
set_max_delay -datapath_only -from [get_clocks clk192] -to [get_clocks clk96]  5.000

set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
