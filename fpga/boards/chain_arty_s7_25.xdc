# ============================================================================
# chain_arty_s7_25.xdc -- chain_arty_s7_25_top on a Digilent Arty S7-25
# (xc7s25csga324-1): emulator, receiver and QSPI egress on one board.
#
# Pin numbers from Digilent's Arty-S7-25-Master.xdc (github.com/Digilent/
# digilent-xdc). On the Arty S7, Pmods JA and JB are the high-speed pair
# (0 R, routed as differential pairs) and JC and JD are standard (200 R series
# resistors, shared with the shield header). The SPI loop carries 48 Mbit/s
# DDR MISO, so it gets JA and JB; the QSPI to the ESP runs at 8-24 MHz and
# takes JC.
#
# JUMPERS (all on this board, straight across, same pin numbers):
#
#   receiver  JA1 cs_n   ->  JB1  emulator cs
#             JA2 sclk   ->  JB2           sclk
#             JA3 mosi   ->  JB3           mosi
#             JA4 miso0  <-  JB4           miso0 (chip 0)
#             JA7 miso1  <-  JB7           miso1 (chip 1)
#
#   No ground jumper is needed for the loop: it never leaves the board.
#   Never jumper JA6/JA12 or JB6/JB12 (3V3) to anything.
#
# ESP32-S3 (pins_s3.h), from Pmod JC:
#
#   JC1  qspi_cs_n   -> GPIO10        JC7  qspi_dq[2]  -> GPIO14
#   JC2  qspi_sclk   -> GPIO12        JC8  qspi_dq[3]  -> GPIO9
#   JC3  qspi_dq[0]  -> GPIO11        JC9  esp_ready   <- GPIO21
#   JC4  qspi_dq[1]  -> GPIO13        JC5 or JC11 GND  -- ESP GND (required)
#
#   Not JC6/JC12 (3V3): each board is powered from its own USB cable.
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
# Receiver SPI master -- Pmod JA (high-speed)
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports { spi_cs_n  }] ;# JA1
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { spi_sclk  }] ;# JA2
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports { spi_mosi  }] ;# JA3
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { spi_miso0 }] ;# JA4
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports { spi_miso1 }] ;# JA7

set_property DRIVE 8    [get_ports { spi_cs_n spi_sclk spi_mosi }]
set_property SLEW  FAST [get_ports { spi_sclk }]

# ----------------------------------------------------------------------------
# Emulator SPI slave -- Pmod JB (high-speed), same positions as JA
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVCMOS33 } [get_ports { emu_cs_n  }] ;# JB1
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 } [get_ports { emu_sclk  }] ;# JB2
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { emu_mosi  }] ;# JB3
set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 } [get_ports { emu_miso0 }] ;# JB4
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports { emu_miso1 }] ;# JB7

set_property DRIVE 8    [get_ports { emu_miso0 emu_miso1 }]
set_property SLEW  FAST [get_ports { emu_miso0 emu_miso1 }]

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
# LEDs: LD2 receiver run, LD3 heartbeat, LD4 QSPI activity, LD5 emulator
# chip-select activity.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { led[0] }] ;# LD2
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports { led[1] }] ;# LD3
set_property -dict { PACKAGE_PIN E13 IOSTANDARD LVCMOS33 } [get_ports { led[2] }] ;# LD4
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33 } [get_ports { led[3] }] ;# LD5

# ----------------------------------------------------------------------------
# Timing.
# MISO is recovered by 4x oversampling with a runtime phase select, so there is
# no setup/hold relationship to declare on it -- PHASE_SELECT absorbs the pad
# and jumper delay. Bound the pad-to-capture path instead, so every capture
# flop sees MISO within a fraction of one 10.4 ns oversample tick. esp_ready
# goes into a two-flop synchroniser.
# ----------------------------------------------------------------------------
set_max_delay -datapath_only -from [get_ports { spi_miso0 spi_miso1 esp_ready }] \
    -to [get_clocks clk96] 8.000

# The emulator oversamples its whole bus through synchronisers of its own.
set_false_path -from [get_ports { emu_cs_n emu_sclk emu_mosi }]
set_false_path -from [get_ports { rst_btn uart_txd_in }]
set_false_path -to   [get_ports { emu_miso0 emu_miso1 led[*] }]

set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
