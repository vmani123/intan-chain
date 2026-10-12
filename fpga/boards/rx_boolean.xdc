# ============================================================================
# rx_boolean.xdc -- rx_arty_s7_top on a RealDigital Boolean board
# (xc7s50csga324-1). The top's ports are board-neutral, so only this file
# differs from rx_arty_s7_50.xdc.
#
# Pins are from the Boolean schematic (Rev V2I1, sheets 3, 5, 9, 11) and
# RealDigital's master XDC. The expansion header J1 carries two Pmods, PMOD_A
# (nets JAx) and PMOD_B (nets JBx), every signal through a 200 R series
# resistor. Bank 15, VCCO 3.3 V.
#
#   signal       net      ball   J1 pin   expected Pmod position
#   ----------   ------   ----   ------   ----------------------
#   spi_cs_n     JA1_P    B13     1       PMOD_A 1
#   spi_sclk     JA1_N    A13     2       PMOD_A 2
#   spi_mosi     JA2_P    B14     3       PMOD_A 3
#   spi_miso0    JA2_N    A14     4       PMOD_A 4
#   spi_miso1    JA3_P    E12    30       PMOD_A 7
#   qspi_cs_n    JB1_P    B15    10       PMOD_B 1
#   qspi_sclk    JB1_N    A15    11       PMOD_B 2
#   qspi_dq[0]   JB2_P    B17    12       PMOD_B 3
#   qspi_dq[1]   JB2_N    A17    13       PMOD_B 4
#   qspi_dq[2]   JB3_P    B16    21       PMOD_B 7
#   qspi_dq[3]   JB3_N    A16    20       PMOD_B 8
#   esp_ready    JB4_P    C17    19       PMOD_B 9
#
# J1 pin numbers are the schematic's (1-15 down one side, 30-16 down the
# other, pin p opposite pin 31-p). The Pmod positions assume the usual
# 1-6 / 7-12 rows.
#
# This map follows RealDigital's own schematic, which is the authority here.
# The community board file github.com/proto17/RealDigital_BooleanBoard
# (part0_pins.xml) agrees on PMOD_A but differs on PMOD_B: it swaps JB2 and
# JB3 into each other's positions. Treat that file as the outlier.
#
# On the schematic, the JB4_P net (esp_ready) touches only its 200 R, the
# ESD array and ball C17, so nothing on the board drives it. If esp_ready
# reads high with the PULLDOWN and no ESP attached, look off-board. Test by
# putting 1k from J1.19 to J1.17 (GND): STALLS should climb and DGRAMS stop.
#
# *** Do not bridge VCC between two separately powered boards. Signal jumpers
# plus at least one ground. ***
# ============================================================================

set_property -dict { PACKAGE_PIN F14 IOSTANDARD LVCMOS33 } [get_ports { clk_100mhz }]
create_clock -name clk_100mhz -period 10.000 [get_ports { clk_100mhz }]
create_generated_clock -name clk96 [get_pins u_mmcm/CLKOUT0]

# BTN0: 10k pulldown, pressed = 3.3 V, so active high as the top expects.
set_property -dict { PACKAGE_PIN J2  IOSTANDARD LVCMOS33 } [get_ports { rst_btn }]

# ----------------------------------------------------------------------------
# Intan SPI -- PMOD_A, same positions as the emulator's Arty JA.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN B13 IOSTANDARD LVCMOS33 } [get_ports { spi_cs_n  }] ;# JA1_P
set_property -dict { PACKAGE_PIN A13 IOSTANDARD LVCMOS33 } [get_ports { spi_sclk  }] ;# JA1_N
set_property -dict { PACKAGE_PIN B14 IOSTANDARD LVCMOS33 } [get_ports { spi_mosi  }] ;# JA2_P
set_property -dict { PACKAGE_PIN A14 IOSTANDARD LVCMOS33 } [get_ports { spi_miso0 }] ;# JA2_N
set_property -dict { PACKAGE_PIN E12 IOSTANDARD LVCMOS33 } [get_ports { spi_miso1 }] ;# JA3_P

set_property DRIVE 8    [get_ports { spi_cs_n spi_sclk spi_mosi }]
set_property SLEW  FAST [get_ports { spi_sclk }]

# ----------------------------------------------------------------------------
# QSPI to the ESP32-S3 -- PMOD_B, same positions as the Arty JB.
# esp_ready is pulled down so an unconnected ESP reads "not ready": STALLS
# then climbs instead of the master clocking into nothing.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN B15 IOSTANDARD LVCMOS33 } [get_ports { qspi_cs_n  }] ;# JB1_P
set_property -dict { PACKAGE_PIN A15 IOSTANDARD LVCMOS33 } [get_ports { qspi_sclk  }] ;# JB1_N
set_property -dict { PACKAGE_PIN B17 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[0] }] ;# JB2_P
set_property -dict { PACKAGE_PIN A17 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[1] }] ;# JB2_N
set_property -dict { PACKAGE_PIN B16 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[2] }] ;# JB3_P
set_property -dict { PACKAGE_PIN A16 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[3] }] ;# JB3_N
set_property -dict { PACKAGE_PIN C17 IOSTANDARD LVCMOS33 PULLDOWN TRUE } [get_ports { esp_ready }] ;# JB4_P

set_property DRIVE 8    [get_ports { qspi_cs_n qspi_sclk qspi_dq[*] }]
set_property SLEW  FAST [get_ports { qspi_sclk qspi_dq[*] }]

# ----------------------------------------------------------------------------
# Control interface -- onboard FT2232 channel B. Named from the bridge's side,
# as on the Arty: UART_TXD_IN is the FPGA's RECEIVE, UART_RXD_OUT its TRANSMIT.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { uart_txd_in  }] ;# PC  -> FPGA
set_property -dict { PACKAGE_PIN U11 IOSTANDARD LVCMOS33 } [get_ports { uart_rxd_out }] ;# FPGA -> PC

# LD0..LD2: run, heartbeat, activity.
set_property -dict { PACKAGE_PIN G1 IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN G2 IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN F1 IOSTANDARD LVCMOS33 } [get_ports { led[2] }]

# ----------------------------------------------------------------------------
# Timing -- identical reasoning to rx_arty_s7_50.xdc.
# ----------------------------------------------------------------------------
set_max_delay -datapath_only -from [get_ports { spi_miso0 spi_miso1 esp_ready }] \
    -to [get_clocks clk96] 8.000

set_false_path -from [get_ports { rst_btn uart_txd_in }]
set_false_path -to   [get_ports { led[*] }]

set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
