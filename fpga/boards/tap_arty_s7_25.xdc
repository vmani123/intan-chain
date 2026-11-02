# ============================================================================
# tap_arty_s7_25.xdc -- tap_arty_s7_25_top on a Digilent Arty S7-25
# (xc7s25csga324-1): a passive tap on an Intan SPI cable (five LVDS pairs in
# on Pmod JA / JB), QSPI to the ESP on Pmod JC.
#
# Pin numbers from Digilent's Arty-S7-25-Master.xdc, the same as
# headstage_arty_s7_25.xdc. JA and JB are the high-speed Pmods (0 R, routed
# as differential pairs): JA1/2, JA3/4, JA7/8, JA9/10 and JB1/2 are each one
# P/N pair of bank 14. Wiring from the C3430 adapter: docs/intan-tap-check.md.
#
# LVDS_25 INPUTS IN A 3.3 V BANK
#   The tap pins sit in bank 14, which the Arty powers at VCCO = 3.3 V.
#   LVDS_25 receivers are allowed in such a bank as long as the internal 100 R
#   termination is off (AMD AR 43989; UG471 on LVDS_25 inputs with VCCO other
#   than 2.5 V): DIFF_TERM FALSE below and in the IBUFDS. The cable is
#   terminated at both ends already, so the tap must not terminate it again.
#   Only Vivado's DRC can confirm the bank rules; build with `make bit-tap`.
#
#   C3430 Y   signal      Arty   FPGA            C3430 Y   signal      Arty   FPGA
#   B1        CS+     ->  JA1    L17             B4        MISO1+  ->  JA9    M18
#   T1        CS-     ->  JA2    L18             T4        MISO1-  ->  JA10   N18
#   B2        SCLK+   ->  JA3    M14             B5        MISO2+  ->  JB1    P17
#   T2        SCLK-   ->  JA4    N14             T5        MISO2-  ->  JB2    P18
#   B3        MOSI+   ->  JA7    M16             T6        ground  ->  JA5 / JA11 GND
#   T3        MOSI-   ->  JA8    M17
#
#   Never connect cable power (C3430 B6) or a Pmod 3V3 pin (6 / 12).
#
# ESP (pins_s3.h), from Pmod JC, unchanged:
#
#   JC1  qspi_cs_n   -> GPIO10        JC7  qspi_dq[2]  -> GPIO14
#   JC2  qspi_sclk   -> GPIO12        JC8  qspi_dq[3]  -> GPIO9
#   JC3  qspi_dq[0]  -> GPIO11        JC9  esp_ready   <- GPIO21
#   JC4  qspi_dq[1]  -> GPIO13        JC5 or JC11 GND  -- ESP GND (required)
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
# The tap: five LVDS_25 receivers, termination off. Both legs of each pair
# are pinned so the file reads like the wiring table; they are the pair
# Vivado would pick from the P pin anyway.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_cs_p    }] ;# JA1
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_cs_n    }] ;# JA2
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_sclk_p  }] ;# JA3
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_sclk_n  }] ;# JA4
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_mosi_p  }] ;# JA7
set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_mosi_n  }] ;# JA8
set_property -dict { PACKAGE_PIN M18 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_miso0_p }] ;# JA9
set_property -dict { PACKAGE_PIN N18 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_miso0_n }] ;# JA10
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_miso1_p }] ;# JB1
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVDS_25 DIFF_TERM FALSE } [get_ports { tap_miso1_n }] ;# JB2

# ----------------------------------------------------------------------------
# QSPI to the ESP -- Pmod JC (standard, 200 R series)
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
# LEDs: LD2 receiver run, LD3 heartbeat, LD4 QSPI activity, LD5 tap hears
# clean cycles (16 + 16 SCLK edges) with no recent framing or known-answer error.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { led[0] }] ;# LD2
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports { led[1] }] ;# LD3
set_property -dict { PACKAGE_PIN E13 IOSTANDARD LVCMOS33 } [get_ports { led[2] }] ;# LD4
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33 } [get_ports { led[3] }] ;# LD5

# ----------------------------------------------------------------------------
# Timing.
# The tap lines are asynchronous to every clock here: each is caught by a pad
# flop and then a two-flop synchroniser, and TAP_DELAY absorbs cable and chip
# delay at run time, so there is no setup/hold relationship to declare.
# esp_ready goes into a two-flop synchroniser.
#
# The self-test path crosses from clk192 (the emulators' MISO) into the tap's
# synchronisers on clk96, and the loopback from clk96 into the emulators. Both
# ends synchronise or oversample, so the paths only need a bound, as in the
# headstage build.
# ----------------------------------------------------------------------------
set_false_path -from [get_ports { tap_*_p tap_*_n esp_ready rst_btn uart_txd_in }]
set_false_path -to   [get_ports { led[*] }]

set_max_delay -datapath_only -from [get_clocks clk96]  -to [get_clocks clk192] 5.000
set_max_delay -datapath_only -from [get_clocks clk192] -to [get_clocks clk96]  5.000

set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
