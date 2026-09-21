# ============================================================================
# rx_arty_s7_50.xdc -- rx_arty_s7_top on a Digilent Arty S7-50 (xc7s50csga324-1)
#
# Pin numbers from Digilent's master XDC (github.com/Digilent/digilent-xdc).
# The S7-25 and S7-50 have identical pinouts, so this file also works on the
# S7-25 if you ever swap the boards; only the Vivado part number changes.
# ============================================================================

# ----------------------------------------------------------------------------
# Clock. On the Arty S7 the 100 MHz oscillator lands on R2, inside the DDR3
# bank, so its IOSTANDARD is SSTL135 and NOT LVCMOS33. This catches everyone.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN R2  IOSTANDARD SSTL135 } [get_ports { clk_100mhz }]
create_clock -name clk_100mhz -period 10.000 [get_ports { clk_100mhz }]
create_generated_clock -name clk96 [get_pins u_mmcm/CLKOUT0]

set_property -dict { PACKAGE_PIN G15 IOSTANDARD LVCMOS33 } [get_ports { rst_btn }]

# ----------------------------------------------------------------------------
# Intan SPI -- Pmod JA. Same five positions as the emulator board's JA with the
# directions reversed, so a straight-through ribbon works.
#
# *** Do not bridge JA6/JA12 (VCC) between two separately powered boards. Five
# signal jumpers plus a ground, or a ribbon with the VCC conductors pulled. ***
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports { spi_cs_n   }] ;# JA1
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { spi_sclk   }] ;# JA2
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports { spi_mosi   }] ;# JA3
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { spi_miso0  }] ;# JA4
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports { spi_miso1  }] ;# JA7

set_property DRIVE 8    [get_ports { spi_cs_n spi_sclk spi_mosi }]
set_property SLEW  FAST [get_ports { spi_sclk }]

# ----------------------------------------------------------------------------
# QSPI to the ESP32-S3 -- Pmod JB.
# esp_ready is the slave-to-master flow control line. It is not optional: without
# it the master overruns the slave at exactly the load being measured.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVCMOS33 } [get_ports { qspi_cs_n  }] ;# JB1
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 } [get_ports { qspi_sclk  }] ;# JB2
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[0] }] ;# JB3
set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[1] }] ;# JB4
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[2] }] ;# JB7
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports { qspi_dq[3] }] ;# JB8
set_property -dict { PACKAGE_PIN N15 IOSTANDARD LVCMOS33 } [get_ports { esp_ready  }] ;# JB9

set_property DRIVE 8    [get_ports { qspi_cs_n qspi_sclk qspi_dq[*] }]
set_property SLEW  FAST [get_ports { qspi_sclk qspi_dq[*] }]

# ----------------------------------------------------------------------------
# Control interface -- onboard USB-UART, same cable as JTAG.
# Digilent names these from the USB bridge's point of view, so uart_rxd_out is
# the FPGA's TRANSMIT and uart_txd_in is the FPGA's RECEIVE. Wire them the other
# way round and you get silence with no error anywhere.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { uart_txd_in  }] ;# PC  -> FPGA
set_property -dict { PACKAGE_PIN R12 IOSTANDARD LVCMOS33 } [get_ports { uart_rxd_out }] ;# FPGA -> PC

# ----------------------------------------------------------------------------
# LEDs: run, heartbeat, activity.
# ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN E13 IOSTANDARD LVCMOS33 } [get_ports { led[2] }]

# ----------------------------------------------------------------------------
# Timing.
# MISO is recovered by 4x oversampling with a runtime phase select, not by a
# source-synchronous capture, so there is no setup/hold relationship to declare
# on it -- the phase register absorbs the cable delay. Bound the path instead.
# ----------------------------------------------------------------------------
set_max_delay -datapath_only -from [get_ports { spi_miso0 spi_miso1 esp_ready }] \
    -to [get_clocks clk96] 8.000

set_false_path -from [get_ports { rst_btn uart_txd_in }]
set_false_path -to   [get_ports { led[*] }]

set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
