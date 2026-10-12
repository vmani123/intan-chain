# ============================================================================
# emu_arty_s7_25.xdc -- rhd2164_top_se (vendored emulator) on a Digilent
# Arty S7-25 (xc7s25csga324-1).
#
# Pmod JA carries the Intan SPI in the same five positions as
# rx_arty_s7_50.xdc, directions mirrored, so a straight-through ribbon works.
#
# *** Do not bridge JA6/JA12 (VCC) between two separately powered boards. Five
# signal jumpers plus a ground, or a ribbon with the VCC conductors pulled. ***
# ============================================================================

set_property -dict { PACKAGE_PIN R2  IOSTANDARD SSTL135 } [get_ports { clk_100mhz }]
create_clock -name clk_100mhz -period 10.000 [get_ports { clk_100mhz }]

set_property -dict { PACKAGE_PIN G15 IOSTANDARD LVCMOS33 } [get_ports { rst_btn }]

set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports { cs_in     }] ;# JA1
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { sclk_in   }] ;# JA2
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports { mosi_in   }] ;# JA3
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { miso0_out }] ;# JA4
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports { miso1_out }] ;# JA7

set_property DRIVE 8    [get_ports { miso0_out miso1_out }]
set_property SLEW  FAST [get_ports { miso0_out miso1_out }]

# LEDs: MMCM locked, heartbeat, CS activity from the receiver.
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { led_locked    }]
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports { led_heartbeat }]
set_property -dict { PACKAGE_PIN E13 IOSTANDARD LVCMOS33 } [get_ports { led_activity  }]

# ----------------------------------------------------------------------------
# Timing. CS/SCLK/MOSI are asynchronous to the fast clock and go straight into
# spi_frontend's synchronizer chain; MISO is recovered on the receiver by
# oversampling with a runtime phase select. Neither side has a setup/hold
# relationship to declare.
# ----------------------------------------------------------------------------
set_false_path -from [get_ports { cs_in sclk_in mosi_in rst_btn }]
set_false_path -to   [get_ports { miso0_out miso1_out led_* }]

set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
