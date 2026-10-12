# ============================================================================
# build.tcl -- non-project Vivado build for both bench bitstreams.
#
#   vivado -mode batch -nojournal -source fpga/vivado/build.tcl -tclargs <target>
#
# or `make bit-chain` / `make bit-rx-boolean` / `make bit-rx` / `make bit-emu`
# from the repo root.
#
#   chain       the single-board bench on the Arty S7-25 (xc7s25): emulator on
#               Pmod JB, receiver on Pmod JA, jumpered; QSPI to the ESP on JC
#   rx_boolean  receiver (SPI in, QSPI out) on the RealDigital Boolean (xc7s50)
#   rx          the same receiver on a Digilent Arty S7-50
#   emu         rhd2164_top_se on the Arty S7-25: two emulated RHD2164s on Pmod JA
#
# Reports land in fpga/vivado/build/<target>/, the bitstream in fpga/bitstreams/.
# The build fails (non-zero exit) on negative setup or hold slack: a bitstream
# that misses timing would produce bit errors that look like a link problem.
# ============================================================================

if {[llength $argv] != 1} {
    error "usage: -tclargs chain|rx|rx_boolean|emu"
}
set target [lindex $argv 0]
set root   [file normalize [file join [file dirname [info script]] .. ..]]
set out    [file join $root fpga vivado build $target]
file mkdir $out
file mkdir [file join $root fpga bitstreams]

# A $readmemh file that cannot be found is only a warning by default, and the
# emulator would then play back all zeros. Make it fatal.
set_msg_config -id {Synth 8-4445} -new_severity ERROR
set_msg_config -id {Synth 8-2898} -new_severity ERROR

switch -- $target {
    rx - rx_boolean {
        # Same top on both xc7s50 boards; only the pin file differs.
        set part xc7s50csga324-1
        set top  rx_arty_s7_top
        if {$target eq "rx"} {
            set base rx_arty_s7_50
            set xdc  rx_arty_s7_50.xdc
        } else {
            set base rx_boolean
            set xdc  rx_boolean.xdc
        }
        read_verilog -sv [lsort [glob [file join $root fpga rtl rx *.sv]]]
        read_verilog -sv [file join $root fpga rtl gen pattern_gen.sv]
        read_verilog -sv [file join $root fpga boards rx_arty_s7_top.sv]
        read_xdc         [file join $root fpga boards $xdc]
        set generics {}
        # QSPI_DIV=6 make bit-rx-boolean -> SCLK = 96 / (2 * 6) = 8 MHz.
        # The ID register cannot tell SCLK rates apart, so a non-default
        # divider goes into the file name: rx_boolean_div6.bit.
        set bit $base.bit
        if {[info exists ::env(QSPI_DIV)]} {
            lappend generics "QSPI_DIV=$::env(QSPI_DIV)"
            if {$::env(QSPI_DIV) != 2} { set bit ${base}_div$::env(QSPI_DIV).bit }
        }
    }
    chain {
        set part xc7s25csga324-1
        set top  chain_arty_s7_25_top
        read_verilog -sv [lsort [glob [file join $root fpga rtl rx *.sv]]]
        read_verilog -sv [file join $root fpga rtl gen pattern_gen.sv]
        set up [file join $root fpga upstream rhd2164-emulator-rtl]
        foreach f {rhd2164_emulator spi_frontend command_decoder register_file
                   ddr_miso} {
            read_verilog -sv [file join $up $f.sv]
        }
        read_verilog -sv [file join $root fpga boards chain_arty_s7_25_top.sv]
        read_xdc         [file join $root fpga boards chain_arty_s7_25.xdc]
        # Absolute paths, so $readmemh does not depend on the run directory.
        set mem [file join $root fpga upstream rhd2164-emulator-mem]
        set generics [list \
            "MEM_C0A=\"[file join $mem chip0_A.mem]\"" \
            "MEM_C0B=\"[file join $mem chip0_B.mem]\"" \
            "MEM_C1A=\"[file join $mem chip1_A.mem]\"" \
            "MEM_C1B=\"[file join $mem chip1_B.mem]\""]
        # Same divider naming as the rx targets: chain_arty_s7_25_div6.bit.
        set bit chain_arty_s7_25.bit
        if {[info exists ::env(QSPI_DIV)]} {
            lappend generics "QSPI_DIV=$::env(QSPI_DIV)"
            if {$::env(QSPI_DIV) != 2} { set bit chain_arty_s7_25_div$::env(QSPI_DIV).bit }
        }
    }
    emu {
        set part xc7s25csga324-1
        set top  rhd2164_top_se
        set bit  emu_arty_s7_25.bit
        set up   [file join $root fpga upstream rhd2164-emulator-rtl]
        foreach f {rhd2164_emulator spi_frontend command_decoder register_file
                   ddr_miso rhd2164_top_se} {
            read_verilog -sv [file join $up $f.sv]
        }
        read_xdc [file join $root fpga boards emu_arty_s7_25.xdc]
        # Absolute paths, so $readmemh does not depend on the run directory.
        set mem [file join $root fpga upstream rhd2164-emulator-mem]
        set generics [list \
            "MEM_C0A=\"[file join $mem chip0_A.mem]\"" \
            "MEM_C0B=\"[file join $mem chip0_B.mem]\"" \
            "MEM_C1A=\"[file join $mem chip1_A.mem]\"" \
            "MEM_C1B=\"[file join $mem chip1_B.mem]\""]
        # Oversampling clock = 800 MHz / FAST_CLK_DIV. The upstream default of
        # 2.0 (400 MHz) misses timing on the -1 S7-25 by ~1.5 ns (worst path
        # is calib_cnt -> register-file CE, 3.7 ns), so build at 4.0 = 200 MHz,
        # ~8x oversampling of the 24 MHz SCLK. Override with EMU_FAST_CLK_DIV.
        set fdiv 4.000
        if {[info exists ::env(EMU_FAST_CLK_DIV)]} {
            set fdiv $::env(EMU_FAST_CLK_DIV)
        }
        lappend generics "FAST_CLK_DIV=$fdiv"
    }
    default {
        error "unknown target '$target' (expected chain, rx, rx_boolean or emu)"
    }
}

set gargs {}
foreach g $generics { lappend gargs -generic $g }

synth_design -top $top -part $part {*}$gargs
report_utilization -file [file join $out post_synth_util.rpt]

opt_design
place_design
phys_opt_design
route_design

report_timing_summary -max_paths 20 -file [file join $out timing.rpt]
report_utilization                   -file [file join $out util.rpt]
report_drc                           -file [file join $out drc.rpt]
report_io                            -file [file join $out io.rpt]

set wns [get_property SLACK [get_timing_paths -delay_type max]]
set whs [get_property SLACK [get_timing_paths -delay_type min]]
puts "TIMING $target: WNS = $wns ns, WHS = $whs ns"
if {$wns eq "" || $wns < 0 || $whs < 0} {
    error "timing not met for $target (see [file join $out timing.rpt])"
}

write_bitstream -force [file join $root fpga bitstreams $bit]
puts "BITSTREAM [file join $root fpga bitstreams $bit]"
