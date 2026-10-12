# ============================================================================
# program.tcl -- load bench bitstreams over JTAG (volatile; lost on power
# cycle). Boards are matched by FPGA part, so cable order cannot swap them:
#
#   xc7s50 -> receiver  (default fpga/bitstreams/rx_boolean.bit)
#   xc7s25 -> emulator  (default fpga/bitstreams/emu_arty_s7_25.bit)
#   xc7s25 -> chain     (fpga/bitstreams/chain_arty_s7_25.bit), when named
#
#   vivado -mode batch -nojournal -nolog -source fpga/vivado/program.tcl \
#          [-tclargs rx|emu|chain|all [path/to/file.bit]]
#
# With no argument, programs every board it recognises with its default.
# An explicit .bit path (e.g. rx_boolean_div6.bit) overrides the default for
# the one target named.
# ============================================================================

set root [file normalize [file join [file dirname [info script]] .. ..]]
set want [expr {[llength $argv] >= 1 ? [lindex $argv 0] : "all"}]
set bitdir [file join $root fpga bitstreams]

array set bits [list \
    xc7s50 [list rx  [file join $bitdir rx_boolean.bit]] \
    xc7s25 [list emu [file join $bitdir emu_arty_s7_25.bit]]]

# The single-board bench is the S7-25 too, so it is chosen by name.
if {$want eq "chain"} {
    set bits(xc7s25) [list chain [file join $bitdir chain_arty_s7_25.bit]]
}

if {[llength $argv] >= 2} {
    if {$want eq "all"} { error "name rx or emu when giving an explicit .bit" }
    # `make prog` runs from fpga/vivado/build/, so a relative path is taken
    # from the repo root, where the user typed it.
    set explicit [lindex $argv 1]
    if {[file pathtype $explicit] eq "relative"} {
        set explicit [file join $root $explicit]
    }
    set explicit [file normalize $explicit]
    foreach part [array names bits] {
        lassign $bits($part) name bit
        if {$name eq $want} { set bits($part) [list $name $explicit] }
    }
}

open_hw_manager
connect_hw_server -allow_non_jtag

set done 0
foreach t [get_hw_targets] {
    open_hw_target $t
    foreach d [get_hw_devices] {
        set part [get_property PART $d]
        if {![info exists bits($part)]} { continue }
        lassign $bits($part) name bit
        if {$want ne "all" && $want ne $name} { continue }
        if {![file exists $bit]} { error "$bit not found (build it first)" }
        current_hw_device $d
        set_property PROGRAM.FILE $bit $d
        program_hw_devices $d
        puts "PROGRAMMED $name ($part) with [file tail $bit] on $t"
        incr done
    }
    close_hw_target
}
if {$done == 0} { error "no matching board found for '$want'" }
