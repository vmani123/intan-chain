#!/usr/bin/env bash
# Compile and run every FPGA testbench with Icarus Verilog.
#
# Each testbench is self-checking and prints ALL CHECKS PASSED on a clean run;
# anything else -- a failure, a timeout, a crash -- makes this script exit
# non-zero, so it can be used directly in CI.
#
# Usage:  ./fpga/sim/run_sim.sh     (run from anywhere; cd's to repo root)
set -euo pipefail

cd "$(dirname "$0")/../.."

IV="iverilog -g2012"

RTL_RX="fpga/rtl/rx/spi_master.sv
        fpga/rtl/rx/miso_phase_select.sv
        fpga/rtl/rx/frame_builder.sv
        fpga/rtl/rx/dgram_mem.sv
        fpga/rtl/rx/slot_fifo.sv
        fpga/rtl/rx/qspi_master.sv
        fpga/rtl/rx/ctrl_regs.sv
        fpga/rtl/rx/uart_rx.sv
        fpga/rtl/rx/uart_tx.sv"

EMU="fpga/upstream/rhd2164-emulator-rtl/rhd2164_emulator.sv
     fpga/upstream/rhd2164-emulator-rtl/spi_frontend.sv
     fpga/upstream/rhd2164-emulator-rtl/command_decoder.sv
     fpga/upstream/rhd2164-emulator-rtl/register_file.sv
     fpga/upstream/rhd2164-emulator-rtl/ddr_miso.sv"

failed=0
names=()
results=()

run_tb () {
    local top="$1"; shift
    echo
    echo "############################################################"
    echo "## $top"
    echo "############################################################"

    local vvp="fpga/sim/${top}.vvp"
    local rc=0
    local out=""

    if ! out="$($IV -o "$vvp" -s "$top" "$@" 2>&1)"; then
        echo "$out"
        echo "::error::$top failed to compile."
        names+=("$top"); results+=("COMPILE-FAIL"); failed=1
        return
    fi
    [ -n "$out" ] && echo "$out"

    set +e
    out="$(vvp "$vvp" 2>&1)"
    rc=$?
    set -e
    echo "$out"

    if [ "$rc" -eq 0 ] && echo "$out" | grep -q "ALL CHECKS PASSED"; then
        names+=("$top"); results+=("PASS")
    else
        echo "::error::$top reported failures (or did not complete)."
        names+=("$top"); results+=("FAIL"); failed=1
    fi
}

run_tb tb_pattern_gen \
    fpga/sim/tb_pattern_gen.sv \
    fpga/rtl/gen/pattern_gen.sv

run_tb tb_ctrl_regs \
    fpga/sim/tb_ctrl_regs.sv \
    fpga/rtl/rx/uart_rx.sv fpga/rtl/rx/uart_tx.sv fpga/rtl/rx/ctrl_regs.sv

run_tb tb_spi_master \
    fpga/sim/tb_spi_master.sv \
    fpga/rtl/rx/spi_master.sv fpga/rtl/rx/miso_phase_select.sv \
    $EMU

run_tb tb_frame_builder \
    fpga/sim/tb_frame_builder.sv \
    fpga/rtl/rx/frame_builder.sv fpga/rtl/rx/dgram_mem.sv

run_tb tb_rx_top \
    fpga/sim/tb_rx_top.sv fpga/sim/qspi_hd_monitor.sv \
    fpga/rtl/rx/rx_top.sv fpga/rtl/gen/pattern_gen.sv \
    $RTL_RX $EMU

# The whole single-board bench: board top, MMCM, resets, jumpers, ESP model.
run_tb tb_chain_arty_s7_25 \
    fpga/sim/tb_chain_arty_s7_25.sv fpga/sim/qspi_hd_monitor.sv \
    fpga/sim/unisim_stubs.sv fpga/boards/chain_arty_s7_25_top.sv \
    fpga/rtl/rx/rx_top.sv fpga/rtl/gen/pattern_gen.sv \
    $RTL_RX $EMU

echo
echo "############################################################"
for i in "${!names[@]}"; do
    printf '## %-20s %s\n' "${names[$i]}" "${results[$i]}"
done
echo "############################################################"

if [ "$failed" -eq 0 ]; then
    echo "SUMMARY: ${#names[@]}/${#names[@]} testbenches passed."
    exit 0
else
    echo "SUMMARY: at least one testbench failed."
    exit 1
fi
