#!/usr/bin/env bash
# Compile + run the UART control interface testbench with Icarus Verilog.
# Exits non-zero unless the self-checking testbench reports a clean run, so it
# can be used directly in CI.
#
# Usage:  ./fpga/sim/run_ctrl_sim.sh     (run from anywhere; cd's to repo root)
set -euo pipefail

cd "$(dirname "$0")/../.."

iverilog -g2012 -o fpga/sim/tb_ctrl_regs.vvp -s tb_ctrl_regs \
    fpga/sim/tb_ctrl_regs.sv \
    fpga/rtl/rx/uart_rx.sv \
    fpga/rtl/rx/uart_tx.sv \
    fpga/rtl/rx/ctrl_regs.sv

# The testbench exits non-zero on failure, but its per-check output is the
# useful part, so capture it before letting the status decide.
set +e
out="$(vvp fpga/sim/tb_ctrl_regs.vvp 2>&1)"
rc=$?
set -e

echo "$out"

if [ "$rc" -eq 0 ] && echo "$out" | grep -q "ALL CHECKS PASSED"; then
    exit 0
else
    echo "::error::Simulation reported failures (or did not complete)."
    exit 1
fi
