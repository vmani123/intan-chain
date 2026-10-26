# intan-chain -- everything runnable with no hardware is behind `make check`.
.PHONY: all check contract check-contract vectors sim host-test lint clean help bit bit-headstage bit-chain bit-rx bit-rx-boolean bit-emu prog

# Vivado must be on PATH (source <Vivado>/settings64.sh), or pass VIVADO=/path/to/vivado.
VIVADO ?= vivado
# Vivado drops .Xil/ and stray reports into its working directory, so it runs
# inside fpga/vivado/build/ (gitignored), never at the repo root.
VIVADO_DIR   = fpga/vivado/build
VIVADO_BATCH = mkdir -p $(VIVADO_DIR) && cd $(VIVADO_DIR) && $(VIVADO) -mode batch -nojournal -source ../build.tcl

all: check

help:
	@echo "make check          contract + host tests + every FPGA testbench (no hardware)"
	@echo "make contract       regenerate the C header and Python module from CONTRACTS.md"
	@echo "make check-contract fail if those generated files are stale"
	@echo "make vectors        regenerate the golden pattern vectors for the RTL testbench"
	@echo "make sim            FPGA testbenches only (Icarus Verilog)"
	@echo "make host-test      host Python tests only (pytest)"
	@echo "make bit-headstage  receiver -> real RHD2164s on Pmod JA, Arty S7-25 -> fpga/bitstreams/headstage_arty_s7_25[_divN].bit"
	@echo "make bit-chain      single-board bench, Arty S7-25 -> fpga/bitstreams/chain_arty_s7_25[_divN].bit"
	@echo "                    QSPI_DIV=6 make bit-chain builds with an 8 MHz QSPI clock"
	@echo "make bit            bench bitstreams (Vivado): receiver on Boolean + emulator on Arty S7-25"
	@echo "make bit-rx-boolean receiver, RealDigital Boolean -> fpga/bitstreams/rx_boolean[_divN].bit"
	@echo "make bit-rx         receiver, Arty S7-50 -> fpga/bitstreams/rx_arty_s7_50[_divN].bit"
	@echo "make bit-emu        emulator bitstream -> fpga/bitstreams/emu_arty_s7_25.bit"
	@echo "make prog           load both over JTAG, matched to boards by part (volatile)"
	@echo "                    PROG=\"rx fpga/bitstreams/rx_boolean_div6.bit\" picks one target and file"
	@echo "make clean          remove simulation build products"

# The whole no-hardware gate, in dependency order: the contract first, because
# a stale generated file makes every other failure a red herring.
check: check-contract host-test sim
	@echo
	@echo "=== all no-hardware checks passed ==="

contract:
	python3 tools/gen_contract.py

check-contract:
	python3 tools/gen_contract.py --check

vectors:
	python3 tools/gen_pattern_vectors.py

sim:
	./fpga/sim/run_sim.sh

host-test:
	cd host && python3 -m pytest tests -q

bit: bit-rx-boolean bit-emu

bit-headstage:
	$(VIVADO_BATCH) -log ../headstage.log -tclargs headstage

bit-chain:
	$(VIVADO_BATCH) -log ../chain.log -tclargs chain

bit-rx:
	$(VIVADO_BATCH) -log ../rx.log -tclargs rx

bit-rx-boolean:
	$(VIVADO_BATCH) -log ../rx_boolean.log -tclargs rx_boolean

bit-emu:
	$(VIVADO_BATCH) -log ../emu.log -tclargs emu

prog:
	mkdir -p $(VIVADO_DIR) && cd $(VIVADO_DIR) && $(VIVADO) -mode batch -nojournal -nolog -source ../program.tcl $(if $(PROG),-tclargs $(PROG))

# One recipe line, so "not installed" really does skip: an `exit 0` on a line
# of its own only ends that line's shell.
lint:
	@if ! command -v verilator >/dev/null 2>&1; then echo "verilator not installed; skipping"; \
	else verilator --lint-only -Wall -Wno-DECLFILENAME \
	    -Ifpga/rtl/rx -Ifpga/rtl/gen \
	    fpga/rtl/rx/rx_top.sv --top-module rx_top; fi

clean:
	rm -f fpga/sim/*.vvp fpga/sim/*.vcd
	find . -name '__pycache__' -type d -prune -exec rm -rf {} +
	find . -name '.pytest_cache' -type d -prune -exec rm -rf {} +
