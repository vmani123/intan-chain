# intan-chain -- everything runnable with no hardware is behind `make check`.
.PHONY: all check contract check-contract vectors sim host-test lint clean help

all: check

help:
	@echo "make check          contract + host tests + every FPGA testbench (no hardware)"
	@echo "make contract       regenerate the C header and Python module from CONTRACTS.md"
	@echo "make check-contract fail if those generated files are stale"
	@echo "make vectors        regenerate the golden pattern vectors for the RTL testbench"
	@echo "make sim            FPGA testbenches only (Icarus Verilog)"
	@echo "make host-test      host Python tests only (pytest)"
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

lint:
	@command -v verilator >/dev/null 2>&1 || { echo "verilator not installed; skipping"; exit 0; }
	verilator --lint-only -Wall -Wno-DECLFILENAME \
	    -Ifpga/rtl/rx -Ifpga/rtl/gen \
	    fpga/rtl/rx/rx_top.sv --top-module rx_top

clean:
	rm -f fpga/sim/*.vvp fpga/sim/*.vcd
	find . -name '__pycache__' -type d -prune -exec rm -rf {} +
	find . -name '.pytest_cache' -type d -prune -exec rm -rf {} +
