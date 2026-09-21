#!/usr/bin/env python3
"""Emit the golden pattern as .mem files for the RTL testbench.

The RTL cannot import host/chain/pattern.py, so the host model is frozen into
vectors that tb_pattern_gen.sv reads with $readmemh. That keeps ONE definition
of the pattern: if the RTL drifts from the host, the FPGA testbench fails
rather than the mismatch waiting to be discovered at the bench.

Regenerate with `make vectors` whenever CONTRACTS.md section 3 changes.
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "host"))

from chain.pattern import PatternStream  # noqa: E402
from chain.contract import N_CH          # noqa: E402

N_SAMPLES = 8

for realistic, name in ((False, "check"), (True, "realistic")):
    stream = PatternStream(realistic=realistic, n_ch=N_CH)
    lines = []
    for _ in range(N_SAMPLES):
        for v in stream.next_sample():
            lines.append(f"{v & 0xFFFF:04x}")
    out = ROOT / "fpga" / "sim" / "vectors" / f"pattern_{name}.mem"
    out.write_text("\n".join(lines) + "\n")
    print(f"wrote {out.relative_to(ROOT)}  ({len(lines)} words)")
