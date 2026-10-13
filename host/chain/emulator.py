"""The RHD2164 emulator's playback memory: the golden reference for real-SPI runs.

When the receiver reads the emulator over the SPI jumpers (MODE bit 2 clear),
each frame carries one playback sample of all 128 channels, straight from
fpga/upstream/rhd2164-emulator-mem/. The emulator plays position p and advances
it once per sample, wrapping at SAMPLES, so within a run (position - seq) mod
SAMPLES is a single constant; metrics finds it the way it finds the pattern
offset, then holds every frame to it.

Channel order is CONTRACTS.md section 1: chip 0 module A 0-31, chip 0 B 32-63,
chip 1 A 64-95, chip 1 B 96-127. Each file is sample-major: addr = sample*32 + ch.
"""
import pathlib

from .contract import N_CH
from .pattern import to_int16

MEM_DIR = pathlib.Path(__file__).resolve().parents[2] / "fpga" / "upstream" / "rhd2164-emulator-mem"
SAMPLES = 256
FILES = ("chip0_A", "chip0_B", "chip1_A", "chip1_B")   # 32 channels each, in channel order


class Playback:
    def __init__(self, mem_dir=MEM_DIR):
        mems = []
        for name in FILES:
            text = (pathlib.Path(mem_dir) / f"{name}.mem").read_text()
            words = [int(l, 16) for l in text.split("\n") if l.strip() and not l.startswith("//")]
            if len(words) != SAMPLES * 32:
                raise ValueError(f"{name}.mem holds {len(words)} words, expected {SAMPLES * 32}")
            mems.append(words)
        self._samples = [[to_int16(mems[ch // 32][p * 32 + ch % 32]) for ch in range(N_CH)]
                         for p in range(SAMPLES)]
        self._by_ch0: dict[int, list[int]] = {}
        for p, s in enumerate(self._samples):
            self._by_ch0.setdefault(s[0], []).append(p)

    def expected(self, position: int) -> list[int]:
        """The 128 values the emulator plays at `position` (mod SAMPLES)."""
        return self._samples[position % SAMPLES]

    def locate(self, samples) -> int | None:
        """The playback position whose 128 values all equal `samples`, or None."""
        s = list(samples)
        for p in self._by_ch0.get(s[0], ()):
            if self._samples[p] == s:
                return p
        return None
