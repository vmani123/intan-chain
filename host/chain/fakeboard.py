"""The headstage FPGA build and two RHD2164s, in memory, with faults on demand.

FakeBoard is a pyserial-shaped object that speaks CONTRACTS.md section 4
byte for byte, including the diagnostic list, the counters, MISO_STAT and the
streaming link check. Behind it sit two behavioural RHD2164 models (two-cycle
pipeline, ROM, RAM registers with 18-21 readable only on module B, WRITE echo,
the CALIBRATE window, CONVERT from a function) and a model of each MISO wire
that turns the chip's two result words into the 78 raw taps the FPGA records,
from a delay in fast clocks. So the host's decode, phase scan and every
classification in bring-up are exercised end to end against waveforms, not
against canned words.

The wire model follows the RHD2164 emulator this repo already runs against
(fpga/upstream/rhd2164-emulator-rtl/ddr_miso.sv): with zero delay, half-bit
slot s (s even: module A bit 15 - s/2; s odd: module B) starts on SPI tick
2s + 2 and lasts two ticks, the last B bit holding past CS rising. Tap k is
the level at the start of tick k + 2. A round-trip delay of D fast clocks at
SPI_DIV n therefore opens the window at PHASE_SELECT D / (n + 1), two phases
wide; the bench's emulator window (5-6 at SPI_DIV 0) is D = 5.

It is also what `python3 -m chain.bringup --fake` runs against, so every
failure report can be seen without hardware.
"""
import math
import random
from dataclasses import dataclass, field

from . import contract as C
from . import rhd2164 as R
from .ctrl import WRITABLE

U32 = 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Faults
# ---------------------------------------------------------------------------

@dataclass
class ChipFaults:
    """What can be wrong between the FPGA pins and one chip."""
    line: str | None = None      # None, 'stuck_low', 'stuck_high', 'absent', 'inverted'
    delay_fc: float = 5.0        # round-trip MISO delay, fast clocks (10.4 ns each)
    ab_swap: bool = False        # chip emits module B in A's slots and vice versa
    mosi_xor: int = 0            # bits flipped in every command the chip receives
    mosi_invert: bool = False    # MOSI pair reversed: the chip receives ~command
    mosi_stuck: int | None = None  # MOSI never toggles at the chip: every command is this
    adc_frozen: int | None = None  # unconfigured ADC: every CONVERT returns this code
    dead_module: str = ""        # 'A', 'B' or 'AB': CONVERT results stuck for that half
    chip_id: int = R.CHIP_ID_RHD2164
    link_flaky: float = 0.0      # probability per streamed sweep of a corrupt READ(59)
    drop_writes: bool = False    # WRITE echoes but the register keeps its old value
    b_echo_xor: int = 0          # module B's WRITE echo differs from A's by this
    calib_len: int = R.CALIBRATE_DUMMIES  # commands the calibration window consumes


@dataclass
class Faults:
    chips: list = field(default_factory=lambda: [ChipFaults(), ChipFaults()])
    silent: bool = False         # board never answers (not programmed / wrong port)
    garbled: bool = False        # board answers noise (wrong baud)
    board_id: int = C.ID_EXPECTED
    caps_taps: int = C.CAP_TAPS
    emulator: bool = True        # CAPS bit8: on-die emulator present
    selftest_broken: bool = False  # loopback path returns nothing useful (FPGA fault)
    decode_offset: int = 0       # FPGA decodes DIAG_RES at PHASE_SELECT + this (RTL bug)
    spi_dead: bool = False       # sequencer never runs: no CS cycles, diag never done
    stream_dead: bool = False    # the diagnostic list runs but streaming sweeps never start
    esp_present: bool = False    # an ESP drains the QSPI FIFO while streaming
    edge_fc: float = 0.0         # samples this close to a MISO transition read random
    emu_delay_fc: float = 3.0    # on-die emulator loop delay


FAULTS = {
    # name -> how to make it. Used by `bringup --fake FAULT` and the tests.
    "none": lambda f: None,
    "chip1_absent": lambda f: setattr(f.chips[1], "line", "absent"),
    "chip0_stuck_low": lambda f: setattr(f.chips[0], "line", "stuck_low"),
    "both_stuck_high": lambda f: [setattr(c, "line", "stuck_high") for c in f.chips],
    "chip0_inverted": lambda f: setattr(f.chips[0], "line", "inverted"),
    "late": lambda f: [setattr(c, "delay_fc", 20.0) for c in f.chips],
    "chip1_ab_swap": lambda f: setattr(f.chips[1], "ab_swap", True),
    "mosi_corrupt": lambda f: [setattr(c, "mosi_xor", 0x0001) for c in f.chips],
    "mosi_inverted": lambda f: [setattr(c, "mosi_invert", True) for c in f.chips],
    # MOSI open before the pair splits: every command arrives as CONVERT(0), answered
    # by an ADC nobody configured. The first real headstage run looked like this.
    "mosi_open": lambda f: [(setattr(c, "mosi_stuck", 0x0000), setattr(c, "adc_frozen", 0x0020))
                            for c in f.chips],
    "mosi_stuck_high": lambda f: [setattr(c, "mosi_stuck", 0xFFFF) for c in f.chips],
    "chip1_mosi_open": lambda f: (setattr(f.chips[1], "mosi_stuck", 0x0000),
                                  setattr(f.chips[1], "adc_frozen", 0x0020)),
    "chip1_dead_b": lambda f: setattr(f.chips[1], "dead_module", "B"),
    "chip1_dead": lambda f: setattr(f.chips[1], "dead_module", "AB"),
    "garbled": lambda f: setattr(f, "garbled", True),
    "no_emulator": lambda f: setattr(f, "emulator", False),
    "chip0_mosi_corrupt": lambda f: setattr(f.chips[0], "mosi_xor", 0x0004),
    "chip0_drops_writes": lambda f: setattr(f.chips[0], "drop_writes", True),
    "spi_dead": lambda f: setattr(f, "spi_dead", True),
    "stream_dead": lambda f: setattr(f, "stream_dead", True),
    "link_dead": lambda f: setattr(f.chips[1], "link_flaky", 1.0),
    "chip0_rhd2132": lambda f: setattr(f.chips[0], "chip_id", 1),
    "old_bitstream": lambda f: setattr(f, "board_id", 0x1C100001),
    "silent": lambda f: setattr(f, "silent", True),
    "decode_bug": lambda f: setattr(f, "decode_offset", 1),
    "selftest_broken": lambda f: setattr(f, "selftest_broken", True),
    "flaky_link": lambda f: setattr(f.chips[0], "link_flaky", 0.01),
    "skewed_chips": lambda f: setattr(f.chips[1], "delay_fc", 8.0),
}


def make_faults(*names) -> Faults:
    f = Faults()
    for n in names:
        FAULTS[n](f)
    return f


# ---------------------------------------------------------------------------
# One RHD2164
# ---------------------------------------------------------------------------

def default_signal(chip: int, module: str, c: int, n: int) -> int:
    """A deterministic few-LSB "noise" around a per-channel offset."""
    k = (chip * 64 + (32 if module == "B" else 0) + c) & 0xFFFF
    h = (k * 2654435761 + n * 40503 + 12345) & 0xFFFFFFFF
    return (k % 7) * 3 - 9 + ((h >> 13) % 41) - 20


class ChipModel:
    """Command in, result words out, two CS cycles later."""

    def __init__(self, chip: int, faults: ChipFaults = None, signal=default_signal):
        self.chip = chip
        self.f = faults or ChipFaults()
        self.signal = signal
        self.ram = [0] * R.N_RAM
        self.pipe = [(0, 0), (0, 0)]     # answers of the two previous commands, oldest first
        self.calib = 0
        self.n = [0] * 32                # conversions per amplifier, for the signal function
        self.rom_a = {**R.ROM_A, R.DIE_REV_REG: 1, R.ID_REG: self.f.chip_id}
        self.rom_b = {**R.ROM_B, R.DIE_REV_REG: 1, 61: 1, 62: 64, R.ID_REG: self.f.chip_id}
        if self.f.chip_id != R.CHIP_ID_RHD2164:   # other RHD2000 parts carry no A/B marker
            self.rom_a[R.MARKER_REG] = self.rom_b[R.MARKER_REG] = 0

    @property
    def twos_comp(self) -> bool:
        return bool(self.ram[4] & 0x40)

    def _encode(self, v: int) -> int:
        v = max(-32768, min(32767, v))
        return v & 0xFFFF if self.twos_comp else (v + 0x8000) & 0xFFFF

    def _amp(self, module: str, c: int) -> int:
        if self.f.adc_frozen is not None:
            return self.f.adc_frozen
        powered = self.ram[(14 if module == "A" else 18) + c // 8] >> (c % 8) & 1
        if not powered or module in self.f.dead_module:
            return self._encode(0)        # an unpowered or dead amplifier: flat
        return self._encode(self.signal(self.chip, module, c, self.n[c]))

    def execute(self, cmd: int) -> tuple[int, int]:
        """What the chip will answer for `cmd`, as received over MOSI."""
        if self.f.mosi_stuck is not None:
            cmd = self.f.mosi_stuck
        if self.f.mosi_invert:
            cmd = ~cmd & 0xFFFF
        cmd ^= self.f.mosi_xor
        msb_only = 0x0000 if self.twos_comp else 0x8000
        if self.calib:
            self.calib -= 1
            return msb_only, msb_only
        reg, data = (cmd >> 8) & 0x3F, cmd & 0xFF
        k = R.kind(cmd)
        if k == "CONVERT":
            if reg < 32:
                out = self._amp("A", reg), self._amp("B", reg)
                self.n[reg] += 1
                return out
            return self._encode(0), self._encode(0)
        if k == "READ":
            if reg < 18:
                return self.ram[reg], self.ram[reg]
            if reg < R.N_RAM:
                return 0x0000, self.ram[reg]   # amplifier power for B reads only on B
            return self.rom_a.get(reg, 0), self.rom_b.get(reg, 0)
        if k == "WRITE":
            if reg < R.N_RAM and not self.f.drop_writes:
                self.ram[reg] = data
            return 0xFF00 | data, 0xFF00 | (data ^ self.f.b_echo_xor)
        if k == "CALIBRATE":
            self.calib = self.f.calib_len
        return msb_only, msb_only

    def step(self, cmd: int) -> tuple[int, int]:
        """One CS cycle: present the answer to the command two cycles ago, run this one."""
        out = self.pipe[0]
        self.pipe = [self.pipe[1], self.execute(cmd)]
        return out


# ---------------------------------------------------------------------------
# One MISO wire
# ---------------------------------------------------------------------------

class Wire:
    """Chip result words -> MISO levels at the FPGA pin, per SPI tick."""

    def __init__(self, faults: ChipFaults, edge_fc: float = 0.0, seed: int = 1):
        self.f = faults
        self.edge_fc = edge_fc
        self.rng = random.Random(seed)
        self.prev = 0          # level the line holds from the previous cycle

    def _slot(self, s: int, a: int, b: int) -> int:
        j, half = divmod(s, 2)
        word = (a if half else b) if self.f.ab_swap else (b if half else a)
        return (word >> (15 - j)) & 1

    def _chip_level(self, t: float, a: int, b: int, D: float, T: int) -> int:
        s = math.floor((t - D - 2 * T) / (2 * T))
        if s < 0:
            return self.prev
        lvl = self._slot(min(s, 31), a, b)
        if self.edge_fc and s <= 32:
            edge = D + 2 * T + 2 * T * round((t - D - 2 * T) / (2 * T))
            if abs(t - edge) < self.edge_fc:
                lvl = self.rng.getrandbits(1)
        return lvl

    def idle_level(self) -> int:
        """The level the FPGA sees between cycles (MISO_STAT bits 4/5)."""
        line = self.f.line
        if line == "stuck_low":
            return 0
        if line in ("stuck_high", "absent"):
            return 1
        return 1 - self.prev if line == "inverted" else self.prev

    def level(self, t: float, a: int, b: int, spi_div: int) -> int:
        line = self.f.line
        if line == "stuck_low":
            return 0
        if line in ("stuck_high", "absent"):
            return 1           # LVDS receiver failsafe: an undriven pair reads high
        lvl = self._chip_level(t, a, b, self.f.delay_fc, spi_div + 1)
        return 1 - lvl if line == "inverted" else lvl

    def cycle(self, a: int, b: int, spi_div: int):
        """(taps, levels while CS was low) for one CS cycle."""
        T = spi_div + 1
        taps = 0
        for k in range(C.CAP_TAPS):
            if self.level((k + 2) * T, a, b, spi_div):
                taps |= 1 << k
        low = [self.level(t * T, a, b, spi_div) for t in range(65)]
        self.prev = self._slot(31, a, b)   # the last bit holds past CS rising
        return taps, low


# ---------------------------------------------------------------------------
# The board
# ---------------------------------------------------------------------------

def _clock_now(clock):
    return clock() if callable(clock) else 0.0


class VirtualClock:
    """Time that moves only when told to: sleep() and every UART round trip."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += max(0.0, dt)


class FakeBoard:
    """pyserial-shaped. See the module docstring."""

    def __init__(self, faults: Faults = None, clock=None, uart_s: float = 0.0015,
                 signal=default_signal, seed: int = 1):
        self.f = faults or Faults()
        self.clock = clock if clock is not None else VirtualClock()
        self.uart_s = uart_s
        self.regs = {addr: 0 for addr in C.REG.values()}
        self.regs[C.REG["SAMPLE_PERIOD"]] = 48000
        self.regs[C.REG["ID"]] = self.f.board_id
        self.regs[C.REG["CAPS"]] = (self.f.caps_taps << 16) | (0x100 if self.f.emulator else 0) | 3
        self.chips = [ChipModel(c, self.f.chips[c], signal) for c in (0, 1)]
        self.emu = [ChipModel(c, ChipFaults(), signal) for c in (0, 1)]
        self.wires = [Wire(self.f.chips[c], self.f.edge_fc, seed + c) for c in (0, 1)]
        self.emu_wires = [Wire(ChipFaults(delay_fc=self.f.emu_delay_fc), 0.0, seed + 7 + c)
                          for c in (0, 1)]
        self.rng = random.Random(seed)
        self.diag_list = [0] * C.DIAG_DEPTH
        self.diag_res = [(0, 0)] * C.DIAG_DEPTH
        self.diag_raw = [(0, 0)] * C.DIAG_DEPTH
        self.diag_status = 0
        self.miso_seen = [[False, False], [False, False]]   # [chip][low, high]
        self.edges = [0, 0]
        self.run_t0 = None
        self.swept = 0
        self.log = []
        self._out = b""
        self._in = ""
        self._names = {a: n for n, a in C.REG.items()}

    # --- pyserial-shaped surface ------------------------------------------
    def reset_input_buffer(self):
        self._out = b""

    def write(self, data: bytes) -> int:
        self._in += data.decode("ascii", "replace")
        while "\n" in self._in:                      # the board acts on whole lines only
            line, self._in = self._in.split("\n", 1)
            reply = self._handle(line.replace("\r", ""))
            if reply is not None:
                self._out += (reply + "\n").encode("latin-1")
        return len(data)

    def readline(self) -> bytes:
        line, nl, rest = self._out.partition(b"\n")
        if not nl:
            return b""             # pyserial's timeout
        self._out = rest
        return line + nl

    def close(self):
        pass

    # --- protocol ---------------------------------------------------------
    def _tick_uart(self):
        if hasattr(self.clock, "sleep") and self.uart_s:
            self.clock.sleep(self.uart_s)

    def _handle(self, line: str):
        self.log.append(line)
        self._tick_uart()
        if self.f.silent:
            return None
        if self.f.garbled:
            return "\x8f\xfe\x7f\xe1"
        parts = line.split()
        try:
            if len(parts) == 2 and parts[0] in ("R", "r") and len(parts[1]) == 2:
                addr = int(parts[1], 16)
                if addr not in self.regs:
                    return "err"
                return f"{addr:02x}={self._read(addr) & U32:08x}"
            if (len(parts) == 3 and parts[0] in ("W", "w") and len(parts[1]) == 2
                    and len(parts[2]) == 8):
                addr, val = int(parts[1], 16), int(parts[2], 16)
                if addr not in self.regs or addr not in WRITABLE:
                    return "err"
                self._write(addr, val)
                return "ok"
        except ValueError:
            return "err"
        return "err"

    # --- registers --------------------------------------------------------
    def _reg(self, name):
        return self.regs[C.REG[name]]

    def _set(self, name, v):
        self.regs[C.REG[name]] = v & U32

    @property
    def running(self) -> bool:
        return bool(self._reg("CTRL") & 1)

    @property
    def loopback(self) -> bool:
        return bool(self._reg("MODE") & 0x8) and self.f.emulator

    def _read(self, addr: int) -> int:
        name = self._names[addr]
        if self.running:
            self._advance()
        ptr = self._reg("DIAG_PTR")
        if name == "DIAG_CMD":
            return self.diag_list[ptr]
        if name == "DIAG_CTRL":
            return self.diag_status
        if name in ("DIAG_RES0", "DIAG_RES1"):
            a, b = self.diag_res[ptr][int(name[-1])]
            return a | (b << 16)
        if name.startswith("DIAG_RAW"):
            chip, part = int(name[8]), int(name[10])
            taps = self.diag_raw[ptr][chip]
            return (taps >> (32 * part)) & (U32 if part < 2 else 0x3FFF)
        if name == "MISO_STAT":
            v = 0
            wires = self.emu_wires if self.loopback else self.wires
            for c in (0, 1):
                v |= (self.miso_seen[c][0] << (2 * c)) | (self.miso_seen[c][1] << (2 * c + 1))
                v |= wires[c].idle_level() << (4 + c)
            return v | (0x100 if self.loopback else 0)
        if name in ("MISO_EDGES0", "MISO_EDGES1"):
            return self.edges[int(name[-1])]
        return self.regs[addr]

    def _write(self, addr: int, val: int) -> None:
        name = self._names[addr]
        if name == "CTRL":
            if self.running:
                self._advance()
            if val & 0x2:
                self._clear()
            was = self.running
            self._set("CTRL", val & 0x1)
            if self.running and not was:
                self.run_t0, self.swept = _clock_now(self.clock), 0
            elif was and not self.running:
                self._finish_run()
        elif name in ("PHASE_SELECT", "SPI_DIV"):
            self._set(name, min(val, 15))
        elif name == "DIAG_PTR":
            self._set(name, min(val, C.DIAG_DEPTH - 1))
        elif name == "DIAG_CMD":
            ptr = self._reg("DIAG_PTR")
            self.diag_list[ptr] = val & 0xFFFF
            self._set("DIAG_PTR", (ptr + 1) % C.DIAG_DEPTH)
        elif name == "DIAG_LEN":
            self._set(name, max(1, min(val, C.DIAG_DEPTH)))
        elif name == "DIAG_CTRL":
            if val & 1:
                self._go()
        else:
            self.regs[addr] = val & U32

    def _clear(self):
        for n in ("FRAMES", "DGRAMS", "FIFO_HIGH", "OVERRUNS", "STALLS", "SPI_CYCLES",
                  "LINK_CHECKS", "LINK_ERR0", "LINK_ERR1"):
            self._set(n, 0)
        self.miso_seen = [[False, False], [False, False]]
        self.edges = [0, 0]
        if self.running:
            self.run_t0, self.swept = _clock_now(self.clock), 0

    # --- one CS cycle -----------------------------------------------------
    def _cycle(self, cmd: int):
        """Run one command through both chips and wires: ((A, B) per chip, taps per chip)."""
        spi_div = self._reg("SPI_DIV")
        chips, wires = (self.emu, self.emu_wires) if self.loopback else (self.chips, self.wires)
        words, taps = [], []
        for c in (0, 1):
            a, b = chips[c].step(cmd)
            t, low = wires[c].cycle(a, b, spi_div)
            if self.loopback and self.f.selftest_broken:
                t, low = 0, [0] * 65
            words.append((a, b))
            taps.append(t)
            seen = self.miso_seen[c]
            seen[0] |= 0 in low
            seen[1] |= 1 in low
            self.edges[c] += sum(1 for x, y in zip(low, low[1:]) if x != y)
        self._set("SPI_CYCLES", self._reg("SPI_CYCLES") + 1)
        return words, taps

    def _decode(self, taps: int, phase: int) -> tuple[int, int]:
        return R.decode_taps(taps, max(0, min(C.PHASE_MAX, phase + self.f.decode_offset)))

    def _go(self):
        if self.running:
            self.diag_status = 0x4                     # rejected
            return
        if self.f.spi_dead:
            self.diag_status = 0x1                     # busy forever
            return
        phase = self._reg("PHASE_SELECT")
        for i in range(self._reg("DIAG_LEN")):
            _words, taps = self._cycle(self.diag_list[i])
            self.diag_raw[i] = (taps[0], taps[1])
            self.diag_res[i] = (self._decode(taps[0], phase), self._decode(taps[1], phase))
        self.diag_status = 0x2                         # done

    # --- streaming --------------------------------------------------------
    SWEEP = [R.convert(c) for c in range(32)] + [R.read(59), R.read(40), R.read(63)]

    def _advance(self):
        """Account for the sweeps run since the last look, in bulk."""
        if self.f.spi_dead or self.f.stream_dead or self.run_t0 is None:
            return
        period = max(self._reg("SAMPLE_PERIOD"), R.min_sample_period(self._reg("SPI_DIV")))
        elapsed = _clock_now(self.clock) - self.run_t0
        total = int(elapsed * C.SYS_CLK_HZ / period)
        n = total - self.swept
        if n <= 0:
            return
        self.swept = total
        lanes_bad = self._sweep_once()                 # deterministic part, per lane
        errs = [n if bad else 0 for bad in lanes_bad]
        for c in (0, 1):
            p = self.f.chips[c].link_flaky
            if p and not self.loopback:
                k = sum(1 for _ in range(n) if self.rng.random() < p)
                errs[2 * c] = min(n, errs[2 * c] + k)
        for c in (0, 1):
            reg = f"LINK_ERR{c}"
            lo, hi = self._reg(reg) & 0xFFFF, self._reg(reg) >> 16
            lo, hi = min(0xFFFF, lo + errs[2 * c]), min(0xFFFF, hi + errs[2 * c + 1])
            self._set(reg, lo | (hi << 16))
        self._set("LINK_CHECKS", self._reg("LINK_CHECKS") + n)
        # _sweep_once counted its own 35 cycles; the other n - 1 sweeps in bulk.
        self._set("SPI_CYCLES", self._reg("SPI_CYCLES") + (n - 1) * len(self.SWEEP))
        frames = self._reg("FRAMES") + n
        self._set("FRAMES", frames)
        if self.f.esp_present:
            self._set("DGRAMS", frames // C.FRAMES_PER_DGRAM)
            self._set("FIFO_HIGH", 1)
        else:
            built = frames // C.FRAMES_PER_DGRAM
            self._set("FIFO_HIGH", min(4, built))
            self._set("OVERRUNS", max(0, built - 4))
            self._set("STALLS", int(elapsed * C.SYS_CLK_HZ))

    def _sweep_once(self) -> list[bool]:
        """Run one real sweep; return, per lane, whether its READ(59) check failed."""
        taps = None
        for cmd in self.SWEEP:
            _w, taps = self._cycle(cmd)
        phase = self._reg("PHASE_SELECT")
        bad = []
        last = []
        for c in (0, 1):
            a, b = self._decode(taps[c], phase)    # the same decoder as DIAG_RES
            last.append(a | (b << 16))
            bad += [a != R.MARKER_A, b != R.MARKER_B]
        self._set("LINK_LAST0", last[0])
        self._set("LINK_LAST1", last[1])
        return bad

    def _finish_run(self):
        self.run_t0 = None
