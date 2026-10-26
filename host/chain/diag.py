"""The FPGA's diagnostic command list, driven over the UART (CONTRACTS.md 4 and 6).

The list is how the host talks to the RHD2164s before anything streams: write
up to 64 command words, `go`, then read back what came out of each CS cycle,
both as the FPGA decoded it and as raw MISO taps. The raw taps are the point.
With them the host decodes every phase itself, so one capture is a complete
eye scan, and the FPGA's own decode can be checked against the host's: when
those two disagree the fault is in the FPGA, not in a wire.

Lanes are numbered in frame channel order: 0 = chip 0 module A, 1 = chip 0 B,
2 = chip 1 A, 3 = chip 1 B. Chip c's two lanes share one MISO wire (miso c).
"""
import time
from dataclasses import dataclass, field

from . import contract as C
from . import rhd2164 as R
from .ctrl import CtrlError

LANES = ("chip0 A", "chip0 B", "chip1 A", "chip1 B")
N_LANES = 4
PHASES = range(C.PHASE_MAX + 1)

GO = 0x1
ST_BUSY, ST_DONE, ST_REJECTED = 0x1, 0x2, 0x4
TAPS_MASK = (1 << C.CAP_TAPS) - 1


def lane_of(chip: int, module: str) -> int:
    return 2 * chip + (1 if module == "B" else 0)


def chip_of(lane: int) -> int:
    return lane // 2


def module_of(lane: int) -> str:
    return "AB"[lane % 2]


def split16(v: int) -> tuple[int, int]:
    """[15:0], [31:16]: module A and module B halves of a RES/LINK register."""
    return v & 0xFFFF, (v >> 16) & 0xFFFF


class DiagError(CtrlError):
    """The diagnostic list did not run."""


class DiagRejected(DiagError):
    """`go` arrived while `run` was set; nothing was sent."""


class DiagTimeout(DiagError):
    """DIAG_CTRL never reported done."""


@dataclass
class Cycle:
    """One CS cycle of a list as the FPGA recorded it."""
    index: int
    command: int                                  # the word sent in this cycle
    words: tuple[int, int, int, int]              # FPGA decode, lane order
    taps: tuple[int, int] | None = None           # raw miso0, miso1; bit k = tap k


@dataclass
class Result:
    """The answer to one command: the cycle two later, re-labelled with its command."""
    command: int
    words: tuple[int, int, int, int]
    taps: tuple[int, int] | None
    chunk: int
    cycle: int

    def word(self, lane: int) -> int:
        return self.words[lane]


@dataclass
class Capture:
    """One run of the list, cycle by cycle, with the settings in force during it."""
    commands: list[int]
    cycles: list[Cycle | None]                    # None: not read back
    phase: int | None                             # read only with raw taps (crosscheck)
    spi_div: int | None

    def results(self, chunk: int = 0) -> list[Result | None]:
        """Pipeline alignment: command i's answer is in cycle i + 2.

        The last two commands of a list have no answer inside it; their slots
        are None rather than the next list's cycles, which do not exist here.
        """
        out = []
        for i, cmd in enumerate(self.commands):
            k = i + R.PIPELINE
            cy = self.cycles[k] if k < len(self.cycles) else None
            out.append(None if cy is None else Result(cmd, cy.words, cy.taps, chunk, k))
        return out


def chunk_plan(commands: list[int], depth: int = C.DIAG_DEPTH) -> list[list[int]]:
    """Split a list longer than the FPGA holds.

    A list that fits is sent as it is: the caller decides whether it ends with
    flushers. A longer one is cut into pieces of depth - 2 commands, each
    followed by two flushers, so every command's answer lands inside its own
    piece; the first two cycles of each later piece carry the previous piece's
    flushers and are ignored.
    """
    if len(commands) <= depth:
        return [list(commands)]
    step = depth - R.PIPELINE
    return [list(commands[i:i + step]) + R.FLUSHERS for i in range(0, len(commands), step)]


class Diag:
    """The list, over a Ctrl. Holds no state the board does not also hold,
    except a note of which list is loaded, so a repeated snapshot costs only
    `go` and the read-back."""

    def __init__(self, ctrl, timeout: float = 1.0, poll: float = 0.0,
                 sleep=time.sleep, clock=time.monotonic):
        self.ctrl = ctrl
        self.timeout = timeout
        self.poll = poll
        self.sleep = sleep
        self.clock = clock
        self._loaded = None

    # --- primitives ------------------------------------------------------
    def load(self, commands: list[int]) -> None:
        if not 1 <= len(commands) <= C.DIAG_DEPTH:
            raise DiagError(f"a list holds 1..{C.DIAG_DEPTH} commands, got {len(commands)}")
        if self._loaded == list(commands):
            return
        self._loaded = None  # a failure half way must not leave a stale note
        self.ctrl.write("DIAG_PTR", 0)
        for cmd in commands:
            self.ctrl.write("DIAG_CMD", cmd & 0xFFFF)  # DIAG_PTR advances by itself
        self.ctrl.write("DIAG_LEN", len(commands))
        self._loaded = list(commands)

    def forget(self) -> None:
        """Make the next load() write the list again (after anyone else touched it)."""
        self._loaded = None

    def status(self) -> int:
        return self.ctrl.read("DIAG_CTRL")

    def go(self) -> None:
        self.ctrl.write("DIAG_CTRL", GO)
        deadline = self.clock() + self.timeout
        while True:
            st = self.status()
            if st & ST_REJECTED:
                raise DiagRejected("DIAG_CTRL rejected `go`: CTRL.run is set; "
                                   "stop streaming before using the diagnostic list")
            if st & ST_DONE and not st & ST_BUSY:
                return
            if self.clock() > deadline:
                raise DiagTimeout(f"DIAG_CTRL still 0x{st:x} after {self.timeout:.1f} s "
                                  f"(want done): the list sequencer is not running")
            if self.poll:
                self.sleep(self.poll)

    def read_cycle(self, i: int, raw: bool = False, command: int = 0) -> Cycle:
        self.ctrl.write("DIAG_PTR", i)
        r0 = self.ctrl.read("DIAG_RES0")
        r1 = self.ctrl.read("DIAG_RES1")
        words = split16(r0) + split16(r1)
        taps = None
        if raw:
            taps = tuple(self._taps(c) for c in (0, 1))
        return Cycle(i, command, words, taps)

    def _taps(self, chip: int) -> int:
        lo = self.ctrl.read(f"DIAG_RAW{chip}_0")
        mid = self.ctrl.read(f"DIAG_RAW{chip}_1")
        hi = self.ctrl.read(f"DIAG_RAW{chip}_2") & ((1 << (C.CAP_TAPS - 64)) - 1)
        return lo | (mid << 32) | (hi << 64)

    # --- whole lists -----------------------------------------------------
    def run(self, commands: list[int], raw: bool = False, cycles=None) -> Capture:
        """Send one list (<= 64 commands) and read back the cycles asked for.

        cycles: which cycle indices to read (default all). Each costs three UART
        round trips, nine with raw taps, so skipping the ones nobody needs is
        what makes a 128-channel snapshot affordable.
        """
        self.load(commands)
        # The settings matter only to someone decoding the raw taps; skipping
        # them keeps a FPGA-decoded snapshot two round trips cheaper.
        phase = self.ctrl.read("PHASE_SELECT") if raw else None
        spi_div = self.ctrl.read("SPI_DIV") if raw else None
        self.go()
        want = set(range(len(commands)) if cycles is None else cycles)
        # A cycle nobody read stays None, so it can never pass for an answer.
        full = [self.read_cycle(i, raw, commands[i]) if i in want else None
                for i in range(len(commands))]
        return Capture(list(commands), full, phase, spi_div)

    def run_results(self, commands: list[int], raw: bool = False) -> list[Result | None]:
        """Answers to every command, chunking past 64 (see chunk_plan)."""
        plan = chunk_plan(commands)
        if len(plan) == 1:
            return self.run(plan[0], raw).results()
        out = []
        for n, piece in enumerate(plan):
            res = self.run(piece, raw).results(chunk=n)
            out.extend(res[:len(piece) - R.PIPELINE])  # drop this piece's own flushers
        return out[:len(commands)]


# ---------------------------------------------------------------------------
# Host-side phase scan
# ---------------------------------------------------------------------------

def lane_word(taps: int, lane: int, phase: int, invert: bool = False,
              swap: bool = False) -> int:
    """The word lane `lane` decodes to at `phase` from its chip's taps."""
    if invert:
        taps = ~taps & TAPS_MASK
    a, b = R.decode_taps(taps, phase)
    want_b = (lane % 2 == 1) != swap
    return b if want_b else a


def rom_expected_by_lane() -> dict[int, dict[int, int]]:
    """Expected ROM words per lane for rhd2164.rom_commands(), indexed by command."""
    exp = R.rom_expectations()
    return {lane: dict(exp[module_of(lane)]) for lane in range(N_LANES)}


@dataclass
class Eye:
    """Matches per lane per phase, from one capture."""
    counts: dict[int, list[int]]
    totals: dict[int, int]

    def passing(self, lane: int) -> list[int]:
        t = self.totals.get(lane, 0)
        return [p for p in PHASES if t and self.counts[lane][p] == t]

    def windows(self, lane: int) -> list[tuple[int, int]]:
        return runs(self.passing(lane))

    def present(self) -> list[int]:
        """Lanes with at least one fully passing phase."""
        return [lane for lane in self.counts if self.passing(lane)]

    def table(self, label: str = "") -> str:
        """Rows of lanes, columns of phases; '#' every word right, digits = words right."""
        head = "  phase    " + " ".join(f"{p:>2}" for p in PHASES)
        lines = [head]
        for lane in sorted(self.counts):
            t = self.totals[lane]
            cells = []
            for p in PHASES:
                n = self.counts[lane][p]
                cells.append(" #" if t and n == t else (" ." if n == 0 else f"{n:>2}"))
            lines.append(f"  {LANES[lane]:<8}" + " ".join(c for c in cells) +
                         f"   ({t} words)")
        if label:
            lines.insert(0, f"  {label}")
        return "\n".join(lines)

    def as_dict(self) -> dict:
        return {LANES[l]: {"matches": self.counts[l], "total": self.totals[l],
                           "windows": self.windows(l)} for l in sorted(self.counts)}


def eye_scan(results: list[Result | None], expected: dict[int, dict[int, int]],
             invert: bool = False, swap: bool = False) -> Eye:
    """Decode every phase of every expected word on every lane; count matches."""
    counts, totals = {}, {}
    for lane, want in expected.items():
        chip = chip_of(lane)
        row = [0] * len(PHASES)
        n = 0
        for idx, w in want.items():
            res = results[idx] if idx < len(results) else None
            if res is None or res.taps is None:
                continue
            n += 1
            for p in PHASES:
                if lane_word(res.taps[chip], lane, p, invert, swap) == w:
                    row[p] += 1
        counts[lane], totals[lane] = row, n
    return Eye(counts, totals)


def runs(phases) -> list[tuple[int, int]]:
    """Consecutive runs, inclusive, e.g. [4, 5, 6, 9] -> [(4, 6), (9, 9)].

    Phases do not wrap: 15 and 0 are a whole SCLK period apart in time.
    """
    out = []
    for p in sorted(phases):
        if out and p == out[-1][1] + 1:
            out[-1] = (out[-1][0], p)
        else:
            out.append((p, p))
    return out


def widest(windows: list[tuple[int, int]]) -> tuple[int, int] | None:
    """The widest window; the earlier one on a tie (less cable delay to explain)."""
    if not windows:
        return None
    return max(windows, key=lambda w: (w[1] - w[0], -w[0]))


def centre(window: tuple[int, int]) -> int:
    """Middle of a window, rounding down on an even width."""
    return window[0] + (window[1] - window[0]) // 2


@dataclass
class PhaseChoice:
    phase: int | None
    window: tuple[int, int] | None          # the common window the phase sits in
    lanes: list[int]                        # lanes the choice serves
    per_lane: dict[int, list[tuple[int, int]]] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"phase": self.phase, "window": self.window,
                "lanes": [LANES[l] for l in self.lanes],
                "per_lane": {LANES[l]: w for l, w in self.per_lane.items()}}


def choose_phase(eye: Eye, lanes=None) -> PhaseChoice:
    """Centre of the widest window every present lane passes in.

    lanes defaults to every lane with any passing phase: an absent chip must not
    veto the phase for the chip that is there, and is reported separately.
    """
    lanes = eye.present() if lanes is None else list(lanes)
    per_lane = {l: eye.windows(l) for l in eye.counts}
    if not lanes:
        return PhaseChoice(None, None, [], per_lane)
    common = set(PHASES)
    for l in lanes:
        common &= set(eye.passing(l))
    w = widest(runs(common))
    if w is None:
        return PhaseChoice(None, None, lanes, per_lane)
    return PhaseChoice(centre(w), w, lanes, per_lane)


# ---------------------------------------------------------------------------
# Evidence for classifying a lane that has no passing phase
# ---------------------------------------------------------------------------

MIN_KNOWN_BITS = 6     # per word, inside the tap window
MIN_ONES = 8           # '1' bits matched across the words: every ROM word's high byte is 0,
                       # so a match on zeros alone is what an idle line gives, not evidence


def offset_scan(results: list[Result | None], expected: dict[int, int], lane: int,
                phases=range(-12, C.CAP_TAPS - 2 - 4 * (MIN_KNOWN_BITS - 1))) -> list[int]:
    """Phases outside 0..PHASE_MAX at which every expected word matches in
    every bit the tap window does hold.

    A hit at phase 18 means the chip answers correctly, three ticks later than
    PHASE_SELECT can reach: delay, not corruption.
    """
    chip = chip_of(lane)
    hits = []
    for p in phases:
        if 0 <= p <= C.PHASE_MAX:
            continue
        ok = n = ones = 0
        for idx, want in expected.items():
            res = results[idx] if idx < len(results) else None
            if res is None or res.taps is None:
                continue
            a, am, b, bm = R.decode_partial(res.taps[chip], p)
            got, mask = (b, bm) if lane % 2 else (a, am)
            n += 1
            if bin(mask).count("1") >= MIN_KNOWN_BITS and (got ^ want) & mask == 0:
                ok += 1
                ones += bin(want & mask).count("1")
        if n and ok == n and ones >= MIN_ONES:
            hits.append(p)
    return hits


def bit_shift(got: int, want: int, max_shift: int = 3) -> int | None:
    """k if got is want shifted left by k bits (negative: right), else None.

    Only the bits both words hold are compared; the bits shifted in are
    whatever was on the wire before or after the word.
    """
    if want == 0:
        return None
    for k in range(1, max_shift + 1):
        mask = (0xFFFF << k) & 0xFFFF
        if (got & mask) == ((want << k) & mask):
            return k
        mask = 0xFFFF >> k
        if (got & mask) == (want >> k):
            return -k
    return None


def best_partial(results, expected: dict[int, int], lane: int):
    """(phase, matches, {cmd index: word}) at the phase that gets most words right."""
    chip = chip_of(lane)
    best = (None, -1, {})
    for p in PHASES:
        words, m = {}, 0
        for idx, want in expected.items():
            res = results[idx] if idx < len(results) else None
            if res is None or res.taps is None:
                continue
            words[idx] = lane_word(res.taps[chip], lane, p)
            m += words[idx] == want
        if m > best[1]:
            best = (p, m, words)
    return best


def chip_taps(results, chip: int) -> list[int]:
    return [r.taps[chip] for r in results if r is not None and r.taps is not None]


# ---------------------------------------------------------------------------
# FPGA decode against host decode
# ---------------------------------------------------------------------------

@dataclass
class Mismatch:
    cycle: int
    lane: int
    fpga: int
    host: int


def crosscheck(capture: Capture) -> list[Mismatch]:
    """Every cycle, every lane: DIAG_RES must equal decode_taps at PHASE_SELECT.

    Independent of whether the phase is a good one. A mismatch means the FPGA's
    decode or tap numbering is not what CONTRACTS.md section 6 says.
    """
    out = []
    for cy in capture.cycles:
        if cy is None or cy.taps is None:
            continue
        for lane in range(N_LANES):
            host = lane_word(cy.taps[chip_of(lane)], lane, capture.phase)
            if host != cy.words[lane]:
                out.append(Mismatch(cy.index, lane, cy.words[lane], host))
    return out


# ---------------------------------------------------------------------------
# Status registers
# ---------------------------------------------------------------------------

@dataclass
class MisoStat:
    seen_low: tuple[bool, bool]
    seen_high: tuple[bool, bool]
    level: tuple[int, int]
    loopback: bool
    edges: tuple[int, int]

    def line(self, chip: int) -> str:
        """'toggling', 'stuck_low', 'stuck_high' or 'unsampled' (CS never low)."""
        lo, hi = self.seen_low[chip], self.seen_high[chip]
        if lo and hi:
            return "toggling"
        if lo:
            return "stuck_low"
        if hi:
            return "stuck_high"
        return "unsampled"

    def as_dict(self) -> dict:
        return {f"miso{c}": {"state": self.line(c), "seen_low": self.seen_low[c],
                             "seen_high": self.seen_high[c], "level": self.level[c],
                             "edges": self.edges[c]} for c in (0, 1)} | {
            "loopback": self.loopback}


def decode_miso_stat(stat: int, edges0: int, edges1: int) -> MisoStat:
    return MisoStat(seen_low=(bool(stat & 0x1), bool(stat & 0x4)),
                    seen_high=(bool(stat & 0x2), bool(stat & 0x8)),
                    level=((stat >> 4) & 1, (stat >> 5) & 1),
                    loopback=bool(stat & 0x100),
                    edges=(edges0, edges1))


def read_miso(ctrl) -> MisoStat:
    return decode_miso_stat(ctrl.read("MISO_STAT"), ctrl.read("MISO_EDGES0"),
                            ctrl.read("MISO_EDGES1"))


LINK_EXPECT = (0x0035, 0x003A, 0x0035, 0x003A)  # READ(59) on A, B of each chip


@dataclass
class LinkStat:
    checks: int
    errors: tuple[int, int, int, int]   # per lane, each saturating at 0xffff
    last: tuple[int, int, int, int]     # per lane, the most recent READ(59) word

    def lane_ok(self, lane: int) -> bool:
        return self.errors[lane] == 0

    def as_dict(self) -> dict:
        return {"checks": self.checks,
                "errors": {LANES[l]: self.errors[l] for l in range(N_LANES)},
                "last": {LANES[l]: f"0x{self.last[l]:04x}" for l in range(N_LANES)}}


def read_link(ctrl) -> LinkStat:
    checks = ctrl.read("LINK_CHECKS")
    e0, e1 = ctrl.read("LINK_ERR0"), ctrl.read("LINK_ERR1")
    l0, l1 = ctrl.read("LINK_LAST0"), ctrl.read("LINK_LAST1")
    return LinkStat(checks, split16(e0) + split16(e1), split16(l0) + split16(l1))


@dataclass
class Caps:
    qspi_div: int
    emulator: bool
    taps: int
    raw: int


def decode_caps(v: int) -> Caps:
    return Caps(qspi_div=v & 0xFF, emulator=bool(v & 0x100), taps=(v >> 16) & 0xFF, raw=v)


# ---------------------------------------------------------------------------
# Snapshots of all 128 channels through the list
# ---------------------------------------------------------------------------

def convert_snapshot(diag: Diag) -> list[int]:
    """128 signed CONVERT results in frame channel order, one diag sweep.

    Reads only the 32 cycles that carry answers (2..33), FPGA decode only.
    """
    cmds = R.convert_sweep()
    cap = diag.run(cmds, raw=False, cycles=range(R.PIPELINE, 32 + R.PIPELINE))
    res = cap.results()
    out = [0] * C.N_CH
    for c in range(32):
        w = res[c].words
        for lane in range(N_LANES):
            out[R.global_channel(chip_of(lane), module_of(lane), c)] = R.to_signed(w[lane])
    return out
