"""Bring up two real RHD2164s behind the FPGA, one layer at a time.

    python3 -m chain.bringup --serial /dev/cu.usbserial-210352BF97961
    python3 -m chain.bringup --serial PORT --spi-div 1 --phase auto --rate 2000 --seconds 5
    python3 -m chain.bringup --fake chip1_absent          # no hardware: see a failure report

Each stage answers one question and stops being ambiguous before the next one
builds on it:

    S0 FPGA        is the right bitstream answering on this port?
    S1 Self-test   can the receiver read its own on-die emulator? (FPGA, not wires)
    S2 Bus         does each MISO line move while CS is low?
    S3 Identity    does each chip read its ROM, at which phase and SPI_DIV?
    S4 Configure   do writes arrive intact (echo) and stay (read-back)?
    S5 Calibrate   does each chip come back from the calibration window?
    S6 Convert     what do the 128 amplifiers read?
    S7 Streaming   does the link check pass at the sample rate, with no ESP?

Every result other than PASS names a LIKELY CAUSE and a CHECK that point at a
physical thing. Findings carry a category (the keys of CAUSES) so tests and
scripts can act on them without parsing prose. Nothing prints PASS that was
not checked: a stage that could not run says SKIP and why.
"""
import argparse
import json
import math
import pathlib
import statistics
import time
import traceback
from dataclasses import asdict, dataclass, field

from . import contract as C
from . import diag as D
from . import rhd2164 as R
from .ctrl import Ctrl, CtrlError

RESULTS = pathlib.Path(__file__).resolve().parents[2] / "docs" / "results"
OLD_ID = 0x1C100001
RETRY_DIVS = (1, 3, 7)
RAIL_LSB = 32700            # |value| at or beyond this is at the ADC rail
NOISY_UV = 100.0            # spread above this on a channel is "noisy" (WARN)
RANK = {"PASS": 0, "SKIP": 0, "INFO": 0, "WARN": 1, "FAIL": 2}

# category -> (LIKELY CAUSE, CHECK). {c}, {lane}, {div} ... are filled per finding.
CAUSES = {
    "PYSERIAL_MISSING": (
        "pyserial is not installed for this Python",
        "python3 -m pip install pyserial"),
    "PORT_OPEN": (
        "the serial port does not exist, or another program holds it",
        "ls /dev/cu.usbserial-*; quit screen or any other tool on the port; replug the "
        "Arty{hint}"),
    "NO_REPLY": (
        "nothing answers on this port: the FPGA is not programmed (the bitstream is "
        "volatile and lost at power-off), this is the wrong one of the Arty's two FTDI "
        "ports (the first is JTAG and never answers), or the baud is wrong",
        "reprogram (openFPGALoader -b arty_s7_25 ...); use the port whose name ends in 1; "
        "--baud 115200{hint}"),
    "GARBLED_REPLY": (
        "something answers but not in the register protocol: wrong baud rate, or another "
        "device on this port",
        "--baud 115200; that the port is the Arty's UART{hint}"),
    "ID_REJECTED": (
        "the board speaks the register protocol but has no ID register: a bitstream older "
        "than the contract",
        "load the headstage bitstream built from this checkout"),
    "OLD_BITSTREAM": (
        "ID 0x1c100001 is the emulator-chain build, which has no diagnostic list, link "
        "check or SPI_DIV",
        "load the headstage build; check the .bit file you programmed is the new one"),
    "WRONG_BITSTREAM": (
        "the ID register does not match this host's CONTRACTS.md",
        "load the headstage bitstream built from this checkout (make check-contract)"),
    "CONTRACT_MISMATCH": (
        "the bitstream disagrees with this host about the register map or tap count: they "
        "were built from different CONTRACTS.md",
        "rebuild the bitstream from this checkout; make check-contract"),
    "UART_LOST": (
        "the board stopped answering in the middle of a stage",
        "the USB cable; board power (a reset also clears the bitstream); another program "
        "opening the port"),
    "SELFTEST_FAIL": (
        "the FPGA cannot read its own on-die emulator: a receiver or bitstream fault, not "
        "wiring",
        "rebuild the bitstream; the FPGA testbenches; do not debug cables until this passes"),
    "LOOPBACK_STUCK": (
        "MODE bit3 is clear but MISO_STAT still reports loopback",
        "the MODE decode in the RTL; reload the bitstream"),
    "DECODE_MISMATCH": (
        "the FPGA's decoded words differ from the host's decode of the same raw taps: the "
        "FPGA's decode or tap numbering disagrees with CONTRACTS.md section 6 (or capture "
        "timing inside the FPGA). This is not a link fault",
        "miso_phase_select and the tap capture in the RTL against CONTRACTS.md section 6"),
    "DIAG_REJECTED": (
        "the diagnostic list refused to start because CTRL.run is set",
        "stop streaming (W 00 00000000) and rerun"),
    "SPI_NOT_RUNNING": (
        "the FPGA's SPI sequencer did not run the list (done never set, or SPI_CYCLES did "
        "not move)",
        "that the headstage bitstream is loaded; clock lock; reload the bitstream"),
    "SPI_CYCLE_COUNT": (
        "SPI_CYCLES moved by a different number than the list length",
        "the list sequencer or the counter in the RTL"),
    "LOOPBACK_STILL_ON": (
        "the receiver is still reading the on-die emulator, not the headstage",
        "MODE bit3 must be clear (W 03 00000000)"),
    "MISO_UNSAMPLED": (
        "MISO_STAT saw neither level on miso{c}: CS never went low during the list",
        "the bitstream; the CS output in the board XDC"),
    "MISO_STUCK_LOW": (
        "miso{c} never went high: the FPGA input's pull-down is all it sees, so nothing "
        "drives it",
        "the MISO{c} wire from the LVDS converter output to the FPGA pin (fpga/boards/*.xdc); "
        "the converter's 3.3 V supply; a common ground between headstage supply, converter "
        "and Arty"),
    "MISO_STUCK_HIGH": (
        "miso{c} never went low: the converter's LVDS receiver is in failsafe, so chip {c} "
        "is not driving its MISO pair",
        "chip {c} power at the headstage; the MISO{c} pair in the cable (open, or wrong "
        "pins); CS and SCLK reaching chip {c} (a chip that never sees CS never drives MISO)"),
    "BOTH_STUCK": (
        "both MISO lines are stuck the same way: a cause the two chips share",
        "headstage power and ground; the cable and its connector; the CS and SCLK pairs; the "
        "converter's supply"),
    "ROM_ALL_ZERO": (
        "{lane} reads 0x0000 at every phase: the line is low throughout (see S2)",
        "as MISO_STUCK_LOW: the MISO{c} wire, converter supply, common ground"),
    "ROM_ALL_ONES": (
        "{lane} reads 0xffff at every phase: the line is high throughout (see S2)",
        "as MISO_STUCK_HIGH: chip {c} power, the MISO{c} pair, CS/SCLK reaching chip {c}"),
    "MISO_INVERTED": (
        "{lane} reads its ROM only with every bit inverted: the MISO{c} LVDS pair is "
        "swapped (P for N) somewhere between chip {c} and the converter",
        "the MISO{c} P/N order at the converter input and at the headstage connector"),
    "AB_SWAPPED": (
        "chip {c}'s module A and B answers trade places: it clocks its two halves out on "
        "the opposite SCLK edges, which is what an inverted SCLK pair does",
        "the SCLK pair's P/N order at the converter and the headstage connector"),
    "DELAY_OUT_OF_WINDOW": (
        "chip {c} answers correctly but {ticks} tick(s) later than PHASE_SELECT 15 reaches "
        "at SPI_DIV {div}: the round-trip cable and converter delay is too long for this SCLK",
        "a larger --spi-div (the window reaches further in time); a shorter cable"),
    "DATA_TOO_EARLY": (
        "{lane}'s answer starts before tap 0: the FPGA's capture window opens later than "
        "CONTRACTS.md section 6 says. Not a cable fault",
        "tap timing in the RTL (tap k is the level on tick k + 2)"),
    "NOT_RHD2164": (
        "chip {c} reads 'INTAN' but its chip ID is {id} ({name}), not 4: it is not an RHD2164",
        "the part fitted to the headstage; a different part needs a different channel map"),
    "ROM_PARTIAL": (
        "{lane} gets {m} of {n} ROM words right at its best phase ({p}): a marginal eye "
        "(noise, reflections, slow edges)",
        "a larger --spi-div; cable termination and ground"),
    "ROM_GARBLED": (
        "miso{c} toggles but no phase reads chip {c}'s ROM: the chip is not executing the "
        "READ commands as sent, or MISO is too noisy to read",
        "the MOSI pair (an inverted MOSI turns READ into CONVERT); the SCLK pair; then "
        "--spi-div 7"),
    "CHIP_NOT_ANSWERING": (
        "chip {other} answers and chip {c} does not, so the shared CS/SCLK/MOSI lines work",
        "chip {c}'s own: power, the MISO{c} pair, and its connector pins"),
    "B_ONLY_FAIL": (
        "{lane} fails while module A on the same wire passes: module B's last bit is "
        "sampled after CS rises, so a CS-high time or capture window too short hurts B only",
        "taps past CS rising in the RTL; if the chip is not an RHD2164 there is no B half"),
    "NO_COMMON_WINDOW": (
        "the lanes read correctly at different phases and PHASE_SELECT is shared: the two "
        "chips' MISO delays differ by two ticks or more",
        "equal-length MISO pairs in one cable; a larger --spi-div shrinks the difference in "
        "ticks"),
    "NARROW_WINDOW": (
        "only one phase reads every lane: the eye is marginal and may close with "
        "temperature or a longer cable",
        "a larger --spi-div widens the window in time"),
    "SPI_DIV_CHANGED": (
        "SPI_DIV {req} did not read every lane, SPI_DIV {div} did: this link needs the "
        "slower SCLK",
        "cable length and termination, if the full 24 MHz SCLK matters"),
    "PHASE_OUTSIDE_WINDOW": (
        "--phase {p} is outside the window the scan found",
        "--phase auto, or a phase inside the window"),
    "CONFIG_WARNING": (
        "the requested amplifier settings are legal but probably not intended",
        "--rate, --upper, --lower, --dsp-cutoff"),
    "MOSI_CORRUPT": (
        "chip {c} echoes different data than was written while it reads its ROM fine: "
        "MISO is good, MOSI is not arriving intact",
        "the MOSI pair (shared: if only one chip is affected, look at its branch or "
        "connector pin); MOSI setup time at the chip: try --spi-div 1"),
    "REG_READBACK": (
        "chip {c} echoed the writes but reads back other values: the registers did not "
        "hold (a reset or brown-out between write and read)",
        "chip {c}'s supply under load and its decoupling; ground"),
    "B_ECHO_DIFFERS": (
        "module B's echo differs while module A's is right on the same wire: not a wiring "
        "fault (A proves MOSI), but not what this host's model of the RHD2164 expects",
        "report the words; the bring-up's model of module B may be wrong"),
    "CALIBRATE_NO_RESUME": (
        "chip {c} did not answer READ(63) normally after the 9-command calibration window",
        "MOSI (a missed command shifts the window); repeat the bring-up"),
    "MODULE_DEAD": (
        "all 32 amplifiers of {lane} read one constant ({value})",
        "registers {regs} (that module's amplifier power, checked in S4); then the chip"),
    "CHIP_DEAD": (
        "every amplifier of chip {c} reads a constant although it reads its ROM",
        "chip {c}'s analog supply; registers 14-21 (S4)"),
    "STUCK_CHANNELS": (
        "these channels read exactly the same value in every snapshot",
        "those inputs (shorted to reference?) or dead amplifiers"),
    "RAILED_CHANNELS": (
        "these channels sit at the ADC rail: floating inputs, or an electrode offset beyond "
        "+-6.4 mV",
        "electrodes and the reference electrode; normal with nothing attached"),
    "NOISY_CHANNELS": (
        "large spread between snapshots: normal for unconnected inputs, which float and "
        "pick up mains",
        "attach electrodes and reference before judging noise"),
    "RATE_TOO_HIGH": (
        "--rate {rate:g} needs a SAMPLE_PERIOD below one sweep's {minp} fast clocks at "
        "SPI_DIV {div}",
        "a lower --rate, or a smaller SPI_DIV if the link allows"),
    "SEQUENCER_IDLE": (
        "LINK_CHECKS did not move while run was set: no sweeps ran",
        "SAMPLE_PERIOD; that run was set; the bitstream"),
    "LINK_CHECK_COUNT": (
        "LINK_CHECKS ({got}) is not rate x time ({lo:.0f}..{hi:.0f})",
        "SAMPLE_PERIOD read-back; the 96 MHz clock"),
    "LINK_ERRORS": (
        "{lane}'s READ(59) check failed on every sweep ({n}) while streaming, though it read "
        "its ROM in the diagnostic list; last word 0x{last:04x}",
        "PHASE_SELECT and SPI_DIV as written for streaming; the RTL link check"),
    "LINK_ERRORS_INTERMITTENT": (
        "{lane} failed {n} of {m} link checks: the eye is marginal or the line is noisy; "
        "last word 0x{last:04x}",
        "a phase nearer the window centre; a larger --spi-div; cable shielding and ground"),
    "NO_FRAMES": (
        "sweeps ran but no frames were built",
        "MODE (bit2 selects the internal pattern); the frame builder"),
    "OVERRUNS": (
        "an ESP is draining QSPI but not fast enough: {n} datagrams dropped",
        "--rate; the ESP's Wi-Fi throughput (chain.sweep measures it)"),
    "ESP_NOT_DRAINING": (
        "no ESP is taking datagrams (DGRAMS {dgrams}, STALLS {stalls}, OVERRUNS {overruns}): "
        "expected when the ESP is not connected",
        "connect and power the ESP if streaming to the PC is wanted"),
    "HOST_ERROR": (
        "this host code met a case it does not handle (a bug, or a board reply nothing "
        "anticipated); the traceback is in the JSON report",
        "the JSON report's traceback; rerun with --skip-loopback or a different --spi-div"),
    "RUN_WAS_SET": ("", ""),
    "INFO": ("", ""),
}


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------

@dataclass
class Finding:
    status: str                 # PASS, WARN, FAIL, INFO
    category: str
    scope: str                  # 'board', 'chip1', 'chip0 B', 'config', ...
    message: str
    cause: str = ""
    check: str = ""


@dataclass
class Stage:
    id: str
    name: str
    status: str = "PASS"
    summary: str = ""
    checked: list = field(default_factory=list)   # what was actually verified
    findings: list = field(default_factory=list)
    lines: list = field(default_factory=list)     # extra printout (eye tables, grids)
    data: dict = field(default_factory=dict)
    hard: bool = False                            # later stages cannot run

    def add(self, status: str, category: str, scope: str, message: str,
            hard: bool = False, **fmt) -> Finding:
        cause, check = CAUSES.get(category, ("", ""))
        fmt.setdefault("hint", "")
        try:
            cause, check = cause.format(**fmt), check.format(**fmt)
        except (KeyError, IndexError, ValueError):
            pass  # an unfilled placeholder is better printed raw than lost
        f = Finding(status, category, scope, message, cause, check)
        self.findings.append(f)
        self.hard |= hard and status == "FAIL"
        return f

    def finish(self) -> "Stage":
        if self.status != "SKIP":
            worst = max((RANK[f.status] for f in self.findings), default=0)
            self.status = {0: "PASS", 1: "WARN", 2: "FAIL"}[worst]
        return self

    def categories(self, status: str = None) -> set:
        return {f.category for f in self.findings if status is None or f.status == status}

    def as_dict(self) -> dict:
        d = asdict(self)
        d.pop("lines")
        return d


@dataclass
class Options:
    serial: str | None = None
    baud: int = C.UART_BAUD
    timeout: float = 1.0
    spi_div: int = 0
    phase: int | None = None          # None: auto
    rate: float = 2000.0
    lower: float = 1.0
    upper: float = 500.0
    dsp: bool = True
    dsp_cutoff: float = 1.0
    seconds: float = 2.0
    snapshots: int = 20
    skip_loopback: bool = False
    leave_running: bool = False
    resync_settle: float = 0.05


def port_hint(port: str | None) -> str:
    """The Arty's FTDI shows up twice; the first interface is JTAG and is silent."""
    if port and "usbserial-" in port and port.endswith("0"):
        return f"; this looks like the FTDI's JTAG interface: try {port[:-1]}1"
    return ""


def _hex(v: int) -> str:
    return f"0x{v:04x}"


# ---------------------------------------------------------------------------
# The bring-up
# ---------------------------------------------------------------------------

class Bringup:
    STAGES = (("S0", "FPGA"), ("S1", "Self-test"), ("S2", "Bus activity"),
              ("S3", "Identity + eye"), ("S4", "Configure"), ("S5", "Calibrate"),
              ("S6", "Convert snapshot"), ("S7", "Streaming"))

    def __init__(self, open_ctrl, opts: Options, out=print, sleep=time.sleep,
                 clock=time.monotonic):
        self.open_ctrl = open_ctrl
        self.o = opts
        self.out = out
        self.sleep = sleep
        self.clock = clock
        self.ctrl = None
        self.diag = None
        self.caps = None
        self.stages: list[Stage] = []
        self.spi_div = opts.spi_div
        self.phase = opts.phase
        self.window = None
        self.line_ok = [True, True]            # S2: miso c toggles
        self.lane_ok = [False] * D.N_LANES     # S3: lane reads its ROM at the chosen phase
        self.chip_cfg_ok = [False, False]      # S4
        self.cfg = None

    # --- driver -----------------------------------------------------------
    def run(self, only=None) -> dict:
        fns = {"S0": self.s0_fpga, "S1": self.s1_selftest, "S2": self.s2_bus,
               "S3": self.s3_identity, "S4": self.s4_configure, "S5": self.s5_calibrate,
               "S6": self.s6_convert, "S7": self.s7_streaming}
        stopped = None
        for sid, name in self.STAGES:
            if only is not None and sid not in only:
                continue
            if stopped is not None:
                st = Stage(sid, name, status="SKIP", summary=f"not run: {stopped} failed")
                self.stages.append(st)
                continue
            st = self._stage(sid, name, fns[sid])
            if st.hard:
                stopped = sid
        return self.report()

    def _stage(self, sid, name, fn) -> Stage:
        st = Stage(sid, name)
        try:
            fn(st)
        except D.DiagRejected as e:
            st.add("FAIL", "DIAG_REJECTED", "board", str(e), hard=True)
        except D.DiagTimeout as e:
            st.add("FAIL", "SPI_NOT_RUNNING", "board", str(e), hard=True)
        except CtrlError as e:
            st.add("FAIL", "UART_LOST", "board", str(e), hard=True)
        except Exception as e:  # never lose the report to a case nobody foresaw
            st.add("FAIL", "HOST_ERROR", "host", f"{type(e).__name__}: {e}", hard=True)
            st.data["traceback"] = traceback.format_exc()
        st.finish()
        self.stages.append(st)
        self._print_stage(st)
        return st

    def _print_stage(self, st: Stage) -> None:
        head = f"{st.id} {st.name} ".ljust(24, ".")
        self.out(f"\n{head} {st.status}  {st.summary}".rstrip())
        for line in st.lines:
            self.out(line)
        for f in st.findings:
            if f.status == "PASS":
                continue
            self.out(f"  [{f.status}] {f.scope}: {f.message}")
            if f.status in ("WARN", "FAIL") and f.cause:
                self.out(f"         LIKELY CAUSE: {f.cause}")
            if f.status in ("WARN", "FAIL") and f.check:
                self.out(f"         CHECK: {f.check}")

    # --- helpers ----------------------------------------------------------
    def _set_reg(self, name: str, value: int) -> int:
        self.ctrl.write(name, value)
        return self.ctrl.read(name)

    def _set_div(self, div: int) -> None:
        got = self._set_reg("SPI_DIV", div)
        if got != div:
            raise CtrlError(f"SPI_DIV wrote {div}, reads back {got}")
        self.spi_div = div

    def _rom_capture(self):
        cap = self.diag.run(R.rom_commands(), raw=True)
        return cap, cap.results()

    def _lanes_of(self, chips) -> list[int]:
        return [l for l in range(D.N_LANES) if D.chip_of(l) in chips]

    # --- S0 ---------------------------------------------------------------
    def s0_fpga(self, st: Stage) -> None:
        hint = port_hint(self.o.serial)
        try:
            self.ctrl = self.open_ctrl()
        except ImportError as e:
            st.add("FAIL", "PYSERIAL_MISSING", "host", str(e), hard=True)
            return
        except Exception as e:  # pyserial's SerialException, OSError, ...
            st.add("FAIL", "PORT_OPEN", "host", f"{self.o.serial}: {e}", hard=True, hint=hint)
            return
        self.diag = D.Diag(self.ctrl, sleep=self.sleep, clock=self.clock)
        try:
            self.ctrl.resync(self.o.resync_settle)
            idv = self.ctrl.read("ID")
        except CtrlError as e:
            msg = str(e)
            if "no complete reply" in msg:
                cat = "NO_REPLY"
            elif "rejected" in msg:
                cat = "ID_REJECTED"
            else:
                cat = "GARBLED_REPLY"
            st.add("FAIL", cat, "board", f"R 1f: {msg}", hard=True, hint=hint)
            return
        st.data["id"] = f"0x{idv:08x}"
        if idv == OLD_ID:
            st.add("FAIL", "OLD_BITSTREAM", "board", f"ID reads 0x{idv:08x}", hard=True)
            return
        if idv != C.ID_EXPECTED:
            st.add("FAIL", "WRONG_BITSTREAM", "board",
                   f"ID reads 0x{idv:08x}, want 0x{C.ID_EXPECTED:08x}", hard=True)
            return
        st.checked.append(f"ID == 0x{C.ID_EXPECTED:08x}")
        self.caps = D.decode_caps(self.ctrl.read("CAPS"))
        st.data["caps"] = asdict(self.caps)
        if self.caps.taps != C.CAP_TAPS:
            st.add("FAIL", "CONTRACT_MISMATCH", "board",
                   f"CAPS says {self.caps.taps} taps per cycle, the contract {C.CAP_TAPS}",
                   hard=True)
            return
        st.checked.append(f"CAPS taps == {C.CAP_TAPS}")
        ctrl = self.ctrl.read("CTRL")
        if ctrl & 1:
            self.ctrl.set_run(False)
            st.add("INFO", "RUN_WAS_SET", "board",
                   "run was set (streaming); cleared it, the diagnostic list needs it clear")
        try:
            self._set_div(self.o.spi_div)
        except CtrlError as e:
            st.add("FAIL", "CONTRACT_MISMATCH", "board", f"SPI_DIV: {e}", hard=True)
            return
        st.checked.append(f"SPI_DIV {self.spi_div} written and read back")
        st.summary = (f"ID 0x{idv:08x}, {self.caps.taps} taps, "
                      f"emulator {'yes' if self.caps.emulator else 'no'}, "
                      f"SCLK {R.sclk_hz(self.spi_div) / 1e6:.1f} MHz")

    # --- S1 ---------------------------------------------------------------
    def s1_selftest(self, st: Stage) -> None:
        if self.o.skip_loopback:
            st.status, st.summary = "SKIP", "--skip-loopback"
            return
        if not self.caps.emulator:
            st.status, st.summary = "SKIP", "CAPS bit8 clear: this bitstream has no on-die emulator"
            return
        mode = self.ctrl.read("MODE")
        self.ctrl.write("MODE", mode | 0x8)
        try:
            if not D.read_miso(self.ctrl).loopback:
                st.add("FAIL", "SELFTEST_FAIL", "board",
                       "MODE bit3 set but MISO_STAT bit8 (loopback active) reads 0", hard=True)
                return
            cap, res = self._rom_capture()
            eye = D.eye_scan(res, D.rom_expected_by_lane())
            st.data["eye"] = eye.as_dict()
            st.lines.append(eye.table(f"loopback eye at SPI_DIV {self.spi_div} "
                                      f"(# = every ROM word right)"))
            bad = D.crosscheck(cap)
            if bad:
                m = bad[0]
                st.add("FAIL", "DECODE_MISMATCH", "board",
                       f"{len(bad)} words differ; first: cycle {m.cycle} {D.LANES[m.lane]} "
                       f"FPGA {_hex(m.fpga)} host {_hex(m.host)} at PHASE_SELECT {cap.phase}",
                       hard=True)
                return
            st.checked.append("FPGA decode == host decode of the raw taps, every word")
            missing = [l for l in range(D.N_LANES) if not eye.passing(l)]
            if missing:
                st.add("FAIL", "SELFTEST_FAIL", "board",
                       "no phase reads the emulator's ROM on " +
                       ", ".join(D.LANES[l] for l in missing), hard=True)
                return
            choice = D.choose_phase(eye)
            if choice.phase is None:
                st.add("FAIL", "SELFTEST_FAIL", "board",
                       "lanes read the emulator at different phases", hard=True)
                return
            st.checked.append("ROM read on all 4 lanes through the on-die emulator")
            st.summary = f"emulator ROM on 4 lanes, window {choice.window[0]}-{choice.window[1]}"
        finally:
            self.ctrl.write("MODE", self.ctrl.read("MODE") & ~0x8)
            self.diag.forget()
            if D.read_miso(self.ctrl).loopback:
                st.add("FAIL", "LOOPBACK_STUCK", "board", "MISO_STAT bit8 still set", hard=True)

    # --- S2 ---------------------------------------------------------------
    def s2_bus(self, st: Stage) -> None:
        cmds = R.rom_commands()
        self.ctrl.clear_counters()
        before = self.ctrl.read("SPI_CYCLES")
        self.diag.run(cmds, raw=False, cycles=())
        after = self.ctrl.read("SPI_CYCLES")
        moved = (after - before) & 0xFFFFFFFF
        st.data["spi_cycles"] = {"before": before, "after": after, "list": len(cmds)}
        if moved == 0:
            st.add("FAIL", "SPI_NOT_RUNNING", "board",
                   f"SPI_CYCLES did not move over a {len(cmds)}-command list", hard=True)
            return
        if moved != len(cmds):
            st.add("FAIL", "SPI_CYCLE_COUNT", "board",
                   f"SPI_CYCLES moved by {moved} for a {len(cmds)}-command list", hard=True)
            return
        st.checked.append(f"SPI_CYCLES advanced by exactly {len(cmds)}")
        ms = D.read_miso(self.ctrl)
        st.data["miso"] = ms.as_dict()
        if ms.loopback:
            st.add("FAIL", "LOOPBACK_STILL_ON", "board", "MISO_STAT bit8 is set", hard=True)
            return
        states = [ms.line(c) for c in (0, 1)]
        for c, s in enumerate(states):
            scope = f"chip{c}"
            self.line_ok[c] = s == "toggling"
            if s == "toggling":
                st.checked.append(f"miso{c} toggles ({ms.edges[c]} edges)")
            elif s == "stuck_low":
                st.add("FAIL", "MISO_STUCK_LOW", scope,
                       f"miso{c} never high while CS was low ({ms.edges[c]} edges)", c=c)
            elif s == "stuck_high":
                st.add("FAIL", "MISO_STUCK_HIGH", scope,
                       f"miso{c} never low while CS was low ({ms.edges[c]} edges)", c=c)
            else:
                st.add("FAIL", "MISO_UNSAMPLED", scope, f"miso{c}: neither level seen",
                       hard=True, c=c)
        if not any(self.line_ok):
            if states[0] == states[1] and states[0].startswith("stuck"):
                st.add("FAIL", "BOTH_STUCK", "both chips",
                       f"both lines {states[0].replace('_', ' ')}", hard=True)
            st.hard = True
        st.summary = ", ".join(f"miso{c} {s.replace('_', ' ')}" for c, s in enumerate(states))

    # --- S3 ---------------------------------------------------------------
    def s3_identity(self, st: Stage) -> None:
        expected = D.rom_expected_by_lane()
        want_lanes = self._lanes_of([c for c in (0, 1) if self.line_ok[c]])
        req = self.spi_div
        attempts = []
        for div in [req] + [d for d in RETRY_DIVS if d != req]:
            self._set_div(div)
            cap, res = self._rom_capture()
            eye = D.eye_scan(res, expected)
            choice = cover_phase(eye)
            att = {"spi_div": div, "cap": cap, "res": res, "eye": eye, "choice": choice}
            attempts.append(att)
            st.lines.append(eye.table(f"eye at SPI_DIV {div} "
                                      f"(SCLK {R.sclk_hz(div) / 1e6:.1f} MHz; # = every ROM "
                                      f"word right, n = n words right)"))
            if (set(want_lanes) <= set(choice.lanes) and choice.window
                    and choice.window[1] > choice.window[0]):
                break
            if not want_lanes:
                break      # S2 found no live line; a slower clock cannot help
        best = max(attempts, key=lambda a: (len(a["choice"].lanes),
                                            _width(a["choice"].window),
                                            a["spi_div"] == req, -a["spi_div"]))
        st.data["attempts"] = [{"spi_div": a["spi_div"], "choice": a["choice"].as_dict(),
                                "eye": a["eye"].as_dict()} for a in attempts]
        if best is not attempts[-1]:
            self._set_div(best["spi_div"])
        cap, res, eye, choice = best["cap"], best["res"], best["eye"], best["choice"]

        # Cross-check the FPGA's decode before anything relies on it.
        bad = D.crosscheck(cap)
        if bad:
            m = bad[0]
            st.add("FAIL", "DECODE_MISMATCH", "board",
                   f"{len(bad)} words differ; first: cycle {m.cycle} {D.LANES[m.lane]} FPGA "
                   f"{_hex(m.fpga)} host {_hex(m.host)} at PHASE_SELECT {cap.phase}", hard=True)
            return
        st.checked.append("FPGA decode == host decode of the raw taps, every word")

        # The phase.
        phase = choice.phase
        if self.o.phase is not None:
            if phase is not None and not all(self.o.phase in eye.passing(l)
                                             for l in choice.lanes):
                st.add("FAIL", "PHASE_OUTSIDE_WINDOW", "board",
                       f"--phase {self.o.phase}; the scan would choose {phase}",
                       p=self.o.phase)
            phase = self.o.phase
        self.phase = phase
        self.window = choice.window
        for lane in range(D.N_LANES):
            self.lane_ok[lane] = phase is not None and phase in eye.passing(lane)

        # Every lane that does not read its ROM at that phase gets a cause.
        swapped = D.eye_scan(res, expected, swap=True)
        inverted = D.eye_scan(res, expected, invert=True)
        swap_chips = swapped_chips(swapped, self.lane_ok)
        for lane in range(D.N_LANES):
            if self.lane_ok[lane]:
                continue
            if D.chip_of(lane) in swap_chips:
                c = D.chip_of(lane)
                both = sorted(set(swapped.passing(2 * c)) & set(swapped.passing(2 * c + 1)))
                st.add("FAIL", "AB_SWAPPED", D.LANES[lane],
                       f"reads {D.LANES[lane ^ 1]}'s answers (A/B marker "
                       f"{R.MARKER_B if lane % 2 == 0 else R.MARKER_A}) at phases "
                       f"{_wins(D.runs(both))}", c=c)
                continue
            if eye.passing(lane):
                st.add("FAIL", "NO_COMMON_WINDOW", D.LANES[lane],
                       f"reads its ROM at phases {_wins(eye.windows(lane))}, not at "
                       f"{phase}; other lanes at {_wins(runs_for(eye, choice.lanes))}")
                continue
            elsewhere = [(a["spi_div"], a["eye"].windows(lane)) for a in attempts
                         if a is not best and a["eye"].passing(lane)]
            if elsewhere:
                st.add("FAIL", "NO_COMMON_WINDOW", D.LANES[lane],
                       "reads its ROM only at " +
                       "; ".join(f"SPI_DIV {d} phases {_wins(w)}" for d, w in elsewhere) +
                       ", never at a phase the other lanes share")
                continue
            cat, msg, fmt = classify_lane(res, expected, lane, eye, swapped, inverted,
                                          self.spi_div)
            st.add("FAIL", cat, D.LANES[lane], msg, **fmt)
        self._chip_level(st, eye)

        if best["spi_div"] != req and any(self.lane_ok):
            # Say why the requested SPI_DIV failed: that is the physical finding.
            first = attempts[0]
            fe, fc = first["eye"], first["choice"]
            sw = D.eye_scan(first["res"], expected, swap=True)
            inv = D.eye_scan(first["res"], expected, invert=True)
            for lane in range(D.N_LANES):
                if not self.lane_ok[lane] or lane in fc.lanes:
                    continue
                if fe.passing(lane):
                    cat, msg, fmt = ("NO_COMMON_WINDOW",
                                     f"reads at phases {_wins(fe.windows(lane))}, the others "
                                     f"at {_wins(runs_for(fe, fc.lanes))}", {})
                else:
                    cat, msg, fmt = classify_lane(first["res"], expected, lane, fe, sw, inv,
                                                  req)
                st.add("INFO", cat, D.LANES[lane], f"at SPI_DIV {req}: {msg} ({cat})", **fmt)
            st.add("WARN", "SPI_DIV_CHANGED", "board",
                   f"SPI_DIV {req} read {len(attempts[0]['choice'].lanes)} lane(s); "
                   f"using SPI_DIV {best['spi_div']} "
                   f"({R.sclk_hz(best['spi_div']) / 1e6:.1f} MHz SCLK)",
                   req=req, div=best["spi_div"])
        elif best["spi_div"] == req and len(attempts) > 1:
            st.lines.append(f"  SPI_DIV {', '.join(str(a['spi_div']) for a in attempts[1:])} "
                            f"tried as well and did not read more lanes: not a timing problem")
        if choice.window and _width(choice.window) == 1 and self.o.phase is None:
            st.add("WARN", "NARROW_WINDOW", "board",
                   f"common window is the single phase {choice.window[0]}")

        if phase is None:
            st.hard = True     # every lane already carries its own FAIL and cause
            st.summary = "no lane reads its ROM: nothing to configure"
            return
        got = self._set_reg("PHASE_SELECT", phase)
        if got != phase:
            st.add("FAIL", "CONTRACT_MISMATCH", "board",
                   f"PHASE_SELECT wrote {phase}, reads back {got}", hard=True)
            return
        # Confirm with the FPGA's own decode at the chosen phase.
        confirm = self.diag.run(R.rom_commands(), raw=False).results()
        for lane in range(D.N_LANES):
            if not self.lane_ok[lane]:
                continue
            wrong = [i for i, w in expected[lane].items() if confirm[i].words[lane] != w]
            if wrong:
                st.add("FAIL", "DECODE_MISMATCH", D.LANES[lane],
                       f"the host decode read the ROM at phase {phase} but the FPGA's "
                       f"DIAG_RES did not ({len(wrong)} words)", hard=True)
                self.lane_ok[lane] = False
        ok = [D.LANES[l] for l in range(D.N_LANES) if self.lane_ok[l]]
        if ok:
            st.checked.append(f"ROM ('INTAN', marker, ID 4) read by the FPGA at PHASE_SELECT "
                              f"{phase} on: {', '.join(ok)}")
        die = {}
        idx60 = R.ROM_LIST_REGS.index(R.DIE_REV_REG)
        for c in (0, 1):
            if self.lane_ok[2 * c]:
                die[f"chip{c}"] = confirm[idx60].words[2 * c] & 0xFF
        st.data.update({"spi_div": self.spi_div, "phase": phase, "window": choice.window,
                        "lanes_ok": {D.LANES[l]: self.lane_ok[l] for l in range(D.N_LANES)},
                        "die_revision": die})
        st.summary = (f"SPI_DIV {self.spi_div}, PHASE_SELECT {phase}"
                      + (f" (window {choice.window[0]}-{choice.window[1]})"
                         if choice.window else "")
                      + f", {len(ok)}/4 lanes read their ROM")
        if not ok:
            st.hard = True

    def _chip_level(self, st: Stage, eye: D.Eye) -> None:
        dead = [c for c in (0, 1) if not self.lane_ok[2 * c] and not self.lane_ok[2 * c + 1]]
        if len(dead) == 1:
            c = dead[0]
            st.add("FAIL", "CHIP_NOT_ANSWERING", f"chip{c}",
                   f"only chip {1 - c} reads its ROM", c=c, other=1 - c)
        for c in (0, 1):
            a, b = 2 * c, 2 * c + 1
            if self.lane_ok[a] and not self.lane_ok[b] and not eye.passing(b):
                cats = {f.category for f in st.findings if f.scope == D.LANES[b]}
                if not cats & {"NOT_RHD2164"}:
                    st.add("FAIL", "B_ONLY_FAIL", D.LANES[b],
                           "module B fails while module A on the same wire reads its ROM",
                           lane=D.LANES[b])

    # --- S4 ---------------------------------------------------------------
    def s4_configure(self, st: Stage) -> None:
        o = self.o
        self.cfg = R.AmpConfig(rate=o.rate, lower=o.lower, upper=o.upper, dsp=o.dsp,
                               dsp_cutoff=o.dsp_cutoff, spi_div=self.spi_div)
        for w in R.config_warnings(self.cfg):
            st.add("WARN", "CONFIG_WARNING", "config", w)
        values = R.register_values(self.cfg)
        st.data["config"] = R.config_summary(self.cfg)
        res = self.diag.run(R.configure_commands(self.cfg), raw=False).results()
        n = R.N_RAM
        corrupt = []
        for c in (0, 1):
            la, lb = 2 * c, 2 * c + 1
            scope = f"chip{c}"
            if not self.lane_ok[la]:
                st.add("INFO", "INFO", scope, "not checked: module A did not read its ROM in S3")
                continue
            echo_a = {r: res[r].words[la] for r in range(n)}
            bad_echo = {r: w for r, w in echo_a.items() if w != 0xFF00 | values[r]}
            bad_read_a = {r: res[n + r].words[la] for r in R.A_REGS
                          if res[n + r].words[la] != values[r]}
            bad_read_b, bad_echo_b = {}, {}
            if self.lane_ok[lb]:
                bad_read_b = {r: res[n + r].words[lb] for r in R.B_ONLY_REGS
                              if res[n + r].words[lb] != values[r]}
                bad_echo_b = {r: res[r].words[lb] for r in range(n)
                              if res[r].words[lb] != 0xFF00 | values[r]}
            if bad_echo:
                corrupt.append(c)
                st.add("FAIL", "MOSI_CORRUPT", scope,
                       f"{len(bad_echo)}/{n} echoes wrong: " + _echo_detail(bad_echo, values),
                       c=c)
            elif bad_read_a or bad_read_b:
                regs = sorted(bad_read_a) + sorted(bad_read_b)
                got = {**bad_read_a, **bad_read_b}
                st.add("FAIL", "REG_READBACK", scope,
                       f"echo right, read-back wrong on {len(regs)} register(s): " +
                       ", ".join(f"{r} (wrote 0x{values[r]:02x}, read {_hex(got[r])})"
                                 for r in regs[:5]) + (" ..." if len(regs) > 5 else ""),
                       c=c)
            else:
                self.chip_cfg_ok[c] = True
                st.checked.append(f"chip {c}: 22 echoes (A) and read-back of 0-17 (A)"
                                  + (", 18-21 (B)" if self.lane_ok[lb] else ""))
                if not self.lane_ok[lb]:
                    st.add("WARN", "INFO", scope,
                           "registers 18-21 not verified: module B did not read its ROM")
            if bad_echo_b and not bad_echo:
                st.add("WARN", "B_ECHO_DIFFERS", f"chip{c} B",
                       _echo_detail(bad_echo_b, values))
        if len(corrupt) == 2:
            st.findings.append(Finding(
                "FAIL", "MOSI_CORRUPT", "both chips", "both chips see corrupted writes",
                "a cause shared by both chips: the MOSI pair itself or its LVDS driver",
                "the MOSI pair P/N and continuity; --spi-div 1 to test setup time"))
        checked = sum(1 for c in (0, 1) if self.lane_ok[2 * c])
        st.summary = (f"{sum(self.chip_cfg_ok)}/{checked} checked chip(s) configured "
                      f"(upper {st.data['config']['upper_actual_hz']:g} Hz, lower "
                      f"{st.data['config']['lower_actual_hz']:g} Hz, DSP "
                      f"{st.data['config']['dsp_cutoff_hz']:g} Hz)")

    # --- S5 ---------------------------------------------------------------
    def s5_calibrate(self, st: Stage) -> None:
        cmds = R.calibrate_commands()
        res = self.diag.run(cmds, raw=False).results()
        resume = [1 + R.CALIBRATE_DUMMIES + k for k in range(2)]
        st.checked.append(f"CALIBRATE + {R.CALIBRATE_DUMMIES} dummies sent (well over 100 us "
                          f"after S4: UART round trips take milliseconds)")
        for c in (0, 1):
            lanes = [l for l in (2 * c, 2 * c + 1) if self.lane_ok[l]]
            if not lanes:
                continue
            wrong = [(D.LANES[l], _hex(res[i].words[l])) for l in lanes for i in resume
                     if res[i].words[l] != R.CHIP_ID_RHD2164]
            if wrong:
                st.add("FAIL", "CALIBRATE_NO_RESUME", f"chip{c}",
                       "after the window: " + ", ".join(f"{l} {w}" for l, w in wrong), c=c)
            else:
                st.checked.append(f"chip {c} answers READ(63) = 4 after the window")
        window = range(1, 1 + R.CALIBRATE_DUMMIES)
        for c in (0, 1):
            if self.lane_ok[2 * c]:
                normal = sum(res[i].words[2 * c] == R.CHIP_ID_RHD2164 for i in window)
                st.data[f"chip{c}_window_answers"] = [_hex(res[i].words[2 * c]) for i in window]
                if normal == R.CALIBRATE_DUMMIES:
                    st.add("INFO", "INFO", f"chip{c}",
                           "all 9 dummies answered like a normal READ(63): no sign the chip "
                           "treated them as calibration (an RHD2164's answers in the window are "
                           "not specified, so this is only a hint; see S4 for MOSI)")
        st.summary = "the chip gives no calibration read-back; checked it resumes after"

    # --- S6 ---------------------------------------------------------------
    def s6_convert(self, st: Stage) -> None:
        k = max(2, self.o.snapshots)
        snaps = [D.convert_snapshot(self.diag) for _ in range(k)]
        stats = channel_stats(snaps)
        judged = [self.lane_ok[ch // 32] for ch in range(C.N_CH)]
        flags = channel_flags(stats, judged)
        st.data["channels"] = [{"ch": ch, **stats[ch], "flag": flags[ch]}
                               for ch in range(C.N_CH)]
        st.lines.append(grid(stats, flags, k))
        dead_lanes = []
        for lane in range(D.N_LANES):
            if not self.lane_ok[lane]:
                continue
            chans = range(32 * lane, 32 * lane + 32)
            if all(stats[ch]["min_lsb"] == stats[ch]["max_lsb"] for ch in chans):
                dead_lanes.append(lane)
        for c in (0, 1):
            pair = [2 * c, 2 * c + 1]
            if all(l in dead_lanes for l in pair):
                st.add("FAIL", "CHIP_DEAD", f"chip{c}", "all 64 amplifiers constant", c=c)
                continue
            for lane in pair:
                if lane in dead_lanes:
                    vals = {stats[ch]["min"] for ch in range(32 * lane, 32 * lane + 32)}
                    st.add("FAIL", "MODULE_DEAD", D.LANES[lane],
                           "every channel constant across all snapshots",
                           lane=D.LANES[lane], c=c,
                           value=", ".join(f"{v:g} uV" for v in sorted(vals)[:3]),
                           regs="14-17" if lane % 2 == 0 else "18-21")
        dead_set = {ch for l in dead_lanes for ch in range(32 * l, 32 * l + 32)}
        for flag, cat in (("S", "STUCK_CHANNELS"), ("R", "RAILED_CHANNELS"),
                          ("N", "NOISY_CHANNELS")):
            chans = [ch for ch in range(C.N_CH) if flags[ch] == flag and ch not in dead_set]
            if chans:
                st.add("WARN", cat, "channels", f"{len(chans)}: {_ranges(chans)}")
        judged_n = sum(judged)
        good = sum(1 for ch in range(C.N_CH) if judged[ch] and flags[ch] == ".")
        st.checked.append(f"{k} CONVERT sweeps of {judged_n} channels on readable lanes")
        st.summary = f"{good}/{judged_n} judged channels unremarkable over {k} snapshots"

    # --- S7 ---------------------------------------------------------------
    def s7_streaming(self, st: Stage) -> None:
        rate = self.o.rate
        period = round(C.SYS_CLK_HZ / rate)
        minp = R.min_sample_period(self.spi_div)
        if period < minp:
            st.add("FAIL", "RATE_TOO_HIGH", "config",
                   f"SAMPLE_PERIOD {period} < {minp}", rate=rate, minp=minp, div=self.spi_div)
            return
        self.ctrl.write("SPI_DIV", self.spi_div)
        self.ctrl.write("PHASE_SELECT", self.phase)
        self.ctrl.write("SAMPLE_PERIOD", period)
        self.ctrl.write("MODE", 0)
        self.ctrl.clear_counters()
        t_a = self.clock()
        self.ctrl.set_run(True)
        t_b = self.clock()
        self.sleep(self.o.seconds)
        t_c = self.clock()
        if self.o.leave_running:
            link = D.read_link(self.ctrl)
            t_d = self.clock()
        else:
            self.ctrl.set_run(False)
            t_d = self.clock()
            link = D.read_link(self.ctrl)
        cnt = self.ctrl.read_many(("FRAMES", "DGRAMS", "OVERRUNS", "STALLS", "FIFO_HIGH",
                                   "SPI_CYCLES"))
        sweep_s = period / C.SYS_CLK_HZ
        lo, hi = 0.9 * (t_c - t_b) / sweep_s - 2, 1.1 * (t_d - t_a) / sweep_s + 2
        st.data.update({"sample_period": period, "seconds": round(t_d - t_a, 3),
                        "link": link.as_dict(), "counters": cnt,
                        "expected_checks": [round(lo), round(hi)],
                        "left_running": self.o.leave_running})
        if link.checks == 0:
            st.add("FAIL", "SEQUENCER_IDLE", "board", "LINK_CHECKS 0")
            return
        if not lo <= link.checks <= hi:
            st.add("WARN", "LINK_CHECK_COUNT", "board", f"{link.checks} checks",
                   got=link.checks, lo=lo, hi=hi)
        else:
            st.checked.append(f"LINK_CHECKS {link.checks} = rate x time")
        for lane in range(D.N_LANES):
            n = link.errors[lane]
            name = D.LANES[lane]
            if not self.lane_ok[lane]:
                if n:
                    st.add("INFO", "INFO", name,
                           f"{n} link errors, expected: this lane failed S3")
                continue
            if n == 0:
                st.checked.append(f"{name}: 0 link errors")
            elif n >= min(link.checks, 0xFFFF):
                st.add("FAIL", "LINK_ERRORS", name, f"{n} errors in {link.checks} checks",
                       lane=name, n=n, last=link.last[lane])
            else:
                st.add("FAIL", "LINK_ERRORS_INTERMITTENT", name,
                       f"{n} errors in {link.checks} checks", lane=name, n=n,
                       m=link.checks, last=link.last[lane])
        if cnt["FRAMES"] == 0:
            st.add("FAIL", "NO_FRAMES", "board", f"FRAMES 0 after {link.checks} sweeps")
        else:
            st.checked.append(f"FRAMES {cnt['FRAMES']}")
        want_dgrams = cnt["FRAMES"] // C.FRAMES_PER_DGRAM
        if want_dgrams and cnt["DGRAMS"] >= 0.9 * (want_dgrams - cnt["OVERRUNS"]) \
                and cnt["DGRAMS"] > 4:
            st.checked.append(f"ESP draining QSPI: DGRAMS {cnt['DGRAMS']}")
            if cnt["OVERRUNS"]:
                st.add("WARN", "OVERRUNS", "esp", f"OVERRUNS {cnt['OVERRUNS']}",
                       n=cnt["OVERRUNS"])
        else:
            st.add("INFO", "ESP_NOT_DRAINING", "esp",
                   f"no ESP draining QSPI (DGRAMS {cnt['DGRAMS']}, STALLS {cnt['STALLS']}, "
                   f"OVERRUNS {cnt['OVERRUNS']}): expected without the ESP",
                   dgrams=cnt["DGRAMS"], stalls=cnt["STALLS"], overruns=cnt["OVERRUNS"])
        bad = sum(1 for l in range(D.N_LANES) if self.lane_ok[l] and link.errors[l])
        st.summary = (f"{link.checks} sweeps at {rate:g} S/s, {bad} lane(s) with link errors"
                      + (", left running" if self.o.leave_running else ""))

    # --- report -----------------------------------------------------------
    def report(self) -> dict:
        worst = max((RANK[s.status] for s in self.stages), default=0)
        return {
            "tool": "chain.bringup",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "result": {0: "PASS", 1: "WARN", 2: "FAIL"}[worst],
            "port": self.o.serial,
            "options": asdict(self.o),
            "chosen": {"spi_div": self.spi_div, "phase": self.phase, "window": self.window,
                       "sclk_hz": R.sclk_hz(self.spi_div)},
            "lanes": {D.LANES[l]: self.lane_ok[l] for l in range(D.N_LANES)},
            "config": R.config_summary(self.cfg) if self.cfg else None,
            "stages": [s.as_dict() for s in self.stages],
        }

    def summary(self, report: dict, path=None) -> str:
        lines = ["", "=" * 30 + " bring-up summary " + "=" * 30]
        for s in self.stages:
            lines.append(f" {s.id} {s.name:<17} {s.status:<5} {s.summary}"[:100])
            for f in s.findings:
                if f.status == "FAIL":
                    lines.append(f"      FAIL {f.category}: {f.scope}")
        ch = report["chosen"]
        lines.append(f" chosen: SPI_DIV {ch['spi_div']} (SCLK {ch['sclk_hz'] / 1e6:.1f} MHz), "
                     f"PHASE_SELECT {ch['phase']}")
        lines.append(" lanes:  " + "  ".join(f"{k} {'ok' if v else '--'}"
                                            for k, v in report["lanes"].items()))
        lines.append(f" result: {report['result']}" + (f"   report: {path}" if path else ""))
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Pure helpers (tested directly)
# ---------------------------------------------------------------------------

def _width(w) -> int:
    return 0 if w is None else w[1] - w[0] + 1


def _wins(ws) -> str:
    return ", ".join(f"{a}-{b}" if a != b else f"{a}" for a, b in ws) or "none"


def runs_for(eye: D.Eye, lanes) -> list:
    common = set(D.PHASES)
    for l in lanes:
        common &= set(eye.passing(l))
    return D.runs(common)


def cover_phase(eye: D.Eye) -> D.PhaseChoice:
    """choose_phase over every present lane, or, when they share no phase, the
    phase that serves the most lanes (the widest such run, then its centre)."""
    choice = D.choose_phase(eye)
    if choice.phase is not None or not eye.present():
        return choice
    sets = {tuple(l for l in eye.present() if p in eye.passing(l)) for p in D.PHASES}
    most = max(len(s) for s in sets)
    subs = [D.choose_phase(eye, s) for s in sets if len(s) == most]
    sub = max(subs, key=lambda c: (_width(c.window), -c.phase))
    sub.per_lane = choice.per_lane
    return sub


def swapped_chips(swapped: D.Eye, lane_ok) -> set:
    """Chips whose two lanes both read the other module's answers at a common phase.

    With A and B traded, A's window lands four ticks from B's (they share one
    wire, so normally the windows coincide) and each lane can pass somewhere on
    its own; the swapped decode passing both at once is the signature.
    """
    out = set()
    for c in (0, 1):
        a, b = 2 * c, 2 * c + 1
        if lane_ok[a] or lane_ok[b]:
            continue
        if set(swapped.passing(a)) & set(swapped.passing(b)):
            out.add(c)
    return out


def classify_lane(res, expected, lane: int, eye: D.Eye, swapped: D.Eye, inverted: D.Eye,
                  spi_div: int):
    """(category, message, format args) for a lane with no passing phase."""
    c = D.chip_of(lane)
    name = D.LANES[lane]
    fmt = {"c": c, "lane": name, "div": spi_div}
    taps = D.chip_taps(res, c)
    if taps and all(t == 0 for t in taps):
        return "ROM_ALL_ZERO", "reads 0x0000 at every phase", fmt
    if taps and all(t == D.TAPS_MASK for t in taps):
        return "ROM_ALL_ONES", "reads 0xffff at every phase", fmt
    if inverted.passing(lane):
        return ("MISO_INVERTED", f"inverted taps read the ROM at phases "
                f"{_wins(inverted.windows(lane))}", fmt)
    if swapped.passing(lane):
        other = D.LANES[lane ^ 1]
        return ("AB_SWAPPED", f"reads {other}'s ROM answers (marker "
                f"{R.MARKER_B if lane % 2 == 0 else R.MARKER_A}) at phases "
                f"{_wins(swapped.windows(lane))}", fmt)
    hits = D.offset_scan(res, expected[lane], lane)
    if hits:
        late = [p for p in hits if p > C.PHASE_MAX]
        if late:
            fmt["ticks"] = min(late) - C.PHASE_MAX
            return ("DELAY_OUT_OF_WINDOW", f"the ROM appears at phase {min(late)}-"
                    f"{max(late)}, past PHASE_SELECT {C.PHASE_MAX}", fmt)
        return "DATA_TOO_EARLY", f"the ROM appears at phase {max(hits)}", fmt
    p, m, words = D.best_partial(res, expected[lane], lane)
    regs = {i: R.ROM_LIST_REGS[i] for i in expected[lane]}
    intan_ok = all(words.get(i) == w for i, w in expected[lane].items()
                   if regs[i] in R.INTAN)
    id_idx = next(i for i, r in regs.items() if r == R.ID_REG)
    chip_id = words.get(id_idx)
    if intan_ok and chip_id is not None and chip_id != R.CHIP_ID_RHD2164 and chip_id < 0x100:
        fmt.update(id=chip_id, name=R.CHIP_NAMES.get(chip_id, "unknown part"))
        return ("NOT_RHD2164", f"'INTAN' at phase {p}, chip ID {chip_id}", fmt)
    n = len(expected[lane])
    if m > 0:
        fmt.update(m=m, n=n, p=p)
        shifts = {D.bit_shift(words[i], w) for i, w in expected[lane].items()
                  if words.get(i) != w}
        extra = ""
        if len(shifts) == 1 and None not in shifts:
            extra = f"; wrong words are the right ones shifted {shifts.pop():+d} bit(s)"
        return "ROM_PARTIAL", f"{m}/{n} words right at phase {p}{extra}", fmt
    return "ROM_GARBLED", f"no ROM word right at any phase (best phase {p})", fmt


def _echo_detail(bad: dict, values: list) -> str:
    """Which registers, and whether the damage is one consistent bit pattern."""
    xors = set()
    not_write = []
    for r, w in bad.items():
        d = R.write_echo(w)
        if d is None:
            not_write.append(r)
        else:
            xors.add(d ^ values[r])
    parts = [", ".join(f"reg {r} wrote 0x{values[r]:02x} echo {_hex(w)}"
                       for r, w in list(bad.items())[:4]) + (" ..." if len(bad) > 4 else "")]
    if len(xors) == 1 and not not_write:
        x = xors.pop()
        bits = [b for b in range(8) if x >> b & 1]
        parts.append(f"always data bit(s) {bits} flipped (xor 0x{x:02x})")
    if not_write:
        parts.append(f"{len(not_write)} answer(s) are not WRITE echoes at all: the chip did "
                     f"not see a WRITE")
    return "; ".join(parts)


def channel_stats(snaps: list[list[int]]) -> list[dict]:
    """Per channel: mean, std, min, max in microvolts, plus the raw extremes."""
    out = []
    for ch in range(len(snaps[0])):
        v = [s[ch] for s in snaps]
        uv = [x * R.SCALE_UV for x in v]
        out.append({"mean_uv": round(statistics.fmean(uv), 2),
                    "std_uv": round(statistics.pstdev(uv), 2),
                    "min": round(min(uv), 2), "max": round(max(uv), 2),
                    "min_lsb": min(v), "max_lsb": max(v)})
    return out


def channel_flags(stats: list[dict], judged: list[bool]) -> list[str]:
    """'x' not judged, 'R' railed, 'S' stuck, 'N' noisy, '.' unremarkable."""
    out = []
    for s, ok in zip(stats, judged):
        if not ok:
            out.append("x")
        elif max(abs(s["min_lsb"]), abs(s["max_lsb"])) >= RAIL_LSB:
            out.append("R")          # before stuck: a floating input often sits at the rail
        elif s["min_lsb"] == s["max_lsb"]:
            out.append("S")
        elif s["std_uv"] > NOISY_UV:
            out.append("N")
        else:
            out.append(".")
    return out


def grid(stats: list[dict], flags: list[str], k: int) -> str:
    """8 x 16: rows are 16-channel halves of each module, cells the spread in uV."""
    lines = [f"  spread over {k} snapshots, uV rms (S stuck, R railed, x lane not read)",
             "            " + "".join(f"{c:>5}" for c in range(16))]
    for row in range(8):
        lane, half = divmod(row, 2)
        name = f"c{lane // 2}{'AB'[lane % 2]} {16 * half:>2}-{16 * half + 15:<2}"
        cells = []
        for col in range(16):
            ch = 16 * row + col
            if flags[ch] in "xSR":
                cells.append(f"{flags[ch]:>5}")
            else:
                v = stats[ch]["std_uv"]
                cells.append(f"{v:5.0f}" if v < 9999.5 else f"{v / 1000:4.0f}k")
        lines.append(f"  {name:<10}" + "".join(cells))
    return "\n".join(lines)


def _ranges(chans: list[int]) -> str:
    return ", ".join(f"{a}-{b}" if a != b else f"{a}" for a, b in D.runs(chans))


# ---------------------------------------------------------------------------
# For the viewer: configure and start streaming with the same checks
# ---------------------------------------------------------------------------

def prepare(ctrl, opts: Options, out=print, sleep=time.sleep, clock=time.monotonic,
            stream: bool = False) -> tuple[Bringup, dict]:
    """S0, S2..S5 on an open Ctrl; then, if stream and nothing failed hard, run.

    The viewer uses this so that what it shows was configured the way the
    bring-up checks, not by a second, divergent recipe.
    """
    b = Bringup(lambda: ctrl, opts, out=out, sleep=sleep, clock=clock)
    rep = b.run(only=("S0", "S2", "S3", "S4", "S5"))
    if stream and not any(s.hard for s in b.stages) and b.phase is not None:
        ctrl.write("SPI_DIV", b.spi_div)
        ctrl.write("PHASE_SELECT", b.phase)
        ctrl.write("SAMPLE_PERIOD", round(C.SYS_CLK_HZ / opts.rate))
        ctrl.write("MODE", 0)
        ctrl.clear_counters()
        ctrl.set_run(True)
    return b, rep


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def phase_arg(s: str):
    if s == "auto":
        return None
    p = int(s)
    if not 0 <= p <= C.PHASE_MAX:
        raise argparse.ArgumentTypeError(f"phase must be 0..{C.PHASE_MAX} or auto")
    return p


def build_parser() -> argparse.ArgumentParser:
    from .fakeboard import FAULTS
    ap = argparse.ArgumentParser(
        prog="python3 -m chain.bringup",
        description="staged bring-up of two RHD2164s behind the FPGA, with diagnosis")
    ap.add_argument("--serial", help="FPGA control UART (the Arty's second FTDI port)")
    ap.add_argument("--baud", type=int, default=C.UART_BAUD)
    ap.add_argument("--spi-div", type=int, default=0, choices=range(C.SPI_DIV_MAX + 1),
                    metavar="N", help="SCLK = 24 MHz / (N + 1); S3 retries 1, 3, 7 if needed")
    ap.add_argument("--phase", type=phase_arg, default=None, metavar="N|auto",
                    help="PHASE_SELECT 0..15, or auto (default): centre of the widest window")
    ap.add_argument("--rate", type=float, default=2000.0, metavar="HZ",
                    help="samples/s per channel (sets DSP cutoff and SAMPLE_PERIOD)")
    ap.add_argument("--lower", type=float, default=1.0, metavar="HZ",
                    help="amplifier lower bandwidth")
    ap.add_argument("--upper", type=float, default=500.0, metavar="HZ",
                    help="amplifier upper bandwidth")
    ap.add_argument("--dsp-cutoff", type=float, default=1.0, metavar="HZ",
                    help="on-chip DSP high-pass cutoff")
    ap.add_argument("--no-dsp", action="store_true", help="disable the DSP high-pass")
    ap.add_argument("--seconds", type=float, default=2.0, metavar="S",
                    help="S7 streaming time")
    ap.add_argument("--snapshots", type=int, default=20, metavar="K",
                    help="S6 CONVERT sweeps")
    ap.add_argument("--skip-loopback", action="store_true", help="skip S1")
    ap.add_argument("--leave-running", action="store_true",
                    help="leave run set after S7 (for chain.viewer --udp)")
    ap.add_argument("--json", type=pathlib.Path, default=None, metavar="PATH",
                    help="report path (default docs/results/bringup_<time>.json)")
    ap.add_argument("--fake", nargs="*", choices=sorted(FAULTS), metavar="FAULT",
                    help="no hardware: run against chain.fakeboard with these faults "
                         f"({', '.join(sorted(FAULTS))})")
    return ap


def options_from(args) -> Options:
    return Options(serial=args.serial, baud=args.baud, spi_div=args.spi_div, phase=args.phase,
                   rate=args.rate, lower=args.lower, upper=args.upper, dsp=not args.no_dsp,
                   dsp_cutoff=args.dsp_cutoff, seconds=args.seconds,
                   snapshots=max(2, args.snapshots), skip_loopback=args.skip_loopback,
                   leave_running=args.leave_running)


def main(argv=None, out=print) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.fake is None and not args.serial:
        ap.error("--serial PORT is required (or --fake for a dry run)")
    opts = options_from(args)
    sleep, clock = time.sleep, time.monotonic
    if args.fake is not None:
        from .fakeboard import FakeBoard, VirtualClock, make_faults
        vclock = VirtualClock()
        board = FakeBoard(make_faults(*args.fake), clock=vclock)
        open_ctrl = lambda: Ctrl(ser=board)
        sleep, clock = vclock.sleep, vclock
        opts.serial = opts.serial or "fake:" + ("+".join(args.fake) or "none")
        opts.resync_settle = 0.0
    else:
        open_ctrl = lambda: Ctrl(args.serial, baud=args.baud, timeout=opts.timeout)
    out(f"bring-up on {opts.serial}: SPI_DIV {opts.spi_div}, phase "
        f"{'auto' if opts.phase is None else opts.phase}, {opts.rate:g} S/s, "
        f"{opts.lower:g}-{opts.upper:g} Hz")
    b = Bringup(open_ctrl, opts, out=out, sleep=sleep, clock=clock)
    try:
        rep = b.run()
    finally:
        if b.ctrl is not None:
            if not opts.leave_running:
                try:
                    if b.ctrl.read("CTRL") & 1:
                        b.ctrl.set_run(False)
                except CtrlError:
                    pass
            b.ctrl.close()
    path = args.json
    if path is None and args.fake is None:
        path = RESULTS / f"bringup_{time.strftime('%Y%m%d-%H%M%S')}.json"
    if args.fake is not None:
        rep["fake"] = args.fake
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(rep, indent=1, default=str) + "\n")
    out(b.summary(rep, path if path is not None else "(fake run: pass --json to save)"))
    return 0 if rep["result"] != "FAIL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
