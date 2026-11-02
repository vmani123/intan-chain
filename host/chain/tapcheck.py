"""Check every sample the chain delivers against Intan's own recording controller.

    python3 -m chain.tapcheck selftest    --serial PORT          # tap path on the bench, no Intan
    python3 -m chain.tapcheck status      --serial PORT          # is the tap wired and hearing?
    python3 -m chain.tapcheck delay-sweep --serial PORT [--apply]
    python3 -m chain.tapcheck record      --serial PORT [--seconds S] [--out DIR]
    python3 -m chain.tapcheck compare     RECORDING RHX [RHX ...]
    python3 -m chain.tapcheck status --fake tap_cs_swapped       # no hardware: see a diagnosis

The Intan RHD recording controller drives the headstage; the tap build of the
FPGA (CONTRACTS.md section 7) only listens to the same five LVDS pairs and
sends what it hears through the normal frame path. `record` keeps every
datagram the Mac receives; `compare` lines that up with RHX's file and checks
every sample. docs/intan-tap-check.md has the wiring and the procedure.

status, delay-sweep and selftest are standard library plus pyserial; compare
needs numpy (chain.tapcompare, chain.rhx).

Exit status: 0 nothing failed, 1 something failed (missing, wrong, or a check
that did not pass), 2 the command could not run (no reply, files that cannot
be compared).
"""
import argparse
import json
import pathlib
import signal
import socket
import struct
import threading
import time

from . import contract as C
from . import diag as D
from . import frame as F
from . import metrics, receiver, taprec
from . import rhd2164 as R
from .ctrl import Ctrl, CtrlError

RESULTS = pathlib.Path(__file__).resolve().parents[2] / "docs" / "results"
RECORDINGS = pathlib.Path.home() / "intan-tap-recordings"

MODE_LOOPBACK = 0x08
MODE_TAP = 0x10
TAP_COUNTS = ("TAP_CYCLES", "TAP_FRAMING", "TAP_SWEEPS", "TAP_PARTIAL", "TAP_CHECKS",
              "TAP_ERR0", "TAP_ERR1")
TAP_REGS = ("TAP_STAT",) + TAP_COUNTS + ("TAP_SCLK_MIN",)
BOARD_COUNTS = ("FRAMES", "DGRAMS", "FIFO_HIGH", "OVERRUNS", "STALLS")
SCLK_NONE = 0xFFFFFFFF

# TAP_STAT bit order ([4:0] levels, [12:8] toggled): cs_n, sclk, mosi, miso0, miso1.
LINES = (("cs_n", "CS", "JA1/JA2", 1), ("sclk", "SCLK", "JA3/JA4", 0),
         ("mosi", "MOSI", "JA7/JA8", None), ("miso0", "MISO1 (chip 0, ch 0-63)", "JA9/JA10", None),
         ("miso1", "MISO2 (chip 1, ch 64-127)", "JB1/JB2", None))   # name, cable, pins, idle

# Rhythm (and RHX on Intan's controllers) clocks 35 commands of 20 SCLK periods
# per sample, so SCLK is about 700 x the sample rate (1.4 MHz at 2 kS/s).
# Only a hint: another controller may time its bus differently.
RHYTHM_SCLK_PER_SAMPLE = 700
ROUND_TRIP_FC = 3            # "a few clocks": tap -> chip -> tap plus tMISO, short cable


class Abort(RuntimeError):
    """A precondition failed; the message says which and what to do (exit 2)."""


# ---------------------------------------------------------------------------
# Board plumbing shared by the commands
# ---------------------------------------------------------------------------

def _fc_ns(fc: float) -> float:
    return fc * 1e9 / C.SYS_CLK_HZ


def connect(ctrl, settle: float = 0.05) -> D.Caps:
    """Resync, check ID, decode CAPS. Raises Abort with a cause the way bring-up names it."""
    try:
        ctrl.resync(settle)
        idv = ctrl.read("ID")
    except CtrlError as e:
        raise Abort(f"no usable reply to R 1f ({e}): the FPGA is not programmed (volatile: "
                    f"reload after power-off), this is the Arty's JTAG port (use the one "
                    f"ending in 1), or the baud is wrong") from None
    if idv != C.ID_EXPECTED:
        raise Abort(f"ID reads 0x{idv:08x}, want 0x{C.ID_EXPECTED:08x}: a bitstream built "
                    f"from another CONTRACTS.md")
    return D.decode_caps(ctrl.read("CAPS"))


def require_tap(caps: D.Caps) -> None:
    if not caps.tap:
        raise Abort(f"CAPS 0x{caps.raw:08x} has bit9 clear: this bitstream has no passive "
                    f"tap, so MODE bit4 does nothing. Load the tap bitstream")


def read_regs(ctrl, names) -> dict:
    return {n: ctrl.read(n) for n in names}


class BoardState:
    """MODE, run and the tap knobs as found, so a command can put them back."""

    def __init__(self, ctrl, extra=()):
        self.ctrl = ctrl
        self.names = ("MODE", "TAP_DELAY") + tuple(extra)
        self.saved = read_regs(ctrl, self.names)
        self.running = bool(ctrl.read("CTRL") & 1)

    def restore(self, keep=()) -> None:
        if self.ctrl.read("CTRL") & 1:
            self.ctrl.set_run(False)
        for n in self.names:
            if n not in keep:
                self.ctrl.write(n, self.saved[n])
        if self.running:
            self.ctrl.set_run(True)


def listen(ctrl, seconds: float, sleep, mode: int = MODE_TAP) -> None:
    """Tap mode, counters cleared, run for `seconds`; the caller reads and restores."""
    if ctrl.read("CTRL") & 1:
        ctrl.set_run(False)
    ctrl.write("MODE", mode)
    ctrl.clear_counters()
    ctrl.set_run(True)
    sleep(seconds)


# ---------------------------------------------------------------------------
# status
# ---------------------------------------------------------------------------

def sclk_facts(sclk_min: int) -> dict | None:
    """What TAP_SCLK_MIN says about SCLK, the sample rate and the usable TAP_DELAY."""
    if sclk_min == SCLK_NONE or sclk_min == 0:
        return None
    h = sclk_min
    f = C.SYS_CLK_HZ / (2 * h)
    lo = ROUND_TRIP_FC
    hi = min(C.TAP_DELAY_MAX, lo + h)
    return {"half_period_fc": h, "half_period_ns": round(_fc_ns(h), 1),
            "sclk_hz": f, "sclk_hz_range": (C.SYS_CLK_HZ / (2 * (h + 1)),
                                            C.SYS_CLK_HZ / (2 * max(1, h - 1))),
            "implied_rate": f / RHYTHM_SCLK_PER_SAMPLE,
            "delay_usable": (lo, hi), "delay_preferred": (lo, min(hi, lo + h // 2))}


def explain(regs: dict, caps: D.Caps, listened: float | None) -> tuple[list[str], list[dict]]:
    """(lines to print, findings) for one readout of the tap registers."""
    out, fs = [], []

    def finding(status, cat, msg):
        fs.append({"status": status, "category": cat, "message": msg})

    out.append(f"bitstream  CAPS 0x{caps.raw:08x}: passive tap {'present' if caps.tap else 'ABSENT'}"
               f" (bit9), on-die emulator {'yes' if caps.emulator else 'no'} (bit8)")
    if not caps.tap:
        finding("FAIL", "NO_TAP", "CAPS bit9 is clear: not the tap bitstream (MODE bit4 does "
                "nothing and TAP_DELAY answers err); load the tap bitstream")
        return out, fs
    stat = regs["TAP_STAT"]
    levels, toggled, running = stat & 0x1F, (stat >> 8) & 0x1F, bool(stat & 0x10000)
    cyc, framing, sweeps, partial = (regs[n] for n in ("TAP_CYCLES", "TAP_FRAMING",
                                                        "TAP_SWEEPS", "TAP_PARTIAL"))
    checks = regs["TAP_CHECKS"]
    errs = D.split16(regs["TAP_ERR0"]) + D.split16(regs["TAP_ERR1"])
    how = (f"listened {listened:g} s in tap mode (MODE 0x10, counters cleared)"
           if listened else "registers as found (no listening: --listen 0)")
    out.append(f"mode       {how}; MODE 0x{regs['MODE']:02x}, TAP_STAT bit16 "
               f"(tap selected and running) {'set' if running else 'clear'}")
    if listened and caps.tap and not running:
        finding("FAIL", "TAP_NOT_RUNNING", "MODE bit4 and run were set but TAP_STAT bit16 is "
                "clear: the tap is not the frame source; reload the tap bitstream")

    out.append("lines      name   toggled  level now  cable pair -> Arty pins")
    any_toggle = bool(toggled)
    for bit, (name, cable, pins, idle) in enumerate(LINES):
        t, lvl = toggled >> bit & 1, levels >> bit & 1
        out.append(f"           {name:<6} {'yes' if t else 'NO':<8} {lvl:<10} {cable} -> {pins}")
        if any_toggle and not t:
            cat = cable.split()[0] + "_SILENT"
            if name.startswith("miso"):
                finding("FAIL", cat,
                        f"{cable} never toggled while the bus ran: that pair is not wired to "
                        f"{pins}, or the chip is not driving it (absent, unpowered, or not "
                        f"reached by CS/SCLK)")
            else:
                finding("FAIL", cat,
                        f"{cable} never toggled while other lines did: the {cable} pair is "
                        f"not wired to {pins} (or open at the C3430)")
    if not any_toggle:
        # The tap watches its lines whatever MODE says, so silence since the
        # last clear means an idle bus; without listening, that clear may have
        # been a moment ago, so it is only a warning.
        finding("FAIL" if listened else "WARN", "BUS_IDLE",
                "no tap line toggled" + ("" if listened else " since the last counter clear")
                + ": nothing is clocking the bus. Is RHX running (the controller only clocks "
                  "the headstage while RHX runs)? Is the tap ground (JA5/JA11 to C3430 Y T6) "
                  "connected, and are the wires on JA/JB?")
        # Idle levels only mean something on an idle bus.
        for bit, (name, cable, pins, idle) in enumerate(LINES):
            if idle is not None and (levels >> bit & 1) != idle:
                finding("WARN", f"{cable}_POLARITY",
                        f"{name} idles at {levels >> bit & 1}, an idle {cable} reads {idle}: "
                        f"the {cable} pair is swapped (+/-) or open between C3430 Y and {pins}")
    else:
        out.append("           (a level snapshot of a busy bus means little; with RHX stopped "
                   "cs_n idles 1 and sclk 0)")

    rate = ""
    if listened and cyc:
        rate = (f" = {cyc / listened:,.0f}/s; {sweeps / listened:,.1f} sweeps/s heard (on a "
                f"healthy tap, the controller's sample rate)")
    out.append(f"cycles     TAP_CYCLES {cyc}{rate}")
    pct = 100.0 * framing / cyc if cyc else 0.0
    out.append(f"framing    TAP_FRAMING {framing} ({pct:.3f} % of cycles without 16 + 16 SCLK edges)")
    out.append(f"sweeps     TAP_SWEEPS {sweeps}, TAP_PARTIAL {partial}")
    if cyc:
        if framing >= 0.9 * cyc:
            finding("FAIL", "FRAMING_ALL",
                    f"{pct:.0f} % of CS cycles have the wrong SCLK edge count: CS pair swapped "
                    f"(the tap then calls the gap between commands 'CS low'), or SCLK not "
                    f"arriving. Check CS (JA1/JA2) and SCLK (JA3/JA4) polarity and contact")
        elif framing:
            finding("WARN", "FRAMING_SOME",
                    f"{framing} CS cycles ({pct:.3f} %) mis-framed: noise or reflections on "
                    f"CS/SCLK. Shorten and twist the tap wires; each one costs a partial frame")
        if not (running or listened):
            finding("INFO", "NOT_FRAMING", "TAP_SWEEPS and TAP_PARTIAL count only while the tap "
                    "is selected and running (MODE bit4, run); rerun without --listen 0 to "
                    "see them")
        elif framing < 0.9 * cyc and sweeps == 0:
            finding("FAIL", "NO_SWEEPS",
                    "CS cycles are heard but no CONVERT(0..31) sweep was recognised: MOSI or "
                    "SCLK pair swapped (commands decode as something else), or the controller "
                    "is not streaming (RHX stopped mid-setup)")
        if sweeps and partial >= 0.9 * sweeps and framing < 0.1 * cyc:
            finding("FAIL", "ALL_PARTIAL",
                    "nearly every sweep is partial without framing errors: commands keep "
                    "reading as CONVERT(0), what an open (constant-0) MOSI pair gives. Check "
                    "MOSI (JA7/JA8)")
        elif sweeps and partial:
            finding("WARN", "PARTIAL", f"{partial} of {sweeps} sweeps partial "
                    f"({100.0 * partial / sweeps:.2f} %): each one is a frame with channels "
                    f"missing")

    out.append(f"checks     TAP_CHECKS {checks}; TAP_ERR chip0 A {errs[0]} B {errs[1]}, "
               f"chip1 A {errs[2]} B {errs[3]} (each half saturates at 65535)")
    if sweeps and not checks:
        finding("INFO", "NO_CHECKS",
                "TAP_CHECKS did not climb: RHX sent no known-answer command (WRITE, READ of "
                "ROM) while listening, so this readout cannot judge TAP_DELAY; `compare` is "
                "the real check")
    elif checks:
        bad = {l for l in range(4) if errs[l]}
        names = ", ".join(D.LANES[l] for l in sorted(bad))
        a_bad = {l for l in bad if l % 2 == 0}
        chips = {l // 2 for l in bad}
        if not bad:
            pass
        elif a_bad == {0, 2}:
            finding("FAIL", "DELAY_OUT",
                    f"known answers fail on {names}, both chips: TAP_DELAY is outside "
                    f"the window (run `tapcheck delay-sweep`), or both MISO pairs are swapped")
        elif a_bad and len(chips) == 1:
            c = chips.pop()
            finding("FAIL", f"CHIP{c}_ERRORS",
                    f"known answers fail on {names} while chip {1 - c} passes: the MISO"
                    f"{c + 1} pair is swapped (+/-), open or marginal at the tap")
        elif not a_bad:
            finding("FAIL", "B_LATE",
                    f"known answers fail on {names} but module A passes: B's last bit is sampled after CS "
                    f"rose; lower TAP_DELAY (run `tapcheck delay-sweep`)")
        else:
            finding("FAIL", "LANE_ERRORS", f"known answers fail on {names}: TAP_DELAY near "
                    f"the edge of the window; run `tapcheck delay-sweep`")

    sc = sclk_facts(regs["TAP_SCLK_MIN"])
    if sc is None:
        out.append("SCLK       TAP_SCLK_MIN ffffffff: no SCLK edge heard since the clear")
    else:
        lo, hi = sc["sclk_hz_range"]
        out.append(f"SCLK       TAP_SCLK_MIN {sc['half_period_fc']} clocks = "
                   f"{sc['half_period_ns']:.0f} ns shortest half period -> SCLK about "
                   f"{sc['sclk_hz'] / 1e6:.2f} MHz ({lo / 1e6:.2f}-{hi / 1e6:.2f} given the "
                   f"one-clock resolution)")
        out.append(f"           if the controller runs Rhythm's timing (SCLK ~ "
                   f"{RHYTHM_SCLK_PER_SAMPLE} x sample rate): about "
                   f"{sc['implied_rate']:,.0f} S/s")
        u0, u1 = sc["delay_usable"]
        p0, p1 = sc["delay_preferred"]
        out.append(f"TAP_DELAY  {regs['TAP_DELAY']} (reset {C.TAP_DELAY_RESET}); usable roughly "
                   f"{u0}..{u1} (from the round trip, a few clocks, to half an SCLK period "
                   f"after it); prefer {p0}..{p1}, the first half, because module B's last "
                   f"bit may not outlive CS by much")
        if not p0 <= regs["TAP_DELAY"] <= p1:
            finding("WARN", "DELAY_RANGE",
                    f"TAP_DELAY {regs['TAP_DELAY']} is outside {p0}..{p1}, the first half of "
                    f"the window this SCLK allows; run `tapcheck delay-sweep`")
        if sc["half_period_fc"] < 6:
            finding("WARN", "SCLK_FAST",
                    f"SCLK half period is only {sc['half_period_fc']} clocks: the window is "
                    f"too narrow to sample reliably; record at 2 kS/s")
    if sc is None:
        out.append(f"TAP_DELAY  {regs['TAP_DELAY']} (reset {C.TAP_DELAY_RESET})")
    return out, fs


def cmd_status(ctrl, args, out=print, sleep=time.sleep) -> int:
    caps = connect(ctrl, args.settle)
    listened = None
    if not caps.tap:
        # Without a tap, TAP_DELAY answers `err` (section 4): read nothing tap-shaped.
        lines, fs = explain({}, caps, None)
        for line in lines:
            out(line)
        for f in fs:
            out(f"  [{f['status']}] {f['category']}: {f['message']}")
        out("STATUS FAIL")
        return 1
    if args.listen > 0:
        state = BoardState(ctrl)
        try:
            listen(ctrl, args.listen, sleep)
            # TAP_STAT while running (bit16), then stop so the counters hold
            # still while they are read one round trip at a time.
            regs = {"TAP_STAT": ctrl.read("TAP_STAT")}
            ctrl.set_run(False)
            regs.update(read_regs(ctrl, TAP_REGS[1:] + ("MODE", "TAP_DELAY")))
        finally:
            state.restore()
        listened = args.listen
    else:
        regs = read_regs(ctrl, TAP_REGS + ("MODE", "TAP_DELAY"))
    lines, fs = explain(regs, caps, listened)
    for line in lines:
        out(line)
    for f in fs:
        out(f"  [{f['status']}] {f['category']}: {f['message']}")
    failed = any(f["status"] == "FAIL" for f in fs)
    out("STATUS " + ("FAIL" if failed else "OK"))
    return 1 if failed else 0


# ---------------------------------------------------------------------------
# delay-sweep
# ---------------------------------------------------------------------------

def sweep_delays(ctrl, delays, dwell: float, sleep, settle: float = 0.02) -> list[dict]:
    """One row per TAP_DELAY: counters accumulated over `dwell` s, cleared at each step.

    The counters are cleared after the new delay has settled, so the glitch a
    delay change can cause inside one CS cycle is never counted against it.
    """
    rows = []
    for d in delays:
        ctrl.write("TAP_DELAY", d)
        got = ctrl.read("TAP_DELAY")
        sleep(settle)
        ctrl.clear_counters()
        sleep(dwell)
        r = read_regs(ctrl, TAP_COUNTS)
        e = D.split16(r["TAP_ERR0"]) + D.split16(r["TAP_ERR1"])
        rows.append({"delay": got, "cycles": r["TAP_CYCLES"], "framing": r["TAP_FRAMING"],
                     "sweeps": r["TAP_SWEEPS"], "partial": r["TAP_PARTIAL"],
                     "checks": r["TAP_CHECKS"], "errors": list(e),
                     "clean": bool(r["TAP_CHECKS"] and not any(e) and not r["TAP_FRAMING"])})
    return rows


def choose_delay(rows: list[dict]):
    """(delay, window) at the centre of the widest run of clean steps, or (None, None)."""
    idx = [i for i, r in enumerate(rows) if r["clean"]]
    if not idx:
        return None, None
    win = D.widest(D.runs(idx))
    window = (rows[win[0]]["delay"], rows[win[1]]["delay"])
    return D.centre(window), window


def delay_table(rows: list[dict]) -> list[str]:
    out = [" delay  cycles   framing  sweeps  partial  checks  err c0A  c0B  c1A  c1B  clean"]
    for r in rows:
        e = r["errors"]
        out.append(f" {r['delay']:>5}  {r['cycles']:>7}  {r['framing']:>7}  {r['sweeps']:>6}  "
                   f"{r['partial']:>7}  {r['checks']:>6}  {e[0]:>7} {e[1]:>4} {e[2]:>4} "
                   f"{e[3]:>4}  {'#' if r['clean'] else '.'}")
    return out


def cmd_delay_sweep(ctrl, args, out=print, sleep=time.sleep) -> int:
    caps = connect(ctrl, args.settle)
    require_tap(caps)
    lo, hi = max(0, args.lo), min(C.TAP_DELAY_MAX, args.hi)
    delays = list(range(lo, hi + 1, max(1, args.step)))
    state = BoardState(ctrl)
    chosen = window = None
    try:
        listen(ctrl, 0.0, sleep)
        rows = sweep_delays(ctrl, delays, args.dwell, sleep)
        sclk = ctrl.read("TAP_SCLK_MIN")      # over the last dwell: SCLK does not depend on delay
        for line in delay_table(rows):
            out(line)
        chosen, window = choose_delay(rows)
        sc = sclk_facts(sclk)
        if not any(r["sweeps"] for r in rows):
            out("no sweep heard at any delay: the bus is idle or miswired; run `tapcheck status`")
        elif chosen is None and not any(r["checks"] for r in rows):
            out("TAP_CHECKS never climbed: RHX sent no known-answer command while streaming, "
                "so the tap cannot judge TAP_DELAY by itself. Keep the reset value "
                f"({C.TAP_DELAY_RESET})"
                + (f", inside the {sc['delay_preferred'][0]}..{sc['delay_preferred'][1]} this "
                   f"SCLK allows" if sc else "")
                + ", and let `tapcheck compare` judge (or record + compare at a few delays)")
        elif chosen is None:
            out("no delay passed every known-answer check: see the error columns; a chip that "
                "fails everywhere has its MISO pair swapped or open (`tapcheck status`)")
        else:
            out(f"clean window TAP_DELAY {window[0]}..{window[1]}; centre {chosen}")
    finally:
        keep = ("TAP_DELAY",) if (args.apply and chosen is not None) else ()
        if keep:
            ctrl.write("TAP_DELAY", chosen)
        state.restore(keep=keep)
    if args.apply and chosen is not None:
        out(f"TAP_DELAY {ctrl.read('TAP_DELAY')} written (volatile: lost when the FPGA is "
            f"reprogrammed; pass `record --delay {chosen}` to be sure)")
    elif chosen is not None:
        out(f"TAP_DELAY left at {state.saved['TAP_DELAY']}; --apply writes {chosen}, or pass "
            f"`record --delay {chosen}`")
    rep = {"tool": "tapcheck", "command": "delay-sweep",
           "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"), "port": args.serial,
           "dwell_s": args.dwell, "rows": rows, "chosen": chosen, "window": window,
           "tap_sclk_min": sclk, "applied": bool(args.apply and chosen is not None)}
    save_json(rep, args, "delay")
    # No choice is not a failure when the bus simply carried no known answers.
    undecidable = any(r["sweeps"] for r in rows) and not any(r["checks"] for r in rows)
    return 0 if (chosen is not None or undecidable) else 1


# ---------------------------------------------------------------------------
# record
# ---------------------------------------------------------------------------

class StopFlag:
    """Ctrl-C ends the capture between datagrams, never in the middle of a write."""

    def __init__(self):
        self.set = False
        self._old = None

    def __enter__(self):
        if threading.current_thread() is threading.main_thread():
            self._old = signal.signal(signal.SIGINT, self._handler)
        return self

    def _handler(self, *_):
        self.set = True

    def __exit__(self, *exc):
        if self._old is not None:
            signal.signal(signal.SIGINT, self._old)


def capture_to(writer: taprec.Writer, sock, seconds, stop: StopFlag, clock=time.monotonic,
               progress=None, every: float = 2.0) -> None:
    """The receive loop: recv, stamp, append. Nothing else between two datagrams.

    `progress(writer, last)` runs at most every `every` seconds, on the
    loop's time, so a slow terminal cannot cost a datagram.
    """
    sock.settimeout(0.2)
    deadline = None if seconds is None else clock() + seconds
    next_p = clock() + every
    last = b""
    recv, add = sock.recv, writer.add
    while not stop.set and (deadline is None or clock() < deadline):
        try:
            data = recv(65535)
        except socket.timeout:
            data = None
        except OSError:
            break
        now = clock()
        if data is not None:
            add(now, data)
            last = data
        if progress is not None and now >= next_p:
            next_p = now + every
            progress(writer, last)


def _last_seq(data: bytes):
    return struct.unpack_from("<I", data, 4)[0] if len(data) >= C.HDR_BYTES else None


def cmd_record(ctrl, args, out=print, sleep=time.sleep, sock=None, clock=time.monotonic) -> int:
    caps = connect(ctrl, args.settle)
    require_tap(caps)
    outdir = pathlib.Path(args.out).expanduser()
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = outdir / f"tap_{stamp}{taprec.SUFFIX}"
    own = sock is None
    if own:
        try:
            sock = receiver.open_socket(args.udp_port)   # open before run: nothing is lost
        except OSError as e:
            raise Abort(f"cannot listen on UDP {args.udp_port} ({e}): is the viewer or "
                        f"another tapcheck running?") from None
    if ctrl.read("CTRL") & 1:
        ctrl.set_run(False)
    ctrl.write("MODE", MODE_TAP)
    if args.delay is not None:
        ctrl.write("TAP_DELAY", args.delay)
    settings = read_regs(ctrl, ("MODE", "TAP_DELAY", "CAPS", "ID"))
    started = time.strftime("%Y-%m-%dT%H:%M:%S")
    out(f"recording to {path}")
    out(f"  MODE 0x{settings['MODE']:02x} (tap), TAP_DELAY {settings['TAP_DELAY']}, UDP "
        f"{args.udp_port}; " + (f"{args.seconds:g} s" if args.seconds else "until Ctrl-C"))
    out("  start the RHX recording now; stop it before this ends")
    t0 = clock()
    warned = [False]

    def progress(w, last):
        el = clock() - t0
        if w.datagrams == 0 and el > 3 and not warned[0]:
            warned[0] = True
            out("  nothing on UDP yet: is RHX running (the bus is only clocked while it runs), "
                "is the ESP on Wi-Fi, does the Mac's firewall let UDP 5001 in?")
        out(f"  {el:7.1f} s  {w.datagrams:>8} datagrams  {w.bytes / 1e6:8.1f} MB"
            + (f"  last seq {_last_seq(last)}" if last else ""))

    interrupted = False
    with taprec.Writer(path, clock=clock) as w, StopFlag() as stop:
        try:
            ctrl.clear_counters()
            ctrl.set_run(True)
            capture_to(w, sock, args.seconds, stop, clock=clock,
                       progress=progress if args.progress else None)
            interrupted = stop.set
        finally:
            try:
                ctrl.set_run(False)
            except CtrlError as e:
                out(f"  could not stop the run: {e}")
            # Datagrams still in the ESP or in the air arrive after the stop.
            stop.set = False
            capture_to(w, sock, args.drain, stop, clock=clock)
            if own:
                sock.close()
    try:
        counters = read_regs(ctrl, BOARD_COUNTS + TAP_REGS + ("TAP_DELAY",))
    except CtrlError as e:
        # The recording is still worth comparing; say why the counters are missing.
        counters = {"error": f"counters not read: {e}"}
        out(f"  {counters['error']}")
    meta = {"tool": "tapcheck", "command": "record", "file": path.name, "started": started,
            "seconds": round(clock() - t0, 3), "interrupted": interrupted,
            "datagrams": w.datagrams, "bytes": w.bytes, "udp_port": args.udp_port,
            "serial": args.serial, "settings": settings, "counters": counters}
    taprec.sidecar(path).write_text(json.dumps(meta, indent=1) + "\n")
    if "error" not in counters:
        lost_air = counters["DGRAMS"] - w.datagrams
        out(f"stopped after {meta['seconds']:.1f} s: {w.datagrams} datagrams "
            f"({w.bytes / 1e6:.1f} MB) received of {counters['DGRAMS']} the FPGA sent"
            + (f" ({lost_air} lost on the way)" if lost_air > 0 else ""))
        out(f"  FRAMES {counters['FRAMES']}  OVERRUNS {counters['OVERRUNS']} (FIFO full, or "
            f"sweeps the tap could not frame)  "
            f"TAP_SWEEPS {counters['TAP_SWEEPS']}  TAP_PARTIAL {counters['TAP_PARTIAL']}  "
            f"TAP_FRAMING {counters['TAP_FRAMING']}  TAP_CHECKS {counters['TAP_CHECKS']}  "
            f"TAP_ERR {counters['TAP_ERR0']:08x} {counters['TAP_ERR1']:08x}")
    out(f"  recording {path}\n  counters  {taprec.sidecar(path)}")
    out(f"next: python3 -m chain.tapcheck compare {path} <RHX folder or .rhd>")
    return 0 if w.datagrams else 1


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------

def _playback():
    try:
        from .emulator import Playback
        return Playback()
    except (OSError, ValueError):
        return None


def selftest_checks(raw, regs: dict, playback, rate: float) -> tuple[list[dict], dict]:
    """Score a loopback capture: (checks, details). Each check is PASS / FAIL / SKIP."""
    checks = []

    def check(name, ok, detail, skip=False):
        checks.append({"check": name, "status": "SKIP" if skip else ("PASS" if ok else "FAIL"),
                       "detail": detail})

    dgrams, malformed = [], 0
    for t, data in raw:
        try:
            frames, trailer = F.parse_datagram(data)
        except F.FrameError:
            malformed += 1
            continue
        dgrams.append((t, frames, trailer))
    frames = [f for _t, fs, _tr in dgrams for f in fs]
    check("frames built", regs["FRAMES"] > 0,
          f"FRAMES {regs['FRAMES']}, TAP_SWEEPS {regs['TAP_SWEEPS']}, TAP_CYCLES "
          f"{regs['TAP_CYCLES']}")
    check("datagrams received", bool(dgrams),
          f"{len(raw)} datagrams on UDP ({malformed} malformed) of {regs['DGRAMS']} sent"
          + ("" if dgrams or regs["DGRAMS"] else
             "; DGRAMS 0: nothing left the FPGA (STALLS climbing = no ESP draining QSPI)"))
    if not dgrams:
        return checks, {"malformed": malformed}
    m = metrics.compute(dgrams, malformed=malformed, golden=playback,
                        check_accuracy=playback is not None)
    nontap = sum(1 for f in frames if not f.flags & C.FLAG_TAP)
    part = sum(1 for f in frames if f.flags & C.FLAG_TAP_PARTIAL)
    check("FLAG_TAP on every frame", nontap == 0, f"{nontap} of {len(frames)} frames without it")
    check("no TAP_PARTIAL", part == 0 and regs["TAP_PARTIAL"] == 0,
          f"{part} partial frames received, TAP_PARTIAL counter {regs['TAP_PARTIAL']}")
    check("no framing errors", regs["TAP_FRAMING"] == 0, f"TAP_FRAMING {regs['TAP_FRAMING']}")
    loss = m["loss"]
    lost_air = regs["DGRAMS"] - len(raw)
    check("seq contiguous", loss["missing"] == 0 and loss["duplicates"] == 0
          and loss["reordered"] == 0,
          f"{loss['missing']} missing, {loss['duplicates']} duplicate, {loss['reordered']} "
          f"reordered of {loss['frames_expected']}; OVERRUNS {regs['OVERRUNS']} (FIFO, or "
          f"sweeps the tap could not frame), "
          f"{max(0, lost_air)} datagrams sent but not received (Wi-Fi or the Mac)")
    check("one frame per sweep", regs["FRAMES"] == regs["TAP_SWEEPS"],
          f"FRAMES {regs['FRAMES']} vs TAP_SWEEPS {regs['TAP_SWEEPS']}")
    want_cycles = C.CMDS_PER_SAMPLE * regs["TAP_SWEEPS"]
    check("35 CS cycles per sweep", abs(regs["TAP_CYCLES"] - want_cycles) <= 2 * C.CMDS_PER_SAMPLE,
          f"TAP_CYCLES {regs['TAP_CYCLES']}, 35 x TAP_SWEEPS = {want_cycles} (the first and "
          f"last sweep may be heard in part)")
    errs = D.split16(regs["TAP_ERR0"]) + D.split16(regs["TAP_ERR1"])
    check("known answers (READ 59, 40, 63 every sweep)",
          regs["TAP_CHECKS"] >= regs["TAP_SWEEPS"] > 0 and not any(errs),
          f"TAP_CHECKS {regs['TAP_CHECKS']} (3 per sweep expected), TAP_ERR "
          + ", ".join(f"{D.LANES[l]} {errs[l]}" for l in range(4)))
    acc = m["accuracy"]
    if playback is None:
        check("values = on-die emulator", False, "emulator memories not found "
              "(fpga/upstream/rhd2164-emulator-mem); values not checked", skip=True)
    else:
        located = sum(1 for f in frames[:64] if playback.locate(f.samples()) is not None)
        check("values = on-die emulator", located > 0 and acc["value_errors"] == 0,
              f"{acc['value_errors']} of {acc['values']} values differ ({acc['bit_errors']} "
              f"bits) from the playback memory at offset {acc.get('playback_offset')}; "
              f"{located} of the first {min(64, len(frames))} frames found in it")
    sc = sclk_facts(regs["TAP_SCLK_MIN"])
    details = {"metrics": m, "malformed": malformed,
               "sclk": None if sc is None else {k: v for k, v in sc.items()},
               "frames_received": len(frames), "rate": rate}
    return checks, details


def cmd_selftest(ctrl, args, out=print, sleep=time.sleep, capture=None) -> int:
    caps = connect(ctrl, args.settle)
    require_tap(caps)
    if not caps.emulator:
        raise Abort("CAPS bit8 clear: no on-die emulator, so the loopback self-test cannot run")
    period = max(round(C.SYS_CLK_HZ / args.rate), R.min_sample_period(args.spi_div))
    rate = C.SYS_CLK_HZ / period
    if rate < args.rate * 0.999:
        out(f"{args.rate:g} S/s is faster than a sweep at SPI_DIV {args.spi_div} allows; "
            f"running at {rate:,.0f} S/s")
    half = 2 * (args.spi_div + 1)
    out(f"self-test: MODE bit3|bit4 (the tap hears our master sweep the on-die emulator), "
        f"SPI_DIV {args.spi_div} (SCLK {R.sclk_hz(args.spi_div) / 1e6:.2f} MHz, half period "
        f"{half} clocks), {rate:,.0f} S/s, {args.seconds:g} s")
    state = BoardState(ctrl, extra=("SPI_DIV", "SAMPLE_PERIOD"))
    sock = None
    try:
        if ctrl.read("CTRL") & 1:
            ctrl.set_run(False)
        ctrl.write("SPI_DIV", args.spi_div)
        ctrl.write("SAMPLE_PERIOD", period)
        if args.delay is not None:
            ctrl.write("TAP_DELAY", args.delay)
        delay = ctrl.read("TAP_DELAY")
        if not 2 <= delay <= half:
            out(f"note: TAP_DELAY {delay} is outside 2..{half}, about where the window lies at "
                f"this SPI_DIV; expect errors (--delay N)")
        ctrl.write("MODE", MODE_LOOPBACK | MODE_TAP)
        if capture is None:
            sock = receiver.open_socket(args.udp_port)
            capture = lambda s: receiver.capture(s, sock=sock)
        ctrl.clear_counters()
        ctrl.set_run(True)
        running = bool(ctrl.read("TAP_STAT") & 0x10000)
        raw = capture(args.seconds)
        ctrl.set_run(False)
        sleep(args.drain)
        regs = read_regs(ctrl, BOARD_COUNTS + TAP_REGS + ("TAP_DELAY", "SPI_DIV"))
    finally:
        if sock is not None:
            sock.close()
        state.restore()
    checks, details = selftest_checks(raw, regs, _playback(), rate)
    checks.insert(0, {"check": "tap selected and running", "status": "PASS" if running else "FAIL",
                      "detail": "TAP_STAT bit16 " + ("set" if running else "clear")})
    for c in checks:
        out(f"  {c['status']:<4}  {c['check']:<44} {c['detail']}")
    failed = any(c["status"] == "FAIL" for c in checks)
    if regs["DGRAMS"] and not raw:
        out("  frames left the FPGA but none arrived: check the ESP (Wi-Fi joined? same "
            "network?) and the Mac's firewall for UDP 5001")
    out("SELFTEST " + ("FAIL" if failed else "PASS"))
    rep = {"tool": "tapcheck", "command": "selftest",
           "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"), "port": args.serial,
           "result": "FAIL" if failed else "PASS", "spi_div": args.spi_div, "rate": rate,
           "seconds": args.seconds, "checks": checks, "registers": regs,
           "details": {k: v for k, v in details.items() if k != "metrics"},
           "metrics": details.get("metrics")}
    save_json(rep, args, "selftest")
    return 1 if failed else 0


# ---------------------------------------------------------------------------
# compare
# ---------------------------------------------------------------------------

def cmd_compare(args, out=print) -> int:
    try:
        from . import tapcompare as T
        from .rhx import RhxError
    except ImportError as e:
        raise Abort(f"compare needs numpy ({e}): pip install -r requirements.txt") from None
    opts = T.Options(port=args.port, anchors=args.anchors, show=args.show)
    try:
        rep = T.compare(args.recording, args.rhx, opts)
    except (T.CompareError, RhxError, taprec.TapRecError, OSError) as e:
        out(f"cannot compare: {e}")
        return 2
    out(T.render(rep, args.show))
    path = save_json(rep, args, "compare")
    if path is not None:
        out(f"report {path}")
    return 0 if rep["result"] == "PASS" else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def save_json(rep: dict, args, what: str):
    """docs/results/tapcheck_<what>_<time>.json, like the other tools; --json overrides.

    Fake runs save only with --json, as bring-up's do.
    """
    path = getattr(args, "json", None)
    if path is None and getattr(args, "fake", None) is None:
        path = RESULTS / f"tapcheck_{what}_{time.strftime('%Y%m%d-%H%M%S')}.json"
    if path is None:
        return None
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rep, indent=1, default=str) + "\n")
    return path


def build_parser() -> argparse.ArgumentParser:
    from .fakeboard import FAULTS
    ap = argparse.ArgumentParser(
        prog="python3 -m chain.tapcheck",
        description="check the chain sample by sample against Intan's RHD recording "
                    "controller (CONTRACTS.md section 7)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def board(p):
        p.add_argument("--serial", help="FPGA control UART (the Arty's second FTDI port)")
        p.add_argument("--baud", type=int, default=C.UART_BAUD)
        p.add_argument("--fake", nargs="*", choices=sorted(FAULTS), metavar="FAULT",
                       help="no hardware: chain.fakeboard as a tap build, with these faults "
                            "(tap_* ones model the tap wiring)")
        p.add_argument("--json", type=pathlib.Path, default=None, metavar="PATH",
                       help="report path (default docs/results/tapcheck_<cmd>_<time>.json)")
        p.set_defaults(settle=0.05)

    p = sub.add_parser("status", help="read and explain the tap registers")
    board(p)
    p.add_argument("--listen", type=float, default=1.0, metavar="S",
                   help="seconds in tap mode with counters cleared before reading (default 1; "
                        "0 reads the registers as they are). MODE and run are put back after")
    p = sub.add_parser("delay-sweep", help="step TAP_DELAY and pick the widest clean window")
    board(p)
    p.add_argument("--from", dest="lo", type=int, default=0, metavar="N")
    p.add_argument("--to", dest="hi", type=int, default=C.TAP_DELAY_MAX, metavar="N")
    p.add_argument("--step", type=int, default=1)
    p.add_argument("--dwell", type=float, default=0.5, metavar="S", help="seconds per delay")
    p.add_argument("--apply", action="store_true", help="write the chosen TAP_DELAY back")
    p = sub.add_parser("record", help="tap mode, then every datagram to a file until "
                                      "--seconds or Ctrl-C")
    board(p)
    p.add_argument("--seconds", type=float, default=None, metavar="S",
                   help="stop after S seconds (default: Ctrl-C)")
    p.add_argument("--delay", type=int, default=None, metavar="N",
                   help=f"TAP_DELAY 0..{C.TAP_DELAY_MAX} (default: leave as is)")
    p.add_argument("--out", type=pathlib.Path, default=RECORDINGS, metavar="DIR",
                   help=f"where recordings go (default {RECORDINGS}; ~0.5 MB/s at 2 kS/s)")
    p.add_argument("--udp-port", type=int, default=C.UDP_PORT)
    p.add_argument("--drain", type=float, default=0.5, metavar="S",
                   help="keep receiving this long after the run stops")
    p.add_argument("--no-progress", dest="progress", action="store_false")
    p = sub.add_parser("selftest", help="MODE bit3|bit4: the tap hears our own master and "
                                        "the on-die emulator; no Intan needed")
    board(p)
    p.add_argument("--seconds", type=float, default=3.0, metavar="S")
    p.add_argument("--rate", type=float, default=2000.0, metavar="HZ")
    p.add_argument("--spi-div", type=int, default=C.SPI_DIV_MAX, choices=range(C.SPI_DIV_MAX + 1),
                   metavar="N", help="SPI_DIV for our master (default 15, SCLK 1.5 MHz, close "
                                     "to Rhythm's 1.4 MHz at 2 kS/s, so TAP_DELAY's window is "
                                     "as wide as on the real bus)")
    p.add_argument("--delay", type=int, default=None, metavar="N", help="TAP_DELAY for the test")
    p.add_argument("--udp-port", type=int, default=C.UDP_PORT)
    p.add_argument("--drain", type=float, default=0.3, metavar="S")
    p = sub.add_parser("compare", help="chain recording vs RHX recording, every sample")
    p.add_argument("recording", type=pathlib.Path, help="the .chaintap file from `record`")
    p.add_argument("rhx", nargs="+", help="RHX save: folder (One File Per Signal Type), "
                                          "info.rhd, or .rhd file(s) (Traditional; several if "
                                          "RHX started a new file every N minutes)")
    p.add_argument("--port", default=None, help="RHX port prefix to compare (default: the "
                                                "one with most amplifier channels)")
    p.add_argument("--anchors", type=int, default=64, help="fingerprint anchors for alignment")
    p.add_argument("--show", type=int, default=20, metavar="N", help="list the first N mismatches")
    p.add_argument("--json", type=pathlib.Path, default=None, metavar="PATH",
                   help="report path (default docs/results/tapcheck_compare_<time>.json)")
    return ap


def fake_board(faults):
    """A tap build of the fake board unless 'no_tap' is asked for; an ESP drains it."""
    from .fakeboard import FakeBoard, VirtualClock, make_faults
    f = make_faults(*faults)
    f.tap = "no_tap" not in faults
    f.esp_present = True
    clock = VirtualClock()
    return FakeBoard(f, clock=clock), clock


def fake_selftest_capture(board, clock, playback):
    """What the ESP would deliver in loopback: the emulator's playback, one frame per sweep."""
    def capture(seconds):
        t0 = clock()
        clock.sleep(seconds)
        period = board.regs[C.REG["SAMPLE_PERIOD"]]
        n = int(seconds * C.SYS_CLK_HZ / period) // C.FRAMES_PER_DGRAM * C.FRAMES_PER_DGRAM
        raw = []
        for d in range(n // C.FRAMES_PER_DGRAM):
            fr = []
            for i in range(C.FRAMES_PER_DGRAM):
                seq = d * C.FRAMES_PER_DGRAM + i
                vals = playback.expected(seq + 11) if playback else [0] * C.N_CH
                fr.append(F.build_frame(seq, vals, flags=C.FLAG_TAP))
            raw.append((t0 + d * 0.0025, F.build_datagram(fr)))
        return raw
    return capture


def main(argv=None, out=print) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.cmd == "compare":
        try:
            return cmd_compare(args, out)
        except Abort as e:
            out(f"cannot run: {e}")
            return 2
    if args.fake is None and not args.serial:
        ap.error("--serial PORT is required (or --fake for a dry run)")
    sleep = time.sleep
    if args.fake is not None:
        board, clock = fake_board(args.fake)
        ctrl, sleep = Ctrl(ser=board), clock.sleep
        args.settle = 0.0
        args.serial = "fake:" + ("+".join(args.fake) or "tap")
    else:
        try:
            ctrl = Ctrl(args.serial, baud=args.baud)
        except ImportError as e:
            out(f"cannot run: pyserial is not installed ({e})")
            return 2
        except Exception as e:  # pyserial's SerialException, OSError
            out(f"cannot open {args.serial}: {e}")
            return 2
    try:
        if args.cmd == "status":
            return cmd_status(ctrl, args, out, sleep)
        if args.cmd == "delay-sweep":
            return cmd_delay_sweep(ctrl, args, out, sleep)
        if args.cmd == "record":
            if args.fake is not None:
                out("record needs the real board and UDP traffic; --fake is for status, "
                    "delay-sweep and selftest")
                return 2
            return cmd_record(ctrl, args, out, sleep)
        if args.cmd == "selftest":
            cap = None
            if args.fake is not None:
                cap = fake_selftest_capture(board, clock, _playback())
            return cmd_selftest(ctrl, args, out, sleep, capture=cap)
    except Abort as e:
        out(f"cannot run: {e}")
        return 2
    except CtrlError as e:
        out(f"control link failed: {e}")
        return 2
    finally:
        ctrl.close()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
