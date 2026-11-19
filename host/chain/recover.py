"""Keep the chain streaming through unplugs and replugs.

What can be pulled out mid-recording, and what each does to the chain:

  sleeve / headstage  The chips lose power, and every register comes back as
                      power-up garbage: most amplifiers off, the wrong ADC
                      output format, no calibration. While it is out, the
                      FPGA's per-sweep READ(59) link check fails on every sweep
                      (the converter's MISO receivers idle high). Once it is
                      back the check passes again -- an unconfigured chip still
                      returns its ROM -- but every channel reads nonsense until
                      the chip is configured and calibrated again. Seen on the
                      bench on 2026-11-18: 43 of 44 registers wrong after a
                      sleeve replug, link check clean.
  ESP32-C5            The TCP stream drops; the node reconnects by itself and
                      TcpSource takes the new connection. Nothing to do here.
  Arty USB            The FPGA loses its bitstream (configuration is volatile)
                      and the serial port disappears. The port has to be
                      reopened, the bitstream reloaded and bring-up run again.

LinkWatch watches the FPGA's link registers from its own thread and puts the
chain back together:

  * a lane whose last READ(59) word stops being the RHD2164 marker, or whose
    error counter jumps, is down (the counters saturate at 0xffff, about 33 s
    at 2 kS/s, so the live last-word register is the primary signal);
  * when a lane comes back and nothing has changed for `settle` seconds, the
    chips are reconfigured with bring-up's own S4/S5 at the running SPI_DIV
    and PHASE_SELECT (a couple of seconds), falling back to a full S0, S2-S5
    with a fresh phase scan if that does not take;
  * a board that stops answering is reprogrammed, if a programmer is given;
  * a port that disappears is reopened every few seconds.

A lane that was down from the start (one chip connected on purpose) is noted
and left alone; if it later comes up, that counts as a replug. A manual
reconfigure can be requested at any time (the viewer binds it to `r`).

It is a drop-in for viewer.LinkPoller: `.poll()` returns the latest LinkStat,
`.error` is a one-line note for the status line, and `.ctrl` is the control
port it currently holds (it may have reopened it).
"""
from __future__ import annotations

import collections
import dataclasses
import pathlib
import shutil
import subprocess
import threading
import time

from . import bringup as B
from . import contract as C
from . import diag as D
from . import rhd2164 as R
from .ctrl import CtrlError

MARKERS = (R.MARKER_A, R.MARKER_B, R.MARKER_A, R.MARKER_B)   # READ(59), per lane
DOWN_MIN_ERRORS = 100      # errors on a lane within one poll that mean "down" ...
DOWN_SHARE = 0.25          # ... or this share of the poll's checks, if larger
RETRY_AFTER_S = 5.0        # after a failed recovery, wait this long and try again

REPO = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_BITSTREAM = REPO / "fpga" / "bitstreams" / "headstage_arty_s7_25_div6.bit"


def openfpgaloader(bitstream: pathlib.Path = DEFAULT_BITSTREAM, board: str = "arty_s7_25",
                   run=subprocess.run):
    """A programmer for LinkWatch: () -> "" on success, else why not."""
    def program() -> str:
        exe = shutil.which("openFPGALoader")
        if not exe:
            return "openFPGALoader is not on PATH"
        if not pathlib.Path(bitstream).exists():
            return f"no bitstream at {bitstream}"
        try:
            r = run([exe, "-b", board, str(bitstream)], capture_output=True, text=True,
                    timeout=120)
        except (OSError, subprocess.SubprocessError) as e:
            return f"openFPGALoader: {e}"
        if r.returncode != 0:
            return "openFPGALoader failed: " + (r.stderr or r.stdout or "").strip()[-200:]
        return ""
    return program


def lanes_text(lanes) -> str:
    return ", ".join(D.LANES[l] for l in sorted(lanes)) or "none"


class LinkWatch:
    """See the module docstring. step() is one poll; start() runs it on a thread."""

    def __init__(self, ctrl, opts: B.Options, open_ctrl=None, program=None,
                 interval: float = 0.5, settle: float = 1.0, reopen_every: float = 2.0,
                 clock=time.monotonic, sleep=time.sleep, out=None):
        self.ctrl = ctrl
        self.o = opts
        self.open_ctrl = open_ctrl        # () -> Ctrl, to reopen a lost port
        self.program = program            # () -> "" or an error, to reload the FPGA
        self.interval, self.settle, self.reopen_every = interval, settle, reopen_every
        self.clock, self.sleep = clock, sleep
        self.out = out or (lambda *_: None)

        self.last = None                  # LinkStat, for the status line
        self.note = ""                    # one line, for the status line
        self.state = "starting"
        self.recoveries = 0
        self.events = collections.deque(maxlen=100)   # (clock, text)

        self._prev = None
        self._up = None                   # set of lanes up at the last poll
        self._seen_up = set()             # lanes ever up while we watched
        self._changed_at = None           # clock of the last lane up/down change
        self._came_back = False           # some lane went down -> up since the last recovery
        self._retry_at = None
        self._next_reopen = 0.0
        self._expect_stream = False      # we have seen it streaming: a stop is not ours
        self._manual = threading.Event()
        self._stop = threading.Event()
        self.th = None

    # --- LinkPoller-compatible surface --------------------------------------
    def poll(self):
        return self.last

    @property
    def error(self) -> str:
        return self.note

    # --- thread -------------------------------------------------------------
    def start(self) -> "LinkWatch":
        self.th = threading.Thread(target=self._loop, name="link-watch", daemon=True)
        self.th.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self.th is not None:
            self.th.join(timeout=10.0)

    def request_reconfigure(self) -> None:
        self._manual.set()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.step()
            except Exception as e:          # never let the watcher die silently
                self._event(f"watcher error: {type(e).__name__}: {e}")
            self._stop.wait(self.interval)

    # --- one poll -----------------------------------------------------------
    def step(self) -> None:
        now = self.clock()
        if self.ctrl is None:
            self._reopen(now)
            return
        try:
            if self._manual.is_set():
                self._manual.clear()
                self._recover("reconfigure requested", full=False)
                return
            if self._retry_at is not None:
                if now < self._retry_at:
                    return
                self._retry_at = None
                self._recover("retrying recovery", full=True)
                return
            ls = D.read_link(self.ctrl)
            running = bool(self.ctrl.read("CTRL") & 1)
        except CtrlError as e:
            self._board_silent(str(e))
            return
        except OSError as e:
            self._port_lost(e)
            return
        self.last = ls
        if not running:
            if self._expect_stream:
                # Nobody here stopped it: the FPGA was reset or reprogrammed
                # (by hand after a power loss, for instance).
                self._recover("the FPGA stopped streaming (reset or reprogrammed)", full=True)
            else:
                self.state, self.note = "idle", "not streaming"
            return
        self._expect_stream = True
        self._track(ls, now)

    def _track(self, ls, now: float) -> None:
        prev, self._prev = self._prev, ls
        dchk = (ls.checks - prev.checks) & 0xFFFFFFFF if prev else 0
        up = set()
        for lane in range(D.N_LANES):
            derr = (ls.errors[lane] - prev.errors[lane]) if prev else 0
            heavy = derr >= max(DOWN_MIN_ERRORS, DOWN_SHARE * dchk)
            if (ls.last[lane] & 0xFFFF) == MARKERS[lane] and not heavy:
                up.add(lane)
        if self._up is None:                      # first look while streaming
            self._up, self._seen_up = up, set(up)
            self._changed_at = now
            self.state = "streaming"
            absent = set(range(D.N_LANES)) - up
            self.note = f"not answering: {lanes_text(absent)}" if absent else ""
            return
        went_down, came_up = self._up - up, up - self._up
        if went_down or came_up:
            self._changed_at = now
            if went_down:
                self._event(f"link down: {lanes_text(went_down)} (unplugged?)")
            if came_up:
                self._event(f"link back: {lanes_text(came_up)}")
                self._came_back = True
            self._up = up
        if self._came_back and up and now - self._changed_at >= self.settle:
            self._came_back = False
            self._recover(f"{lanes_text(up)} back after an unplug", full=False)
            return
        lost = self._seen_up - up
        if lost:
            self.state = "down"
            self.note = (f"LINK DOWN {lanes_text(lost)} for {now - self._changed_at:.0f} s: "
                         f"plug it back in; the chips are reconfigured when it returns")
        elif self._came_back:
            self.note = "link back: waiting a moment, then reconfiguring the chips"
        else:
            self.state = "streaming"
            absent = set(range(D.N_LANES)) - up
            done = f"chips reconfigured x{self.recoveries}" if self.recoveries else ""
            gone = f"not answering: {lanes_text(absent)}" if absent else ""
            self.note = " | ".join(x for x in (done, gone) if x)

    # --- recovery -----------------------------------------------------------
    def _recover(self, why: str, full: bool) -> None:
        self.state = "recovering"
        self.note = f"reconfiguring the chips ({why}) ..."
        self._event(f"recovering: {why}")
        t0 = self.clock()
        try:
            how = "quick: S4+S5 at the running clock and phase"
            ok = not full and self._quick()
            if not ok:
                how = "full bring-up with a phase scan"
                ok = self._full()
        except CtrlError as e:
            self._board_silent(str(e))
            return
        except OSError as e:
            self._port_lost(e)
            return
        took = self.clock() - t0
        if ok:
            self.recoveries += 1
            self.state = "streaming"
            self.note = f"chips reconfigured ({why}), {took:.1f} s"
            self._event(f"recovered in {took:.1f} s ({how})")
            self._prev, self._up, self._came_back = None, None, False
        else:
            self.state = "failed"
            self._retry_at = self.clock() + RETRY_AFTER_S
            self.note = (f"could not reconfigure the chips ({why}); retrying every "
                         f"{RETRY_AFTER_S:.0f} s -- check the sleeve and the GND wires")
            self._event("recovery failed")

    def _quick(self) -> bool:
        """S4 + S5 at the running clock and phase, on the lanes that answer."""
        c = self.ctrl
        lanes = sorted(self._up or ())
        if not lanes:
            return False
        c.set_run(False)
        div, phase = c.read("SPI_DIV"), c.read("PHASE_SELECT")
        period = c.read("SAMPLE_PERIOD")
        b = B.Bringup(lambda: c, dataclasses.replace(self.o, spi_div=div, phase=phase),
                      out=self.out, sleep=self.sleep, clock=self.clock)
        b.ctrl, b.diag, b.spi_div, b.phase = c, D.Diag(c, sleep=self.sleep, clock=self.clock), \
            div, phase
        b.lane_ok = [l in lanes for l in range(D.N_LANES)]
        b.line_ok = [b.lane_ok[2 * k] or b.lane_ok[2 * k + 1] for k in (0, 1)]
        b.run(only=("S4", "S5"))
        want = [k for k in (0, 1) if b.lane_ok[2 * k]]
        if not want or not all(b.chip_cfg_ok[k] for k in want) or \
                any(s.status == "FAIL" for s in b.stages):
            self._event("quick reconfigure did not take; running full bring-up")
            return False
        c.write("SAMPLE_PERIOD", period)
        c.write("MODE", 0)
        c.set_run(True)
        return bool(c.read("CTRL") & 1)

    def _full(self) -> bool:
        """S0, S2-S5 with a fresh phase scan, then run."""
        try:
            div = self.ctrl.read("SPI_DIV") or self.o.spi_div
        except CtrlError:
            div = self.o.spi_div
        opts = dataclasses.replace(self.o, spi_div=div, phase=None)
        try:
            self.ctrl.set_run(False)
        except CtrlError:
            pass
        _b, rep = B.prepare(self.ctrl, opts, out=self.out, sleep=self.sleep,
                            clock=self.clock, stream=True)
        return rep.get("result") != "FAIL" and bool(self.ctrl.read("CTRL") & 1)

    # --- the board or the port went away ---------------------------------------
    def _board_silent(self, why: str) -> None:
        """Port open, board not answering: it lost its bitstream (power) or is mid-load."""
        try:
            self.ctrl.resync(0.05)
            if self.ctrl.read("ID") == C.ID_EXPECTED:
                return                    # a one-off garbled line; carry on
        except CtrlError:
            pass
        except OSError as e:
            self._port_lost(e)
            return
        self._event(f"FPGA not answering ({why})")
        if self.program is None:
            self.state = "fpga"
            self.note = ("FPGA not answering: reload the bitstream (openFPGALoader -b arty_s7_25 "
                         "...); the chips are reconfigured once it answers")
            return
        self.state = "fpga"
        self.note = "FPGA lost its bitstream: reloading it ..."
        self._close()                     # let the programmer have the FTDI to itself
        err = self.program()
        if err:
            self.note = f"could not reload the FPGA: {err}"
            self._event(self.note)
        else:
            self._event("bitstream reloaded")
        self._next_reopen = 0.0           # reopen at once, then full bring-up

    def _port_lost(self, e) -> None:
        self._event(f"control port lost: {e}")
        self._close()
        self.state = "port"
        self.note = "Arty USB disconnected: waiting for it to come back ..."
        self._next_reopen = self.clock() + self.reopen_every

    def _close(self) -> None:
        if self.ctrl is not None:
            try:
                self.ctrl.close()
            except Exception:
                pass
        self.ctrl = None
        self._prev = self._up = None

    def _reopen(self, now: float) -> None:
        if self.open_ctrl is None or now < self._next_reopen:
            return
        self._next_reopen = now + self.reopen_every
        try:
            ctrl = self.open_ctrl()
        except (OSError, CtrlError) as e:
            self.note = f"Arty USB disconnected: waiting for it ({type(e).__name__})"
            return
        self.ctrl = ctrl
        self._event("control port reopened")
        try:
            ctrl.resync(0.05)
            ok = ctrl.read("ID") == C.ID_EXPECTED
        except CtrlError:
            ok = False
        except OSError as e:
            self._port_lost(e)
            return
        if not ok:
            self._board_silent("no reply after reopening the port")
            return
        # A board that was unplugged lost its configuration with its power: the
        # FPGA registers and the chips both start over.
        self._recover("Arty reconnected", full=True)

    # --- log ------------------------------------------------------------------
    def _event(self, text: str) -> None:
        if self.events and self.events[-1][1] == text:
            return                        # a state that persists is logged once
        self.events.append((self.clock(), text))
        self.out(f"[link] {text}")
