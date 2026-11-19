"""Live view of all 128 channels.

    python3 -m chain.viewer --udp                          # board already streaming via the ESP
    python3 -m chain.viewer --tcp                          # same, node built with CHAIN_SINK_TCP
    python3 -m chain.viewer --udp --serial PORT --start    # configure (bring-up S0, S2-S5), run
    python3 -m chain.viewer --uart --serial PORT           # no ESP: diagnostic CONVERT snapshots
    python3 -m chain.viewer --udp --headless 10            # no display: per-channel stats

Two sources feed one set of per-channel buffers:

  --udp   datagrams on UDP_PORT, parsed with chain.frame. Time comes from each
          frame's t_cap (the FPGA's own clock), so the x axis is exact and gaps
          in seq are counted, never papered over. With --serial the status line
          also shows the FPGA's LINK_ERR counters.
  --tcp   the same datagrams, back to back on a TCP stream the node opens to
          this machine's port (node built with CONFIG_CHAIN_SINK_TCP). Cut by
          size, since every record is DGRAM_BYTES; everything after is as --udp.
  --uart  repeated diagnostic-list CONVERT sweeps over the control UART. Every
          snapshot costs ~100 UART round trips, so this runs at a few Hz at best
          (less where the USB serial driver adds latency per read): enough to
          see which channels are alive without an ESP, not to see EMG.

The window shows all 128 channels as horizontal traces in one panel, channel 0
on top, and keeps --history seconds (default 60) so you can scroll back
through them: the scrollbar under the panel, or the keys listed in Scope. The
history is a numpy ring (chain.scroll) filled on the acquisition thread; the
panel is decimated to one min/max column per pixel and rasterized by numpy.

Everything up to the drawing (buffers, loss, statistics, the status line) is
plain Python and tested; matplotlib and numpy are imported only to draw.
"""
import argparse
import array
import collections
import itertools
import select
import socket
import threading
import time
from dataclasses import dataclass, replace

from . import bringup as B
from . import contract as C
from . import frame as F
from . import rhd2164 as R
from .ctrl import Ctrl, CtrlError

U32 = 0xFFFFFFFF
RAIL_LSB = 32700
RESTART_BACK = 1 << 16     # a seq this far behind the newest is a new run (~33 s at 2 kS/s)
MODULES = ("chip0 A", "chip0 B", "chip1 A", "chip1 B")


# ---------------------------------------------------------------------------
# Plumbing
# ---------------------------------------------------------------------------

@dataclass
class ChannelStat:
    ch: int
    n: int
    mean_uv: float
    rms_uv: float            # about the mean: the signal, not the offset
    min_uv: float
    max_uv: float
    dead: bool               # no change at all over the window
    railed: bool             # touched the ADC rail in the window

    @property
    def flag(self) -> str:
        return "railed" if self.railed else "dead" if self.dead else ""


class Buffers:
    """Rolling samples, their times, and an honest count of what is missing.

    One int16 row per frame rather than one list per channel: a redraw then
    hands the whole window to numpy in one frombuffer instead of converting
    a quarter of a million Python ints ten times a second.

    `history`, when set (the display sets a chain.scroll.History), also gets
    every stored frame, with a flag where frames went missing just before it:
    the long scrollable record, which these deques are too slow to hold.
    """

    def __init__(self, n_ch: int = C.N_CH, depth: int = 4000, rate: float = 2000.0):
        self.n_ch = n_ch
        self.depth = depth
        # Nominal FPGA ticks per frame: picks the right 44.7 s wrap count of
        # t_cap across an outage (see push_frame).
        self.period_ticks = C.SYS_CLK_HZ / rate if rate > 0 else C.SYS_CLK_HZ / 2000.0
        self.restarts = 0          # the stream started over (seq cleared, FPGA reset)
        self.history = None
        self.rows = collections.deque(maxlen=depth)   # array('h') of n_ch per frame
        self.t = collections.deque(maxlen=depth)
        self.frames = 0
        self.first_seq = None
        self.last_seq = None
        self.gaps = 0              # places where seq jumped
        self.lost = 0              # frames those jumps skipped and that never came late
        self.duplicates = 0
        self.reordered = 0
        self.malformed = 0
        self.arrivals = collections.deque()  # (host time, frames) for the rate
        self._tcap_last = None
        self._tcap_s = 0.0
        self._seen = set()         # recent seqs, to tell a duplicate from a late frame
        self._order = collections.deque()

    def _remember(self, seq: int) -> None:
        self._seen.add(seq)
        self._order.append(seq)
        if len(self._order) > 4 * self.depth:
            self._seen.discard(self._order.popleft())

    def _new_run(self) -> None:
        """The stream started over: forget the old run's seqs, keep its samples."""
        self.restarts += 1
        self.last_seq = None
        self._seen.clear()
        self._order.clear()

    # --- input --------------------------------------------------------------
    def push(self, seq: int, samples, t: float, host_t: float = None,
             restart: bool = False) -> bool:
        """One frame. False if it was a duplicate or arrived out of order (not stored).

        A seq far behind the newest (RESTART_BACK) is a new run, not a late frame:
        the board cleared its counters (a bring-up, a recovery) or was reset. Read
        as "late", every frame of the new run would be dropped until it passed the
        old run's last seq -- a frozen display for as long as the old run lasted.
        """
        if not restart and self.last_seq is not None:
            back = (self.last_seq - seq) & U32
            restart = RESTART_BACK <= back < 1 << 31
        if restart:
            self._new_run()
        elif seq in self._seen:
            self.duplicates += 1
            return False
        gap = restart              # the trace breaks where a new run starts
        if self.last_seq is not None:
            d = (seq - self.last_seq) & U32
            if d >= 1 << 31:
                # Older than what is already drawn: not stored, but it did arrive,
                # so it no longer counts as lost if it fell inside a counted gap.
                self.reordered += 1
                if (seq - self.first_seq) & U32 < 1 << 31 and self.lost:
                    self.lost -= 1
                self._remember(seq)
                return False
            if d > 1:
                self.gaps += 1
                self.lost += d - 1
                gap = True
        else:
            self.first_seq = seq
        self._remember(seq)
        self.last_seq = seq
        row = array.array("h", samples[:self.n_ch])
        if len(row) < self.n_ch:
            row.extend([0] * (self.n_ch - len(row)))
        self.rows.append(row)
        self.t.append(t)
        if self.history is not None:
            self.history.append(row, t, gap)
        self.frames += 1
        self.arrivals.append((t if host_t is None else host_t, 1))
        return True

    def push_frame(self, fr: F.Frame, host_t: float) -> bool:
        """A parsed frame; time from t_cap, unwrapped across the 44.7 s counter wrap.

        t_cap is a 32-bit tick counter, so a delta alone cannot tell 5 s from
        49.7 s. The seq delta can: the FPGA numbers every frame it builds,
        including the ones an outage lost, so (seq delta x nominal period) picks
        the wrap count. Only frames that will be stored move the time base: a
        late or duplicate frame (older seq, older t_cap) must not drag it back.
        A frame with an older seq but a *newer* t_cap is a new run (counters
        cleared; t_cap kept counting), and so is one far behind (FPGA reset).
        """
        if self._tcap_last is None:
            self._tcap_last = fr.t_cap
            return self.push(fr.seq, fr.samples(), self._tcap_s, host_t)
        raw = (fr.t_cap - self._tcap_last) & U32
        signed = raw if raw < 1 << 31 else raw - (1 << 32)
        restart = False
        dseq = 1
        if self.last_seq is not None:
            dseq = (fr.seq - self.last_seq) & U32
            if dseq == 0 or dseq >= 1 << 31:          # not newer than what is stored
                back = (self.last_seq - fr.seq) & U32
                if signed > 0 or back >= RESTART_BACK:
                    restart = True
                else:                                 # duplicate or late: counted only
                    return self.push(fr.seq, fr.samples(), self._tcap_s, host_t)
        if restart:
            ticks = raw   # through a counter clear t_cap kept counting; after an FPGA
                          # reset it is a guess, and the trace breaks there anyway
        else:
            k = max(0, round((dseq * self.period_ticks - raw) / (1 << 32)))
            ticks = raw + k * (1 << 32)
        self._tcap_last = fr.t_cap
        self._tcap_s += ticks / C.SYS_CLK_HZ
        return self.push(fr.seq, fr.samples(), self._tcap_s, host_t, restart=restart)

    def push_datagram(self, data: bytes, host_t: float) -> int:
        """Frames stored from one datagram; a malformed datagram is counted, not raised."""
        try:
            frames, _trailer = F.parse_datagram(data)
        except F.FrameError:
            self.malformed += 1
            return 0
        n = 0
        for fr in frames:
            try:
                n += self.push_frame(fr, host_t)
            except F.FrameError:      # e.g. a compressed frame this viewer cannot unpack
                self.malformed += 1
        return n

    # --- output -------------------------------------------------------------
    def frame_rate(self, now: float, span: float = 2.0) -> float:
        while self.arrivals and self.arrivals[0][0] < now - span:
            self.arrivals.popleft()
        if not self.arrivals:
            return 0.0
        first = self.arrivals[0][0]
        dt = max(now - first, 1e-9)
        return sum(n for _t, n in self.arrivals) / dt if dt >= 0.05 else 0.0

    def loss_pct(self) -> float:
        seen = self.frames + self.lost
        return 100.0 * self.lost / seen if seen else 0.0

    def channel(self, ch: int, last: int = None) -> list[int]:
        rows = list(self.rows)[-last:] if last else self.rows
        return [r[ch] for r in rows]

    def raw(self) -> bytes:
        """The window as native int16 bytes, frame-major (n_frames x n_ch)."""
        return b"".join(r.tobytes() for r in self.rows)

    def window(self, seconds: float):
        """(t, data) for the newest `seconds`: float seconds and int16 (n, n_ch).

        Copies, so the caller may keep them while this buffer moves on. Only
        the thread that pushes may call this (the deques are not locked).
        """
        import numpy as np
        n = len(self.t)
        if not n:
            return np.empty(0), np.empty((0, self.n_ch), np.int16)
        t = np.fromiter(self.t, float, n)
        k = int(np.searchsorted(t, t[-1] - seconds, side="left"))
        rows = itertools.islice(self.rows, k, n)
        data = np.frombuffer(b"".join(r.tobytes() for r in rows), np.int16)
        return t[k:], data.reshape(-1, self.n_ch)

    def stats(self, last: int = None) -> list[ChannelStat]:
        out = []
        for ch in range(self.n_ch):
            v = self.channel(ch, last)
            n = len(v)
            if not n:
                out.append(ChannelStat(ch, 0, 0.0, 0.0, 0.0, 0.0, False, False))
                continue
            mean = sum(v) / n
            rms = (sum((x - mean) ** 2 for x in v) / n) ** 0.5
            lo, hi = min(v), max(v)
            out.append(ChannelStat(ch, n, mean * R.SCALE_UV, rms * R.SCALE_UV,
                                   lo * R.SCALE_UV, hi * R.SCALE_UV,
                                   dead=n >= 2 and lo == hi,
                                   railed=max(abs(lo), abs(hi)) >= RAIL_LSB))
        return out


def status_line(buf: Buffers, now: float, source: str, link=None, note: str = "") -> str:
    """One line: rate, frames, loss, malformed, and LINK_ERR when a port is open."""
    stream = source != "uart"       # udp or tcp: datagrams, not snapshots
    unit = "fr/s" if stream else "snap/s"
    parts = [f"{source.upper()} {buf.frame_rate(now):7.1f} {unit}",
             f"frames {buf.frames}",
             f"seq gaps {buf.gaps} ({buf.lost} lost, {buf.loss_pct():.2f}%)"]
    if buf.duplicates or buf.reordered:
        parts.append(f"dup {buf.duplicates} reord {buf.reordered}")
    if buf.restarts:
        parts.append(f"restarts {buf.restarts}")
    if stream:
        parts.append(f"malformed {buf.malformed}")
    if link is not None:
        errs = " ".join(f"{MODULES[l].replace('chip', 'c').replace(' ', '')} {link.errors[l]}"
                        for l in range(4))
        parts.append(f"LINK_ERR {errs} / {link.checks} checks")
    if note:
        parts.append(note)
    return " | ".join(parts)


def stats_table(stats: list[ChannelStat]) -> str:
    lines = ["  ch  module   amp   mean uV    rms uV    min uV    max uV  n      flag"]
    for s in stats:
        lines.append(f"{s.ch:4d}  {MODULES[s.ch // 32]:<7} {s.ch % 32:4d} {s.mean_uv:9.1f} "
                     f"{s.rms_uv:9.2f} {s.min_uv:9.1f} {s.max_uv:9.1f} {s.n:<6d} {s.flag}")
    return "\n".join(lines)


def flagged_summary(stats: list[ChannelStat]) -> str:
    from .diag import runs
    out = []
    for flag in ("dead", "railed"):
        chans = [s.ch for s in stats if s.flag == flag]
        if chans:
            out.append(f"{flag}: " + ", ".join(f"{a}-{b}" if a != b else str(a)
                                               for a, b in runs(chans)))
    return "; ".join(out) or "no dead or railed channels"


# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------

class UdpSource:
    """Datagrams from a socket, received on a thread so drawing never drops one."""

    def __init__(self, port: int = C.UDP_PORT, sock=None):
        if sock is None:
            from .receiver import open_socket
            sock = open_socket(port)
        self.sock = sock
        self.sock.settimeout(0.2)
        self.q = collections.deque()
        self._done = threading.Event()
        self.th = threading.Thread(target=self._rx, daemon=True)
        self.th.start()

    def _rx(self):
        while not self._done.is_set():
            try:
                data = self.sock.recv(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            self.q.append((time.monotonic(), data))

    def poll(self, buf: Buffers) -> int:
        n = 0
        while self.q:
            t, data = self.q.popleft()
            n += buf.push_datagram(data, t)
        return n

    def close(self):
        self._done.set()
        self.th.join(timeout=1.0)
        self.sock.close()


class TcpSource:
    """Datagrams from the node's TCP stream (CONFIG_CHAIN_SINK_TCP), on a thread.

    The node connects to us and writes each DGRAM_BYTES datagram, trailer and
    all, straight into the stream. TCP has no message boundaries, but every
    record is the same size, so records are cut by count from the start of each
    connection. A new connection (the node rebooted, or dropped a connection
    whose send failed) replaces the current one at once: a rebooted node never
    closes its old connection, which would otherwise sit open and silent. Any
    partial record left on the old connection is thrown away and counted in
    `partial_drops`, never glued onto the new stream.
    """

    def __init__(self, port: int = C.UDP_PORT, bind: str = ""):
        ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            # Inherited by accepted sockets: a deep window rides out a pause on
            # this side without the node's send blocking.
            ls.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        except OSError:
            pass
        ls.bind((bind, port))
        ls.listen(2)
        self.lsock = ls
        self.port = ls.getsockname()[1]
        self.conn = None
        self.peer = None
        self.connections = 0
        self.partial_drops = 0
        self.q = collections.deque()
        self._done = threading.Event()
        self.th = threading.Thread(target=self._rx, daemon=True)
        self.th.start()

    def state(self) -> str:
        if self.conn is not None and self.peer:
            return f"connected to {self.peer[0]}"
        return f"waiting for the node on tcp :{self.port}"

    def _drop_conn(self, pend: bytearray) -> None:
        if pend:
            self.partial_drops += 1
            pend.clear()
        if self.conn is not None:
            try:
                self.conn.close()
            except OSError:
                pass
        self.conn = None

    def _rx(self):
        pend = bytearray()
        while not self._done.is_set():
            socks = [self.lsock] + ([self.conn] if self.conn is not None else [])
            try:
                ready, _, _ = select.select(socks, [], [], 0.2)
            except (OSError, ValueError):
                break
            if self.lsock in ready:
                try:
                    c, peer = self.lsock.accept()
                except OSError:
                    continue
                # Whole records the old connection still holds came before
                # anything on the new one: take them before letting it go.
                self._drain(pend)
                self._drop_conn(pend)
                self.conn, self.peer = c, peer
                self.connections += 1
                continue
            if self.conn is not None and self.conn in ready:
                try:
                    chunk = self.conn.recv(1 << 16)
                except OSError:
                    chunk = b""
                if not chunk:
                    self._drop_conn(pend)
                    continue
                pend += chunk
                self._cut(pend)

    def _cut(self, pend: bytearray) -> None:
        """Queue every whole record at the front of pend."""
        size = C.DGRAM_BYTES
        n = len(pend) // size
        if n:
            t = time.monotonic()
            for i in range(n):
                self.q.append((t, bytes(pend[i * size:(i + 1) * size])))
            del pend[:n * size]

    def _drain(self, pend: bytearray) -> None:
        """Read whatever the current connection has already received, without waiting."""
        if self.conn is None:
            return
        try:
            self.conn.setblocking(False)
            while True:
                chunk = self.conn.recv(1 << 16)
                if not chunk:
                    break
                pend += chunk
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            pass
        self._cut(pend)

    def poll(self, buf: Buffers) -> int:
        n = 0
        while self.q:
            t, data = self.q.popleft()
            n += buf.push_datagram(data, t)
        return n

    def close(self):
        self._done.set()
        self.th.join(timeout=1.0)
        self._drop_conn(bytearray())
        self.lsock.close()


def source_note(source) -> str:
    """What the source itself has to say on the status line (TCP: connection)."""
    state = getattr(source, "state", None)
    return state() if callable(state) else ""


class UartSource:
    """One diagnostic CONVERT sweep per poll: 128 values, no ESP needed."""

    def __init__(self, diag, clock=time.monotonic):
        self.diag = diag
        self.clock = clock
        self.seq = 0

    def poll(self, buf: Buffers) -> int:
        from .diag import convert_snapshot
        vals = convert_snapshot(self.diag)
        t = self.clock()
        buf.push(self.seq, vals, t, t)
        self.seq += 1
        return 1

    def close(self):
        pass


class LinkPoller:
    """LINK_* every `interval` s from the control port, while streaming."""

    def __init__(self, ctrl, interval: float = 1.0, clock=time.monotonic):
        self.ctrl, self.interval, self.clock = ctrl, interval, clock
        self.last = None
        self.error = ""
        self._t = -1e9

    def poll(self):
        from .diag import read_link
        now = self.clock()
        if now - self._t >= self.interval:
            self._t = now
            try:
                self.last = read_link(self.ctrl)
                self.error = ""
            except CtrlError as e:
                self.error = f"control port: {e}"
        return self.last


def run_headless(source, buf: Buffers, seconds: float, out=print, clock=time.monotonic,
                 sleep=time.sleep, link: LinkPoller = None, kind: str = "udp",
                 every: float = 1.0) -> list[ChannelStat]:
    """Feed the buffers for `seconds`, print a status line each second, then the stats."""
    t0 = clock()
    next_print = t0 + every
    while clock() - t0 < seconds:
        got = source.poll(buf)
        lk = link.poll() if link else None
        now = clock()
        if now >= next_print:
            note = (link.error if link else "") or source_note(source)
            out(status_line(buf, now, kind, lk, note))
            next_print = now + every
        if not got and kind != "uart":
            sleep(0.02)
    lk = link.poll() if link else None
    st = buf.stats()
    out(status_line(buf, clock(), kind, lk, source_note(source)))
    out(stats_table(st))
    out(flagged_summary(st))
    return st


# ---------------------------------------------------------------------------
# What the display shows, as plain functions (tested without a display)
# ---------------------------------------------------------------------------
# The activity strip is 4 rows x 32 columns: module m (chip0 A, chip0 B,
# chip1 A, chip1 B) is row m, its channels in order left to right.
GRID_ROWS, GRID_COLS = 4, 32
MIN_HALF_UV = 2.0        # a lane never magnifies below ±2 µV: ADC noise stays small
LANE_FILL = 0.45         # a channel's 99th-percentile excursion fills this much of a lane
LANE_CLIP = 0.48         # and nothing ever leaves it
STATS_MAX = 8_000        # frames sampled for mean / RMS; longer windows are strided
SCALE_COLS = 512         # columns sampled for each lane's centre and scale

# Colours (light surface). One sequential hue for magnitude, one series colour
# for the traces, reserved status colours that always come with a text tag.
SURFACE = "#fcfcfb"
INK, INK_2, INK_3 = "#0b0b0b", "#52514e", "#8c8b86"
TRACE = "#2a78d6"
CRITICAL = "#d03b3b"     # railed; frames missing
DEAD_FILL = "#e4e3df"
RULE, GRID_RULE, GAP_TINT = "#efeeea", "#e8e7e2", "#f6dcdc"
TRACK = "#e9e8e3"        # scrollbar track
SEQ_BLUE = ("#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5",
            "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b")
ACT_MIN_UV, ACT_MAX_UV = 1.0, 10_000.0   # activity colour scale, log


def grid_pos(ch: int) -> tuple[int, int]:
    """(row, column) of a channel's tile in the activity strip."""
    return divmod(ch, GRID_COLS)


def grid_channel(row: int, col: int) -> int:
    """The channel shown at (row, column); the inverse of grid_pos."""
    return GRID_COLS * row + col


def fmt_uv(v: float) -> str:
    """Short microvolt label: '850 µV', '1.8 mV', '12 mV'."""
    a = abs(v)
    if a >= 10_000:
        return f"{v / 1000:.0f} mV"
    if a >= 1000:
        return f"{v / 1000:.1f} mV"
    if a >= 10:
        return f"{v:.0f} µV"
    return f"{v:.1f} µV"


def fmt_width(s: float) -> str:
    return f"{s * 1000:.0f} ms" if s < 1 else f"{s:g} s"


def lane_half(p99, shared_half=None):
    """Per channel, the µV that LANE_FILL of a lane stands for: the channel's own
    99th-percentile excursion, or one shared value; never below MIN_HALF_UV."""
    import numpy as np
    if shared_half is None:
        return np.maximum(np.asarray(p99, float), MIN_HALF_UV)
    return np.full(len(p99), max(float(shared_half), MIN_HALF_UV))


def to_lane(uv, centre, half):
    """µV -> lane units, clipped to ±LANE_CLIP so no trace leaves its lane."""
    import numpy as np
    return np.clip(LANE_FILL * (uv - centre) / half, -LANE_CLIP, LANE_CLIP).astype(np.float32)


def lane_values(uv, shared_half=None):
    """Each channel's samples mapped into its own lane, never past it.

    uv is (samples, channels) in microvolts. Every channel is centred on its
    median. With shared_half None each channel is scaled by its own
    99th-percentile excursion, so a channel's activity is visible whatever its
    size; with shared_half (uV) all lanes use one scale and compare directly.
    Values are clipped to +-LANE_CLIP of a lane, so a trace can never draw over
    its neighbour -- the failure that made the first version unreadable.
    Returns (y in lane units, half-range in uV that a full lane means).
    """
    import numpy as np
    centre = np.median(uv, axis=0)
    p99 = np.percentile(np.abs(uv - centre), 99, axis=0)
    half = lane_half(p99, shared_half)
    return to_lane(uv, centre, half), half / LANE_FILL * LANE_CLIP


def scale_stats(lo, hi, draw, ok):
    """(centre, p99, shared p99) per channel from the drawn spans (µV), as
    lane_values computes them from samples (median, 99th percentile of the
    distance from it), on at most SCALE_COLS columns and by order statistics
    (np.partition) rather than np.median / np.percentile, several times faster."""
    import numpy as np
    n_ch = lo.shape[1]
    sel = np.flatnonzero(draw)
    if not len(sel):
        return np.zeros(n_ch, np.float32), np.zeros(n_ch, np.float32), 100.0
    if len(sel) > SCALE_COLS:
        sel = sel[np.linspace(0, len(sel) - 1, SCALE_COLS).astype(np.int64)]
    env = np.ascontiguousarray(np.concatenate([lo[sel], hi[sel]]).T)    # (n_ch, 2k)
    m = env.shape[1]
    centre = np.partition(env, m // 2, axis=1)[:, m // 2]
    dev = np.abs(env - centre[:, None])
    k = int(0.99 * (m - 1))
    p99 = np.partition(dev, k, axis=1)[:, k]
    shared = 100.0
    if ok.any():
        flat = dev[ok].ravel()
        shared = float(np.partition(flat, int(0.99 * (len(flat) - 1)))[int(0.99 * (len(flat) - 1))])
    return centre.astype(np.float32), p99.astype(np.float32), shared


def palette() -> dict:
    """Packed RGBA words for everything the numpy painters draw."""
    from . import scroll as S
    names = dict(bg=SURFACE, trace=TRACE, rail=CRITICAL, dead=INK_3, rule=RULE,
                 sep=INK_3, grid=GRID_RULE, gap_tint=GAP_TINT, gap_mark=CRITICAL,
                 outline=INK, dead_fill=DEAD_FILL)
    packed = S.pack_hex(list(names.values()))
    out = {k: packed[i] for i, k in enumerate(names)}
    out["lut"] = S.seq_lut(SEQ_BLUE)
    return out


@dataclass(frozen=True)
class Request:
    """What the window wants drawn: the time and channel view, the scale mode,
    and the pixel size of the trace panel (w x h, plus a gm-row gap strip) and
    of the activity strip (0 x 0 when hidden or unknown)."""
    seq: int
    tv: object                 # chain.scroll.TimeView
    lv: object                 # chain.scroll.LaneView
    shared: bool = False
    spacing_uv: float = None
    w: int = 1000
    h: int = 0
    gm: int = 0
    map_w: int = 0
    map_h: int = 0
    max_ticks: int = 10


@dataclass
class Snapshot:
    """One ready-to-show frame. Built on the render thread; read-only afterwards."""
    version: int
    req: int           # Request.seq this answers
    total: int         # history frames when built
    k0: int            # first pixel column (absolute; column k covers [k dt, (k+1) dt))
    dt: float
    ncols: int
    final: int         # columns below this could no longer change when built
    t0: float          # visible time span (s since the first frame)
    t1: float
    oldest: float      # what the history holds
    newest: float
    right: float       # right edge of the view
    live: bool
    width: float
    lo: object         # (ncols, n_ch) µV: each column's drawn span
    hi: object
    draw: object       # (ncols,) columns with a trace
    gaps: object       # (ncols,) columns where frames are missing
    n: int             # frames in the window
    mean: object       # (n_ch,) µV
    rms: object        # (n_ch,) µV about the mean
    mn: object         # (n_ch,) µV
    mx: object         # (n_ch,) µV
    dead: object       # (n_ch,) bool: no change at all
    railed: object     # (n_ch,) bool: touched the ADC rail
    centre: object     # (n_ch,) µV, each lane's middle
    p99: object        # (n_ch,) µV, each channel's 99th-percentile excursion
    shared_p99: float
    scale_id: int
    half: object       # (n_ch,) µV that LANE_FILL of each lane stands for, as drawn
    lv: object         # lanes drawn
    shared: bool
    spacing_uv: float
    raster: object     # (h + gm, w) uint32: the trace panel, or None
    geom: tuple        # (w, h, gm) of raster
    colors: object     # (n,) uint32 lane colours used
    ticks: list        # [(t, label)] time ticks in the window
    tick_step: float
    map_raster: object  # (map_h, map_w) uint32 or None
    status: str

    def stat(self, ch: int) -> ChannelStat:
        return ChannelStat(ch, self.n, float(self.mean[ch]), float(self.rms[ch]),
                           float(self.mn[ch]), float(self.mx[ch]), bool(self.dead[ch]),
                           bool(self.railed[ch]))


class Renderer:
    """History + Request -> Snapshot: decimation, statistics, and the finished
    images of the trace panel and the activity strip, all in numpy.

    Work is reused wherever it cannot have changed: decimated columns (see
    chain.scroll.Decimator); the panel image, shifted left by however many
    columns the live view moved, with only the new columns painted (whenever
    lane scale, colours and size are unchanged); the lane scales, recomputed at
    most every SCALE_EVERY s while live; the statistics and the activity strip,
    at most every STATS_EVERY s. A parked view with nothing new costs nothing.
    """
    SCALE_EVERY = 0.25
    STATS_EVERY = 0.1

    def __init__(self, pal: dict = None):
        from . import scroll as S
        self.S = S
        self.pal = pal or palette()
        self.dec = S.Decimator()
        self.prev = None
        self.version = 0
        self._scale = None             # (centre, p99, shared, id)
        self._scale_key, self._scale_t, self._scale_id = None, -1e9, 0
        self._stats = None
        self._stats_key, self._stats_t = None, -1e9
        self._painter = None
        self._map_painter = None

    def render(self, hist, req: Request, status: str = "", now: float = 0.0):
        """The Snapshot for `req`, or None while the history is empty."""
        import numpy as np
        S = self.S
        hist.flush()
        if not hist.count:
            return None
        prev = self.prev
        old, new = hist.t_oldest, hist.t_newest
        tv = req.tv
        ncols = max(1, int(req.w))
        right = S.right_edge(tv, old, new)
        k0, dt = S.column_range(right, tv.width, ncols)
        if prev is not None and prev.req == req.seq and prev.k0 == k0 and prev.dt == dt:
            if prev.total == hist.total and prev.status == status:
                return prev                                     # nothing new
            if k0 + ncols <= prev.final and k0 * dt >= old:     # a parked, finished view
                return self._keep(replace(prev, total=hist.total, oldest=old, newest=new,
                                          status=status))
        t0, t1 = k0 * dt, (k0 + ncols) * dt
        cols = self.dec.columns(hist, k0, ncols, dt)
        i0, i1 = hist.search(t0), hist.search(t1)
        before = hist.frame(i0 - 1) if i0 > 0 else None
        after = hist.frame(i1) if i1 < hist.count else None
        lo, hi, draw, gaps, has = S.assemble(cols, dt, before, after, t_min=old)
        k = np.float32(R.SCALE_UV)
        lo *= k
        hi *= k

        # statistics over the window
        view_key = (tv.live, tv.width, ncols, None if tv.live else k0)
        if (self._stats is None or view_key != self._stats_key
                or now - self._stats_t >= self.STATS_EVERY):
            n = int(cols.count[has].sum())
            if has.any():
                mn = cols.lo[has].min(axis=0).astype(np.int32)
                mx = cols.hi[has].max(axis=0).astype(np.int32)
            else:
                mn = mx = np.zeros(hist.n_ch, np.int32)
            stride = max(1, -(-(i1 - i0) // STATS_MAX))
            _, d, _ = hist.take(np.arange(i0, i1, stride))
            x = d.astype(np.float32)
            mean = x.mean(axis=0) * k if len(x) else np.zeros(hist.n_ch, np.float32)
            rms = x.std(axis=0) * k if len(x) else np.zeros(hist.n_ch, np.float32)
            dead = (mn == mx) & (n >= 2)
            railed = np.maximum(np.abs(mn), np.abs(mx)) >= RAIL_LSB
            self._stats = (n, mean, rms, mn * k, mx * k, dead, railed)
            self._stats_key, self._stats_t = view_key, now
            fresh_stats = True
        else:
            fresh_stats = False
        n, mean, rms, mn_uv, mx_uv, dead, railed = self._stats

        # lane scales
        if (self._scale is None or view_key != self._scale_key
                or (tv.live and now - self._scale_t >= self.SCALE_EVERY)):
            c, p, sh = scale_stats(lo, hi, draw, ~dead & ~railed)
            self._scale_id += 1
            self._scale = (c, p, sh, self._scale_id)
            self._scale_key, self._scale_t = view_key, now
        centre, p99, shared_p99, scale_id = self._scale
        if req.shared:
            half = lane_half(p99, req.spacing_uv or shared_p99)
        else:
            half = lane_half(p99)

        # time ticks, pinned to absolute time like the columns
        ticks, step = S.time_ticks(t0, t1, req.max_ticks)
        gridc = np.zeros(ncols, bool)
        tc = np.array([S.col_of(t, dt) - k0 for t in ticks], np.int64)
        gridc[tc[(tc >= 0) & (tc < ncols)]] = True

        # the trace panel image
        lv = req.lv
        sl = slice(lv.top, lv.top + lv.n)
        colors = np.where(railed[sl], self.pal["rail"],
                          np.where(dead[sl], self.pal["dead"], self.pal["trace"])).astype(np.uint32)
        raster, geom = None, (ncols, req.h, req.gm)
        if req.h >= lv.n:
            p = self._painter
            if p is None or (p.h, p.w, p.gm, p.lv) != (req.h, ncols, req.gm, lv):
                p = self._painter = S.LanePainter(req.h, ncols, req.gm, lv, self.pal,
                                                  sep_px=max(1, req.gm // 2))
            reuse = (prev is not None and prev.raster is not None and prev.geom == geom
                     and prev.lv == lv and prev.scale_id == scale_id and prev.dt == dt
                     and prev.shared == req.shared and prev.spacing_uv == req.spacing_uv
                     and prev.tick_step == step and np.array_equal(prev.colors, colors)
                     and 0 <= k0 - prev.k0 < ncols)
            # Columns to paint: everything, or -- shifting the previous image left --
            # only those that may differ: from the first one that was not final, and
            # at the left edge any the ring has overwritten since (window ~ history).
            spans = [(0, ncols)]
            if reuse:
                shift = k0 - prev.k0
                a = max(0, min(ncols - shift, prev.final - k0))
                left = max(0, min(ncols, S.col_of(old, dt) - k0 + 1))
                spans = [(0, min(left, a)), (a, ncols)]
            if reuse and all(b <= a for a, b in spans):
                raster = prev.raster                            # nothing changed on screen
            else:
                raster = np.empty((req.h + req.gm, ncols), np.uint32)
                if reuse:
                    raster[:, :ncols - shift] = prev.raster[:, shift:]
                for a, b in spans:
                    if b > a:
                        p.paint(raster, a, b, to_lane(lo[a:b, sl], centre[sl], half[sl]),
                                to_lane(hi[a:b, sl], centre[sl], half[sl]), draw[a:b],
                                gaps[a:b], gridc[a:b], colors)

        # the activity strip
        map_raster = prev.map_raster if prev is not None else None
        if req.map_w and req.map_h:
            mp = self._map_painter
            stale = (mp is None or (mp.h, mp.w) != (req.map_h, req.map_w) or map_raster is None
                     or prev.lv != lv or fresh_stats)
            if stale:
                if mp is None or (mp.h, mp.w) != (req.map_h, req.map_w):
                    mp = self._map_painter = S.MapPainter(req.map_h, req.map_w, self.pal,
                                                          GRID_ROWS, GRID_COLS)
                tiles = S.tile_colors(rms, dead, self.pal["lut"], self.pal["dead_fill"],
                                      ACT_MIN_UV, ACT_MAX_UV).reshape(GRID_ROWS, GRID_COLS)
                vis = None
                if lv.n < hist.n_ch:
                    vis = np.zeros(hist.n_ch, bool)
                    vis[sl] = True
                    vis = vis.reshape(GRID_ROWS, GRID_COLS)
                map_raster = mp.paint(tiles, railed.reshape(GRID_ROWS, GRID_COLS), vis)
        else:
            map_raster = None

        return self._keep(Snapshot(
            0, req.seq, hist.total, k0, dt, ncols, self.dec.final, t0, t1, old, new, right,
            tv.live, tv.width, lo, hi, draw, gaps, n, mean, rms, mn_uv, mx_uv, dead, railed,
            centre, p99, shared_p99, scale_id, half, lv, req.shared, req.spacing_uv, raster,
            geom, colors, [(t, S.fmt_time(t, step)) for t in ticks], step, map_raster, status))

    def _keep(self, snap):
        self.version += 1
        snap.version = self.version
        self.prev = snap
        return snap


class Acquirer:
    """Reads the source on one thread and renders snapshots on another.

    The window must never wait on a serial port (a UART snapshot is ~100 round
    trips, about a second), on parsing a few thousand frames a second, or on
    turning 60 s of history into pixels. So:

      acquire thread   source -> Buffers (and the History staging), LINK_*,
                       the status line every 0.25 s
      render thread    History -> Snapshot (Renderer), up to `hz` times a second
                       and at once whenever the window asks for a new view

    The window reads the newest Snapshot (a locked reference swap); after a key
    or a drag it can wait_for() the snapshot answering its request, so the
    screen follows the hand within one render.
    """

    def __init__(self, source, buf: Buffers, kind: str, link: LinkPoller = None,
                 window_s: float = 2.0, hz: float = 60.0, clock=time.monotonic,
                 history_s: float = 60.0, rate: float = 2000.0):
        from . import scroll as S
        self.source, self.buf, self.kind, self.link = source, buf, kind, link
        self.window_s, self.period, self.clock = window_s, 1.0 / hz, clock
        stream = kind != "uart"
        history_s = max(history_s, window_s)
        cap = int(round(history_s * rate)) + 16 if stream else max(256, int(history_s * 20))
        self.history = S.History(buf.n_ch, cap, gap_s=max(0.05, 20.0 / rate) if stream else None)
        buf.history = self.history
        self.renderer = Renderer()
        self.error = ""
        self.render_error = ""
        self.render_s = 0.0               # how long the last render took
        self._req = Request(0, S.TimeView(min(window_s, history_s)), S.LaneView(buf.n_ch, 0))
        self._status = ""
        self._snap = None
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._wake = threading.Event()
        self.th = threading.Thread(target=self._acquire, name="viewer-acquire", daemon=True)
        self.th_render = threading.Thread(target=self._render, name="viewer-render", daemon=True)

    def start(self) -> "Acquirer":
        self.th.start()
        self.th_render.start()
        return self

    # --- the window's side ----------------------------------------------------
    def latest(self):
        with self._lock:
            return self._snap

    def status(self) -> str:
        """The newest status line, also before any frame has arrived."""
        return self._status

    def set_request(self, req: Request) -> None:
        self._req = req
        self._wake.set()

    def wait_for(self, seq: int, timeout: float):
        """The newest snapshot once it answers request `seq` (or later), or None on timeout."""
        with self._cond:
            ok = self._cond.wait_for(lambda: self._snap is not None and self._snap.req >= seq,
                                     timeout)
            return self._snap if ok else None

    # --- work -----------------------------------------------------------------
    def _status_now(self, now: float) -> str:
        lk = self.link.last if self.link else None
        note = ((self.link.error if self.link else "") or self.error or self.render_error
                or source_note(self.source))
        return status_line(self.buf, now, self.kind, lk, note)

    def publish(self) -> None:
        """Render one snapshot for the current request (the render thread's step;
        tests and screenshots call it directly)."""
        now = self.clock()
        status = self._status if self.th.is_alive() else self._status_now(now)
        snap = self.renderer.render(self.history, self._req, status, now)
        if snap is not None:
            with self._cond:
                self._snap = snap
                self._cond.notify_all()

    def _acquire(self) -> None:
        next_status = 0.0
        while not self._stop.is_set():
            try:
                n = self.source.poll(self.buf)
                self.error = ""
            except Exception as e:          # a serial hiccup must not kill the window
                n = 0
                self.error = f"source: {e}"
                self._stop.wait(0.2)
            if self.link:
                try:
                    self.link.poll()
                except Exception as e:      # an unplugged control port must not stop
                    self.error = f"control port: {e}"   # acquisition with it
            now = self.clock()
            if now >= next_status or (n and self.kind == "uart"):
                self._status = self._status_now(now)
                next_status = now + 0.25
                if n and self.kind == "uart":
                    self._wake.set()        # each UART snapshot is news
            if not n and self.kind != "uart":
                self._stop.wait(0.003)      # let the receiver thread fill up

    def _render(self) -> None:
        import traceback
        wait = 0.0
        while not self._stop.is_set():
            self._wake.wait(wait)
            self._wake.clear()
            if self._stop.is_set():
                break
            t0 = time.perf_counter()
            try:
                self.publish()
                self.render_error = ""
            except Exception as e:          # keep the window alive; say what broke
                if not self.render_error:
                    traceback.print_exc()
                self.render_error = f"render: {e}"
            self.render_s = time.perf_counter() - t0
            # Up to `hz` frames a second, but always leave the other threads at
            # least half as long as a frame took (adaptive under load).
            wait = max(self.period - self.render_s, 0.5 * self.render_s)

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        for th in (self.th, self.th_render):
            if th.is_alive():
                th.join(timeout=3.0)


# ---------------------------------------------------------------------------
# Drawing (matplotlib imported here only)
# ---------------------------------------------------------------------------

class Sprites:
    """Short strings as cached coverage bitmaps, blended straight into an RGBA buffer.

    matplotlib lays a string out again every time it draws it (about a
    millisecond each), and a live window redraws a dozen moving time labels per
    frame, nearly all of them strings it drew a frame ago. Here each (text,
    size, family, rotation) is rasterized once, by the same FreeType call Agg
    makes, and then blended in with numpy in a few tens of microseconds. Long
    lines (status, hover, view) are cut into pieces at runs of spaces and " | "
    and laid out on one baseline, so only the pieces that changed cost anything.
    """

    def __init__(self, dpi: float):
        self.dpi = dpi
        self._cache = {}
        self._metrics = {}

    def _font(self, size: float, family: str):
        from matplotlib.font_manager import FontProperties, findfont, get_font
        font = get_font(findfont(FontProperties(family=family, size=size)))
        font.clear()
        font.set_size(size, self.dpi)
        return font

    def bitmap(self, s: str, size: float, family: str = "sans-serif", rotation: int = 0):
        """(coverage (h, w) float32 0..1, pixels of it below the baseline)."""
        key = (s, size, family, rotation)
        hit = self._cache.get(key)
        if hit is None:
            import numpy as np
            from matplotlib.backends.backend_agg import get_hinting_flag
            font = self._font(size, family)
            font.set_text(s, 0, flags=get_hinting_flag())
            font.draw_glyphs_to_bitmap(antialiased=True)
            a = np.asarray(font.get_image(), np.float32) / 255.0
            if rotation == 90:
                a = np.ascontiguousarray(np.rot90(a))
            if len(self._cache) > 4000:
                self._cache.clear()
            hit = self._cache[key] = (a, font.get_descent() / 64.0)
        return hit

    def metrics(self, size: float, family: str):
        """(ascent, width of a space, width of one monospace cell) in pixels."""
        key = (size, family)
        m = self._metrics.get(key)
        if m is None:
            # over 20 characters, so whole-pixel bitmap widths still give sub-pixel advances
            w = {t: self.bitmap(t, size, family)[0].shape[1]
                 for t in ("0", "00", "0" * 21, "0" + " " * 20 + "0")}
            font = self._font(size, family)
            asc = font.ascender / font.units_per_EM * size * self.dpi / 72.0
            m = self._metrics[key] = (asc, (w["0" + " " * 20 + "0"] - w["00"]) / 20,
                                      (w["0" * 21] - w["0"]) / 20)
        return m

    def draw(self, buf, x: float, y: float, s: str, size: float, rgb, ha: str = "left",
             va: str = "center", family: str = "sans-serif", rotation: int = 0, clip=None):
        """Blend `s` into buf (H, W, 4 uint8) at display point (x, y) (y up),
        aligned by ha and va (top, center, bottom, baseline), inside
        clip = (row0, row1, col0, col1)."""
        if not s:
            return 0
        a, desc = self.bitmap(s, size, family, rotation)
        h, w = a.shape
        H, W = buf.shape[:2]
        c0 = int(round(x - {"left": 0.0, "center": w / 2, "right": float(w)}[ha]))
        up = {"top": 0.0, "center": h / 2, "bottom": float(h), "baseline": h - desc}[va]
        r0 = int(round(H - (y + up)))
        lo_r, hi_r, lo_c, hi_c = clip if clip is not None else (0, H, 0, W)
        R0, R1 = max(r0, lo_r, 0), min(r0 + h, hi_r, H)
        C0, C1 = max(c0, lo_c, 0), min(c0 + w, hi_c, W)
        if R1 > R0 and C1 > C0:
            sub = buf[R0:R1, C0:C1, :3]
            al = a[R0 - r0:R1 - r0, C0 - c0:C1 - c0, None]
            sub[...] = (sub * (1.0 - al) + rgb * al + 0.5).astype(buf.dtype)
        return w

    def draw_line(self, buf, x: float, y_top: float, text: str, size: float, rgb,
                  family: str = "sans-serif", clip=None):
        """A long left-aligned line whose top is at y_top, drawn piece by piece."""
        import re
        asc, space, cell = self.metrics(size, family)
        base = y_top - asc
        mono = family == "monospace"
        for tok in re.split(r"( {2,}| \| )", text):
            if not tok:
                continue
            if tok.isspace():
                x += len(tok) * (cell if mono else space)
            elif tok == " | ":
                gap = cell if mono else 2 * space
                self.draw(buf, x + 1.5 * gap, base, "|", size, rgb, "center", "baseline",
                          family=family, clip=clip)
                x += 3 * gap
            else:
                w = self.draw(buf, x, base, tok, size, rgb, va="baseline", family=family,
                              clip=clip)
                x += len(tok) * cell if mono else w + 1


def _rgb(color):
    import numpy as np
    c = color.lstrip("#")
    return np.array([int(c[i:i + 2], 16) for i in (0, 2, 4)], np.float32)


class Scope:
    """All channels as horizontal traces in one panel, scrollable through the history.

    Top to bottom: status line, hover readout, key help; the activity strip
    (each channel's RMS over the visible window, one row per module); the
    trace panel, one lane per channel, channel 0 on top, module boundaries
    ruled and labelled on the left; time labels; the scrollbar over the whole
    history, with the view state under it.

    Keys  <- / ->  pan a quarter window (shift: a whole one)   Home / h  oldest
          End / l  back to live      space  park here / back to live
          [ / ]    shorter / longer window
          up / down, wheel  scroll channels    PageUp / PageDown  a page
          + / -    fewer / more lanes (128, 64, 32, 16, 8)   1-4  one module   0  all
          s  per-channel <-> shared scale    m  activity strip on / off
    Mouse drag or click the scrollbar (wheel over it pans); click a tile to show
          that channel; hover a lane or a tile for its numbers.

    Speed. matplotlib draws the static frame once (axes, the help line, the
    strip's labels and colorbar) and its regions are cached. After that the
    GUI thread draws nothing through matplotlib: the render thread (Acquirer)
    hands it finished uint32 images of the trace panel and the activity strip,
    which are copied into the canvas buffer; labels are cached glyph bitmaps
    (Sprites) blended in with numpy; each region is repainted and blitted only
    when what it shows changed. A key or a drag sends a request to the render
    thread and waits (at most WAIT_S) for its answer, then paints at once
    rather than at the next timer tick.
    """

    FRAME_MS = 15
    WAIT_S = 0.08
    # The header is three lines -- status, hover readout, key help -- placed in
    # points below the window's top edge, so they cannot run into each other or
    # into the plots whatever the window size; _layout() keeps the plots
    # HEAD_PT below the top as the window is resized.
    STATUS_PT, HOVER_PT, HELP_PT = 6, 21, 36    # top of each line
    BAND_PT = 34                                # status + hover: the repainted band
    HEAD_PT = 54                                # top of the plots
    MAP_IN, MAP_GAP_IN = 0.9, 0.26              # activity strip and the space under it
    LEFT_IN, RIGHT_IN = 0.92, 0.72              # margins: channel labels / scale labels
    GAPMARK_PT, TICK_PT = 2.5, 12               # red gap strip, time labels under the panel
    BAR_IN, NAV_PT, PAD_IN = 0.13, 13, 0.06     # scrollbar, view line, bottom margin
    LANE_LABEL_PT = 7.5                         # a lane needs this much height for a scale label
    SCALE_EVERY = 0.5                           # s between updates of the scale labels
    NAV_EVERY = 0.25                            # s between updates of the view line, unprompted

    def __init__(self, acq, kind: str, window_s: float = 2.0, spacing_uv: float = None,
                 history_s: float = 60.0, clock=time.monotonic):
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.cm import ScalarMappable
        from matplotlib.colors import LinearSegmentedColormap, LogNorm
        from matplotlib.transforms import offset_copy
        from . import scroll as S
        self.np, self.plt, self.S = np, plt, S
        self.acq, self.kind, self.clock = acq, kind, clock
        self.n_ch = acq.buf.n_ch
        self.history_s = max(float(history_s), float(window_s))
        self.tv = S.TimeView(float(window_s))
        self.lv = S.LaneView(self.n_ch, 0)
        self.shared_uv = spacing_uv
        self.shared = False
        self.show_map = True
        self.snap = None
        self.sprites = None
        self._seq = 0
        self._seen = None                    # version of the last snapshot taken
        self._geom = None                    # (w, h, gm, map_w, map_h) of the images
        self._rect = None                    # rows/cols of the trace panel in the buffer
        self._map_rect = None
        self._rg = None                      # region -> (bbox, background, clip)
        self._shown = {}                     # region -> what it shows (skip when unchanged)
        self._labels_lv = None
        self._per_lane_scale = False
        self._ticks, self._ylabels, self._modules, self._scales = [], [], [], []
        self._thumb = (0.0, 1.0)
        self._thumb_live = True
        self._nav, self._nav_t = "", -1e9
        self._scale_t = -1e9
        self._status_t = -1e9
        self._status_txt = ""
        self._hover_txt = "hover a lane or a tile for its numbers"
        self._drag = None
        self._wheel_acc = 0.0
        self._hover_ch = None
        self._nav = f"waiting for data ({kind})"
        self._rgb = {k: _rgb(v) for k, v in dict(ink=INK, ink2=INK_2, ink3=INK_3, trace=TRACE,
                                                 track=TRACK).items()}

        plt.rcParams.update({"font.family": "sans-serif", "font.size": 8,
                             "axes.edgecolor": INK_3, "axes.labelcolor": INK_2,
                             "xtick.color": INK_2, "ytick.color": INK_2,
                             "text.color": INK,
                             # matplotlib's own keys that would collide with ours
                             # (s saved the figure, arrows and Home moved the axes)
                             "keymap.save": ["ctrl+s"], "keymap.back": ["c", "backspace"],
                             "keymap.forward": ["v"], "keymap.home": [],
                             "keymap.xscale": [], "keymap.yscale": []})
        fig = self.fig = plt.figure(figsize=(16, 10), facecolor=SURFACE)
        try:
            fig.canvas.manager.set_window_title("intan-chain: 128 channels")
        except Exception:
            pass

        # ---- activity strip: a numpy image, static labels around it ----
        self.amap = fig.add_axes([0.05, 0.8, 0.9, 0.1])
        self.amap.set_facecolor(SURFACE)
        self.amap.set_xlim(-0.5, GRID_COLS - 0.5)
        self.amap.set_ylim(GRID_ROWS - 0.5, -0.5)
        self.amap.set_xticks(range(0, GRID_COLS, 4))
        self.amap.xaxis.tick_top()
        self.amap.tick_params(axis="x", labelsize=6.5, length=0, pad=2)
        self.amap.set_yticks(range(GRID_ROWS))
        self.amap.set_yticklabels([f"{MODULES[m]}  {32 * m}-{32 * m + 31}" for m in range(4)],
                                  fontsize=7, color=INK)
        self.amap.tick_params(axis="y", length=0, pad=4)
        for s in self.amap.spines.values():
            s.set_visible(False)
        cmap = LinearSegmentedColormap.from_list("seq_blue", SEQ_BLUE)
        self.cax = fig.add_axes([0.96, 0.8, 0.008, 0.1])
        cb = fig.colorbar(ScalarMappable(LogNorm(ACT_MIN_UV, ACT_MAX_UV), cmap), cax=self.cax)
        cb.set_ticks([1, 10, 100, 1000, 10000])
        cb.set_ticklabels(["1 µV", "10 µV", "100 µV", "1 mV", "10 mV"])
        cb.ax.tick_params(labelsize=6.5, color=INK_3, length=2)
        cb.outline.set_visible(False)
        self.cax.set_title("RMS", fontsize=7, color=INK_2, pad=3)

        # ---- trace panel and scrollbar: plain axes, the content is painted ----
        self.tax = fig.add_axes([0.05, 0.1, 0.9, 0.6])
        self.sax = fig.add_axes([0.05, 0.03, 0.9, 0.012])
        for ax, face in ((self.tax, SURFACE), (self.sax, TRACK)):
            ax.set_facecolor(face)
            ax.set_xticks([])
            ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)

        def below_top(pt):
            return offset_copy(fig.transFigure, fig=fig, y=-pt, units="points")
        self.help = fig.text(0.01, 1.0, "", fontsize=7.5, va="top", color=INK_2,
                             transform=below_top(self.HELP_PT))
        self.help.set_text(
            "←/→ pan (shift: page)   Home/h oldest   End/l live   space park/live   "
            "[ ] window   ↑/↓ wheel channels   +/- lanes   1-4 module   0 all   "
            "s scale   m strip   ·   drag the bar to scroll   ·   "
            "red rim = ADC rail, grey = no change, red marks under the traces = frames lost")
        self._layout()
        c = fig.canvas
        c.mpl_connect("resize_event", self._layout)
        c.mpl_connect("draw_event", self._on_draw)
        c.mpl_connect("key_press_event", self._key)
        c.mpl_connect("motion_notify_event", self._motion)
        c.mpl_connect("button_press_event", self._press)
        c.mpl_connect("button_release_event", self._release)
        c.mpl_connect("scroll_event", self._wheel)
        c.mpl_connect("close_event", lambda _ev: self._close())

    # ---- layout ----
    def _layout(self, _ev=None):
        """Place everything in inches from the edges, so the header, margins and
        bars keep their size and the trace panel takes whatever is left."""
        W, H = self.fig.get_size_inches()
        left, right = self.LEFT_IN, self.RIGHT_IN
        pw = max(W - left - right, 1.0)
        y = H - self.HEAD_PT / 72
        if self.show_map:
            self.amap.set_position([left / W, (y - self.MAP_IN) / H, pw / W, self.MAP_IN / H])
            self.cax.set_position([(W - right + 0.16) / W, (y - self.MAP_IN) / H, 0.09 / W,
                                   self.MAP_IN / H])
            y -= self.MAP_IN + self.MAP_GAP_IN
        self.amap.set_visible(self.show_map)
        self.cax.set_visible(self.show_map)
        bar_y = self.PAD_IN + self.NAV_PT / 72 + 0.03
        tax_y = bar_y + self.BAR_IN + 0.06 + (self.GAPMARK_PT + self.TICK_PT) / 72
        self.tax.set_position([left / W, tax_y / H, pw / W, max(y - tax_y, 0.5) / H])
        self.sax.set_position([left / W, bar_y / H, pw / W, self.BAR_IN / H])
        self.help.set_x(left / W * 0.25)

    def _pt(self, pt: float) -> float:
        return pt * self.fig.dpi / 72.0

    def _geometry(self):
        """Pixel rectangles of the two images in the canvas buffer, from the axes."""
        H = int(self.fig.bbox.height)
        tb = self.tax.bbox
        X0, X1, Y0, Y1 = int(tb.x0), int(tb.x1), int(tb.y0), int(tb.y1)
        gm = max(2, int(round(self._pt(self.GAPMARK_PT))))
        self._rect = (H - Y1, H - Y0 + gm, X0, X1)
        mw = mh = 0
        self._map_rect = None
        if self.show_map:
            mb = self.amap.bbox
            mx0, mx1, my0, my1 = int(mb.x0), int(mb.x1), int(mb.y0), int(mb.y1)
            self._map_rect = (H - my1, H - my0, mx0, mx1)
            mw, mh = mx1 - mx0, my1 - my0
        return (X1 - X0, Y1 - Y0, gm, mw, mh)

    def _regions(self):
        """name -> display bbox of every repainted region (disjoint)."""
        from matplotlib.transforms import Bbox
        fb = self.fig.bbox
        H = int(fb.height)
        r0, r1, X0, X1 = self._rect
        Y1, Y0g = H - r0, H - r1                       # panel top, bottom of the gap strip
        Y0 = Y0g + self._geom[2]
        tick_px = int(round(self._pt(self.TICK_PT)))
        bar_top = int(self.sax.bbox.y1) + 2
        band = int(round(self._pt(self.BAND_PT)))
        out = {
            "lanes": (X0, Y0g, X1, Y1),
            "xaxis": (max(0, X0 - 40), Y0g - tick_px, min(int(fb.x1), X1 + 40), Y0g),
            "ylab": (0, Y0, X0, Y1),
            "scale": (X1, Y0, int(fb.x1), Y1),
            "scroll": (0, 0, int(fb.x1), bar_top),
            "status": (0, H - band, int(fb.x1), H),
        }
        if self._map_rect is not None:
            m0, m1, mx0, mx1 = self._map_rect
            out["map"] = (mx0, H - m1, mx1, H - m0)
        return {k: Bbox.from_extents(*v) for k, v in out.items()}

    def _on_draw(self, _ev):
        """A full redraw (first show, resize, m) drew only the static frame:
        cache each region's clean background, then paint all of them back on."""
        c = self.fig.canvas
        if c.is_saving():                    # savefig draws at its own dpi: leave it alone
            return
        if self.sprites is None or self.sprites.dpi != self.fig.dpi:
            self.sprites = Sprites(self.fig.dpi)
        geom = self._geometry()
        resized = geom != self._geom
        self._geom = geom
        H = int(self.fig.bbox.height)
        self._rg = {}
        for k, bb in self._regions().items():
            clip = (H - int(bb.y1), H - int(bb.y0), int(bb.x0), int(bb.x1))
            self._rg[k] = (bb, c.copy_from_bbox(bb), clip)
        self._shown = {}
        self._labels_lv = None
        if resized:
            self._send()                     # images of the new size, on their way
        if self.snap is not None:
            self._take(self.snap, force=True)
        else:
            for k in ("status", "scroll"):
                self._paint(k)

    # ---- talking to the render thread ----
    def _send(self, wait: bool = False):
        """Ask for the current view; with wait, paint its answer before returning."""
        if self._geom is None:
            return
        w, h, gm, mw, mh = self._geom
        self._seq += 1
        max_ticks = max(2, min(14, int(w / self._pt(70))))
        self.acq.set_request(Request(self._seq, self.tv, self.lv, self.shared, self.shared_uv,
                                     w, h, gm, mw, mh, max_ticks))
        self._nav_t = -1e9                   # the view line follows at once
        if wait:
            snap = self.acq.wait_for(self._seq, self.WAIT_S)
            if snap is not None:
                self._take(snap)

    # ---- painting: numpy into the canvas buffer, then blit ----
    def _paint(self, name):
        if self._rg is None or name not in self._rg:
            return
        bbox, bg, clip = self._rg[name]
        c = self.fig.canvas
        c.restore_region(bg)
        buf = self.np.asarray(c.buffer_rgba())
        getattr(self, "_draw_" + name)(buf, clip)
        c.blit(bbox)

    def _copy(self, buf, rect, img):
        np = self.np
        if rect is None or img is None:
            return
        r0, r1, c0, c1 = rect
        dst = buf.view(np.uint32)[..., 0][r0:r1, c0:c1]
        hh, ww = min(dst.shape[0], img.shape[0]), min(dst.shape[1], img.shape[1])
        dst[:hh, :ww] = img[:hh, :ww]

    def _draw_lanes(self, buf, _clip):
        s = self.snap
        if s is not None and s.raster is not None and s.geom == self._geom[:3]:
            self._copy(buf, self._rect, s.raster)

    def _draw_map(self, buf, _clip):
        s = self.snap
        if s is not None and s.map_raster is not None and self._map_rect is not None:
            r0, r1, c0, c1 = self._map_rect
            if s.map_raster.shape == (r1 - r0, c1 - c0):
                self._copy(buf, self._map_rect, s.map_raster)

    def _draw_xaxis(self, buf, clip):
        H = buf.shape[0]
        y = H - self._rect[1] - self._pt(1.5)
        for x, lab in self._ticks:
            self.sprites.draw(buf, x, y, lab, 7, self._rgb["ink2"], "center", "top", clip=clip)

    def _draw_ylab(self, buf, clip):
        sp, ink2, ink, ink3 = self.sprites, self._rgb["ink2"], self._rgb["ink"], self._rgb["ink3"]
        x_lab = self._rect[2] - self._pt(4)
        for y, lab, fs in self._ylabels:
            sp.draw(buf, x_lab, y, lab, fs, ink2, "right", "center", clip=clip)
        H = buf.shape[0]
        xb = int(round(self._rect[2] - self._pt(24)))
        lw = max(1, int(round(self._pt(0.8))))
        for y, lab, ytop, ybot in self._modules:
            sp.draw(buf, self._rect[2] - self._pt(34), y, lab, 8, ink, "center", "center",
                    rotation=90, clip=clip)
            r0, r1 = int(round(H - ytop)), int(round(H - ybot))
            buf[max(r0, clip[0]):min(r1, clip[1]), xb:xb + lw, :3] = ink3

    def _draw_scale(self, buf, clip):
        x = self._rect[3] + self._pt(4)
        for y, lab in self._scales:
            self.sprites.draw(buf, x, y, lab, 6, self._rgb["ink3"], "left", "center", clip=clip)

    def _draw_scroll(self, buf, clip):
        H = buf.shape[0]
        sb = self.sax.bbox
        a, b = self._thumb
        c0 = int(round(sb.x0 + a * sb.width))
        c1 = max(c0 + 3, int(round(sb.x0 + b * sb.width)))
        inset = max(1, int(round(sb.height * 0.12)))
        r0, r1 = int(round(H - sb.y1)) + inset, int(round(H - sb.y0)) - inset
        buf[r0:r1, c0:c1, :3] = self._rgb["trace" if self._thumb_live else "ink2"]
        self.sprites.draw_line(buf, sb.x0, self._pt(self.PAD_IN * 72 + self.NAV_PT - 2), self._nav,
                               8, self._rgb["ink"], clip=clip)

    def _draw_status(self, buf, clip):
        H, x = buf.shape[0], self.fig.bbox.width * self.LEFT_IN / self.fig.get_figwidth() * 0.25
        self.sprites.draw_line(buf, x, H - self._pt(self.STATUS_PT), self._status_txt, 9,
                               self._rgb["ink"], family="monospace", clip=clip)
        self.sprites.draw_line(buf, x, H - self._pt(self.HOVER_PT), self._hover_txt, 9,
                               self._rgb["ink2"], clip=clip)

    # ---- snapshot -> what each region shows ----
    def _take(self, snap, force: bool = False):
        """Show a snapshot: repaint exactly the regions whose content changed."""
        S, shown, now = self.S, self._shown, self.clock()
        self.snap, self._seen = snap, snap.version
        if self._rg is None:
            return
        if snap.raster is not None and (force or shown.get("lanes") is not snap.raster):
            self._paint("lanes")
            shown["lanes"] = snap.raster
        if self._map_rect is not None and (force or shown.get("map") is not snap.map_raster):
            self._paint("map")
            shown["map"] = snap.map_raster
        key = (snap.t0, snap.t1, snap.tick_step)
        if force or shown.get("ticks") != key:
            X0, X1 = self._rect[2], self._rect[3]
            span = max(snap.t1 - snap.t0, 1e-12)
            self._ticks = [(X0 + (t - snap.t0) / span * (X1 - X0), lab) for t, lab in snap.ticks]
            self._paint("xaxis")
            shown["ticks"] = key
        if force or self._labels_lv != snap.lv:
            self._apply_lanes(snap.lv)
            self._paint("ylab")
            self._scale_t = -1e9
        if self._scale_labels(snap, now) or force:
            self._paint("scale")
        a, b = S.thumb(snap.t0, snap.t1, snap.oldest, snap.newest)
        a, b = max(0.0, min(a, 1.0)), max(0.0, min(b, 1.0))
        if now - self._nav_t >= self.NAV_EVERY or force:
            self._nav, self._nav_t = self._nav_line(snap), now
        state = (round(a, 4), round(b, 4), snap.live, self._nav)
        if force or shown.get("scroll") != state:
            self._thumb, self._thumb_live = (a, b), snap.live
            self._paint("scroll")
            shown["scroll"] = state
        if force or (now - self._status_t >= 0.25 and self._status_txt != snap.status):
            self._status_t, self._status_txt = now, snap.status
            self._paint("status")

    def _lane_y(self, lv):
        """Display y of each visible lane's centre, the way the raster places it."""
        H = int(self.fig.bbox.height)
        r = self.S.lane_rows(self._geom[1], lv.n).astype(float)
        return H - (self._rect[0] + (r[:-1] + r[1:]) / 2)

    def _apply_lanes(self, lv):
        """Channel numbers, module labels and brackets for the lanes `lv`."""
        self._labels_lv = lv
        n, H = lv.n, int(self.fig.bbox.height)
        ys = self._lane_y(lv)
        lane_pt = self._geom[1] / n * 72.0 / self.fig.dpi
        step = self.S.label_step(lane_pt)
        fs = 6.5 if lane_pt < 11 else 7.5
        self._ylabels = [(ys[i], str(lv.top + i), fs) for i in range(n)
                         if (lv.top + i) % step == 0]
        r = self.S.lane_rows(self._geom[1], n)
        self._modules = []
        for m in range(self.n_ch // 32):
            a, b = max(32 * m, lv.top), min(32 * m + 31, lv.top + n - 1)
            if a <= b:
                ytop = H - (self._rect[0] + r[a - lv.top]) - 2
                ybot = H - (self._rect[0] + r[b - lv.top + 1]) + 2
                self._modules.append(((ytop + ybot) / 2, MODULES[m], ytop, ybot))
        self._per_lane_scale = lane_pt >= self.LANE_LABEL_PT

    def _scale_labels(self, snap, now) -> bool:
        """Per-lane '±X' labels when every lane has room and scales differ; True if changed."""
        if not (self._per_lane_scale and not snap.shared):
            changed = bool(self._scales)
            self._scales = []
            return changed
        if now - self._scale_t < self.SCALE_EVERY:
            return False
        self._scale_t = now
        lv, ys = snap.lv, self._lane_y(snap.lv)
        full = snap.half / LANE_FILL * LANE_CLIP
        new = [(ys[i], f"±{fmt_uv(full[lv.top + i])}") for i in range(lv.n)]
        changed = new != self._scales
        self._scales = new
        return changed

    def _nav_line(self, s) -> str:
        lv = s.lv
        if s.live:
            where = "● LIVE"
        else:
            where = (f"■ {s.newest - s.right:.1f} s behind live (End or l: live)    "
                     f"t {s.t0:.2f} – {s.t1:.2f} s")
        if s.shared:
            scale = f"shared scale, full lane ±{fmt_uv(float(s.half[0]) / LANE_FILL * LANE_CLIP)}"
        elif self._per_lane_scale:
            scale = "each lane scaled to its channel (range at right)"
        else:
            scale = "each lane scaled to its channel (hover for its range)"
        return "    ".join([where, f"window {fmt_width(s.width)}",
                            f"history {s.newest - s.oldest:.0f} of {self.history_s:g} s",
                            f"ch {lv.top}–{lv.top + lv.n - 1} ({lv.n} lanes)", scale])

    # ---- the frame clock ----
    def _tick(self):
        if self._rg is None:
            return
        snap = self.acq.latest()
        if snap is not None and snap.version != self._seen and not self.acq.render_error:
            self._take(snap)
        elif snap is None or self.acq.render_error:
            # Nothing yet, or rendering is failing and no new snapshot will come:
            # still say what the link (or the renderer) is doing.
            now, st = self.clock(), self.acq.status()
            if st != self._status_txt and now - self._status_t >= 0.25:
                self._status_t, self._status_txt = now, st
                self._paint("status")

    # ---- interaction ----
    def _extent(self):
        s = self.snap
        return (s.oldest, s.newest) if s is not None else (0.0, 0.0)

    def _set_view(self, tv, wait: bool = True):
        if tv != self.tv:
            self.tv = tv
            self._send(wait=wait)

    def _set_lanes(self, lv, wait: bool = True):
        if lv != self.lv:
            self.lv = lv
            self._send(wait=wait)

    def _key(self, ev):
        S, k = self.S, ev.key
        old, new = self._extent()
        if k in ("left", "right", "shift+left", "shift+right"):
            frac = (1.0 if k.startswith("shift") else 0.25) * (-1 if k.endswith("left") else 1)
            self._set_view(S.pan(self.tv, frac, old, new))
        elif k in ("home", "h"):
            self._set_view(S.go_oldest(self.tv, old, new))
        elif k in ("end", "l"):
            self._set_view(S.go_live(self.tv))
        elif k == " ":
            self._set_view(S.go_live(self.tv) if not self.tv.live else S.freeze(self.tv, old, new))
        elif k in ("[", "]"):
            self._set_view(S.zoom_time(self.tv, 1 if k == "]" else -1, old, new, self.history_s))
        elif k in ("up", "down", "pageup", "pagedown"):
            d = max(1, self.lv.n // 4) if k in ("up", "down") else self.lv.n
            self._set_lanes(S.lane_scroll(self.lv, -d if k in ("up", "pageup") else d, self.n_ch))
        elif k in ("+", "=", "-", "_"):
            self._set_lanes(S.lane_zoom(self.lv, 1 if k in ("+", "=") else -1, self._hover_ch,
                                        self.n_ch))
        elif k == "0":
            self._set_lanes(S.lanes(self.n_ch, 0, self.n_ch))
        elif k in ("1", "2", "3", "4") and 32 * int(k) <= self.n_ch:
            self._set_lanes(S.lanes(32, 32 * (int(k) - 1), self.n_ch))
        elif k == "s":
            self.shared = not self.shared
            self._send(wait=True)
        elif k == "m":
            self.show_map = not self.show_map
            self._layout()
            self.fig.canvas.draw_idle()      # the layout changed: one full redraw
        elif k == "r":
            link = getattr(self.acq, "link", None)
            if link is not None and hasattr(link, "request_reconfigure"):
                link.request_reconfigure()   # the watcher thread does it, not the GUI

    def _wheel(self, ev):
        # A trackpad sends a stream of small steps: add them up, and do not wait
        # for each one's frame (the timer paints the newest), or they queue up.
        S = self.S
        if ev.inaxes is self.tax:
            self._wheel_acc += ev.step
            steps = int(self._wheel_acc)
            self._wheel_acc -= steps
            if steps:
                self._set_lanes(S.lane_scroll(self.lv, -steps * max(1, self.lv.n // 8),
                                              self.n_ch), wait=False)
        elif ev.inaxes is self.sax:
            old, new = self._extent()
            self._set_view(S.pan(self.tv, 0.1 * ev.step, old, new), wait=False)

    def _bar_frac(self, ev) -> float:
        b = self.sax.bbox
        return (ev.x - b.x0) / max(b.width, 1.0)

    def _press(self, ev):
        if ev.button != 1 or self.snap is None:
            return
        if ev.inaxes is self.sax:
            f = self._bar_frac(ev)
            a, b = self._thumb
            self._drag = f - a if a <= f <= b else (b - a) / 2
            self._drag_to(f)
        elif ev.inaxes is self.amap:
            ch = self._channel_at(ev)
            lv = self.lv
            if ch is not None and not lv.top <= ch < lv.top + lv.n:
                self._set_lanes(self.S.lanes(lv.n, (ch // lv.n) * lv.n, self.n_ch))

    def _drag_to(self, f):
        """The thumb follows the mouse at once; the traces follow at the next frame."""
        S, (old, new) = self.S, self._extent()
        w = self.tv.width
        r = S.right_for_thumb(f - self._drag, w, old, new)
        self._set_view(S.TimeView(w, r >= new, r), wait=False)
        a, b = S.thumb(r - w, r, old, new)
        self._thumb, self._thumb_live = (max(0.0, a), min(1.0, b)), r >= new
        self._paint("scroll")

    def _release(self, _ev):
        self._drag = None

    def _channel_at(self, ev):
        if ev.x is None or self._geom is None:
            return None
        if self.show_map and ev.inaxes is self.amap:
            b = self.amap.bbox
            r = int((b.y1 - ev.y) / b.height * GRID_ROWS)
            c = int((ev.x - b.x0) / b.width * GRID_COLS)
            if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
                return grid_channel(r, c)
        if ev.inaxes is self.tax and self.snap is not None:
            lv = self.snap.lv
            row = int(int(self.fig.bbox.height) - ev.y) - self._rect[0]
            lane = int(self.np.searchsorted(self.S.lane_rows(self._geom[1], lv.n), row,
                                            "right")) - 1
            if 0 <= lane < lv.n:
                return lv.top + lane
        return None

    def _motion(self, ev):
        if self._drag is not None and ev.x is not None:
            self._drag_to(self._bar_frac(ev))
            return
        ch = self._channel_at(ev)
        self._hover_ch = ch
        if ch is None or self.snap is None:
            return
        s = self.snap
        st = s.stat(ch)
        tag = {"railed": "  RAIL", "dead": "  DEAD"}.get(st.flag, "")
        where = ""
        if ev.inaxes is self.tax:
            b = self.tax.bbox
            where = f"   at t {s.t0 + (ev.x - b.x0) / b.width * (s.t1 - s.t0):.3f} s"
        self._hover_txt = (
            f"ch {ch} ({MODULES[ch // 32]} #{ch % 32}):  rms {fmt_uv(st.rms_uv)}"
            f"   mean {fmt_uv(st.mean_uv)}   min {fmt_uv(st.min_uv)}   max {fmt_uv(st.max_uv)}"
            f"   lane ±{fmt_uv(float(s.half[ch]) / LANE_FILL * LANE_CLIP)}"
            f"   over {s.t0:.2f}–{s.t1:.2f} s, {s.n} frames{where}{tag}")
        self._paint("status")

    def _close(self):
        if getattr(self, "timer", None) is not None:
            self.timer.stop()
        self.acq.stop()

    # ---- running ----
    def sync(self):
        """Lay out, render the current view on this thread and paint it (tests,
        screenshots: no timer, no render thread needed)."""
        self.fig.canvas.draw()
        self.acq.publish()
        snap = self.acq.latest()
        if snap is not None:
            self._take(snap, force=True)
        return snap

    def render(self, snap):
        """Show one snapshot synchronously (tests, screenshots)."""
        self._take(snap, force=True)

    def save_png(self, path: str):
        """The canvas exactly as on screen, images and labels included."""
        np = self.np
        self.fig.canvas.draw()               # full redraw; _on_draw repaints every region
        self.plt.imsave(path, np.asarray(self.fig.canvas.buffer_rgba()))

    def show(self):
        self.timer = self.fig.canvas.new_timer(interval=self.FRAME_MS)
        self.timer.add_callback(self._tick)
        self.timer.start()
        self.plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="python3 -m chain.viewer",
                                 description="live view of the 128 RHD2164 channels")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--udp", action="store_true", help="read datagrams on --port (UDP)")
    src.add_argument("--tcp", action="store_true",
                     help="accept the node's TCP stream on --port (node built with "
                          "CONFIG_CHAIN_SINK_TCP)")
    src.add_argument("--uart", action="store_true",
                     help="diagnostic CONVERT snapshots over --serial (no ESP)")
    ap.add_argument("--serial", help="FPGA control UART")
    ap.add_argument("--baud", type=int, default=C.UART_BAUD)
    ap.add_argument("--port", "--udp-port", dest="udp_port", type=int, default=C.UDP_PORT,
                    help="data port, UDP or TCP (default %(default)s)")
    ap.add_argument("--start", action="store_true",
                    help="with --udp/--tcp --serial: configure the chips (bring-up S0, S2-S5) and "
                         "set run; cleared again on exit unless --leave-running")
    ap.add_argument("--no-config", action="store_true",
                    help="with --uart: skip configuring (the chips were just brought up)")
    ap.add_argument("--leave-running", action="store_true")
    ap.add_argument("--no-recover", action="store_true",
                    help="with --serial: do not watch the link or put the chain back together "
                         "after an unplug (default: reconfigure the chips when a sleeve comes "
                         "back, reopen a lost port, reload a blank FPGA)")
    ap.add_argument("--bitstream", default=None, metavar="PATH",
                    help="bitstream to reload if the FPGA loses power (default: "
                         "fpga/bitstreams/headstage_arty_s7_25_div6.bit)")
    ap.add_argument("--no-reload", action="store_true",
                    help="never run openFPGALoader; just say the FPGA needs reloading")
    ap.add_argument("--spi-div", type=int, default=0)
    ap.add_argument("--phase", type=B.phase_arg, default=None, metavar="N|auto",
                    help="PHASE_SELECT for --start/--uart, or auto (default)")
    ap.add_argument("--rate", type=float, default=2000.0)
    ap.add_argument("--lower", type=float, default=1.0)
    ap.add_argument("--upper", type=float, default=500.0)
    ap.add_argument("--window", type=float, default=2.0,
                    help="seconds shown across the panel (keys [ and ] change it)")
    ap.add_argument("--history", type=float, default=60.0, metavar="SECONDS",
                    help="seconds kept to scroll back through (default %(default)s; "
                         "about 0.5 MB per second at 2 kS/s)")
    ap.add_argument("--spacing", type=float, default=None, metavar="UV",
                    help="half-height of a lane in the shared-scale view (key s); "
                         "default: the 99th percentile of all live channels")
    ap.add_argument("--headless", type=float, default=None, metavar="N",
                    help="no display: run N seconds, print status and per-channel stats")
    return ap


def main(argv=None, out=print) -> int:
    ap = build_parser()
    a = ap.parse_args(argv)
    if a.uart and not a.serial:
        ap.error("--uart needs --serial PORT")
    if a.start and not ((a.udp or a.tcp) and a.serial):
        ap.error("--start needs --udp or --tcp, and --serial")
    if a.window <= 0 or a.history <= 0 or a.rate <= 0:
        ap.error("--window, --history and --rate must be positive")
    from .diag import Diag
    opts = B.Options(serial=a.serial, baud=a.baud, spi_div=a.spi_div,
                     phase=a.phase, rate=a.rate, lower=a.lower, upper=a.upper)
    ctrl = Ctrl(a.serial, baud=a.baud) if a.serial else None
    started = False
    source = link = None
    try:
        if ctrl is not None and (a.uart and not a.no_config or a.start):
            b, rep = B.prepare(ctrl, opts, out=out, stream=a.start)
            if rep["result"] == "FAIL":
                out("bring-up reported failures (above); showing what can be shown")
            started = a.start and bool(ctrl.read("CTRL") & 1)
        if a.udp or a.tcp:
            source = TcpSource(a.udp_port) if a.tcp else UdpSource(a.udp_port)
            if ctrl is None:
                link = None
            elif a.no_recover:
                link = LinkPoller(ctrl)
            else:
                from . import recover as RC
                program = None if a.no_reload else RC.openfpgaloader(
                    a.bitstream or RC.DEFAULT_BITSTREAM)
                link = RC.LinkWatch(
                    ctrl, opts, program=program, out=out,
                    open_ctrl=lambda: Ctrl(a.serial, baud=a.baud)).start()
            depth = int(a.window * a.rate * 1.5) + 10
        else:
            if ctrl.read("CTRL") & 1:
                out("run is set: stopping it, the diagnostic list needs it clear")
                ctrl.set_run(False)
            source = UartSource(Diag(ctrl))
            depth = 2000
        buf = Buffers(depth=depth, rate=a.rate)
        kind = "udp" if a.udp else "tcp" if a.tcp else "uart"
        if a.headless is not None:
            run_headless(source, buf, a.headless, out=out, link=link, kind=kind)
        else:
            win = a.window if kind != "uart" else max(a.window, 20)
            hist = max(a.history, win)
            acq = Acquirer(source, buf, kind, link, window_s=win, history_s=hist,
                           rate=a.rate).start()
            try:
                Scope(acq, kind, window_s=win, spacing_uv=a.spacing, history_s=hist).show()
            finally:
                acq.stop()          # before the port closes below
    finally:
        if source is not None:
            source.close()
        if link is not None and hasattr(link, "stop"):
            link.stop()             # before anything else touches the port
            ctrl = link.ctrl        # the watcher may have reopened it (or lost it)
        if ctrl is not None:
            if started and not a.leave_running:
                try:
                    ctrl.set_run(False)
                except (CtrlError, OSError):
                    pass
            try:
                ctrl.close()
            except OSError:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
