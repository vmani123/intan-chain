"""Live view of all 128 channels.

    python3 -m chain.viewer --udp                          # board already streaming via the ESP
    python3 -m chain.viewer --udp --serial PORT --start    # configure (bring-up S0, S2-S5), run
    python3 -m chain.viewer --uart --serial PORT           # no ESP: diagnostic CONVERT snapshots
    python3 -m chain.viewer --udp --headless 10            # no display: per-channel stats

Two sources feed one set of per-channel buffers:

  --udp   datagrams on UDP_PORT, parsed with chain.frame. Time comes from each
          frame's t_cap (the FPGA's own clock), so the x axis is exact and gaps
          in seq are counted, never papered over. With --serial the status line
          also shows the FPGA's LINK_ERR counters.
  --uart  repeated diagnostic-list CONVERT sweeps over the control UART. Every
          snapshot costs ~100 UART round trips, so this runs at a few Hz at best
          (less where the USB serial driver adds latency per read): enough to
          see which channels are alive without an ESP, not to see EMG.

Everything up to the drawing (buffers, loss, statistics, the status line) is
plain Python and tested; matplotlib and numpy are imported only to draw.
"""
import argparse
import array
import collections
import itertools
import socket
import threading
import time
from dataclasses import dataclass

from . import bringup as B
from . import contract as C
from . import frame as F
from . import rhd2164 as R
from .ctrl import Ctrl, CtrlError

U32 = 0xFFFFFFFF
RAIL_LSB = 32700
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
    """

    def __init__(self, n_ch: int = C.N_CH, depth: int = 4000):
        self.n_ch = n_ch
        self.depth = depth
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

    # --- input --------------------------------------------------------------
    def push(self, seq: int, samples, t: float, host_t: float = None) -> bool:
        """One frame. False if it was a duplicate or arrived out of order (not stored)."""
        if seq in self._seen:
            self.duplicates += 1
            return False
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
        else:
            self.first_seq = seq
        self._remember(seq)
        self.last_seq = seq
        row = array.array("h", samples[:self.n_ch])
        if len(row) < self.n_ch:
            row.extend([0] * (self.n_ch - len(row)))
        self.rows.append(row)
        self.t.append(t)
        self.frames += 1
        self.arrivals.append((t if host_t is None else host_t, 1))
        return True

    def push_frame(self, fr: F.Frame, host_t: float) -> bool:
        """A parsed frame; time from t_cap, unwrapped across the 44.7 s counter wrap."""
        if self._tcap_last is not None:
            d = (fr.t_cap - self._tcap_last) & U32
            self._tcap_s += (d if d < 1 << 31 else d - (1 << 32)) / C.SYS_CLK_HZ
        self._tcap_last = fr.t_cap
        return self.push(fr.seq, fr.samples(), self._tcap_s, host_t)

    def push_datagram(self, data: bytes, host_t: float) -> int:
        """Frames stored from one datagram; a malformed datagram is counted, not raised."""
        try:
            frames, _trailer = F.parse_datagram(data)
        except F.FrameError:
            self.malformed += 1
            return 0
        return sum(self.push_frame(fr, host_t) for fr in frames)

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
    unit = "fr/s" if source == "udp" else "snap/s"
    parts = [f"{source.upper()} {buf.frame_rate(now):7.1f} {unit}",
             f"frames {buf.frames}",
             f"seq gaps {buf.gaps} ({buf.lost} lost, {buf.loss_pct():.2f}%)"]
    if buf.duplicates or buf.reordered:
        parts.append(f"dup {buf.duplicates} reord {buf.reordered}")
    if source == "udp":
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
            out(status_line(buf, now, kind, lk, link.error if link else ""))
            next_print = now + every
        if not got and kind == "udp":
            sleep(0.02)
    lk = link.poll() if link else None
    st = buf.stats()
    out(status_line(buf, clock(), kind, lk))
    out(stats_table(st))
    out(flagged_summary(st))
    return st


# ---------------------------------------------------------------------------
# What the display shows, as plain functions (tested without a display)
# ---------------------------------------------------------------------------
# The activity map is 8 rows x 16 columns: module m (chip0 A, chip0 B, chip1 A,
# chip1 B) fills rows 2m and 2m+1, channels in order left to right.
GRID_ROWS, GRID_COLS = 8, 16
MIN_HALF_UV = 2.0        # a lane never magnifies below ±2 µV: ADC noise stays small
LANE_FILL = 0.45         # a channel's 99th-percentile excursion fills this much of a lane
LANE_CLIP = 0.48         # and nothing ever leaves it
MAX_BINS = 300           # min/max columns per lane: about a panel's width in pixels

# Colours (light surface). One sequential hue for magnitude, one series colour
# for the traces, reserved status colours that always come with a text tag.
SURFACE = "#fcfcfb"
INK, INK_2, INK_3 = "#0b0b0b", "#52514e", "#8c8b86"
TRACE = "#2a78d6"
CRITICAL = "#d03b3b"     # railed
DEAD_FILL = "#e4e3df"
SEQ_BLUE = ("#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5",
            "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b")
ACT_MIN_UV, ACT_MAX_UV = 1.0, 10_000.0   # activity colour scale, log


def grid_pos(ch: int) -> tuple[int, int]:
    """(row, column) of a channel's tile in the activity map."""
    m, i = divmod(ch, 32)
    return 2 * m + i // GRID_COLS, i % GRID_COLS


def grid_channel(row: int, col: int) -> int:
    """The channel shown at (row, column); the inverse of grid_pos."""
    m, r = divmod(row, 2)
    return 32 * m + GRID_COLS * r + col


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
    dev = uv - centre
    if shared_half is None:
        half = np.maximum(np.percentile(np.abs(dev), 99, axis=0), MIN_HALF_UV)
    else:
        half = np.full(uv.shape[1], max(float(shared_half), MIN_HALF_UV))
    y = np.clip(LANE_FILL * dev / half, -LANE_CLIP, LANE_CLIP)
    return y, half / LANE_FILL * LANE_CLIP


def minmax_envelope(t, uv, max_bins: int = MAX_BINS):
    """At most 2 * max_bins points per channel, keeping every bin's min and max.

    A panel is a few hundred pixels wide; 4000 samples a lane only cost time to
    draw. Keeping each bin's extremes, in the order they happened, means a
    spike narrower than a pixel still reaches its full height. Short windows
    (UART snapshots) come back untouched. The oldest few samples that do not
    fill a bin are dropped, never the newest.
    """
    import numpy as np
    n = len(t)
    if n <= 2 * max_bins:
        return t, uv
    bs = n // max_bins
    start = n - bs * max_bins
    blk = uv[start:].reshape(max_bins, bs, uv.shape[1])
    i_lo, i_hi = blk.argmin(axis=1), blk.argmax(axis=1)
    lo = np.take_along_axis(blk, i_lo[:, None, :], 1)[:, 0, :]
    hi = np.take_along_axis(blk, i_hi[:, None, :], 1)[:, 0, :]
    lo_first = i_lo <= i_hi
    y = np.empty((2 * max_bins, uv.shape[1]), uv.dtype)
    y[0::2] = np.where(lo_first, lo, hi)
    y[1::2] = np.where(lo_first, hi, lo)
    tb = t[start:].reshape(max_bins, bs)
    x = np.empty(2 * max_bins, float)
    x[0::2], x[1::2] = tb[:, 0], tb[:, -1]
    return x, y


def pack_rgba(colors) -> "np.ndarray":
    """Matplotlib colours -> uint32 RGBA in the canvas buffer's byte order (never 0)."""
    import numpy as np
    from matplotlib.colors import to_rgba_array
    rgba = (to_rgba_array(colors) * 255).round().astype(np.uint8)
    rgba[:, 3] = 255
    return rgba.view(np.uint32)[:, 0].copy()


def lane_blocks(y, lane_px: int, colors, width: int):
    """Rasterize lanes: (n_lanes, lane_px, width) uint32, 0 where nothing is drawn.

    y is a min/max envelope in lane units, shape (2 * bins, n_lanes), pairs in
    time order (minmax_envelope then lane_values). Each pixel column gets a
    vertical span from its min to its max, stretched to meet the previous
    column's last value so a fast edge stays connected. This replaces drawing
    128 polylines: a handful of vectorised numpy operations, the same cost at
    any sample rate. colors is (n_lanes,) uint32 from pack_rgba.
    """
    import numpy as np
    a, b = y[0::2].T, y[1::2].T                       # (lanes, bins): first, last of each bin
    lo, hi = np.minimum(a, b), np.maximum(a, b)
    prev = np.concatenate([b[:, :1], b[:, :-1]], axis=1)
    lo, hi = np.minimum(lo, prev), np.maximum(hi, prev)
    bins = lo.shape[1]
    if width != bins:                                 # bins -> pixel columns
        idx = (np.arange(width) * bins) // width
        lo, hi = lo[:, idx], hi[:, idx]
    top = np.clip(((0.5 - hi) * lane_px).astype(np.int32), 0, lane_px - 1)
    bot = np.clip(((0.5 - lo) * lane_px).astype(np.int32), 0, lane_px - 1)
    r = np.arange(lane_px, dtype=np.int32)[None, :, None]
    mask = (r >= top[:, None, :]) & (r <= bot[:, None, :])
    return np.where(mask, np.asarray(colors, np.uint32)[:, None, None], np.uint32(0))


def compose_lanes(blocks, height: int, centers):
    """Stack lane blocks into one (height, width) uint32 image, each centred on its row."""
    import numpy as np
    n, px, w = blocks.shape
    out = np.zeros((height, w), np.uint32)
    for i in range(n):
        r0 = int(round(centers[i] - px / 2))
        a, b = max(r0, 0), min(r0 + px, height)
        if b > a:
            seg = blocks[i, a - r0:b - r0]
            np.copyto(out[a:b], seg, where=seg != 0)
    return out


@dataclass
class Snapshot:
    """One ready-to-draw window: everything the display needs, nothing it must compute
    from raw samples. Built on the acquisition thread; read-only afterwards."""
    version: int
    x: object          # (m,) seconds relative to the newest sample (<= 0)
    env: object        # (m, n_ch) µV after min/max decimation
    n: int             # samples in the window
    mean: object       # (n_ch,) µV
    rms: object        # (n_ch,) µV about the mean
    lo: object         # (n_ch,) µV
    hi: object         # (n_ch,) µV
    dead: object       # (n_ch,) bool: no change at all
    railed: object     # (n_ch,) bool: touched the ADC rail
    status: str

    def stat(self, ch: int) -> ChannelStat:
        return ChannelStat(ch, self.n, float(self.mean[ch]), float(self.rms[ch]),
                           float(self.lo[ch]), float(self.hi[ch]), bool(self.dead[ch]),
                           bool(self.railed[ch]))


def make_snapshot(buf: Buffers, window_s: float, status: str, version: int,
                  max_bins: int = MAX_BINS):
    """The newest window of `buf` as a Snapshot, or None if nothing arrived yet."""
    import numpy as np
    t, data = buf.window(window_s)
    if not len(t):
        return None
    lo_c, hi_c = data.min(axis=0), data.max(axis=0)
    dead = (lo_c == hi_c) & (len(data) >= 2)
    railed = np.maximum(np.abs(lo_c.astype(np.int32)), np.abs(hi_c.astype(np.int32))) >= RAIL_LSB
    scale = np.float32(R.SCALE_UV)
    uv = data.astype(np.float32) * scale
    x, env = minmax_envelope(t - t[-1], uv, max_bins)
    return Snapshot(version, x, env, len(t), uv.mean(axis=0), uv.std(axis=0),
                    lo_c * scale, hi_c * scale, dead, railed, status)


class Acquirer:
    """Reads the source on its own thread and publishes ready-to-draw snapshots.

    The window must never wait on a serial port (a UART snapshot is ~100 round
    trips, about a second) or on parsing a few thousand frames a second, or every
    hover and key press waits too. So the source, the frame bookkeeping and the
    per-window statistics all live here; the display only picks up the newest
    Snapshot, which is a single locked reference swap.
    """

    def __init__(self, source, buf: Buffers, kind: str, link: LinkPoller = None,
                 window_s: float = 2.0, hz: float = 30.0, clock=time.monotonic):
        self.source, self.buf, self.kind, self.link = source, buf, kind, link
        self.window_s, self.period, self.clock = window_s, 1.0 / hz, clock
        self.error = ""
        self._snap = None
        self._version = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.th = threading.Thread(target=self._run, name="viewer-acquire", daemon=True)

    def start(self) -> "Acquirer":
        self.th.start()
        return self

    def latest(self):
        with self._lock:
            return self._snap

    def publish(self) -> None:
        lk = self.link.last if self.link else None
        note = (self.link.error if self.link else "") or self.error
        status = status_line(self.buf, self.clock(), self.kind, lk, note)
        snap = make_snapshot(self.buf, self.window_s, status, self._version + 1)
        if snap is not None:
            with self._lock:
                self._version += 1
                self._snap = snap

    def _run(self) -> None:
        next_pub = 0.0
        while not self._stop.is_set():
            try:
                n = self.source.poll(self.buf)
                self.error = ""
            except Exception as e:          # a serial hiccup must not kill the window
                n = 0
                self.error = f"source: {e}"
                self._stop.wait(0.2)
            if self.link:
                self.link.poll()
            now = self.clock()
            # UART snapshots are slow and each one is news; UDP frames arrive in a
            # stream and are worth publishing at the display's frame rate only.
            if (n and self.kind == "uart") or now >= next_pub:
                self.publish()
                next_pub = now + self.period
            if not n and self.kind != "uart":
                self._stop.wait(0.004)      # let the receiver thread fill up

    def stop(self) -> None:
        self._stop.set()
        if self.th.is_alive():
            self.th.join(timeout=3.0)


# ---------------------------------------------------------------------------
# Drawing (matplotlib and numpy imported here only)
# ---------------------------------------------------------------------------

class Scope:
    """Activity map (top) and per-channel lanes (bottom) for all 128 channels.

    Keys: s  per-channel <-> shared scale for the lanes
          space  freeze the display (acquisition carries on)
    Hover a tile or a lane for that channel's numbers.

    Speed. Nothing here waits on hardware (see Acquirer). The axes, labels and
    colorbar are drawn once; after that only small screen regions are repainted
    by blitting, each when its own content changed:
      lanes (4 regions)    on every new snapshot, up to 30 fps
      status line          on new data, hover or a key press
      activity map rows    one row of 16 tiles per frame, round robin
      lane-scale labels    one panel's 32 labels per frame, only when they change
    Text is what costs most in matplotlib, so no frame ever draws more than
    one row or one panel of it. Lanes are 4 LineCollections fed with a min/max
    envelope and rasterized by numpy (lane_blocks) at each panel's exact pixel
    size, then written straight into the canvas buffer under the cached
    background -- matplotlib's line and image machinery never sees the
    dense data. Sparse UART snapshots are drawn as lines and dots instead.
    """

    FRAME_MS = 33
    # The header is three lines -- status, hover readout, key help -- placed in
    # points below the window's top edge, one line each, so they cannot run into
    # each other or into the map title whatever the window size; _layout() keeps
    # the plots PLOTS_PT below the top as the window is resized.
    STATUS_PT, HOVER_PT, HELP_PT = 6, 21, 36    # top of each line
    BAND_PT = 34                                # status + hover: the repainted band
    PLOTS_PT = 74                               # top of the activity map, under its title

    def __init__(self, acq, kind: str, window_s: float = 2.0, spacing_uv: float = None,
                 clock=time.monotonic):
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.collections import LineCollection
        from matplotlib.colors import LinearSegmentedColormap, LogNorm
        from matplotlib.patches import Rectangle
        from matplotlib.transforms import blended_transform_factory, offset_copy
        self.np, self.plt = np, plt
        self.acq, self.kind, self.window_s, self.clock = acq, kind, window_s, clock
        self.shared_uv = spacing_uv          # None: auto when shared mode is on
        self.shared = False
        self.paused = False
        self.snap = None
        self.half = None
        self._drawn = None
        self._rg = None                      # region name -> (bbox, background, artists)
        self._rows_pending, self._scales_pending = set(), set()
        self._row_cursor = 0
        self._status_dirty = True
        self._status_t = -1e9
        self._tick_n = 0

        plt.rcParams.update({"font.family": "sans-serif", "font.size": 8,
                             "axes.edgecolor": INK_3, "axes.labelcolor": INK_2,
                             "xtick.color": INK_2, "ytick.color": INK_2,
                             "text.color": INK, "path.simplify": True,
                             "path.simplify_threshold": 0.5})
        self.fig = plt.figure(figsize=(16, 10), facecolor=SURFACE)
        try:
            self.fig.canvas.manager.set_window_title("intan-chain: 128-channel activity")
        except Exception:
            pass
        gs = self.gs = self.fig.add_gridspec(2, 4, height_ratios=[1.0, 1.75], hspace=0.16,
                                             wspace=0.32, left=0.045, right=0.955, top=0.9,
                                             bottom=0.05)    # top: set by _layout()

        # ---- activity map ----
        self.amap = self.fig.add_subplot(gs[0, :])
        self.amap.set_facecolor(SURFACE)
        cmap = LinearSegmentedColormap.from_list("seq_blue", SEQ_BLUE)
        cmap.set_bad(DEAD_FILL)
        self.norm = LogNorm(ACT_MIN_UV, ACT_MAX_UV, clip=True)
        self.img = self.amap.imshow(np.full((GRID_ROWS, GRID_COLS), np.nan), cmap=cmap,
                                    norm=self.norm, aspect="auto", interpolation="nearest",
                                    animated=True)
        self.amap.set_xlim(-0.5, GRID_COLS - 0.5)
        self.amap.set_ylim(GRID_ROWS - 0.5, -0.5)
        self.amap.set_xticks([])
        self.amap.set_yticks([2 * m + 0.5 for m in range(4)])
        self.amap.set_yticklabels(MODULES, fontsize=9, color=INK)
        self.amap.tick_params(length=0)
        for s in self.amap.spines.values():
            s.set_visible(False)
        self.amap.set_title("activity: RMS over the window, per channel", loc="left",
                            fontsize=10, color=INK, pad=6)
        # Tile gaps, per row, so each row repaints with only its own artists.
        self.row_art = []
        for r in range(GRID_ROWS):
            lw_top = 3 if r % 2 == 0 and r else 1           # thicker rule between modules
            art = [self.amap.axhline(r - 0.5, color=SURFACE, lw=lw_top, animated=True),
                   self.amap.axhline(r + 0.5, color=SURFACE, lw=1, animated=True)]
            art += [self.amap.plot([c - 0.5, c - 0.5], [r - 0.5, r + 0.5], color=SURFACE,
                                   lw=1, animated=True)[0] for c in range(GRID_COLS + 1)]
            self.row_art.append(art)
        self.tile_txt, self.tile_box = [], []
        for ch in range(C.N_CH):
            r, c = grid_pos(ch)
            self.tile_txt.append(self.amap.text(c, r, f"{ch}", ha="center", va="center",
                                                fontsize=7.5, color=INK, animated=True))
            box = Rectangle((c - 0.46, r - 0.42), 0.92, 0.84, fill=False, lw=1.8,
                            edgecolor=CRITICAL, visible=False, animated=True)
            self.amap.add_patch(box)
            self.tile_box.append(box)
        cax = self.cax = self.fig.add_axes([0.962, 0.635, 0.008, 0.265])   # see _layout()
        cb = self.fig.colorbar(self.img, cax=cax)
        cb.set_ticks([1, 10, 100, 1000, 10000])
        cb.set_ticklabels(["1 µV", "10 µV", "100 µV", "1 mV", "10 mV"])
        cb.ax.tick_params(labelsize=7, color=INK_3)
        cb.outline.set_visible(False)

        # ---- lanes and their scale labels ----
        self.axes, self.lcs, self.dots, self.scale_txt, self.rimgs = [], [], [], [], []
        self.lane_px = 16
        self._rect = [None] * 4              # (row0, row1, col0, col1) of each panel in the buffer
        self._raster = [None] * 4            # (h, w) uint32 per panel when the data is dense
        self._raster_c0 = [0] * 4
        self.lane_y = np.arange(32)[::-1].astype(float)   # channel 0 on top
        for m in range(4):
            ax = self.fig.add_subplot(gs[1, m])
            ax.set_facecolor(SURFACE)
            ax.set_title(f"{MODULES[m]}  (ch {32 * m}-{32 * m + 31})", fontsize=9,
                         color=INK, loc="left")
            ax.set_ylim(-0.6, 31.6)
            ax.set_xlim(-window_s, 0)
            ax.set_yticks(self.lane_y)
            ax.set_yticklabels([str(32 * m + i) for i in range(32)], fontsize=6.5)
            ax.tick_params(axis="y", length=0, pad=2)
            ax.tick_params(axis="x", labelsize=7)
            ax.set_xlabel("seconds before now", fontsize=7)
            for y in self.lane_y:
                ax.axhline(y - 0.5, color="#eeede9", lw=0.6, zorder=0)
            for s in ("top", "right"):
                ax.spines[s].set_visible(False)
            lc = LineCollection([], linewidths=0.9, colors=TRACE, animated=True,
                                antialiaseds=False)
            ax.add_collection(lc)
            dots = ax.scatter([], [], s=3, color=TRACE, animated=True, visible=False)
            rimg = ax.imshow(np.zeros((1, 1, 4), np.uint8), extent=(-window_s, 0, -0.5, 31.5),
                             aspect="auto", interpolation="nearest", origin="upper",
                             animated=True, visible=False, zorder=2)
            ax.set_xlim(-window_s, 0)
            ax.set_ylim(-0.6, 31.6)
            self.rimgs.append(rimg)
            tr = blended_transform_factory(ax.transAxes, ax.transData)
            self.scale_txt.append([ax.text(1.01, y, "", transform=tr, fontsize=6,
                                           color=INK_3, va="center", ha="left",
                                           animated=True, clip_on=False)
                                   for y in self.lane_y])
            self.axes.append(ax)
            self.lcs.append(lc)
            self.dots.append(dots)

        def below_top(pt):
            return offset_copy(self.fig.transFigure, fig=self.fig, y=-pt, units="points")
        self.status = self.fig.text(0.045, 1.0, "", fontsize=9, family="monospace",
                                    va="top", color=INK, animated=True,
                                    transform=below_top(self.STATUS_PT))
        self.hover = self.fig.text(0.045, 1.0, "hover a tile or a lane for details",
                                   fontsize=9, va="top", color=INK_2, animated=True,
                                   transform=below_top(self.HOVER_PT))
        self.help = self.fig.text(0.045, 1.0, "", fontsize=8, va="top", color=INK_2,
                                  transform=below_top(self.HELP_PT))
        self._set_help()
        self._layout()
        c = self.fig.canvas
        c.mpl_connect("resize_event", self._layout)
        c.mpl_connect("draw_event", self._on_draw)
        c.mpl_connect("key_press_event", self._key)
        c.mpl_connect("motion_notify_event", self._motion)
        c.mpl_connect("close_event", lambda _ev: self._close())

    # ---- layout ----
    def _layout(self, _ev=None):
        """Start the plots PLOTS_PT below the top edge at any window height, under
        the header, instead of at a fixed fraction of the window."""
        top = 1.0 - self.PLOTS_PT / 72.0 / self.fig.get_figheight()
        self.gs.update(top=max(0.5, top))
        for ax in [self.amap] + self.axes:
            ax.set_position(ax.get_subplotspec().get_position(self.fig))
        p = self.amap.get_position()
        self.cax.set_position([0.962, p.y0, 0.008, p.height])

    # ---- regions ----
    def _regions(self):
        """name -> (display bbox, artists drawn into it), for every repainted region."""
        from matplotlib.transforms import Bbox
        fig, out = self.fig, {}
        for r in range(GRID_ROWS):
            (x0, y0), (x1, y1) = self.amap.transData.transform(
                [(-0.5, r - 0.5), (GRID_COLS - 0.5, r + 0.5)])
            bb = Bbox.from_extents(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))
            chans = [grid_channel(r, c) for c in range(GRID_COLS)]
            out[f"row{r}"] = (bb, [self.img] + self.row_art[r]
                              + [self.tile_box[ch] for ch in chans]
                              + [self.tile_txt[ch] for ch in chans])
        for m, ax in enumerate(self.axes):
            nxt = self.axes[m + 1].bbox.x0 if m < 3 else fig.bbox.x1
            x0, x1 = ax.bbox.x1 + 1, ax.bbox.x1 + 0.62 * (nxt - ax.bbox.x1)
            out[f"scale{m}"] = (Bbox.from_extents(x0, ax.bbox.y0 - 6, x1, ax.bbox.y1 + 6),
                                self.scale_txt[m])
            out[f"lanes{m}"] = (ax.bbox.frozen(), [self.rimgs[m], self.lcs[m], self.dots[m]])
        # The full width of the status and hover lines, so a line that gets shorter
        # is erased in full; the help line below the band is static and stays out.
        band = self.BAND_PT * fig.dpi / 72.0
        out["status"] = (Bbox.from_extents(fig.bbox.x0, fig.bbox.y1 - band,
                                           fig.bbox.x1, fig.bbox.y1),
                         [self.status, self.hover])
        return out

    def _on_draw(self, _ev):
        """A full redraw (first show, resize) skipped every animated artist:
        cache the clean regions, then paint all of them back on."""
        c = self.fig.canvas
        if c.is_saving():                    # savefig (incl. the toolbar's save button) draws
            return                           # at its own dpi: not the screen, leave it alone
        H = int(round(self.fig.bbox.height))
        for m, ax in enumerate(self.axes):
            x0, y0, x1, y1 = (int(round(v)) for v in ax.bbox.extents)
            self._rect[m] = (H - y1, H - y0, x0, x1)
        h = self._rect[0][1] - self._rect[0][0]
        self.lane_px = max(4, int(h / 32.2))
        if self.snap is not None:
            self._prep_lanes()
        self._rg = {k: (b, c.copy_from_bbox(b), a) for k, (b, a) in self._regions().items()}
        for k in self._rg:
            self._paint(k)

    def _paint(self, name):
        bbox, bg, artists = self._rg[name]
        c = self.fig.canvas
        c.restore_region(bg)
        if name.startswith("lanes") and self._raster[int(name[-1])] is not None:
            m = int(name[-1])
            np = self.np
            r0, r1, c0, c1 = self._rect[m]
            buf = np.asarray(c.buffer_rgba()).view(np.uint32)[..., 0]
            img = self._raster[m]
            dst = buf[r0:r1, c0 + self._raster_c0[m]:c1]
            np.copyto(dst, img[:dst.shape[0], :dst.shape[1]],
                      where=img[:dst.shape[0], :dst.shape[1]] != 0)
            c.blit(bbox)
            return
        clip = name.startswith("row")
        for a in artists:
            if a.get_visible():
                if clip:                       # keep the buffer outside this row untouched
                    a.set_clip_box(bbox)
                self.fig.draw_artist(a)
        c.blit(bbox)

    # ---- data -> artists ----
    def _prep_lanes(self):
        np, s = self.np, self.snap
        shared = None
        if self.shared:
            ok = ~s.railed & ~s.dead
            shared = self.shared_uv or (float(np.percentile(
                np.abs(s.env[:, ok] - np.median(s.env[:, ok], axis=0)), 99))
                if ok.any() else 100.0)
        y, self.half = lane_values(s.env, shared)
        x = s.x
        dense = len(x) >= 2 * 100 and len(x) % 2 == 0
        colors = np.where(s.railed, CRITICAL, np.where(s.dead, INK_3, TRACE))
        packed = pack_rgba(colors) if dense else None
        for m in range(4):
            sl = slice(32 * m, 32 * m + 32)
            self.lcs[m].set_visible(not dense)
            self.dots[m].set_visible(not dense)
            self._raster[m] = None
            if dense and self._rect[m] is not None:
                r0, r1, c0, c1 = self._rect[m]
                h, w = r1 - r0, c1 - c0
                # the window may not be full yet: start at the oldest sample's column
                cs = min(w - 1, max(0, int(round((float(x[0]) + self.window_s)
                                                  / self.window_s * w))))
                blocks = lane_blocks(y[:, sl], self.lane_px, packed[sl], w - cs)
                centers = (0.6 + np.arange(32)) / 32.2 * h
                self._raster[m] = compose_lanes(blocks, h, centers)
                self._raster_c0[m] = cs
            elif not dense:
                seg = np.empty((32, len(x), 2))
                seg[:, :, 0] = x
                seg[:, :, 1] = y[:, sl].T + self.lane_y[:, None]
                self.lcs[m].set_segments(seg)
                self.lcs[m].set_colors(list(colors[sl]))
                self.dots[m].set_offsets(seg.reshape(-1, 2))
            labels = [f"±{fmt_uv(self.half[32 * m + i])}" for i in range(32)]
            if labels != [t.get_text() for t in self.scale_txt[m]]:
                for t, lab in zip(self.scale_txt[m], labels):
                    t.set_text(lab)
                self._scales_pending.add(m)
        now = self.clock()
        if now - self._status_t >= 0.25:
            self._status_t = now
            self.status.set_text(s.status)
            self._status_dirty = True

    def _prep_map(self):
        np, s = self.np, self.snap
        v = np.maximum(s.rms, ACT_MIN_UV)
        shade = self.img.cmap(self.norm(v))                       # one call, all 128 tiles
        dark = (0.2126 * shade[:, 0] + 0.7152 * shade[:, 1] + 0.0722 * shade[:, 2]) < 0.45
        grid = np.full((GRID_ROWS, GRID_COLS), np.nan)
        for ch in range(C.N_CH):
            r, c = grid_pos(ch)
            txt = self.tile_txt[ch]
            if s.dead[ch]:
                txt.set_text(f"{ch}\nDEAD")
                txt.set_color(INK_2)
            else:
                grid[r, c] = v[ch]
                txt.set_text(f"{ch}\n{fmt_uv(s.rms[ch])}" + ("\nRAIL" if s.railed[ch] else ""))
                txt.set_color(SURFACE if dark[ch] else INK)
            self.tile_box[ch].set_visible(bool(s.railed[ch]))
        self.img.set_data(grid)
        self._rows_pending = set(range(GRID_ROWS))

    # ---- the frame clock ----
    def _tick(self):
        if self._rg is None:
            return
        self._tick_n += 1
        snap = self.acq.latest()
        if snap is not None and snap.version != self._drawn and not self.paused:
            self.snap, self._drawn = snap, snap.version
            self._prep_lanes()
            if not self._rows_pending:
                self._prep_map()
            for m in range(4):
                self._paint(f"lanes{m}")
        if self._status_dirty:
            self._paint("status")
            self._status_dirty = False
        # At most one text-heavy region per frame: a map row, or a scale panel.
        if self._scales_pending and (self._tick_n % 2 or not self._rows_pending):
            self._paint(f"scale{self._scales_pending.pop()}")
        elif self._rows_pending:
            for k in range(GRID_ROWS):
                r = (self._row_cursor + k) % GRID_ROWS
                if r in self._rows_pending:
                    break
            self._rows_pending.discard(r)
            self._row_cursor = (r + 1) % GRID_ROWS
            self._paint(f"row{r}")

    # ---- interaction ----
    def _set_help(self):
        mode = "shared scale" if self.shared else "each lane scaled to its own channel"
        self.help.set_text(f"lanes: {mode}  [s] switch   [space] "
                           f"{'resume' if self.paused else 'freeze'}   "
                           "red box + RAIL = at the ADC rail; grey + DEAD = no change")

    def _key(self, ev):
        if ev.key == "s":
            self.shared = not self.shared
            if self.snap is not None:
                self._prep_lanes()
                for m in range(4):
                    self._paint(f"lanes{m}")
        elif ev.key == " ":
            self.paused = not self.paused
        else:
            return
        self._set_help()
        self.fig.canvas.draw_idle()          # the help line is static: redraw everything once

    def _channel_at(self, ev):
        if ev.inaxes is self.amap and ev.xdata is not None:
            r, c = int(round(ev.ydata)), int(round(ev.xdata))
            if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
                return grid_channel(r, c)
        for m, ax in enumerate(self.axes):
            if ev.inaxes is ax and ev.ydata is not None:
                lane = int(round(31 - ev.ydata))
                if 0 <= lane < 32:
                    return 32 * m + lane
        return None

    def _motion(self, ev):
        ch = self._channel_at(ev)
        if ch is None or self.snap is None:
            return
        s = self.snap.stat(ch)
        tag = {"railed": "  RAIL", "dead": "  DEAD"}.get(s.flag, "")
        self.hover.set_text(f"ch {ch} ({MODULES[ch // 32]} #{ch % 32}):  rms {fmt_uv(s.rms_uv)}"
                            f"   mean {fmt_uv(s.mean_uv)}   min {fmt_uv(s.min_uv)}"
                            f"   max {fmt_uv(s.max_uv)}   n {s.n}{tag}")
        self._status_dirty = True            # painted on the next tick, <= 33 ms away

    def _close(self):
        if getattr(self, "timer", None) is not None:
            self.timer.stop()
        self.acq.stop()

    # ---- running ----
    def render(self, snap):
        """Load one snapshot into every artist synchronously (tests, screenshots)."""
        self.snap, self._drawn = snap, snap.version
        self._prep_lanes()
        self._prep_map()

    def save_png(self, path: str, dpi: int = 90):
        """Everything, including the animated layers, into one image file."""
        np = self.np
        for m, img in enumerate(self._raster):
            if img is not None:
                rgba = img.view(np.uint8).reshape(img.shape[0], img.shape[1], 4)
                cs = self._raster_c0[m] / max(1, self._rect[m][3] - self._rect[m][2])
                self.rimgs[m].set_data(rgba)
                self.rimgs[m].set_extent((-self.window_s * (1 - cs), 0.0, -0.6, 31.6))
                self.rimgs[m].set_visible(True)
        animated = [a for a in self.fig.findobj() if a.get_animated()]
        for a in animated:
            a.set_animated(False)
            a.set_clip_box(a.axes.bbox if getattr(a, "axes", None) is not None else None)
        try:
            self.fig.savefig(path, dpi=dpi, facecolor=self.fig.get_facecolor())
        finally:
            for a in animated:
                a.set_animated(True)
            for r in self.rimgs:
                r.set_visible(False)

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
    src.add_argument("--udp", action="store_true", help="read datagrams on --udp-port")
    src.add_argument("--uart", action="store_true",
                     help="diagnostic CONVERT snapshots over --serial (no ESP)")
    ap.add_argument("--serial", help="FPGA control UART")
    ap.add_argument("--baud", type=int, default=C.UART_BAUD)
    ap.add_argument("--udp-port", type=int, default=C.UDP_PORT)
    ap.add_argument("--start", action="store_true",
                    help="with --udp --serial: configure the chips (bring-up S0, S2-S5) and "
                         "set run; cleared again on exit unless --leave-running")
    ap.add_argument("--no-config", action="store_true",
                    help="with --uart: skip configuring (the chips were just brought up)")
    ap.add_argument("--leave-running", action="store_true")
    ap.add_argument("--spi-div", type=int, default=0)
    ap.add_argument("--phase", type=B.phase_arg, default=None, metavar="N|auto",
                    help="PHASE_SELECT for --start/--uart, or auto (default)")
    ap.add_argument("--rate", type=float, default=2000.0)
    ap.add_argument("--lower", type=float, default=1.0)
    ap.add_argument("--upper", type=float, default=500.0)
    ap.add_argument("--window", type=float, default=2.0, help="seconds shown")
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
    if a.start and not (a.udp and a.serial):
        ap.error("--start needs --udp and --serial")
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
        if a.udp:
            source = UdpSource(a.udp_port)
            link = LinkPoller(ctrl) if ctrl is not None else None
            depth = int(a.window * a.rate * 1.5) + 10
        else:
            if ctrl.read("CTRL") & 1:
                out("run is set: stopping it, the diagnostic list needs it clear")
                ctrl.set_run(False)
            source = UartSource(Diag(ctrl))
            depth = 2000
        buf = Buffers(depth=depth)
        kind = "udp" if a.udp else "uart"
        if a.headless is not None:
            run_headless(source, buf, a.headless, out=out, link=link, kind=kind)
        else:
            win = a.window if a.udp else max(a.window, 20)
            acq = Acquirer(source, buf, kind, link, window_s=win).start()
            try:
                Scope(acq, kind, window_s=win, spacing_uv=a.spacing).show()
            finally:
                acq.stop()          # before the port closes below
    finally:
        if source is not None:
            source.close()
        if ctrl is not None:
            if started and not a.leave_running:
                try:
                    ctrl.set_run(False)
                except CtrlError:
                    pass
            ctrl.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
