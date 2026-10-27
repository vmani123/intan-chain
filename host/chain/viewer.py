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


# ---------------------------------------------------------------------------
# Drawing (matplotlib and numpy imported here only)
# ---------------------------------------------------------------------------

class Scope:
    """Activity map (top) and per-channel lanes (bottom) for all 128 channels.

    Keys: s  per-channel <-> shared scale for the lanes
          space  freeze the display (acquisition carries on)
    Hover a tile or a lane for that channel's numbers.
    """

    def __init__(self, source, buf: Buffers, kind: str, link: LinkPoller = None,
                 window_s: float = 2.0, spacing_uv: float = None, demean: bool = True,
                 max_points: int = 1500, clock=time.monotonic):
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.colors import LinearSegmentedColormap, LogNorm
        from matplotlib.patches import Rectangle
        self.np, self.plt = np, plt
        self.source, self.buf, self.kind, self.link = source, buf, kind, link
        self.window_s, self.max_points, self.clock = window_s, max_points, clock
        self.shared_uv = spacing_uv          # None: auto when shared mode is on
        self.shared = False
        self.paused = False
        self.stats = None                    # per-channel numbers for hover
        self.labels_t = -1e9

        plt.rcParams.update({"font.family": "sans-serif", "font.size": 8,
                             "axes.edgecolor": INK_3, "axes.labelcolor": INK_2,
                             "xtick.color": INK_2, "ytick.color": INK_2,
                             "text.color": INK})
        self.fig = plt.figure(figsize=(16, 10), facecolor=SURFACE)
        try:
            self.fig.canvas.manager.set_window_title("intan-chain: 128-channel activity")
        except Exception:
            pass
        gs = self.fig.add_gridspec(2, 4, height_ratios=[1.0, 1.75], hspace=0.16,
                                   wspace=0.32, left=0.045, right=0.955, top=0.9,
                                   bottom=0.05)

        # ---- activity map ----
        self.amap = self.fig.add_subplot(gs[0, :])
        self.amap.set_facecolor(SURFACE)
        cmap = LinearSegmentedColormap.from_list("seq_blue", SEQ_BLUE)
        cmap.set_bad(DEAD_FILL)
        self.norm = LogNorm(ACT_MIN_UV, ACT_MAX_UV, clip=True)
        self.grid = np.full((GRID_ROWS, GRID_COLS), np.nan)
        self.img = self.amap.imshow(self.grid, cmap=cmap, norm=self.norm, aspect="auto",
                                    interpolation="nearest")
        self.amap.set_xticks([])
        self.amap.set_yticks([2 * m + 0.5 for m in range(4)])
        self.amap.set_yticklabels(MODULES, fontsize=9, color=INK)
        self.amap.tick_params(length=0)
        for s in self.amap.spines.values():
            s.set_visible(False)
        for m in range(1, 4):                 # white rules between modules
            self.amap.axhline(2 * m - 0.5, color=SURFACE, lw=3)
        for r in range(GRID_ROWS):            # thin surface gaps between tiles
            self.amap.axhline(r - 0.5, color=SURFACE, lw=1)
        for c in range(GRID_COLS + 1):
            self.amap.axvline(c - 0.5, color=SURFACE, lw=1)
        self.amap.set_title("activity: RMS over the window, per channel", loc="left",
                            fontsize=10, color=INK, pad=6)
        self.tile_txt, self.tile_box = [], []
        for ch in range(C.N_CH):
            r, c = grid_pos(ch)
            self.tile_txt.append(self.amap.text(c, r, f"{ch}", ha="center", va="center",
                                                fontsize=7.5, color=INK))
            box = Rectangle((c - 0.46, r - 0.42), 0.92, 0.84, fill=False, lw=1.8,
                            edgecolor=CRITICAL, visible=False)
            self.amap.add_patch(box)
            self.tile_box.append(box)
        cax = self.fig.add_axes([0.962, 0.635, 0.008, 0.265])
        cb = self.fig.colorbar(self.img, cax=cax)
        cb.set_ticks([1, 10, 100, 1000, 10000])
        cb.set_ticklabels(["1 µV", "10 µV", "100 µV", "1 mV", "10 mV"])
        cb.ax.tick_params(labelsize=7, color=INK_3)
        cb.outline.set_visible(False)

        # ---- lanes ----
        self.axes, self.lines, self.scale_ax = [], [], []
        self.lane_y = np.arange(32)[::-1].astype(float)   # channel 0 on top
        for m in range(4):
            ax = self.fig.add_subplot(gs[1, m])
            ax.set_facecolor(SURFACE)
            ax.set_title(f"{MODULES[m]}  (ch {32 * m}-{32 * m + 31})", fontsize=9,
                         color=INK, loc="left")
            ax.set_ylim(-0.6, 31.6)
            ax.set_yticks(self.lane_y)
            ax.set_yticklabels([str(32 * m + i) for i in range(32)], fontsize=6.5)
            ax.tick_params(axis="y", length=0, pad=2)
            ax.tick_params(axis="x", labelsize=7)
            ax.set_xlabel("seconds before now", fontsize=7)
            for y in self.lane_y:              # faint lane separators
                ax.axhline(y - 0.5, color="#eeede9", lw=0.6, zorder=0)
            for s in ("top", "right"):
                ax.spines[s].set_visible(False)
            sx = ax.twinx()                    # right edge: what a full lane means
            sx.set_ylim(ax.get_ylim())
            sx.set_yticks(self.lane_y)
            sx.set_yticklabels([""] * 32, fontsize=6, color=INK_3)
            sx.tick_params(axis="y", length=0, pad=1)
            for s in sx.spines.values():
                s.set_visible(False)
            for i in range(32):
                (ln,) = ax.plot([], [], lw=0.8, color=TRACE, solid_capstyle="round")
                self.lines.append(ln)
            self.axes.append(ax)
            self.scale_ax.append(sx)

        self.status = self.fig.text(0.045, 0.975, "", fontsize=9, family="monospace",
                                    va="top", color=INK)
        self.hover = self.fig.text(0.045, 0.947, "hover a tile or a lane for details",
                                   fontsize=9, va="top", color=INK_2)
        self.help = self.fig.text(0.955, 0.975, "", fontsize=8, ha="right", va="top",
                                  color=INK_2)
        self.fig.canvas.mpl_connect("key_press_event", self._key)
        self.fig.canvas.mpl_connect("motion_notify_event", self._motion)
        self._set_help()

    # ---- interaction ----
    def _set_help(self):
        mode = "shared scale" if self.shared else "each lane scaled to its own channel"
        self.help.set_text(f"lanes: {mode}  [s] switch   [space] "
                           f"{'resume' if self.paused else 'freeze'}   "
                           "red box + RAIL = at the ADC rail; grey + DEAD = no change")

    def _key(self, ev):
        if ev.key == "s":
            self.shared = not self.shared
            self.labels_t = -1e9
        elif ev.key == " ":
            self.paused = not self.paused
        self._set_help()
        self.fig.canvas.draw_idle()

    def _channel_at(self, ev):
        if ev.inaxes is self.amap and ev.xdata is not None:
            r, c = int(round(ev.ydata)), int(round(ev.xdata))
            if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
                return grid_channel(r, c)
        for m, ax in enumerate(self.axes + self.scale_ax):
            if ev.inaxes is ax and ev.ydata is not None:
                lane = int(round(31 - ev.ydata))
                if 0 <= lane < 32:
                    return 32 * (m % 4) + lane
        return None

    def _motion(self, ev):
        ch = self._channel_at(ev)
        if ch is None or self.stats is None:
            return
        s = self.stats[ch]
        tag = {"railed": "  RAIL", "dead": "  DEAD"}.get(s.flag, "")
        self.hover.set_text(f"ch {ch} ({MODULES[ch // 32]} #{ch % 32}):  rms {fmt_uv(s.rms_uv)}"
                            f"   mean {fmt_uv(s.mean_uv)}   min {fmt_uv(s.min_uv)}"
                            f"   max {fmt_uv(s.max_uv)}   n {s.n}{tag}")
        self.fig.canvas.draw_idle()

    # ---- one frame ----
    def update(self, _frame=None):
        np, buf = self.np, self.buf
        self.source.poll(buf)
        lk = self.link.poll() if self.link else None
        now = self.clock()
        self.status.set_text(status_line(buf, now, self.kind, lk,
                                         self.link.error if self.link else ""))
        if self.paused or not buf.t:
            return self.lines
        t = np.fromiter(buf.t, float, len(buf.t))
        data = np.frombuffer(buf.raw(), dtype=np.int16).reshape(-1, buf.n_ch)
        n = min(len(t), len(data))
        t, data = t[-n:], data[-n:]
        keep = t >= t[-1] - self.window_s
        t, data = t[keep], data[keep]
        lo, hi = data.min(axis=0), data.max(axis=0)
        dead = (lo == hi) & (len(data) >= 2)
        railed = np.maximum(np.abs(lo.astype(int)), np.abs(hi.astype(int))) >= RAIL_LSB
        uv = data.astype(float) * R.SCALE_UV
        rms = uv.std(axis=0)
        self.stats = [ChannelStat(ch, len(uv), float(uv[:, ch].mean()), float(rms[ch]),
                                  float(uv[:, ch].min()), float(uv[:, ch].max()),
                                  bool(dead[ch]), bool(railed[ch]))
                      for ch in range(buf.n_ch)]

        # activity map: dead tiles are NaN, drawn in the 'bad' grey
        grid = np.full((GRID_ROWS, GRID_COLS), np.nan)
        for ch in range(buf.n_ch):
            if not dead[ch]:
                r, c = grid_pos(ch)
                grid[r, c] = max(rms[ch], ACT_MIN_UV)
        self.img.set_data(grid)

        # lanes
        shared = None
        if self.shared:
            ok = ~railed & ~dead
            shared = self.shared_uv or (float(np.percentile(
                np.abs(uv[:, ok] - np.median(uv[:, ok], axis=0)), 99)) if ok.any() else 100.0)
        y, half = lane_values(uv, shared)
        step = max(1, len(t) // self.max_points)
        x = t[::step] - t[-1]
        y = y[::step]
        marker = "o" if len(x) < 120 else ""            # sparse UART snapshots: show the points
        for ch, ln in enumerate(self.lines):
            ln.set_data(x, y[:, ch] + self.lane_y[ch % 32])
            ln.set_color(CRITICAL if railed[ch] else INK_3 if dead[ch] else TRACE)
            ln.set_marker(marker)
            ln.set_markersize(1.6)
        for ax in self.axes:
            ax.set_xlim(-self.window_s, 0)

        if now - self.labels_t > 1.0:      # text is the slow part: once a second
            self.labels_t = now
            for ch in range(buf.n_ch):
                r, c = grid_pos(ch)
                txt = self.tile_txt[ch]
                if dead[ch]:
                    txt.set_text(f"{ch}\nDEAD")
                    txt.set_color(INK_2)
                else:
                    txt.set_text(f"{ch}\n{fmt_uv(rms[ch])}" + ("\nRAIL" if railed[ch] else ""))
                    shade = self.img.cmap(self.norm(max(rms[ch], ACT_MIN_UV)))
                    lum = 0.2126 * shade[0] + 0.7152 * shade[1] + 0.0722 * shade[2]
                    txt.set_color(SURFACE if lum < 0.45 else INK)
                self.tile_box[ch].set_visible(bool(railed[ch]))
            for m, sx in enumerate(self.scale_ax):
                sx.set_yticklabels([f"±{fmt_uv(half[32 * m + i])}" for i in range(32)])
        return self.lines

    def show(self):
        from matplotlib.animation import FuncAnimation
        self.anim = FuncAnimation(self.fig, self.update,
                                  interval=100 if self.kind == "udp" else 50,
                                  cache_frame_data=False)   # kept on self, or it is dropped
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
    ap.add_argument("--no-demean", action="store_true", help="draw raw values, offsets included")
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
            Scope(source, buf, kind, link, window_s=a.window if a.udp else max(a.window, 20),
                  spacing_uv=a.spacing, demean=not a.no_demean).show()
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
