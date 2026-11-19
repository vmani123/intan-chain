"""Scrolling through the viewer's history: plain numpy, no matplotlib, no display.

  History          every frame of the last N seconds in one numpy ring (int16)
  column_stats     frames -> one min/max column per pixel column of the panel
  Decimator        the same, reusing the columns that can no longer change
  assemble         columns -> vertical spans that join up, broken where frames are missing
  TimeView         what time is shown: live, or parked at a right edge; pan, zoom
  LaneView         which channels are shown: how many lanes, which one on top
  rasterize_lanes  spans -> one RGBA image, one lane per channel

Time is in seconds since the first frame (the viewer's t_cap clock), and the
panel's pixel columns are pinned to it: column k always covers [k dt, (k+1) dt).
So a min/max column never changes once its time has passed, the live view
does not shimmer as it scrolls, and only the newest columns are recomputed.

chain.viewer imports this module only when it draws, so headless runs still
need neither numpy nor matplotlib.
"""
import math
import threading
from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# History: a ring of frames
# ---------------------------------------------------------------------------


class History:
    """The newest `capacity` frames: samples (int16), time (s) and a break flag.

    gap[i] is True when something is missing between frame i-1 and frame i:
    a seq gap (the caller says so), time going backwards, or a jump in time
    longer than `gap_s` (None: never). Frames are staged by append(), which is
    cheap enough to call per frame, and copied into the ring in one go by
    flush(). append() may run on another thread than everything else (the
    viewer's acquisition thread appends, its render thread flushes and reads):
    only the staging lists are shared, under a lock held for a list swap.
    """

    def __init__(self, n_ch: int = 128, capacity: int = 120_000, gap_s: float = None):
        if capacity < 2:
            raise ValueError("a history needs room for at least two frames")
        self.n_ch, self.cap, self.gap_s = n_ch, int(capacity), gap_s
        self.data = np.zeros((self.cap, n_ch), np.int16)
        self.t = np.zeros(self.cap)
        self.gap = np.zeros(self.cap, bool)
        self.total = 0                      # frames ever stored
        self._rows = bytearray()
        self._ts, self._gaps = [], []
        self._origin = None
        self._t_last = None
        self._lock = threading.Lock()

    # --- input --------------------------------------------------------------
    def append(self, row, t: float, gap: bool = False) -> None:
        """One frame: n_ch int16 values (bytes-like, e.g. array('h')) at time t."""
        if self._origin is None:
            self._origin = t
        t = t - self._origin
        last = self._t_last
        if last is not None:
            if t < last:                    # clock went backwards: keep order, mark a break
                t, gap = last, True
            elif self.gap_s is not None and t - last > self.gap_s:
                gap = True
        self._t_last = t
        with self._lock:
            self._rows += row
            self._ts.append(t)
            self._gaps.append(bool(gap))

    def flush(self) -> int:
        """Copy staged frames into the ring; returns how many."""
        with self._lock:
            raw, ts, gs = self._rows, self._ts, self._gaps
            self._rows, self._ts, self._gaps = bytearray(), [], []
        k = len(ts)
        if not k:
            return 0
        rows = np.frombuffer(raw, np.int16).reshape(k, self.n_ch)
        ts = np.asarray(ts, float)
        gs = np.asarray(gs, bool)
        if k > self.cap:                    # more than fits: only the newest survive
            self.total += k - self.cap
            rows, ts, gs, k = rows[-self.cap:], ts[-self.cap:], gs[-self.cap:], self.cap
        p = self.total % self.cap
        a = min(k, self.cap - p)
        self.data[p:p + a], self.t[p:p + a], self.gap[p:p + a] = rows[:a], ts[:a], gs[:a]
        if k > a:
            self.data[:k - a], self.t[:k - a], self.gap[:k - a] = rows[a:], ts[a:], gs[a:]
        self.total += k
        return k

    # --- output: logical index 0 is the oldest frame held --------------------
    @property
    def count(self) -> int:
        return min(self.total, self.cap)

    def _start(self) -> int:
        return (self.total - self.count) % self.cap

    @property
    def t_oldest(self) -> float:
        return float(self.t[self._start()]) if self.total else 0.0

    @property
    def t_newest(self) -> float:
        return float(self.t[(self.total - 1) % self.cap]) if self.total else 0.0

    def search(self, t: float) -> int:
        """Logical index of the first frame at or after time t (count if none)."""
        n, s = self.count, self._start()
        if s + n <= self.cap:
            return int(np.searchsorted(self.t[s:s + n], t, "left"))
        a = self.t[s:]
        if t <= a[-1]:
            return int(np.searchsorted(a, t, "left"))
        return len(a) + int(np.searchsorted(self.t[:s + n - self.cap], t, "left"))

    def get(self, i0: int, i1: int):
        """(t, data, gap) for logical frames [i0, i1), oldest first.

        Views into the ring when the range does not wrap, copies when it does;
        either way valid only until the next flush().
        """
        i0, i1 = max(0, i0), min(self.count, i1)
        n = i1 - i0
        if n <= 0:
            return np.empty(0), np.empty((0, self.n_ch), np.int16), np.empty(0, bool)
        a = (self._start() + i0) % self.cap
        if a + n <= self.cap:
            return self.t[a:a + n], self.data[a:a + n], self.gap[a:a + n]
        b = a + n - self.cap
        return (np.concatenate([self.t[a:], self.t[:b]]),
                np.concatenate([self.data[a:], self.data[:b]]),
                np.concatenate([self.gap[a:], self.gap[:b]]))

    def take(self, idx):
        """(t, data, gap) at logical indices idx (copies)."""
        p = (self._start() + np.asarray(idx, np.int64)) % self.cap
        return self.t[p], self.data[p], self.gap[p]

    def frame(self, i: int):
        """(t, samples, gap) of logical frame i."""
        p = (self._start() + i) % self.cap
        return float(self.t[p]), self.data[p], bool(self.gap[p])


# ---------------------------------------------------------------------------
# Decimation: frames -> pixel columns
# ---------------------------------------------------------------------------

def col_of(t: float, dt: float) -> int:
    """The column k with k dt <= t < (k+1) dt, computed the way column_stats
    assigns frames (edges are k * dt), so the two never disagree by one."""
    k = math.floor(t / dt)
    if (k + 1) * dt <= t:
        k += 1
    elif k * dt > t:
        k -= 1
    return k


class Columns:
    """Statistics of the frames in absolute columns k0 .. k0+m-1, one row per column.

    has      at least one frame        count   frames
    lo, hi   min and max (int16)       first, last   first and last sample
    tf, tl   time of first and last    gfirst  frames are missing just before `first`
    ginside  frames are missing between two frames of this column
    """
    FIELDS = ("has", "count", "lo", "hi", "first", "last", "tf", "tl", "gfirst", "ginside")

    def __init__(self, k0: int, m: int = 0, n_ch: int = 128, **fields):
        self.k0 = k0
        if fields:
            for f in self.FIELDS:
                setattr(self, f, fields[f])
            return
        self.has, self.count = np.zeros(m, bool), np.zeros(m, np.int64)
        for f in ("lo", "hi", "first", "last"):
            setattr(self, f, np.zeros((m, n_ch), np.int16))
        self.tf, self.tl = np.zeros(m), np.zeros(m)
        self.gfirst, self.ginside = np.zeros(m, bool), np.zeros(m, bool)

    def __len__(self) -> int:
        return len(self.has)

    def part(self, a: int, b: int) -> "Columns":
        """Absolute columns [a, b)."""
        i, j = a - self.k0, b - self.k0
        return Columns(a, **{f: getattr(self, f)[i:j] for f in self.FIELDS})

    @staticmethod
    def join(parts) -> "Columns":
        return Columns(parts[0].k0, **{f: np.concatenate([getattr(p, f) for p in parts])
                                       for f in Columns.FIELDS})


def segment_minmax(data, s, e):
    """Per-segment min and max of data rows [s[j], e[j]) (all non-empty, in order)."""
    cnt = e - s
    kmax = int(cnt.max())
    if kmax == 1:
        return data[s], data[s]
    if kmax * len(s) <= 2 * int(cnt.sum()) + 4096:
        # Even segments (the normal case: a steady sample rate). Gather each into
        # a padded block -- repeating its last row changes neither min nor max --
        # and reduce over the block axis, which numpy vectorizes; reduceat along
        # axis 0 of a (frames, 128) array is ten times slower.
        idx = np.minimum(s[:, None] + np.arange(kmax), (e - 1)[:, None])
        blk = data[idx]
        return blk.min(axis=1), blk.max(axis=1)
    seg = data[s[0]:e[-1]]                  # uneven (sparse snapshots): segments are contiguous
    return np.minimum.reduceat(seg, s - s[0], axis=0), np.maximum.reduceat(seg, s - s[0], axis=0)


def column_stats(t, data, gap, k0: int, m: int, dt: float) -> Columns:
    """Columns k0 .. k0+m-1 of width dt from frames (t ascending; frames outside are ignored)."""
    edges = (k0 + np.arange(m + 1)) * dt
    idx = np.searchsorted(t, edges, "left")
    starts, ends = idx[:-1], idx[1:]
    c = Columns(k0, m, data.shape[1])
    c.count = ends - starts
    c.has = c.count > 0
    if c.has.any():
        s, e = starts[c.has], ends[c.has]
        c.lo[c.has], c.hi[c.has] = segment_minmax(data, s, e)
        c.first[c.has], c.last[c.has] = data[s], data[e - 1]
        c.tf[c.has], c.tl[c.has] = t[s], t[e - 1]
        c.gfirst[c.has] = gap[s]
        cg = np.cumsum(gap)                 # gaps among frames s+1 .. e-1
        c.ginside[c.has] = (cg[e - 1] - cg[s]) > 0
    return c


class Decimator:
    """Columns for a view of a History, recomputing only what may have changed.

    History only grows at the new end, so every column wholly older than the
    newest frame is final. A live view that moved on by a frame's worth of
    time reuses all but its last few columns; a parked view reuses everything.
    """

    def __init__(self):
        self.key = None
        self.cols = None
        self.final = None                   # columns below this were complete when computed

    def _compute(self, hist: History, k0: int, m: int, dt: float) -> Columns:
        i0, i1 = hist.search(k0 * dt), hist.search((k0 + m) * dt)
        t, data, gap = hist.get(i0, i1)
        return column_stats(t, data, gap, k0, m, dt)

    def columns(self, hist: History, k0: int, m: int, dt: float) -> Columns:
        key = (dt, id(hist), hist.n_ch)
        k1 = k0 + m
        parts = []
        c = self.cols
        if self.key == key and c is not None:
            a, b = max(k0, c.k0), min(k1, c.k0 + len(c), self.final)
            if b > a:
                if a > k0:
                    parts.append(self._compute(hist, k0, a - k0, dt))
                parts.append(c.part(a, b))
                if k1 > b:
                    parts.append(self._compute(hist, b, k1 - b, dt))
        if not parts:
            parts = [self._compute(hist, k0, m, dt)]
        cols = parts[0] if len(parts) == 1 else Columns.join(parts)
        self.key, self.cols = key, cols
        self.final = col_of(hist.t_newest, dt) if hist.total else k0
        return cols


def assemble(c: Columns, dt: float, before=None, after=None, t_min: float = None):
    """What to draw in each column: (lo, hi, draw, gapcol, has).

    lo, hi  (m, n_ch) float32, the vertical span of each drawn column, stretched
            to meet the previous column's last value so the trace stays joined.
            Columns with no frame between two frames are filled by linear
            interpolation (zoomed in further than the sample rate).
    draw    (m,) columns that hold a trace.
    gapcol  (m,) columns where frames are missing. Nothing is drawn across a
            gap: the columns it spans stay empty, and a gap narrower than a
            column blanks the column it falls in, so every gap shows as at
            least one empty pixel column at any zoom.
    has     (m,) columns that hold real frames (after t_min).
    before / after: (t, samples, gap) of the nearest frame outside the columns
    on each side (or None), so a zoomed-in trace joins up to the panel edges.
    t_min: columns ending before it (frames the ring has overwritten) are empty.
    """
    m, n_ch = c.lo.shape
    has = c.has.copy()
    if t_min is not None:
        has &= (c.k0 + np.arange(m) + 1) * dt > t_min
    M = m + 2                               # one virtual column on each side
    H = np.zeros(M, bool)
    H[1:-1] = has
    F = np.zeros((M, n_ch), np.float32)
    L, LO, HI = F.copy(), F.copy(), F.copy()
    F[1:-1], L[1:-1], LO[1:-1], HI[1:-1] = c.first, c.last, c.lo, c.hi
    TF, TL = np.zeros(M), np.zeros(M)
    TF[1:-1], TL[1:-1] = c.tf, c.tl
    GF, GI = np.zeros(M, bool), np.zeros(M, bool)
    GF[1:-1], GI[1:-1] = c.gfirst, c.ginside
    for j, nb in ((0, before), (M - 1, after)):
        if nb is not None:
            t, row, g = nb
            H[j] = True
            F[j] = L[j] = LO[j] = HI[j] = row
            TF[j] = TL[j] = t
            GF[j] = g if j == M - 1 else False

    idx = np.arange(M)
    last_ne = np.maximum.accumulate(np.where(H, idx, -1))
    prev = np.concatenate([[-1], last_ne[:-1]])                   # last non-empty before j
    next_ne = np.minimum.accumulate(np.where(H, idx, M)[::-1])[::-1]
    nxt = np.concatenate([next_ne[1:], [M]])                      # first non-empty after j
    inner = ~H & (prev >= 0) & (nxt < M)
    gapped = inner & GF[np.minimum(nxt, M - 1)]
    interp = inner & ~gapped
    blank = H & (GI | (GF & (prev >= 0) & (prev == idx - 1)))

    j = np.flatnonzero(interp)
    if len(j):
        p, q = prev[j], nxt[j]
        tc = (c.k0 - 1 + j + 0.5) * dt
        w = np.clip((tc - TL[p]) / np.maximum(TF[q] - TL[p], 1e-12), 0.0, 1.0)
        v = L[p] + (F[q] - L[p]) * w[:, None].astype(np.float32)
        F[j] = L[j] = LO[j] = HI[j] = v
    valid = H | interp
    link = valid[1:] & valid[:-1] & ~(H[1:] & GF[1:])             # join column j-1 to j
    jj = np.flatnonzero(link) + 1
    LO[jj] = np.minimum(LO[jj], L[jj - 1])
    HI[jj] = np.maximum(HI[jj], L[jj - 1])
    draw = valid & ~blank
    return LO[1:-1], HI[1:-1], draw[1:-1], (gapped | blank)[1:-1], has


# ---------------------------------------------------------------------------
# Time: what is shown, and moving it
# ---------------------------------------------------------------------------
WIDTHS = (0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 60.0, 120.0, 300.0, 600.0)
TICK_STEPS = (0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0,
              15.0, 30.0, 60.0, 120.0, 300.0, 600.0)


@dataclass(frozen=True)
class TimeView:
    """width seconds wide; live (right edge = newest frame) or parked at `right`."""
    width: float
    live: bool = True
    right: float = 0.0


def clamp_right(right: float, width: float, oldest: float, newest: float) -> float:
    """A right edge that keeps the window inside the history (newest if it is shorter)."""
    lo = oldest + width
    if lo >= newest:
        return newest
    return min(max(right, lo), newest)


def right_edge(tv: TimeView, oldest: float, newest: float) -> float:
    return newest if tv.live else clamp_right(tv.right, tv.width, oldest, newest)


def column_range(right: float, width: float, ncols: int):
    """(k0, dt): ncols columns of width/ncols seconds, the last one holding `right`."""
    dt = width / ncols
    return col_of(right, dt) + 1 - ncols, dt


def pan(tv: TimeView, frac: float, oldest: float, newest: float) -> TimeView:
    """Move by frac of a window (negative: back in time); reaching the newest frame is live."""
    r = right_edge(tv, oldest, newest) + frac * tv.width
    if r >= newest:
        return TimeView(tv.width, True, newest)
    return TimeView(tv.width, False, clamp_right(r, tv.width, oldest, newest))


def go_oldest(tv: TimeView, oldest: float, newest: float) -> TimeView:
    return TimeView(tv.width, False, clamp_right(oldest + tv.width, tv.width, oldest, newest))


def go_live(tv: TimeView) -> TimeView:
    return TimeView(tv.width, True, tv.right)


def freeze(tv: TimeView, oldest: float, newest: float) -> TimeView:
    """Park the view where it is now."""
    return TimeView(tv.width, False, right_edge(tv, oldest, newest))


def step_width(width: float, direction: int, max_width: float) -> float:
    """The next window width up (direction > 0) or down the ladder, at most max_width."""
    ladder = sorted({w for w in WIDTHS if w < max_width} | {max_width})
    if direction > 0:
        up = [w for w in ladder if w > width * 1.0001]
        return up[0] if up else width
    down = [w for w in ladder if w < width / 1.0001]
    return down[-1] if down else width


def zoom_time(tv: TimeView, direction: int, oldest: float, newest: float,
              max_width: float) -> TimeView:
    """Wider (direction > 0) or narrower window: live stays live, a parked view keeps its centre."""
    w = step_width(tv.width, direction, max_width)
    if tv.live:
        return TimeView(w, True, tv.right)
    centre = right_edge(tv, oldest, newest) - tv.width / 2
    return TimeView(w, False, clamp_right(centre + w / 2, w, oldest, newest))


def time_ticks(t0: float, t1: float, max_ticks: int = 10):
    """(tick times, step): round times inside [t0, t1], at most max_ticks of them."""
    span = max(t1 - t0, 1e-9)
    step = next((s for s in TICK_STEPS if span / s <= max_ticks), TICK_STEPS[-1])
    first = math.ceil(t0 / step - 1e-9)
    last = math.floor(t1 / step + 1e-9)
    return [k * step for k in range(first, last + 1)], step


def fmt_time(t: float, step: float) -> str:
    """'12.5 s', '3.125 s': as many decimals as the tick step needs."""
    dec = max(0, -math.floor(math.log10(step) + 1e-9))
    return f"{t:.{dec}f} s"


def thumb(t0: float, t1: float, oldest: float, newest: float):
    """(a, b): the part of the scrollbar (0..1) that the visible window [t0, t1] covers."""
    start, end = min(oldest, t0), max(newest, t1)
    span = max(end - start, 1e-9)
    return (t0 - start) / span, (t1 - start) / span


def right_for_thumb(a: float, width: float, oldest: float, newest: float) -> float:
    """The right edge that puts the scrollbar thumb's left end at a (0..1)."""
    end = max(newest, oldest + width)
    return clamp_right(oldest + a * (end - oldest) + width, width, oldest, newest)


# ---------------------------------------------------------------------------
# Channels: which lanes are shown
# ---------------------------------------------------------------------------
LANE_STEPS = (128, 64, 32, 16, 8)


@dataclass(frozen=True)
class LaneView:
    """n lanes, channel `top` in the first one."""
    n: int = 128
    top: int = 0


def lanes(n: int, top: int, n_ch: int = 128) -> LaneView:
    n = max(1, min(int(n), n_ch))
    return LaneView(n, max(0, min(int(top), n_ch - n)))


def lane_scroll(lv: LaneView, delta: int, n_ch: int = 128) -> LaneView:
    return lanes(lv.n, lv.top + delta, n_ch)


def lane_zoom(lv: LaneView, direction: int, anchor: int = None, n_ch: int = 128) -> LaneView:
    """Fewer (direction > 0) or more lanes, showing the block of the new size that holds
    `anchor` (default: the middle lane), so + from 128 lands on whole modules."""
    steps = [s for s in LANE_STEPS if s <= n_ch] or [n_ch]
    if direction > 0:
        n = next((s for s in steps if s < lv.n), lv.n)
    else:
        n = next((s for s in reversed(steps) if s > lv.n), lv.n)
    if anchor is None:
        anchor = lv.top + lv.n // 2
    return lanes(n, (anchor // n) * n, n_ch)


def label_step(lane_pt: float, min_pt: float = 9.0) -> int:
    """Label every channel if a lane is tall enough, else every 2nd, 4th, 8th ..."""
    for s in (1, 2, 4, 8, 16, 32):
        if s * lane_pt >= min_pt:
            return s
    return 32


# ---------------------------------------------------------------------------
# Pixels
# ---------------------------------------------------------------------------


def pack_hex(colors) -> np.ndarray:
    """'#rrggbb' strings -> uint32 RGBA words in the byte order of an RGBA buffer."""
    out = np.empty((len(colors), 4), np.uint8)
    for i, c in enumerate(colors):
        c = c.lstrip("#")
        out[i] = (int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16), 255)
    return out.view(np.uint32)[:, 0].copy()


def seq_lut(anchors, n: int = 256) -> np.ndarray:
    """n packed colours evenly interpolated through `anchors` ('#rrggbb'), the way
    matplotlib's LinearSegmentedColormap.from_list does, so a colorbar built from
    the same anchors matches."""
    rgb = np.array([[int(a.lstrip("#")[i:i + 2], 16) for i in (0, 2, 4)] for a in anchors], float)
    x, xi = np.linspace(0, 1, len(anchors)), np.linspace(0, 1, n)
    out = np.empty((n, 4), np.uint8)
    for k in range(3):
        out[:, k] = np.round(np.interp(xi, x, rgb[:, k]))
    out[:, 3] = 255
    return out.view(np.uint32)[:, 0].copy()


def lane_rows(h: int, n: int):
    """First pixel row of each of n lanes stacked in h rows, plus h at the end."""
    return np.floor(np.arange(n + 1) * (h / n)).astype(np.int64)


def lane_geometry(h: int, n: int):
    """(first rows (n+1,), lane of each pixel row (h,), row within its lane (h,))."""
    r0 = lane_rows(h, n)
    row_lane = np.clip(np.searchsorted(r0, np.arange(h), "right") - 1, 0, n - 1)
    local = np.arange(h) - r0[row_lane]
    dtype = np.int8 if (r0[1:] - r0[:-1]).max() < 127 else np.int16
    return r0, row_lane, local.astype(dtype)


def rasterize_lanes(y_lo, y_hi, draw, h: int, colors, bg, geom=None):
    """An (h, w) uint32 image of n lanes stacked top to bottom.

    y_lo, y_hi (w, n): each column's span in lane units (+0.5 top of the lane,
    -0.5 bottom). draw (w,): columns to draw. colors (n,) uint32; bg uint32.
    Lane i owns pixel rows lane_rows(h, n)[i] .. [i+1]-1 and never paints
    outside them. Each pixel column gets one vertical run from its span's top to
    its bottom. Rows are compared in lane-local int8 coordinates: four
    vectorised passes over the image, whatever the sample rate.
    """
    w, n = y_lo.shape
    if h < n or w == 0:
        return np.full((h, w), bg, np.uint32)
    r0, row_lane, local = geom if geom is not None else lane_geometry(h, n)
    lane_h = h / n
    centres = (np.arange(n) + 0.5) * lane_h - r0[:-1]       # lane-local centre row
    last = r0[1:] - r0[:-1] - 1
    top = np.clip(np.floor(centres - y_hi * lane_h), 0, last).astype(local.dtype)
    bot = np.clip(np.floor(centres - y_lo * lane_h), 0, last).astype(local.dtype)
    top[~np.asarray(draw, bool)] = np.iinfo(local.dtype).max  # nothing in this column
    tops = np.take(top.T, row_lane, axis=0)
    bots = np.take(bot.T, row_lane, axis=0)
    r = local[:, None]
    mask = tops <= r
    mask &= r <= bots
    return np.where(mask, np.asarray(colors, np.uint32)[row_lane][:, None], np.uint32(bg))


def overlay_cols(img, cols, color, bg) -> None:
    """Paint whole pixel columns in `color` wherever they show only background."""
    cols = np.asarray(cols, np.int64)
    cols = cols[(cols >= 0) & (cols < img.shape[1])]
    if len(cols):
        sub = img[:, cols]
        sub[sub == bg] = color
        img[:, cols] = sub


def overlay_rows(img, rows, color, bg) -> None:
    """Paint whole pixel rows in `color` wherever they show only background."""
    rows = np.asarray(rows, np.int64)
    rows = rows[(rows >= 0) & (rows < img.shape[0])]
    if len(rows):
        sub = img[rows]
        sub[sub == bg] = color
        img[rows] = sub


class LanePainter:
    """Paints columns of the trace panel: (h + gm, w) uint32, lanes on top and a
    gm-pixel strip under them that is red wherever frames are missing.

    Under the traces (only where nothing else is drawn): a tint over gap
    columns, the time grid, and faint rules between lanes when they are tall
    enough to need them. Over everything: a darker rule at every module
    boundary (every 32 channels), so the blocks stay apart however busy the
    traces. Columns are independent, so a live view can shift the previous
    image left and paint only its new columns.
    """

    def __init__(self, h: int, w: int, gm: int, lv: LaneView, pal: dict, sep_px: int = 1):
        self.h, self.w, self.gm, self.lv, self.pal = h, w, gm, lv, pal
        self.geom = lane_geometry(h, lv.n) if h >= lv.n else None
        r0 = lane_rows(h, lv.n)
        self.rule_rows = r0[1:-1] if h / lv.n >= 10 else r0[:0]
        first = r0[[i for i in range(1, lv.n) if (lv.top + i) % 32 == 0]]
        self.sep_rows = (first[:, None] + np.arange(sep_px) - sep_px // 2).ravel()

    def paint(self, out, a: int, b: int, y_lo, y_hi, draw, gapc, gridc, colors) -> None:
        """Columns a..b-1 of `out` from per-column arrays that hold just those columns."""
        if b <= a:
            return
        p, bg = self.pal, self.pal["bg"]
        main = rasterize_lanes(y_lo, y_hi, draw, self.h, colors, bg, self.geom)
        overlay_cols(main, np.flatnonzero(gapc), p["gap_tint"], bg)
        overlay_cols(main, np.flatnonzero(gridc), p["grid"], bg)
        overlay_rows(main, self.rule_rows, p["rule"], bg)
        rows = self.sep_rows[(self.sep_rows >= 0) & (self.sep_rows < self.h)]
        main[rows] = p["sep"]
        out[:self.h, a:b] = main
        strip = out[self.h:, a:b]
        strip[...] = bg
        strip[:, np.asarray(gapc, bool)] = p["gap_mark"]


class MapPainter:
    """The activity strip as an image: rows x cols tiles with 1-pixel gaps, a red
    inner border on tiles at the ADC rail, and a dark outline around the
    channels the trace panel shows (when it shows fewer than all)."""

    def __init__(self, h: int, w: int, pal: dict, rows: int = 4, cols: int = 32, border: int = 2):
        self.h, self.w, self.pal = h, w, pal
        ry, rx = lane_rows(h, rows), lane_rows(w, cols)
        self.row_of = np.clip(np.searchsorted(ry, np.arange(h), "right") - 1, 0, rows - 1)
        self.col_of = np.clip(np.searchsorted(rx, np.arange(w), "right") - 1, 0, cols - 1)
        ly, lx = np.arange(h) - ry[self.row_of], np.arange(w) - rx[self.col_of]
        th, tw = (ry[1:] - ry[:-1])[self.row_of], (rx[1:] - rx[:-1])[self.col_of]
        self.gap_y, self.gap_x = (ly == 0) & (self.row_of > 0), (lx == 0) & (self.col_of > 0)
        lo_y, lo_x = np.where(self.row_of > 0, 1, 0), np.where(self.col_of > 0, 1, 0)
        self.edge_y = (ly < lo_y + border) | (ly >= th - border)
        self.left_x, self.right_x = lx < lo_x + border, lx >= tw - border

    def paint(self, tiles, railed, visible=None):
        """tiles (rows, cols) uint32 colours; railed, visible (rows, cols) bool."""
        p = self.pal
        img = tiles[self.row_of][:, self.col_of]
        rl = railed[self.row_of][:, self.col_of]
        img[rl & (self.edge_y[:, None] | self.left_x | self.right_x)] = p["rail"]
        if visible is not None:
            v = np.asarray(visible, bool)
            first = v & ~np.pad(v, ((0, 0), (1, 0)))[:, :-1]
            last = v & ~np.pad(v, ((0, 0), (0, 1)))[:, 1:]
            on = v[self.row_of][:, self.col_of]
            edge = (self.edge_y[:, None] | (first[self.row_of][:, self.col_of] & self.left_x)
                    | (last[self.row_of][:, self.col_of] & self.right_x))
            img[on & edge] = p["outline"]
        img[self.gap_y] = p["bg"]
        img[:, self.gap_x] = p["bg"]
        return img


def tile_colors(rms_uv, dead, lut, dead_color, lo_uv: float = 1.0, hi_uv: float = 10_000.0):
    """Each channel's RMS on a log scale lo_uv..hi_uv through `lut`; dead channels grey."""
    v = np.log10(np.clip(np.asarray(rms_uv, float), lo_uv, hi_uv))
    f = (v - math.log10(lo_uv)) / (math.log10(hi_uv) - math.log10(lo_uv))
    out = lut[np.round(f * (len(lut) - 1)).astype(np.int64)]
    return np.where(np.asarray(dead, bool), np.uint32(dead_color), out)
