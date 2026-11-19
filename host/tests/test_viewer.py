"""Viewer plumbing: datagrams in, per-channel buffers, statistics and loss out.

No matplotlib here: drawing is the only part that needs it, and it is kept apart.
"""
import socket
import sys
import time

import pytest

from chain import bringup as B
from chain import contract as C
from chain import diag as D
from chain import fakeboard as FB
from chain import frame as F
from chain import viewer as V
from chain.ctrl import Ctrl

PERIOD = 48000   # fast clocks per sample at 2 kS/s


def value(seq, ch):
    return seq * 100 + ch


def dgram(first_seq, values=value, t_cap0=0):
    frames = [F.build_frame(s, [values(s, ch) for ch in range(C.N_CH)],
                            t_cap=(t_cap0 + s * PERIOD) & 0xFFFFFFFF)
              for s in range(first_seq, first_seq + C.FRAMES_PER_DGRAM)]
    return F.build_datagram(frames)


def test_datagrams_fill_per_channel_buffers_in_channel_order():
    buf = V.Buffers()
    for d in range(3):
        assert buf.push_datagram(dgram(5 * d), host_t=d * 0.01) == 5
    assert buf.frames == 15 and buf.gaps == 0 and buf.lost == 0
    assert buf.channel(0) == [value(s, 0) for s in range(15)]
    assert buf.channel(127) == [value(s, 127) for s in range(15)]
    assert list(buf.t)[:3] == pytest.approx([0.0, 0.0005, 0.001])   # from t_cap, not arrival
    assert len(buf.raw()) == 15 * C.N_CH * 2


def test_time_survives_the_t_cap_wrap():
    buf = V.Buffers()
    buf.push_datagram(dgram(0, t_cap0=0xFFFFFFFF - 2 * PERIOD), 0.0)
    t = list(buf.t)
    assert all(b - a == pytest.approx(PERIOD / C.SYS_CLK_HZ) for a, b in zip(t, t[1:]))


def test_a_missing_datagram_is_counted_as_loss():
    buf = V.Buffers()
    for first in (0, 5, 15):               # 10..14 never arrive
        buf.push_datagram(dgram(first), 0.0)
    assert (buf.frames, buf.gaps, buf.lost) == (15, 1, 5)
    assert buf.loss_pct() == pytest.approx(25.0)


def test_duplicates_late_frames_and_garbage_are_counted_not_stored():
    buf = V.Buffers()
    buf.push_datagram(dgram(0), 0.0)
    buf.push_datagram(dgram(10), 0.0)
    buf.push_datagram(dgram(10), 0.0)      # duplicate
    buf.push_datagram(dgram(5), 0.0)       # arrives after 10..14: older than drawn
    assert buf.duplicates == 5 and buf.reordered == 5 and buf.frames == 10
    assert buf.lost == 0                   # the late ones did arrive
    assert buf.push_datagram(dgram(15)[:-3], 0.0) == 0
    assert buf.malformed == 1


def test_seq_wrap_is_not_a_gap():
    buf = V.Buffers()
    buf.push(0xFFFFFFFF, [0] * C.N_CH, 0.0)
    buf.push(0, [0] * C.N_CH, 0.001)
    assert buf.gaps == 0 and buf.frames == 2


def test_the_window_rolls():
    buf = V.Buffers(depth=7)
    for d in range(4):
        buf.push_datagram(dgram(5 * d), 0.0)
    assert len(buf.rows) == 7 and buf.channel(3) == [value(s, 3) for s in range(13, 20)]


def test_stats_in_microvolts_with_dead_and_railed_flags():
    buf = V.Buffers(n_ch=4)
    for i in range(10):
        buf.push(i, [100 if i % 2 else -100, 42, 32767, i], float(i))
    s = buf.stats()
    assert s[0].mean_uv == pytest.approx(0.0) and s[0].rms_uv == pytest.approx(19.5)
    assert s[1].dead and s[1].flag == "dead" and s[1].rms_uv == 0.0
    assert s[2].railed and s[2].flag == "railed"
    assert not s[3].dead and s[3].flag == "" and s[3].max_uv == pytest.approx(9 * 0.195)
    assert "railed: 2" in V.flagged_summary(s) and "dead: 1" in V.flagged_summary(s)


def test_frame_rate_and_status_line():
    buf = V.Buffers()
    for d in range(20):
        buf.push_datagram(dgram(5 * d), host_t=d * 0.0025)    # 2000 frames/s
    assert buf.frame_rate(now=0.05) == pytest.approx(2000, rel=0.05)
    link = D.LinkStat(checks=1000, errors=(0, 3, 0, 0), last=(0x35, 0x3A, 0x35, 0x3A))
    line = V.status_line(buf, 0.05, "udp", link)
    assert "seq gaps 0 (0 lost" in line and "malformed 0" in line
    assert "LINK_ERR c0A 0 c0B 3 c1A 0 c1B 0 / 1000 checks" in line
    assert "LINK_ERR" not in V.status_line(buf, 0.05, "udp")


class ListSource:
    def __init__(self, dgrams, clock):
        self.dgrams, self.clock = list(dgrams), clock

    def poll(self, buf):
        self.clock.sleep(0.25)
        if not self.dgrams:
            return 0
        return buf.push_datagram(self.dgrams.pop(0), self.clock())

    def close(self):
        pass


def test_headless_prints_status_and_every_channel():
    clock = FB.VirtualClock()
    out = []
    buf = V.Buffers()
    stats = V.run_headless(ListSource([dgram(0), dgram(5), dgram(15)], clock), buf, 2.0,
                           out=out.append, clock=clock, sleep=clock.sleep)
    text = "\n".join(out)
    assert len(stats) == C.N_CH and buf.frames == 15 and buf.lost == 5
    assert "seq gaps 1 (5 lost" in text
    assert sum(1 for l in out[-2].splitlines() if l.strip()[:1].isdigit()) == C.N_CH
    assert "matplotlib" not in sys.modules


def test_uart_source_reads_the_chips_through_the_diagnostic_list():
    def signal(chip, module, c, n):
        return 64 * chip + (32 if module == "B" else 0) + c + 1000 * (n % 2)

    clock = FB.VirtualClock()
    board = FB.FakeBoard(clock=clock, signal=signal)
    ctrl = Ctrl(ser=board)
    B.prepare(ctrl, B.Options(resync_settle=0), out=lambda *_: None, sleep=clock.sleep,
              clock=clock)
    src = V.UartSource(D.Diag(ctrl), clock=clock)
    buf = V.Buffers()
    for _ in range(3):
        src.poll(buf)
    assert buf.frames == 3 and buf.lost == 0
    first = [buf.channel(ch)[0] % 1000 for ch in range(C.N_CH)]
    assert first == list(range(C.N_CH))


def test_link_poller_reads_the_counters_while_streaming():
    clock = FB.VirtualClock()
    board = FB.FakeBoard(FB.make_faults("chip1_absent"), clock=clock)
    ctrl = Ctrl(ser=board)
    B.prepare(ctrl, B.Options(resync_settle=0), out=lambda *_: None, sleep=clock.sleep,
              clock=clock, stream=True)
    lp = V.LinkPoller(ctrl, interval=1.0, clock=clock)
    clock.sleep(0.5)
    first = lp.poll()
    assert first.checks > 0 and first.errors[0] == 0 and first.errors[2] > 0
    assert lp.poll() is first                       # not again within the interval
    clock.sleep(1.0)
    assert lp.poll().checks > first.checks


def test_udp_source_receives_on_a_thread():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    port = rx.getsockname()[1]
    src = V.UdpSource(sock=rx)
    try:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for d in range(3):
            tx.sendto(dgram(5 * d), ("127.0.0.1", port))
        tx.close()
        buf = V.Buffers()
        deadline = time.monotonic() + 2.0
        while buf.frames < 15 and time.monotonic() < deadline:
            src.poll(buf)
            time.sleep(0.01)
    finally:
        src.close()
    assert buf.frames == 15 and buf.lost == 0


def _poll_until(src, buf, frames, timeout=2.0):
    deadline = time.monotonic() + timeout
    while buf.frames < frames and time.monotonic() < deadline:
        src.poll(buf)
        time.sleep(0.01)


def test_tcp_source_cuts_the_stream_into_datagrams():
    # TCP has no message boundaries: send three datagrams in chunks that split
    # headers and payloads mid-way, and expect exactly three back.
    src = V.TcpSource(port=0, bind="127.0.0.1")
    try:
        tx = socket.create_connection(("127.0.0.1", src.port))
        stream = b"".join(dgram(5 * d) for d in range(3))
        for i in range(0, len(stream), 700):
            tx.sendall(stream[i:i + 700])
            time.sleep(0.005)
        buf = V.Buffers()
        _poll_until(src, buf, 15)
        tx.close()
    finally:
        src.close()
    assert buf.frames == 15 and buf.lost == 0 and buf.malformed == 0
    assert src.connections == 1


def test_tcp_source_a_new_connection_replaces_the_old_one():
    # The node rebooted mid-datagram: the old connection is never closed, a new
    # one arrives. The half datagram must be dropped, not glued onto the new
    # stream (that would misalign every record after it).
    src = V.TcpSource(port=0, bind="127.0.0.1")
    try:
        old = socket.create_connection(("127.0.0.1", src.port))
        old.sendall(dgram(0) + dgram(5)[:600])
        buf = V.Buffers()
        _poll_until(src, buf, 5)
        new = socket.create_connection(("127.0.0.1", src.port))
        new.sendall(dgram(10) + dgram(15))
        _poll_until(src, buf, 15)
        state = src.state()
        old.close()
        new.close()
    finally:
        src.close()
    assert buf.frames == 15 and buf.malformed == 0
    assert src.connections == 2 and src.partial_drops == 1
    assert state == "connected to 127.0.0.1"


def test_tcp_status_line_says_tcp_and_counts_malformed():
    line = V.status_line(V.Buffers(), 0.0, "tcp")
    assert line.startswith("TCP") and "fr/s" in line and "malformed 0" in line


def test_cli_argument_rules():
    for argv in ([], ["--uart"], ["--udp", "--start"], ["--udp", "--uart"],
                 ["--tcp", "--start"], ["--tcp", "--udp"]):
        with pytest.raises(SystemExit):
            V.main(argv, out=lambda *_: None)


def test_cli_headless_udp_runs_without_a_display():
    out = []
    assert V.main(["--udp", "--udp-port", "0", "--headless", "0.3"], out=out.append) == 0
    assert any("frames 0" in l for l in out)
    assert "matplotlib" not in sys.modules


def test_cli_headless_tcp_runs_and_says_it_is_waiting():
    out = []
    assert V.main(["--tcp", "--port", "0", "--headless", "0.3"], out=out.append) == 0
    assert any(l.startswith("TCP") and "waiting" in l for l in out)


# --- the activity strip and the lanes ---------------------------------------

def test_grid_puts_each_module_in_one_row_and_round_trips():
    seen = set()
    for ch in range(128):
        r, c = V.grid_pos(ch)
        assert 0 <= r < V.GRID_ROWS and 0 <= c < V.GRID_COLS
        assert r == ch // 32                      # module m owns row m of the strip
        assert V.grid_channel(r, c) == ch
        seen.add((r, c))
    assert len(seen) == 128


def test_fmt_uv_picks_a_readable_unit():
    assert V.fmt_uv(2.34) == "2.3 µV"
    assert V.fmt_uv(850) == "850 µV"
    assert V.fmt_uv(1830) == "1.8 mV"
    assert V.fmt_uv(12_400) == "12 mV"


def test_lanes_never_cross_and_say_what_a_lane_means():
    np = pytest.importorskip("numpy")
    rng = np.random.default_rng(1)
    uv = rng.normal(0, 1, (500, 4)) * np.array([3.0, 300.0, 3000.0, 0.0]) + 500.0
    y, half = V.lane_values(uv)
    assert np.all(np.abs(y) <= V.LANE_CLIP)            # nothing leaves its lane
    assert half[2] > half[1] > half[0]                       # labels track each channel
    assert np.all(y[:, 3] == 0)                              # a flat channel stays flat
    assert half[3] >= V.MIN_HALF_UV                     # and is not magnified
    # each live channel uses its lane: its excursion reaches near LANE_FILL
    for ch in range(3):
        assert 0.3 < np.percentile(np.abs(y[:, ch]), 99) <= V.LANE_CLIP


def test_shared_scale_uses_one_half_range_for_every_lane():
    np = pytest.importorskip("numpy")
    uv = np.outer(np.sin(np.linspace(0, 20, 400)), [10.0, 100.0, 1000.0])
    y, half = V.lane_values(uv, shared_half=100.0)
    assert np.allclose(half, half[0])
    assert np.abs(y[:, 0]).max() < np.abs(y[:, 1]).max()     # small stays small
    assert np.abs(y[:, 2]).max() == pytest.approx(V.LANE_CLIP)   # big clips at the edge


# --- speed: the window, decimation, rasterized lanes, the acquisition thread --

def test_buffers_window_returns_only_the_newest_seconds():
    np = pytest.importorskip("numpy")
    buf = V.Buffers(depth=100)
    for k in range(50):
        buf.push(k, [k] * 128, k * 0.1)
    t, data = buf.window(1.0)
    assert t[0] == pytest.approx(3.9) and t[-1] == pytest.approx(4.9)
    assert data.shape == (len(t), 128) and data[0, 0] == 39 and data[-1, 127] == 49


def test_columns_keep_every_extreme_and_the_time_order():
    # 2 s at 2 kS/s decimated to 300 pixel columns: a one-sample spike must
    # still reach its full height, in the column of the time it happened.
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    t = np.arange(4000) * 0.0005
    data = np.zeros((4000, 2), np.int16)
    data[1234, 0], data[3001, 1] = 500, -700
    c = S.column_stats(t, data, np.zeros(4000, bool), 0, 300, 2.0 / 300)
    assert c.has.all() and c.count.sum() == 4000
    assert c.hi[:, 0].max() == 500 and c.lo[:, 1].min() == -700
    assert int(np.argmax(c.hi[:, 0])) == S.col_of(t[1234], 2.0 / 300)
    assert np.all(np.diff(c.tf) > 0) and np.all(c.tl >= c.tf)
    sparse = S.column_stats(t[:10], data[:10], np.zeros(10, bool), 0, 300, 2.0 / 300)
    assert sparse.count.sum() == 10 and (~sparse.has).sum() > 280   # nothing invented


def test_rasterize_lanes_draws_a_spike_full_height_and_a_flat_line_thin():
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    w, h = 50, 42                                   # two lanes of 21 rows
    y_lo, y_hi = np.zeros((w, 2), np.float32), np.zeros((w, 2), np.float32)
    y_hi[20, 0], y_lo[20, 0] = 0.48, -0.48          # lane 0 spans its height in column 20
    colors = np.array([7, 9], np.uint32)
    img = S.rasterize_lanes(y_lo, y_hi, np.ones(w, bool), h, colors, 0)
    assert img.shape == (h, w)
    assert set(np.unique(img[:21])) == {0, 7} and set(np.unique(img[21:])) == {0, 9}
    assert (img[:21, 20] == 7).sum() > 15           # the spike is a tall run
    assert (img[21:] == 9).sum(axis=0).max() == 1   # flat: one pixel per column
    draw = np.ones(w, bool)
    draw[5] = False
    assert not S.rasterize_lanes(y_lo, y_hi, draw, h, colors, 0)[:, 5].any()   # a break


def test_rasterized_lanes_stay_on_their_own_rows():
    # Every value clipped to the lane edges: lane i may only ever touch its own rows.
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    n, h, w = 5, 37, 8
    y = np.full((w, n), 0.5, np.float32)
    img = S.rasterize_lanes(-y, y, np.ones(w, bool), h, np.arange(1, n + 1, dtype=np.uint32), 0)
    r = S.lane_rows(h, n)
    for i in range(n):
        assert set(np.unique(img[r[i]:r[i + 1]])) == {i + 1}


def test_acquirer_publishes_snapshots_off_the_drawing_thread():
    pytest.importorskip("numpy")

    class Counting:
        def __init__(self):
            self.k = 0

        def poll(self, buf):
            buf.push(self.k, [self.k % 7] * 128, self.k * 0.001)
            self.k += 1
            return 1

        def close(self):
            pass

    acq = V.Acquirer(Counting(), V.Buffers(depth=500), "uart", window_s=0.2).start()
    try:
        deadline = time.monotonic() + 5
        while acq.latest() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        snap = acq.latest()
        assert snap is not None and snap.n > 0 and snap.lo.shape[1] == 128
        assert snap.stat(0).max_uv >= snap.stat(0).min_uv
    finally:
        acq.stop()
    assert not acq.th.is_alive() and not acq.th_render.is_alive()


# --- scrolling: the history ring, views, decimation, breaks, the render thread --

def history_of(values, lost=(), rate=2000.0, cap=None, n_ch=2):
    """A History fed through Buffers: frame seq k carries values[k] (ch 0) and
    -values[k] (ch 1); seqs in `lost` never arrive."""
    from chain import scroll as S
    buf = V.Buffers(n_ch=n_ch)
    hist = S.History(n_ch, cap or len(values) + 8)
    buf.history = hist
    for k, v in enumerate(values):
        if k not in lost:
            buf.push(k, [int(v), -int(v)][:n_ch], k / rate)
    hist.flush()
    return buf, hist


def test_buffers_feed_the_history_with_a_break_where_frames_were_lost():
    pytest.importorskip("numpy")
    buf, hist = history_of(range(20), lost=set(range(10, 15)))
    assert (buf.frames, buf.lost) == (15, 5) and hist.count == 15
    t, data, gap = hist.get(0, hist.count)
    assert list(data[:, 0]) == [k for k in range(20) if not 10 <= k < 15]
    assert list(gap.nonzero()[0]) == [10]               # frame 15, the first after the hole
    assert t[10] == pytest.approx(15 / 2000)            # time keeps the hole's width


def test_history_ring_wraps_and_keeps_order():
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    h = S.History(1, capacity=10)
    for k in range(7):
        h.append(np.array([k], np.int16).tobytes(), k * 0.5)
    h.flush()
    for k in range(7, 25):                              # wraps the ring, twice over
        h.append(np.array([k], np.int16).tobytes(), k * 0.5)
        if k % 4 == 0:
            h.flush()
    h.flush()
    assert (h.total, h.count) == (25, 10)
    assert (h.t_oldest, h.t_newest) == (7.5, 12.0)
    t, data, _ = h.get(0, 10)                           # spans the physical wrap
    assert list(data[:, 0]) == list(range(15, 25)) and np.all(np.diff(t) > 0)
    assert h.search(9.0) == 3 and h.search(0.0) == 0 and h.search(99.0) == 10
    assert list(h.get(2, 6)[1][:, 0]) == [17, 18, 19, 20]
    assert list(h.take([0, 9])[1][:, 0]) == [15, 24] and h.frame(4)[1][0] == 19
    big = S.History(1, capacity=4)                      # more staged than fits
    for k in range(9):
        big.append(np.array([k], np.int16).tobytes(), float(k))
    big.flush()
    assert list(big.get(0, 4)[1][:, 0]) == [5, 6, 7, 8] and big.total == 9


def test_history_breaks_on_time_jumps_and_time_going_backwards():
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    h = S.History(1, 10, gap_s=0.05)
    for t in (100.0, 100.001, 100.2, 100.1, 100.201):   # jump, then a step back
        h.append(np.zeros(1, np.int16).tobytes(), t)
    h.flush()
    t, _, gap = h.get(0, 5)
    assert t[0] == 0.0                                  # seconds since the first frame
    assert list(gap) == [False, False, True, True, False]
    assert np.all(np.diff(t) >= 0)


def test_a_parked_view_shows_its_own_time_and_stays_put():
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    _, hist = history_of(np.arange(20_000) % 30_000)    # 10 s; ch 0 = frame number
    r = V.Renderer()
    req = V.Request(1, S.TimeView(2.0, False, 6.0), S.LaneView(2, 0), w=400)
    snap = r.render(hist, req)
    assert not snap.live and snap.t0 == pytest.approx(4.0, abs=0.006)
    assert snap.t1 == pytest.approx(6.0, abs=0.006)
    k = V.R.SCALE_UV
    assert snap.mn[0] == pytest.approx(int(round(snap.t0 * 2000)) * k, abs=2 * k)
    assert snap.mx[0] == pytest.approx(int(snap.t1 * 2000) * k, abs=2 * k)
    assert np.all(np.diff(snap.hi[snap.draw, 0]) >= 0)  # a ramp stays a ramp
    # more data arrives: the parked view does not move, the history extent does
    buf2, hist2 = history_of(np.arange(24_000) % 30_000)
    snap2 = V.Renderer().render(hist2, req)
    assert (snap2.t0, snap2.t1) == (snap.t0, snap.t1) and snap2.newest > snap.newest
    assert np.array_equal(snap2.hi[snap2.draw], snap.hi[snap.draw])
    live = V.Renderer().render(hist2, V.Request(2, S.TimeView(2.0), S.LaneView(2, 0), w=400))
    assert live.live and live.t0 <= live.newest - 2.0 + 0.006 and live.t1 > live.newest


def test_lost_frames_break_the_trace_at_every_zoom():
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    values = np.where(np.arange(4000) < 2000, 1000, -1000)   # a step, right at the hole
    _, hist = history_of(values, lost=set(range(1995, 2005)))
    t_lo, t_hi = 1994 / 2000, 2005 / 2000                    # last frame before, first after
    for width, ncols in ((2.0, 3000), (2.0, 600), (2.0, 50), (0.05, 1000)):
        right = 1.02 if width < 1 else 2.0
        k0, dt = S.column_range(right - 1e-9, width, ncols)
        c = S.Decimator().columns(hist, k0, ncols, dt)
        i0, i1 = hist.search(k0 * dt), hist.search((k0 + ncols) * dt)
        before = hist.frame(i0 - 1) if i0 > 0 else None
        after = hist.frame(i1) if i1 < hist.count else None
        lo, hi, draw, gapcol, _ = S.assemble(c, dt, before, after)
        k = np.arange(ncols) + k0
        inside = ((k + 1) * dt > t_lo) & (k * dt <= t_hi)    # columns that see the hole
        assert gapcol.any() and gapcol[~inside].sum() == 0, (width, ncols)
        # never a line across the hole: no drawn column spans both sides of the step
        spans_both = draw & (lo[:, 0] < 0) & (hi[:, 0] > 0)
        assert not spans_both.any(), (width, ncols)
        # and the break is visible: at least one empty column inside the hole's columns
        assert (~draw[inside]).any() or not inside.any(), (width, ncols)
        # everywhere else, every column with time in the history is drawn
        assert draw[(~inside) & (k * dt >= 0) & ((k + 1) * dt <= 3999 / 2000)].all()


def test_zoomed_past_the_sample_rate_the_trace_is_joined_by_interpolation():
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    _, hist = history_of(np.arange(400) * 10)                # a ramp, 0.2 s
    k0, dt = S.column_range(0.1, 0.01, 200)                  # 50 us columns, 500 us samples
    c = S.Decimator().columns(hist, k0, 200, dt)
    assert c.has.sum() < 30                                  # mostly empty columns
    i0, i1 = hist.search(k0 * dt), hist.search((k0 + 200) * dt)
    lo, hi, draw, gapcol, _ = S.assemble(c, dt, hist.frame(i0 - 1), hist.frame(i1))
    assert draw.all() and not gapcol.any()
    mid = (lo[:, 0] + hi[:, 0]) / 2
    assert np.all(np.diff(mid) >= 0)                         # a straight, rising line
    assert np.all(hi[1:, 0] >= lo[:-1, 0] - 1e-3)            # each column meets the last


def test_the_decimator_reuses_columns_and_matches_a_full_computation():
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    rng = np.random.default_rng(3)
    vals = rng.integers(-3000, 3000, 6000)
    lost = set(range(2500, 2505)) | set(range(4100, 4300))
    buf = V.Buffers(n_ch=2)
    hist = S.History(2, 5000)                                # wraps: 6000 frames in
    buf.history = hist
    dec = S.Decimator()
    for k, v in enumerate(vals):
        if k not in lost:
            buf.push(k, [int(v), int(-v)], k / 2000)
        if k % 97 == 0 and k:
            hist.flush()
            k0, dt = S.column_range(hist.t_newest, 1.0, 333)
            got = dec.columns(hist, k0, 333, dt)
            want = S.Decimator().columns(hist, k0, 333, dt)
            for f in S.Columns.FIELDS:
                assert np.array_equal(getattr(got, f), getattr(want, f)), (k, f)


def test_live_frames_repaint_only_new_columns_and_match_a_full_repaint():
    np = pytest.importorskip("numpy")
    from chain import scroll as S
    rng = np.random.default_rng(5)
    vals = rng.integers(-2000, 2000, 9000)
    buf = V.Buffers(n_ch=128)
    hist = S.History(128, 20_000)
    buf.history = hist
    r = V.Renderer()
    req = V.Request(1, S.TimeView(1.0), S.LaneView(32, 32), w=300, h=160, gm=3,
                    map_w=64, map_h=8)
    lost = set(range(8500, 8505))                            # in the last window
    shifted = 0
    for k, v in enumerate(vals):
        if k not in lost:
            buf.push(k, [int(v)] * 64 + [int(-v)] * 64, k / 2000)
        if k >= 2000 and k % 66 == 0:
            prev = r.prev
            snap = r.render(hist, req, now=0.0)   # a constant clock: one lane scale throughout
            ref = V.Renderer()                    # the same request painted from scratch
            ref._scale, ref._scale_key, ref._scale_t = r._scale, r._scale_key, r._scale_t
            full = ref.render(hist, req, now=0.0)
            assert np.array_equal(snap.raster, full.raster), k
            assert snap.raster.shape == (163, 300) and snap.map_raster.shape == (8, 64)
            shifted += prev is not None and prev.raster is not None and snap.k0 > prev.k0
    assert shifted > 50                                      # the shift-and-paint path ran
    assert (snap.raster[160:] == r.pal["gap_mark"]).any()    # the lost datagram, marked


def test_time_view_pans_zooms_and_returns_to_live():
    from chain import scroll as S
    old, new = 10.0, 70.0
    tv = S.TimeView(2.0)
    back = S.pan(tv, -0.25, old, new)
    assert not back.live and back.right == pytest.approx(69.5)
    assert S.pan(back, 0.25, old, new).live                  # reaching the newest is live
    assert S.go_oldest(tv, old, new).right == pytest.approx(12.0)
    assert S.pan(S.go_oldest(tv, old, new), -1, old, new).right == pytest.approx(12.0)
    assert S.freeze(tv, old, new) == S.TimeView(2.0, False, 70.0)
    assert S.go_live(back).live
    z = S.zoom_time(back, 1, old, new, 60.0)
    assert z.width == 5.0 and not z.live and z.right == 70.0  # wider, but not past the newest
    z = S.zoom_time(S.TimeView(2.0, False, 40.0), 1, old, new, 60.0)
    assert z.right - z.width / 2 == pytest.approx(39.0)       # a parked view keeps its centre
    assert S.zoom_time(tv, -1, old, new, 60.0).width == 1.0
    assert S.step_width(60.0, 1, 60.0) == 60.0 and S.step_width(45.0, 1, 45.0) == 45.0
    assert S.step_width(30.0, 1, 45.0) == 45.0
    assert S.clamp_right(5.0, 2.0, 10.0, 11.0) == 11.0       # history shorter than the window
    k0, dt = S.column_range(70.0, 2.0, 1000)
    assert dt == pytest.approx(0.002) and k0 * dt <= 68.0 + dt and (k0 + 1000) * dt > 70.0


def test_lane_view_scrolls_and_zooms_onto_whole_blocks():
    from chain import scroll as S
    lv = S.LaneView()
    assert S.lane_scroll(lv, 8) == lv                        # all 128 shown: nowhere to go
    z = S.lane_zoom(lv, 1, anchor=40)
    assert z == S.LaneView(64, 0)
    z = S.lane_zoom(z, 1, anchor=40)
    assert z == S.LaneView(32, 32)                           # the module holding ch 40
    assert S.lane_scroll(z, 100) == S.LaneView(32, 96)       # clamped at the end
    assert S.lane_zoom(S.LaneView(32, 40), -1) == S.LaneView(64, 0)
    assert S.lane_zoom(S.LaneView(8, 0), 1) == S.LaneView(8, 0)
    assert S.label_step(4.2) == 4 and S.label_step(8.4) == 2 and S.label_step(17) == 1


def test_scrollbar_thumb_and_its_inverse():
    from chain import scroll as S
    a, b = S.thumb(30.0, 32.0, 10.0, 70.0)
    assert (a, b) == (pytest.approx(1 / 3), pytest.approx(22 / 60))
    assert S.right_for_thumb(a, 2.0, 10.0, 70.0) == pytest.approx(32.0)
    assert S.right_for_thumb(1.0, 2.0, 10.0, 70.0) == 70.0   # dragged to the end: newest
    assert S.right_for_thumb(-1.0, 2.0, 10.0, 70.0) == 12.0


def test_time_ticks_are_round_and_inside_the_window():
    from chain import scroll as S
    ticks, step = S.time_ticks(41.37, 43.37, 10)
    assert step == 0.2 and ticks[0] == pytest.approx(41.4) and ticks[-1] <= 43.37
    assert S.fmt_time(ticks[0], step) == "41.4 s"
    ticks, step = S.time_ticks(0.0, 60.0, 8)
    assert step == 10.0 and S.fmt_time(30.0, step) == "30 s"
    assert S.fmt_time(1.25, 0.05) == "1.25 s"


def test_cli_history_option():
    a = V.build_parser().parse_args(["--udp", "--history", "120", "--window", "5"])
    assert (a.history, a.window) == (120.0, 5.0)
    assert V.build_parser().parse_args(["--tcp"]).history == 60.0
    with pytest.raises(SystemExit):
        V.main(["--udp", "--port", "0", "--history", "0", "--headless", "0.1"],
               out=lambda *_: None)


def test_scope_draws_and_answers_keys_off_screen():
    # The real window on the Agg backend: no display, no render thread.
    np = pytest.importorskip("numpy")
    mpl = pytest.importorskip("matplotlib")
    mpl.use("Agg", force=True)
    from types import SimpleNamespace
    buf = V.Buffers(depth=4000)
    acq = V.Acquirer(None, buf, "udp", window_s=1.0, history_s=10.0)
    rng = np.random.default_rng(0)
    for k in range(6000):
        if not 4000 <= k < 4005:
            buf.push(k, list(rng.integers(-500, 500, 128) * (1 + np.arange(128) % 4)), k / 2000)
    sc = V.Scope(acq, "udp", window_s=1.0, history_s=10.0)
    try:
        snap = sc.sync()
        w, h, gm, mw, mh = sc._geom
        assert snap.raster.shape == (h + gm, w) and snap.map_raster.shape == (mh, mw)
        pal = V.palette()
        assert (snap.raster == pal["trace"]).sum() > 128 * w // 4      # every lane has a trace
        assert len(sc._ylabels) == 32                                   # every 4th of 128
        r0, r1, c0, c1 = sc._rect
        shown = np.asarray(sc.fig.canvas.buffer_rgba()).view(np.uint32)[..., 0][r0:r1, c0:c1]
        assert np.array_equal(shown, snap.raster)                       # what is on the canvas
        key = lambda k: sc._key(SimpleNamespace(key=k))                 # noqa: E731
        key("left")
        assert not sc.tv.live and sc.tv.right < snap.newest
        key("l")
        assert sc.tv.live
        key("+")
        assert sc.lv.n == 64
        key("2")
        assert (sc.lv.n, sc.lv.top) == (32, 32)                        # module chip0 B
        key("]")
        assert sc.tv.width == 2.0
        wheel = lambda st: sc._wheel(SimpleNamespace(inaxes=sc.tax, step=st))   # noqa: E731
        wheel(-0.5)
        assert sc.lv.top == 32                                          # half a notch: nothing yet
        wheel(-0.5)
        assert sc.lv.top == 36                                          # a notch: n // 8 lanes
        b = sc.sax.bbox
        at = SimpleNamespace(button=1, inaxes=sc.sax, x=b.x0 + 0.05 * b.width, y=b.y0 + 1)
        sc._press(at)                                                   # click near the start
        sc._release(at)
        assert not sc.tv.live and sc.tv.right == pytest.approx(sc.snap.oldest + 2.0)  # the start
        key("end")
        assert sc.tv.live
        key("s")
        snap = sc.sync()
        assert snap.lv.n == 32 and snap.shared and snap.width == 2.0
        assert len(sc._scales) == 0                                     # shared: one scale
        key("m")
        sc.sync()
        assert sc._map_rect is None and sc.acq.latest().map_raster is None
    finally:
        import matplotlib.pyplot as plt
        plt.close(sc.fig)


# --- unplug and replug: seq restarts, long outages, connections, the control port --
CLK = C.SYS_CLK_HZ


def frame_at(seq, t_cap):
    return F.parse_frame(F.build_frame(seq, [seq % 1000] * C.N_CH, t_cap=t_cap & 0xFFFFFFFF))


def feed(buf, seqs, tcap_of):
    return sum(buf.push_frame(frame_at(s, tcap_of(s)), 0.0) for s in seqs)


def test_a_seq_restart_after_a_short_run_is_shown_at_once():
    # clear_counters (a bring-up, a recovery): seq back to 0, t_cap keeps counting.
    buf = V.Buffers(depth=4000)
    assert feed(buf, range(3000), lambda s: s * PERIOD) == 3000
    resume = 3000 + 2000                       # a 1 s pause, then the new run
    assert feed(buf, range(1000), lambda s: (resume + s) * PERIOD) == 1000
    assert buf.restarts == 1 and buf.reordered == 0 and buf.frames == 4000
    assert buf.t[-1] == pytest.approx((resume + 999) * PERIOD / CLK)
    assert buf.lost == 0                        # a restart is not loss


def test_an_fpga_reset_after_a_long_run_is_a_new_run_too():
    # Reset: seq and t_cap both start over, so time cannot tell; the seq distance can.
    buf = V.Buffers(depth=4000)
    feed(buf, range(10), lambda s: s * PERIOD)
    feed(buf, range(70_000, 70_010), lambda s: s * PERIOD)       # long run (gap: fine)
    assert feed(buf, range(500), lambda s: 7 + s * PERIOD) == 500
    assert buf.restarts == 1 and buf.reordered == 0
    assert list(buf.t) == sorted(buf.t)         # time never runs backwards in the store


def test_a_late_frame_does_not_move_the_time_base():
    buf = V.Buffers(depth=100)
    feed(buf, [s for s in range(12) if s != 7], lambda s: s * PERIOD)
    assert feed(buf, [7], lambda s: s * PERIOD) == 0               # late: counted only
    feed(buf, [12], lambda s: s * PERIOD)
    assert buf.reordered == 1 and buf.restarts == 0
    assert buf.t[-1] == pytest.approx(12 * PERIOD / CLK)


@pytest.mark.parametrize("outage_s", [10.0, 30.0, 50.0, 100.0])
def test_an_outage_lands_at_the_right_time_whatever_its_length(outage_s):
    # t_cap wraps every 44.7 s; a delta alone reads a 30 s outage as -14.7 s.
    buf = V.Buffers(depth=100)
    feed(buf, range(10), lambda s: s * PERIOD)
    back = 10 + int(outage_s * 2000)
    feed(buf, [back, back + 1], lambda s: s * PERIOD)
    assert buf.t[-2] == pytest.approx(back * PERIOD / CLK)
    assert buf.t[-1] - buf.t[-2] == pytest.approx(PERIOD / CLK)
    assert buf.gaps == 1 and buf.lost == back - 10


def test_tcp_source_takes_the_old_connections_records_before_switching():
    src = V.TcpSource(port=0, bind="127.0.0.1")
    src._done.set()                               # drive it by hand, no thread racing us
    src.th.join(timeout=2)
    a, b = socket.socketpair()
    try:
        src.conn = a
        b.sendall(dgram(0) + dgram(5) + dgram(10)[:100])
        time.sleep(0.05)
        pend = bytearray()
        src._drain(pend)
        assert len(src.q) == 2 and len(pend) == 100   # two whole records kept
        src._drop_conn(pend)
        assert src.partial_drops == 1 and src.conn is None
    finally:
        b.close()
        src.lsock.close()


def test_a_failing_link_poll_does_not_stop_acquisition():
    class Source:
        k = 0

        def poll(self, buf):
            buf.push(self.k, [0] * 128, self.k * 0.0005)
            self.k += 1
            return 1

        def close(self):
            pass

    class UnpluggedPort:
        last, error = None, ""

        def poll(self):
            raise OSError(6, "Device not configured")

    buf = V.Buffers(depth=500)
    acq = V.Acquirer(Source(), buf, "udp", UnpluggedPort(), window_s=0.2).start()
    try:
        deadline = time.monotonic() + 5
        while buf.frames < 200 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert buf.frames >= 200 and acq.th.is_alive()
        assert "control port" in acq.error
    finally:
        acq.stop()


def test_r_asks_the_link_watcher_to_reconfigure():
    pytest.importorskip("numpy")
    mpl = pytest.importorskip("matplotlib")
    mpl.use("Agg", force=True)
    from types import SimpleNamespace
    asked = []
    link = SimpleNamespace(last=None, error="", poll=lambda: None,
                           request_reconfigure=lambda: asked.append(1))
    buf = V.Buffers(depth=500)
    acq = V.Acquirer(None, buf, "tcp", link, window_s=1.0, history_s=2.0)
    for k in range(400):
        buf.push(k, [k % 50] * 128, k / 2000)
    sc = V.Scope(acq, "tcp", window_s=1.0, history_s=2.0)
    try:
        assert "r" not in mpl.rcParams["keymap.home"]      # matplotlib no longer owns r
        sc._key(SimpleNamespace(key="r"))
        assert asked == [1]
    finally:
        import matplotlib.pyplot as plt
        plt.close(sc.fig)
