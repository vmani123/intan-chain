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


def test_cli_argument_rules():
    for argv in ([], ["--uart"], ["--udp", "--start"], ["--udp", "--uart"]):
        with pytest.raises(SystemExit):
            V.main(argv, out=lambda *_: None)


def test_cli_headless_udp_runs_without_a_display():
    out = []
    assert V.main(["--udp", "--udp-port", "0", "--headless", "0.3"], out=out.append) == 0
    assert any("frames 0" in l for l in out)
    assert "matplotlib" not in sys.modules


# --- the activity map and the lanes -----------------------------------------

def test_grid_puts_each_module_in_two_rows_and_round_trips():
    seen = set()
    for ch in range(128):
        r, c = V.grid_pos(ch)
        assert 0 <= r < V.GRID_ROWS and 0 <= c < V.GRID_COLS
        assert r // 2 == ch // 32                 # module m owns rows 2m and 2m+1
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


def test_minmax_envelope_keeps_every_extreme_and_the_time_order():
    np = pytest.importorskip("numpy")
    t = np.arange(4000) * 0.0005
    uv = np.zeros((4000, 2), np.float32)
    uv[1234, 0], uv[3001, 1] = 500.0, -700.0               # one-sample spikes
    x, y = V.minmax_envelope(t, uv, max_bins=300)
    assert len(x) == 600 and y.shape == (600, 2)
    assert y[:, 0].max() == 500.0 and y[:, 1].min() == -700.0
    assert np.all(np.diff(x) >= 0)
    x2, y2 = V.minmax_envelope(t[:100], uv[:100], max_bins=300)   # short: untouched
    assert len(x2) == 100


def test_lane_blocks_draw_a_spike_full_height_and_a_flat_line_thin():
    np = pytest.importorskip("numpy")
    bins, px = 50, 21
    y = np.zeros((2 * bins, 2))
    y[2 * 20 + 1, 0] = 0.48                               # channel 0 spikes in bin 20
    colors = np.array([7, 9], np.uint32)
    b = V.lane_blocks(y, px, colors, width=bins)
    assert b.shape == (2, px, bins)
    assert set(np.unique(b[0])) == {0, 7} and set(np.unique(b[1])) == {0, 9}
    assert (b[0, :, 20] == 7).sum() > px // 3              # the spike is a tall span
    assert (b[1] == 9).sum(axis=0).max() == 1              # flat: one pixel per column
    assert V.lane_blocks(y, px, colors, width=2 * bins).shape == (2, px, 2 * bins)


def test_compose_lanes_puts_each_lane_on_its_own_rows():
    np = pytest.importorskip("numpy")
    blocks = np.zeros((3, 4, 5), np.uint32)
    blocks[0], blocks[1], blocks[2] = 1, 2, 3
    img = V.compose_lanes(blocks, 12, centers=[2, 6, 10])
    assert img.shape == (12, 5)
    assert set(np.unique(img[0:4])) == {1} and set(np.unique(img[4:8])) == {2}
    assert set(np.unique(img[8:12])) == {3}


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
        assert snap is not None and snap.n > 0 and snap.env.shape[1] == 128
        assert snap.stat(0).max_uv >= snap.stat(0).min_uv
    finally:
        acq.stop()
    assert not acq.th.is_alive()
