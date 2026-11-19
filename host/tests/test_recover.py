"""LinkWatch against the fake board: unplug, replug, power loss, port loss."""
import random
import time

from chain import bringup as B
from chain import contract as C
from chain import fakeboard as FB
from chain import recover as RC
from chain import rhd2164 as R
from chain.ctrl import Ctrl

QUIET = lambda *_: None


def streaming(*faults):
    clock = FB.VirtualClock()
    board = FB.FakeBoard(FB.make_faults(*faults), clock=clock)
    ctrl = Ctrl(ser=board)
    opts = B.Options(resync_settle=0)
    B.prepare(ctrl, opts, out=QUIET, sleep=clock.sleep, clock=clock, stream=True)
    assert ctrl.read("CTRL") & 1
    return clock, board, ctrl, opts


def watch(clock, ctrl, opts, **kw):
    return RC.LinkWatch(ctrl, opts, clock=clock, sleep=clock.sleep, settle=1.0, **kw)


def run_for(w, clock, seconds, interval=0.5):
    end = clock() + seconds
    while clock() < end:
        w.step()
        clock.sleep(interval)


def unplug(board, chips=(0, 1)):
    for c in chips:
        board.f.chips[c].line = "absent"      # MISO receivers idle high: READ(59) fails


def replug(board, chips=(0, 1), seed=7):
    """Back in, with every register at power-up garbage, as on the bench."""
    rng = random.Random(seed)
    for c in chips:
        board.f.chips[c].line = None
        board.chips[c].ram = [rng.randrange(256) for _ in range(R.N_RAM)]
        board.chips[c].calib = 0


def configured(board, ctrl, opts):
    cfg = R.AmpConfig(rate=opts.rate, lower=opts.lower, upper=opts.upper, dsp=opts.dsp,
                      dsp_cutoff=opts.dsp_cutoff, spi_div=ctrl.read("SPI_DIV"))
    want = R.register_values(cfg)
    return [board.chips[c].ram == want for c in (0, 1)]


def test_a_steady_link_is_left_alone():
    clock, board, ctrl, opts = streaming()
    w = watch(clock, ctrl, opts)
    run_for(w, clock, 5.0)
    assert w.state == "streaming" and w.recoveries == 0 and w.note == ""
    assert w.poll() is w.last and w.last.checks > 0


def test_replug_reconfigures_both_chips_and_streams_again():
    clock, board, ctrl, opts = streaming()
    w = watch(clock, ctrl, opts)
    run_for(w, clock, 2.0)
    unplug(board)
    run_for(w, clock, 3.0)
    assert w.state == "down" and "LINK DOWN" in w.note
    assert w.recoveries == 0                     # nothing to do while it is out
    replug(board)
    assert configured(board, ctrl, opts) == [False, False]
    run_for(w, clock, 3.0)
    assert w.recoveries == 1 and w.state == "streaming"
    assert configured(board, ctrl, opts) == [True, True]
    assert ctrl.read("CTRL") & 1
    texts = [t for _, t in w.events]
    assert any("link down" in t for t in texts) and any("recovered" in t for t in texts)
    assert any("recovered" in t and "(quick" in t for t in texts)   # the quick path was enough
    run_for(w, clock, 3.0)
    assert w.recoveries == 1                     # and it does not keep reconfiguring


def test_a_counter_saturated_by_a_long_unplug_still_reads_as_down():
    clock, board, ctrl, opts = streaming()
    w = watch(clock, ctrl, opts)
    run_for(w, clock, 1.0)
    unplug(board)
    run_for(w, clock, 40.0)                      # > 0xffff sweeps: counters stop moving
    assert board._reg("LINK_ERR0") & 0xFFFF == 0xFFFF
    assert w.state == "down" and w.recoveries == 0


def test_occasional_link_errors_do_not_trigger_a_reconfigure():
    clock, board, ctrl, opts = streaming("flaky_link")      # 1 % of sweeps on chip 0
    w = watch(clock, ctrl, opts)
    run_for(w, clock, 10.0)
    assert w.last.errors[0] > 0
    assert w.recoveries == 0 and w.state == "streaming"


def test_a_chip_absent_from_the_start_is_noted_then_configured_when_plugged_in():
    clock, board, ctrl, opts = streaming("chip1_absent")
    w = watch(clock, ctrl, opts)
    run_for(w, clock, 3.0)
    assert w.state == "streaming" and w.recoveries == 0
    assert "not answering: chip1 A, chip1 B" in w.note
    replug(board, chips=(1,))
    run_for(w, clock, 3.0)
    assert w.recoveries == 1
    assert configured(board, ctrl, opts) == [True, True]


def test_one_chip_unplugged_while_the_other_keeps_running():
    clock, board, ctrl, opts = streaming()
    w = watch(clock, ctrl, opts)
    run_for(w, clock, 1.0)
    unplug(board, chips=(1,))
    run_for(w, clock, 3.0)
    assert w.state == "down" and "chip1 A, chip1 B" in w.note and "chip0" not in w.note
    replug(board, chips=(1,))
    run_for(w, clock, 3.0)
    assert w.recoveries == 1 and configured(board, ctrl, opts) == [True, True]


def test_an_fpga_reloaded_by_hand_gets_a_full_bringup():
    clock, board, ctrl, opts = streaming()
    w = watch(clock, ctrl, opts)            # no programmer: it can only ask
    run_for(w, clock, 1.0)
    board.f.silent = True                   # power lost: the bitstream is gone
    run_for(w, clock, 2.0)
    assert w.state == "fpga" and "reload the bitstream" in w.note
    board.f.silent = False                  # someone ran openFPGALoader: fresh registers
    for name in ("CTRL", "SPI_DIV", "PHASE_SELECT", "MODE"):
        board._set(name, 0)
    board._finish_run()
    replug(board)
    run_for(w, clock, 3.0)
    assert w.recoveries == 1 and w.state == "streaming"
    assert ctrl.read("CTRL") & 1 and configured(board, ctrl, opts) == [True, True]
    assert any("(full bring-up" in t for _, t in w.events)     # registers were reset: rescan


def test_power_loss_is_reprogrammed_and_brought_up():
    clock, board, ctrl, opts = streaming()
    calls = []

    def program():
        calls.append(clock())
        board.f.silent = False
        for name in ("CTRL", "SPI_DIV", "PHASE_SELECT", "MODE"):
            board._set(name, 0)
        board._finish_run()
        return ""

    w = watch(clock, ctrl, opts, program=program, open_ctrl=lambda: Ctrl(ser=board))
    run_for(w, clock, 1.0)
    board.f.silent = True
    replug(board)
    run_for(w, clock, 4.0)
    assert len(calls) == 1
    assert w.recoveries == 1 and w.state == "streaming"
    assert w.ctrl is not None and w.ctrl.read("CTRL") & 1
    assert configured(board, w.ctrl, opts) == [True, True]


class Unplugging:
    """A serial port that can vanish, as the Arty's does when its USB is pulled."""

    def __init__(self, board):
        self.board, self.gone = board, False

    def __getattr__(self, name):
        attr = getattr(self.board, name)
        if callable(attr) and name in ("write", "readline", "reset_input_buffer"):
            def guarded(*a, **k):
                if self.gone:
                    raise OSError(6, "Device not configured")
                return attr(*a, **k)
            return guarded
        return attr


def test_a_lost_port_is_reopened_and_the_chain_rebuilt():
    clock, board, _ctrl, opts = streaming()
    port = Unplugging(board)

    def reopen():
        if port.gone:
            raise OSError(2, "No such file or directory")
        return Ctrl(ser=port)

    w = watch(clock, Ctrl(ser=port), opts, open_ctrl=reopen, reopen_every=1.0)
    run_for(w, clock, 1.0)
    port.gone = True
    run_for(w, clock, 3.0)
    assert w.state == "port" and w.ctrl is None and "disconnected" in w.note
    port.gone = False
    run_for(w, clock, 4.0)
    assert w.recoveries == 1 and w.state == "streaming"
    assert w.ctrl.read("CTRL") & 1


def test_a_manual_request_reconfigures_on_the_next_poll():
    clock, board, ctrl, opts = streaming()
    w = watch(clock, ctrl, opts)
    run_for(w, clock, 1.0)
    replug(board)                            # e.g. a brown-out the link check never saw
    w.request_reconfigure()
    w.step()
    assert w.recoveries == 1 and configured(board, ctrl, opts) == [True, True]


def test_the_thread_starts_and_stops_cleanly():
    clock, board, ctrl, opts = streaming()
    w = RC.LinkWatch(ctrl, opts, interval=0.01, settle=1.0)
    w.start()
    time.sleep(0.2)
    w.stop()
    assert not w.th.is_alive() and w.last is not None


def test_openfpgaloader_programmer_reports_what_is_missing(tmp_path):
    assert "no bitstream" in RC.openfpgaloader(tmp_path / "none.bit",
                                               run=lambda *a, **k: None)() or \
        "not on PATH" in RC.openfpgaloader(tmp_path / "none.bit")()
    bit = tmp_path / "x.bit"
    bit.write_bytes(b"\0")

    class Done:
        returncode, stdout, stderr = 1, "", "JTAG init failed"

    import shutil
    if shutil.which("openFPGALoader"):
        assert "JTAG init failed" in RC.openfpgaloader(bit, run=lambda *a, **k: Done())()


def test_bringup_names_a_port_that_vanishes_mid_run_and_still_reports():
    clock = FB.VirtualClock()
    board = FB.FakeBoard(clock=clock)
    port = Unplugging(board)
    writes = {"n": 0}
    real_write = board.write

    def counting_write(data):
        writes["n"] += 1
        if writes["n"] > 40:          # partway through S3
            port.gone = True
        return real_write(data)

    board.write = counting_write
    b = B.Bringup(lambda: Ctrl(ser=port), B.Options(resync_settle=0), out=QUIET,
                  sleep=clock.sleep, clock=clock)
    rep = b.run()
    assert rep["result"] == "FAIL"
    codes = [f.category for s in b.stages for f in s.findings]
    assert "PORT_LOST" in codes and "HOST_ERROR" not in codes
    B.stop_and_close(b.ctrl)          # the CLI's cleanup on a dead port must not raise
