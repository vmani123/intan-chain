"""The diagnostic list client, the host-side phase scan, and the board model under them."""
import pytest

from chain import contract as C
from chain import diag as D
from chain import fakeboard as FB
from chain import rhd2164 as R
from chain.ctrl import Ctrl


def board(*faults, **kw):
    b = FB.FakeBoard(FB.make_faults(*faults), **kw)
    return b, Ctrl(ser=b), D.Diag(Ctrl(ser=b))


# --- the board model speaks section 4 -----------------------------------------

def test_fakeboard_replies_exactly_as_section_4_says():
    b, ctrl, _ = board()
    b.write(b"R 1f\n")
    assert b.readline() == f"1f={C.ID_EXPECTED:08x}\n".encode()
    b.write(b"W 22 00000040\n")
    assert b.readline() == b"ok\n"
    for bad in (b"W 10 00000001\n", b"R 99\n", b"X\n", b"Rr 1f\n", b"\n"):
        b.write(bad)
        assert b.readline() == b"err\n"
    assert ctrl.read("CAPS") >> 16 & 0xFF == C.CAP_TAPS


def test_fakeboard_saturates_and_clamps_like_the_contract():
    _b, ctrl, _ = board()
    ctrl.write("PHASE_SELECT", 99)
    ctrl.write("SPI_DIV", 40)
    ctrl.write("DIAG_LEN", 0)
    assert (ctrl.read("PHASE_SELECT"), ctrl.read("SPI_DIV"), ctrl.read("DIAG_LEN")) == (15, 15, 1)
    ctrl.write("DIAG_LEN", 500)
    assert ctrl.read("DIAG_LEN") == 64


def test_diag_cmd_write_advances_the_pointer_and_reads_back():
    _b, ctrl, _ = board()
    ctrl.write("DIAG_PTR", 0)
    for w in (0x1111, 0x2222, 0x3333):
        ctrl.write("DIAG_CMD", w)
    assert ctrl.read("DIAG_PTR") == 3
    ctrl.write("DIAG_PTR", 1)
    assert ctrl.read("DIAG_CMD") == 0x2222 and ctrl.read("DIAG_PTR") == 1  # read: no side effect


def test_chip_model_pipeline_rom_and_b_only_registers():
    chip = FB.ChipModel(0)
    seq = [R.write(20, 0x5A), R.read(20), R.read(59), R.read(40), R.FLUSH, R.FLUSH]
    out = [chip.step(c) for c in seq]
    assert out[2] == (0xFF5A, 0xFF5A)          # cycle 2 answers command 0: the echo
    assert out[3] == (0x0000, 0x005A)          # register 20 reads back on module B only
    assert out[4] == (0x0035, 0x003A)          # the A/B marker
    assert out[5] == (ord("I"), ord("I"))


def test_chip_model_calibration_window_consumes_nine_commands():
    chip = FB.ChipModel(0)
    chip.step(R.write(4, 0x40))                 # two's complement: MSB-only answer is 0
    for c in [R.CALIBRATE] + [R.FLUSH] * 9 + [R.FLUSH] * 4:
        chip.step(c)
    answers = [chip.execute(R.FLUSH) for _ in range(1)]
    assert answers == [(4, 4)]
    chip2 = FB.ChipModel(0)
    chip2.execute(R.CALIBRATE)
    assert [chip2.execute(R.FLUSH) for _ in range(10)][-2:] == [(0x8000, 0x8000), (4, 4)]


# --- the wire model and decode_taps agree at every phase -----------------------

@pytest.mark.parametrize("spi_div", [0, 1, 3])
def test_decode_taps_recovers_the_words_inside_the_window(spi_div):
    # Delay D fast clocks opens the window at phases [D/T, D/T + 2), T = SPI_DIV + 1.
    for delay in (0.0, 3.0, 5.0, 7.5, 12.0):
        w = FB.Wire(FB.ChipFaults(delay_fc=delay))
        for a, b in ((0x0035, 0x003A), (0xA5C3, 0x5A3C), (0xFFFF, 0x0000)):
            taps, _low = w.cycle(a, b, spi_div)
            d = delay / (spi_div + 1)
            for p in range(C.PHASE_MAX + 1):
                if d <= p < d + 2:
                    assert R.decode_taps(taps, p) == (a, b), (delay, spi_div, p)


def test_outside_the_window_the_decode_is_shifted_not_random():
    w = FB.Wire(FB.ChipFaults(delay_fc=5.0))
    w.cycle(0, 0, 0)
    taps, _ = w.cycle(0x0035, 0x003A, 0)
    assert R.decode_taps(taps, 7) == (0x003A, 0x006A)   # half a bit late: A sees B, B sees A<<1
    assert R.decode_taps(taps, 9)[0] == 0x006A           # a whole bit late: one bit left


# --- pipeline alignment and chunking -------------------------------------------

def test_results_are_the_cycle_two_later():
    _b, _ctrl, diag = board()
    cap = diag.run(R.rom_commands(), raw=True)
    ctrl = diag.ctrl
    ctrl.write("PHASE_SELECT", 5)
    cap = diag.run(R.rom_commands(), raw=True)
    res = cap.results()
    assert res[-1] is None and res[-2] is None          # the flushers have no answer here
    assert res[0].cycle == 2 and res[0].command == R.read(40)
    assert res[0].words == (ord("I"), ord("I"), ord("I"), ord("I"))
    i59 = R.ROM_LIST_REGS.index(59)
    assert res[i59].words == (53, 58, 53, 58)


def test_chunk_plan_appends_flushers_per_piece():
    assert D.chunk_plan(list(range(64))) == [list(range(64))]
    plan = D.chunk_plan(list(range(100)))
    assert [len(p) for p in plan] == [64, 40]
    assert plan[0][:62] == list(range(62)) and plan[0][62:] == R.FLUSHERS
    assert plan[1][:38] == list(range(62, 100)) and plan[1][38:] == R.FLUSHERS


def test_a_long_list_is_chunked_and_every_answer_lines_up():
    _b, ctrl, diag = board()
    ctrl.write("PHASE_SELECT", 5)
    regs = [40, 41, 42, 43, 44, 59, 61, 62, 63] * 12          # 108 commands
    res = diag.run_results([R.read(r) for r in regs])
    assert len(res) == len(regs)
    assert {r.chunk for r in res} == {0, 1}
    for r, got in zip(regs, res):
        assert got.words[0] == R.ROM_A[r] and got.words[2] == R.ROM_A[r]
        if r in R.ROM_B:
            assert got.words[1] == R.ROM_B[r] and got.words[3] == R.ROM_B[r]


def test_a_repeated_list_is_not_written_again():
    b, ctrl, diag = board()
    diag.run(R.convert_sweep(), cycles=())
    n = sum(1 for l in b.log if l.startswith("W 21"))
    diag.run(R.convert_sweep(), cycles=())
    assert sum(1 for l in b.log if l.startswith("W 21")) == n == 34


def test_go_while_running_is_rejected_and_a_dead_sequencer_times_out():
    _b, ctrl, diag = board()
    ctrl.set_run(True)
    with pytest.raises(D.DiagRejected):
        diag.run(R.rom_commands())
    _b, _ctrl, diag = board("none")
    diag.ctrl.ser.f.spi_dead = True
    clock = FB.VirtualClock()
    diag.clock, diag.sleep, diag.poll = clock, clock.sleep, 0.01
    with pytest.raises(D.DiagTimeout):
        diag.run(R.rom_commands())


# --- phase scan ---------------------------------------------------------------

def test_eye_and_choice_on_a_healthy_board():
    _b, _ctrl, diag = board()
    res = diag.run(R.rom_commands(), raw=True).results()
    eye = D.eye_scan(res, D.rom_expected_by_lane())
    for lane in range(4):
        assert eye.passing(lane) == [5, 6]
    ch = D.choose_phase(eye)
    assert ch.phase == 5 and ch.window == (5, 6) and ch.lanes == [0, 1, 2, 3]
    assert "chip1 B" in eye.table()


def test_window_choice_helpers():
    assert D.runs([9, 4, 5, 6]) == [(4, 6), (9, 9)]
    assert D.runs([15, 0]) == [(0, 0), (15, 15)]          # no wrap
    assert D.widest([(1, 2), (5, 8), (10, 13)]) == (5, 8)  # tie: the earlier
    assert D.centre((5, 8)) == 6 and D.centre((5, 6)) == 5 and D.centre((3, 3)) == 3
    eye = D.Eye({0: [0] * 3 + [1] * 5 + [0] * 8, 1: [0] * 5 + [1] * 6 + [0] * 5,
                 2: [0] * 16, 3: [0] * 16},
                {0: 1, 1: 1, 2: 1, 3: 1})
    ch = D.choose_phase(eye)                              # common 5..7, lanes 2/3 absent
    assert ch.lanes == [0, 1] and ch.window == (5, 7) and ch.phase == 6
    assert ch.per_lane[0] == [(3, 7)] and ch.per_lane[2] == []


def test_offset_scan_finds_a_late_answer():
    _b, _ctrl, diag = board("late")
    res = diag.run(R.rom_commands(), raw=True).results()
    exp = D.rom_expected_by_lane()
    assert D.eye_scan(res, exp).present() == []
    assert D.offset_scan(res, exp[0], 0) == [20, 21]


def test_inverted_and_swapped_scans():
    _b, _ctrl, diag = board("chip0_inverted", "chip1_ab_swap")
    res = diag.run(R.rom_commands(), raw=True).results()
    exp = D.rom_expected_by_lane()
    assert D.eye_scan(res, exp, invert=True).passing(0) == [5, 6]
    sw = D.eye_scan(res, exp, swap=True)
    assert sw.passing(2) == [5, 6] and sw.passing(3) == [5, 6]


def test_bit_shift():
    assert D.bit_shift(0x006A, 0x0035) == 1
    assert D.bit_shift(0x001A, 0x0035) == -1
    assert D.bit_shift(0x1234, 0x0035) is None
    assert D.bit_shift(0x0000, 0x0000) is None


# --- cross-check and status registers -------------------------------------------

def test_crosscheck_is_clean_and_catches_an_fpga_decode_bug():
    _b, _ctrl, diag = board()
    assert D.crosscheck(diag.run(R.rom_commands(), raw=True)) == []
    _b, _ctrl, diag = board("decode_bug")
    bad = D.crosscheck(diag.run(R.rom_commands(), raw=True))
    assert bad and all(m.fpga != m.host for m in bad)


def test_miso_stat_and_link_decoders():
    ms = D.decode_miso_stat(0b1_0001_1011, 7, 0)
    assert ms.seen_low == (True, False) and ms.seen_high == (True, True)
    assert ms.level == (1, 0) and ms.loopback and ms.edges == (7, 0)
    assert ms.line(0) == "toggling" and ms.line(1) == "stuck_high"
    assert D.decode_miso_stat(0, 0, 0).line(0) == "unsampled"
    caps = D.decode_caps((78 << 16) | 0x100 | 3)
    assert (caps.taps, caps.emulator, caps.qspi_div) == (78, True, 3)


def test_read_miso_and_link_from_the_board():
    b, ctrl, diag = board("chip1_absent")
    ctrl.clear_counters()
    diag.run(R.rom_commands(), cycles=())
    ms = D.read_miso(ctrl)
    assert ms.line(0) == "toggling" and ms.line(1) == "stuck_high"
    assert ms.edges[0] > 0 and ms.edges[1] == 0
    ctrl.write("PHASE_SELECT", 5)
    ctrl.write("SAMPLE_PERIOD", 48000)
    ctrl.set_run(True)
    b.clock.sleep(0.1)
    ctrl.set_run(False)            # counters move between reads while running
    link = D.read_link(ctrl)
    assert link.checks >= 200
    assert link.errors[0] == link.errors[1] == 0
    assert link.errors[2] == link.errors[3] == link.checks
    assert link.last[0] == 0x0035 and link.last[1] == 0x003A and link.last[2] == 0xFFFF


def test_convert_snapshot_is_in_frame_channel_order():
    def signal(chip, module, c, n):
        return 64 * chip + (32 if module == "B" else 0) + c

    b = FB.FakeBoard(signal=signal)
    ctrl = Ctrl(ser=b)
    ctrl.write("PHASE_SELECT", 5)
    diag = D.Diag(ctrl)
    diag.run([R.write(4, 0x40)] + [R.write(r, 0xFF) for r in range(14, 22)] + R.FLUSHERS)
    vals = D.convert_snapshot(diag)
    assert vals == list(range(C.N_CH))
