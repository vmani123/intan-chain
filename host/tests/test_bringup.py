"""Each bring-up stage's classification under each fault, against the board model.

Assertions are on the finding categories (the keys of bringup.CAUSES), never on
prose, so the wording can improve without breaking the tests.
"""
import json

import pytest

from chain import bringup as B
from chain import fakeboard as FB
from chain.ctrl import Ctrl


def run(*faults, board_kw=None, faults_obj=None, **opt):
    clock = FB.VirtualClock()
    f = faults_obj or FB.make_faults(*faults)
    board = FB.FakeBoard(f, clock=clock, **(board_kw or {}))
    opt.setdefault("serial", "fake")
    opt.setdefault("seconds", 0.2)
    opt.setdefault("snapshots", 4)
    o = B.Options(resync_settle=0.0, **opt)
    lines = []
    bu = B.Bringup(lambda: Ctrl(ser=board), o, out=lines.append, sleep=clock.sleep, clock=clock)
    rep = bu.run()
    bu.board, bu.text = board, "\n".join(lines)
    return bu, rep


def st(bu, sid):
    return next(s for s in bu.stages if s.id == sid)


def cats(bu, sid, status="FAIL"):
    return st(bu, sid).categories(status)


def statuses(bu):
    return {s.id: s.status for s in bu.stages}


# --- the healthy board ---------------------------------------------------------

def test_a_healthy_board_passes_every_stage_and_says_why():
    bu, rep = run()
    assert statuses(bu) == {f"S{i}": "PASS" for i in range(8)}
    assert rep["result"] == "PASS"
    assert rep["chosen"]["spi_div"] == 0 and rep["chosen"]["phase"] == 5
    assert all(rep["lanes"].values())
    assert "LIKELY CAUSE" not in bu.text
    assert all(st(bu, s).checked for s in ("S0", "S1", "S2", "S3", "S4", "S5", "S6", "S7"))
    assert "ESP_NOT_DRAINING" in cats(bu, "S7", "INFO")   # no ESP is a note, not a failure
    assert not bu.board.running                           # S7 stops the run it started


def test_an_esp_draining_qspi_is_recognised():
    f = FB.make_faults()
    f.esp_present = True
    bu, _ = run(faults_obj=f)
    assert "ESP_NOT_DRAINING" not in cats(bu, "S7", "INFO")
    assert any("ESP draining" in c for c in st(bu, "S7").checked)


def test_leave_running():
    bu, _ = run(leave_running=True)
    assert bu.board.running and st(bu, "S7").status == "PASS"


# --- S0 ------------------------------------------------------------------------

def test_port_that_will_not_open_is_hard_and_names_the_uart_port():
    def boom():
        raise OSError("[Errno 2] No such file or directory")
    o = B.Options(serial="/dev/cu.usbserial-210352BF97960", resync_settle=0)
    bu = B.Bringup(boom, o, out=lambda *_: None)
    bu.run()
    s0 = st(bu, "S0")
    assert s0.categories("FAIL") == {"PORT_OPEN"} and s0.hard
    assert "/dev/cu.usbserial-210352BF97961" in s0.findings[0].check
    assert {s.status for s in bu.stages[1:]} == {"SKIP"}


def test_missing_pyserial_is_its_own_cause():
    def no_serial():
        raise ModuleNotFoundError("No module named 'serial'")
    bu = B.Bringup(no_serial, B.Options(serial="x"), out=lambda *_: None)
    bu.run()
    assert cats(bu, "S0") == {"PYSERIAL_MISSING"}


@pytest.mark.parametrize("fault, cat", [("silent", "NO_REPLY"), ("garbled", "GARBLED_REPLY"),
                                        ("old_bitstream", "OLD_BITSTREAM")])
def test_s0_board_identity_failures_are_hard(fault, cat):
    bu, rep = run(fault, serial="/dev/cu.usbserial-ABC0")
    assert cats(bu, "S0") == {cat} and st(bu, "S0").hard
    assert statuses(bu)["S7"] == "SKIP" and rep["result"] == "FAIL"
    if fault == "silent":
        assert "usbserial-ABC1" in st(bu, "S0").findings[0].check


def test_s0_wrong_id_and_wrong_tap_count():
    f = FB.make_faults()
    f.board_id = 0xDEADBEEF
    bu, _ = run(faults_obj=f)
    assert cats(bu, "S0") == {"WRONG_BITSTREAM"}
    f = FB.make_faults()
    f.caps_taps = 74
    bu, _ = run(faults_obj=f)
    assert cats(bu, "S0") == {"CONTRACT_MISMATCH"}


def test_s0_clears_a_run_left_set():
    clock = FB.VirtualClock()
    board = FB.FakeBoard(clock=clock)
    Ctrl(ser=board).set_run(True)
    bu = B.Bringup(lambda: Ctrl(ser=board), B.Options(resync_settle=0, seconds=0.1,
                                                      snapshots=2),
                   out=lambda *_: None, sleep=clock.sleep, clock=clock)
    bu.run()
    assert "RUN_WAS_SET" in cats(bu, "S0", "INFO") and statuses(bu)["S2"] == "PASS"


# --- S1 ------------------------------------------------------------------------

def test_s1_skips_honestly():
    bu, _ = run("no_emulator")
    assert statuses(bu)["S1"] == "SKIP" and "CAPS bit8" in st(bu, "S1").summary
    bu, _ = run(skip_loopback=True)
    assert statuses(bu)["S1"] == "SKIP"


@pytest.mark.parametrize("fault, cat", [("selftest_broken", "SELFTEST_FAIL"),
                                        ("decode_bug", "DECODE_MISMATCH")])
def test_s1_failures_blame_the_fpga_and_stop(fault, cat):
    bu, _ = run(fault)
    assert cat in cats(bu, "S1") and st(bu, "S1").hard
    assert statuses(bu)["S2"] == "SKIP"
    assert not bu.board.loopback                           # loopback cleared on the way out


def test_decode_mismatch_without_loopback_is_caught_in_s3():
    bu, _ = run("decode_bug", skip_loopback=True)
    assert "DECODE_MISMATCH" in cats(bu, "S3")


# --- S2 ------------------------------------------------------------------------

def test_s2_one_chip_absent():
    bu, rep = run("chip1_absent")
    s2 = st(bu, "S2")
    assert s2.categories("FAIL") == {"MISO_STUCK_HIGH"} and not s2.hard
    assert s2.findings[0].scope == "chip1"
    assert {"ROM_ALL_ONES", "CHIP_NOT_ANSWERING"} <= cats(bu, "S3")
    assert rep["lanes"] == {"chip0 A": True, "chip0 B": True, "chip1 A": False,
                            "chip1 B": False}
    assert statuses(bu)["S4"] == "PASS" and bu.chip_cfg_ok == [True, False]
    assert statuses(bu)["S7"] == "PASS"                    # chip1's link errors are expected
    assert rep["result"] == "FAIL"


def test_s2_stuck_low_points_at_the_wire_and_ground():
    bu, _ = run("chip0_stuck_low")
    assert cats(bu, "S2") == {"MISO_STUCK_LOW"}
    assert "ground" in st(bu, "S2").findings[0].check
    assert {"ROM_ALL_ZERO", "CHIP_NOT_ANSWERING"} <= cats(bu, "S3")


def test_s2_both_lines_stuck_is_a_shared_cause_and_stops():
    bu, _ = run("both_stuck_high")
    assert "BOTH_STUCK" in cats(bu, "S2") and st(bu, "S2").hard
    assert statuses(bu)["S3"] == "SKIP"


def test_s2_sequencer_dead():
    bu, _ = run("spi_dead", skip_loopback=True)
    assert cats(bu, "S2") == {"SPI_NOT_RUNNING"} and st(bu, "S2").hard


# --- S3 ------------------------------------------------------------------------

def test_s3_inverted_miso_pair():
    bu, _ = run("chip0_inverted")
    assert "MISO_INVERTED" in cats(bu, "S3")
    assert {f.scope for f in st(bu, "S3").findings if f.category == "MISO_INVERTED"} == {
        "chip0 A", "chip0 B"}


def test_s3_ab_swap():
    bu, _ = run("chip1_ab_swap")
    assert "AB_SWAPPED" in cats(bu, "S3")
    assert "NO_COMMON_WINDOW" not in cats(bu, "S3")


def test_s3_late_answers_retry_a_slower_sclk_and_say_why():
    bu, rep = run("late")
    s3 = st(bu, "S3")
    assert s3.status == "WARN" and "SPI_DIV_CHANGED" in s3.categories("WARN")
    assert "DELAY_OUT_OF_WINDOW" in s3.categories("INFO")
    assert rep["chosen"]["spi_div"] == 1 and all(rep["lanes"].values())
    assert [a["spi_div"] for a in s3.data["attempts"]] == [0, 1]
    assert statuses(bu)["S7"] == "PASS"                    # streams at the new SPI_DIV
    assert bu.board.regs[0x04] == 1


def test_s3_late_at_a_slow_enough_div_needs_no_retry():
    bu, rep = run("late", spi_div=1)
    assert statuses(bu)["S3"] == "PASS" and rep["chosen"]["spi_div"] == 1


def test_s3_classifies_a_delay_out_of_window_when_no_retry_helps():
    f = FB.make_faults()
    for c in f.chips:
        c.delay_fc = 160.0       # 1.7 us: past the window at every SPI_DIV tried
    bu, _ = run(faults_obj=f, spi_div=7)
    assert "DELAY_OUT_OF_WINDOW" in cats(bu, "S3")
    assert st(bu, "S3").hard


def test_s3_skewed_chips_find_a_div_with_a_common_window():
    bu, rep = run("skewed_chips")
    s3 = st(bu, "S3")
    assert "SPI_DIV_CHANGED" in s3.categories("WARN")
    assert "NO_COMMON_WINDOW" in s3.categories("INFO")
    assert rep["chosen"]["spi_div"] == 3 and all(rep["lanes"].values())


def test_s3_no_common_window_at_any_div():
    f = FB.make_faults()
    f.chips[1].delay_fc = 45.0
    bu, rep = run(faults_obj=f)
    assert "NO_COMMON_WINDOW" in cats(bu, "S3")
    assert sum(rep["lanes"].values()) == 2


def test_s3_narrow_window_warns():
    f = FB.make_faults()
    for c in f.chips:
        c.delay_fc = 8.0
    f.edge_fc = 0.5              # samples on an edge read random: one phase survives
    bu, rep = run(faults_obj=f, spi_div=7)
    assert "NARROW_WINDOW" in cats(bu, "S3", "WARN")
    assert all(rep["lanes"].values())


def test_s3_wrong_part():
    bu, _ = run("chip0_rhd2132")
    assert "NOT_RHD2164" in cats(bu, "S3")


def test_s3_inverted_mosi_leaves_no_rom_and_stops():
    bu, _ = run("mosi_inverted")
    assert cats(bu, "S3") == {"ROM_GARBLED"} and st(bu, "S3").hard
    assert "MOSI" in st(bu, "S3").findings[0].check


def test_s3_phase_given_by_hand():
    bu, rep = run(phase=6)
    assert statuses(bu)["S3"] == "PASS" and rep["chosen"]["phase"] == 6
    bu, _ = run(phase=10)
    assert "PHASE_OUTSIDE_WINDOW" in cats(bu, "S3")


# --- S4 / S5 -------------------------------------------------------------------

def test_s4_mosi_corruption_on_both_chips():
    bu, _ = run("mosi_corrupt")
    assert statuses(bu)["S3"] == "PASS"                    # reads fine: MISO is good
    s4 = st(bu, "S4")
    assert s4.categories("FAIL") == {"MOSI_CORRUPT"}
    assert {f.scope for f in s4.findings} >= {"chip0", "chip1", "both chips"}
    assert "flipped" in s4.findings[0].message


def test_s4_mosi_corruption_on_one_chip():
    bu, _ = run("chip0_mosi_corrupt")
    s4 = st(bu, "S4")
    assert {(f.category, f.scope) for f in s4.findings if f.status == "FAIL"} == {
        ("MOSI_CORRUPT", "chip0")}
    assert bu.chip_cfg_ok == [False, True]


def test_s4_registers_that_do_not_hold():
    bu, _ = run("chip0_drops_writes")
    assert cats(bu, "S4") == {"REG_READBACK"}


def test_s4_module_b_echo_difference_is_a_warning():
    f = FB.make_faults()
    f.chips[1].b_echo_xor = 0x80
    bu, _ = run(faults_obj=f)
    assert cats(bu, "S4", "WARN") == {"B_ECHO_DIFFERS"} and not cats(bu, "S4")


def test_s4_config_warnings_do_not_change_values():
    bu, rep = run(upper=1500.0)
    assert "CONFIG_WARNING" in cats(bu, "S4", "WARN")
    assert rep["config"]["upper_requested_hz"] == 1500.0


def test_s5_calibration_window_longer_than_nine():
    f = FB.make_faults()
    f.chips[0].calib_len = 12
    bu, _ = run(faults_obj=f)
    assert cats(bu, "S5") == {"CALIBRATE_NO_RESUME"}


# --- S6 ------------------------------------------------------------------------

def test_s6_dead_module_and_dead_chip():
    bu, _ = run("chip1_dead_b")
    s6 = st(bu, "S6")
    assert s6.categories("FAIL") == {"MODULE_DEAD"}
    assert s6.findings[0].scope == "chip1 B" and "18-21" in s6.findings[0].check
    bu, _ = run("chip1_dead")
    assert cats(bu, "S6") == {"CHIP_DEAD"}


def test_s6_stuck_railed_and_noisy_channels_are_warnings():
    def signal(chip, module, c, n):
        ch = 64 * chip + (32 if module == "B" else 0) + c
        if ch == 3:
            return 17                       # stuck
        if ch == 10:
            return 32767                    # railed
        if ch == 20:
            return 2000 if n % 2 else -2000  # 390 uV: noisy
        return (n * 7 + ch) % 11 - 5
    bu, rep = run(board_kw={"signal": signal})
    s6 = st(bu, "S6")
    assert s6.status == "WARN"
    assert s6.categories("WARN") == {"STUCK_CHANNELS", "RAILED_CHANNELS", "NOISY_CHANNELS"}
    flags = {c["ch"]: c["flag"] for c in s6.data["channels"]}
    assert (flags[3], flags[10], flags[20], flags[0]) == ("S", "R", "N", ".")
    assert len(B.grid([{"std_uv": 1.0}] * 128, ["."] * 128, 4).splitlines()) == 10


# --- S7 ------------------------------------------------------------------------

def test_s7_intermittent_and_persistent_link_errors():
    bu, _ = run("flaky_link")
    s7 = st(bu, "S7")
    assert s7.categories("FAIL") == {"LINK_ERRORS_INTERMITTENT"}
    assert s7.findings[0].scope == "chip0 A"
    bu, _ = run("link_dead")
    assert {"LINK_ERRORS"} == {f.category for f in st(bu, "S7").findings
                               if f.status == "FAIL"}


def test_s7_rate_too_high_for_the_sweep():
    bu, _ = run(rate=40000.0)
    assert cats(bu, "S7") == {"RATE_TOO_HIGH"}


def test_s7_sequencer_idle():
    bu, _ = run("stream_dead")
    assert cats(bu, "S7") == {"SEQUENCER_IDLE"}


def test_s7_counts_checks_against_rate_and_time():
    bu, _ = run(seconds=1.0)
    d = st(bu, "S7").data
    lo, hi = d["expected_checks"]
    assert lo <= d["link"]["checks"] <= hi and lo > 1700


# --- report and CLI ------------------------------------------------------------

def test_json_report_shape(tmp_path):
    path = tmp_path / "r.json"
    lines = []
    rc = B.main(["--fake", "chip1_absent", "--json", str(path), "--snapshots", "3",
                 "--seconds", "0.2"], out=lines.append)
    assert rc == 1
    rep = json.loads(path.read_text())
    assert set(rep) >= {"tool", "timestamp", "result", "port", "options", "chosen", "lanes",
                        "config", "stages", "fake"}
    assert rep["result"] == "FAIL" and rep["fake"] == ["chip1_absent"]
    assert [s["id"] for s in rep["stages"]] == [f"S{i}" for i in range(8)]
    for s in rep["stages"]:
        assert set(s) >= {"id", "name", "status", "summary", "checked", "findings", "data",
                          "hard"}
        assert s["status"] in ("PASS", "WARN", "FAIL", "SKIP")
        for f in s["findings"]:
            assert set(f) == {"status", "category", "scope", "message", "cause", "check"}
            assert f["category"] in B.CAUSES
    s2 = rep["stages"][2]
    assert s2["findings"][0]["category"] == "MISO_STUCK_HIGH"
    assert len(rep["config"]["registers"]) == 22
    assert "bring-up summary" in "\n".join(lines)


def test_cli_fake_run_saves_nothing_unless_asked(tmp_path, monkeypatch):
    monkeypatch.setattr(B, "RESULTS", tmp_path)
    assert B.main(["--fake", "--snapshots", "2", "--seconds", "0.1"], out=lambda *_: None) == 0
    assert list(tmp_path.iterdir()) == []


def test_cli_requires_a_port_or_fake():
    with pytest.raises(SystemExit):
        B.main([], out=lambda *_: None)


def test_prepare_configures_and_optionally_streams():
    clock = FB.VirtualClock()
    board = FB.FakeBoard(clock=clock)
    ctrl = Ctrl(ser=board)
    b, rep = B.prepare(ctrl, B.Options(resync_settle=0), out=lambda *_: None,
                       sleep=clock.sleep, clock=clock, stream=True)
    assert [s.id for s in b.stages] == ["S0", "S2", "S3", "S4", "S5"]
    assert rep["result"] == "PASS" and board.running
    board2 = FB.FakeBoard(FB.make_faults("both_stuck_high"), clock=clock)
    b, rep = B.prepare(Ctrl(ser=board2), B.Options(resync_settle=0), out=lambda *_: None,
                       sleep=clock.sleep, clock=clock, stream=True)
    assert rep["result"] == "FAIL" and not board2.running


def test_every_category_used_has_a_cause():
    import re
    src = open(B.__file__).read()
    used = set(re.findall(r'"([A-Z][A-Z0-9_]+)"', src)) & {
        m for m in re.findall(r'st\.add\("\w+", "([A-Z0-9_]+)"', src)}
    assert used <= set(B.CAUSES)
    for cat, (cause, check) in B.CAUSES.items():
        assert cat.isupper()


def test_helpers():
    assert B.port_hint("/dev/cu.usbserial-210352BF97960").endswith("97961")
    assert B.port_hint("/dev/cu.usbserial-210352BF97961") == ""
    d = B._echo_detail({0: 0xFFDF, 1: 0xFF43}, [0xDE, 0x42] + [0] * 20)
    assert "xor 0x01" in d
    assert "not WRITE echoes" in B._echo_detail({0: 0x0000}, [0xDE] + [0] * 21)
