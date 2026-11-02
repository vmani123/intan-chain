"""tapcheck: the tap registers on the fake board, record, selftest, and compare's exact counts.

compare is driven with synthetic sessions: one signal, heard by the chain as
seq 0..2999 and saved by "RHX" as timestamps seq + D for a window inside that,
then damaged in one known way per test. Every count the report gives is
asserted, not just pass/fail. Findings are asserted by category, never prose.
"""
import json
import os
import re
import signal
import socket
import threading
import time

import pytest

from chain import contract as C
from chain import tapcheck as TC
from chain import taprec
from chain.ctrl import Ctrl, FakeSerial

np = pytest.importorskip("numpy")

import rhxfake as RF  # noqa: E402

from chain import tapcompare as T  # noqa: E402

N = 3000            # sweeps the chain heard: seq 0..N-1
D = 77777           # RHX timestamp = seq + D
LO, NR = 500, 2048  # RHX saved seq LO .. LO + NR - 1 (16 blocks of 128)
SIG = RF.signal(N + 400, seed=3)


def session(tmp_path, words=None, drop=(), flags=None, seqs=None, codes=None, fmt="trad",
            groups=None, extra=(), stale=None, hdr=None, t=None):
    words = SIG[:N].copy() if words is None else words
    rec = RF.write_recording(tmp_path / "c.chaintap",
                             RF.datagrams(words, flags=flags, drop=drop, stale=stale),
                             extra=extra)
    seqs = np.arange(LO, LO + NR) if seqs is None else np.asarray(seqs)
    codes = SIG[seqs] if codes is None else codes
    t = seqs + D if t is None else t
    if fmt == "trad":
        path = tmp_path / "rhx.rhd"
        RF.write_traditional(path, codes, t, groups, **(hdr or {}))
    else:
        path = tmp_path / "rhx_fps"
        RF.write_per_signal_type(path, codes, t, groups, **(hdr or {}))
    return rec, path


def cats(rep, status=None):
    return {f["category"] for f in rep["findings"] if status is None or f["status"] == status}


# ---------------------------------------------------------------------------
# compare
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("fmt", ["trad", "fps"])
def test_a_clean_session_passes_with_every_sample_compared(tmp_path, fmt):
    rec, rhxp = session(tmp_path, fmt=fmt)
    rep = T.compare(rec, [rhxp])
    ov = rep["overlap"]
    assert rep["result"] == "PASS" and not cats(rep, "FAIL")
    assert rep["alignment"]["rhx_timestamp_minus_chain_seq"] == D
    assert rep["alignment"]["agree"] == rep["alignment"]["matched"] == 64
    assert (ov["overlap_samples"], ov["frames_compared"]) == (NR, NR)
    assert ov["compared"] == ov["equal"] == NR * 128 and ov["mismatched"] == 0
    assert (ov["frames_missing"], ov["frames_partial"], ov["rhx_gaps"]) == (0, 0, 0)
    assert rep["outside"] == {"chain_frames_before_rhx": LO,
                              "chain_frames_after_rhx": N - LO - NR,
                              "rhx_samples_before_chain": 0, "rhx_samples_after_chain": 0}
    assert rep["convention"]["by_transform"] == {"identity": "0-127"}
    assert rep["mapping"]["as_expected"]
    assert rep["mapping"]["channels"]["70"] == {"rhx": "A-070", "how": "expected",
                                                "transform": "identity"}
    assert rep["xor_bit_histogram"] == [0] * 16 and rep["first_mismatches"] == []


def test_the_cli_writes_the_json_and_exits_by_result(tmp_path):
    rec, rhxp = session(tmp_path)
    lines = []
    out = tmp_path / "report.json"
    assert TC.main(["compare", str(rec), str(rhxp), "--json", str(out)], out=lines.append) == 0
    rep = json.loads(out.read_text())
    assert (rep["tool"], rep["command"], rep["result"]) == ("tapcheck", "compare", "PASS")
    assert lines[-2].endswith("\nRESULT PASS") and lines[-1] == f"report {out}"


def test_lost_datagrams_are_counted_in_and_outside_the_overlap(tmp_path):
    # datagram 10 = seq 50-54 (before RHX's window), 150-151 = seq 750-759 (inside)
    rec, rhxp = session(tmp_path, drop={10, 150, 151})
    rep = T.compare(rec, [rhxp])
    ov, ch = rep["overlap"], rep["chain"]
    assert rep["result"] == "FAIL" and "MISSING_FRAMES" in cats(rep, "FAIL")
    assert (ch["datagrams"], ch["frames"], ch["seq_gaps"], ch["frames_missing"]) == \
        (597, 2985, 2, 15)
    assert (ov["frames_missing"], ov["frames_compared"]) == (10, NR - 10)
    assert ov["compared"] == ov["equal"] == (NR - 10) * 128 and ov["mismatched"] == 0
    assert rep["first_missing_seqs"] == list(range(750, 760))
    assert rep["outside"]["chain_frames_before_rhx"] == LO - 5


def test_bit_flips_are_counted_per_channel_and_per_bit(tmp_path):
    words = SIG[:N].copy()
    flips = [(1000, 5, 0x0004), (1200, 70, 0x8001), (2000, 0, 0x0100), (2000, 127, 0x0100),
             (100, 3, 0x0001)]                      # the last is outside RHX's window
    for s, k, x in flips:
        words[s, k] ^= x
    rec, rhxp = session(tmp_path, words=words)
    rep = T.compare(rec, [rhxp])
    ov = rep["overlap"]
    assert rep["result"] == "FAIL" and cats(rep, "FAIL") == {"MISMATCH"}
    assert (ov["mismatched"], ov["equal"]) == (4, NR * 128 - 4)
    assert rep["mismatches_per_channel"] == {"0": 1, "5": 1, "70": 1, "127": 1}
    assert rep["xor_bit_histogram"] == [1, 0, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1]
    assert rep["bits_flipped_histogram"] == {"1": 3, "2": 1}
    got = [(m["seq"], m["chain_ch"], m["rhx_ch"], m["rhx_timestamp"], m["xor"])
           for m in rep["first_mismatches"]]
    assert got == [(1000, 5, "A-005", 1000 + D, "0x0004"), (1200, 70, "A-070", 1200 + D, "0x8001"),
                   (2000, 0, "A-000", 2000 + D, "0x0100"), (2000, 127, "A-127", 2000 + D, "0x0100")]
    m = rep["first_mismatches"][1]
    assert m["expected"] == f"0x{int(SIG[1200, 70]):04x}"
    assert m["got"] == f"0x{int(SIG[1200, 70]) ^ 0x8001:04x}"


def test_a_chip_swap_is_found_reported_and_other_errors_stay_visible(tmp_path):
    words = np.concatenate([SIG[:N, 64:], SIG[:N, :64]], axis=1)   # MISO1/MISO2 swapped at the tap
    words[1000, 69] ^= 0x0002                                        # RHX A-005, heard on chain 69
    rec, rhxp = session(tmp_path, words=words, fmt="fps")
    rep = T.compare(rec, [rhxp])
    assert rep["result"] == "FAIL" and cats(rep, "FAIL") == {"MAPPING", "MISMATCH"}
    assert rep["mapping"]["discovered"] == ["chain 0-63 = RHX A-064..A-127",
                                            "chain 64-127 = RHX A-000..A-063"]
    assert rep["overlap"]["mismatched"] == 1 and rep["mismatches_per_channel"] == {"69": 1}
    assert rep["first_mismatches"][0]["rhx_ch"] == "A-005"


def test_an_arbitrary_permutation_is_recovered_channel_by_channel(tmp_path):
    perm = np.random.default_rng(7).permutation(128)
    rec, rhxp = session(tmp_path, words=SIG[:N][:, perm])
    rep = T.compare(rec, [rhxp])
    chans = rep["mapping"]["channels"]
    assert all(chans[str(k)]["rhx"] == f"A-{perm[k]:03d}" for k in range(128))
    assert rep["overlap"]["mismatched"] == 0 and "MAPPING" in cats(rep, "FAIL")


def test_module_halves_traded_within_a_chip_read_as_runs(tmp_path):
    words = SIG[:N].copy()
    words[:, :32], words[:, 32:64] = SIG[:N, 32:64], SIG[:N, :32]
    rec, rhxp = session(tmp_path, words=words)
    rep = T.compare(rec, [rhxp])
    assert rep["mapping"]["discovered"] == ["chain 0-31 = RHX A-032..A-063",
                                            "chain 32-63 = RHX A-000..A-031"]


def test_two_complement_words_are_reported_as_a_convention_not_as_errors(tmp_path):
    rec, rhxp = session(tmp_path, words=SIG[:N] ^ np.uint16(0x8000))
    rep = T.compare(rec, [rhxp])
    assert rep["convention"]["by_transform"] == {"top bit flipped": "0-127"}
    assert not rep["convention"]["identity_everywhere"]
    assert cats(rep, "FAIL") == {"CONVENTION"} and rep["mapping"]["as_expected"]
    assert rep["overlap"]["mismatched"] == 0


def test_one_inverted_miso_pair_shows_as_that_chips_convention(tmp_path):
    words = SIG[:N].copy()
    words[:, 64:] ^= np.uint16(0xFFFF)
    rec, rhxp = session(tmp_path, words=words)
    rep = T.compare(rec, [rhxp])
    assert rep["convention"]["by_transform"] == {"identity": "0-63", "inverted": "64-127"}
    assert rep["overlap"]["mismatched"] == 0 and cats(rep, "FAIL") == {"CONVENTION"}


def test_partial_frames_count_unheard_channels_apart_from_wrong_ones(tmp_path):
    words = SIG[:N].copy()
    flags = np.full(N, C.FLAG_TAP)
    flags[1500:1505] |= C.FLAG_TAP_PARTIAL
    words[1500:1505, 10:20] = 0                     # not heard: read 0
    words[1502, 30] ^= 1                            # heard, but wrong
    rec, rhxp = session(tmp_path, words=words, flags=flags)
    rep = T.compare(rec, [rhxp])
    ov = rep["overlap"]
    assert cats(rep, "FAIL") == {"PARTIAL_FRAMES"}
    assert (ov["frames_partial"], ov["frames_compared"]) == (5, NR - 5)
    assert (ov["partial_compared"], ov["partial_unheard"], ov["partial_wrong"],
            ov["partial_equal"]) == (640, 50, 1, 589)
    assert ov["mismatched"] == 0 and rep["chain"]["partial_frames"] == 5


def test_rhx_gaps_are_rhxs_own_and_leave_chain_frames_unchecked(tmp_path):
    seqs = np.r_[np.arange(LO, 1100), np.arange(1110, LO + NR)]
    rec, rhxp = session(tmp_path, seqs=seqs, fmt="fps")
    rep = T.compare(rec, [rhxp])
    ov = rep["overlap"]
    assert cats(rep, "FAIL") == {"RHX_GAPS"}
    assert (ov["overlap_samples"], ov["rhx_gaps"], ov["rhx_gap_samples"],
            ov["chain_frames_without_rhx_sample"], ov["frames_missing"]) == (NR - 10, 1, 10, 10, 0)
    assert ov["mismatched"] == 0 and rep["rhx"]["gaps"] == 1


def test_rhx_samples_before_the_chain_recording_are_reported_not_failed(tmp_path):
    # The chain started 200 sweeps after RHX began saving.
    rec, rhxp = session(tmp_path, words=SIG[200:200 + N], seqs=np.arange(NR),
                        t=np.arange(NR) - 200 + D)
    rep = T.compare(rec, [rhxp])
    assert rep["result"] == "PASS" and "RHX_OUTSIDE" in cats(rep, "WARN")
    assert rep["outside"]["rhx_samples_before_chain"] == 200
    assert rep["overlap"]["overlap_samples"] == rep["overlap"]["frames_compared"] == NR - 200
    assert rep["alignment"]["rhx_timestamp_minus_chain_seq"] == D


def test_stale_frames_from_the_previous_run_are_dropped(tmp_path):
    rec, rhxp = session(tmp_path, stale=[(9998, SIG[3000]), (9999, SIG[3001])])
    rep = T.compare(rec, [rhxp])
    assert rep["result"] == "PASS" and "STALE" in cats(rep, "INFO")
    assert (rep["chain"]["stale_frames"], rep["chain"]["first_seq"], rep["chain"]["last_seq"]) \
        == (2, 0, N - 3)


def test_frames_without_the_tap_flag_fail(tmp_path):
    flags = np.full(N, C.FLAG_TAP)
    flags[1600:1610] = 0
    rec, rhxp = session(tmp_path, flags=flags)
    rep = T.compare(rec, [rhxp])
    assert cats(rep, "FAIL") == {"NOT_TAP"}
    assert (rep["overlap"]["frames_not_tap"], rep["chain"]["not_tap_frames"]) == (10, 10)


def test_duplicates_and_malformed_datagrams_are_accounted_for(tmp_path):
    dg = RF.datagrams(SIG[:N])
    bad = bytearray(dg[301])
    bad[C.HDR_BYTES + 3] ^= 0xFF                           # a different payload, same seq
    rec, rhxp = session(tmp_path, extra=[dg[300], b"garbage", bytes(bad)])
    rep = T.compare(rec, [rhxp])
    assert (rep["chain"]["duplicates"], rep["chain"]["malformed_datagrams"]) == (10, 1)
    assert {"DUPLICATES"} <= cats(rep, "FAIL") and "MALFORMED" in cats(rep, "WARN")
    assert rep["overlap"]["mismatched"] == 0      # the first copy is the one compared


def test_unrelated_recordings_cannot_be_aligned(tmp_path):
    rec, rhxp = session(tmp_path, codes=RF.signal(NR, seed=99))
    with pytest.raises(T.CompareError, match="no consistent offset"):
        T.compare(rec, [rhxp])
    lines = []
    assert TC.main(["compare", str(rec), str(rhxp), "--json", str(tmp_path / "r.json")],
                   out=lines.append) == 2
    assert lines[0].startswith("cannot compare: no consistent offset")


def test_a_skipped_sweep_shows_as_an_inconsistent_offset(tmp_path):
    words = SIG[:N].copy()
    words[2000:] = SIG[2001:N + 1]                  # the tap lost one sweep and carried on
    rec, rhxp = session(tmp_path, words=words)
    rep = T.compare(rec, [rhxp])
    assert "OFFSET_INCONSISTENT" in cats(rep, "FAIL")
    assert rep["alignment"]["rhx_timestamp_minus_chain_seq"] == D        # the majority
    assert rep["overlap"]["mismatched"] > 0.9 * (LO + NR - 2000) * 128


def test_rhx_notch_and_software_reference_are_warned_about(tmp_path):
    rec, rhxp = session(tmp_path, hdr={"notch_mode": 2, "reference": "A-031"})
    rep = T.compare(rec, [rhxp])
    assert {"RHX_NOTCH", "RHX_REFERENCE"} <= cats(rep, "WARN") and rep["result"] == "PASS"


def test_several_traditional_files_are_one_recording(tmp_path):
    rec, _ = session(tmp_path)
    seqs = np.arange(LO, LO + NR)
    a, b = tmp_path / "s_1.rhd", tmp_path / "s_2.rhd"
    RF.write_traditional(a, SIG[seqs[:1024]], seqs[:1024] + D)
    RF.write_traditional(b, SIG[seqs[1024:]], seqs[1024:] + D)
    rep = T.compare(rec, [b, a])
    assert rep["result"] == "PASS" and rep["overlap"]["compared"] == NR * 128


def test_an_rhx_port_with_one_chip_checks_those_64_channels(tmp_path):
    seqs = np.arange(LO, LO + NR)
    groups = [RF.amp_port("A", 64), dict(RF.amp_port("B", 4), name="Port B")]
    codes = np.concatenate([SIG[seqs][:, :64], RF.signal(NR, 4, seed=5)], axis=1)
    rec, rhxp = session(tmp_path, codes=codes, groups=groups)
    rep = T.compare(rec, [rhxp])
    assert rep["result"] == "PASS" and rep["rhx"]["port"] == "A"
    assert rep["alignment"]["fingerprint_channels"] == "expected channels"
    assert "UNRECORDED" in cats(rep, "WARN") and "RHX_OTHER_PORTS" in cats(rep, "INFO")
    assert rep["mapping"]["unrecorded"] == list(range(64, 128))
    assert rep["overlap"]["compared"] == NR * 64


def test_a_truncated_last_record_is_ignored(tmp_path):
    rec, rhxp = session(tmp_path)
    with open(rec, "ab") as f:
        f.write(taprec.RECORD.pack(1.0, 1392) + b"\x00" * 100)
    rep = T.compare(rec, [rhxp])
    assert rep["result"] == "PASS" and "TRUNCATED" in cats(rep, "INFO")


# ---------------------------------------------------------------------------
# The fake board's tap registers
# ---------------------------------------------------------------------------

def test_fake_board_tap_registers_follow_section_4():
    board, clock = TC.fake_board(["tap_idle"])           # nothing on the bus yet
    c = Ctrl(ser=board)
    caps = TC.D.decode_caps(c.read("CAPS"))
    assert caps.tap and caps.emulator
    assert c.read("TAP_DELAY") == C.TAP_DELAY_RESET and c.read("TAP_SCLK_MIN") == 0xFFFFFFFF
    c.write("TAP_DELAY", 200)
    assert c.read("TAP_DELAY") == C.TAP_DELAY_MAX                 # saturates
    board.f.bus.present = True                                     # RHX starts running
    clock.sleep(0.1)
    # The tap decodes whatever MODE says; only sweep assembly needs bit4 and run.
    assert c.read("TAP_STAT") & 0x11F00 == 0x01F00
    assert c.read("TAP_CHECKS") > 150 and c.read("TAP_SWEEPS") == 0
    c.write("MODE", TC.MODE_TAP)
    c.set_run(True)
    clock.sleep(0.5)
    assert c.read("TAP_STAT") & 0x11F00 == 0x11F00
    assert c.read("TAP_SWEEPS") > 900 and c.read("TAP_SCLK_MIN") == 34
    board.f.bus.present = False                                    # RHX stops
    c.clear_counters()
    assert (c.read("TAP_SCLK_MIN"), c.read("TAP_STAT") & 0x1F00, c.read("TAP_CYCLES")) == \
        (0xFFFFFFFF, 0, 0)
    assert TC.D.decode_caps(Ctrl(ser=TC.fake_board(["no_tap"])[0]).read("CAPS")).tap is False
    assert Ctrl(ser=TC.fake_board(["no_tap"])[0]).read("TAP_STAT") == 0


def test_without_a_tap_tap_delay_answers_err_and_status_never_asks():
    from chain.ctrl import CtrlError
    board = TC.fake_board(["no_tap"])[0]
    c = Ctrl(ser=board)
    for op in (lambda: c.read("TAP_DELAY"), lambda: c.write("TAP_DELAY", 8)):
        with pytest.raises(CtrlError, match="rejected"):
            op()
    rc, found, _l, board = run_status(["no_tap"])
    assert rc == 1 and found == {"FAIL": {"NO_TAP"}}
    assert not [l for l in board.log if l.startswith(("R 05", "R 3", "W 05"))]


def test_fakeserial_knows_the_tap_registers_too():
    c = Ctrl(ser=FakeSerial())
    assert c.read("TAP_DELAY") == 8 and c.read("TAP_SCLK_MIN") == 0xFFFFFFFF
    c.write("TAP_DELAY", 64)
    assert c.read("TAP_DELAY") == 63
    c.ser.regs[C.REG["TAP_CYCLES"]] = 5
    c.ser.regs[C.REG["TAP_SCLK_MIN"]] = 34
    c.clear_counters()
    assert (c.read("TAP_CYCLES"), c.read("TAP_SCLK_MIN")) == (0, 0xFFFFFFFF)
    for name in ("TAP_STAT", "TAP_CHECKS", "TAP_SCLK_MIN"):
        with pytest.raises(Exception, match="rejected"):
            c.write(name, 1)


# ---------------------------------------------------------------------------
# status
# ---------------------------------------------------------------------------

def run_status(faults, listen=1.0, before=None, board=None):
    board, clock = (board, board.clock) if board is not None else TC.fake_board(faults)
    ctrl = Ctrl(ser=board)
    if before:
        before(ctrl)
    args = TC.build_parser().parse_args(["status", "--fake", "--listen", str(listen)])
    args.settle = 0.0
    lines = []
    rc = TC.cmd_status(ctrl, args, out=lines.append, sleep=clock.sleep)
    found = {}
    for line in lines:
        m = re.match(r"\s+\[(\w+)\] (\w+):", line)
        if m:
            found.setdefault(m.group(1), set()).add(m.group(2))
    return rc, found, lines, board


@pytest.mark.parametrize("faults,status,expect", [
    (["tap"], "FAIL", set()),
    (["tap_idle"], "FAIL", {"BUS_IDLE"}),
    (["tap_cs_swapped"], "FAIL", {"FRAMING_ALL"}),
    (["tap_sclk_open"], "FAIL", {"SCLK_SILENT", "FRAMING_ALL"}),
    (["tap_sclk_swapped"], "FAIL", {"NO_SWEEPS"}),
    (["tap_mosi_open"], "FAIL", {"MOSI_SILENT", "ALL_PARTIAL"}),
    (["tap_miso2_open"], "FAIL", {"MISO2_SILENT", "CHIP1_ERRORS"}),
    (["tap_miso1_swapped"], "FAIL", {"CHIP0_ERRORS"}),
    (["no_tap"], "FAIL", {"NO_TAP"}),
    (["tap_framing"], "WARN", {"FRAMING_SOME", "PARTIAL"}),
    (["tap_no_checks"], "INFO", {"NO_CHECKS"}),
    (["tap_idle", "tap_cs_swapped", "tap_sclk_swapped"], "WARN", {"CS_POLARITY", "SCLK_POLARITY"}),
])
def test_status_names_each_wiring_fault(faults, status, expect):
    rc, found, _lines, _b = run_status(faults)
    assert found.get(status, set()) == expect
    assert rc == (1 if found.get("FAIL") else 0)


def test_status_reports_rate_sclk_and_the_usable_delay():
    rc, found, lines, _b = run_status(["tap"])
    text = "\n".join(lines)
    assert rc == 0 and not found.get("FAIL") and lines[-1] == "STATUS OK"
    assert "TAP_SCLK_MIN 34 clocks = 354 ns" in text and "SCLK about 1.41 MHz" in text
    assert "about 2,017 S/s" in text and "usable roughly 3..37" in text and "prefer 3..20" in text
    assert re.search(r"2,0\d\d\.\d sweeps/s heard", text)


def test_status_flags_a_delay_outside_the_window():
    rc, found, _l, _b = run_status(["tap"], before=lambda c: c.write("TAP_DELAY", 50))
    assert "DELAY_OUT" in found["FAIL"] and "DELAY_RANGE" in found["WARN"]


@pytest.mark.parametrize("mode,run", [(0, False), (TC.MODE_TAP, True), (0x4, True)])
def test_status_puts_mode_and_run_back(mode, run):
    def before(c):
        c.write("MODE", mode)
        c.set_run(run)
    _rc, _f, _l, board = run_status(["tap"], before=before)
    assert board.regs[C.REG["MODE"]] == mode and board.running == run


def test_status_without_listening_reads_registers_as_they_are():
    # The tap decodes whatever MODE says (spi_tap.sv): cycles and checks count
    # with MODE 0 and run clear; only sweeps need the tap selected and running.
    board, clock = TC.fake_board([])
    clock.sleep(0.5)
    _rc, found, lines, board = run_status(["tap"], listen=0, board=board)
    text = "\n".join(lines)
    assert not found.get("FAIL") and not board.running and board.regs[C.REG["MODE"]] == 0
    assert "TAP_SWEEPS 0," in text and "TAP_SCLK_MIN 34 clocks" in text
    assert int(re.search(r"TAP_CYCLES (\d+)", text).group(1)) > 900 * 35
    _rc, found, _l, _b = run_status(["tap_idle"], listen=0)
    assert found.get("WARN") == {"BUS_IDLE"} and not found.get("FAIL")


# ---------------------------------------------------------------------------
# delay-sweep
# ---------------------------------------------------------------------------

def run_sweep(tmp_path, faults, *extra):
    board, clock = TC.fake_board(faults)
    ctrl = Ctrl(ser=board)
    out = tmp_path / "sweep.json"
    args = TC.build_parser().parse_args(["delay-sweep", "--dwell", "0.05", "--json", str(out),
                                         *extra])
    args.settle = 0.0
    lines = []
    rc = TC.cmd_delay_sweep(ctrl, args, out=lines.append, sleep=clock.sleep)
    return rc, json.loads(out.read_text()), lines, board


def test_delay_sweep_picks_the_centre_of_the_clean_window_and_restores(tmp_path):
    rc, rep, lines, board = run_sweep(tmp_path, ["tap"])
    assert rc == 0 and (rep["chosen"], rep["window"]) == (16, [3, 30])
    assert [r["delay"] for r in rep["rows"]] == list(range(64))
    assert [r["clean"] for r in rep["rows"]] == [3 <= d <= 30 for d in range(64)]
    assert board.regs[C.REG["TAP_DELAY"]] == C.TAP_DELAY_RESET     # not --apply
    assert board.regs[C.REG["MODE"]] == 0 and not board.running
    assert "clean window TAP_DELAY 3..30; centre 16" in lines


def test_delay_sweep_apply_writes_the_choice(tmp_path):
    rc, rep, _l, board = run_sweep(tmp_path, ["tap"], "--apply", "--from", "0", "--to", "40",
                                   "--step", "2")
    assert rep["window"] == [4, 30]                    # the even delays inside 3..30
    assert rc == 0 and rep["applied"] and board.regs[C.REG["TAP_DELAY"]] == rep["chosen"] == 17


def test_delay_sweep_without_known_answers_says_compare_must_judge(tmp_path):
    rc, rep, lines, board = run_sweep(tmp_path, ["tap_no_checks"], "--to", "10")
    assert rc == 0 and rep["chosen"] is None
    assert any("TAP_CHECKS never climbed" in l for l in lines)
    assert board.regs[C.REG["TAP_DELAY"]] == C.TAP_DELAY_RESET


def test_delay_sweep_with_a_dead_miso_finds_no_window(tmp_path):
    rc, rep, _l, _b = run_sweep(tmp_path, ["tap_miso2_open"], "--to", "10")
    assert rc == 1 and rep["chosen"] is None
    assert all(r["errors"][2] and not r["errors"][0] for r in rep["rows"] if 3 <= r["delay"])


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------

def run_selftest(tmp_path, damage=None, faults=()):
    board, clock = TC.fake_board(list(faults))
    ctrl = Ctrl(ser=board)
    out = tmp_path / "selftest.json"
    args = TC.build_parser().parse_args(["selftest", "--seconds", "1", "--json", str(out)])
    args.settle = 0.0
    cap = TC.fake_selftest_capture(board, clock, TC._playback())

    def capture(s):
        raw = cap(s)
        return damage(raw) if damage else raw
    lines = []
    rc = TC.cmd_selftest(ctrl, args, out=lines.append, sleep=clock.sleep, capture=capture)
    rep = json.loads(out.read_text())
    return rc, {c["check"]: c["status"] for c in rep["checks"]}, rep, board


def test_selftest_passes_on_a_healthy_tap_and_restores_the_board(tmp_path):
    rc, checks, rep, board = run_selftest(tmp_path)
    assert rc == 0 and set(checks.values()) == {"PASS"} and len(checks) == 11
    assert rep["metrics"]["accuracy"]["value_errors"] == 0
    assert rep["metrics"]["loss"]["frames_expected"] == 2000
    assert board.regs[C.REG["MODE"]] == 0 and board.regs[C.REG["SPI_DIV"]] == 0
    assert not board.running


def _edit(raw, i, offset, xor):
    t, d = raw[i]
    b = bytearray(d)
    b[offset] ^= xor
    raw[i] = (t, bytes(b))
    return raw


@pytest.mark.parametrize("damage,failing", [
    (lambda raw: raw[:10] + raw[11:], {"seq contiguous"}),
    (lambda raw: _edit(raw, 5, C.HDR_BYTES + 2 * 17, 0x01), {"values = on-die emulator"}),
    (lambda raw: _edit(raw, 7, 3, C.FLAG_TAP), {"FLAG_TAP on every frame"}),
    (lambda raw: _edit(raw, 7, 3, C.FLAG_TAP_PARTIAL), {"no TAP_PARTIAL"}),
])
def test_selftest_names_what_is_wrong(tmp_path, damage, failing):
    rc, checks, _rep, _b = run_selftest(tmp_path, damage)
    assert rc == 1 and {k for k, v in checks.items() if v == "FAIL"} == failing


def test_selftest_needs_the_on_die_emulator(tmp_path):
    lines = []
    rc = TC.main(["selftest", "--fake", "no_emulator"], out=lines.append)
    assert rc == 2 and "CAPS bit8 clear" in lines[-1]


# ---------------------------------------------------------------------------
# record
# ---------------------------------------------------------------------------

def _sender(port, payloads, gap=0.002):
    def send():
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for p in payloads:
            tx.sendto(p, ("127.0.0.1", port))
            time.sleep(gap)
        tx.close()
    th = threading.Thread(target=send, daemon=True)
    th.start()
    return th


def run_record(tmp_path, payloads, *extra, board=None):
    from chain import receiver
    board = board or TC.fake_board([])[0]
    ctrl = Ctrl(ser=board)
    sock = receiver.open_socket(port=0)
    port = sock.getsockname()[1]
    args = TC.build_parser().parse_args(["record", "--serial", "fake", "--out", str(tmp_path),
                                         "--no-progress", "--drain", "0.2", *extra])
    args.settle = 0.0
    th = _sender(port, payloads)
    lines = []
    rc = TC.cmd_record(ctrl, args, out=lines.append, sock=sock)
    th.join(timeout=5)
    sock.close()
    path = next(tmp_path.glob("tap_*" + taprec.SUFFIX))
    return rc, path, lines, board


def test_record_keeps_every_datagram_and_the_counters(tmp_path):
    payloads = RF.datagrams(SIG[:200])
    rc, path, lines, board = run_record(tmp_path, payloads, "--seconds", "1.0", "--delay", "12")
    _head, got, trailing = taprec.read(path)
    assert rc == 0 and [d for _t, d in got] == payloads and trailing == 0
    assert all(a <= b for (a, _x), (b, _y) in zip(got, got[1:]))
    meta = json.loads(taprec.sidecar(path).read_text())
    assert meta["datagrams"] == len(payloads) and not meta["interrupted"]
    assert meta["settings"]["MODE"] == TC.MODE_TAP and meta["settings"]["TAP_DELAY"] == 12
    assert {"FRAMES", "DGRAMS", "OVERRUNS", "STALLS", "TAP_SWEEPS", "TAP_PARTIAL",
            "TAP_SCLK_MIN"} <= set(meta["counters"])
    assert not board.running and board.regs[C.REG["TAP_DELAY"]] == 12
    assert any(str(path) in l for l in lines)


def test_record_stops_cleanly_on_ctrl_c(tmp_path):
    done = threading.Event()

    def interrupt():
        while not done.wait(0.25):
            os.kill(os.getpid(), signal.SIGINT)
    old = signal.signal(signal.SIGINT, lambda *a: None)    # a stray one must not end pytest
    th = threading.Thread(target=interrupt, daemon=True)
    th.start()
    try:
        rc, path, _l, board = run_record(tmp_path, RF.datagrams(SIG[:50]))
    finally:
        done.set()
        th.join()
        signal.signal(signal.SIGINT, old)
    meta = json.loads(taprec.sidecar(path).read_text())
    assert meta["interrupted"] and not board.running
    assert taprec.read(path)[2] == 0                       # no half-written record


def test_record_then_compare_end_to_end(tmp_path):
    words = SIG[:1000]
    rc, path, _l, _b = run_record(tmp_path, RF.datagrams(words), "--seconds", "1.5")
    seqs = np.arange(300, 300 + 512)
    rhxp = tmp_path / "rhx_fps"
    RF.write_per_signal_type(rhxp, SIG[seqs], seqs + D)
    rep = T.compare(path, [rhxp])
    assert rep["result"] == "PASS" and rep["overlap"]["compared"] == 512 * 128
    assert rep["chain"]["sidecar"]["settings"]["MODE"] == TC.MODE_TAP


def test_record_refuses_a_bitstream_without_the_tap(tmp_path):
    board = TC.fake_board(["no_tap"])[0]
    with pytest.raises(TC.Abort, match="bit9 clear"):
        run_record(tmp_path, [], "--seconds", "0.1", board=board)
