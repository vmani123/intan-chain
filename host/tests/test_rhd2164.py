"""Chip knowledge: command words, result words, register recipe, sequences."""
import math

import pytest

from chain import contract as C
from chain import rhd2164 as R


def test_command_words_match_the_contract_table():
    assert R.convert(0) == 0x0000
    assert R.convert(31) == 0x1F00
    assert R.convert(5, h=1) == 0x0501
    assert R.read(63) == 0xFF00
    assert R.read(40) == 0xE800
    assert R.write(4, 0xD8) == 0x84D8
    assert R.CALIBRATE == 0x5500 and R.CLEAR == 0x6A00
    with pytest.raises(ValueError):
        R.read(64)
    with pytest.raises(ValueError):
        R.write(1, 256)


def test_kind_and_describe_round_trip_the_encoders():
    assert R.kind(R.convert(7)) == "CONVERT" and R.describe(R.convert(7)) == "CONVERT(7)"
    assert R.kind(R.read(59)) == "READ" and R.describe(R.read(59)) == "READ(59)"
    assert R.describe(R.write(4, 0xD8)) == "WRITE(4, 0xd8)"
    assert R.kind(R.CALIBRATE) == "CALIBRATE" and R.kind(R.CLEAR) == "CLEAR"
    assert R.kind(0x4000) == "OTHER"


def test_result_decoders():
    assert R.read_result(0x0035) == 0x35
    assert R.read_result(0xFF35) is None          # a WRITE echo is not a READ result
    assert R.write_echo(0xFFDE) == 0xDE
    assert R.write_echo(0x00DE) is None
    assert R.to_signed(0xFFFF) == -1 and R.to_signed(0x8000) == -32768
    assert R.to_uv(100) == pytest.approx(19.5)


def test_rom_expectations_follow_the_contract():
    assert [R.ROM_A[r] for r in range(40, 45)] == [ord(c) for c in "INTAN"]
    assert R.ROM_A[59] == 53 and R.ROM_B[59] == 58
    assert R.ROM_A[61] == 1 and R.ROM_A[62] == 64 and R.ROM_A[63] == 4
    assert 60 not in R.ROM_A and 60 not in R.ROM_B   # die revision: informational only
    exp = R.rom_expectations()
    cmds = R.rom_commands()
    for module, rom in (("A", R.ROM_A), ("B", R.ROM_B)):
        for idx, word in exp[module].items():
            assert cmds[idx] == R.read(R.ROM_LIST_REGS[idx])
            assert word == rom[R.ROM_LIST_REGS[idx]]


# The RHD2000 datasheet's bandwidth tables. The prompt's spot checks are the
# first five upper rows and three lower rows; the rest are the datasheet's own.
UPPER = {7500: (22, 0, 23, 0), 1000: (46, 2, 30, 3), 500: (30, 5, 43, 6),
         300: (6, 9, 2, 11), 100: (38, 26, 5, 31),
         20000: (8, 0, 4, 0), 10000: (17, 0, 16, 0), 5000: (33, 0, 37, 0),
         3000: (3, 1, 13, 1), 2000: (27, 1, 44, 1), 1500: (1, 2, 23, 2),
         750: (41, 3, 36, 4), 250: (42, 10, 5, 13), 200: (24, 13, 7, 16), 150: (44, 17, 8, 21)}
LOWER = {10: (5, 1, 0), 1: (44, 6, 0), 0.1: (16, 60, 1),
         500: (13, 0, 0), 100: (25, 0, 0), 20: (54, 0, 0), 5: (40, 1, 0),
         2: (8, 3, 0), 0.5: (35, 17, 0), 0.25: (56, 54, 0)}


@pytest.mark.parametrize("f", sorted(UPPER))
def test_upper_bandwidth_dacs_match_the_datasheet(f):
    d = R.upper_dacs(f)
    assert (d["rh1_dac1"], d["rh1_dac2"], d["rh2_dac1"], d["rh2_dac2"]) == UPPER[f]
    assert d["actual_hz"] == pytest.approx(f, rel=0.05)


@pytest.mark.parametrize("f", sorted(LOWER))
def test_lower_bandwidth_dacs_match_the_datasheet(f):
    d = R.lower_dacs(f)
    assert (d["rl_dac1"], d["rl_dac2"], d["rl_dac3"]) == LOWER[f]
    assert d["actual_hz"] == pytest.approx(f, rel=0.1)


def test_dsp_table_is_the_closed_form():
    # k(n) = ln(2^n / (2^n - 1)) / (2 pi); the cutoff is k(n) * fs.
    for n in range(1, 16):
        x = 2.0 ** n
        assert R.DSP_K[n] == pytest.approx(math.log(x / (x - 1)) / (2 * math.pi), rel=1e-3)


def test_dsp_cutoff_picks_the_log_nearest_setting():
    n, fc = R.dsp_cutoff_code(1.0, 2000.0)
    assert n == 8 and fc == pytest.approx(1.2458)       # 0.62 Hz (n=9) is further in log
    n, fc = R.dsp_cutoff_code(1.0, 30000.0)              # Rhythm's default rate
    assert n == 12 and fc == pytest.approx(1.166, rel=1e-3)
    assert R.dsp_cutoff_code(1e6, 2000.0)[0] == 1
    assert R.dsp_cutoff_code(1e-6, 2000.0)[0] == 15


def test_bias_table_edges():
    assert R.bias_for(2000) == (40, 32)
    assert R.bias_for(3333.9) == (40, 32) and R.bias_for(3334) == (40, 16)
    assert R.bias_for(9000) == (18, 4)
    assert R.bias_for(15000) == (7, 3) and R.bias_for(15001) == (4, 2)


def test_biases_follow_the_in_sweep_conversion_rate():
    # Commands go out back to back, so the ADC converts at the SPI command rate
    # inside a sweep whatever the average rate: 32.65 kS/s equivalent at SPI_DIV 0.
    assert R.burst_rate(0) == pytest.approx(C.SYS_CLK_HZ / (84 * 35))
    regs = R.register_values(R.AmpConfig(rate=2000, spi_div=0))
    assert (regs[2], regs[1] & 0x3F) == (4, 2)
    regs = R.register_values(R.AmpConfig(rate=2000, spi_div=7))      # 4.08 kS/s equivalent
    assert (regs[2], regs[1] & 0x3F) == (40, 8)
    regs = R.register_values(R.AmpConfig(rate=2000, bias_rate=2000))  # explicit override
    assert (regs[2], regs[1] & 0x3F) == (40, 32)


def test_default_registers_field_by_field():
    cfg = R.AmpConfig()
    r = R.register_values(cfg)
    assert len(r) == 22 and all(0 <= v <= 0xFF for v in r)
    assert r[0] == 0xDE
    assert r[1] & 0x40                         # vddSenseEnable
    assert r[3] == 0x02
    assert r[4] & 0x80 and r[4] & 0x40         # weakMiso, twos complement
    assert not r[4] & 0x20                     # absMode off
    assert r[4] & 0x10 and r[4] & 0x0F == 8    # DSP on, ~1 Hz at 2 kS/s
    assert (r[5], r[6], r[7]) == (0x00, 0x80, 0x00)
    up, lo = R.upper_dacs(500), R.lower_dacs(1.0)
    assert r[8] == up["rh1_dac1"] and r[9] == 0x80 | up["rh1_dac2"]
    assert r[10] == up["rh2_dac1"] and r[11] == 0x80 | up["rh2_dac2"]
    assert r[12] == lo["rl_dac1"] and r[13] == 0x80 | lo["rl_dac3"] << 6 | lo["rl_dac2"]
    assert r[14:22] == [0xFF] * 8
    no_dsp = R.register_values(R.AmpConfig(dsp=False))
    assert not no_dsp[4] & 0x10


def test_very_low_lower_bandwidth_sets_rl_dac3():
    r = R.register_values(R.AmpConfig(lower=0.1))
    assert r[13] & 0x40


def test_upper_above_nyquist_warns_and_is_not_silently_changed():
    cfg = R.AmpConfig(rate=2000, upper=1000)
    warns = R.config_warnings(cfg)
    assert any("alias" in w for w in warns)
    assert R.register_values(cfg)[8:12] == R.register_values(
        R.AmpConfig(rate=30000, upper=1000))[8:12]
    assert R.config_warnings(R.AmpConfig()) == []


def test_sequences_end_with_flushers_where_results_are_wanted():
    cfg = R.AmpConfig()
    assert R.rom_commands()[-2:] == [R.FLUSH, R.FLUSH] and len(R.rom_commands()) == 12
    assert R.init_commands(cfg) == [R.write(i, v) for i, v in enumerate(R.register_values(cfg))]
    assert R.verify_commands() == [R.read(i) for i in range(22)] + [R.FLUSH] * 2
    cal = R.calibrate_commands()
    assert cal[0] == R.CALIBRATE and cal[1:10] == [R.FLUSH] * 9 and len(cal) == 14
    sw = R.convert_sweep()
    assert sw[:32] == [R.convert(c) for c in range(32)] and sw[32:] == [R.FLUSH] * 2
    assert len(R.configure_commands(cfg)) == 46 <= C.DIAG_DEPTH


def test_decode_taps_follows_the_contract_formula():
    # A bit (15 - j) = tap[p + 4j], B bit (15 - j) = tap[p + 2 + 4j].
    for p in range(C.PHASE_MAX + 1):
        for j in range(16):
            a, b = R.decode_taps(1 << (p + 4 * j), p)
            assert a == 1 << (15 - j) and b == 0
            a, b = R.decode_taps(1 << (p + 2 + 4 * j), p)
            assert a == 0 and b == 1 << (15 - j)
    with pytest.raises(ValueError):
        R.decode_taps(0, 16)


def test_decode_partial_agrees_inside_the_window_and_masks_outside():
    taps = 0x2A5A5_5A5A5_DEAD_BEEF & ((1 << C.CAP_TAPS) - 1)
    for p in range(C.PHASE_MAX + 1):
        a, am, b, bm = R.decode_partial(taps, p)
        assert (a, b) == R.decode_taps(taps, p) and am == bm == 0xFFFF
    _a, am, _b, bm = R.decode_partial(taps, 17)
    assert am == 0xFFFF and bm == 0xFFFE       # B's last bit would be tap 79
    _a, am, _b, bm = R.decode_partial(taps, 18)
    assert am == 0xFFFE and bm == 0xFFFE       # both last bits lie past tap 77
    _a, am, _b, bm = R.decode_partial(taps, -1)
    assert am == 0x7FFF and bm == 0xFFFF


def test_timing_helpers():
    assert R.sclk_hz(0) == 24e6 and R.sclk_hz(1) == 12e6
    assert R.min_sample_period(0) >= 35 * 84 and R.min_sample_period(1) >= 2 * 35 * 84
    assert R.max_rate(0) < 32654
    assert R.global_channel(1, "B", 5) == 101
