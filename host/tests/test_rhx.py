"""chain.rhx against files written to Intan's layout, and against real RHX files when present.

Real files: set INTAN_RHX_SAMPLES to a folder holding, from the NeuralEnsemble
GIN repository (gin.g-node.org/NeuralEnsemble/ephy_testing_data, folder intan):

    trad/intan_rhd_test_1.rhd                                  Traditional, v1.5, 60/block
    split/time_split_with_digital_stream_240205_120954.rhd     Traditional, v3.3, 128/block
    fps/{info.rhd, time.dat, amplifier.dat}                    from intan_fps_test_231117_052500

The expected numbers below were read from those files with Intan's own
importrhdutilities.py, which agreed with chain.rhx on every header field,
timestamp and amplifier code. Without the folder those tests skip.
"""
import os
import pathlib

import pytest

np = pytest.importorskip("numpy")

import rhxfake as RF  # noqa: E402

from chain import rhx  # noqa: E402


def full_groups():
    """A port with a disabled channel, a disabled port, and every other signal type."""
    port = RF.amp_port("A", 8, disabled=(3,))
    off = dict(RF.amp_port("B", 4), enabled=False)
    aux = RF.board_group("A-AUX", "Aux", 1, 3)
    sup = RF.board_group("A-VDD", "Supply", 2, 1)
    adc = RF.board_group("ANALOG-IN", "Board ADC", 3, 2)
    din = RF.board_group("DIGITAL-IN", "Digital in", 4, 2)
    dout = RF.board_group("DIGITAL-OUT", "Digital out", 5, 1)
    return [port, off, aux, sup, adc, din, dout]


def test_header_round_trips_every_field(tmp_path):
    groups = full_groups()
    codes = RF.signal(256, 7)
    t = np.arange(256) + 1000
    p = tmp_path / "x.rhd"
    RF.write_traditional(p, codes, t, groups, notes=("one", "", "three"), reference="Hardware",
                         notch_mode=2, temp=2, board_mode=13, sample_rate=2000.0)
    h = rhx.read_header_file(p)
    assert h.version == (3, 3) and h.samples_per_block == 128 and h.sample_rate == 2000.0
    assert h.notes == ("one", "", "three") and h.reference_channel == "Hardware"
    assert h.notch_hz == 60 and h.num_temp_sensor_channels == 2 and h.eval_board_mode == 13
    assert (h.dsp_enabled, h.actual_lower_bandwidth, h.actual_upper_bandwidth) == (1, 1.0, 500.0)
    # The disabled port's channels are not in the header at all; the disabled
    # channel is, marked, and is not a data column.
    assert [c.native_name for c in h.amplifier_channels] == \
        ["A-000", "A-001", "A-002", "A-004", "A-005", "A-006", "A-007"]
    assert [c.native_name for c in h.channels if not c.enabled] == ["A-003"]
    assert [g.prefix for g in h.groups] == ["A", "B", "A-AUX", "A-VDD", "ANALOG-IN",
                                             "DIGITAL-IN", "DIGITAL-OUT"]
    assert not h.groups[1].enabled
    a5 = h.amplifier_channels[4]
    assert (a5.native_order, a5.chip_channel, a5.board_stream, a5.port_prefix,
            a5.port_number) == (5, 5, 0, "A", 1)
    assert h.block_dtype().itemsize == h.bytes_per_block()


def test_traditional_codes_and_timestamps_come_back_exactly(tmp_path):
    groups = full_groups()
    codes = RF.signal(384, 7)
    t = np.arange(384) + 123456
    p = tmp_path / "x.rhd"
    RF.write_traditional(p, codes, t, groups, temp=1)
    rec = rhx.load(p)
    assert rec.format == rhx.TRADITIONAL and rec.n == 384 and not rec.warnings
    assert np.array_equal(rec.t, t)
    assert np.array_equal(rec.codes(0, rec.n), codes)
    for a, b in ((0, 1), (127, 129), (100, 300), (383, 384), (250, 999)):
        assert np.array_equal(rec.codes(a, b), codes[a:min(b, 384)])
    assert rec.gaps() == []


def test_old_traditional_files_have_60_sample_blocks_and_unsigned_timestamps(tmp_path):
    groups = [RF.amp_port("A", 4)]
    codes = RF.signal(120, 4)
    t = np.arange(120, dtype=np.int64) + (1 << 31) + 5      # only fits unsigned
    p = tmp_path / "old.rhd"
    RF.write_traditional(p, codes, t, groups, version=(1, 0))
    rec = rhx.load(p)
    h = rec.header
    assert h.samples_per_block == 60 and not h.timestamps_signed
    assert h.reference_channel is None and h.num_temp_sensor_channels == 0
    assert np.array_equal(rec.t, t) and np.array_equal(rec.codes(0, 120), codes)


def test_per_signal_type_folder_reads_by_folder_or_by_info_rhd(tmp_path):
    codes = RF.signal(1000, 16)
    t = np.arange(1000) + 50
    folder = tmp_path / "session_241102_120000"
    RF.write_per_signal_type(folder, codes, t, [RF.amp_port("A", 16)])
    (folder / "settings.xml").write_text("<xml/>")
    for where in (folder, folder / "info.rhd"):
        rec = rhx.load([where])
        assert rec.format == rhx.PER_SIGNAL_TYPE and rec.n == 1000
        assert np.array_equal(rec.codes(0, 1000), codes) and np.array_equal(rec.t, t)


# One File Per Signal Type stores int16 v = code - 32768 (x 0.195 uV); the
# reader hands back the code the chip sent.
@pytest.mark.parametrize("stored,code", [(-32768, 0x0000), (-1, 0x7FFF), (0, 0x8000),
                                          (1, 0x8001), (32767, 0xFFFF)])
def test_amplifier_dat_values_map_to_offset_binary_codes(tmp_path, stored, code):
    folder = tmp_path / "f"
    RF.write_per_signal_type(folder, np.full((3, 1), code, np.uint16), [0, 1, 2],
                             [RF.amp_port("A", 1)])
    raw = np.fromfile(folder / "amplifier.dat", "<i2")
    assert raw.tolist() == [stored] * 3
    assert rhx.load(folder).codes(0, 3)[:, 0].tolist() == [code] * 3


def test_a_partial_last_block_is_dropped_with_a_warning(tmp_path):
    codes = RF.signal(256, 4)
    p = tmp_path / "x.rhd"
    RF.write_traditional(p, codes, np.arange(256), [RF.amp_port("A", 4)])
    with open(p, "ab") as f:
        f.write(b"\x00" * 100)
    rec = rhx.load(p)
    assert rec.n == 256 and "100 bytes after the last whole data block" in rec.warnings[0]


def test_per_signal_type_files_of_unequal_length_use_the_common_part(tmp_path):
    folder = tmp_path / "f"
    RF.write_per_signal_type(folder, RF.signal(100, 4), np.arange(100), [RF.amp_port("A", 4)])
    with open(folder / "time.dat", "ab") as f:
        f.write(np.arange(100, 103, dtype="<i4").tobytes())
    rec = rhx.load(folder)
    assert rec.n == 100 and "time.dat holds 103 samples" in rec.warnings[0]


def test_one_file_per_channel_and_foreign_files_are_refused(tmp_path):
    folder = tmp_path / "fpc"
    folder.mkdir()
    (folder / "info.rhd").write_bytes(RF.header_bytes([RF.amp_port("A", 2)]))
    (folder / "amp-A-000.dat").write_bytes(b"\x00\x00" * 4)
    with pytest.raises(rhx.RhxError, match="One File Per Channel"):
        rhx.load(folder)
    rhs = tmp_path / "x.rhs"
    rhs.write_bytes(b"\xac\x84\xb1\xd6" + b"\x00" * 64)
    with pytest.raises(rhx.RhxError, match="not an Intan RHD file"):
        rhx.load(rhs)
    with pytest.raises(rhx.RhxError, match="no such file"):
        rhx.load(tmp_path / "nope.rhd")


def test_segments_are_joined_in_timestamp_order_and_must_agree(tmp_path):
    codes = RF.signal(512, 8)
    t = np.arange(512) + 9000
    a, b = tmp_path / "s_120000.rhd", tmp_path / "s_120100.rhd"
    RF.write_traditional(a, codes[:256], t[:256], [RF.amp_port("A", 8)])
    RF.write_traditional(b, codes[256:], t[256:], [RF.amp_port("A", 8)])
    rec = rhx.load([b, a])                                  # given out of order
    assert np.array_equal(rec.t, t)
    assert np.array_equal(rec.codes(200, 300), codes[200:300])   # across the join
    c = tmp_path / "other.rhd"
    RF.write_traditional(c, RF.signal(128, 4), np.arange(128) + 99999, [RF.amp_port("A", 4)])
    with pytest.raises(rhx.RhxError, match="different set of amplifier channels"):
        rhx.load([a, c])


def test_gaps_name_the_row_after_each_jump_and_how_many_samples_are_missing(tmp_path):
    t = np.r_[np.arange(0, 50), np.arange(60, 100), np.arange(101, 111)]
    folder = tmp_path / "g"
    RF.write_per_signal_type(folder, RF.signal(len(t), 2), t, [RF.amp_port("A", 2)])
    assert rhx.load(folder).gaps() == [(50, 10), (90, 1)]


# --- real RHX / RHD files -------------------------------------------------------

SAMPLES = os.environ.get("INTAN_RHX_SAMPLES")
real = pytest.mark.skipif(not SAMPLES, reason="set INTAN_RHX_SAMPLES to the GIN intan test files")


def _real(rel):
    p = pathlib.Path(SAMPLES) / rel
    if not p.exists():
        pytest.skip(f"{p} not present")
    return p


@real
def test_real_traditional_v1_5_file():
    rec = rhx.load(_real("trad/intan_rhd_test_1.rhd"))
    h = rec.header
    assert (h.version, h.samples_per_block, h.sample_rate) == ((1, 5), 60, 30000.0)
    assert len(rec.channels) == 192 and h.notch_hz == 50 and h.header_bytes == 16226
    assert (len(h.enabled(rhx.SUPPLY_VOLTAGE)), len(h.enabled(rhx.BOARD_ADC)),
            len(h.enabled(rhx.BOARD_DIG_IN))) == (6, 8, 8)
    assert rec.n == 30000 and (rec.t[0], rec.t[-1]) == (0, 29999) and rec.gaps() == []
    codes = rec.codes(0, rec.n)
    assert int(codes.astype(np.int64).sum()) == 188803519826
    assert codes[0, :4].tolist() == [32918, 32671, 32668, 32677]
    assert codes[-1, -2:].tolist() == [32711, 32696]


@real
def test_real_traditional_v3_3_file_with_aux_and_digital_inputs():
    rec = rhx.load(_real("split/time_split_with_digital_stream_240205_120954.rhd"))
    h = rec.header
    assert (h.version, h.samples_per_block, h.sample_rate) == ((3, 3), 128, 30000.0)
    assert [c.native_name for c in rec.channels][:2] == ["B-000", "B-001"]
    assert (len(rec.channels), len(h.enabled(rhx.AUX_INPUT)), len(h.enabled(rhx.BOARD_DIG_IN))) \
        == (32, 3, 3)
    assert rec.n == 6016 and (rec.t[0], rec.t[-1]) == (138880, 144895)
    codes = rec.codes(0, rec.n)
    assert int(codes.astype(np.int64).sum()) == 6578257254
    assert codes[0, :4].tolist() == [35050, 35000, 35066, 34956]
    assert codes[-1, -2:].tolist() == [33263, 33192]


@real
def test_real_one_file_per_signal_type_folder():
    rec = rhx.load(_real("fps"))
    h = rec.header
    assert rec.format == rhx.PER_SIGNAL_TYPE and h.version == (3, 3) and h.sample_rate == 30000.0
    assert len(rec.channels) == 64 and h.reference_channel == "n/a"
    assert rec.n == 24320 and (rec.t[0], rec.t[-1]) == (0, 24319) and rec.gaps() == []
    codes = rec.codes(0, rec.n)
    assert int(codes.astype(np.int64).sum()) == 51105507938
    assert codes[0, :4].tolist() == [32582, 32557, 32629, 32624]
    # amplifier.dat is code - 32768: microvolts centre on zero, as recorded EMG should
    uv = (codes.astype(np.int64) - 32768) * rhx.SCALE_UV
    assert abs(float(np.median(uv))) < 100
