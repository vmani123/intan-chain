import pytest

from chain import contract as C
from chain.ctrl import Ctrl, CtrlError, FakeSerial


@pytest.fixture
def ctrl():
    return Ctrl(ser=FakeSerial())


def test_read_and_write_use_the_text_protocol(ctrl):
    ctrl.write("SAMPLE_PERIOD", 4800)
    assert ctrl.read("SAMPLE_PERIOD") == 4800
    assert ctrl.read(0x01) == 4800  # address or name, same register
    assert ctrl.ser.log == ["W 01 000012c0", "R 01", "R 01"]  # lowercase hex, as section 4 says


def test_check_id_passes_on_the_expected_bitstream(ctrl):
    assert ctrl.check_id() == C.ID_EXPECTED


def test_check_id_raises_on_the_wrong_bitstream(ctrl):
    ctrl.ser.regs[C.REG["ID"]] = 0xDEADBEEF
    with pytest.raises(CtrlError, match="ID register reads 0xdeadbeef"):
        ctrl.check_id()


def test_counters_read_back_and_clear(ctrl):
    for name in ("FRAMES", "DGRAMS", "OVERRUNS"):
        ctrl.ser.regs[C.REG[name]] = 1234
    assert ctrl.counters()["FRAMES"] == 1234
    ctrl.clear_counters()
    assert set(ctrl.counters().values()) == {0}
    assert ctrl.read("CTRL") & 0x2 == 0  # clear_counters is self-clearing


def test_run_bit_survives_a_counter_clear(ctrl):
    ctrl.set_run(True)
    ctrl.clear_counters()
    assert ctrl.read("CTRL") & 0x1
    ctrl.set_run(False)
    assert ctrl.read("CTRL") & 0x1 == 0


def test_board_rejects_nonsense_and_read_only_writes(ctrl):
    with pytest.raises(CtrlError, match="rejected"):
        ctrl.write("FRAMES", 1)  # counters are read-only
    with pytest.raises(CtrlError, match="rejected"):
        ctrl.read(0x42)  # no such register
    with pytest.raises(CtrlError, match="unknown register name"):
        ctrl.read("NOPE")


def test_malformed_replies_are_reported_not_guessed(ctrl):
    for bad in ("01=zzzzzzzz", "01=1234", "yes", "01 00000001", ""):
        ctrl.ser.reply_override = bad
        with pytest.raises(CtrlError, match="malformed reply|no complete reply"):
            ctrl.read("SAMPLE_PERIOD")


def test_reply_for_the_wrong_register_is_caught(ctrl):
    ctrl.ser.reply_override = "02=00000001"
    with pytest.raises(CtrlError, match="reply is for register 0x02"):
        ctrl.read("SAMPLE_PERIOD")


def test_unexpected_write_ack_is_caught(ctrl):
    ctrl.ser.reply_override = "01=00000001"
    with pytest.raises(CtrlError, match="unexpected reply"):
        ctrl.write("SAMPLE_PERIOD", 1)


def test_diagnostic_list_registers_are_writable_counters_are_not(ctrl):
    for name in ("DIAG_PTR", "DIAG_CMD", "DIAG_LEN", "DIAG_CTRL", "SPI_DIV"):
        ctrl.write(name, 1)
    for name in ("SPI_CYCLES", "LINK_ERR0", "MISO_STAT", "CAPS", "DIAG_RES0", "DIAG_RAW1_2"):
        with pytest.raises(CtrlError, match="rejected"):
            ctrl.write(name, 1)


def test_read_many_is_one_read_per_register(ctrl):
    ctrl.ser.regs[C.REG["FRAMES"]] = 7
    got = ctrl.read_many(("FRAMES", 0x1F))
    assert got == {"FRAMES": 7, 0x1F: C.ID_EXPECTED}
    assert ctrl.ser.log == ["R 10", "R 1f"]


def test_resync_discards_whatever_the_board_says_to_a_bare_newline(ctrl):
    ctrl.resync(settle=0)
    assert ctrl.read("ID") == C.ID_EXPECTED
