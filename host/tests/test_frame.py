import struct

import pytest

from chain import contract as C
from chain import frame as F
from chain import pattern


def make_dgram(first_seq=0, n_ch=C.N_CH, **trailer):
    frames = [F.build_frame(first_seq + i, pattern.expected_sample(first_seq + i, n_ch=n_ch))
              for i in range(C.FRAMES_PER_DGRAM)]
    return F.build_datagram(frames, **trailer)


def test_frame_round_trip():
    want = pattern.expected_sample(7)
    raw = F.build_frame(7, want, t_cap=0x1111, t_tx=0x2222, flags=C.FLAG_REALISTIC)
    assert len(raw) == C.RAW16_FRAME
    f = F.parse_frame(raw)
    assert (f.seq, f.t_cap, f.t_tx, f.n_ch, f.payload_len) == (7, 0x1111, 0x2222, C.N_CH, 256)
    assert f.realistic and f.samples() == want


def test_datagram_round_trip_and_size():
    data = make_dgram(100, esp_rx_us=5, esp_tx_us=305, pipe_depth=7)
    assert len(data) == C.DGRAM_BYTES
    frames, trailer = F.parse_datagram(data)
    assert [f.seq for f in frames] == [100, 101, 102, 103, 104]
    assert trailer == {"esp_rx_us": 5, "esp_tx_us": 305, "pipe_depth": 7}


def test_bad_magic_is_named():
    raw = bytearray(F.build_frame(0, [0] * C.N_CH))
    struct.pack_into("<H", raw, 0, 0x1234)
    with pytest.raises(F.FrameError, match="bad magic 0x1234"):
        F.parse_header(bytes(raw))


def test_bad_payload_len_is_named():
    raw = bytearray(F.build_frame(0, [0] * C.N_CH))
    struct.pack_into("<H", raw, 18, 200)  # RAW16 must carry exactly n_ch*2
    with pytest.raises(F.FrameError, match="payload_len 200"):
        F.parse_header(bytes(raw))
    struct.pack_into("<H", raw, 18, 9999)  # longer than the fixed segment
    with pytest.raises(F.FrameError, match="must be 1"):
        F.parse_header(bytes(raw))


def test_short_buffers_are_rejected():
    with pytest.raises(F.FrameError, match="header needs 20 bytes"):
        F.parse_header(b"\x5a\xa5")
    with pytest.raises(F.FrameError, match="too short"):
        F.parse_datagram(b"\x00" * 8)
    with pytest.raises(F.FrameError, match="datagram is"):
        F.parse_datagram(make_dgram()[:-1])


def test_compressed_payload_may_be_shorter_than_the_segment():
    body = b"\xAA" * 64 + b"\x00" * (C.RAW16_PAYLOAD - 64)  # padded to fixed size
    hdr = struct.pack(C.HDR_FMT, C.MAGIC, C.TYPE_COMPRESSED, C.FLAG_CODEC, 3, 0, 0, C.N_CH, 64)
    f = F.parse_frame(hdr + body)
    assert f.payload_len == 64
    with pytest.raises(F.FrameError, match="not RAW16"):
        f.samples()
