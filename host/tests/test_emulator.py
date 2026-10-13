"""The emulator's playback memory as the golden reference for real-SPI runs."""
from chain import contract as C
from chain import emulator, metrics, pattern
from chain import frame as F


def test_channel_order_follows_the_contract():
    # Channels 0-31 chip 0 module A, 32-63 chip 0 B, 64-95 chip 1 A, 96-127 chip 1 B,
    # each file sample-major (sample * 32 + channel).
    pb = emulator.Playback()
    first = {}
    for name in ("chip0_A", "chip0_B", "chip1_A", "chip1_B"):
        lines = [l for l in (emulator.MEM_DIR / f"{name}.mem").read_text().split("\n")
                 if l.strip() and not l.startswith("//")]
        first[name] = (int(lines[0], 16), int(lines[33], 16))   # sample 0 ch 0, sample 1 ch 1
    s0, s1 = pb.expected(0), pb.expected(1)
    for k, name in enumerate(("chip0_A", "chip0_B", "chip1_A", "chip1_B")):
        assert s0[32 * k] == pattern.to_int16(first[name][0])
        assert s1[32 * k + 1] == pattern.to_int16(first[name][1])


def test_locate_finds_the_playback_position():
    pb = emulator.Playback()
    assert pb.locate(pb.expected(37)) == 37
    bad = list(pb.expected(37)); bad[90] ^= 1
    assert pb.locate(bad) is None


def emu_dgrams(pb, offset, n=4, corrupt=None):
    out = []
    for d in range(n):
        frames = []
        for i in range(C.FRAMES_PER_DGRAM):
            seq = d * C.FRAMES_PER_DGRAM + i
            s = list(pb.expected((seq + offset) % emulator.SAMPLES))
            if corrupt == (d, i):
                s[5] ^= 0x0300
            frames.append(F.build_frame(seq, s))
        out.append((d * 0.001,) + F.parse_datagram(F.build_datagram(frames)))
    return out


def test_metrics_score_emulator_traffic_at_its_playback_offset():
    # The pointer advances once per frame, so (position - seq) mod 256 is one
    # constant per run (149, 244, 159, 211 and 80 on the bench).
    pb = emulator.Playback()
    acc = metrics.compute(emu_dgrams(pb, 149), golden=pb)["accuracy"]
    assert acc["source"] == "emulator" and acc["playback_offset"] == 149
    assert acc["bit_errors"] == 0 and acc["values"] == 20 * C.N_CH


def test_metrics_count_a_corrupted_emulator_value():
    pb = emulator.Playback()
    acc = metrics.compute(emu_dgrams(pb, 211, corrupt=(2, 3)), golden=pb)["accuracy"]
    assert acc["playback_offset"] == 211
    assert acc["value_errors"] == 1 and acc["bit_errors"] == 2
