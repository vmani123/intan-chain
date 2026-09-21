from chain import contract as C
from chain import frame as F
from chain import metrics, pattern


def dgram(first_seq, t_host, t_cap=0, t_tx=0, esp_rx=0, esp_tx=0, corrupt=None, realistic=False):
    """One parsed datagram of pattern data; corrupt=(frame_index, value_index, value)."""
    frames = []
    for i in range(C.FRAMES_PER_DGRAM):
        seq = first_seq + i
        samples = pattern.expected_sample(seq, realistic)
        if corrupt and corrupt[0] == i:
            samples = list(samples)
            samples[corrupt[1]] = corrupt[2]
        frames.append(F.build_frame(seq, samples, t_cap=t_cap, t_tx=t_tx,
                                    flags=C.FLAG_REALISTIC if realistic else 0))
    raw = F.build_datagram(frames, esp_rx_us=esp_rx, esp_tx_us=esp_tx)
    return (t_host,) + F.parse_datagram(raw)


def clean(n=4, **kw):
    return [dgram(d * C.FRAMES_PER_DGRAM, d * 0.001, **kw) for d in range(n)]


def test_throughput_and_channel_rate():
    # 4 datagrams, 1 ms apart: 3 ms between first and last arrival, 20 frames.
    r = metrics.compute(clean(4))["throughput"]
    assert r["payload_bytes"] == 20 * C.RAW16_PAYLOAD
    assert r["bytes_per_s"] == (20 * C.RAW16_PAYLOAD) / 0.003
    assert round(r["samples_per_s_per_ch"]) == round(20 / 0.003)


def test_loss_counts_a_deliberate_gap():
    dgrams = [dgram(0, 0.000), dgram(5, 0.001), dgram(15, 0.002)]  # seqs 10..14 never arrive
    loss = metrics.compute(dgrams)["loss"]
    assert loss["frames_expected"] == 20
    assert loss["frames_received"] == 15
    assert loss["missing"] == 5
    assert loss["missing_pct"] == 25.0
    assert loss["reordered"] == 0 and loss["duplicates"] == 0


def test_duplicates_and_reordering_are_separate_from_loss():
    dgrams = [dgram(0, 0.000), dgram(10, 0.001), dgram(5, 0.002), dgram(10, 0.003)]
    loss = metrics.compute(dgrams)["loss"]
    assert loss["missing"] == 0
    assert loss["duplicates"] == C.FRAMES_PER_DGRAM
    assert loss["reordered"] == C.FRAMES_PER_DGRAM  # the 5..9 datagram came late


def test_loss_survives_the_seq_wrap():
    start = 0xFFFFFFFF - 4  # last five seq values before the wrap
    dgrams = [dgram(start, 0.0), dgram((start + 5) & 0xFFFFFFFF, 0.001)]
    loss = metrics.compute(dgrams)["loss"]
    assert loss["frames_expected"] == 10 and loss["missing"] == 0


def test_accuracy_is_zero_on_clean_traffic_in_both_modes():
    for realistic in (False, True):
        acc = metrics.compute(clean(3, realistic=realistic))["accuracy"]
        assert acc["values"] == 15 * C.N_CH
        assert acc["value_errors"] == 0 and acc["bit_errors"] == 0 and acc["ber"] == 0.0


def test_accuracy_counts_a_corrupted_payload():
    good = pattern.expected_sample(7)[3]
    acc = metrics.compute([dgram(5, 0.0, corrupt=(2, 3, good ^ 0x0005))])["accuracy"]
    assert acc["value_errors"] == 1
    assert acc["bit_errors"] == 2  # two bits flipped in one value
    assert acc["ber"] == 2 / (5 * C.N_CH * 16)


def test_accuracy_stays_aligned_when_frames_are_lost():
    # Frames are checked against the pattern for their own seq, so a gap must
    # not shift the comparison and invent errors.
    dgrams = [dgram(0, 0.0), dgram(500, 0.001)]
    acc = metrics.compute(dgrams)["accuracy"]
    assert acc["bit_errors"] == 0 and metrics.compute(dgrams)["loss"]["missing"] == 495


def test_latency_legs_including_the_32_bit_wrap():
    # t_cap just below the wrap, t_tx just after it: 0x200 ticks, not a negative.
    d = dgram(0, 0.0, t_cap=0xFFFFFF00, t_tx=0x00000100,
              esp_rx=0xFFFFFF00, esp_tx=0x00000100)
    lat = metrics.compute([d], air_us=[1000.0, 2000.0, 3000.0])["latency"]
    assert lat["fpga_us"]["max"] == 0x200 * 1e6 / C.SYS_CLK_HZ
    assert lat["esp_us"]["max"] == float(0x200)
    assert lat["air_us"]["median"] == 2000.0 and lat["air_us"]["p99"] == 3000.0


def test_air_latency_is_absent_when_the_probe_did_not_run():
    assert metrics.compute(clean(2))["latency"]["air_us"] is None
