"""Turn captured datagrams into one results dict.

Throughput, loss, accuracy and latency are kept apart on purpose: they are four
different failure modes (radio ceiling, radio quality, sampling phase, pipeline
depth) and averaging them into one "score" hides which one is biting.
"""
import math

from . import contract as C
from . import pattern

U32 = 0xFFFFFFFF


def _stats(values: list[float]) -> dict | None:
    """min / median / p99 / max. Nearest-rank p99: with a few hundred samples an
    interpolated percentile invents a number no packet actually had."""
    if not values:
        return None
    v = sorted(values)
    n = len(v)
    return {
        "n": n,
        "min": v[0],
        "median": v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2.0,
        "p99": v[min(n - 1, math.ceil(0.99 * n) - 1)],
        "max": v[-1],
    }


def compute(dgrams, air_us=None, check_accuracy=True, malformed=0, golden=None) -> dict:
    """dgrams: list of (t_host_s, frames, trailer) from frame.parse_datagram.

    air_us: one-way estimates from the echo probe, measured separately because
    they are the only leg that does not come from a single device's clock.
    golden: an emulator.Playback to score real-SPI runs against the emulator's
    memory; None scores against the internal test pattern.
    """
    dgrams, stale = _drop_stale(dgrams)
    frames = [f for _t, fs, _tr in dgrams for f in fs]
    times = [t for t, _fs, _tr in dgrams]

    # Throughput, measured at the PC: first to last arrival, so the numbers do
    # not include the dead time before the FPGA was told to run. Dividing all
    # the bytes by that span overstates by one datagram, which is noise over a
    # run of seconds and keeps the number easy to explain.
    elapsed = (times[-1] - times[0]) if len(times) > 1 else 0.0
    payload_bytes = sum(f.payload_len for f in frames)
    wire_bytes = sum(C.HDR_BYTES + f.payload_len for f in frames) + len(dgrams) * C.TRAILER_BYTES
    if elapsed > 0:
        bytes_per_s = payload_bytes / elapsed
        frames_per_s = len(frames) / elapsed
    else:
        bytes_per_s = frames_per_s = 0.0
    throughput = {
        "payload_bytes": payload_bytes,
        "wire_bytes": wire_bytes,
        "elapsed_s": elapsed,
        "bytes_per_s": bytes_per_s,
        "mbit_per_s": bytes_per_s * 8 / 1e6,
        # One frame is one sample of every channel, so this is the rate the
        # electrophysiology side actually cares about.
        "frames_per_s": frames_per_s,
        "samples_per_s_per_ch": frames_per_s,
    }

    # Loss. seq offsets are taken relative to the first arrival so the 32-bit
    # wrap costs nothing; this assumes a run shorter than 2^32 frames and that
    # nothing arrives earlier than the first frame seen.
    seqs = [f.seq for f in frames]
    reordered = duplicates = 0
    seen = set()
    offsets = []
    if seqs:
        base = seqs[0]
        prev = 0
        for s in seqs:
            off = (s - base) & U32
            offsets.append(off)
            if off in seen:
                duplicates += 1
            elif off < prev:
                reordered += 1
            seen.add(off)
            prev = max(prev, off)
    span = max(offsets) + 1 if offsets else 0
    missing = span - len(seen)
    loss = {
        "datagrams": len(dgrams),
        "frames_received": len(frames),
        "frames_unique": len(seen),
        "frames_expected": span,
        "missing": missing,
        "missing_pct": 100.0 * missing / span if span else 0.0,
        "stale_frames": stale,
        "reordered": reordered,
        "duplicates": duplicates,
        "malformed_datagrams": malformed,
    }

    # Accuracy. Each frame is checked against the pattern for its own seq, so a
    # gap in the sequence costs only the frames that went missing: the ones that
    # did arrive stay aligned.
    acc = {"checked": False, "values": 0, "value_errors": 0,
           "bits": 0, "bit_errors": 0, "ber": None, "skipped_frames": 0,
           "pattern_offset": 0}
    if check_accuracy:
        acc["checked"] = True
        if golden is None:
            acc["source"] = "pattern"
            offset = acc["pattern_offset"] = _pattern_offset(frames)
            want_of = lambda f: pattern.expected_sample(f.seq + offset, f.realistic, f.n_ch)
        else:
            acc["source"] = "emulator"
            del acc["pattern_offset"]
            pos = acc["playback_offset"] = _playback_offset(frames, golden)
            want_of = lambda f: golden.expected(f.seq + pos)
        for f in frames:
            if f.type != C.TYPE_RAW16:
                acc["skipped_frames"] += 1  # a codec blob is not comparable here
                continue
            got = f.samples()
            want = want_of(f)
            acc["values"] += len(got)
            acc["bits"] += 16 * len(got)
            for g, w in zip(got, want):
                x = (g ^ w) & 0xFFFF
                if x:
                    acc["value_errors"] += 1
                    acc["bit_errors"] += bin(x).count("1")
        acc["ber"] = acc["bit_errors"] / acc["bits"] if acc["bits"] else None

    # Latency. fpga and esp are exact: each is a difference of two timestamps
    # taken from one device's own free-running counter. Only the air leg crosses
    # clock domains (PC clock vs ESP clock) and is therefore an estimate.
    fpga = [((f.t_tx - f.t_cap) & U32) * 1e6 / C.SYS_CLK_HZ for f in frames]
    esp = [float((tr["esp_tx_us"] - tr["esp_rx_us"]) & U32) for _t, _fs, tr in dgrams]
    latency = {
        "fpga_us": _stats(fpga),
        "esp_us": _stats(esp),
        "air_us": _stats(list(air_us) if air_us else []),
    }

    return {"throughput": throughput, "loss": loss, "accuracy": acc, "latency": latency}


def _drop_stale(dgrams):
    """Drop frames a run's first datagram carried over from the run before.

    The FPGA finishes a partly built datagram across a stop and start, so the
    first datagram of a capture can hold the previous run's last frames ahead
    of this run's first (seen on the bench: seq 6043..6046, then 0). Frames of
    the first datagram that do not run in sequence into its last frame are
    removed and counted separately; they are neither this run's data nor loss.
    """
    if not dgrams:
        return dgrams, 0
    t, first, tr = dgrams[0]
    cut = 0
    for k in range(1, len(first)):
        if first[k].seq != (first[k - 1].seq + 1) & U32:
            cut = k
    if not cut:
        return dgrams, 0
    return [(t, first[cut:], tr)] + list(dgrams[1:]), cut


def _pattern_offset(frames, probe: int = 32) -> int:
    """Pattern index minus seq for this run, 0 when the two start together.

    clear_counters zeroes seq but the FPGA's pattern generator carries on, so a
    run after the first carries pattern index seq + (samples generated before).
    Voted over the first CHECK frames, so one corrupt frame cannot pick it.
    REALISTIC values do not identify their index on their own and are scored
    at offset 0.
    """
    votes: dict[int, int] = {}
    n = 0
    for f in frames:
        if f.type != C.TYPE_RAW16 or f.realistic:
            continue
        i = pattern.check_index(f.samples())
        if i is not None:
            off = (i - f.seq) % pattern.PERIOD
            votes[off] = votes.get(off, 0) + 1
        n += 1
        if n >= probe:
            break
    return max(votes, key=votes.get) if votes else 0


def _playback_offset(frames, golden, probe: int = 32) -> int:
    """(emulator playback position - seq) mod its length, voted like _pattern_offset."""
    from .emulator import SAMPLES
    votes: dict[int, int] = {}
    for f in frames[:probe]:
        if f.type != C.TYPE_RAW16:
            continue
        p = golden.locate(f.samples())
        if p is not None:
            off = (p - f.seq) % SAMPLES
            votes[off] = votes.get(off, 0) + 1
    return max(votes, key=votes.get) if votes else 0
