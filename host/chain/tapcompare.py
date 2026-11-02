"""Compare a tap recording with RHX's recording of the same session, every sample.

The chain heard the controller's SPI traffic (CONTRACTS.md section 7); RHX
saved what the controller read. Both count the same sweeps, so after one
constant offset between the chain's `seq` and RHX's timestamp, every RHX
sample has exactly one chain frame and the two must be equal word for word.
Nothing here is assumed that the data can check instead:

  alignment   The offset is unknown (the chain clears seq when it starts; RHX
              numbers samples from its own Run). Each sample is reduced to a
              fingerprint that survives a channel permutation and a constant
              XOR per channel: the sorted multiset of |x[t] - x[t-1]| (mod
              2^16) over the channels. A permutation reorders the multiset, a
              top-bit flip (offset binary vs two's complement) or an inverted
              MISO pair leaves |difference| alone. Fingerprints of RHX samples
              spread across its recording ("anchors") are looked up among the
              chain's; every anchor that matches votes for an offset, and one
              offset must win. Lost datagrams and RHX gaps only cost the
              anchors that touch them, because offsets are counted in seq and
              timestamps, never in rows.
  convention  With the offset known, each chain channel is tried against its
              expected RHX channel under each XOR in TRANSFORMS. Section 7
              says the tap carries words as heard, so only "identity" passes.
  mapping     Expected: chain channel k is RHX native channel k of the port
              (MISO1's chip first), as section 7 and RHX both number them. A
              channel that does not match its expected partner is searched for
              among all RHX channels and transforms; what is found is reported
              as a finding and then used, so a wiring swap does not bury every
              other error under 64 channels of noise.
  comparison  Every RHX sample in the overlap, every mapped channel.

numpy is required.
"""
import json
import mmap
import pathlib
import time
from dataclasses import dataclass, field

import numpy as np

from . import contract as C
from . import frame as F
from . import rhx, taprec

U32 = 0xFFFFFFFF
CHUNK = 16384               # RHX rows compared per step: bounded memory, numpy-sized work
MATCH = 0.5                 # fraction of sampled rows a channel pairing must equal exactly

# chain word == RHX code ^ mask
TRANSFORMS = {
    0x0000: ("identity", "chain words equal RHX's raw codes (offset binary, mid-scale 0x8000), "
                         "as section 7 says the tap carries them"),
    0x8000: ("top bit flipped", "chain word = RHX code XOR 0x8000: the same value in two's "
                                "complement, so something re-encoded it"),
    0xFFFF: ("inverted", "every bit inverted: the MISO pair of that chip is swapped (+/-) "
                         "between the cable and the tap"),
    0x7FFF: ("inverted, top bit kept", "every bit but the top one inverted: an inverted MISO "
                                       "pair and a two's complement re-encoding together"),
}
LANE_NAMES = ("chip0 A", "chip0 B", "chip1 A", "chip1 B")


class CompareError(RuntimeError):
    """The two recordings cannot be put side by side (exit status 2)."""


# ---------------------------------------------------------------------------
# The chain side
# ---------------------------------------------------------------------------

def frame_dtype(n_ch: int) -> np.dtype:
    """CONTRACTS.md section 1 as a numpy record; asserted against the generated header size."""
    dt = np.dtype([("magic", "<u2"), ("type", "u1"), ("flags", "u1"), ("seq", "<u4"),
                   ("t_cap", "<u4"), ("t_tx", "<u4"), ("n_ch", "<u2"), ("payload_len", "<u2"),
                   ("payload", "<u2", (n_ch,))])
    assert dt.itemsize == C.HDR_BYTES + 2 * n_ch
    return dt


def dgram_dtype(n_ch: int) -> np.dtype:
    dt = np.dtype([("frames", frame_dtype(n_ch), (C.FRAMES_PER_DGRAM,)),
                   ("trailer", "<u4", (3,))])
    assert dt.itemsize == C.FRAMES_PER_DGRAM * (C.HDR_BYTES + 2 * n_ch) + C.TRAILER_BYTES
    return dt


@dataclass
class ChainRec:
    path: pathlib.Path
    n_ch: int
    rel: np.ndarray             # int64 seq relative to the first frame, sorted, unique
    seq0: int                   # absolute seq of rel 0
    words: np.ndarray           # uint16 (frames, n_ch): the payload words as heard
    flags: np.ndarray           # uint8
    t_host: np.ndarray          # float64, host monotonic seconds of the frame's datagram
    datagrams: int
    malformed: int
    malformed_examples: list
    stale: int
    duplicates: int
    duplicates_differing: int
    trailing_bytes: int
    sidecar: dict | None = None
    _row: np.ndarray | None = field(default=None, repr=False)

    @property
    def n(self) -> int:
        return len(self.rel)

    def seq_of(self, rel) -> int:
        return (self.seq0 + int(rel)) & U32

    def row_of_rel(self) -> np.ndarray:
        """rel - rel.min() -> frame row, -1 where seq is missing."""
        if self._row is None:
            span = int(self.rel[-1] - self.rel[0]) + 1 if self.n else 0
            row = np.full(span, -1, np.int64)
            row[self.rel - self.rel[0]] = np.arange(self.n)
            self._row = row
        return self._row

    def gaps(self) -> tuple[int, int]:
        """(gap events, frames missing) between the first and last seq heard."""
        if self.n < 2:
            return 0, 0
        d = np.diff(self.rel)
        return int(np.count_nonzero(d > 1)), int((d[d > 1] - 1).sum())


def load_chain(path) -> ChainRec:
    """Every frame of a tap recording, by seq, without holding the file twice.

    The file is memory-mapped and, in the normal case of equal-sized records,
    viewed in place as an array of datagrams; only the payload words of the
    frames kept are copied out. A 5-minute recording at 2 kS/s is ~170 MB of
    file and ~150 MB of words.
    """
    path = pathlib.Path(path)
    with open(path, "rb") as fh:
        if path.stat().st_size == 0:
            raise CompareError(f"{path}: empty file")
        buf = mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ)
    try:
        return _load_chain(path, buf)
    finally:
        try:
            buf.close()
        except BufferError:
            pass                    # a view still alive; the mapping goes with it


def _load_chain(path, buf) -> ChainRec:
    _head, times, offsets, lengths, trailing = taprec.scan(buf)
    if not offsets:
        raise CompareError(f"{path}: no datagrams recorded")
    n_ch = None
    for o, n in zip(offsets[:1000], lengths[:1000]):
        try:
            frames, _tr = F.parse_datagram(buf[o:o + n])
        except F.FrameError:
            continue
        n_ch = frames[0].n_ch
        break
    if n_ch is None:
        raise CompareError(f"{path}: none of the first {min(1000, len(offsets))} datagrams "
                           f"parses as a chain datagram (CONTRACTS.md sections 1-2)")
    ddt = dgram_dtype(n_ch)
    want = ddt.itemsize
    sized = np.array([i for i, n in enumerate(lengths) if n == want], np.int64)
    off = np.asarray(offsets, np.int64)
    stride = taprec.RECORD.size + want
    if len(sized) == len(offsets) and np.all(np.diff(off) == stride):
        arr = np.ndarray((len(off),), ddt, buffer=buf, offset=int(off[0]), strides=(stride,))
    else:                                   # odd-sized records in between: gather the rest
        joined = b"".join(buf[offsets[i]:offsets[i] + want] for i in sized)
        arr = np.frombuffer(joined, ddt)
    fr = arr["frames"]                      # (datagrams, 5) records, still in the file
    ok = ((fr["magic"] == C.MAGIC).all(1) & (fr["n_ch"] == n_ch).all(1)
          & (fr["payload_len"] == 2 * n_ch).all(1) & (fr["type"] == C.TYPE_RAW16).all(1))
    bad = sorted(set(range(len(lengths))) - set(sized.tolist()) | set(sized[~ok].tolist()))
    examples = []
    for i in bad[:5]:
        try:
            F.parse_datagram(buf[offsets[i]:offsets[i] + lengths[i]])
            examples.append(f"datagram {i}: frames are not RAW16 with {n_ch} channels")
        except F.FrameError as e:
            examples.append(f"datagram {i}: {e}")
    good = np.nonzero(ok)[0]
    if len(good) == 0:
        raise CompareError(f"{path}: no valid datagrams ({len(bad)} malformed)")
    per = C.FRAMES_PER_DGRAM
    di = np.repeat(good, per)               # (datagram, frame) of every frame kept
    fi = np.tile(np.arange(per), len(good))
    seq = fr["seq"][good].reshape(-1).astype(np.int64)
    flags = fr["flags"][good].reshape(-1)
    t_host = np.repeat(np.asarray(times, np.float64)[sized[good]], per)

    # Frames the run's first datagram carried over from the run before: the
    # same rule as metrics._drop_stale (the FPGA finishes a partly built
    # datagram across stop and start).
    stale = 0
    for k in range(1, per):
        if seq[k] != (seq[k - 1] + 1) & U32:
            stale = k
    di, fi, seq, flags, t_host = di[stale:], fi[stale:], seq[stale:], flags[stale:], \
        t_host[stale:]

    seq0 = int(seq[0])
    rel = (seq - seq0) & U32
    rel[rel >= 1 << 31] -= 1 << 32          # an early straggler, not a 24-day jump
    order = np.argsort(rel, kind="stable")
    rel_s = rel[order]
    keep = np.ones(len(rel_s), bool)
    keep[1:] = rel_s[1:] != rel_s[:-1]
    dup = order[~keep]
    rows = order[keep]
    payload = fr["payload"]
    differing = 0
    if len(dup):
        first = rows[np.searchsorted(rel_s[keep], rel[dup])]
        differing = int(np.count_nonzero(
            (payload[di[dup], fi[dup]] != payload[di[first], fi[first]]).any(1)))
    words = np.ascontiguousarray(payload[di[rows], fi[rows]])
    del arr, fr, payload
    side = taprec.sidecar(path)
    sc = json.loads(side.read_text()) if side.exists() else None
    return ChainRec(path, n_ch, rel_s[keep], seq0, words, flags[rows].copy(), t_host[rows],
                    len(lengths), len(bad), examples, stale, len(dup), differing, trailing, sc)


# ---------------------------------------------------------------------------
# Fingerprints
# ---------------------------------------------------------------------------

_COEF = np.random.default_rng(0x7A9C0DE).integers(1, 1 << 62, size=4096, dtype=np.uint64) | 1


def fingerprint(prev: np.ndarray, cur: np.ndarray) -> np.ndarray:
    """uint64 per row: the sorted |cur - prev| (mod 2^16, signed) over the columns, hashed.

    Invariant under a permutation of the columns and under XOR of any column
    with 0x8000 or 0xffff, so it can find the offset before the channel order
    and the number format are known.
    """
    d = cur.astype(np.int32) - prev.astype(np.int32)
    d = (d + 32768) % 65536 - 32768
    a = np.abs(d).astype(np.uint64)
    a.sort(axis=1)
    return (a * _COEF[:a.shape[1]]).sum(axis=1, dtype=np.uint64)


def chain_fingerprints(ch: ChainRec, cols, step: int = 65536):
    """(hash, rel) for every frame whose predecessor was heard, both whole TAP frames."""
    good = ((ch.flags & C.FLAG_TAP) != 0) & ((ch.flags & C.FLAG_TAP_PARTIAL) == 0)
    pair = np.nonzero((np.diff(ch.rel) == 1) & good[1:] & good[:-1])[0] + 1
    cols = np.asarray(cols)
    hs = []
    for s in range(0, len(pair), step):
        r = pair[s:s + step]
        hs.append(fingerprint(ch.words[r - 1][:, cols], ch.words[r][:, cols]))
    h = np.concatenate(hs) if hs else np.zeros(0, np.uint64)
    return h, ch.rel[pair]


def rhx_anchor_rows(rec: rhx.Recording, k: int) -> np.ndarray:
    """k rows spread over the recording, each one timestamp after its predecessor."""
    ok = np.nonzero(np.diff(rec.t) == 1)[0] + 1
    if len(ok) <= k:
        return ok
    return ok[np.linspace(0, len(ok) - 1, k).round().astype(np.int64)]


# ---------------------------------------------------------------------------
# The RHX side: which columns are the port, and what chain channel each should be
# ---------------------------------------------------------------------------

def port_columns(rec: rhx.Recording, port: str | None) -> tuple[str, list[int]]:
    """(port prefix, column indices) of the port to compare: --port, else the biggest."""
    by = {}
    for j, c in enumerate(rec.channels):
        by.setdefault(c.port_prefix, []).append(j)
    if not by:
        raise CompareError("the RHX recording has no amplifier channels")
    if port is None:
        port = max(by, key=lambda p: len(by[p]))
    if port not in by:
        raise CompareError(f"RHX has no amplifier channels on port {port!r} "
                           f"(it has {', '.join(sorted(by))})")
    return port, by[port]


# ---------------------------------------------------------------------------
# The comparison
# ---------------------------------------------------------------------------

@dataclass
class Options:
    port: str | None = None
    anchors: int = 64
    sample_rows: int = 4096
    show: int = 20


def _finding(out, status, category, message):
    out.append({"status": status, "category": category, "message": message})


def _ranges(xs) -> str:
    xs = sorted(int(x) for x in xs)
    if not xs:
        return "-"
    runs, a, b = [], xs[0], xs[0]
    for x in xs[1:]:
        if x == b + 1:
            b = x
        else:
            runs.append((a, b))
            a = b = x
    runs.append((a, b))
    return ", ".join(f"{p}-{q}" if p != q else f"{p}" for p, q in runs)


def align(ch: ChainRec, rec: rhx.Recording, cols: list[int], expected: dict, opts: Options,
          findings: list) -> dict:
    """Find D with RHX timestamp = chain rel + D, from fingerprints at anchors."""
    n_ch = ch.n_ch
    subsets = []
    exp_set = sorted(k for k in expected.values() if k is not None)
    if len(exp_set) == len(cols):
        subsets.append(("expected channels", exp_set))
    if len(cols) == n_ch:
        subsets.append(("all channels", list(range(n_ch))))
    if 2 * len(cols) == n_ch:
        subsets += [("chip 0 (channels 0-63)", list(range(n_ch // 2))),
                    ("chip 1 (channels 64-127)", list(range(n_ch // 2, n_ch)))]
    seen, uniq = set(), []
    for name, s in subsets:
        if tuple(s) not in seen:
            seen.add(tuple(s))
            uniq.append((name, s))
    if not uniq:
        raise CompareError(f"RHX port has {len(cols)} amplifier channels and the chain {n_ch}; "
                           f"alignment needs the same channels on both sides (enable all "
                           f"{n_ch} in RHX)")
    anchors = rhx_anchor_rows(rec, opts.anchors)
    if len(anchors) == 0:
        raise CompareError("the RHX recording has no two consecutive samples")
    rprev = np.concatenate([rec.codes(int(a) - 1, int(a) + 1)[None] for a in anchors])
    rh = fingerprint(rprev[:, 0, cols], rprev[:, 1, cols])
    tried = []
    for name, subset in uniq:
        ch_h, ch_rel = chain_fingerprints(ch, subset)
        order = np.argsort(ch_h, kind="stable")
        sh, srel = ch_h[order], ch_rel[order]
        u, first, counts = np.unique(sh, return_index=True, return_counts=True)
        pos = np.searchsorted(u, rh)
        pos_c = np.minimum(pos, max(len(u) - 1, 0))
        hit = (len(u) > 0) & (u[pos_c] == rh) if len(u) else np.zeros(len(rh), bool)
        unique_hit = hit & (counts[pos_c] == 1) if len(u) else hit
        ambiguous = int(np.count_nonzero(hit & ~unique_hit))
        votes = {}
        anchor_rows = {}
        for i in np.nonzero(unique_hit)[0]:
            d = int(rec.t[anchors[i]]) - int(srel[first[pos_c[i]]])
            votes[d] = votes.get(d, 0) + 1
            anchor_rows.setdefault(d, []).append(int(anchors[i]))
        tried.append({"subset": name, "anchors": int(len(anchors)),
                      "matched": int(sum(votes.values())), "ambiguous": ambiguous,
                      "chain_fingerprints": int(len(ch_h)),
                      "votes": {str(k): v for k, v in sorted(votes.items(),
                                                             key=lambda kv: -kv[1])}})
        if not votes:
            continue
        best = max(votes, key=votes.get)
        agree = votes[best]
        if agree < 2 and len(anchors) >= 2:
            continue
        others = {d: v for d, v in votes.items() if d != best}
        if others:
            _finding(findings, "FAIL" if any(v >= 2 for v in others.values()) else "WARN",
                     "OFFSET_INCONSISTENT",
                     f"{agree} anchors put RHX timestamp = chain seq + {best - ch.seq0}, but "
                     + "; ".join(f"{v} put it at + {d - ch.seq0} (RHX rows {anchor_rows[d][:3]})"
                                 for d, v in sorted(others.items(), key=lambda kv: -kv[1]))
                     + ": section 7 gives every sweep one frame except a sweep the tap could "
                       "not frame at all (counted in OVERRUNS, see the recording's .json), so "
                       "the offset must not move otherwise. Comparing under the majority "
                       "offset; the region where it moved will show as mismatches")
        return {"D": best, "seq_offset": best - ch.seq0, "agree": agree,
                "matched": int(sum(votes.values())), "anchors": int(len(anchors)),
                "subset": name, "tried": tried}
    raise CompareError(
        "no consistent offset: " + "; ".join(
            f"{t['subset']}: {t['matched']} of {t['anchors']} anchors matched"
            + (f", {t['ambiguous']} ambiguous" if t["ambiguous"] else "") for t in tried)
        + ". Either the chain did not hear this session (another recording, or the tap was "
          "not in MODE bit4), or most words are corrupted (run `tapcheck status` and "
          "`tapcheck delay-sweep`), or the signal is too quiet to tell samples apart "
          "(constant channels: connect the electrodes or a test signal)")


def sample_overlap_rows(ch: ChainRec, rec: rhx.Recording, D: int, n: int):
    """Up to n RHX rows in the overlap whose chain frame is a whole TAP frame: (rows, frame rows)."""
    rel = rec.t - D
    inside = np.nonzero((rel >= ch.rel[0]) & (rel <= ch.rel[-1]))[0]
    if len(inside) == 0:
        return inside, inside
    fr = ch.row_of_rel()[rel[inside] - ch.rel[0]]
    good = fr >= 0
    good[good] = ((ch.flags[fr[good]] & C.FLAG_TAP) != 0) & \
        ((ch.flags[fr[good]] & C.FLAG_TAP_PARTIAL) == 0)
    rows, frs = inside[good], fr[good]
    if len(rows) > n:
        pick = np.linspace(0, len(rows) - 1, n).round().astype(np.int64)
        rows, frs = rows[pick], frs[pick]
    return rows, frs


def _codes_at(rec: rhx.Recording, rows: np.ndarray) -> np.ndarray:
    """RHX codes at arbitrary (sorted) rows, read in contiguous runs."""
    out = np.empty((len(rows), len(rec.channels)), np.uint16)
    i = 0
    while i < len(rows):
        j = i + 1
        while j < len(rows) and rows[j] - rows[i] < CHUNK and j - i < CHUNK:
            j += 1
        block = rec.codes(int(rows[i]), int(rows[j - 1]) + 1)
        out[i:j] = block[rows[i:j] - rows[i]]
        i = j
    return out


def map_channels(A: np.ndarray, B: np.ndarray, expected_col: dict) -> dict:
    """Per chain channel: (RHX column, XOR mask, how), from sampled aligned rows.

    A: chain words (S, n_ch); B: RHX codes (S, port columns). expected_col:
    chain k -> column index into B, or None where RHX has no such channel.
    """
    S, n_ch = A.shape
    ncols = B.shape[1]
    masks = list(TRANSFORMS)
    result = {}
    failing = []
    for k in range(n_ch):
        j = expected_col.get(k)
        if j is None:
            continue
        fr = [float(np.mean(A[:, k] == (B[:, j] ^ np.uint16(m)))) for m in masks]
        best = int(np.argmax(fr))
        if fr[best] >= MATCH:
            result[k] = {"col": j, "mask": masks[best], "match": fr[best], "how": "expected"}
        else:
            failing.append(k)
    discovered = {}
    if failing:
        counts = {m: np.zeros((len(failing), ncols), np.int64) for m in masks}
        fa = A[:, failing]
        for s in range(0, S, 512):
            a = fa[s:s + 512][:, :, None]
            for m in masks:
                counts[m] += (a == (B[s:s + 512] ^ np.uint16(m))[:, None, :]).sum(0)
        for i, k in enumerate(failing):
            best = None
            for m in masks:
                row = counts[m][i]
                j = int(np.argmax(row))
                top = int(row[j])
                tie = int(np.count_nonzero(row == top))
                if best is None or top > best[0]:
                    best = (top, j, m, tie)
            top, j, m, tie = best
            if top >= MATCH * S:
                discovered[k] = {"col": j, "mask": m, "match": top / S, "how": "discovered",
                                 "ambiguous": tie > 1}
        result.update(discovered)
    unmatched = [k for k in failing if k not in discovered]
    for k in unmatched:
        # Still compared against its expected partner, so its errors are counted.
        result[k] = {"col": expected_col[k], "mask": 0, "match": None, "how": "unmatched"}
    return {"map": result, "failing": failing, "discovered": discovered, "unmatched": unmatched}


def describe_mapping(mapping: dict, names: list[str]) -> list[str]:
    """Runs like 'chain 0-31 = RHX A-032..A-063 (inverted)' for every channel not as expected."""
    lines = []
    ks = sorted(k for k, v in mapping.items() if v["how"] == "discovered")
    i = 0
    while i < len(ks):
        j = i
        while (j + 1 < len(ks) and ks[j + 1] == ks[j] + 1
               and mapping[ks[j + 1]]["col"] == mapping[ks[j]]["col"] + 1
               and mapping[ks[j + 1]]["mask"] == mapping[ks[i]]["mask"]):
            j += 1
        a, b = ks[i], ks[j]
        ca, cb = mapping[a]["col"], mapping[b]["col"]
        m = mapping[a]["mask"]
        tag = "" if m == 0 else f" ({TRANSFORMS[m][0]})"
        rng = f"{names[ca]}" if a == b else f"{names[ca]}..{names[cb]}"
        lines.append(f"chain {a if a == b else f'{a}-{b}'} = RHX {rng}{tag}")
        i = j + 1
    return lines


def compare(chain_path, rhx_paths, opts: Options = None) -> dict:
    """Everything `tapcheck compare` reports, as one dict. Raises CompareError if impossible."""
    opts = opts or Options()
    findings: list = []
    ch = load_chain(chain_path)
    rec = rhx.load(rhx_paths)
    port, cols = port_columns(rec, opts.port)
    chans = [rec.channels[j] for j in cols]
    names = [c.native_name for c in chans]
    n_ch = ch.n_ch

    # ---- RHX settings that make its file differ from the wire ------------
    h = rec.header
    for w in rec.warnings:
        _finding(findings, "WARN", "RHX_FILE", w)
    if h.notch_hz and h.version[0] >= 3:
        _finding(findings, "WARN", "RHX_NOTCH",
                 f"RHX's {h.notch_hz} Hz notch filter was on: since RHX 3.0 the saved amplifier "
                 f"data is notch-filtered (Intan's loader says so and does not re-apply it), so "
                 f"it is not the chip's raw codes. Set Notch Filter to Disabled and record again")
    if h.reference_channel not in (None, "", "n/a", "Hardware", "hardware"):
        _finding(findings, "WARN", "RHX_REFERENCE",
                 f"RHX reference is {h.reference_channel!r}: a software reference subtracts "
                 f"another channel before saving; use the hardware reference")
    other_ports = sorted({c.port_prefix for c in rec.channels} - {port})
    if other_ports:
        _finding(findings, "INFO", "RHX_OTHER_PORTS",
                 f"RHX also recorded port(s) {', '.join(other_ports)}; comparing port {port} "
                 f"(--port to choose)")

    # ---- chain-side bookkeeping ------------------------------------------
    if ch.malformed:
        _finding(findings, "WARN", "MALFORMED",
                 f"{ch.malformed} of {ch.datagrams} datagrams did not parse and were dropped; "
                 f"their frames count as missing. " + "; ".join(ch.malformed_examples[:3]))
    if ch.stale:
        _finding(findings, "INFO", "STALE",
                 f"the first datagram carried {ch.stale} frame(s) from the previous run; dropped")
    if ch.duplicates:
        _finding(findings, "FAIL" if ch.duplicates_differing else "INFO", "DUPLICATES",
                 f"{ch.duplicates} frames arrived twice"
                 + (f", {ch.duplicates_differing} with different payloads"
                    if ch.duplicates_differing else " (identical copies; compared once)"))
    if ch.trailing_bytes:
        _finding(findings, "INFO", "TRUNCATED",
                 f"the recording ends in a partial record ({ch.trailing_bytes} bytes ignored)")
    nontap_all = int(np.count_nonzero((ch.flags & C.FLAG_TAP) == 0))

    # ---- expected mapping: chain k <-> native channel k of the port ------
    expected_col = {}
    by_order = {c.native_order: i for i, c in enumerate(chans)}
    for k in range(n_ch):
        expected_col[k] = by_order.get(k)

    al = align(ch, rec, cols, {k: (k if expected_col[k] is not None else None)
                               for k in range(n_ch)}, opts, findings)
    D = al["D"]

    rows, frs = sample_overlap_rows(ch, rec, D, opts.sample_rows)
    if len(rows) == 0:
        raise CompareError("the recordings do not overlap at the offset found")
    A = ch.words[frs]
    B = _codes_at(rec, rows)[:, cols]
    mp = map_channels(A, B, expected_col)
    mapping = mp["map"]

    conv = {}
    for k, v in mapping.items():
        if v["how"] != "unmatched":
            conv.setdefault(v["mask"], []).append(k)
    if set(conv) - {0}:
        for m, ks in sorted(conv.items()):
            if m:
                _finding(findings, "FAIL", "CONVENTION",
                         f"chain channels {_ranges(ks)}: {TRANSFORMS[m][1]}")
    if mp["discovered"]:
        desc = describe_mapping(mapping, names)
        _finding(findings, "FAIL", "MAPPING",
                 f"{len(mp['discovered'])} chain channels are not RHX's channel of the same "
                 f"number (expected: chain k = {port}-k, MISO1's chip first). Found in the "
                 f"data: " + "; ".join(desc) + ". Compared under the discovered mapping")
        amb = [k for k, v in mp["discovered"].items() if v.get("ambiguous")]
        if amb:
            _finding(findings, "WARN", "MAPPING_AMBIGUOUS",
                     f"chain channels {_ranges(amb)} match more than one RHX channel equally "
                     f"well (constant data?); the first was taken")
    if mp["unmatched"]:
        _finding(findings, "FAIL", "UNMATCHED",
                 f"chain channels {_ranges(mp['unmatched'])} match no RHX channel under any "
                 f"convention; compared against their expected partners, so every error shows")
    unrecorded = [k for k in range(n_ch) if k not in mapping]
    if unrecorded:
        _finding(findings, "WARN", "UNRECORDED",
                 f"chain channels {_ranges(unrecorded)} have no channel in RHX's port {port} "
                 f"(not enabled there?) and were not checked")
    used = {v["col"] for v in mapping.values()}
    dup_cols = [names[j] for j in used if sum(1 for v in mapping.values() if v["col"] == j) > 1]
    if dup_cols:
        _finding(findings, "FAIL", "MAPPING_DUPLICATE",
                 f"RHX channel(s) {', '.join(dup_cols[:8])} match more than one chain channel")

    # ---- every sample ----------------------------------------------------
    kk = np.array(sorted(mapping), np.int64)
    jj = np.array([cols[mapping[k]["col"]] for k in kk], np.int64)
    mm = np.array([mapping[k]["mask"] for k in kk], np.uint16)
    rel_all = rec.t - D
    in_span = (rel_all >= ch.rel[0]) & (rel_all <= ch.rel[-1])
    rows_ov = np.nonzero(in_span)[0]
    row_of = ch.row_of_rel()
    stats = dict(overlap_samples=int(len(rows_ov)), frames_compared=0, frames_missing=0,
                 frames_partial=0, frames_not_tap=0, compared=0, equal=0, mismatched=0,
                 partial_compared=0, partial_equal=0, partial_unheard=0, partial_wrong=0)
    per_ch = np.zeros(n_ch, np.int64)
    bit_hist = np.zeros(16, np.int64)
    nbits_hist = np.zeros(17, np.int64)
    first = []
    missing_seqs = []
    for s in range(0, len(rows_ov), CHUNK):
        r = rows_ov[s:s + CHUNK]
        block = rec.codes(int(r[0]), int(r[-1]) + 1)[r - r[0]]
        E = block[:, jj] ^ mm
        fr = row_of[rel_all[r] - ch.rel[0]]
        present = fr >= 0
        stats["frames_missing"] += int(np.count_nonzero(~present))
        if len(missing_seqs) < opts.show and not present.all():
            missing_seqs += [ch.seq_of(x) for x in rel_all[r][~present][:opts.show]]
        fp = fr[present]
        G = ch.words[fp][:, kk]
        Ep = E[present]
        flags = ch.flags[fp]
        part = (flags & C.FLAG_TAP_PARTIAL) != 0
        stats["frames_not_tap"] += int(np.count_nonzero((flags & C.FLAG_TAP) == 0))
        neq = G != Ep
        full = ~part
        nf = neq[full]
        stats["frames_compared"] += int(np.count_nonzero(full))
        stats["frames_partial"] += int(np.count_nonzero(part))
        stats["compared"] += int(nf.size)
        bad = int(nf.sum())
        stats["mismatched"] += bad
        stats["equal"] += int(nf.size) - bad
        if bad:
            np.add.at(per_ch, kk, nf.sum(0))
            x = (G[full] ^ Ep[full])[nf]
            for b in range(16):
                bit_hist[b] += int(np.count_nonzero((x >> b) & 1))
            pc = np.zeros(len(x), np.int64)
            for b in range(16):
                pc += (x >> b) & 1
            nbits_hist += np.bincount(pc, minlength=17)[:17]
        if part.any():
            npart = neq[part]
            stats["partial_compared"] += int(npart.size)
            stats["partial_equal"] += int(npart.size - npart.sum())
            unheard = npart & (G[part] == 0)
            stats["partial_unheard"] += int(unheard.sum())
            stats["partial_wrong"] += int((npart & ~unheard).sum())
        if len(first) < opts.show and nf.any():
            ri, ci = np.nonzero(neq & full[:, None])
            rr = r[present]
            for a, b in zip(ri[:opts.show - len(first)], ci[:opts.show - len(first)]):
                k = int(kk[b])
                first.append({"seq": ch.seq_of(rel_all[rr[a]]), "rhx_timestamp": int(rec.t[rr[a]]),
                              "chain_ch": k, "rhx_ch": rec.channels[int(jj[b])].native_name,
                              "expected": f"0x{int(Ep[a, b]):04x}", "got": f"0x{int(G[a, b]):04x}",
                              "xor": f"0x{int(Ep[a, b] ^ G[a, b]):04x}",
                              "partial_frame": bool(part[a])})

    # ---- what lies outside, and gaps -------------------------------------
    t0, t1 = int(rec.t[0]), int(rec.t[-1])
    chain_t = ch.rel + D
    before = int(np.count_nonzero(chain_t < t0))
    after = int(np.count_nonzero(chain_t > t1))
    within = chain_t[(chain_t >= t0) & (chain_t <= t1)]
    no_rhx = int(len(within) - np.count_nonzero(np.isin(within, rec.t)))
    gaps = rec.gaps()
    gaps_ov = [(r, n) for r, n in gaps if in_span[r] or (r > 0 and in_span[r - 1])]
    rhx_before = int(np.count_nonzero(rel_all < ch.rel[0]))
    rhx_after = int(np.count_nonzero(rel_all > ch.rel[-1]))
    cg_events, cg_missing = ch.gaps()

    if stats["frames_missing"]:
        _finding(findings, "FAIL", "MISSING_FRAMES",
                 f"{stats['frames_missing']} RHX samples in the overlap have no chain frame "
                 f"(lost datagrams: seq gaps), first seqs {missing_seqs[:5]}")
    if stats["frames_partial"]:
        _finding(findings, "FAIL", "PARTIAL_FRAMES",
                 f"{stats['frames_partial']} frames in the overlap are TAP_PARTIAL: "
                 f"{stats['partial_unheard']} channel-samples read 0 (not heard), "
                 f"{stats['partial_wrong']} differ otherwise")
    if stats["frames_not_tap"] or nontap_all:
        _finding(findings, "FAIL", "NOT_TAP",
                 f"{nontap_all} frames lack the TAP flag ({stats['frames_not_tap']} in the "
                 f"overlap): the board was not in tap mode (MODE bit4) for all of the recording")
    if stats["mismatched"]:
        worst = np.argsort(-per_ch)[:8]
        _finding(findings, "FAIL", "MISMATCH",
                 f"{stats['mismatched']} of {stats['compared']} channel-samples differ; most on "
                 + ", ".join(f"ch {int(k)} ({int(per_ch[k])})" for k in worst if per_ch[k]))
    if gaps_ov:
        _finding(findings, "FAIL", "RHX_GAPS",
                 f"RHX itself skipped {sum(n for _r, n in gaps_ov)} samples inside the overlap "
                 f"({len(gaps_ov)} gaps; the controller's or RHX's loss, not the chain's): "
                 f"{no_rhx} chain frames there could not be checked")
    if rhx_before or rhx_after:
        _finding(findings, "WARN", "RHX_OUTSIDE",
                 f"{rhx_before} RHX samples come before the chain recording and {rhx_after} "
                 f"after it, so they were not checked: start `tapcheck record` before RHX "
                 f"records and stop it after")
    if before or after:
        _finding(findings, "INFO", "CHAIN_OUTSIDE",
                 f"{before} chain frames before the RHX recording and {after} after it "
                 f"(expected: the tap records while RHX runs, RHX saves a window)")
    if stats["compared"] == 0:
        _finding(findings, "FAIL", "NOTHING_COMPARED", "no channel-sample was compared")

    rank = {"INFO": 0, "WARN": 1, "FAIL": 2}
    worst = max((rank[f["status"]] for f in findings), default=0)
    result = {0: "PASS", 1: "PASS", 2: "FAIL"}[worst]
    conv_summary = {TRANSFORMS[m][0]: _ranges(ks) for m, ks in sorted(conv.items())}
    return {
        "tool": "tapcheck",
        "command": "compare",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "result": result,
        "chain": {"path": str(ch.path), "datagrams": ch.datagrams, "frames": ch.n,
                  "n_ch": n_ch, "first_seq": ch.seq_of(ch.rel[0]),
                  "last_seq": ch.seq_of(ch.rel[-1]), "seq_gaps": cg_events,
                  "frames_missing": cg_missing, "malformed_datagrams": ch.malformed,
                  "stale_frames": ch.stale, "duplicates": ch.duplicates,
                  "not_tap_frames": nontap_all,
                  "partial_frames": int(np.count_nonzero(ch.flags & C.FLAG_TAP_PARTIAL)),
                  "sidecar": ch.sidecar},
        "rhx": {**rec.describe(), "port": port, "port_channels": len(cols),
                "gaps": len(gaps), "samples_missing": int(sum(n for _r, n in gaps))},
        "alignment": {"rhx_timestamp_minus_chain_seq": al["seq_offset"],
                      "anchors": al["anchors"], "matched": al["matched"], "agree": al["agree"],
                      "fingerprint_channels": al["subset"], "tried": al["tried"]},
        "convention": {"by_transform": conv_summary,
                       "identity_everywhere": set(conv) <= {0}},
        "mapping": {"expected": f"chain k = {port}-k (native order, MISO1's chip first)",
                    "as_expected": not mp["discovered"] and not mp["unmatched"],
                    "discovered": describe_mapping(mapping, names),
                    "unmatched": mp["unmatched"], "unrecorded": unrecorded,
                    "channels": {str(k): {"rhx": names[v["col"]], "how": v["how"],
                                          "transform": TRANSFORMS[v["mask"]][0]}
                                 for k, v in sorted(mapping.items())}},
        "overlap": {"rhx_first": int(rec.t[rows_ov[0]]) if len(rows_ov) else None,
                    "rhx_last": int(rec.t[rows_ov[-1]]) if len(rows_ov) else None,
                    **stats,
                    "chain_frames_without_rhx_sample": no_rhx,
                    "rhx_gaps": len(gaps_ov),
                    "rhx_gap_samples": int(sum(n for _r, n in gaps_ov))},
        "outside": {"chain_frames_before_rhx": before, "chain_frames_after_rhx": after,
                    "rhx_samples_before_chain": rhx_before, "rhx_samples_after_chain": rhx_after},
        "mismatches_per_channel": {str(int(k)): int(per_ch[k]) for k in np.nonzero(per_ch)[0]},
        "xor_bit_histogram": [int(x) for x in bit_hist],
        "bits_flipped_histogram": {str(i): int(n) for i, n in enumerate(nbits_hist) if n},
        "first_mismatches": first,
        "first_missing_seqs": missing_seqs[:opts.show],
        "findings": findings,
    }


def render(rep: dict, show: int = 20) -> str:
    """The report as a person reads it: headline numbers first, then what failed and why."""
    c, r, al, ov = rep["chain"], rep["rhx"], rep["alignment"], rep["overlap"]
    out = [f"chain  {c['path']}",
           f"       {c['datagrams']} datagrams, {c['frames']} frames, seq {c['first_seq']}.."
           f"{c['last_seq']}, {c['seq_gaps']} seq gaps ({c['frames_missing']} frames), "
           f"{c['partial_frames']} partial, {c['malformed_datagrams']} malformed",
           f"RHX    {', '.join(r['paths'])}",
           f"       {r['format']} v{r['version']}, {r['sample_rate']:g} S/s, port {r['port']} "
           f"({r['port_channels']} amplifier channels), {r['samples']} samples "
           f"(t {r['first_timestamp']}..{r['last_timestamp']}), {r['gaps']} timestamp gaps",
           f"align  RHX timestamp = chain seq + {al['rhx_timestamp_minus_chain_seq']}  "
           f"({al['agree']} of {al['matched']} matched anchors agree, {al['anchors']} tried, "
           f"fingerprints over {al['fingerprint_channels']})",
           "format " + "; ".join(f"{k}: chain ch {v}" for k, v in
                                 rep["convention"]["by_transform"].items()),
           "map    " + ("as expected: " + rep["mapping"]["expected"]
                        if rep["mapping"]["as_expected"] else
                        "NOT as expected: " + "; ".join(rep["mapping"]["discovered"])
                        + (f"; unmatched {rep['mapping']['unmatched']}"
                           if rep["mapping"]["unmatched"] else "")),
           f"overlap {ov['overlap_samples']} RHX samples (t {ov['rhx_first']}..{ov['rhx_last']})",
           f"       compared {ov['compared']} channel-samples in {ov['frames_compared']} frames: "
           f"equal {ov['equal']}, mismatched {ov['mismatched']}",
           f"       frames missing {ov['frames_missing']}, partial {ov['frames_partial']}, "
           f"not TAP {ov['frames_not_tap']}, RHX gaps {ov['rhx_gaps']} "
           f"({ov['rhx_gap_samples']} samples)",
           f"outside chain frames before/after RHX {rep['outside']['chain_frames_before_rhx']}/"
           f"{rep['outside']['chain_frames_after_rhx']}, RHX samples before/after chain "
           f"{rep['outside']['rhx_samples_before_chain']}/{rep['outside']['rhx_samples_after_chain']}"]
    if rep["mismatches_per_channel"]:
        items = sorted(rep["mismatches_per_channel"].items(), key=lambda kv: -kv[1])
        out.append("mismatches per chain channel: "
                   + ", ".join(f"{k}:{v}" for k, v in items[:32])
                   + (" ..." if len(items) > 32 else ""))
        out.append("XOR bit histogram (bit 15..0): "
                   + " ".join(str(x) for x in reversed(rep["xor_bit_histogram"])))
        out.append("bits flipped per bad sample: "
                   + ", ".join(f"{k}:{v}" for k, v in rep["bits_flipped_histogram"].items()))
    if rep["first_mismatches"]:
        out.append(f"first {min(show, len(rep['first_mismatches']))} mismatches:")
        out.append("   seq        rhx t  chain ch  rhx ch    expected  got     xor")
        for m in rep["first_mismatches"][:show]:
            out.append(f"   {m['seq']:<10} {m['rhx_timestamp']:<6} {m['chain_ch']:>8}  "
                       f"{m['rhx_ch']:<8}  {m['expected']}    {m['got']}  {m['xor']}"
                       + ("  (partial frame)" if m["partial_frame"] else ""))
    for f in rep["findings"]:
        out.append(f"  [{f['status']}] {f['category']}: {f['message']}")
    out.append(f"RESULT {rep['result']}")
    return "\n".join(out)
