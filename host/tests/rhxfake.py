"""Test-only writers: RHX files in both save formats, and tap recordings.

The RHX writer lays bytes out the way Intan's importrhdutilities.py reads them
(and so the way chain.rhx reads them); test_rhx.py also checks the reader
against real RHX files when they are available, so the two cannot drift
together unnoticed. The tap-recording builder makes datagrams with
chain.frame and writes them with chain.taprec, exactly as `tapcheck record`
would have stored them.
"""
import struct

import numpy as np

from chain import contract as C
from chain import frame as F
from chain import taprec


# ---------------------------------------------------------------------------
# RHX
# ---------------------------------------------------------------------------

def qstring(s: str | None) -> bytes:
    if s is None:
        return struct.pack("<I", 0xFFFFFFFF)
    raw = s.encode("utf-16-le")
    return struct.pack("<I", len(raw)) + raw


def channel(native, custom=None, native_order=0, custom_order=None, signal_type=0,
            enabled=True, chip_channel=0, board_stream=0, impedance=(1.0e5, -30.0)):
    return dict(native=native, custom=custom if custom is not None else native,
                native_order=native_order,
                custom_order=native_order if custom_order is None else custom_order,
                signal_type=signal_type, enabled=enabled, chip_channel=chip_channel,
                board_stream=board_stream, impedance=impedance)


def amp_port(prefix="A", n=128, name=None, disabled=()):
    """One headstage port: n amplifier channels <prefix>-000.., MISO1's chip first.

    chip_channel and board_stream follow how an RHD2164 appears to Rhythm: 32
    channels per data stream (A or B half of one chip), four streams for two
    chips. They are metadata only; nothing here depends on them.
    """
    chans = [channel(f"{prefix}-{k:03d}", native_order=k, chip_channel=k % 64,
                     board_stream=k // 32, enabled=k not in disabled) for k in range(n)]
    return dict(name=name or f"Port {prefix}", prefix=prefix, enabled=True, channels=chans)


def board_group(prefix, name, signal_type, n, start=0):
    return dict(name=name, prefix=prefix, enabled=True,
                channels=[channel(f"{prefix}-{k:02d}", native_order=start + k,
                                  signal_type=signal_type) for k in range(n)])


def header_bytes(groups, version=(3, 3), sample_rate=2000.0, notch_mode=0,
                 notes=("tap check", "", ""), reference="n/a", temp=0, board_mode=0,
                 dsp=(1, 1.0, 1.0, 500.0, 1.0, 1.0, 500.0), impedance=(1000.0, 1000.0)) -> bytes:
    major, minor = version
    out = struct.pack("<I", 0xC6912702) + struct.pack("<hh", major, minor)
    out += struct.pack("<f", sample_rate)
    out += struct.pack("<hffffff", *dsp)
    out += struct.pack("<h", notch_mode)
    out += struct.pack("<ff", *impedance)
    out += b"".join(qstring(n) for n in notes)
    if (major == 1 and minor >= 1) or major > 1:
        out += struct.pack("<h", temp)
    if (major == 1 and minor >= 3) or major > 1:
        out += struct.pack("<h", board_mode)
    if major > 1:
        out += qstring(reference)
    out += struct.pack("<h", len(groups))
    for g in groups:
        nch = len(g["channels"])
        namp = sum(1 for c in g["channels"] if c["signal_type"] == 0)
        out += qstring(g["name"]) + qstring(g["prefix"])
        out += struct.pack("<hhh", 1 if g["enabled"] else 0, nch, namp)
        if nch and g["enabled"]:
            for c in g["channels"]:
                out += qstring(c["native"]) + qstring(c["custom"])
                out += struct.pack("<hhhhhh", c["native_order"], c["custom_order"],
                                   c["signal_type"], 1 if c["enabled"] else 0,
                                   c["chip_channel"], c["board_stream"])
                out += struct.pack("<hhhh", 0, 0, 0, 0)
                out += struct.pack("<ff", *c["impedance"])
    return out


def _enabled(groups, st):
    return [c for g in groups if g["enabled"] for c in g["channels"]
            if c["enabled"] and c["signal_type"] == st]


def write_traditional(path, codes, t, groups=None, version=(3, 3), temp=0, **hdr):
    """codes: (n, amp channels) uint16; n a whole number of blocks.

    Other enabled signal types get deterministic filler, so the block layout
    around the amplifier data is exercised, not skipped.
    """
    groups = groups if groups is not None else [amp_port()]
    spb = 128 if version[0] > 1 else 60
    codes = np.asarray(codes, np.uint16)
    n, nch = codes.shape
    assert n % spb == 0, "traditional files hold whole blocks"
    signed = version[0] > 1 or (version[0] == 1 and version[1] >= 2)
    naux, nsup = len(_enabled(groups, 1)), len(_enabled(groups, 2))
    nadc, ndin, ndout = (len(_enabled(groups, s)) for s in (3, 4, 5))
    body = bytearray()
    for b in range(n // spb):
        rows = slice(b * spb, (b + 1) * spb)
        body += np.asarray(t[rows], "<i4" if signed else "<u4").tobytes()
        body += codes[rows].T.astype("<u2").tobytes()                 # channel-major
        body += (np.arange(naux * (spb // 4)) + 7 * b).astype("<u2").tobytes()
        body += (np.arange(nsup) + 100 + b).astype("<u2").tobytes()
        body += (np.arange(temp) + 2500).astype("<u2").tobytes()
        body += (np.arange(nadc * spb) % 65536).astype("<u2").tobytes()
        if ndin:
            body += (np.arange(spb) % 16).astype("<u2").tobytes()
        if ndout:
            body += (np.arange(spb) % 8).astype("<u2").tobytes()
    with open(path, "wb") as f:
        f.write(header_bytes(groups, version=version, temp=temp, **hdr))
        f.write(bytes(body))


def write_per_signal_type(folder, codes, t, groups=None, version=(3, 3), **hdr):
    """info.rhd (header only), time.dat (int32), amplifier.dat (int16 = code - 32768)."""
    groups = groups if groups is not None else [amp_port()]
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "info.rhd").write_bytes(header_bytes(groups, version=version, **hdr))
    (folder / "time.dat").write_bytes(np.asarray(t, "<i4").tobytes())
    amp = (np.asarray(codes, np.uint16) ^ np.uint16(0x8000)).view("<i2")
    (folder / "amplifier.dat").write_bytes(amp.tobytes())


# ---------------------------------------------------------------------------
# Signals and tap recordings
# ---------------------------------------------------------------------------

def signal(n, n_ch=C.N_CH, seed=1, noise=12.0):
    """Offset-binary codes that look like quiet EMG: per-channel offset, a slow wave, noise."""
    rng = np.random.default_rng(seed)
    k = np.arange(n_ch)
    off = 0x8000 + (k * 37) % 400 - 200
    wave = 60 * np.sin(2 * np.pi * np.arange(n)[:, None] / (400 + 3 * k[None, :]))
    x = off[None, :] + wave + rng.normal(0, noise, (n, n_ch))
    return np.clip(np.round(x), 0, 65535).astype(np.uint16)


def datagrams(words, seq0=0, flags=None, drop=(), stale=None):
    """Datagrams for frames (seq0 + i, words[i]); `drop` lists datagram indices to lose.

    `stale` (a list of (seq, words)) is put ahead of the first frames in the
    first datagram, as the FPGA does with a datagram left from the last run.
    """
    words = np.asarray(words, np.uint16)
    n = len(words)
    frames = []
    for i in range(n):
        vals = words[i].astype(np.int32)
        vals = np.where(vals >= 0x8000, vals - 0x10000, vals).tolist()
        fl = C.FLAG_TAP if flags is None else int(flags[i])
        frames.append(F.build_frame((seq0 + i) & 0xFFFFFFFF, vals, t_cap=i * 48000,
                                    t_tx=i * 48000 + 900, flags=fl))
    if stale:
        old = []
        for s, w in stale:
            v = np.asarray(w, np.int32)
            old.append(F.build_frame(s, np.where(v >= 0x8000, v - 0x10000, v).tolist(),
                                     flags=C.FLAG_TAP))
        frames = old + frames
    out = []
    for d in range(len(frames) // C.FRAMES_PER_DGRAM):
        if d in drop:
            continue
        chunk = frames[d * C.FRAMES_PER_DGRAM:(d + 1) * C.FRAMES_PER_DGRAM]
        out.append(F.build_datagram(chunk, esp_rx_us=d * 2500, esp_tx_us=d * 2500 + 300))
    return out


def write_recording(path, dgrams, t0=1000.0, dt=0.0025, extra=()):
    """A .chaintap file holding these datagrams (plus raw `extra` ones appended)."""
    clock = iter([t0] + [t0 + i * dt for i in range(len(dgrams) + len(extra))])
    with taprec.Writer(path, clock=lambda: next(clock), wall=lambda: 1.7e9) as w:
        for i, d in enumerate(list(dgrams) + list(extra)):
            w.add(t0 + i * dt, d)
    return path
