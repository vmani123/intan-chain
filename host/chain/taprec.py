"""The tap check's capture file: every datagram exactly as it arrived, plus when.

    file    = header, then records until end of file
    header  = b"CHAINTAP" (8 bytes), u16 version (1), u16 reserved,
              f64 wall-clock start (time.time()), f64 host monotonic at start
    record  = f64 host monotonic when recv() returned, u16 length, `length` bytes

All little-endian. Nothing is parsed while recording: a parse in the receive
loop puts Python work between two packets and turns host jitter into apparent
loss (the same reasoning as receiver.py), so the bytes go to disk as they came
and `tapcheck compare` parses them afterwards. Datagrams that do not parse are
still kept, so a malformed one can be looked at later instead of vanishing.

A recording cut short (power, kill -9) ends in a partial record; readers stop
at the last whole one and say so.

The JSON sidecar (same name, .json) holds the board's counters and settings;
it is written by `tapcheck record` when the run stops.
"""
import pathlib
import struct
import time

MAGIC = b"CHAINTAP"
VERSION = 1
HEADER = struct.Struct("<8sHHdd")
RECORD = struct.Struct("<dH")
SUFFIX = ".chaintap"


class TapRecError(ValueError):
    """Not a tap recording, or one this version cannot read."""


class Writer:
    """Append-only, buffered: one write() per datagram, flushed by the OS in big pieces."""

    def __init__(self, path, buffering: int = 1 << 20, clock=time.monotonic, wall=time.time):
        self.path = pathlib.Path(path)
        self.fh = open(self.path, "wb", buffering=buffering)
        self.fh.write(HEADER.pack(MAGIC, VERSION, 0, wall(), clock()))
        self.datagrams = 0
        self.bytes = 0

    def add(self, t: float, data: bytes) -> None:
        self.fh.write(RECORD.pack(t, len(data)) + data)
        self.datagrams += 1
        self.bytes += len(data)

    def close(self) -> None:
        if not self.fh.closed:
            self.fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def sidecar(path) -> pathlib.Path:
    return pathlib.Path(path).with_suffix(".json")


def read_header(buf: bytes) -> dict:
    if len(buf) < HEADER.size:
        raise TapRecError(f"{len(buf)} bytes: too short for a tap recording header")
    magic, version, _r, wall, mono = HEADER.unpack_from(buf, 0)
    if magic != MAGIC:
        raise TapRecError(f"magic {magic!r}, want {MAGIC!r}: not a `tapcheck record` file")
    if version != VERSION:
        raise TapRecError(f"recording format version {version}; this host reads {VERSION}")
    return {"version": version, "wall_start": wall, "mono_start": mono}


def scan(buf: bytes) -> tuple[dict, list, list, list, int]:
    """(header, times, offsets, lengths, trailing bytes) without copying any datagram.

    Big recordings are handed to numpy by offset; copying every datagram into
    its own bytes object first would double the memory for nothing.
    """
    head = read_header(buf)
    times, offsets, lengths = [], [], []
    pos, end = HEADER.size, len(buf)
    unpack = RECORD.unpack_from
    while pos + RECORD.size <= end:
        t, n = unpack(buf, pos)
        start = pos + RECORD.size
        if start + n > end:
            break
        times.append(t)
        offsets.append(start)
        lengths.append(n)
        pos = start + n
    return head, times, offsets, lengths, end - pos


def read(path) -> tuple[dict, list[tuple[float, bytes]], int]:
    """(header, [(t_host, datagram), ...], trailing bytes ignored). For small files."""
    buf = pathlib.Path(path).read_bytes()
    head, times, offsets, lengths, trailing = scan(buf)
    return head, [(t, buf[o:o + n]) for t, o, n in zip(times, offsets, lengths)], trailing
