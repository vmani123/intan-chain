"""Frame and datagram encode/decode, CONTRACTS.md sections 1 and 2."""
import struct
from dataclasses import dataclass

from . import contract as C


class FrameError(ValueError):
    """Anything the wire format says must not happen."""


@dataclass
class Frame:
    magic: int
    type: int
    flags: int
    seq: int
    t_cap: int
    t_tx: int
    n_ch: int
    payload_len: int
    payload: bytes  # the whole fixed-size segment, padding included

    @property
    def realistic(self) -> bool:
        return bool(self.flags & C.FLAG_REALISTIC)

    def samples(self) -> list[int]:
        """The int16 values that are actually real (payload_len says how many)."""
        if self.type != C.TYPE_RAW16:
            raise FrameError(f"frame seq {self.seq} is type {self.type}, not RAW16; cannot read samples")
        return list(struct.unpack_from(f"<{self.payload_len // 2}h", self.payload, 0))


def parse_header(buf: bytes) -> dict:
    """Header fields as a dict, or FrameError naming exactly what was wrong."""
    if len(buf) < C.HDR_BYTES:
        raise FrameError(f"header needs {C.HDR_BYTES} bytes, got {len(buf)}")
    h = dict(zip(C.HDR_FIELDS, struct.unpack_from(C.HDR_FMT, buf, 0)))
    if h["magic"] != C.MAGIC:
        raise FrameError(f"bad magic 0x{h['magic']:04x}, want 0x{C.MAGIC:04x}")
    if h["n_ch"] == 0:
        raise FrameError("bad n_ch 0")
    seg = h["n_ch"] * 2
    if h["payload_len"] == 0 or h["payload_len"] > seg:
        raise FrameError(
            f"bad payload_len {h['payload_len']}: must be 1..{seg} for n_ch={h['n_ch']}")
    if h["payload_len"] % 2:
        raise FrameError(f"bad payload_len {h['payload_len']}: not a whole number of int16")
    # Segments are fixed size always, so a short RAW16 payload means the
    # producer disagrees about the frame layout, not that it sent less data.
    if h["type"] == C.TYPE_RAW16 and h["payload_len"] != seg:
        raise FrameError(f"RAW16 payload_len {h['payload_len']} != n_ch*2 = {seg}")
    return h


def parse_frame(buf: bytes) -> Frame:
    """One fixed-size segment: header plus n_ch*2 payload bytes."""
    h = parse_header(buf)
    seg = h["n_ch"] * 2
    if len(buf) != C.HDR_BYTES + seg:
        raise FrameError(f"frame is {len(buf)} bytes, want {C.HDR_BYTES + seg} for n_ch={h['n_ch']}")
    return Frame(payload=bytes(buf[C.HDR_BYTES:]), **h)


def parse_datagram(data: bytes) -> tuple[list[Frame], dict]:
    """FRAMES_PER_DGRAM frames plus the ESP trailer."""
    if len(data) < C.HDR_BYTES + C.TRAILER_BYTES:
        raise FrameError(f"datagram is {len(data)} bytes, too short to hold a frame and a trailer")
    stride = C.HDR_BYTES + parse_header(data)["n_ch"] * 2
    want = C.FRAMES_PER_DGRAM * stride + C.TRAILER_BYTES
    if len(data) != want:
        raise FrameError(f"datagram is {len(data)} bytes, want {want} "
                         f"({C.FRAMES_PER_DGRAM} x {stride} + {C.TRAILER_BYTES})")
    frames = [parse_frame(data[i * stride:(i + 1) * stride]) for i in range(C.FRAMES_PER_DGRAM)]
    trailer = dict(zip(C.TRAILER_FIELDS,
                       struct.unpack_from(C.TRAILER_FMT, data, C.FRAMES_PER_DGRAM * stride)))
    return frames, trailer


def build_frame(seq: int, samples: list[int], t_cap: int = 0, t_tx: int = 0,
                flags: int = 0, type_: int = C.TYPE_RAW16) -> bytes:
    """One RAW16 segment. Used by the tests and by the dry-run traffic source."""
    n_ch = len(samples)
    body = struct.pack(f"<{n_ch}h", *samples)
    hdr = struct.pack(C.HDR_FMT, C.MAGIC, type_, flags, seq & 0xFFFFFFFF,
                      t_cap & 0xFFFFFFFF, t_tx & 0xFFFFFFFF, n_ch, len(body))
    return hdr + body


def build_trailer(esp_rx_us: int = 0, esp_tx_us: int = 0, pipe_depth: int = 0) -> bytes:
    return struct.pack(C.TRAILER_FMT, esp_rx_us & 0xFFFFFFFF, esp_tx_us & 0xFFFFFFFF, pipe_depth)


def build_datagram(frames: list[bytes], esp_rx_us: int = 0, esp_tx_us: int = 0,
                   pipe_depth: int = 0) -> bytes:
    if len(frames) != C.FRAMES_PER_DGRAM:
        raise FrameError(f"datagram needs {C.FRAMES_PER_DGRAM} frames, got {len(frames)}")
    return b"".join(frames) + build_trailer(esp_rx_us, esp_tx_us, pipe_depth)
