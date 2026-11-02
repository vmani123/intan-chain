"""Read Intan RHX recordings as raw ADC codes, whichever way RHX saved them.

    from chain import rhx
    rec = rhx.load(["/path/to/session_241102_120000"])     # folder, info.rhd or .rhd
    rec.t                   # int64 sample timestamps, one per sweep of the controller
    rec.codes(0, 1000)      # uint16 (rows, channels): the RHD2164's ADC words, offset binary
    rec.channels            # amplifier channels, in column order, with RHX's metadata

The tap check compares what the chain heard on the SPI wires with what RHX
wrote to disk, word for word, so this reader returns the ADC codes RHX saved
(offset binary, mid-scale 0x8000) and never microvolts: a float conversion
would hide a one-LSB disagreement behind rounding.

Two RHX save formats are read:

  Traditional Intan File Format   one .rhd: header, then data blocks of 60
                                  (software before v2.0) or 128 samples.
  One File Per Signal Type        a folder: info.rhd (header only), time.dat
                                  (int32 per sample), amplifier.dat (int16,
                                  time-major, value = code - 32768), ...

The header layout follows Intan's own Python loader (importrhdutilities.py in
github.com/Intan-Technologies/load-rhd-notebook-python, September 2023),
including its version branches; every field and branch below names the
loader function it mirrors. Where this reader differs from that loader it
says so: it keeps disabled channels in the metadata (marked), it does not
apply a notch filter (the codes are what the chip sent), and a file that ends
in a partial data block is read up to the last whole block with a warning
instead of being refused, because a check after an RHX crash should still be
able to say what it did get.

"One File Per Channel" is recognised and refused with a clear message; the
tap procedure asks for one of the two formats above.

numpy is required here (imported at module level): a minute at 2 kS/s is
15 million codes, and the reader memory-maps them instead of copying.
"""
import os
import pathlib
import struct
from dataclasses import dataclass, field

import numpy as np

MAGIC = 0xC6912702
SCALE_UV = 0.195            # microvolts per LSB of an amplifier code
MID = 0x8000                # amplifier codes are offset binary around this

# signal_type in the channel record (importrhdutilities.append_new_channel)
AMPLIFIER, AUX_INPUT, SUPPLY_VOLTAGE, BOARD_ADC, BOARD_DIG_IN, BOARD_DIG_OUT = range(6)
SIGNAL_TYPES = ("amplifier", "aux_input", "supply_voltage", "board_adc",
                "board_dig_in", "board_dig_out")

TRADITIONAL = "traditional"
PER_SIGNAL_TYPE = "one-file-per-signal-type"


class RhxError(ValueError):
    """The file is not what this reader can honestly decode."""


# ---------------------------------------------------------------------------
# Header
# ---------------------------------------------------------------------------

@dataclass
class Channel:
    """One channel record of the header (importrhdutilities.read_new_channel)."""
    port_name: str
    port_prefix: str
    port_number: int            # signal group index, counted from 1 as Intan does
    native_name: str            # e.g. "A-005"
    custom_name: str
    native_order: int
    custom_order: int
    signal_type: int
    enabled: bool
    chip_channel: int
    board_stream: int
    voltage_trigger_mode: int
    voltage_threshold: int
    digital_trigger_channel: int
    digital_edge_polarity: int
    impedance_magnitude: float
    impedance_phase: float

    def as_dict(self) -> dict:
        return {"native_name": self.native_name, "custom_name": self.custom_name,
                "port": self.port_prefix, "native_order": self.native_order,
                "custom_order": self.custom_order, "chip_channel": self.chip_channel,
                "board_stream": self.board_stream}


@dataclass
class Group:
    """One signal group (port) of the header (add_signal_group_information)."""
    name: str
    prefix: str
    enabled: bool
    num_channels: int
    num_amp_channels: int


@dataclass
class Header:
    version: tuple[int, int]
    samples_per_block: int
    sample_rate: float
    dsp_enabled: int
    actual_dsp_cutoff: float
    actual_lower_bandwidth: float
    actual_upper_bandwidth: float
    desired_dsp_cutoff: float
    desired_lower_bandwidth: float
    desired_upper_bandwidth: float
    notch_hz: int               # 0, 50 or 60
    desired_impedance_test_frequency: float
    actual_impedance_test_frequency: float
    notes: tuple[str, str, str]
    num_temp_sensor_channels: int
    eval_board_mode: int
    reference_channel: str | None
    groups: list[Group]
    channels: list[Channel]     # every record read, enabled or not
    header_bytes: int

    def enabled(self, signal_type: int) -> list[Channel]:
        """Channels whose data is in the file, in file order (append_new_channel)."""
        return [c for c in self.channels if c.enabled and c.signal_type == signal_type]

    @property
    def amplifier_channels(self) -> list[Channel]:
        return self.enabled(AMPLIFIER)

    @property
    def timestamps_signed(self) -> bool:
        """importrhdutilities.get_timestamp_signed: signed from v1.2 on."""
        major, minor = self.version
        return major > 1 or (major == 1 and minor >= 2)

    def bytes_per_block(self) -> int:
        """importrhdutilities.get_bytes_per_data_block, field for field."""
        n = self.samples_per_block
        b = n * 4                                                  # timestamps
        b += n * len(self.enabled(AMPLIFIER)) * 2
        b += (n // 4) * len(self.enabled(AUX_INPUT)) * 2          # aux at 1/4 rate
        b += 1 * len(self.enabled(SUPPLY_VOLTAGE)) * 2            # once per block
        b += n * len(self.enabled(BOARD_ADC)) * 2
        if self.enabled(BOARD_DIG_IN):
            b += n * 2                                             # one word, all inputs
        if self.enabled(BOARD_DIG_OUT):
            b += n * 2
        if self.num_temp_sensor_channels > 0:
            b += 1 * self.num_temp_sensor_channels * 2
        return b

    def block_dtype(self) -> np.dtype:
        """One data block as a numpy record, in the order read_one_data_block reads.

        The order differs from bytes_per_block's summation: the loader reads
        amplifier, aux, supply, temperature, board ADC, then digital in and out
        (read_analog_signals, read_digital_signals). Each analog type is stored
        channel-major inside the block (read_analog_signal_type reshapes to
        (channels, samples)).
        """
        n = self.samples_per_block
        fields = [("t", "<i4" if self.timestamps_signed else "<u4", (n,))]

        def add(name, nch, ns):
            if nch > 0 and ns > 0:
                fields.append((name, "<u2", (nch, ns)))
        add("amp", len(self.enabled(AMPLIFIER)), n)
        add("aux", len(self.enabled(AUX_INPUT)), n // 4)
        add("supply", len(self.enabled(SUPPLY_VOLTAGE)), 1)
        add("temp", self.num_temp_sensor_channels, 1)
        add("adc", len(self.enabled(BOARD_ADC)), n)
        if self.enabled(BOARD_DIG_IN):
            fields.append(("din", "<u2", (n,)))
        if self.enabled(BOARD_DIG_OUT):
            fields.append(("dout", "<u2", (n,)))
        dt = np.dtype(fields)
        if dt.itemsize != self.bytes_per_block():
            raise RhxError(f"internal: block record is {dt.itemsize} bytes, "
                           f"Intan's arithmetic says {self.bytes_per_block()}")
        return dt

    def summary(self) -> dict:
        return {
            "version": f"{self.version[0]}.{self.version[1]}",
            "sample_rate": self.sample_rate,
            "samples_per_block": self.samples_per_block,
            "amplifier_channels": len(self.amplifier_channels),
            "ports": [g.prefix for g in self.groups if g.num_amp_channels and g.enabled],
            "dsp_enabled": bool(self.dsp_enabled),
            "dsp_cutoff_hz": round(self.actual_dsp_cutoff, 4),
            "bandwidth_hz": [round(self.actual_lower_bandwidth, 4),
                             round(self.actual_upper_bandwidth, 2)],
            "notch_hz": self.notch_hz,
            "reference_channel": self.reference_channel,
            "eval_board_mode": self.eval_board_mode,
            "notes": [n for n in self.notes if n],
        }


class _Reader:
    """Little-endian fields off a file object, with the file's size for QString checks."""

    def __init__(self, fid):
        self.fid = fid
        self.size = os.fstat(fid.fileno()).st_size

    def unpack(self, fmt: str):
        n = struct.calcsize(fmt)
        raw = self.fid.read(n)
        if len(raw) != n:
            raise RhxError(f"header ends early at byte {self.fid.tell()}: wanted {n} more")
        return struct.unpack(fmt, raw)

    def qstring(self) -> str:
        """importrhdutilities.read_qstring: u32 byte length, 0xffffffff = null, UTF-16LE."""
        (length,) = self.unpack("<I")
        if length == 0xFFFFFFFF:
            return ""
        if length > self.size - self.fid.tell() + 1:
            raise RhxError(f"QString of {length} bytes at byte {self.fid.tell() - 4} "
                           f"runs past the end of the file: not an RHD header")
        raw = self.fid.read(length - length % 2)
        return raw.decode("utf-16-le", "replace")


def read_header(fid) -> Header:
    """importrhdutilities.read_header, in its order, with its version branches."""
    r = _Reader(fid)
    (magic,) = r.unpack("<I")                                   # check_magic_number
    if magic != MAGIC:
        raise RhxError(f"magic 0x{magic:08x}, want 0x{MAGIC:08x}: not an Intan RHD file "
                       f"(an .rhs file is the stimulation controller's, not this one)")
    major, minor = r.unpack("<hh")                              # read_version_number
    spb = 128 if major > 1 else 60                              # set_num_samples_per_data_block
    (sample_rate,) = r.unpack("<f")                             # read_sample_rate
    (dsp_enabled, a_dsp, a_lo, a_hi, d_dsp, d_lo, d_hi) = r.unpack("<hffffff")  # read_freq_settings
    (notch_mode,) = r.unpack("<h")                              # read_notch_filter_frequency
    notch = {1: 50, 2: 60}.get(notch_mode, 0)
    d_imp, a_imp = r.unpack("<ff")                              # read_impedance_test_frequencies
    notes = (r.qstring(), r.qstring(), r.qstring())             # read_notes
    temp = 0
    if (major == 1 and minor >= 1) or major > 1:                # read_num_temp_sensor_channels
        (temp,) = r.unpack("<h")
    board_mode = 0
    if (major == 1 and minor >= 3) or major > 1:                # read_eval_board_mode
        (board_mode,) = r.unpack("<h")
    reference = r.qstring() if major > 1 else None              # read_reference_channel

    groups, channels = [], []
    (n_groups,) = r.unpack("<h")                                # read_signal_summary
    for g in range(1, n_groups + 1):                            # add_signal_group_information
        name, prefix = r.qstring(), r.qstring()
        g_enabled, g_nch, g_namp = r.unpack("<hhh")
        groups.append(Group(name, prefix, g_enabled > 0, g_nch, g_namp))
        # Channel records exist only for an enabled group with channels; the
        # loader reads nothing else, so neither does this.
        if g_nch > 0 and g_enabled > 0:
            for _ in range(g_nch):                              # read_new_channel
                native, custom = r.qstring(), r.qstring()
                (n_ord, c_ord, stype, ch_en, chip_ch, stream) = r.unpack("<hhhhhh")
                (vt_mode, vt_thr, dt_ch, dt_pol) = r.unpack("<hhhh")
                (imp_mag, imp_ph) = r.unpack("<ff")
                if not 0 <= stype <= 5:
                    raise RhxError(f"channel {native!r}: unknown signal type {stype}")
                channels.append(Channel(name, prefix, g, native, custom, n_ord, c_ord, stype,
                                        bool(ch_en), chip_ch, stream, vt_mode, vt_thr, dt_ch,
                                        dt_pol, imp_mag, imp_ph))
    return Header((major, minor), spb, sample_rate, dsp_enabled, a_dsp, a_lo, a_hi, d_dsp,
                  d_lo, d_hi, notch, d_imp, a_imp, notes, temp, board_mode, reference, groups,
                  channels, fid.tell())


def read_header_file(path) -> Header:
    with open(path, "rb") as fid:
        return read_header(fid)


# ---------------------------------------------------------------------------
# Data: one segment is one file (traditional) or one folder (per signal type)
# ---------------------------------------------------------------------------

@dataclass
class Segment:
    path: pathlib.Path
    format: str
    header: Header
    t: np.ndarray                       # int64 timestamps
    _amp: object = field(repr=False, default=None)    # memmap behind codes()
    warnings: list = field(default_factory=list)

    @property
    def n(self) -> int:
        return len(self.t)

    def codes(self, start: int, stop: int) -> np.ndarray:
        """uint16 (stop - start, channels) amplifier codes, offset binary."""
        start, stop = max(0, start), min(self.n, stop)
        nch = len(self.header.amplifier_channels)
        if stop <= start:
            return np.zeros((0, nch), np.uint16)
        if self.format == PER_SIGNAL_TYPE:
            # amplifier.dat holds code - 32768 as int16; flipping the top bit of
            # the two's-complement pattern is exactly adding 32768 mod 2^16.
            return self._amp[start:stop].view(np.uint16) ^ np.uint16(MID)
        spb = self.header.samples_per_block
        b0, b1 = start // spb, (stop - 1) // spb + 1
        amp = np.asarray(self._amp[b0:b1]["amp"])            # (blocks, ch, spb)
        rows = amp.transpose(0, 2, 1).reshape(-1, nch)
        return np.ascontiguousarray(rows[start - b0 * spb:stop - b0 * spb])


def _traditional(path: pathlib.Path, h: Header) -> Segment:
    size = path.stat().st_size
    per = h.bytes_per_block()
    rest = size - h.header_bytes
    nblk, extra = divmod(rest, per)
    warnings = []
    if extra:
        # Intan's calculate_data_size raises here; a check after an RHX crash
        # is better served by every whole block and a loud note.
        warnings.append(f"{path.name}: {extra} bytes after the last whole data block "
                        f"({per} bytes each) were ignored")
    if nblk == 0:
        mm = np.zeros(0, h.block_dtype())
        t = np.zeros(0, np.int64)
    else:
        mm = np.memmap(path, dtype=h.block_dtype(), mode="r", offset=h.header_bytes,
                       shape=(nblk,))
        t = np.asarray(mm["t"]).reshape(-1).astype(np.int64)
    return Segment(path, TRADITIONAL, h, t, mm, warnings)


def _per_signal_type(folder: pathlib.Path, h: Header) -> Segment:
    nch = len(h.amplifier_channels)
    tpath, apath = folder / "time.dat", folder / "amplifier.dat"
    if not tpath.exists():
        raise RhxError(f"{folder}: info.rhd without time.dat; is this a complete "
                       f"'One File Per Signal Type' folder?")
    if nch == 0:
        raise RhxError(f"{folder}: the header lists no enabled amplifier channels")
    if not apath.exists():
        raise RhxError(f"{folder}: no amplifier.dat (amplifier channels were not saved)")
    warnings = []
    nt = tpath.stat().st_size // 4
    na, extra = divmod(apath.stat().st_size, 2 * nch)
    if extra:
        warnings.append(f"{apath.name}: {extra} trailing bytes are not a whole sample of "
                        f"{nch} channels and were ignored")
    n = min(nt, na)
    if nt != na:
        # RHX writes the files side by side; a stop mid-write can leave one a
        # little longer. Only samples present in both can be checked.
        warnings.append(f"time.dat holds {nt} samples, amplifier.dat {na}: using the "
                        f"first {n}")
    tdt = "<i4" if h.timestamps_signed else "<u4"
    t = np.fromfile(tpath, dtype=tdt, count=n).astype(np.int64)
    amp = (np.memmap(apath, dtype="<i2", mode="r", shape=(n, nch)) if n
           else np.zeros((0, nch), "<i2"))
    return Segment(folder, PER_SIGNAL_TYPE, h, t, amp, warnings)


def open_segment(path) -> Segment:
    """One RHX save: a traditional .rhd, an info.rhd, or the folder holding either."""
    p = pathlib.Path(path).expanduser()
    if p.is_dir():
        if (p / "info.rhd").exists():
            if any(p.glob("amp-*.dat")) and not (p / "amplifier.dat").exists():
                raise RhxError(f"{p} is a 'One File Per Channel' save (amp-*.dat); record "
                               f"with 'One File Per Signal Type' or 'Traditional' instead")
            return _per_signal_type(p, read_header_file(p / "info.rhd"))
        rhds = sorted(p.glob("*.rhd"))
        if len(rhds) == 1:
            return open_segment(rhds[0])
        raise RhxError(f"{p}: no info.rhd and {len(rhds)} .rhd files; name the file(s)")
    if not p.exists():
        raise RhxError(f"{p}: no such file or folder")
    h = read_header_file(p)
    if p.stat().st_size > h.header_bytes:
        return _traditional(p, h)
    # A header with no data is the info.rhd of a per-signal-type folder.
    if (p.parent / "amplifier.dat").exists() or (p.parent / "time.dat").exists():
        return _per_signal_type(p.parent, h)
    if any(p.parent.glob("amp-*.dat")):
        raise RhxError(f"{p.parent} is a 'One File Per Channel' save; record with 'One "
                       f"File Per Signal Type' or 'Traditional' instead")
    raise RhxError(f"{p}: a header with no data and no amplifier.dat beside it")


# ---------------------------------------------------------------------------
# A recording: one or more segments in timestamp order
# ---------------------------------------------------------------------------

@dataclass
class Recording:
    segments: list[Segment]
    t: np.ndarray
    warnings: list

    @property
    def header(self) -> Header:
        return self.segments[0].header

    @property
    def channels(self) -> list[Channel]:
        return self.header.amplifier_channels

    @property
    def sample_rate(self) -> float:
        return self.header.sample_rate

    @property
    def format(self) -> str:
        fmts = sorted({s.format for s in self.segments})
        return fmts[0] if len(fmts) == 1 else "+".join(fmts)

    @property
    def n(self) -> int:
        return len(self.t)

    def codes(self, start: int, stop: int) -> np.ndarray:
        """uint16 amplifier codes for global rows [start, stop), across segments."""
        start, stop = max(0, start), min(self.n, stop)
        parts, base = [], 0
        for s in self.segments:
            lo, hi = max(start, base), min(stop, base + s.n)
            if lo < hi:
                parts.append(s.codes(lo - base, hi - base))
            base += s.n
        if not parts:
            return np.zeros((0, len(self.channels)), np.uint16)
        return parts[0] if len(parts) == 1 else np.concatenate(parts)

    def gaps(self) -> list[tuple[int, int]]:
        """(row, missing samples) wherever the timestamp does not step by exactly 1."""
        if self.n < 2:
            return []
        d = np.diff(self.t)
        idx = np.nonzero(d != 1)[0]
        return [(int(i) + 1, int(d[i]) - 1) for i in idx]

    def describe(self) -> dict:
        return {"format": self.format,
                "paths": [str(s.path) for s in self.segments],
                "samples": self.n,
                "first_timestamp": int(self.t[0]) if self.n else None,
                "last_timestamp": int(self.t[-1]) if self.n else None,
                "seconds": round(self.n / self.sample_rate, 3) if self.sample_rate else None,
                **self.header.summary()}


def load(paths) -> Recording:
    """One recording from one or more saves ("start a new file every N minutes").

    Segments are ordered by first timestamp and must agree on sample rate and
    amplifier channels; anything else would make one column mean two things.
    """
    if isinstance(paths, (str, os.PathLike)):
        paths = [paths]
    segs = [open_segment(p) for p in paths]
    if not segs:
        raise RhxError("no RHX file given")
    segs.sort(key=lambda s: int(s.t[0]) if s.n else 1 << 62)
    ref = segs[0].header
    names = [c.native_name for c in ref.amplifier_channels]
    for s in segs[1:]:
        if s.header.sample_rate != ref.sample_rate:
            raise RhxError(f"{s.path}: {s.header.sample_rate:g} S/s, the first segment "
                           f"{ref.sample_rate:g}")
        if [c.native_name for c in s.header.amplifier_channels] != names:
            raise RhxError(f"{s.path}: a different set of amplifier channels than {segs[0].path}")
    t = np.concatenate([s.t for s in segs]) if segs else np.zeros(0, np.int64)
    warnings = [w for s in segs for w in s.warnings]
    if len(t) > 1 and np.any(np.diff(t) <= 0):
        warnings.append("timestamps go backwards or repeat somewhere; samples are compared "
                        "by timestamp, so repeated ones are compared once")
    return Recording(segs, t, warnings)
