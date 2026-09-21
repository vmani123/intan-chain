"""Golden test pattern, CONTRACTS.md section 3.

The FPGA generates this and the host regenerates it; a single bit of
disagreement here would show up as a permanent "bit error" in every accuracy
number, so the conversions below are spelled out rather than implied.
"""
from .contract import N_CH

MASK16 = 0xFFFF
TAPS = 0xB400  # x^16 + x^14 + x^13 + x^11, Fibonacci form


def lfsr_next(state: int) -> int:
    """One step of the shared LFSR."""
    state &= MASK16
    return (state >> 1) ^ (TAPS if state & 1 else 0)


def seed(ch: int) -> int:
    """Reset state of one channel. Never 0: an all-zero LFSR is stuck."""
    return ch + 1


def to_int16(value: int) -> int:
    """Reinterpret the low 16 bits as two's complement.

    The RTL just drives 16 wires; "signed" is a host-side interpretation, so do
    it in one place instead of letting struct's 'h' and arithmetic disagree.
    """
    value &= MASK16
    return value - 0x10000 if value & 0x8000 else value


def _values(index: int, states: list[int], realistic: bool) -> list[int]:
    if not realistic:
        # CHECK: the value *is* the state, read as signed.
        return [to_int16(s) for s in states]
    out = []
    for ch, s in enumerate(states):
        phase = (index + 8 * ch) & 0xFF
        tri = phase * 128 - 8192 if phase < 128 else (255 - phase) * 128 - 8192
        # Truncating to 16 bits cannot bite at these amplitudes, but the RTL
        # truncates, so do the same and stay bit-exact if the shape ever grows.
        out.append(to_int16(tri + (s >> 12) - 8))
    return out


class PatternStream:
    """Per-channel LFSR state, advanced one step per sample."""

    def __init__(self, realistic: bool = False, n_ch: int = N_CH):
        self.realistic = realistic
        self.n_ch = n_ch
        self.index = 0
        self.state = [seed(ch) for ch in range(n_ch)]

    def next_sample(self) -> list[int]:
        """Values for the current sample index, then advance every channel.

        Sample 0 therefore emits the seed state itself, matching an RTL
        register that is sampled combinationally and clocked on the sample
        strobe. CONTRACTS.md does not pin the order down; this is the choice.
        """
        vals = _values(self.index, self.state, self.realistic)
        self.state = [lfsr_next(s) for s in self.state]
        self.index += 1
        return vals


# Random access. A forward cursor makes the common sequential case O(1) per
# sample; checkpoints bound the cost of a jump backwards (reordered frames)
# without caching every sample's 128 values.
_STRIDE = 4096
_CKPTS: dict[int, list[list[int]]] = {}
_CURSOR: dict[int, tuple[int, list[int]]] = {}


def _states_at(index: int, n_ch: int) -> list[int]:
    ck = _CKPTS.setdefault(n_ch, [[seed(ch) for ch in range(n_ch)]])
    idx, st = _CURSOR.get(n_ch, (0, list(ck[0])))
    if index < idx:
        k = min(index // _STRIDE, len(ck) - 1)
        idx, st = k * _STRIDE, list(ck[k])
    while idx < index:
        st = [lfsr_next(s) for s in st]
        idx += 1
        if idx % _STRIDE == 0 and idx // _STRIDE == len(ck):
            ck.append(list(st))
    _CURSOR[n_ch] = (idx, st)
    return st


PERIOD = 65535  # the tap set is maximal, so every nonzero seed rides one cycle


def expected_sample(index: int, realistic: bool = False, n_ch: int = N_CH) -> list[int]:
    """Values the FPGA must have produced for sample `index`.

    The LFSR half is reduced modulo the period: seq is a 32-bit counter, and a
    frame that arrives after the wrap must not cost four billion steps. The
    triangle half keeps the true index (it only wants index mod 256).
    """
    return _values(index, _states_at(index % PERIOD, n_ch), realistic)
