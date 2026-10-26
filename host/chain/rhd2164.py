"""What the host knows about the Intan RHD2164: command words, ROM, registers, MISO.

Pure functions, standard library only, so the bring-up logic and its tests run
on a machine with no serial port and no numpy.

Sources, in order of authority: CONTRACTS.md section 6 (command words, ROM,
tap numbering, the two-cycle pipeline), the RHD2000 series datasheet, and
Intan's Rhythm `Rhd2000Registers` class, whose register recipe is copied here
field for field so a chip configured by this file behaves like one configured
by Intan's own software.
"""
import math
from dataclasses import dataclass

from . import contract as C

# ---------------------------------------------------------------------------
# Command words (CONTRACTS.md section 6)
# ---------------------------------------------------------------------------

SCALE_UV = 0.195            # microvolts per LSB of a CONVERT result
CALIBRATE = 0x5500
CLEAR = 0x6A00
CALIBRATE_DUMMIES = 9       # commands consumed by calibration after CALIBRATE
PIPELINE = 2                # a result arrives this many CS cycles after its command


def convert(c: int, h: int = 0) -> int:
    """CONVERT(c): 00cccccc 0000000h. h=1 resets the DSP offset-removal filter."""
    if not 0 <= c <= 63:
        raise ValueError(f"CONVERT channel {c} out of range 0..63")
    return (c << 8) | (h & 1)


def read(r: int) -> int:
    """READ(r): 11rrrrrr 00000000."""
    if not 0 <= r <= 63:
        raise ValueError(f"register {r} out of range 0..63")
    return 0xC000 | (r << 8)


def write(r: int, d: int) -> int:
    """WRITE(r, d): 10rrrrrr dddddddd."""
    if not 0 <= r <= 63:
        raise ValueError(f"register {r} out of range 0..63")
    if not 0 <= d <= 0xFF:
        raise ValueError(f"register value {d} out of range 0..255")
    return 0x8000 | (r << 8) | d


# READ(63) is the conventional pipeline flusher: harmless, and its answer (the
# chip ID) is itself a link check if anyone looks.
FLUSH = read(63)
FLUSHERS = [FLUSH] * PIPELINE


def kind(cmd: int) -> str:
    """'CONVERT', 'READ', 'WRITE', 'CALIBRATE', 'CLEAR' or 'OTHER'."""
    top = cmd >> 14
    if top == 0:
        return "CONVERT"
    if top == 3:
        return "READ"
    if top == 2:
        return "WRITE"
    return {CALIBRATE: "CALIBRATE", CLEAR: "CLEAR"}.get(cmd, "OTHER")


def describe(cmd: int) -> str:
    """A command word as a person would write it: READ(63), WRITE(4, 0xd8)."""
    k = kind(cmd)
    reg = (cmd >> 8) & 0x3F
    if k == "CONVERT":
        return f"CONVERT({reg}{', H' if cmd & 1 else ''})"
    if k == "READ":
        return f"READ({reg})"
    if k == "WRITE":
        return f"WRITE({reg}, 0x{cmd & 0xFF:02x})"
    if k == "OTHER":
        return f"0x{cmd:04x}"
    return k


# ---------------------------------------------------------------------------
# Result words
# ---------------------------------------------------------------------------

def read_result(word: int) -> int | None:
    """The register byte of a READ result (0x00 then the byte), or None if malformed."""
    return word & 0xFF if (word >> 8) == 0x00 else None


def write_echo(word: int) -> int | None:
    """The data byte of a WRITE echo (0xFF then d), or None if malformed."""
    return word & 0xFF if (word >> 8) == 0xFF else None


def to_signed(word: int) -> int:
    """A CONVERT result as two's complement, which register 4 bit 6 selects.

    The frame's payload is int16 and this file configures twos_comp = 1, so the
    diagnostic path and the streaming path agree on every value.
    """
    word &= 0xFFFF
    return word - 0x10000 if word & 0x8000 else word


def to_uv(word: int) -> float:
    return to_signed(word) * SCALE_UV


# ---------------------------------------------------------------------------
# ROM (CONTRACTS.md section 6; Rhythm's deviceId() for what module B returns)
# ---------------------------------------------------------------------------

MARKER_REG = 59
DIE_REV_REG = 60
ID_REG = 63
MARKER_A = 53
MARKER_B = 58
CHIP_ID_RHD2164 = 4
CHIP_NAMES = {1: "RHD2132", 2: "RHD2216", 4: "RHD2164"}
INTAN = {40 + i: ord(ch) for i, ch in enumerate("INTAN")}

# What each module must answer. Module A: everything the contract lists.
# Module B: the A/B marker (the contract), plus "INTAN" and the chip ID, which
# Rhythm's deviceId() reads on RHD2164 DDR (module B) streams and requires
# before it accepts one. Die revision (60) is informational: it varies by lot.
ROM_A = {**INTAN, MARKER_REG: MARKER_A, 61: 1, 62: 64, ID_REG: CHIP_ID_RHD2164}
ROM_B = {**INTAN, MARKER_REG: MARKER_B, ID_REG: CHIP_ID_RHD2164}
ROM_LIST_REGS = (40, 41, 42, 43, 44, 59, 60, 61, 62, 63)

N_RAM = 22                  # registers 0..21
B_ONLY_REGS = range(18, 22)  # amplifier power for module B: read back only on B
A_REGS = range(0, 18)


# ---------------------------------------------------------------------------
# MISO decode (CONTRACTS.md section 6)
# ---------------------------------------------------------------------------

def decode_taps(taps: int, phase: int) -> tuple[int, int]:
    """Module A and B words from one cycle's raw taps, exactly as the FPGA does.

    A bit (15 - j) = tap[p + 4j], B bit (15 - j) = tap[p + 2 + 4j]; tap k is
    the MISO level sampled on SPI tick k + 2.
    """
    if not 0 <= phase <= C.PHASE_MAX:
        raise ValueError(f"phase {phase} out of range 0..{C.PHASE_MAX}")
    a = b = 0
    for j in range(16):
        a |= ((taps >> (phase + 4 * j)) & 1) << (15 - j)
        b |= ((taps >> (phase + 2 + 4 * j)) & 1) << (15 - j)
    return a, b


def decode_partial(taps: int, phase: int, ntaps: int = C.CAP_TAPS):
    """decode_taps at any integer phase, with a mask of the bits that exist.

    A phase outside 0..PHASE_MAX would need taps before tap 0 or past the last
    one. Decoding what does exist is how the host tells "the right answer, but
    later than the window reaches" from "no answer at all".
    Returns (a, a_mask, b, b_mask).
    """
    a = am = b = bm = 0
    for j in range(16):
        bit = 1 << (15 - j)
        ka, kb = phase + 4 * j, phase + 2 + 4 * j
        if 0 <= ka < ntaps:
            am |= bit
            a |= bit if (taps >> ka) & 1 else 0
        if 0 <= kb < ntaps:
            bm |= bit
            b |= bit if (taps >> kb) & 1 else 0
    return a, am, b, bm


# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------

SWEEP_OVERHEAD = 16  # "a few clocks" between sweeps (CONTRACTS.md section 4), with margin


def sclk_hz(spi_div: int) -> float:
    return C.SYS_CLK_HZ / C.OVERSAMPLE / (spi_div + 1)


def tick_ns(spi_div: int) -> float:
    return 1e9 * (spi_div + 1) / C.SYS_CLK_HZ


def min_sample_period(spi_div: int = 0) -> int:
    """Fast clocks one 35-command sweep needs at this SPI_DIV."""
    return C.CMDS_PER_SAMPLE * C.CMD_TICKS * (spi_div + 1) + SWEEP_OVERHEAD


def max_rate(spi_div: int = 0) -> float:
    return C.SYS_CLK_HZ / min_sample_period(spi_div)


def burst_rate(spi_div: int = 0) -> float:
    """The per-channel rate Rhythm would need to issue commands this fast.

    The FPGA sends a sweep's 35 commands back to back and then idles, so inside
    a sweep the ADC converts at the SPI command rate however low the average
    sample rate is. The ADC buffer and MUX biases set how fast the ADC settles
    between conversions, so they must be chosen for this rate, not the average.
    """
    return C.SYS_CLK_HZ / (C.CMD_TICKS * (spi_div + 1) * C.CMDS_PER_SAMPLE)


# ---------------------------------------------------------------------------
# Register configuration (Rhythm Rhd2000Registers, RHD2000 datasheet)
# ---------------------------------------------------------------------------

def bias_for(rate: float) -> tuple[int, int]:
    """(muxBias, adcBufferBias) for a per-channel rate, Rhythm's table."""
    for limit, mux, buf in ((3334.0, 40, 32), (4001.0, 40, 16), (5001.0, 40, 8),
                            (6251.0, 32, 8), (8001.0, 26, 8), (10001.0, 18, 4),
                            (12501.0, 16, 3), (15001.0, 7, 3)):
        if rate < limit:
            return mux, buf
    return 4, 2


def _rh1(f: float) -> float:
    l = math.log10(f)
    return 0.9730 * 10 ** (8.0968 - 1.1892 * l + 0.04767 * l * l)


def _rh2(f: float) -> float:
    l = math.log10(f)
    return 1.0191 * 10 ** (8.1009 - 1.0821 * l + 0.03383 * l * l)


def _rl(f: float) -> float:
    l = math.log10(f)
    if f < 4.0:
        return 1.0061 * 10 ** (4.9391 - 1.2088 * l + 0.5698 * l * l + 0.1442 * l ** 3)
    return 1.0061 * 10 ** (4.7351 - 0.5916 * l + 0.08482 * l * l)


def _dac_search(target: float, base: float, u1: float, n1: int, u2: float, n2: int,
                actual: float = None):
    """Rhythm's two-DAC search: coarse DAC2 first, then fine DAC1."""
    actual = base if actual is None else actual
    d1 = d2 = 0
    for _ in range(n2):
        if actual < target - (u2 - u1 / 2):
            actual += u2
            d2 += 1
    for _ in range(n1):
        if actual < target - u1 / 2:
            actual += u1
            d1 += 1
    return d1, d2, actual


def _invert(fwd, r: float, lo: float, hi: float) -> float:
    """f such that fwd(f) == r, by bisection in log f (fwd falls with f)."""
    a, b = math.log10(lo), math.log10(hi)
    for _ in range(80):
        m = (a + b) / 2
        if fwd(10 ** m) > r:
            a = m
        else:
            b = m
    return 10 ** ((a + b) / 2)


def upper_dacs(f: float) -> dict:
    """RH1/RH2 DAC codes for an upper bandwidth, and the bandwidth they give."""
    f = min(f, 30000.0)  # the amplifiers do not work well above 30 kHz (Rhythm)
    h1d1, h1d2, r1 = _dac_search(_rh1(f), 2200.0, 600.0, 63, 29400.0, 31)
    h2d1, h2d2, r2 = _dac_search(_rh2(f), 8700.0, 763.0, 63, 38400.0, 31)
    actual = math.sqrt(_invert(_rh1, r1, 1.0, 1e6) * _invert(_rh2, r2, 1.0, 1e6))
    return {"rh1_dac1": h1d1, "rh1_dac2": h1d2, "rh2_dac1": h2d1, "rh2_dac2": h2d2,
            "actual_hz": actual}


def lower_dacs(f: float) -> dict:
    """RL DAC codes for a lower bandwidth, and the bandwidth they give."""
    f = min(f, 1500.0)  # the amplifiers do not work well above 1.5 kHz (Rhythm)
    d3 = 1 if f < 0.15 else 0
    d1, d2, r = _dac_search(_rl(f), 3500.0, 175.0, 127, 12700.0, 63,
                            actual=3500.0 + 3e6 * d3)
    return {"rl_dac1": d1, "rl_dac2": d2, "rl_dac3": d3,
            "actual_hz": _invert(_rl, r, 0.01, 1e4)}


# k(n) = ln(2^n / (2^n - 1)) / (2 pi): the datasheet's DSP cutoff table, so the
# cutoff is k(n) * fs. Kept as the datasheet prints it; a test checks the two
# agree.
DSP_K = (None, 0.1103, 0.04579, 0.02125, 0.01027, 0.005053, 0.002506, 0.001248,
         0.0006229, 0.0003112, 0.0001555, 0.00007773, 0.00003886, 0.00001943,
         0.000009714, 0.000004857)


def dsp_cutoff_code(f: float, rate: float) -> tuple[int, float]:
    """(n, cutoff Hz): the DSP high-pass setting nearest f on a log scale."""
    best = min(range(1, 16), key=lambda n: abs(math.log10(f) - math.log10(DSP_K[n] * rate)))
    return best, DSP_K[best] * rate


@dataclass
class AmpConfig:
    """Everything that decides registers 0..21.

    Defaults are for surface / HD-EMG at 2 kS/s per channel:
      lower 1 Hz: low enough to keep the EMG band flat, high enough that the
        amplifier recovers from a saturating artefact in seconds rather than
        the tens of seconds 0.1 Hz takes; motion artefacts are better removed
        later in software at 10-20 Hz, where the cutoff can be changed.
      upper 500 Hz: surface EMG has little energy above ~450 Hz, and 500 Hz is a
        quarter of the sample rate, leaving an octave before Nyquist.
      DSP high-pass ~1 Hz: removes amplifier offset drift, as Rhythm's default.
    """
    rate: float = 2000.0         # per-channel sample rate, S/s
    lower: float = 1.0           # amplifier lower bandwidth, Hz
    upper: float = 500.0         # amplifier upper bandwidth, Hz
    dsp: bool = True
    dsp_cutoff: float = 1.0      # Hz
    spi_div: int = 0             # sets the in-sweep conversion rate, hence the biases
    bias_rate: float | None = None  # override the rate the biases are chosen for
    twos_comp: bool = True       # CONVERT results signed, as the frame's int16 is
    amps_on: bool = True         # registers 14..21: all 64 amplifiers powered

    def bias_basis(self) -> float:
        return self.bias_rate if self.bias_rate is not None else burst_rate(self.spi_div)


def config_warnings(cfg: AmpConfig) -> list[str]:
    """Settings that work but probably are not what anyone wants. Never silent fixes."""
    out = []
    if cfg.upper > 0.45 * cfg.rate:
        out.append(f"upper bandwidth {cfg.upper:g} Hz is above 0.45 x the {cfg.rate:g} S/s "
                   f"sample rate ({0.45 * cfg.rate:g} Hz): signal above Nyquist will alias")
    if cfg.lower >= cfg.upper:
        out.append(f"lower bandwidth {cfg.lower:g} Hz is not below upper {cfg.upper:g} Hz")
    if cfg.upper > 30000:
        out.append(f"upper bandwidth {cfg.upper:g} Hz exceeds 30 kHz; programmed as 30 kHz")
    if cfg.lower > 1500:
        out.append(f"lower bandwidth {cfg.lower:g} Hz exceeds 1.5 kHz; programmed as 1.5 kHz")
    if cfg.dsp:
        n, fc = dsp_cutoff_code(cfg.dsp_cutoff, cfg.rate)
        if abs(math.log10(fc / cfg.dsp_cutoff)) > 0.2:
            out.append(f"DSP cutoff {cfg.dsp_cutoff:g} Hz is not reachable at {cfg.rate:g} S/s; "
                       f"nearest is {fc:.3g} Hz (n={n})")
    if cfg.rate > max_rate(cfg.spi_div):
        out.append(f"rate {cfg.rate:g} S/s exceeds the {max_rate(cfg.spi_div):.0f} S/s one sweep "
                   f"allows at SPI_DIV {cfg.spi_div}")
    return out


def register_values(cfg: AmpConfig) -> list[int]:
    """Registers 0..21, Rhythm's field layout, this file's choices of value."""
    mux_bias, adc_buf = bias_for(cfg.bias_basis())
    up = upper_dacs(cfg.upper)
    lo = lower_dacs(cfg.lower)
    dsp_n, _fc = dsp_cutoff_code(cfg.dsp_cutoff, cfg.rate)
    amps = 0xFF if cfg.amps_on else 0x00
    r = [0] * N_RAM
    # 0: adcReferenceBw 3, ampFastSettle 0, ampVrefEnable 1, adcComparatorBias 3,
    #    adcComparatorSelect 2.
    r[0] = (3 << 6) | (0 << 5) | (1 << 4) | (3 << 2) | 2
    r[1] = (1 << 6) | adc_buf        # vddSenseEnable 1: supply sensor on (aux channel)
    r[2] = mux_bias
    # 3: muxLoad 0, tempS2/S1/En 0, digOutHiZ 1, digOut 0.
    r[3] = (0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) | (1 << 1) | 0
    # 4: weakMiso 1 (MISO held, not floating, while CS is high), twosComp, absMode
    #    0, DSPen, dspCutoffFreq.
    r[4] = (1 << 7) | (int(cfg.twos_comp) << 6) | (0 << 5) | (int(cfg.dsp) << 4) | dsp_n
    r[5] = 0                          # impedance-check DAC off: never used here
    r[6] = 0x80                       # zcheck DAC midscale
    r[7] = 0                          # zcheck channel 0
    r[8] = up["rh1_dac1"]             # offChipRH1 0
    r[9] = (1 << 7) | up["rh1_dac2"]  # adcAux1En 1
    r[10] = up["rh2_dac1"]            # offChipRH2 0
    r[11] = (1 << 7) | up["rh2_dac2"]  # adcAux2En 1
    r[12] = lo["rl_dac1"]             # offChipRL 0
    r[13] = (1 << 7) | (lo["rl_dac3"] << 6) | lo["rl_dac2"]  # adcAux3En 1
    for k in range(14, 22):
        r[k] = amps
    return r


def config_summary(cfg: AmpConfig) -> dict:
    """The settings as the chip will actually have them, for reports."""
    up = upper_dacs(cfg.upper)
    lo = lower_dacs(cfg.lower)
    n, fc = dsp_cutoff_code(cfg.dsp_cutoff, cfg.rate)
    mux, buf = bias_for(cfg.bias_basis())
    return {
        "rate": cfg.rate, "spi_div": cfg.spi_div,
        "upper_requested_hz": cfg.upper, "upper_actual_hz": round(up["actual_hz"], 1),
        "lower_requested_hz": cfg.lower, "lower_actual_hz": round(lo["actual_hz"], 4),
        "dsp": cfg.dsp, "dsp_n": n, "dsp_cutoff_hz": round(fc, 4),
        "bias_rate": round(cfg.bias_basis(), 1), "mux_bias": mux, "adc_buffer_bias": buf,
        "twos_comp": cfg.twos_comp,
        "registers": [f"0x{v:02x}" for v in register_values(cfg)],
        "warnings": config_warnings(cfg),
    }


# ---------------------------------------------------------------------------
# Command sequences. Each ends with the pipeline flushers when its results are
# wanted, so the last real command's answer is captured inside the same list.
# ---------------------------------------------------------------------------

def rom_commands() -> list[int]:
    """READ 40..44, 59..63, then two flushers."""
    return [read(r) for r in ROM_LIST_REGS] + FLUSHERS


def rom_expectations() -> dict[str, dict[int, int]]:
    """{'A': {command index: word}, 'B': {...}} for rom_commands()."""
    out = {"A": {}, "B": {}}
    for i, r in enumerate(ROM_LIST_REGS):
        if r in ROM_A:
            out["A"][i] = ROM_A[r]
        if r in ROM_B:
            out["B"][i] = ROM_B[r]
    return out


def init_commands(cfg: AmpConfig) -> list[int]:
    """WRITE 0..21 from cfg."""
    return [write(r, v) for r, v in enumerate(register_values(cfg))]


def verify_commands() -> list[int]:
    """READ 0..21, then two flushers."""
    return [read(r) for r in range(N_RAM)] + FLUSHERS


def configure_commands(cfg: AmpConfig) -> list[int]:
    """WRITE 0..21, READ 0..21, flushers: echoes and read-back in one 46-command list."""
    return init_commands(cfg) + verify_commands()


def calibrate_commands(resume_checks: int = 2) -> list[int]:
    """CALIBRATE, its 9 dummies, then READ(63)s that must answer normally again.

    The dummies are READ(63) too. Calibration gives no readback of its own; the
    commands after the window are the only evidence the chip came out of it.
    """
    return [CALIBRATE] + [FLUSH] * CALIBRATE_DUMMIES + [FLUSH] * resume_checks + FLUSHERS


def convert_sweep() -> list[int]:
    """CONVERT 0..31, then two flushers: one snapshot of all 64 amplifiers."""
    return [convert(c) for c in range(32)] + FLUSHERS


def global_channel(chip: int, module: str, c: int) -> int:
    """Frame channel number (CONTRACTS.md section 1) of amplifier c of one module."""
    return 64 * chip + (32 if module == "B" else 0) + c
