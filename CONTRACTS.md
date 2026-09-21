# CONTRACTS

Everything in this repo agrees on the definitions below. Three independent
implementations — FPGA RTL, ESP firmware, host Python — have to produce and
parse the same bytes, and drift between them is the most common way a chain
like this produces plausible-looking garbage.

So two of the three are **generated from this file**, by `tools/gen_contract.py`:

- `esp/main/chain_contract.h`
- `host/chain/contract.py`

Do not hand-edit either. Change this file, run `make contract`, commit all three.
`make check-contract` fails if the generated files are stale, and CI runs it.

All multi-byte fields are **little-endian**.

---

## 1. Frame

One frame is one time sample of every channel.

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0  | 2 | `magic`       | `0xA55A` |
| 2  | 1 | `type`        | 0 = RAW16, 2 = COMPRESSED |
| 3  | 1 | `flags`       | bit0 = pattern REALISTIC (else CHECK), bit1 = codec enabled |
| 4  | 4 | `seq`         | frame counter, starts at 0, wraps at 2^32. Loss and ordering |
| 8  | 4 | `t_cap`       | FPGA fast-clock tick when the sample finished capture |
| 12 | 4 | `t_tx`        | FPGA fast-clock tick when the frame entered the QSPI transfer |
| 16 | 2 | `n_ch`        | channels in the payload |
| 18 | 2 | `payload_len` | payload bytes that follow the header |
| 20 | n | payload       | `n_ch` × int16, channel order below (RAW16) |

Header is 20 bytes. At 128 channels a RAW16 frame is 20 + 256 = **276 bytes**.

`t_cap` and `t_tx` are the same free-running counter, so `t_tx - t_cap` is the
time the frame spent inside the FPGA, exactly, with no clock-domain guesswork.
At 96 MHz one tick is 10.417 ns and the counter wraps every 44.7 s.

### Channel order

Ascending: chip 0 module A channels 0–31, chip 0 module B 32–63, chip 1 module A
64–95, chip 1 module B 96–127. Module A is the rising-edge half of the RHD2164's
double-data-rate MISO, module B the falling-edge half.

---

## 2. Datagram

One QSPI transfer carries one finished UDP payload. The FPGA builds the whole
thing; the ESP appends a 12-byte trailer and hands it to the socket. The ESP
never reframes, never parses a frame, never allocates.

```
[ frame 0 ][ frame 1 ][ frame 2 ][ frame 3 ][ frame 4 ][ trailer ]
```

`FRAMES_PER_DGRAM = 5`. At 128 channels: 5 × 276 + 12 = **1392 bytes**, inside a
1472-byte MTU payload, so no IP fragmentation.

### Trailer (12 bytes, written by the ESP)

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 4 | `esp_rx_us`  | `esp_timer_get_time()` when the QSPI transfer completed |
| 4 | 4 | `esp_tx_us`  | `esp_timer_get_time()` when handed to the socket |
| 8 | 4 | `pipe_depth` | bytes queued in the ESP pipe at that moment |

Segments are **fixed size always**. When the codec is on, the compressed blob is
padded to the same length and `payload_len` says how much is real. Variable-length
DMA transfers are not worth the class of bug they introduce.

---

## 3. Test pattern

The FPGA generates it, the host regenerates it, and they must agree bit for bit.
It is closed-form on both sides, so accuracy checking needs no data files.

Both modes use the same 16-bit Fibonacci LFSR, `x^16 + x^14 + x^13 + x^11`:

```
next(s) = (s >> 1) ^ (0xB400 if (s & 1) else 0)
```

Each channel has its own state, seeded at reset with `seed(ch) = ch + 1`
(never 0 — an all-zero LFSR is stuck). Every channel advances exactly once per
sample, so state depends only on `(ch, sample_index)`.

**Alignment — emit, then advance.** Sample index `i` reports the state reached
after `i` advances from the seed, so **sample 0 reports the seed itself** and the
advance happens after the value is emitted. In RTL that is the natural shape: the
state register is read combinationally and clocked on the sample strobe.

This is the one detail three implementations can each get right on their own and
still disagree about, and a one-step offset looks like total data corruption
rather than like an off-by-one. It is pinned here on purpose. The check value:
channel 0, sample 0 reads `1`; sample 1 reads `0xB400`; sample 2 reads `0x5A00`.

**CHECK** (`flags` bit0 = 0) — the sample value *is* the LFSR state, as a signed
16-bit number. Maximally sensitive to single-bit errors, and worthless for the
codec, which is the point: use it to prove correctness.

**REALISTIC** (`flags` bit0 = 1) — smooth, so the compressor sees a plausible
ratio:

```
phase    = (sample_index + 8 * ch) mod 256
triangle = (phase < 128) ? (phase * 128 - 8192) : ((255 - phase) * 128 - 8192)
noise    = (lfsr_state >> 12) - 8            # -8 .. +7
value    = triangle + noise                  # signed 16-bit
```

---

## 4. UART control interface

115200 baud, 8N1, no flow control, on the Arty's onboard USB-UART.
**Text, not binary** — so the board can be driven by hand from `screen` before
any host tool exists.

Host sends one line, terminated `\n`. Board replies one line, terminated `\n`.

```
R aa        -> "aa=vvvvvvvv"      read register aa (2 hex digits)
W aa vvvvvvvv -> "ok"             write register aa (8 hex digits)
anything else -> "err"
```

Lowercase hex both ways. The board never sends anything unsolicited.

| Addr | R/W | Register | Meaning |
|---|---|---|---|
| `00` | RW | `CTRL` | bit0 `run`, bit1 `clear_counters` (self-clearing) |
| `01` | RW | `SAMPLE_PERIOD` | fast-clock ticks between sample starts. Sets the rate |
| `02` | RW | `PHASE_SELECT` | 0–11. MISO sampling phase, in quarter-SCLK steps |
| `03` | RW | `MODE` | bit0 pattern REALISTIC, bit1 codec enable, bit2 use internal pattern instead of SPI |
| `10` | R  | `FRAMES` | frames built since the last clear |
| `11` | R  | `DGRAMS` | datagrams handed to QSPI |
| `12` | R  | `FIFO_HIGH` | high-water mark, in datagram slots |
| `13` | R  | `OVERRUNS` | datagrams dropped because the FIFO was full |
| `14` | R  | `STALLS` | fast-clock cycles spent waiting on the ESP ready line |
| `1F` | R  | `ID` | `0x1C10_0001` — magic + version, proves which bitstream is loaded |

`SAMPLE_PERIOD` must be at least the time one command sequence takes:
`35 commands × 16 SCLK bits × OVERSAMPLE + overhead`. Writing less makes the
sequencer free-run and `OVERRUNS` climb; the host treats that as
source-limited rather than as a radio ceiling.

---

## 5. Fixed numbers

| Name | Value | Where it comes from |
|---|---|---|
| `N_CH` | 128 | 2 × RHD2164 |
| `FRAMES_PER_DGRAM` | 5 | largest count that keeps a datagram under one MTU |
| `SYS_CLK_HZ` | 96_000_000 | 100 MHz osc → MMCM; /4 gives SCLK = 24 MHz, the RHD2164 max |
| `OVERSAMPLE` | 4 | fast-clock ticks per SCLK bit, as in Intan's Rhythm |
| `CMDS_PER_SAMPLE` | 35 | 32 CONVERT + 3 AUX |
| `UART_BAUD` | 115200 | |
| `UDP_PORT` | 5001 | data |
| `ECHO_PORT` | 5002 | round-trip clock offset probe |
