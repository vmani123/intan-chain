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
| `02` | RW | `PHASE_SELECT` | 0–15. MISO sampling phase, in SPI ticks (quarter-SCLK steps); larger writes saturate at 15 |
| `03` | RW | `MODE` | bit0 pattern REALISTIC, bit1 codec enable, bit2 use internal pattern instead of SPI, bit3 SPI loopback to the on-die emulator (headstage build only; see `CAPS`) |
| `04` | RW | `SPI_DIV` | 0–15. One SPI tick = `SPI_DIV + 1` fast clocks, so SCLK = 24 MHz / (`SPI_DIV` + 1). Phase steps stretch by the same factor. Larger writes saturate at 15 |
| `10` | R  | `FRAMES` | frames built since the last clear |
| `11` | R  | `DGRAMS` | datagrams handed to QSPI |
| `12` | R  | `FIFO_HIGH` | high-water mark, in datagram slots |
| `13` | R  | `OVERRUNS` | datagrams dropped because the FIFO was full |
| `14` | R  | `STALLS` | fast-clock cycles spent waiting on the ESP ready line |
| `15` | R  | `SPI_CYCLES` | CS cycles issued since the last clear, sweeps and diagnostic lists alike. Climbing proves the SPI master runs |
| `16` | R  | `LINK_CHECKS` | sweeps whose AUX `READ(59)` result was checked (section 6) |
| `17` | R  | `LINK_ERR0` | chip 0 link check failures: [15:0] module A, [31:16] module B. Each half saturates at `ffff` |
| `18` | R  | `LINK_ERR1` | chip 1, same layout |
| `19` | R  | `LINK_LAST0` | chip 0's most recent `READ(59)` result: [15:0] module A (expect `0035`), [31:16] module B (expect `003a`) |
| `1A` | R  | `LINK_LAST1` | chip 1, same layout |
| `1B` | R  | `MISO_STAT` | bit0 / bit1: miso0 seen low / seen high while CS was low (sticky until clear); bit2 / bit3: the same for miso1; bit4 / bit5: miso0 / miso1 level right now; bit8: loopback active |
| `1C` | R  | `MISO_EDGES0` | miso0 transitions seen while CS was low, since the last clear |
| `1D` | R  | `MISO_EDGES1` | miso1, same |
| `1E` | R  | `CAPS` | build facts: [7:0] `QSPI_DIV`, bit8 on-die emulator present (loopback works), [23:16] capture taps per CS cycle (78) |
| `1F` | R  | `ID` | `0x1C10_0002` — magic + version, proves which bitstream is loaded |
| `20` | RW | `DIAG_PTR` | 0–63. Index into the diagnostic command list and its results |
| `21` | RW | `DIAG_CMD` | write: command list entry `DIAG_PTR` = value[15:0], then `DIAG_PTR` += 1. Read: entry `DIAG_PTR`, no side effect |
| `22` | RW | `DIAG_LEN` | 1–64. Commands in the list. 0 reads back as 1; larger writes saturate at 64 |
| `23` | RW | `DIAG_CTRL` | write bit0 `go` (self-clearing). Read: bit0 busy, bit1 done (since the last `go`), bit2 rejected (the last `go` arrived while `run` was set; nothing was sent) |
| `24` | R  | `DIAG_RES0` | cycle `DIAG_PTR`, chip 0, decoded at the `PHASE_SELECT` in force during it: [15:0] module A, [31:16] module B |
| `25` | R  | `DIAG_RES1` | chip 1, same layout |
| `26` | R  | `DIAG_RAW0_0` | cycle `DIAG_PTR`, raw miso0 taps 0–31 (bit k = tap k) |
| `27` | R  | `DIAG_RAW0_1` | taps 32–63 |
| `28` | R  | `DIAG_RAW0_2` | taps 64–77 in [13:0]; [31:14] read 0 |
| `29` | R  | `DIAG_RAW1_0` | miso1, taps 0–31 |
| `2A` | R  | `DIAG_RAW1_1` | taps 32–63 |
| `2B` | R  | `DIAG_RAW1_2` | taps 64–77 in [13:0] |

Writes to read-only or unknown addresses answer `err`, as do reads of unknown
addresses.

`SAMPLE_PERIOD` must be at least the time one sweep takes:
`35 cycles × 84 ticks × (SPI_DIV + 1)` fast clocks, plus a few clocks of
overhead (2940 at `SPI_DIV` = 0, about 32.6 kS/s). Writing less makes the
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
| `CMD_TICKS` | 84 | SPI ticks per CS cycle: 65 with CS low, 19 high |
| `CAP_TAPS` | 78 | MISO taps recorded per CS cycle; supports `PHASE_SELECT` 0–15 |
| `DIAG_DEPTH` | 64 | diagnostic list entries |
| `UART_BAUD` | 115200 | |
| `UDP_PORT` | 5001 | data |
| `ECHO_PORT` | 5002 | round-trip clock offset probe |

---

## 6. SPI cycle, AUX slots and the diagnostic list

### One CS cycle

One command is one CS cycle of 84 SPI ticks (`OVERSAMPLE` = 4 ticks per SCLK
bit; one tick is `SPI_DIV + 1` fast clocks):

| Ticks | Bus |
|---|---|
| 0–64 | CS low. Bit b of the command (MSB first) occupies ticks 4b..4b+3: MOSI changes on tick 4b, SCLK is high on ticks 4b+2 and 4b+3 |
| 65–83 | CS high (19 ticks = 198 ns at `SPI_DIV` = 0, over the 154 ns minimum) |

MISO is recorded on every tick from 2 to 79: **tap k is the MISO level sampled
on tick k + 2**, 78 taps per cycle. Decoding at phase p, exactly as
`miso_phase_select.sv` does:

```
module A bit (15 - j) = tap[p + 4*j]          j = 0..15   (falling-edge half)
module B bit (15 - j) = tap[p + 2 + 4*j]                  (rising-edge half; LSB after CS rises)
```

so p = 0..15 is the full range the window supports.

### The two-cycle pipeline

The RHD2164 returns a command's result two CS cycles later. The FPGA never
re-aligns diagnostic results: **cycle i carries the result of the command sent
in cycle i − 2**. A list that wants every result back ends with two harmless
commands (`READ(63)` is the convention).

### The sweep

One sample is 35 cycles: `CONVERT(0)` .. `CONVERT(31)` in cycles 0–31, then
three AUX commands, fixed in this build:

| Cycle | Command | Result arrives |
|---|---|---|
| 32 | `READ(59)` — MISO A/B marker | cycle 34: module A `0x0035`, module B `0x003A`, on both chips |
| 33 | `READ(40)` — `'I'` | cycle 0 of the next sweep (unchecked) |
| 34 | `READ(63)` — chip ID, `4` | cycle 1 of the next sweep (unchecked) |

In cycle 34 of every sweep the FPGA compares the four 16-bit words (chip 0 A
and B, chip 1 A and B) with `0035` / `003a` / `0035` / `003a`. Each mismatch
increments that lane's half of `LINK_ERR0` / `LINK_ERR1`; `LINK_CHECKS`
increments once per sweep; `LINK_LAST0` / `LINK_LAST1` hold the words read.
A healthy link therefore shows `LINK_CHECKS` climbing and `LINK_ERR*` at zero
for as long as it runs, with no reference data and no ESP. The marker differs
between modules on purpose: a phase that swaps or shifts the two halves cannot
pass. LED LD5 is lit while the most recent check passed on all four lanes.

### The diagnostic list

With `run` clear, `DIAG_CTRL` `go` sends commands `0 .. DIAG_LEN-1` of the
list back to back, one CS cycle each, with the same timing as a sweep, and
records each cycle's decoded words (`DIAG_RES*`) and raw taps (`DIAG_RAW*`).
Nothing reaches the frame builder. A `go` while `run` is set is refused and
sets `DIAG_CTRL` bit2. Results stay readable until the next `go`.

RHD2164 command words (datasheet; RHD2000 series register map):

| Command | Word | Result two cycles later |
|---|---|---|
| `CONVERT(c)` | `00cccccc 0000000h` | ADC value: module A channel c, module B channel c + 32 |
| `READ(r)` | `11rrrrrr 00000000` | `0x00` then the register byte |
| `WRITE(r, d)` | `10rrrrrr dddddddd` | `0xFF` then d |
| `CALIBRATE` | `0x5500` | the next 9 commands are consumed by calibration |
| `CLEAR` | `0x6A00` | clears calibration; never needed in normal use |

ROM registers, read on module A: 40–44 `"INTAN"`, 59 = 53 (module B: 58),
60 die revision, 61 = 1 (unipolar), 62 = 64 amplifiers, 63 = 4 (RHD2164).
Registers 18–21 read back correctly only on module B.
