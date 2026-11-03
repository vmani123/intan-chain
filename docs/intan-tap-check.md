# Checking every sample against the Intan RHD recording controller

The chain's own numbers (`seq` gaps, `LINK_ERR`, the pattern check) prove the
link carries bits faithfully. They cannot prove that the values the Mac shows
are the values the RHD2164s produced. This check does: the Intan RHD recording
controller records the headstage as usual, the chain hears the same SPI
traffic, and the Mac compares the two recordings sample by sample.

```
                      cable A                 cable B
 Intan RHD recording ========= [C3430 X]--[C3430 Y] ========= headstage
 controller (master)               pass-through |              (2 x RHD2164)
       | USB                                    | 5 LVDS pairs + GND, listen only
       v                                        v
     RHX  --> amplifier.dat          Arty S7-25 (tap build) --QSPI--> ESP32-C5 --Wi-Fi--> Mac
                         \                                                               /
                          `------------------ python -m chain.tapcheck compare ---------'
```

An RHD2164 has one SPI master, so the chain cannot run its own sweeps at the
same time. In this check the controller is the master and the Arty only
listens. What is checked, end to end, is everything from the SPI wires onward:
capture, framing, FIFO, QSPI, the ESP, Wi-Fi and the host parser. The chain's
own SPI master is not in the loop here; the bring-up script, `LINK_ERR` and the
emulator loopback cover it.

## Hardware

- Intan RHD recording controller (or RHD USB interface board) with RHX
- the headstage (2 x RHD2164 = 128 channels on one port)
- **two** Intan RHD SPI cables (C3203 / C3206 / C3211 / ...)
- **two** Intan SPI cable adapter boards, **C3430** (12 signals to 0.1" holes,
  no active parts)
- the Arty S7-25 with the tap bitstream, the ESP32-C5 on Pmod JC as before
- short wires; twisted pairs if longer than ~10 cm

The tap is the pass-through Intan describes for commutators (application note
"Adapting SPI Cables to a Commutator"), plus five pairs and a ground to the
Arty. The Arty's inputs are LVDS receivers with internal termination **off**
(the cable is already terminated: CS, SCLK and MOSI on the headstage, both MISO
lines inside the controller's FPGA), so the tap only loads the lines; it never
drives them.

## Wiring

### 1. Pass-through between the two C3430 boards

`X` is the C3430 on the cable from the **controller**, `Y` the C3430 on the
cable from the **headstage**. Every `T` hole on one board goes to the `B` hole
with the same number on the other (the cable flips top and bottom):

| X | Y | Signal |
|---|---|---|
| T1 | B1 | CS+ |
| B1 | T1 | CS− |
| T2 | B2 | SCLK+ |
| B2 | T2 | SCLK− |
| T3 | B3 | MOSI+ |
| B3 | T3 | MOSI− |
| T4 | B4 | MISO1+ |
| B4 | T4 | MISO1− |
| T5 | B5 | MISO2+ |
| B5 | T5 | MISO2− |
| T6 | B6 | power (3.5 V from the controller) |
| B6 | T6 | ground |

Keep each pair together (twist + with −). With this in place and nothing else
connected, RHX must record the headstage exactly as with one cable; check that
before adding the tap.

### 2. Tap from C3430 Y to the Arty

| C3430 Y hole | Signal | Arty pin | FPGA pin |
|---|---|---|---|
| B1 | CS+    | JA1  | L17 |
| T1 | CS−    | JA2  | L18 |
| B2 | SCLK+  | JA3  | M14 |
| T2 | SCLK−  | JA4  | N14 |
| B3 | MOSI+  | JA7  | M16 |
| T3 | MOSI−  | JA8  | M17 |
| B4 | MISO1+ | JA9  | M18 |
| T4 | MISO1− | JA10 | N18 |
| B5 | MISO2+ | JB1  | P17 |
| T5 | MISO2− | JB2  | P18 |
| T6 | ground | JA5 (or JA11) | GND |

- **Never connect Y B6 (cable power) or any Pmod 3V3 pin (pin 6 / 12).** Only
  ground is shared.
- Pairs land on the Arty's differential pairs (JA1/2, JA3/4, JA7/8, JA9/10,
  JB1/2). A swapped + and − inverts that line; `tapcheck status` names which.
- The LVDS converter board (C3490) and its JA wiring are not used in this
  setup; unplug it.
- Optional: a 220–470 Ω resistor in each tap wire, right at the C3430 holes,
  isolates the stub. At 2 kS/s short wires work without them.

MISO1 is chip 0 (channels 0–63), MISO2 chip 1 (64–127), the same split the
chain and RHX use.

### 3. Building it on the bench

Everything powered off while you solder and wire: the controller, the Arty,
the ESP.

1. **Label the boards.** Write X on the C3430 that will take the cable from
   the controller and Y on the one that will take the cable from the
   headstage. The + / − names in both tables depend on which is which; mixing
   them up inverts every pair.
2. **Mount them.** Tape or hot-glue both C3430s to a scrap of perfboard,
   connectors facing outward, holes about 2–3 cm apart. The Omnetics nano
   connectors are fragile: the cables should pull on the perfboard, never on
   the connector.
3. **Pass-through (table 1).** Twelve short wires, X `T`n to Y `B`n and X `B`n
   to Y `T`n. Twist each signal pair (1–5) together. Keep power (6) away from
   the signal pairs.
4. **Tap wires (table 2).** Solder a second wire into each Y hole next to the
   pass-through wire (or onto it, right at the hole). Twist each + / − pair
   and keep them short, ideally under 10 cm to the Arty. If you use the
   optional 220–470 Ω resistors, put them at the Y end. End the wires on a 2x6
   header plug, or on jumper leads, so the tap comes off the Arty in one
   piece.

   **Jumper-lead variant.** No splitter chip is needed: the split is just
   the point where three wires meet (X, Y, Arty). Each line has one driver
   and one terminating resistor, and the Arty only listens, so a parallel
   wire overhears it the way a scope probe does. To make the tap pluggable,
   solder 0.1" header pins into the eleven Y holes you need (all but B6), and
   solder the pass-through wires to the same pads. Then run female-to-male
   jumpers from the Y pins to JA/JB, pair by pair, and twist them. Keep three
   things soldered:
   - **The X ↔ Y pass-through.** The Intan recording you compare against
     travels over it. A loose jumper there corrupts the reference, and the
     compare blames the chain for it.
   - **The Omnetics end.** Jumpers cannot reach the cable's own connector
     (0.025" pitch), so the two C3430s are the minimum hardware.
   - **No breadboard.** The T and B holes with the same number sit side by
     side, so a header pushed into a breadboard shorts + to − in one strip.
5. **Find the Pmod pins.** Pin 1 is marked on the Arty's silkscreen. Pins
   1–4 / 7–10 are signals, 5 / 11 GND, 6 / 12 3.3 V. Each tap pair lands on
   two neighbouring pins in one row: JA1/2, JA3/4, JA7/8, JA9/10, JB1/2.
   Unplug the C3490 converter and its JA wires. The ESP on JC stays.
6. **Meter it before any power.**
   - Continuity X → Y on all twelve pass-through wires, and from each Y hole
     to its Pmod pin.
   - No short between the two wires of any pair, between neighbouring pairs,
     or between power (X T6 / Y B6) and ground.
   - Nothing connected to Pmod pins 6 or 12, and nothing from the cable's
     power reaching the Arty.
7. **Pass-through alone first.** Leave the tap unplugged from the Arty.
   Connect controller -> cable A -> X, Y -> cable B -> headstage. Power the
   controller, run RHX: the waveforms must look exactly as with one cable
   (no corrupt-data spikes, the impedance test works). If they don't, fix the
   pass-through before going further.
8. **Power order with the tap.** Tap unplugged: power the Arty, load the
   tap bitstream, and check `tapcheck status` reports the tap (CAPS bit9).
   Stop RHX and power the controller off. Plug the tap into JA/JB, ground
   first. Then power the controller and start RHX. An unpowered FPGA clamps
   its inputs, which loads the lines the headstage is using; a configured
   tap build only listens.
   - **Only the tap build may be loaded while the tap is plugged in.** The
     headstage, chain and emulator builds drive JA1–JA3 as 3.3 V outputs,
     straight into the cable's CS and SCLK pairs against the controller's
     drivers. Unplug the tap before loading any other build.
   - **After every Arty power cycle, unplug the tap first.** The bitstream is
     volatile, and on power-up the Arty boots whatever is in its flash.

Two Intan cables plus the pass-through stay well inside Intan's length limit
for a 128-channel headstage (21 ft of standard cable, 12 ft of ultra thin).

## Procedure

Everything runs from `host/` with the venv Python, because `compare` needs
numpy (`host/requirements.txt`). The serial port is the Arty's **second** FTDI
port, the one ending in `1` (the first is JTAG and never answers):

```bash
cd host
PY=~/.venvs/intan-chain/bin/python
PORT=/dev/cu.usbserial-210352BF97961
```

The order matters: each step proves something the next one relies on.

| # | Step | Proves |
|---|---|---|
| 1 | load the tap bitstream | the FPGA has the tap (CAPS bit9) |
| 2 | `tapcheck selftest` | tap decode, framing, QSPI, ESP, Wi-Fi, host, with no Intan hardware |
| 3 | pass-through only, RHX records | the C3430 pass-through is right before anything else is attached |
| 4 | wire the tap, RHX running, `tapcheck status` | every pair arrives, the right way round |
| 5 | `tapcheck delay-sweep` | a TAP_DELAY that reads both chips |
| 6 | `tapcheck record`, then RHX records, then both stop | the two recordings of one session |
| 7 | `tapcheck compare` | every sample, against RHX |

### 1. Load the tap bitstream

```bash
QSPI_DIV=6 make bit-tap          # on the Vivado machine -> fpga/bitstreams/tap_arty_s7_25_div6.bit
openFPGALoader -b arty_s7_25 fpga/bitstreams/tap_arty_s7_25_div6.bit
```

The ESP stays on Pmod JC with the QSPI firmware (`-DBENCH_INGRESS=qspi`) and
joins Wi-Fi as for any chain run. The bitstream is volatile: reload after every
power cycle.

### 2. Self-test, nothing plugged into JA/JB

```bash
$PY -m chain.tapcheck selftest --serial $PORT
```

MODE bit3|bit4 points the tap at the bus between the receiver's own SPI master
and the on-die emulator. The master runs at `SPI_DIV` 15 (SCLK 1.5 MHz, close to
Rhythm's 1.4 MHz at 2 kS/s, because the tap's sampling window is half an SCLK
period wide) and 2000 S/s, for 3 s. Every check must say PASS:

- tap selected and running (TAP_STAT bit16), frames built, datagrams received;
- FLAG_TAP on every frame, no TAP_PARTIAL, no framing errors, seq contiguous;
- one frame per sweep (FRAMES = TAP_SWEEPS) and 35 CS cycles per sweep;
- known answers: the master's READ(59), READ(40), READ(63) every sweep, TAP_ERR 0;
- values: every word equals the emulator's playback memory
  (`fpga/upstream/rhd2164-emulator-mem/`) at one constant offset, the same
  golden reference `sweep --source emulator` uses.

MODE, SPI_DIV, SAMPLE_PERIOD and TAP_DELAY are put back afterwards. A report
lands in `docs/results/tapcheck_selftest_<time>.json`. If this fails, the
problem is the FPGA, the ESP or the network, not the Intan wiring: fix it first.

### 3. RHX settings, and the pass-through alone

With only the C3430 pass-through between the two cables (section 1 of the
wiring), RHX must record the headstage exactly as with one cable. Use these
settings for the check:

- **Sample rate 2.00 kS/s.** The tap's window is widest at a slow SCLK; at
  5 kS/s module B's last bit can be gone by the time the tap samples it.
- **All 128 channels of the port enabled** (A-000 .. A-127; the headstage on
  MISO1 is A-000 .. A-063). The comparison checks only channels RHX saved.
- **Notch filter: Disabled.** Since RHX 3.0 the saved amplifier data is
  notch-filtered (Intan's own loader says so and does not re-apply it), so with
  the notch on the file no longer holds the codes on the wire.
- **No software re-referencing** (the header's reference reads `n/a` or
  `Hardware`).
- **File format: One File Per Signal Type** (a folder with `info.rhd`,
  `time.dat`, `amplifier.dat`). Traditional Intan File Format (`.rhd`) works
  too. One File Per Channel is not read. If "start a new file every N minutes"
  is on, pass every file or folder of the session to `compare`.
- Bandwidth, DSP high-pass, amplifier settle: anything. They act inside the
  chip, before the tap and RHX see the same words.
- Do not run an impedance test during the recording.

### 4. Wire the tap, then `status` with RHX running

Wire section 2 of the wiring with the controller **off**. Then power up, start
RHX and press Run (the controller clocks the headstage only while RHX runs):

```bash
$PY -m chain.tapcheck status --serial $PORT
```

`status` selects the tap, clears the counters, listens for one second
(`--listen S`), puts MODE and run back, and explains what it heard:

- **lines**: which of CS, SCLK, MOSI, MISO1, MISO2 toggled. A line that never
  toggles while the others do is not wired (or, for a MISO, its chip is not
  driving). With RHX stopped, `cs_n` must idle at 1 and `sclk` at 0; anything
  else is a swapped (+/-) pair.
- **cycles, sweeps**: TAP_CYCLES and TAP_SWEEPS per second. Sweeps per second
  is the controller's sample rate (2000).
- **framing**: TAP_FRAMING, CS cycles without exactly 16 + 16 SCLK edges. Must
  be 0.
- **checks**: TAP_CHECKS and TAP_ERR per lane. TAP_CHECKS climbs only if RHX
  sends commands with a fixed answer (register writes, ROM reads) while it
  streams; if it stays 0 the readout cannot judge TAP_DELAY and `compare` is
  the real check.
- **SCLK**: TAP_SCLK_MIN is the shortest SCLK half period heard, in 10.4 ns
  clocks: 34 is 354 ns, SCLK about 1.41 MHz, which under Rhythm's timing (SCLK
  about 700 x the sample rate) is about 2 kS/s. From it, the usable TAP_DELAY:
  from the round trip (a few clocks) to half an SCLK period after it, preferably
  the first half (about 3..20 at 2 kS/s). The reset value 8 is inside.

Exit status 0 means nothing failed. `--listen 0` reads the registers as they
are without touching MODE or run; the tap decodes its lines whatever MODE says,
but TAP_SWEEPS and TAP_PARTIAL move only while the tap is selected and running.

### 5. Delay sweep, RHX still running

```bash
$PY -m chain.tapcheck delay-sweep --serial $PORT            # look
$PY -m chain.tapcheck delay-sweep --serial $PORT --apply    # and keep the choice
```

Steps TAP_DELAY 0..63 (`--from`, `--to`, `--step`), clears the counters at
each step after it settles, waits `--dwell` (0.5 s) and prints one row per
delay: cycles, framing, sweeps, partial, checks and the four lanes' errors. A
delay is clean when TAP_CHECKS climbed and nothing failed; the choice is the
centre of the widest clean run. Without `--apply` the original TAP_DELAY is put
back; either way `record --delay N` sets it explicitly. If TAP_CHECKS never
climbs, the sweep says so: keep the reset value (or the middle of the range
`status` printed) and let `compare` decide, recording at a few delays if
needed. The table goes to `docs/results/tapcheck_delay_<time>.json`.

### 6. Record

Start the chain recording first, then the RHX recording; stop RHX first, then
the chain. That way the RHX window lies inside the chain's and every RHX
sample can be checked:

```bash
$PY -m chain.tapcheck record --serial $PORT --delay 8      # Ctrl-C to stop
```

1. `tapcheck record` sets MODE to the tap only (bit4), sets TAP_DELAY if
   `--delay` is given, clears the counters, sets run, and writes every datagram
   that reaches UDP 5001 to `~/intan-tap-recordings/tap_<time>.chaintap`
   (`--out DIR` to change; outside the repo on purpose, about 0.55 MB/s at
   2 kS/s, 33 MB a minute). It prints a progress line every 2 s.
2. In RHX, press Record. Record at least 30 s.
3. In RHX, press Stop.
4. Ctrl-C `tapcheck record` (or give `--seconds S`). It stops the run, keeps
   listening 0.5 s for datagrams still in flight, and writes the counters
   (FRAMES, DGRAMS, OVERRUNS, STALLS, every TAP_* register) and settings next
   to the recording as `tap_<time>.json`. It prints how many datagrams the FPGA
   sent and how many arrived.

The receive loop only timestamps and appends; nothing is parsed until
`compare`, so host load cannot turn into apparent loss.

### 7. Compare

```bash
$PY -m chain.tapcheck compare ~/intan-tap-recordings/tap_20261102-150000.chaintap \
    ~/Documents/RHX/session_241102_150012          # the RHX folder, info.rhd, or .rhd file(s)
```

Options: `--port A` if RHX recorded more than one port (default: the one with
most amplifier channels), `--show N` mismatches to list (20), `--anchors N`
(64), `--json PATH` (default `docs/results/tapcheck_compare_<time>.json`).
Exit status: 0 every RHX sample in the overlap was checked and equal, 1
something is missing or wrong, 2 the recordings could not be compared at all.
A 5-minute session takes a few seconds and well under 1 GB of memory.

## How compare decides, and how to read the report

```
chain  .../tap_20261102-150000.chaintap
       598 datagrams, 2990 frames, seq 0..2999, 1 seq gaps (10 frames), 0 partial, 0 malformed
RHX    .../session_241102_150012
       one-file-per-signal-type v3.3, 2000 S/s, port A (128 amplifier channels), 2048 samples (t 78277..80324), 0 timestamp gaps
align  RHX timestamp = chain seq + 77777  (64 of 64 matched anchors agree, 64 tried, fingerprints over expected channels)
format identity: chain ch 0-127
map    as expected: chain k = A-k (native order, MISO1's chip first)
overlap 2048 RHX samples (t 78277..80324)
       compared 260864 channel-samples in 2038 frames: equal 260862, mismatched 2
       frames missing 10, partial 0, not TAP 0, RHX gaps 0 (0 samples)
outside chain frames before/after RHX 500/452, RHX samples before/after chain 0/0
mismatches per chain channel: 5:1, 70:1
XOR bit histogram (bit 15..0): 1 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1
bits flipped per bad sample: 1:1, 2:1
first 2 mismatches:
   seq        rhx t  chain ch  rhx ch    expected  got     xor
   1000       78777         5  A-005     0x8008    0x800c  0x0004
   1200       78977        70  A-070     0x7fee    0xffef  0x8001
  [FAIL] MISSING_FRAMES: 10 RHX samples in the overlap have no chain frame (lost datagrams: seq gaps), first seqs [750, 751, 752, 753, 754]
  [FAIL] MISMATCH: 2 of 260864 channel-samples differ; most on ch 5 (1), ch 70 (1)
  [INFO] CHAIN_OUTSIDE: 500 chain frames before the RHX recording and 452 after it (expected: the tap records while RHX runs, RHX saves a window)
RESULT FAIL
```

- **align.** Both sides count the controller's sweeps, so one constant `D`
  relates them: RHX timestamp = chain seq + D. Neither the channel order nor
  the number format is trusted at this point. Each sample is reduced to a
  fingerprint that survives a permutation of channels and an XOR of any channel
  with 0x8000 or 0xffff: the sorted |x[t] - x[t-1]| over the channels. 64 RHX
  samples spread over the recording are looked up among the chain's
  fingerprints; each match votes for a D, and one D must win. Seq gaps and RHX
  gaps cost only the anchors that touch them. If anchors disagree, an
  OFFSET_INCONSISTENT finding lists them.
- **format** is per channel: `identity` (chain word = RHX raw code, offset
  binary, which is what section 7 promises), `top bit flipped` (two's
  complement of the same value), `inverted` (every bit flipped: that chip's
  MISO pair is swapped at the tap), `inverted, top bit kept`. Anything but
  identity is a CONVENTION failure.
- **map.** Expected: chain channel k is RHX native channel k of the port, MISO1's
  chip first, as section 7 and RHX both number them. If a channel does not
  match its partner, it is searched for among all RHX channels and formats, the
  result is printed as runs (`chain 0-63 = RHX A-064..A-127`), and the
  comparison then uses what was found, so a swap does not hide the other errors.
- **overlap** is every RHX sample whose timestamp falls in the chain's seq span.
  Each one is compared on every mapped channel: `equal` + `mismatched` =
  `compared`. `frames missing` are RHX samples with no chain frame (lost
  datagrams). `partial` frames are counted apart: their unheard channels read
  0 and would otherwise flood the mismatch counts. `RHX gaps` are RHX's own
  missing timestamps: those chain frames could not be checked.
- **outside**: chain frames before and after the RHX window are expected (the
  chain records longer). RHX samples outside the chain's recording were not
  checked; start `record` earlier next time.
- **mismatches**: per chain channel, the XOR between expected and received
  words as a per-bit histogram (bit 15 first), the number of bits flipped per
  bad sample, and the first `--show` mismatches with seq, RHX timestamp, both
  channel names and the words.
- **findings** carry a category (below) and FAIL, WARN or INFO. RESULT is FAIL
  when any finding is FAIL. The JSON has every number above, the RHX header
  summary, the alignment votes, the per-channel map and the recording's
  sidecar counters.

## Troubleshooting

| Seen | Likely cause | Check |
|---|---|---|
| `status` BUS_IDLE | RHX not running; no common ground; wires not on JA/JB | RHX Run; JA5/JA11 to C3430 Y T6; the pin table |
| `status` CS_SILENT, SCLK_SILENT, MOSI_SILENT, MISO1_SILENT or MISO2_SILENT | that pair open between C3430 Y and the Arty; for a MISO, that chip not driving | the pair's two wires; the chip's power |
| `status` CS_POLARITY / SCLK_POLARITY (RHX stopped) | + and - swapped on that pair | JA1/JA2, JA3/JA4 against B1/T1, B2/T2 |
| `status` FRAMING_ALL | CS pair swapped (the gaps between commands look like CS low), or SCLK not arriving | CS and SCLK polarity and contact |
| `status` FRAMING_SOME, PARTIAL | noise or reflections on CS/SCLK | shorter, twisted tap wires; the 220-470 ohm stub resistors |
| `status` NO_SWEEPS | MOSI or SCLK pair swapped (commands decode as something else) | JA7/JA8, JA3/JA4 polarity |
| `status` ALL_PARTIAL | MOSI open: every command reads CONVERT(0) | JA7/JA8 |
| `status` CHIP0_ERRORS / CHIP1_ERRORS; compare CONVENTION `inverted` on 0-63 or 64-127 | that chip's MISO pair swapped (+/-) at the tap | JA9/JA10 (MISO1) or JB1/JB2 (MISO2) |
| `status` DELAY_OUT or DELAY_RANGE | TAP_DELAY outside the window | `delay-sweep --apply` |
| `status` B_LATE; compare mismatches on chain 32-63 / 96-127, mostly bit 0 | module B's last bit sampled after CS rose | lower TAP_DELAY; record at 2 kS/s |
| `status` NO_CHECKS, `delay-sweep` "TAP_CHECKS never climbed" | RHX sends no fixed-answer commands while streaming | nothing wrong; judge TAP_DELAY with `compare` |
| compare MISSING_FRAMES | lost datagrams | the recording's `.json`: OVERRUNS > 0 is the FPGA (FIFO full, or a sweep the tap could not frame); DGRAMS > datagrams received is Wi-Fi or the Mac (firewall, sleep, another program on UDP 5001) |
| compare PARTIAL_FRAMES | framing errors while recording | TAP_FRAMING in the `.json`; the wiring rows above |
| compare MAPPING `chain 0-63 = RHX A-064..A-127` | MISO1 and MISO2 exchanged at the Arty | JA9/JA10 vs JB1/JB2 |
| compare MAPPING, other patterns | a real channel-order disagreement: keep the report | the per-channel map in the JSON |
| compare CONVENTION `top bit flipped` | something re-encodes the words (the tap must not) | an FPGA or host bug: keep the report |
| compare OFFSET_INCONSISTENT | a sweep left no frame | OVERRUNS in the `.json` (a burst the tap could not frame); otherwise an FPGA bug |
| compare RHX_GAPS | RHX's own lost samples (USB) | repeat; it is not the chain |
| compare RHX_NOTCH / RHX_REFERENCE | RHX filtered or re-referenced the saved data | notch Disabled, hardware reference, record again |
| compare RHX_OUTSIDE | RHX recorded before `record` started or after it stopped | start `record` first, stop it last |
| compare UNRECORDED | channels not enabled in RHX | enable all 128 |
| `cannot compare: no consistent offset` | the two files are not the same session; the tap heard garbage; or every channel is constant | file names and times; `status`; connect electrodes or a test signal (noise is enough) |
