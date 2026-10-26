# Real RHD2164 bring-up

The receiver on the Arty S7-25, talking to two real RHD2164 chips (128
channels) through an LVDS <-> single-ended converter board, instead of to the
emulator. Bitstream `headstage_arty_s7_25*.bit`, ID `0x1c100002`.

```
 2 x RHD2164 ==LVDS== converter ──3.3 V SPI── Arty S7-25 ──QSPI── ESP32-S3 ──UDP── Mac
 (headstage)   cable    board      Pmod JA      receiver   Pmod JC  (later)           viewer
                                                   └──── USB-UART: control + diagnostics ──── Mac
```

Everything up to "the chips answer, every channel converts, the link check is
clean at the sample rate" works over the USB-UART alone, with no ESP and no
Wi-Fi. The ESP only adds the full-rate live view.

## 1. Wiring

Take the five JA -> JB jumpers off; JB is unused. The SPI bus keeps the JA
pins the single-board bench used.

| Arty pin | Ball | Signal | Dir | Converter board (3.3 V side) |
|---|---|---|---|---|
| JA1 | L17 | `cs_n` | out | CS, both chips |
| JA2 | L18 | `sclk` | out | SCLK, both chips |
| JA3 | M14 | `mosi` | out | MOSI, both chips |
| JA4 | N14 | `miso0` | in | MISO of chip 0 -> channels 0–63 |
| JA7 | M16 | `miso1` | in | MISO of chip 1 -> channels 64–127 |
| JA5 or JA11 | | GND | | GND (required) |
| JA6 | | 3V3 | | converter board supply |

The ESP stays on Pmod JC exactly as before (JC1 -> GPIO10, JC2 -> GPIO12,
JC3 -> GPIO11, JC4 -> GPIO13, JC7 -> GPIO14, JC8 -> GPIO9, JC9 <- GPIO21,
JC5 -> GND). It can be left unplugged until step 4.

The MISO inputs have the FPGA's weak pull-down on, so a MISO wire that is off
reads **stuck low**. A chip that is not driving its MISO pair usually reads
**stuck high**, because LVDS receivers fail safe to 1. Bring-up reports which.

LEDs: **LD2** run, **LD3** heartbeat (dark = no clock / not programmed),
**LD4** QSPI datagrams leaving, **LD5** link good: the last sweep's
`READ(59)` came back `0x35` / `0x3A` from both chips, both modules.

## 2. Build and program

```bash
# on a Hammerschlag machine, Vivado on PATH
QSPI_DIV=6 make bit-headstage      # -> fpga/bitstreams/headstage_arty_s7_25_div6.bit

# on the Mac, Arty on USB (volatile: reload after every power cycle)
openFPGALoader -b arty_s7_25 fpga/bitstreams/headstage_arty_s7_25_div6.bit
```

`QSPI_DIV=6` (8 MHz QSPI) is the divider the ESP link was proven at.

## 3. Bring-up, staged

```bash
python3 -m pip install pyserial matplotlib numpy    # once
cd host
python3 -m chain.bringup --serial /dev/cu.usbserial-210352BF97961
```

The Arty enumerates two ports; the one ending in `1` is the UART, the other is
JTAG. Each stage prints PASS / WARN / FAIL, and every non-PASS line names a
LIKELY CAUSE and what to CHECK. A JSON report lands in `docs/results/`.
`python3 -m chain.bringup --fake chip1_absent` shows what a failure report
looks like without hardware.

| Stage | Proves | Needs |
|---|---|---|
| S0 FPGA | right bitstream loaded, UART works | USB |
| S1 Self-test | the receiver itself works: ROM reads from the **on-die** emulator over MODE bit3 loopback. JA stays idle | nothing wired |
| S2 Bus activity | the master clocks JA; each MISO line moves (or is stuck low / stuck high) | converter + chips |
| S3 Identity + eye | each chip, each module returns "INTAN", 53/58, 64, 4; finds the PHASE_SELECT window from raw taps; retries slower SCLK if none | |
| S4 Configure | MOSI reaches each chip intact: WRITE echoes, register read-back | |
| S5 Calibrate | ADC self-calibration ran and the chip resumed | |
| S6 Convert snapshot | all 128 channels convert; stuck, railed, dead module/chip flagged | |
| S7 Streaming | the always-on link check stays clean at the sample rate for N seconds | |

## 4. Seeing the data

```bash
# no ESP: UART snapshots, a few per second
python3 -m chain.viewer --uart --serial /dev/cu.usbserial-210352BF97961

# ESP on JC, flashed with -DBENCH_INGRESS=qspi: full rate over Wi-Fi
python3 -m chain.viewer --udp --serial /dev/cu.usbserial-210352BF97961 --start
```

`--start` configures the chips (S3–S5) and sets `run`. Channels are shown in
microvolts (0.195 µV/LSB, two's complement), four panels: chip 0 A/B, chip 1 A/B.

## 5. Pinpointing a fault

Read top to bottom; the first row that matches is where to look.

| Symptom | Where the fault is |
|---|---|
| no reply to `R 1f`, LD3 dark | not programmed (volatile), or the JTAG port instead of the UART one |
| `R 1f` = `1c100001` | the old emulator-chain bitstream is loaded |
| S1 fails | the FPGA build itself; nothing outside matters yet |
| S2: `SPI_CYCLES` does not move | receiver not running a list (FPGA side) |
| S2: a MISO **stuck low** | that MISO wire, the converter's power or ground, or its single-ended driver |
| S2: a MISO **stuck high** | that chip is not driving: chip power, that MISO LVDS pair, or CS/SCLK not reaching the chip |
| S2: both stuck high | the shared CS/SCLK/MOSI path, or headstage power |
| S3: bit-inverted ROM words | an LVDS pair's + and − swapped on the converter |
| S3: ROM right but shifted / no phase window at 24 MHz, window at slower SPI_DIV | delay beyond the capture window or marginal edges: cable length, converter speed; keep the slower `SPI_DIV` |
| S3: one chip only | the other chip's own MISO pair, power, or connector; the shared lines are fine |
| S4: echoes wrong, ROM fine | MOSI path (MISO is proven by S3) |
| S6: one module or chip all flat | that chip's amplifiers are powered down, or its reference/ground |
| S7: link errors climb slowly | marginal phase: re-run S3 and look at window width; try `SPI_DIV` 1 |
| S7 clean, viewer `--udp` empty | ESP or Wi-Fi: rerun the single-board bench's step 3 (`MODE` bit2 internal pattern) |

## 6. Doing it by hand

The registers are in `CONTRACTS.md` section 4 and the diagnostic list in
section 6. In `screen /dev/cu.usbserial-…1 115200` (end lines with Ctrl-J):

```
R 1f              -> 1f=1c100002     headstage bitstream
W 03 00000008     -> ok              loopback to the on-die emulator
W 03 00000000     -> ok              back to the real chips
W 02 00000006     -> ok              PHASE_SELECT 6
W 00 00000001     -> ok              run (2 kS/s by default)
R 16              -> link checks, climbing
R 17 / R 18       -> link errors chip 0 / chip 1, [15:0] A, [31:16] B: want 0
R 19 / R 1a       -> what READ(59) returned: want 003a0035
R 1b              -> MISO_STAT: 0000000f = both lines seen low and high
```

Running sweeps reads only ROM and CONVERT results; unconfigured chips still
pass the link check, so a clean `LINK_ERR` with garbage channel data points at
configuration (S4/S5), not at the wiring.
