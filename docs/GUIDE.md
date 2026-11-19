# intan-chain: the guide

*Written 2026-11-18, at the end of the first wireless demo with real chips and
an HD-EMG sleeve. Branch `headstage-rhd2164`.*

This is the document to read first. It explains what the system is, why each
piece is built the way it is, the exact configuration that works today, what
went wrong on the way there and how each problem was found, and what every
file in the repository is for. The other documents in `docs/` go deeper on
single topics and are linked where they matter.

Contents

1. [What this is](#1-what-this-is)
2. [Background: the parts and why they were chosen](#2-background-the-parts-and-why-they-were-chosen)
3. [High-level design](#3-high-level-design)
4. [The working configuration](#4-the-working-configuration)
5. [Running a demo, step by step](#5-running-a-demo-step-by-step)
6. [Learnings from getting it working](#6-learnings-from-getting-it-working)
7. [Measured results](#7-measured-results)
8. [Open issues](#8-open-issues)
9. [Every file, briefly](#9-every-file-briefly)

---

## 1. What this is

A wireless high-density surface EMG (HD-EMG) acquisition chain, built as a
measurable prototype:

```
 HD-EMG sleeve ─ 2 × RHD2164 ═LVDS═ C3490 LVDS adapter ─3.3 V SPI─ Arty S7-25 FPGA ─QSPI─ ESP32-C5 ─Wi-Fi─ MacBook
 (electrodes)    (headstage,         (RHD half)          Pmod JA    receiver      Pmod JC  node     5 GHz   viewer
                  128 channels)                                        │                     CMU-DEVICE
                                                                       └──── USB-UART: control, bring-up, link diagnostics ──── Mac
```

Two Intan RHD2164 amplifier chips digitise 128 electrode channels. An FPGA
drives their SPI bus, captures the results, packs them into finished network
datagrams and hands those to an ESP32-C5 over quad SPI. The ESP adds a small
trailer and sends them over Wi-Fi to the Mac, where a Python viewer plots all
128 channels live. A text UART on the FPGA board configures everything and
exposes counters that make every stage of the chain observable.

The project started as a benchmark rig with a simulated chip (an RHD2164
emulator in FPGA logic) to answer three questions honestly: how much
**throughput** the wireless link sustains, what the **latency** is, and
whether the data arrives **bit-exact**. That design discipline is still
visible everywhere: every frame is timestamped at capture and at
transmission, a closed-form test pattern lets the host score every bit, and
every place data can be lost has its own counter.

---

## 2. Background: the parts and why they were chosen

### The RHD2164

Intan's RHD2164 is a 64-channel biopotential amplifier and 16-bit ADC on one
die. Things about it that shape the whole design:

- **SPI, 16-bit commands, a two-deep pipeline.** Every chip-select cycle
  sends one 16-bit command on MOSI and returns 16 bits on MISO, but the result
  that comes back belongs to the command sent *two cycles earlier*. Commands
  are `CONVERT(ch)`, `READ(reg)`, `WRITE(reg, val)` and `CALIBRATE`.
- **Two modules per chip, double data rate.** The 64 channels are two
  32-channel halves ("module A" and "module B"). Each CONVERT returns one
  sample from each, on the same MISO line: A on the rising SCLK edge, B on the
  falling edge. One CS cycle therefore yields 32 bits.
- **A ROM that says "INTAN".** Registers 40–44 read back the ASCII letters
  `I N T A N`, and 59–63 identify the die. Reading them is the cheapest proof
  that the bus works, and the link check uses `READ(59)`, which returns `0x35`
  on module A and `0x3A` on module B.
- **LVDS headstage.** On a real headstage the SPI lines are differential
  (LVDS) pairs. Intan's C3490 adapter board converts them to 3.3 V
  single-ended signals the FPGA can use. Its LVDS receivers *fail safe high*:
  a MISO pair that nobody drives reads as constant 1.
- **Units.** 0.195 µV per LSB, so the ±32767 range is about ±6.4 mV. Surface
  EMG is 10–100 µV at rest and up to a few mV in a strong contraction.
- **Each channel is electrode minus REF.** All 64 amplifiers measure against
  one common reference input. A reference that is not on the body puts the
  same large mains hum on every channel (see [6.4](#64-the-analog-front-end-and-mains-hum)).

### Why an FPGA reads the chips

At 2 kS/s per channel, one sample of 128 channels takes 35 SPI commands (32
CONVERTs plus 3 auxiliary slots). The timing has to be exact and the MISO
data has to be sampled at the right moment after a cable and two LVDS
conversions have delayed it. The FPGA oversamples MISO 4× and picks the
right sample point with a *phase select* register, the same mechanism Intan
uses in their Rhythm FPGA design. A microcontroller cannot do that
deterministically at 24 MHz.

### Why an ESP32 sends the data

The ESP provides the Wi-Fi. It is deliberately "dumb": the FPGA builds the
complete datagram, five frames and 1380 bytes, and the ESP only appends a
12-byte trailer and sends it. It never parses, reframes or allocates, so its
behaviour is easy to reason about and any throughput number cannot be credited
to clever ESP code. The link between them is the ESP's quad-SPI slave in
"half-duplex segment" mode, a protocol fixed by the ESP silicon. The FPGA's
`qspi_master` was written to match it bit for bit from the ESP-IDF source.

The node started on an ESP32-S3. It moved to an **ESP32-C5-DevKitC-1 (N8R8)**
when the S3 board began browning out at Wi-Fi start-up. The C5 has one CPU
core, a dual-band (2.4 + 5 GHz) radio, and 8 MB of PSRAM, and the last two
turned out to matter a great deal.

### The contract

`CONTRACTS.md` is the single definition of every byte the three
implementations (FPGA RTL, ESP C, host Python) exchange: the frame, the
datagram, the test pattern and the UART registers. The C header and the Python
module are *generated* from it (`make contract`), and CI fails if they are
stale. In summary:

- **Frame** (276 B): a 20-byte header (magic `0xA55A`, type, flags, `seq`,
  `t_cap`, `t_tx`, `n_ch`, `payload_len`) plus 128 × int16. `t_cap` and `t_tx`
  are FPGA ticks at 96 MHz: when the sample was captured and when it entered
  the QSPI transfer.
- **Datagram** (1392 B): 5 frames plus a 12-byte ESP trailer (`esp_rx_us`,
  `esp_tx_us`, `pipe_depth`). It fits in one Ethernet MTU, so no IP
  fragmentation.
- **Test pattern:** a per-channel 16-bit LFSR (CHECK) or a smooth triangle
  (REALISTIC). Both the FPGA and the host generate it, so the host can verify
  every bit without data files.

---

## 3. High-level design

### Data path, in order

| Stage | Where | What happens | Buffering |
|---|---|---|---|
| 1. Sweep | `spi_master.sv` | Every `SAMPLE_PERIOD` ticks (48000 at 96 MHz = 2 kS/s), 35 SPI commands go to both chips over the shared CS/SCLK/MOSI; each chip answers on its own MISO. MISO is sampled 4× per bit and `miso_phase_select` picks the bit at `PHASE_SELECT`. SCLK = 24 MHz / (`SPI_DIV`+1). | none |
| 2. Link check | `rx_top.sv` | One AUX slot per sweep reads ROM register 59; every result is compared with `0x35`/`0x3A` and failures counted per chip and module (`LINK_ERR0/1`). Every streamed sweep proves its own alignment. | — |
| 3. Frame build | `frame_builder.sv` | The 128 values (arriving interleaved 0, 32, 64, 96, 1, …) are written into a datagram slot in block RAM in contract order, with the header. | 4 slots (`slot_fifo`), 10 ms at 2 kS/s |
| 4. QSPI | `qspi_master.sv` | When the ESP raises its *ready* line, the oldest finished slot is shifted out as one WRDMA + WR_DONE transaction at 8 MHz (`QSPI_DIV=6`); `t_tx` is patched in on the way out. If ready is low, the FPGA waits (`STALLS`); if all 4 slots fill, it drops one (`OVERRUNS`). | — |
| 5. ESP ingress | `source_qspi.c` | The GP-SPI2 slave DMA-receives the 1380 bytes straight into a pipe buffer. | 16 DMA buffers (`pipe.c`) |
| 6. ESP egress | `sink_tcp.c` (or `sink_udp.c`) | TCP: a spool task copies each datagram into a 2000-slot PSRAM spool, and a sender task writes the spool into one TCP connection to the Mac. UDP: one `sendto()` per datagram, dropped if the radio is behind. | 2000 datagrams = 5 s (TCP) |
| 7. Air | Wi-Fi | CMU-DEVICE, 5 GHz only (`BENCH_WIFI_BAND=5g`). | — |
| 8. Host | `viewer.py`, `scroll.py` | `TcpSource` accepts the connection and cuts the byte stream into 1392-byte records; `Buffers` counts seq gaps; a numpy ring (`scroll.History`) keeps the last `--history` seconds; a render thread decimates it to one min/max column per pixel and paints 128 lanes. | 60 s by default (`--history`) |

The sizes chain together: 1380-byte payload, 1392 bytes with the trailer;
400 datagrams/s at 2 kS/s; 4.45 Mbit/s of payload on the air.

### Control path

The Arty's USB-UART (115200 baud, the board's *second* FTDI port) carries a
line-based text protocol: `R aa` reads a register, `W aa vvvvvvvv` writes
one. It is plain text so it can be driven by hand from `screen` (end lines
with Ctrl-J, not Enter). The register map is in `CONTRACTS.md` §4:
- **Setup:** run/clear, sample period, phase select, mode, SPI divider.
- **Counters:** frames, datagrams, FIFO high-water, overruns, stalls, SPI
  cycles.
- **Link checks:** check count, error counts, last results, MISO activity.
- **Diagnostic command list (`DIAG_*`):** up to 64 arbitrary SPI commands
  that the FPGA plays once and records, both decoded and as raw oversampled
  MISO taps.

The diagnostic list is how the host talks to the chips before streaming. With
the raw taps the host can decode every phase itself, so one capture is a whole
"eye scan". `chain/bringup.py` uses it to walk up the link in stages, each of
which names a physical cause when it fails:

| Stage | Proves |
|---|---|
| S0 FPGA | right bitstream (`ID 0x1c100002`), UART works |
| S1 Self-test | the receiver works, via MODE bit 3 loopback onto an emulator compiled into the same bitstream; JA stays idle |
| S2 Bus activity | SCLK runs; each MISO line toggles (a stuck-low line is a loose wire; stuck-high means the chip is not driving) |
| S3 Identity + eye | each chip and module returns "INTAN" and its ID; finds the `PHASE_SELECT` window; retries slower `SPI_DIV` if needed |
| S4 Configure | writes registers 0–21 (Intan's Rhythm recipe: 1–500 Hz band, DSP high-pass) and reads them back, which proves MOSI |
| S5 Calibrate | ADC self-calibration ran and the chip resumed |
| S6 Convert snapshot | all channels convert; stuck, railed and dead channels flagged |
| S7 Streaming | the per-sweep link check stays at zero errors for N seconds at the sample rate |

### Observability: where each kind of loss is counted

| Where | Counter | Meaning |
|---|---|---|
| FPGA | `STALLS` | ticks spent waiting for the ESP's ready line; normal in small amounts |
| FPGA | `OVERRUNS` | datagrams dropped because the 4-slot FIFO was full |
| FPGA | `LINK_ERR0/1` | sweeps whose ROM read came back wrong: the SPI eye is closing |
| ESP, 1 Hz `pipe:` line | `drops`, `txerr` | source found no buffer; send failed or was discarded |
| ESP, 1 Hz `tcp:` line | spool now / high / drops, dropped while unconnected | spool occupancy and overflow |
| Host | seq gaps, frames lost, malformed | what actually arrived |

---

## 4. The working configuration

The state of the bench on 2026-11-18, after the demo was working.

### Hardware

- **Electrodes:** HD-EMG sleeve on a 128-channel headstage with two RHD2164s.
  REF and GND both connected to the subject's skin.
- **Adapter:** Intan C3490 LVDS adapter board, **RHD half** (the RHS half is
  electrically separate, grounds included).
- **FPGA:** Digilent Arty S7-25 (xc7s25csga324-1).
- **Node:** ESP32-C5-DevKitC-1, N8R8 (8 MB flash, 8 MB PSRAM), CP2102N USB
  bridge.
- **Host:** MacBook (macOS 14), **running on battery** while recording.

### Wiring: FPGA Pmod JA to the C3490 RHD header

| Arty | Signal | Direction | Adapter |
|---|---|---|---|
| JA1 (L17) | cs_n | out | CS |
| JA2 (L18) | sclk | out | SCLK |
| JA3 (M14) | mosi | out | MOSI |
| JA4 (N14) | miso0 | in | MISO1: chip 0, channels 0–63 |
| JA7 (M16) | miso1 | in | MISO2: chip 1, channels 64–127 |
| JA6 | 3V3 | | VDD (adapter supply) |
| JA5 **and** JA11 | GND | | **two or more** GND pins |

The adapter's RHD header: back row `VDD MOSI MISO1 MISO2 GND`, front row
`GND CS SCLK GND GND`. MOSI sits behind CS, not beside SCLK.

### Wiring: FPGA Pmod JC to the ESP32-C5

| Arty JC | Signal | C5 GPIO | DevKitC-1 pin |
|---|---|---|---|
| JC1 | qspi_cs_n | GPIO10 | J1-11 |
| JC2 | qspi_sclk | GPIO6 | J1-7 |
| JC3 | qspi_dq0 | GPIO8 | J1-9 |
| JC4 | qspi_dq1 | GPIO9 | J1-10 |
| JC7 | qspi_dq2 | GPIO5 | J3-9 |
| JC8 | qspi_dq3 | GPIO4 | J3-8 |
| JC9 | esp_ready (C5 → FPGA) | GPIO23 | J3-5 |
| JC5 / JC11 | GND | G | J1-15, J3-1/12/15 |

Never connect a 3V3 pin (Pmod pins 6 and 12) to the ESP: each board is
powered from its own USB. The S3 pin map is different (`esp/main/pins_s3.h`);
do not mix them up.

### FPGA

- **Bitstream:** `fpga/bitstreams/headstage_arty_s7_25_div6.bit`. Built from
  `ababfb1` with Vivado 2023.2, WNS +0.513 ns, ID `0x1c100002`, SHA-1
  `2204cd3eb96292d9ad675ca1a5c55a24d69d5383`. QSPI at 8 MHz. The `.bit` is
  gitignored; `fpga/bitstreams/README.md` records how to rebuild it.
- **Loading:** configuration is volatile, so reload after every power cycle:
  `openFPGALoader -b arty_s7_25 fpga/bitstreams/headstage_arty_s7_25_div6.bit`
- **SPI:** `SPI_DIV 3` (SCLK 6 MHz) at `PHASE_SELECT 2`, chosen by bring-up.
  At 6 MHz both chips stream with zero link errors. 12 MHz works with one chip
  but corrupts about 4% of sweeps with two.
- **Rate and band:** 2000 S/s per channel; analog band 1–500 Hz; DSP
  high-pass about 1.2 Hz.

### ESP32-C5 firmware

The `build/c5_qspi_tcp` build:

| Setting | Value | Why |
|---|---|---|
| ingress | `-DBENCH_INGRESS=qspi` | data from the FPGA |
| band | `-DBENCH_WIFI_BAND=5g` | 2.4 GHz channel was congested (see §7) |
| egress | `CONFIG_CHAIN_SINK_TCP=y` | no holes in the stream |
| spool | `CONFIG_CHAIN_TCP_SPOOL_DGRAMS=2000`, `CONFIG_SPIRAM=y` | 5 s of outage absorbed in PSRAM |
| Wi-Fi/lwIP buffers | internal RAM (`SPIRAM_TRY_ALLOCATE_WIFI_LWIP` off) | in PSRAM, TCP managed only 230 of 400 datagrams/s |
| TCP send buffer | 32 kB | keeps the internal heap above 30 kB |
| SSID | `CMU-DEVICE`, open, MAC registered | |
| host IP | the Mac's current IPv4 (`ipconfig getifaddr en0`) | compiled in; **rebuild when it changes** |

All of the non-network settings are in the committed overlay
`esp/sdkconfig.defaults.tcp`. The SSID and host IP are set in menuconfig and
stay out of git. ESP-IDF is `v6.0-dev-2980-gab149384e1` at
`~/Desktop/Research/esp-idf`.

The UDP builds are kept as fallbacks: `build/c5_qspi` (2.4/auto band) and
`build/c5_qspi_5g`. The viewer must be in `--udp` mode for those.

### Mac

- Python venv `~/.venvs/intan-chain` (`host/requirements-viewer.txt`: numpy,
  matplotlib, pyserial).
- Ports: the Arty UART is `/dev/cu.usbserial-210352BF97961`; `…960` is the
  JTAG port and never answers. The C5 is `/dev/cu.usbserial-210`.
- Network: the Mac on CMU-SECURE, the C5 on CMU-DEVICE; the two subnets route
  to each other. The macOS application firewall is off.
- **Run Python tools from `host/`.** `python -m chain.viewer` from anywhere
  else fails with "No module named 'chain'".

### Where the code lives on this Mac

The repository's main checkout (`BASIC_INTAN_CHAIN_CODE/intan-chain`) is on an
older branch (`single-board-s7-25`) whose README shows the *ESP32-S3* pin map.
The working code is the `headstage-rhd2164` branch, checked out in the git
worktree `intan-chain/.claude/worktrees/study-doc`. The `intan-tap` worktree
holds a later branch for checking against Intan's own controller; it is not
needed for the demo.

---

## 5. Running a demo, step by step

```bash
REPO=~/Desktop/Research/HDEMG_PCB_WORK/BASIC_INTAN_CHAIN_CODE/intan-chain/.claude/worktrees/study-doc
```

1. **Power and safety.** Unplug the Mac's charger. Wire JA and JC (§4) with
   the boards unpowered. Attach the sleeve, REF and GND to the subject.
2. **Program the FPGA:**
   `cd $REPO && openFPGALoader -b arty_s7_25 fpga/bitstreams/headstage_arty_s7_25_div6.bit`
3. **Check it** (optional): in `screen /dev/cu.usbserial-210352BF97961 115200`,
   type `R 1f` then Ctrl-J. Expect `1f=1c100002`.
4. **The C5** boots on USB power by itself. Its log should show `band: 5 GHz
   only`, `connected with CMU-DEVICE`, `spool 2000 datagrams … in PSRAM`, then
   `no listener` until the viewer starts. Opening its serial port reboots it.
5. **Start the viewer**, which configures the chips (S0, S2–S5) and starts
   streaming:
   ```bash
   cd $REPO/host && ~/.venvs/intan-chain/bin/python -m chain.viewer --tcp \
       --serial /dev/cu.usbserial-210352BF97961 --start --spi-div 3
   ```
   The status line should say `connected to <C5 address>`, about 2000 fr/s,
   `seq gaps 0`, and LINK_ERR 0 on all four lanes.

   The window shows all 128 channels as horizontal traces in one panel
   (channel 0 on top, module boundaries marked) with an activity strip above.
   It keeps 60 s of history (`--history`). Main controls:
   - **Time:** drag the bar under the panel, or ← / → to pan, Home / End to
     jump; space parks the view and resumes live; `[` / `]` change the window
     length.
   - **Channels:** ↑ / ↓ or the wheel scroll through them; `+` / `-` change
     the number of lanes (128 down to 8); `1`–`4` show one module, `0` all.
   - **Display:** `s` toggles a shared scale, `m` toggles the activity strip;
     hover a lane or tile for its numbers.

   Lost frames break the trace and get a red mark under the panel. The full
   key table is in `host/README.md`.
6. **Full diagnosis** when something looks wrong: close the viewer, then
   `python -m chain.bringup --serial /dev/cu.usbserial-210352BF97961 --spi-div 3 --snapshots 5 --seconds 10 --leave-running`
   and read the summary. Then reattach the viewer *without* `--start`, so the
   settings bring-up chose stay in force.

What healthy data looks like: tens of µV RMS at rest, no RAIL tiles, traces
that rise with contraction. Millivolts of identical 60 Hz on every channel
means the reference or the mains path (§6.4).

---

## 6. Learnings from getting it working

Roughly in the order they were hit. Each one cost real time.

### 6.1 FPGA ↔ ESP link

- **The ESP's QSPI slave only worked through the GPIO matrix.** On the
  direct IOMUX route it never decoded a command (`cmd_err` on every
  transaction) although all six lines were live. `slave_init()` forces the
  matrix. The cause is still open; the suspicion is ringing on the jumper
  wires.
- **The receiver must re-arm on a timer, not only on completions.** If the
  buffer pool ran dry with nothing armed, nothing ever completed again, ready
  stayed low and the FPGA stalled for good.
- **The ready line has a race.** At 8 kS/s one datagram arrived twice. The
  proposed fix (drive ready from the driver's callbacks) is not written.
- **Idle pin levels prove little.** `cs10=high` on the C5's idle report proves
  CS and ground are wired. Only data proves the clock and the four data lines
  are, which is why the check is the FPGA's internal CHECK pattern (`MODE`
  bit 2) scored bit by bit on the Mac.
- **"Loss 100%" can be a scoring artifact.** One `chain.sweep` run reported
  100% loss with zero bit errors. Leftover frames from an earlier run made the
  seq span wrap to about 2³². Count unique datagrams directly when a number
  looks absurd.

### 6.2 ESP32 boards

- **The S3 board browned out** at Wi-Fi start-up on every USB port; the C5
  replaced it with no FPGA change.
- **C5 strapping pins:** the C5's SPI2 IOMUX MOSI/MISO pads (GPIO7, GPIO2) are
  strapping pins, so D0/D1 use GPIO8/9 through the matrix.
- **Opening the C5's serial port reboots it** (CP2102N DTR/RTS auto-reset),
  even with DTR/RTS held low. Plan for a 3–4 s Wi-Fi rejoin after every look
  at its log.
- **The host IP is compiled into the firmware.** When the Mac's DHCP address
  changed, the C5 kept streaming to the old address and the viewer saw
  nothing. Check `ipconfig getifaddr en0` before a demo.
- **CMU-DEVICE admits registered MACs only.** An unregistered board associates
  and is dropped right after.

### 6.3 SPI link to the real chips

- **Ground with more than one wire.** With one GND jumper to the adapter, SCLK
  and CS looked clean on a scope yet the chips executed nothing: the adapter's
  ground bounced on every SCLK edge and its inputs misread MOSI. Bring-up
  reports this as `MOSI_IGNORED`. Two or three ground wires fixed it.
- **The eye is narrow and moves.** Only one `PHASE_SELECT` value works for all
  four lanes at a time. On different runs bring-up chose 12 MHz/phase 4,
  6 MHz/phase 2 and 3 MHz/phase 1. After a sleeve replug, 6 MHz read nothing
  at all once and 3 MHz worked. Let bring-up choose, and re-run it after
  touching any connector.
- **Two chips are harder than one.** One chip read cleanly at 12 MHz. With
  both connected, 12 MHz corrupted 391 of 10 036 sweeps on *all four lanes at
  once*: a shared cause (clock, chip select, ground), not a per-chip one.
  6 MHz gave 0 errors in 20 040 sweeps.
- **"Stuck high" means not driving.** With one chip unplugged, its MISO reads
  constant 1 (the adapter's LVDS failsafe), and bring-up says
  `MISO_STUCK_HIGH` / `CHIP_NOT_ANSWERING`. A loose JA wire instead reads
  stuck *low* (the FPGA input has a pull-down).

### 6.4 The analog front end and mains hum

This took the longest, and the first explanation was wrong.

- **Symptom:** every channel about 4 mV RMS, many at the ADC rail.
- **Measurement that located it:** about 90% of the power was at 60 Hz and its
  harmonics. The 60 Hz component had nearly the same phase on all 128 channels
  (within about 15°), and 87–97% of all signal power was identical on every
  channel. Common-mode hum on every channel at once points at the shared
  reference, not at individual electrodes.
- **First fix, needed but not enough:** connect REF and GND to the body. The
  hum stayed.
- **Real cause: the Mac's charger.** The boards are powered from the Mac over
  USB. With the charger in the wall, the circuit ground is linked to mains
  through the charger's noise-filter capacitor. With REF/GND on the subject,
  that closes a loop through the body (wall → charger → Mac → USB → boards →
  GND wire → person → capacitive coupling → wall). The 60 Hz current through
  the skin is no longer identical at every electrode, so the amplifiers can't
  cancel it. **On battery the loop is open and the hum disappears.** This is
  why biopotential recorders run on batteries or isolated supplies. If the Mac
  must charge, use a USB isolator between the Mac and the boards.

### 6.5 Wi-Fi: from 75% to 100% delivery

- **What UDP loss actually was.** The ESP's `sendto()` failed with ENOMEM when
  the radio could not get airtime (`txerr`). Every datagram the ESP did hand
  to the socket reached the Mac, bit-exact. So the loss was at the node's
  transmit queue, during bursts of 0.1–0.4 s with no airtime.
- **2.4 GHz was congested and unstable:** 64% to 93% delivery from one run to
  the next on channel 1.
- **5 GHz is cleaner:** 94.8%, and error-free after the first seconds, even
  at a weaker −70 dBm. CMU-DEVICE is broadcast on 5 GHz channels 36, 52, 112,
  124 and 149 here. The driver's "auto" band picked 2.4 GHz, so the band is
  forced at build time.
- **TCP alone did not help.** The first TCP sink blocked on a slow radio and
  so backed up into the 16-buffer pipe and the 4-slot FPGA FIFO, about 170 ms
  in all, against outages of 280–430 ms. It lost as much as UDP (now as FPGA
  overruns) and pushed the internal heap down to 6.9 kB.
- **TCP plus a PSRAM spool works.** Every datagram is copied into a
  2000-slot spool in PSRAM at once, so the FPGA never waits. A separate task
  drains it into the socket and simply waits out bad air; TCP retransmits
  underneath. Result: 8011 of 8011 datagrams in 20 s, peak spool 345 (0.9 s
  behind), heap low point 36 kB.
- **But keep Wi-Fi buffers out of PSRAM.** With
  `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` on, the single-core C5 sustained only about
  230 datagrams/s over TCP, below the 400 needed, and the spool overflowed.
- **The cost of TCP is latency, not data.** During a bad patch the display
  falls up to about a second behind and then catches up. Time on the x axis
  comes from the FPGA's `t_cap`, so the traces stay correct.

### 6.6 The viewer

- **Four panels of 32 lanes was the wrong shape** for watching a sleeve: the
  user wanted every channel as one horizontal trace and the ability to scroll
  back in time. The viewer is now one panel of 128 lanes over a 60 s history,
  with time and channel navigation.
- **Keeping it fast means drawing pixels, not samples.** 128 channels ×
  2000 S/s is too much for matplotlib lines. The history is decimated to one
  min/max column per pixel. The columns are pinned to time, so a live update
  shifts the image and paints only the newest columns, and the GUI thread only
  blits finished images. On an M1 Pro Retina display that is about 5 ms render
  + 4 ms GUI per live frame.
- **Gaps are drawn, not hidden.** A lost datagram breaks every trace and gets
  a red mark, so loss is visible even at a 60 s zoom. That is how the Wi-Fi
  problem became obvious in the first place.

### 6.7 Tooling

- **Two processes cannot share the UART or the data port.** Bring-up, sweeps
  and the viewer each take the Arty UART and port 5001. Close one before
  starting another; a stray command in the middle of someone else's bring-up
  steals a reply line.
- **Bring-up leaves the FPGA running** with `--leave-running`; attach the
  viewer without `--start` to keep its choices. `--start` re-runs S0 and S2–S5
  and may choose a different SPI speed and phase.
- The `sweep`/`metrics` tools score UDP captures; there is no TCP mode in
  `chain.sweep` yet.

---

## 7. Measured results

All runs at 2 kS/s × 128 channels, using the FPGA's CHECK pattern unless noted.

| Date | Path | Delivered | Notes |
|---|---|---|---|
| 2026-10-13 | emulator → jumpers → FPGA → S3 → home Wi-Fi | bit-exact at 2, 4, 8 kS/s | single-board bench, `docs/report-single-board-chain.md` |
| 2026-10-27 | FPGA → C5 → CMU-DEVICE 2.4 GHz, UDP | 20 026 / 20 026 frames | latency: FPGA 1.14 ms median, ESP 13 µs, air ~3.3 ms (p99 11.6 ms) |
| 2026-11-18 | same, UDP, 2.4 GHz | 64–93% across 5 runs | 0 bit errors; loss = `sendto` ENOMEM bursts |
| 2026-11-18 | TCP without spool, 2.4 GHz | 92.6% | loss moved to FPGA overruns; heap low 6.9 kB |
| 2026-11-18 | UDP, 5 GHz (ch 36, −70 dBm) | 94.8% | 73 gaps in 10 s, then clean |
| 2026-11-18 | TCP + PSRAM spool, Wi-Fi buffers in PSRAM, 5 GHz | 230 of 400 datagrams/s | spool overflowed |
| 2026-11-18 | **TCP + PSRAM spool, Wi-Fi buffers internal, 5 GHz** | **8011 / 8011** | 0 gaps, 0 bit errors in 82 Mbit, spool peak 345 |
| 2026-11-18 | real chips, both, 6 MHz SPI, phase 2 | 0 link errors in 20 040 sweeps | `bringup_20261118-190507.json` |

Raw records are in `docs/results/` (one JSON per run; `python -m chain.report`
tabulates them).

---

## 8. Open issues

- **Narrow SPI eye.** Every run found a single-phase window. Better
  grounding, a shorter cable or a slower SCLK would widen it; the per-sweep
  link check makes any closure visible immediately.
- **Host IP compiled in.** The node should learn the host address (for
  example from the first packet the viewer sends) instead of needing a rebuild.
- **Ready-line race** in `source_qspi.c` (one duplicate datagram seen at
  8 kS/s).
- **`chain.sweep` has no TCP mode**, so TCP runs are measured with an ad-hoc
  script for now.
- **Codec** (`MODE` bit 1, frame type 2) is reserved but not written.
- **`main` has never been merged with this branch line.**

---

## 9. Every file, briefly

### Top level

| File | Purpose |
|---|---|
| `README.md` | Overview, the single-board emulator bench, bring-up order, known ceilings. |
| `CONTRACTS.md` | The one definition of frames, datagrams, the test pattern, the UART registers, the SPI cycle and the diagnostic list. |
| `Makefile` | `make check` (contract + host tests + FPGA sims), `make contract`, `make vectors`, `make bit-*` (Vivado builds). |
| `.gitignore` | Build products, bitstreams, ESP `sdkconfig` and `build/`, Python caches. |
| `.github/workflows/ci.yml` | CI: contract freshness, host pytest, every FPGA testbench under Icarus. |
| `.github/workflows/esp.yml` | CI: builds the ESP firmware (S3, ESP-IDF v5.4 image). |

### `docs/`

| File | Purpose |
|---|---|
| `GUIDE.md` | This document. |
| `headstage-bringup.md` | Wiring to real chips, staged bring-up, a symptom → fault table. |
| `report-single-board-chain.md` | Full write-up of the emulator bench: architecture, verification, results. |
| `study-how-it-works.md` | A code walkthrough of the FPGA → ESP → Mac path for a human reader. |
| `bench-log-2026-10-13.md` | Overnight bench log of the single-board chain. |
| `results/*.json` | One file per measurement or bring-up run (`rate*`, `pattern_*`, `emulator_*`, `bringup_*`). |

### `tools/`

| File | Purpose |
|---|---|
| `gen_contract.py` | Generates `esp/main/chain_contract.h` and `host/chain/contract.py` from the tables mirroring `CONTRACTS.md`; `--check` fails if stale. |
| `gen_pattern_vectors.py` | Freezes the host's test pattern into `.mem` vectors for the RTL testbench. |

### `fpga/rtl/` — the receiver (vendor-neutral, simulates in Icarus)

| File | Purpose |
|---|---|
| `gen/pattern_gen.sv` | The CHECK/REALISTIC test pattern in hardware, one channel per clock. |
| `rx/rx_top.sv` | The receiver: control, SPI or pattern source, frame builder, slot FIFO, QSPI egress, link check, MISO monitor, diagnostic list. |
| `rx/spi_master.sv` | RHD2164 command sequencer (sweeps and diagnostic lists) with 4× oversampled DDR capture. |
| `rx/miso_phase_select.sv` | Picks the 16 data bits out of the oversampled MISO at `PHASE_SELECT` (Intan's Rhythm mechanism). |
| `rx/frame_builder.sv` | Lays the channel stream and header into a datagram slot in contract order. |
| `rx/dgram_mem.sv` | The datagram slot RAM (inferred block RAM). |
| `rx/slot_fifo.sv` | Bookkeeping of which slots are full; high-water and overrun counting. |
| `rx/qspi_master.sv` | Shifts a slot to the ESP's slave-HD peripheral (WRDMA + WR_DONE), gated by the ready line; patches `t_tx`. |
| `rx/diag_mem.sv` | The diagnostic command list and its decoded and raw-tap results. |
| `rx/ctrl_regs.sv` | Parses the UART text protocol; the register file. |
| `rx/uart_rx.sv`, `rx/uart_tx.sv` | 8N1 UART at 115200. |

### `fpga/boards/` — board wrappers and pin constraints

| File | Purpose |
|---|---|
| `headstage_arty_s7_25_top.sv` + `.xdc` | **The demo bitstream:** receiver to real chips on JA, QSPI on JC, on-die emulator for loopback self-test, MMCM 96 MHz. |
| `chain_arty_s7_25_top.sv` + `.xdc` | Single-board bench: emulator and receiver on one Arty, looped through JA→JB jumpers. |
| `rx_arty_s7_top.sv` | Receiver-only wrapper (Arty S7-50 or Boolean). |
| `rx_arty_s7_50.xdc`, `rx_boolean.xdc` | Pins for the receiver-only wrapper on each board. |
| `emu_arty_s7_25.xdc` | Pins for the vendored emulator alone on an Arty S7-25. |

### `fpga/sim/` — testbenches (all self-checking; `run_sim.sh` runs them)

| File | Purpose |
|---|---|
| `tb_spi_master.sv` | Closed loop: `spi_master` against two unmodified emulator cores; all 128 channels recovered. |
| `tb_rx_top.sv` | The whole receiver driven over the real UART wires; data checked off the QSPI pins. |
| `tb_frame_builder.sv` | Datagram layout, out-of-order channels, overrun and drop behaviour. |
| `tb_pattern_gen.sv` | RTL pattern against the host's golden vectors. |
| `tb_ctrl_regs.sv` | UART control interface, bit-banged at the true baud rate. |
| `tb_chain_arty_s7_25.sv` | The single-board top from the pins out, jumpers modelled. |
| `tb_headstage_arty_s7_25.sv` | The headstage top with modelled chips behind a delaying converter, plus loopback. |
| `qspi_hd_monitor.sv` | Model of the ESP's slave-HD receiver, built from the ESP-IDF source. |
| `unisim_stubs.sv` | Minimal IBUF/BUFG/MMCM stand-ins so board tops simulate. |
| `run_sim.sh`, `run_ctrl_sim.sh` | Compile and run the testbenches; non-zero exit on any failure. |
| `vectors/pattern_*.mem` | Golden pattern vectors (generated). |

### `fpga/vivado/`, `fpga/bitstreams/`, `fpga/upstream/`

| File | Purpose |
|---|---|
| `vivado/build.tcl` | Non-project batch build for every target (`make bit-headstage`, …). |
| `vivado/program.tcl` | Loads bitstreams over JTAG, matching boards by FPGA part. |
| `bitstreams/README.md` | Which bitstream the bench runs, its source commit, timing and SHA-1. |
| `upstream/rhd2164-emulator-rtl/*.sv` | Vendored RHD2164 emulator (stimulus and the on-die loopback); never edited. |
| `upstream/rhd2164-emulator-mem/chip*_[AB].mem` | The emulator's playback data, also the golden reference for emulator runs. |

### `esp/` — the ESP32-C5 / S3 node (ESP-IDF)

| File | Purpose |
|---|---|
| `CMakeLists.txt` | Top-level ESP-IDF project. |
| `sdkconfig.defaults` | Throughput settings shared by every build (240 MHz, lwIP mailboxes, Wi-Fi buffers, A-MPDU, power save off). |
| `sdkconfig.defaults.tcp` | **Overlay for the demo node:** TCP egress, PSRAM spool, Wi-Fi buffers internal, 32 kB TCP send buffer. |
| `README.md` | Build variants, the C5 pin map, link test, UDP vs TCP, bench learnings. |
| `main/CMakeLists.txt` | Component sources; the `BENCH_INGRESS` (synth/qspi) and `BENCH_WIFI_BAND` (5g/2g/auto) build switches. |
| `main/Kconfig.projbuild` | menuconfig: SSID, password, host IP, synthetic rate, TCP sink, spool depth, link test, pin scan. |
| `main/app_main.c` | Start-up order: NVS, banner, Wi-Fi, pipe, sink, echo, source. |
| `main/wifi.c`, `wifi.h` | Station bring-up: band selection, HT40 request, power save off, link report. |
| `main/source_qspi.c` | QSPI slave-HD ingress with DMA into pipe buffers, ready-line control, idle diagnostics, optional pin scan. |
| `main/source_synth.c` | Synthetic ingress: byte-identical datagrams made on the ESP (measures the radio alone). |
| `main/pipe.c`, `pipe.h` | The 16-buffer DMA pool and its two queues; the single throughput accounting point; the 1 Hz report. |
| `main/sink_tcp.c` | **TCP egress with a PSRAM spool:** spool task + sender task, reconnects, 1 Hz `tcp:` report. |
| `main/sink_udp.c` | UDP egress: trailer, `sendto()`, count. |
| `main/sink_check.c` | Link-test egress: checks every datagram's headers on the node with the radio off. |
| `main/echo.c` | UDP echo on port 5002 for air-latency and clock-offset probes. |
| `main/pattern.h` | The test pattern in plain C (synthetic source). |
| `main/chain_contract.h` | Generated from `CONTRACTS.md`. |
| `main/target.h`, `pins_c5.h`, `pins_s3.h` | Per-chip differences: pin maps, core assignment, register names. |

### `host/` — the Mac side (Python)

| File | Purpose |
|---|---|
| `README.md` | Module list, how to run sweeps, bring-up and the viewer, how to read the numbers. |
| `requirements.txt` | pyserial, pytest. |
| `requirements-viewer.txt` | Plus numpy and matplotlib for the viewer. |
| `chain/contract.py` | Generated from `CONTRACTS.md`. |
| `chain/frame.py` | Frame and datagram encode/decode. |
| `chain/pattern.py` | The golden test pattern. |
| `chain/emulator.py` | The emulator's playback memory as a golden reference. |
| `chain/ctrl.py` | UART text protocol client, plus `FakeSerial` for tests. |
| `chain/receiver.py` | UDP capture and the echo probe. |
| `chain/metrics.py` | Throughput, loss, accuracy and latency from a capture. |
| `chain/sweep.py` | Measurement loop CLI: set a rate, capture, score, write JSON. |
| `chain/report.py` | Tabulates `docs/results/*.json`. |
| `chain/rhd2164.py` | Chip knowledge: command words, ROM, Rhythm's register recipe, MISO tap decode. |
| `chain/diag.py` | Drives the FPGA's diagnostic list; host-side phase scan and status readers. |
| `chain/bringup.py` | Staged bring-up S0–S7 with a cause and a check for every failure; JSON report. |
| `chain/fakeboard.py` | The headstage FPGA build and two chip models in memory, with ~25 injectable faults (for tests and `--fake`). |
| `chain/viewer.py` | The live viewer: sources (UDP, TCP, UART snapshots), loss accounting, the single-panel scrolling window and its keys, headless statistics. |
| `chain/scroll.py` | The viewer's numpy core: 60 s history ring, per-pixel min/max decimation pinned to time, gap breaks, time and lane navigation, lane rasterizer. No display needed; unit-tested. |
| `tests/*.py` | pytest suite for every module above, no hardware needed. |
