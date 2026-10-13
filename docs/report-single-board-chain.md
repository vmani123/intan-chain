# intan-chain: single-board bench report

**Status, 2026-10-13.** The basic, uncompressed chain works end to end on real
hardware: an emulated pair of Intan RHD2164 chips, a receiver that reads them
over SPI, a quad-SPI link to an ESP32-S3, Wi-Fi, and a Mac that scores every
sample. 180,629 frames (370 Mbit of sample data) crossed the chain overnight
with **zero bit errors**, and with no loss at 2 and 4 kS/s per channel. The
first ceiling is the Wi-Fi link, somewhere between 4 and 8 kS/s per channel on
the network used.

This report covers the design, how it was verified, what was measured, what had
to be fixed or tuned on the bench, what limits performance, and what is still
open. The night-by-night record is in [bench-log-2026-10-13.md](bench-log-2026-10-13.md);
every number below comes from a result file in [results/](results/).

---

## 1. What the experiment is for

The rig exists to produce three honest numbers for one acquisition pipeline:
**throughput**, **latency** and **accuracy**. It deliberately leaves things out
(no compression yet, no real electrodes) so that each number can be traced to
one part of the chain.

```
 RHD2164 emulator ──SPI──► receiver ──QSPI──► ESP32-S3 ──Wi-Fi UDP──► Mac
 (2 chips, 128 ch)   jumpers  (FPGA)   jumpers   (relay)    2.4 GHz     (scoring)
 \________________ one Arty S7-25 FPGA board ______________/
```

Two design decisions make the numbers trustworthy:

- **The FPGA builds the finished UDP payload.** Five frames, 1380 bytes, go to
  the ESP in one QSPI transfer. The ESP appends a 12-byte trailer and calls
  `sendto()`. It never parses, reframes or allocates. So the throughput cannot
  be credited to, or blamed on, clever ESP code.
- **Every byte carries its own evidence.** Each frame carries two FPGA clock
  stamps, and each datagram carries two ESP clock stamps. Three of the four
  latency legs are therefore exact. Accuracy is scored against a reference the
  Mac computes independently: a closed-form test pattern, or the emulator's own
  playback memory.

## 2. Hardware and tools

| Part | What is used |
|---|---|
| FPGA | Digilent Arty S7-25 (xc7s25csga324-1), emulator and receiver in one bitstream |
| ESP | ESP32-S3 DevKitC-1 clone (CH343 USB bridge, 8 MB flash and 8 MB PSRAM) |
| Wiring | 5 jumpers from Pmod JA to JB (the SPI loop); 7 signal jumpers plus GND from Pmod JC to the ESP (QSPI) |
| Network | Home router, 2.4 GHz channel 1, WPA3, ESP at -55 dBm; the Mac on the same network |
| FPGA build | Vivado 2023.2, batch flow (`make bit-chain`), on the CMU ECE servers |
| FPGA programming | `openFPGALoader -b arty_s7_25` from the Mac (volatile: reload after power-off) |
| ESP build | ESP-IDF v6.0-dev on the Mac; CI also builds with v5.4 |
| Host | Python 3, standard library plus pyserial |

The project started as a two-board design: the receiver on an Arty S7-50 or a
RealDigital Boolean, the emulator on the S7-25. When the larger boards became
unavailable, both designs moved onto the S7-25. Separately they used about
2,100 LUTs and 18 of 45 block RAMs, a small fraction of the S7-25, so they fit
together easily. The two halves still talk only through pins and jumper wire,
so the SPI bus crosses real pads, as it will with a real headstage.

## 3. Architecture

### 3.1 The contract

`CONTRACTS.md` defines every byte the three implementations exchange: the
frame, the datagram, the test pattern and the control registers.
`tools/gen_contract.py` generates `esp/main/chain_contract.h` and
`host/chain/contract.py` from it, and CI fails if they are stale. Drift between
three hand-written copies of a byte layout is the most common way a chain like
this produces plausible-looking garbage.

**Frame** (276 bytes, little-endian): a 20-byte header (`magic 0xA55A`, type,
flags, `seq`, `t_cap`, `t_tx`, `n_ch`, `payload_len`), then 128 int16 samples.
Channels 0-31 are chip 0 module A, 32-63 chip 0 module B, 64-95 chip 1 A and
96-127 chip 1 B.

**Datagram** (1392 bytes, under one MTU): 5 frames plus a 12-byte ESP trailer
(`esp_rx_us`, `esp_tx_us`, `pipe_depth`).

### 3.2 FPGA: the emulator

Two unmodified RHD2164 emulator cores (`fpga/upstream/rhd2164-emulator-rtl/`)
share one SPI bus, each with its own MISO line, as two real chips would.

- **Front end.** `spi_frontend` oversamples the incoming CS, SCLK and MOSI on a
  fast clock: 192 MHz, so 8 samples per bit at the 24 MHz SCLK.
- **Command handling.** `command_decoder` and `register_file` implement the
  chip's command set: CONVERT, READ/WRITE register, CALIBRATE and its dummy
  window.
- **Data.** Samples are played back from memory, 256 samples × 32 channels per
  module (`fpga/upstream/rhd2164-emulator-mem/*.mem`). The playback pointer
  advances once per sampling sweep.
- **Output.** `ddr_miso` puts module A on the rising half of each SCLK and
  module B on the falling half, which is the RHD2164's double-data-rate MISO.
  MISO is registered in the pad's output flop, so its timing does not depend
  on placement.

### 3.3 FPGA: the receiver

Board-independent `rx_top` at 96 MHz, with no vendor primitives, so the whole
receiver simulates in Icarus Verilog:

| Module | Job |
|---|---|
| `spi_master` | One sample = 35 commands (32 CONVERT plus 3 AUX), 16 SCLK bits each, SCLK = 96/4 = 24 MHz (the RHD2164's maximum). MISO is captured 4× oversampled. One sweep takes ~2800 clocks. |
| `miso_phase_select` | Picks the 16 data bits from the oversampled MISO at a runtime phase (`PHASE_SELECT` 0-11, quarter-SCLK steps), separately for the rising and falling halves. This absorbs cable and pad delay, as in Intan's Rhythm. |
| `pattern_gen` | The alternative source (`MODE` bit 2): a 16-bit LFSR per channel, which the host can recompute exactly. |
| `frame_builder` | Writes header plus 128 samples into a datagram slot; stamps `t_cap`. |
| `dgram_mem` + `slot_fifo` | 4 datagram slots. A full FIFO drops the datagram and counts `OVERRUNS`. |
| `qspi_master` | Sends one slot per datagram in the ESP's slave-HD protocol (§3.5), waits on `esp_ready`, counts `STALLS` and patches `t_tx` live as each frame leaves. |
| `ctrl_regs` + `uart_rx/tx` | Text register interface on the board's USB serial port (`R aa` / `W aa vvvvvvvv`, 115200 baud). |

**Board top** (`fpga/boards/chain_arty_s7_25_top.sv`): one clock generator
(VCO 960 MHz = 100/5 × 48) makes 96 MHz for the receiver and 192 MHz for the
emulator, and each domain has its own reset. LED LD5 lights when the emulator
sees chip-select on JB, which makes a missing jumper visible.

### 3.4 ESP32-S3 firmware

A deliberately small relay (`esp/main/`). Source and sink run on different
cores, with a buffer pipe between them:

```
 source_qspi.c ──► pipe: 16 × 1392 B DMA buffers ──► sink_udp.c ──► Wi-Fi
 (core 1, prio 8)    (free and filled queues)        (core 0, prio 6)
 echo.c   UDP 5002, round-trip probe (core 0, prio 2)
 pipe.c   1 Hz report (core 0, prio 1)
```

- **`source_qspi.c`** runs the SPI2 peripheral as a half-duplex segment-mode
  slave with DMA. It keeps 4 receive buffers armed and drives `esp_ready`
  (GPIO21) high whenever at least one is armed.
- **`source_synth.c`** is the control arm. It generates the same CHECK pattern
  on the ESP itself, so the Wi-Fi path can be measured with no FPGA at all.
  Both sources share the pipe, sink, sockets, task placement and priorities;
  only the ingress differs.
- **`sink_udp.c`** stamps `esp_tx_us`, appends the trailer, calls `sendto()`,
  and counts bytes in exactly one place.
- **`sdkconfig.defaults`** sets the throughput options: 240 MHz CPU, deep lwIP
  and Wi-Fi buffer pools, an A-MPDU window, and Wi-Fi power save off.

### 3.5 The QSPI link

The ESP's slave hardware fixes the protocol. It was read out of the ESP-IDF
source: `hal/esp32s3/.../spi_ll.h`, `spi_slave_hd_hal.c`, and Espressif's own
master driver `essl_spi.c`. One datagram is two transactions, SPI mode 0:

```
WRDMA    CS↓  0x23 on D0 (8 clk)  0x00 on D0 (8 clk)  8 dummy clk  1380 B on D0-D3  CS↑
WR_DONE  CS↓  0x27 on D0 (8 clk)  0x00 on D0 (8 clk)  8 dummy clk                    CS↑
then CS stays high for 10 µs while the ESP's interrupt routine re-arms its DMA
```

The receive completes only on WR_DONE; CS rising after the data does nothing.
`fpga/sim/qspi_hd_monitor.sv` models the slave this way, and both FPGA
testbenches check the link against it.

### 3.6 The Mac side

The `host/chain/` Python package:

| Module | Job |
|---|---|
| `receiver` | Timestamps UDP datagrams into a list. No parsing during capture; an 8 MB socket buffer. |
| `frame` | Parses and builds frames and datagrams. |
| `pattern` | The golden LFSR pattern; `check_index` recovers a sample's index from its values. |
| `emulator` | The emulator's playback memory, as the golden reference for real-SPI runs. |
| `metrics` | Throughput, loss (missing, reordered, duplicate, stale), accuracy (bit errors against the golden source, with a per-run offset) and latency legs. |
| `ctrl` | The FPGA's register interface over serial. |
| `sweep` | The measurement loop CLI: configure, capture, score and save one JSON per cell. Supports `--source pattern\|emulator`, `--phase-scan` and `--listen-only`. Tonight's hardware runs used the same capture and scoring code from a script; the sweep CLI's new options are tested in dry runs only. |
| `report` | Renders `docs/results/` as a table. |

## 4. How it was verified before hardware

| Layer | Check |
|---|---|
| Contract | Generated files must match `CONTRACTS.md` (CI) |
| Host | 50 pytest tests: framing, pattern, metrics, receiver over loopback, control protocol, sweep dry runs |
| FPGA | 6 Icarus testbenches. The receiver runs closed-loop against the real emulator RTL, recovering all 128 channels (`tb_rx_top`, 46 checks). The whole board top runs with the clock generator, resets and modelled jumpers (`tb_chain_arty_s7_25`, 14 checks). |
| ESP | Builds clean for both variants on ESP-IDF v6.0-dev (local) and v5.4 (CI) |

The ESP slave model proved its worth. Against the original FPGA code it
received nothing, exactly as the real ESP would have, because no WR_DONE was
ever sent. And a deliberate one-cycle dummy error corrupts every datagram.

## 5. What was measured

All runs use the CHECK pattern or the emulator's memory as the reference, so
"0 bit errors" means every sample matched bit for bit.

### 5.1 ESP alone (synthetic source, no FPGA)

| Offered | Delivered | Loss | Bit errors | ESP queue latency (median / p99) |
|---|---|---|---|---|
| 2 kS/s/ch | 4.09 Mbit/s | 0 of 19,990 frames | 0 of 40.9 M | 15 / 367 µs |
| 20 kS/s/ch | **21.5 Mbit/s** (10.5 kS/s/ch) | 0.8 % | 0 of 214.8 M | 5.8 / 11.3 ms |

Throughput figures here and below count sample payload only (256 bytes per
frame). On the wire each kS/s/ch costs 2.23 Mbit/s with headers and trailer.

### 5.2 Full chain, emulator through the jumpers (QSPI at 8 MHz)

| Rate | PHASE_SELECT | Frames | Missing | Bit errors |
|---|---|---|---|---|
| 2 kS/s | 5 | 20,028 | 0 | 0 of 41.0 M |
| 2 kS/s | 6 | 20,005 | 0 | 0 of 41.0 M |
| 4 kS/s | 5 | 40,000 | 0 | 0 of 81.9 M |
| 8 kS/s | 5 | 62,600 | 16,270 (20.6 %) | 0 of 128.2 M |
| 2 kS/s, internal pattern (3 runs) | – | 21,861 | 0 | 0 of 44.8 M |

**Phase window.** PHASE_SELECT 5 and 6 recover all 128 channels on every
frame; 0-4 and 7-11 recover none. Simulation predicted 3-4, so the board adds
two quarter-SCLK steps, about 21 ns of pad and jumper delay.

### 5.3 Latency, per sample, at 2 kS/s

| Leg | Median | Tail | How measured |
|---|---|---|---|
| FPGA: sample captured → on the QSPI wire | 1.14 ms (0.28-2.0 ms by position) | fixed | exact (FPGA clock) |
| QSPI transfer | ~0.1 ms | fixed | from the 8 MHz clock |
| ESP: QSPI done → handed to Wi-Fi | 15 µs | p99 0.21 ms | exact (ESP clock) |
| Wi-Fi to the Mac | ~3.7 ms (min 2.3) | p99 ~17 ms | estimate: half a round trip, ESP-alone run |
| **Total** | **~5 ms** | see below | |

**End-to-end spread.** This compares Mac arrival time with FPGA capture time,
with clock drift removed, and measures above the fastest datagram. At 2 kS/s:
median +0.6 ms, p90 +20 ms, p99 +138 ms, worst +238 ms. At 4 kS/s: median
+1.7 ms, p99 +148 ms. The Mac saw delivery gaps of up to 237 ms with nothing
lost, so the tail lies after the ESP's handoff: in its Wi-Fi driver queue, the
air, the router, or the Mac's Wi-Fi.

## 6. What had to change on the bench

| Problem seen | Cause | Fix |
|---|---|---|
| Found before any hardware run: the ESP would have received nothing | The FPGA's QSPI format did not match the ESP slave: command on 4 lines, no dummy cycles, no WR_DONE | `qspi_master` rewritten to the slave-HD protocol (§3.5), with the slave model in simulation |
| On hardware: FPGA sent 4,837 datagrams, ESP received 0; slave flagged `cmd_err` on every transaction | The ESP's direct IOMUX pin route never decoded a command, although all six lines were live | Slave forced through the GPIO matrix. A new pin-scan diagnostic proved the wiring right and found this |
| At 8 kS/s the link froze after 24 datagrams | ESP receiver deadlock: an empty buffer pool with nothing armed never re-armed | Re-arm on a 2 ms timer while short of buffers |
| Every run after the first scored ~50 % bit errors on perfect data | The FPGA's pattern generator does not restart with `seq` | Host finds the per-run offset |
| One run reported ~4 billion lost frames | A partial datagram from the previous run arrived first | Host drops such frames and reports `stale_frames` |
| First datagrams of every run missing | Socket opened after the FPGA started | Socket opens before `run` |
| `--phase-scan` could not work | It used the internal pattern, where PHASE_SELECT does nothing | `--source emulator`, scored against the emulator's memory |
| `write_bitstream` failed (DRC AVAL-139) | MMCM multiplier 9.6 is off the 0.125 grid | 100/5 × 48 = 960 MHz VCO |
| Typing in `screen` "did nothing" | The board ends lines on LF only and never echoes | Press Ctrl-J (documented) |

## 7. What was tuned, and to what

| Knob | Value | Why |
|---|---|---|
| Receiver clock | 96 MHz | SCLK = 96/4 = 24 MHz, the RHD2164 maximum |
| Emulator oversampling clock | 192 MHz | 400 MHz missed timing on the -1 part by 1.55 ns; 200 MHz met it alone (+0.366 ns); 192 MHz shares the receiver's 960 MHz VCO |
| QSPI clock | 8 MHz (`QSPI_DIV=6`) | First light over jumpers through the Arty's 200 Ω JC resistors; 24 MHz (`QSPI_DIV=2`) still untested |
| Gap after WR_DONE | 10 µs | Time for the ESP's ISR to re-arm DMA; no loss attributable to it at 2-4 kS/s |
| PHASE_SELECT | 5 (window 5-6) | Measured with the phase scan |
| ESP pin routing | GPIO matrix | IOMUX did not decode on this bench |
| ESP armed receives / pipe | 4 / 16 buffers | Unchanged from design |
| Network | Home 2.4 GHz router | Phone hotspot too slow |
| Timing result | WNS +0.497 ns, WHS +0.060 ns | Single-board bitstream with `QSPI_DIV=6` |

## 8. What limits performance

In the order they bite in this configuration:

1. **Wi-Fi uplink.** About 21.5 Mbit/s of payload on this network, about
   10.5 kS/s/ch with 0.8 % loss even from the ESP alone. The chain lost 20 % at
   8 kS/s. Clean ceiling here: between 4 and 8 kS/s/ch. 128 channels at
   30 kS/s would need ~67 Mbit/s on the wire, beyond what this link delivers.
   That is the case for the codec.
2. **ESP buffering versus Wi-Fi stalls.** 16 buffers hold 40 ms at 2 kS/s but
   only 10 ms at 8 kS/s. The Wi-Fi stalls seen were up to ~240 ms. Beyond the
   rate the radio sustains, back-pressure reaches the FPGA (`STALLS`), then its
   4-slot FIFO overflows (`OVERRUNS`).
3. **QSPI clock.** One datagram is about 34,700 FPGA clocks at `QSPI_DIV=6`,
   which caps the rate at ~13.8 kS/s/ch. At `QSPI_DIV=2` it is ~12,200 clocks,
   ~39 kS/s/ch.
4. **RHD2164 sequencer.** One 35-command sweep takes ~2800 clocks, about
   34 kS/s/ch. That becomes the cap at `QSPI_DIV=2`.
5. **Datagram fill latency.** With 5 frames per datagram, the first sample
   waits up to 4 sample periods (2 ms at 2 kS/s). Fewer frames per datagram
   would trade this for more packets per second.
6. **Physical link.** ~15 cm jumpers and 200 Ω series resistors on JC; the
   ESP's GPIO-matrix route is specified lower than IOMUX. Both are fine at
   8 MHz; 24 MHz is unproven.
7. **Wi-Fi latency tail.** The median is a few ms, but p99 is ~140 ms and the
   worst ~240 ms. That matters for closed-loop use more than for recording.

## 9. Open issues and questions

1. **Why the ESP's IOMUX route fails.** ESP-IDF says IOMUX and the GPIO matrix
   behave the same below 80 MHz. Leading guess: ringing on SCLK or CS over the
   jumpers. Next step: scope GPIO12 and GPIO10 at the ESP. A cheap FPGA-side
   experiment is `SLEW SLOW` / `DRIVE 4` on `qspi_sclk`.
2. **ESP ready-line race under saturation.** The 8 kS/s run delivered one
   datagram twice. The ESP received the two copies 13.7 ms apart, but their
   FPGA `t_tx` stamps are identical, so the FPGA sent it once. Likely
   mechanism: the ESP lowers `esp_ready` from task context, so just after its
   last armed buffer fills, the FPGA can still see "ready" and send a segment
   into no buffer. The next buffer armed then completes at once with stale
   data. Proposed fix, already noted in `drive_ready()`: drive the ready line
   from the driver's receive callbacks (in interrupt context).
3. **Where the 8 kS/s loss happens.** The FPGA handed 15,595 datagrams to QSPI
   and the Mac received 12,520. How much was lost in the air and how much at
   the ESP's input (issue 2) is not separated. Capturing the ESP log during the
   run would split it.
4. **24 MHz QSPI** (`QSPI_DIV=2`) has not been built or tried.
5. **Wi-Fi latency under load** has not been measured: the round-trip probe did
   not run during chain runs. Rerunning with `--target <esp-ip>` gives it.
6. **FPGA behaviour the host works around** (needs synthesis access to change):
   - `clear_counters` does not restart the pattern generator or the emulator's
     playback pointer. Either reset them, or define the index as free-running
     in `CONTRACTS.md`.
   - A partly built datagram survives a stop/start. Flush it when `run` falls.
7. **ESP once-per-second log.** It divides by a nominal second, so its rates
   are wrong when the reporting task is starved. Use the host's numbers.
8. **IDF versions.** The firmware has only run when built with ESP-IDF
   v6.0-dev; CI builds v5.4 but nothing built with v5.4 has run on hardware.
9. **Compression** is not implemented. `CONTRACTS.md` defines compression per
   frame (one time sample), while the existing codec (in `hdemg-bench`)
   compresses 256-sample blocks per channel. The contract needs a decision
   before a codec goes in.
10. **Shared ESP32-S3.** Another study uses the same board. A lock directory
    (`~/.esp32s3-usbmodem5B8F0676581.lock`) and a quiet-air rule coordinate the
    two; see the bench log.

## 10. Reproducing

```bash
make check                                   # contract + host tests + 6 FPGA testbenches

# FPGA (on a machine with Vivado), then program from the Mac
QSPI_DIV=6 make bit-chain                    # -> fpga/bitstreams/chain_arty_s7_25_div6.bit
openFPGALoader -b arty_s7_25 fpga/bitstreams/chain_arty_s7_25_div6.bit

# ESP (Wi-Fi name, password and Mac IP in menuconfig; never committed)
cd esp && idf.py -B build/qspi -DBENCH_INGRESS=qspi -p <esp-port> menuconfig build flash

# Mac, from host/
python3 -m chain.sweep --serial /dev/cu.usbserial-<arty>1 --phase-scan --phase-rate 2000
python3 -m chain.sweep --serial /dev/cu.usbserial-<arty>1 --source emulator \
                       --rates 2000 4000 --seconds 10 --target <esp-ip>
python3 -m chain.report
```

Wiring: JA1-4 and JA7 to the same positions on JB; JC1 → GPIO10, JC2 → GPIO12,
JC3 → GPIO11, JC4 → GPIO13, JC7 → GPIO14, JC8 → GPIO9, JC9 ← GPIO21, JC5 → GND.
Never connect 3V3 between boards. Full pin tables are in the README.
