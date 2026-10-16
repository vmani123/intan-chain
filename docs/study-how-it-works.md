# Intan Chain: how the FPGA → ESP32 → Mac bench works

*2026-10-12. A walkthrough of the code, written for a human reader. The measured
results and the full bench story are in
[report-single-board-chain.md](report-single-board-chain.md); this document
explains how the pieces fit and what each file does. The diagrams in the
original page (five of them) are reduced to text figures here.*

## What the study measures

The bench tests one question: can an ESP32-S3 carry 128 channels of Intan
RHD2164 data over Wi-Fi to a computer, and up to what sample rate does the data
stay complete and bit-exact? It answers with four numbers per run:
**throughput, loss, accuracy and latency**.

The chain has five stages. Two emulated RHD2164 chips (64 channels each) stand
in for a real headstage. An FPGA receiver reads them over SPI exactly as an
Intan controller would. It sends the data to the ESP32-S3 over a 4-bit QSPI
bus, and the ESP forwards it over Wi-Fi as UDP to a Mac. Both FPGA designs
share one Digilent Arty S7-25 board.

```
  Arty S7-25 (one FPGA, two independent designs)
  ┌──────────────────────────────────────────────┐
  │  2 x RHD2164 emulator   192 MHz              │
  │     ▲ miso0, miso1   │ cs, sclk, mosi        │
  │     │  SPI 24 MHz, 5 jumpers JA <-> JB       │
  │     │                ▼                       │
  │  FPGA receiver          96 MHz               │
  └───────────┬──────────────────────────────────┘
              │ QSPI, 4 lines at 8 MHz, Pmod JC   ▲ esp_ready (GPIO21 -> JC9)
              ▼                                   │
        ESP32-S3 relay  (DMA into a buffer, add 12-byte trailer, sendto)
              │ Wi-Fi UDP :5001 (1392-byte datagrams); echo probes :5002
              ▼
        Mac, host/chain (Python)  ── USB-UART 115200 baud ──► FPGA (set/read registers)
```

One design decision shapes everything: **the FPGA builds the finished UDP
payload** (5 frames, 1380 bytes), and the ESP only appends a 12-byte trailer
and calls `sendto`. The ESP never parses or reframes data, so a throughput
limit cannot be blamed on clever ESP code. It is the radio or the bus.

| Measure | How it is measured | Where it is computed |
| --- | --- | --- |
| Throughput | Bytes and frames received per second at the Mac | `host/chain/metrics.py` |
| Loss | Gaps, duplicates and reordering in each frame's sequence number | `host/chain/metrics.py` |
| Accuracy | Every sample compared bit-for-bit with a reference the Mac regenerates independently | `host/chain/metrics.py`, `pattern.py`, `emulator.py` |
| Latency | Two FPGA clock stamps per frame, two ESP stamps per datagram, and an echo probe for the Wi-Fi hop | FPGA, ESP and Mac together |

## The bench hardware

Two boards, each powered by its own USB cable from the Mac: a **Digilent Arty
S7-25** (Spartan-7, xc7s25csga324-1) carrying both the emulator and the
receiver, and an **ESP32-S3 DevKitC-1** clone (N8R8: 8 MB flash, 8 MB PSRAM).

**Clocks.** The Arty's 100 MHz oscillator feeds one MMCM (VCO 960 MHz) that
makes two clocks:

- **96 MHz** for the receiver. Its SPI clock is 96 / 4 = **24 MHz**, the
  RHD2164's maximum.
- **192 MHz** for the emulator, which samples the incoming SPI bus 8 times per
  SCLK period.

No signal crosses between the two clock domains inside the FPGA.

**Why jumper wires on one board.** The emulator and receiver never talk inside
the chip. The receiver's SPI leaves on Pmod JA, crosses five jumpers, and
enters the emulator on Pmod JB. So the bus goes through real pads and wire, and
the receiver has to compensate for a real cable delay, the same job it has
against a real headstage.

**Pmod choice.** On the Arty S7, JA and JB are the high-speed Pmods (0 Ω), so
they carry the 24 MHz SPI loop. JC is a standard Pmod (200 Ω series
resistors), fine for QSPI at 8 MHz.

| Link | Arty pin | Signal | Other end | Direction |
| --- | --- | --- | --- | --- |
| SPI loop | JA1 | cs_n | JB1 | receiver → emulator |
| SPI loop | JA2 | sclk | JB2 | receiver → emulator |
| SPI loop | JA3 | mosi | JB3 | receiver → emulator |
| SPI loop | JA4 | miso0 (chip 0) | JB4 | emulator → receiver |
| SPI loop | JA7 | miso1 (chip 1) | JB7 | emulator → receiver |
| QSPI | JC1 | qspi_cs_n | ESP GPIO10 | FPGA → ESP |
| QSPI | JC2 | qspi_sclk | ESP GPIO12 | FPGA → ESP |
| QSPI | JC3 | qspi_dq0 | ESP GPIO11 | FPGA → ESP |
| QSPI | JC4 | qspi_dq1 | ESP GPIO13 | FPGA → ESP |
| QSPI | JC7 | qspi_dq2 | ESP GPIO14 | FPGA → ESP |
| QSPI | JC8 | qspi_dq3 | ESP GPIO9 | FPGA → ESP |
| Ready line | JC9 | esp_ready | ESP GPIO21 | ESP → FPGA |
| Ground | JC5 or JC11 | GND | ESP GND | required |

The SPI loop needs no ground jumper because it never leaves the board; the ESP
link does. **Never connect a 3V3 pin (Pmod pins 6 and 12) between boards.**
`esp_ready` has a pull-down in the FPGA, so an unplugged ESP reads as "not
ready" and no data is sent into nothing.

**Reading Pmod JC.** Looking into the connector holes from outside the board
edge with the Arty lying flat, pin 1 is the **top-right** hole:

```
 top row     JC6 VCC | JC5 GND | JC4 | JC3 | JC2 | JC1     <- pin 1 end
 bottom row  JC12 VCC| JC11 GND| JC10| JC9 | JC8 | JC7
 (the power end is the one the board silkscreen marks "GND VCC")
```

On the ESP (antenna up, USB ports down) the left header runs GPIO9, 10, 11,
12, 13, 14 going down, directly above 5V and G. GPIO21 is the 18th pin of the
right header. Source: Digilent Arty S7 reference manual, Fig. 8.1, and
Espressif's DevKitC-1 v1.1 pin layout.

Four LEDs show state: LD2 receiver running, LD3 heartbeat, LD4 QSPI activity,
LD5 emulator sees chip select (dark while LD2 is lit means a JA/JB jumper is
off). Pin assignments live in `fpga/boards/chain_arty_s7_25.xdc`.

## FPGA code, part 1: the RHD2164 emulator

The emulator makes FPGA logic behave like an Intan RHD2164 on the SPI bus,
replaying stored data instead of digitizing electrodes. It is vendored code in
`fpga/upstream/rhd2164-emulator-rtl/` and is never edited here. The board top
instantiates it **twice** (chip 0 and chip 1). The two copies share cs, sclk
and mosi, and each drives its own MISO line.

Each chip has two 32-channel halves, **module A** (channels 0–31) and
**module B** (channels 32–63), so two chips give 128 channels.

| File | What it does |
| --- | --- |
| `rhd2164_emulator.sv` | One chip. Wires the four blocks below to two channel memories (A and B). |
| `spi_frontend.sv` | Passes CS, SCLK and MOSI through 3-flop synchronizers into the 192 MHz clock, turns their edges into one-cycle strobes, and assembles each 16-bit command, MSB first. |
| `command_decoder.sv` | Decodes the top two bits: 00 CONVERT, 01 CALIBRATE/CLEAR, 10 WRITE, 11 READ. Reproduces the RHD2000 pipeline, where a command's result comes out **two CS cycles later**. |
| `register_file.sv` | The chip's 22 RAM registers plus the ROM registers: "INTAN" at 40–44, the A/B marker at 59, identity at 60–63. |
| `ddr_miso.sv` | Puts both halves on one wire: module A's bits are launched on SCLK rising edges, module B's on falling edges (double data rate). |
| `rhd2164_top.sv`, `rhd2164_top_se.sv` | Stand-alone emulator boards (differential and single-ended). Not used on this bench; `chain_arty_s7_25_top.sv` instantiates the emulator core directly. |
| `../rhd2164-emulator-mem/chip{0,1}_{A,B}.mem` | The "ADC data": per half, 256 time samples × 32 channels of 16-bit two's-complement values, loaded at build time. |

One sampled channel goes like this:

1. The receiver pulls CS low and clocks a 16-bit CONVERT(c) command out on MOSI at 24 MHz.
2. The emulator samples the bus 8 times per SCLK period and recovers the command.
3. It reads channel c of the current time sample from memories A and B. CONVERT(0) advances the playback pointer, so the chip loops through its 256-sample recording.
4. Two CS cycles later, the A and B values stream out together on MISO, A on rising edges and B on falling edges.
5. The board top re-registers MISO in the output-pad flip-flop, so its delay is short and fixed whatever the placement.

The emulator returns its recorded `.mem` data, **not** the LFSR test pattern.
The simulations check emulator data against the `.mem` files directly; the Mac
does the same through `chain/emulator.py` when a run uses `--source emulator`.

## FPGA code, part 2: the receiver

The receiver (`fpga/rtl/rx/`, top module `rx_top.sv`) reads 128 channels over
SPI and turns them into finished UDP payloads, which it pushes to the ESP over
QSPI. `rx_top` takes an already-made 96 MHz clock and uses no Xilinx
primitives, so the whole receiver simulates in plain Icarus Verilog. The MMCM,
pads and LEDs live in the board wrapper `fpga/boards/chain_arty_s7_25_top.sv`.

```
 sample timer ─► spi_master (24 MHz SPI, miso_phase_select) ──┐
  (start every SAMPLE_PERIOD)                                 ├─► frame_builder ─► dgram_mem + slot_fifo ─► qspi_master ─► JC
              └► pattern_gen (LFSR test pattern) ─────────────┘   (header + 128      (4 slots)               (WRDMA + WR_DONE,
                    MODE bit 2 picks one source                    values, t_cap)                             waits for esp_ready,
 uart_rx/uart_tx ◄─► ctrl_regs (settings, counters)                                                           stamps t_tx)
 96 MHz tick counter -> t_cap (frame_builder) and t_tx (qspi_master)
```

| Stage | Module | What it does |
| --- | --- | --- |
| Timing | sample timer (inside `rx_top`) | Fires a `start` every `SAMPLE_PERIOD` clocks. If the source is still busy, it counts an **overrun** instead of quietly slowing down. |
| Source | `spi_master.sv` | One sample = 35 SPI commands (32 CONVERT + 3 AUX), 16 bits each at 24 MHz. Records MISO on every 96 MHz tick, 4 per bit. A command's result arrives two commands later, so result c is credited to command c−2. |
| Source | `miso_phase_select.sv` | From the 4× oversampled MISO, keeps one of the 4 samples per bit. `PHASE_SELECT` (0–11) shifts that choice in quarter-bit steps to cancel cable delay. The same shift register, offset by half a bit, separates module A from module B. Intan's Rhythm mechanism. |
| Source (alt.) | `../gen/pattern_gen.sv` | The LFSR test pattern (CHECK or REALISTIC), chosen with `MODE` bit 2, so QSPI and Wi-Fi can be tested with SPI out of the way. |
| Framing | `frame_builder.sv` | Writes a 20-byte header and 128 int16 values per sample into a slot, and latches `t_cap`. Each value goes to its channel's position, because SPI delivers channels in the order 0, 32, 64, 96, 1, 33, … Five frames make one datagram (1380 bytes). |
| Buffer | `dgram_mem.sv` + `slot_fifo.sv` | 4 slots of one datagram each. If all 4 are full, the next whole datagram is dropped: `seq` still advances so the Mac sees the gap, and `OVERRUNS` counts it. |
| Egress | `qspi_master.sv` | Sends one slot per QSPI transfer, only while `esp_ready` is high. As each frame goes out, overwrites its `t_tx` field with the live clock. |
| Control | `uart_rx.sv`, `uart_tx.sv`, `ctrl_regs.sv` | 115200-baud text commands from the Mac set and read the registers below. |

**One QSPI datagram** is two transactions in the ESP32-S3's "slave
half-duplex" protocol, which is fixed by Espressif's silicon:

1. **WRDMA.** CS low; command `0x23` on dq0; address `0x00` on dq0; 8 dummy clocks; 1380 bytes on dq0–dq3, high nibble first (2760 clocks); CS high.
2. **WR_DONE.** After an 8-clock gap: CS low; command `0x27`, address and 8 dummy clocks; CS high. Without it the ESP never delivers the data.
3. **Re-arm gap.** CS stays high for 10 µs so the ESP's interrupt can queue its next DMA buffer.

At `QSPI_DIV=6` (8 MHz SCLK) a datagram takes about 34,700 clocks, which caps
the link near **13.8 kS/s per channel**. At `QSPI_DIV=2` (24 MHz) the cap is
about 39 kS/s, and the SPI sweep (2800 clocks, about 34 kS/s) becomes the limit.

**Control registers.** The Mac sends `R aa` to read and `W aa vvvvvvvv` to
write, in hex; the board answers `aa=vvvvvvvv`, `ok` or `err`. Lines must end in
LF: in `screen`, press Ctrl-J, because Enter sends only CR, which the board
ignores.

| Address | Register | Meaning |
| --- | --- | --- |
| `00` | CTRL | bit 0 run, bit 1 clear counters |
| `01` | SAMPLE_PERIOD | 96 MHz clocks per sample; 48,000 gives 2 kS/s |
| `02` | PHASE_SELECT | MISO sampling phase, 0–11 |
| `03` | MODE | bit 0 REALISTIC pattern, bit 1 codec (not written yet), bit 2 internal pattern instead of SPI |
| `10` | FRAMES | frames built since the last clear |
| `11` | DGRAMS | datagrams sent over QSPI |
| `12` | FIFO_HIGH | most slots ever full at once |
| `13` | OVERRUNS | samples or datagrams dropped |
| `14` | STALLS | clocks spent waiting for `esp_ready` |
| `1F` | ID | `1c100001`; proves which bitstream is loaded |

## The data contract: what the bytes look like

The FPGA, the ESP and the Mac are three separate implementations that must
agree on every byte, so the format is defined once, in `CONTRACTS.md`.
`tools/gen_contract.py` generates the ESP header `esp/main/chain_contract.h` and
the Python module `host/chain/contract.py` from it. `make check-contract`,
which CI also runs, fails if either is out of date. All multi-byte fields are
little-endian.

```
 datagram, 1392 bytes
 ┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
 │ frame 0 │ frame 1 │ frame 2 │ frame 3 │ frame 4 │ trailer │
 │  276 B  │  276 B  │  276 B  │  276 B  │  276 B  │  12 B   │
 └─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
 └──────── 1380 B, built by the FPGA ────────────────┘ └ added by the ESP
 frame   = 20-byte header + 128 x int16 (chip0 A,B; chip1 A,B)
 trailer = esp_rx_us, esp_tx_us, pipe_depth (4 B each)
```

**Frame** = one time sample of all 128 channels: a 20-byte header plus
128 × int16 = **276 bytes**.

| Offset | Bytes | Field | Meaning |
| --- | --- | --- | --- |
| 0 | 2 | `magic` | `0xA55A`, marks a valid frame |
| 2 | 1 | `type` | 0 = RAW16, 2 = COMPRESSED (codec not written yet) |
| 3 | 1 | `flags` | bit 0 REALISTIC pattern, bit 1 codec on |
| 4 | 4 | `seq` | frame counter from 0; used to find loss and reordering |
| 8 | 4 | `t_cap` | FPGA clock tick when the sample finished |
| 12 | 4 | `t_tx` | FPGA clock tick when the frame left on QSPI |
| 16 | 2 | `n_ch` | channels in the payload (128) |
| 18 | 2 | `payload_len` | payload bytes (256) |
| 20 | 256 | payload | channel 0–127: chip 0 A, chip 0 B, chip 1 A, chip 1 B |

`t_cap` and `t_tx` come from the same free-running 96 MHz counter (10.417 ns per
tick, wrapping every 44.7 s), so `t_tx − t_cap` is the exact time the frame
spent inside the FPGA.

**Datagram** = 5 frames (1380 bytes, built by the FPGA) + a 12-byte trailer
(added by the ESP) = **1392 bytes**. That fits in one 1472-byte UDP payload, so
the IP layer never fragments it.

| Trailer offset | Bytes | Field | Meaning |
| --- | --- | --- | --- |
| 0 | 4 | `esp_rx_us` | ESP microsecond clock when the QSPI transfer completed |
| 4 | 4 | `esp_tx_us` | ESP microsecond clock when the datagram was handed to the socket |
| 8 | 4 | `pipe_depth` | bytes waiting in the ESP's buffer pipe at that moment |

**Test pattern.** Every channel has its own 16-bit LFSR, seeded with
`channel + 1` and advanced once per sample:
`next(s) = (s >> 1) ^ (0xB400 if s is odd else 0)`. In CHECK mode the sample
value is the LFSR state itself, so any flipped bit shows up. In REALISTIC mode
it is a triangle wave plus a little LFSR noise, for testing compression later.
Three copies of the pattern exist: `fpga/rtl/gen/pattern_gen.sv`,
`esp/main/pattern.h` (for the ESP's synthetic source) and
`host/chain/pattern.py`. The contract pins check values (channel 0 reads `1`,
`0xB400`, `0x5A00` for samples 0, 1, 2), because a one-sample offset would look
like total corruption.

## ESP32-S3 firmware

The ESP firmware (`esp/`, an ESP-IDF project) is a deliberately simple relay.
It takes 1380 bytes in, writes a 12-byte trailer, and sends the result as one
UDP datagram. Nothing in the data path allocates memory or looks inside a
frame.

**Two builds from one source tree.** The ingress is chosen at build time, and
everything downstream of it is shared:

- `idf.py -DBENCH_INGRESS=synth build`: the ESP generates the test pattern itself. With no FPGA attached, this measures the Wi-Fi ceiling alone.
- `idf.py -DBENCH_INGRESS=qspi build`: the data comes from the FPGA over QSPI.

Wi-Fi SSID and password, the Mac's IP, and the synthetic sample rate are set
with `idf.py menuconfig` (defined in `main/Kconfig.projbuild`). They are stored
in `esp/sdkconfig`, which git ignores, so credentials never reach the public
repo.

| File | What it does |
| --- | --- |
| `app_main.c` | Startup, in order: flash storage, a banner printing every contract size, Wi-Fi, pipe, sink, echo, and the source last, so nothing is generated before it has somewhere to go. Reboots if Wi-Fi fails. |
| `wifi.c` | Joins the access point as a station, with retries. |
| `pipe.c`, `pipe.h` | 16 buffers of 1392 bytes (22 kB of DMA-capable RAM), allocated once, passed between a **free** queue and a **filled** queue. Keeps counters and logs a report once a second. |
| `source_qspi.c` | Runs the SPI2 peripheral as a half-duplex slave in segment mode on GPIO 9–14. Keeps 4 receive buffers queued, holds GPIO21 (ready) high while at least one is armed, and drops any transfer that is not exactly 1380 bytes. |
| `source_synth.c` | Builds byte-identical datagrams from `pattern.h` at the configured rate. Its timestamps are ESP microseconds, so the FPGA-latency number means nothing in this mode. |
| `sink_udp.c` | Takes a filled buffer, writes the trailer, calls `sendto` to the Mac on port 5001, counts the bytes, and recycles the buffer. The **only** place throughput is counted. |
| `echo.c` | Bounces UDP packets on port 5002 back to the Mac, for the Wi-Fi latency probe. |
| `pins_s3.h` | Every pin number. GPIO 9–14 are SPI2's direct IOMUX pins, chosen for their direct 80 MHz route. On this bench that route never decoded a command, so the firmware sends them through the GPIO matrix instead (cause still open). |
| `chain_contract.h`, `pattern.h` | Sizes and ports generated from `CONTRACTS.md`, and the C copy of the test pattern. |

```
        QSPI from FPGA ─► source task ──pipe_submit──► filled queue
        ready (GPIO21) ◄─  core 1, prio 8                  │ pipe_take
              ▲                │ pipe_acquire              ▼
              │            free queue ◄──pipe_recycle── sink task ──► Wi-Fi UDP :5001
              │          16 x 1392 B, once at boot       core 0, prio 6
```

When Wi-Fi falls behind, buffers pile up in the filled queue, the free queue
empties, and the QSPI source cannot arm a new receive, so the ready line drops.

| Task | Core | Priority | Why there |
| --- | --- | --- | --- |
| `qspi` or `synth` source | 1 | 8 | Alone on core 1, so Wi-Fi activity cannot delay re-arming the QSPI receives |
| `sink` | 0 | 6 | Beside the Wi-Fi driver and lwIP, which are pinned to core 0 |
| `echo` | 0 | 2 | Low on purpose: the probe must not slow the data path, so its round-trip time is an upper bound |
| `pipe_rep` | 0 | 1 | Once-a-second log line |

**Back-pressure.** If Wi-Fi stalls, the filled queue grows and the source runs
out of free buffers. In QSPI mode the ready line then drops and the FPGA waits;
its `STALLS` counter shows the ESP, not the radio, was the limit. In synth mode
the source counts a drop instead.

## The Mac: conductor and scorekeeper

The Mac does not package the data: the FPGA has already built every packet, and
the ESP only adds a trailer. The Mac's Python package (`host/chain/`, standard
library plus pyserial) does two jobs. It **drives the FPGA** over the Arty's
USB-UART, and it **catches, unpacks and scores** the UDP stream arriving over
Wi-Fi.

The Mac has three connections. The Arty's USB cable gives JTAG for programming
and the 115200-baud control UART. Wi-Fi brings the data in on UDP port 5001 and
carries echo probes to the ESP on port 5002. The ESP's USB cable is for
flashing and its log.

| File | What it does |
| --- | --- |
| `contract.py` | Sizes, offsets, ports and register addresses, generated from `CONTRACTS.md`. |
| `pattern.py` | The reference test pattern, which must match the FPGA bit for bit. |
| `emulator.py` | The emulator's playback memory, the reference for real-SPI runs. |
| `frame.py` | Packs and unpacks frames and datagrams; rejects bad magic, sizes or lengths. |
| `ctrl.py` | The UART text protocol (`R aa`, `W aa vvvvvvvv`), plus `FakeSerial`, an in-memory board for tests. |
| `receiver.py` | UDP capture, and the echo probe that estimates Wi-Fi latency. |
| `metrics.py` | Turns parsed datagrams into throughput, loss, accuracy and latency. |
| `sweep.py` | The command-line measurement loop. |
| `report.py` | Prints every saved result as one markdown table. |

**One measurement cell**, as `sweep.py` runs it:

1. Work out `SAMPLE_PERIOD` = 96,000,000 / rate (never below 2,240 clocks).
2. Open the UDP socket, then over the UART write `SAMPLE_PERIOD` and `MODE` (and `PHASE_SELECT` when scanning), clear the counters, set run.
3. If `--target` gives the ESP's IP, start a background thread that sends an echo probe every 50 ms.
4. Capture UDP on port 5001 for the set time. Each datagram is stored raw with its arrival time; parsing waits until afterwards, because Python work between packets would look like loss. The socket gets an 8 MB buffer to absorb Wi-Fi bursts.
5. Stop the run and read `FRAMES`, `DGRAMS`, `FIFO_HIGH`, `OVERRUNS` and `STALLS`.
6. Unpack every datagram into 5 frames and a trailer, counting any that are malformed.
7. Score the cell and save it as one JSON file in `docs/results/`.

`python -m chain.report` then prints all saved cells as a table.

| Number | How it is computed |
| --- | --- |
| Throughput | Payload bytes ÷ (last arrival − first arrival). Frames per second equals samples per second per channel. |
| Loss | Each frame's `seq` relative to the first frame seen gives counts of missing, duplicate and reordered frames. |
| Accuracy | Every RAW16 frame is compared with the reference for **its own** position, so a lost frame never shifts the comparison. Gives bit errors and bit-error rate. |
| FPGA latency | (`t_tx` − `t_cap`) ÷ 96 MHz. Exact. |
| ESP latency | `esp_tx_us` − `esp_rx_us`. Exact. |
| Air latency | Half the echo round trip. An estimate, biased low under load, because the busy uplink is not symmetric with the idle downlink. |

Each latency is reported as min, median, 99th percentile and max. Extra modes:

- `--dry-run` uses `FakeSerial` and synthetic traffic.
- `--listen-only` opens no UART: for the ESP-alone stage, or when the board is driven by hand.
- `--phase-scan` runs 12 cells, `PHASE_SELECT` 0–11; with `--source emulator` it scores against the emulator's memory.

## How the code is checked and built

Everything that needs no hardware runs with one command, `make check`: the
contract check, the host Python tests, and 6 FPGA testbenches in Icarus
Verilog. GitHub Actions runs the same on every push, and also builds both ESP
firmware variants with ESP-IDF v5.4.

| Testbench | What it proves |
| --- | --- |
| `tb_pattern_gen` | The RTL test pattern matches `pattern.py` bit for bit, using vectors generated from the Python. |
| `tb_ctrl_regs` | UART and the register parser, including the real 96 MHz / 115200 baud divisor. |
| `tb_spi_master` | The receiver against the **real** emulator RTL: scans the phases and recovers all 128 channels of `.mem` data. |
| `tb_frame_builder` | Frame layout, plus overrun and drop behaviour. |
| `tb_rx_top` | The whole receiver (46 checks): both sources, and QSPI decoded by `qspi_hd_monitor.sv`, a model of the ESP slave built from the ESP-IDF source. Also back-pressure and counters. |
| `tb_chain_arty_s7_25` | The whole board top (14 checks), with stand-ins for the MMCM and the jumper wires modelled. Its clean `PHASE_SELECT` window was 3–4; the bench measured 5–6. |

**Build and load:**

1. Edit on the Mac (the master copy), then push to AFS with VS Code SFTP.
2. On a CMU ECE machine, source Vivado 2023.2's `settings64.sh` and run `QSPI_DIV=6 make bit-chain`. The batch script `fpga/vivado/build.tcl` writes `fpga/bitstreams/chain_arty_s7_25_div6.bit`. The last build met timing: WNS 0.497 ns, WHS 0.060 ns.
3. Copy the bitstream to the Mac and load it with `openFPGALoader -b arty_s7_25 chain_arty_s7_25_div6.bit`. This loads the FPGA's configuration RAM, so it is lost at power-off.
4. For the ESP: activate ESP-IDF (`. <esp-idf>/export.sh`), then from `esp/`, `idf.py -p <esp-port> -DBENCH_INGRESS=qspi build flash monitor`.

## Results and open issues

**The basic, uncompressed chain works end to end.** Overnight on 2026-10-13,
180,629 frames (370 Mbit of samples) crossed emulator → receiver → QSPI →
ESP32-S3 → Wi-Fi → Mac with **0 bit errors**, and with no loss at 2 and 4 kS/s
per channel. The first ceiling is Wi-Fi: at 8 kS/s per channel 20.6% of frames
were lost. Figures come from the JSON files in `docs/results/` and the longer
write-up [report-single-board-chain.md](report-single-board-chain.md).

| Stage | Setup | Result |
| --- | --- | --- |
| 1. ESP alone → Mac | Synthetic source, 20 kS/s offered | Ceiling 21.5 Mbit/s ≈ 10.5 kS/s per channel; 0.8% loss; 0 bit errors in 214.8 M bits |
| 1. ESP alone → Mac | Synthetic source, 2 kS/s | 4.09 Mbit/s; 0 of 19,990 frames missing; 0 bit errors in 40.9 M bits |
| 2. FPGA alone | Internal pattern, ESP unplugged | ID `1c100001`; `FRAMES` and `STALLS` climb, `DGRAMS` stays 0, as designed |
| 3. Chain, internal pattern | QSPI at 8 MHz, 2 kS/s, 3 runs | 21,861 frames; 0 missing; 0 bit errors in 44.8 M bits |
| 4. Chain, emulator through jumpers | `PHASE_SELECT` 5, 2 kS/s | 20,028 frames; 0 missing; 0 of 41.0 M bits wrong |
| 4. Chain, emulator through jumpers | `PHASE_SELECT` 5, 4 kS/s | 40,000 frames; 0 missing; 0 of 81.9 M bits wrong |
| 4. Chain, emulator through jumpers | `PHASE_SELECT` 5, 8 kS/s | 62,600 frames received, 16,270 missing (20.6%); 0 of 128.2 M bits wrong |
| 5. Phase scan | `PHASE_SELECT` 0–11 | 5 and 6 recover all 128 channels; every other setting recovers none. Simulation predicted 3–4, so the board adds about 21 ns of pad and jumper delay |
| 6. Latency at 2 kS/s | Per sample | FPGA 1.14 ms median, ESP 15 µs, Wi-Fi about 3.7 ms (estimate); about 5 ms total. The tail is long: 99th percentile +138 ms, worst +238 ms, all after the ESP's handoff |

The first chain attempt, before the fixes below, got zero datagrams through.
Two things changed that: the ESP's SPI slave had to be routed through the GPIO
matrix instead of the direct IOMUX pins (the cause is still unknown), and the
ESP's receive buffers had to be re-armed on a 2 ms timer, because an empty pool
deadlocked the link at 8 kS/s.

Open issues:

- [ ] **Why the ESP's IOMUX route fails.** Espressif says it should behave like the GPIO matrix below 80 MHz. Leading guess: ringing on SCLK or CS over the jumpers. Next step: scope GPIO12 and GPIO10.
- [ ] **Ready-line race at saturation.** At 8 kS/s one datagram reached the Mac twice. The fix is to drive `esp_ready` from the driver's receive callbacks instead of task context.
- [ ] **Where the 8 kS/s loss happens.** The FPGA sent 15,595 datagrams and the Mac got 12,520. How much was lost in the air versus at the ESP's input is not separated.
- [ ] **24 MHz QSPI** (`QSPI_DIV=2`) has not been built or tried.
- [ ] **Wi-Fi latency under load** was not measured, because the echo probe did not run during chain runs. Rerun with `--target <esp-ip>`.
- [ ] **Two FPGA quirks the host works around.** `clear_counters` does not restart the pattern generator or the emulator's playback pointer, and a half-built datagram survives a stop and start. Fixing either needs a rebuild.
- [ ] **The ESP's once-a-second log divides by a nominal second**, so its rates are wrong under load. Use the Mac's numbers.
- [ ] **ESP-IDF v5.4** (what CI builds) has never been run on hardware; only v6.0-dev has.
- [ ] **Compression is not written.** The contract compresses one time sample per frame, but the existing codec works on 256-sample blocks per channel. This needs a decision first.
