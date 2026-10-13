# intan-chain

A minimal, measurable prototype of one pipeline:

```
RHD2164 emulator  ->  receiver  ->  QSPI  ->  ESP32-S3  ->  Wi-Fi UDP  ->  PC
\___ one Arty S7-25, SPI over jumpers ___/                                MacBook
```

The emulator and the receiver can also sit on separate boards (`fpga/boards/`
has tops for an Arty S7-50 and a RealDigital Boolean); the bench currently
uses the single-board build, described below.

It exists to produce three numbers honestly: **throughput**, **latency** and
**accuracy**. Everything else was left out on purpose.

**Read first:** [docs/report-single-board-chain.md](docs/report-single-board-chain.md),
the full write-up: architecture, verification, bench results, what limits
performance, and open issues.

## Run everything that does not need hardware

```bash
make check
```

That is the contract check, the host Python tests, and six FPGA testbenches
under Icarus Verilog. It needs `iverilog`, `python3` and `pytest`, and nothing
else. On a Mac: `brew install icarus-verilog`.

## What is proven, and what is not

| Part | Status |
|---|---|
| `host/` Python: pattern, framing, metrics, control, sweep, emulator golden | 50 pytest tests pass |
| `fpga/rtl/gen/pattern_gen.sv` | matches the host's pattern bit for bit |
| `fpga/rtl/rx/spi_master.sv` + `miso_phase_select.sv` | closed loop against the **real** RHD2164 emulator RTL; all 128 channels recovered |
| `fpga/rtl/rx/uart_rx/tx`, `ctrl_regs.sv` | 47 checks, including at the true 96 MHz / 115200 divisor |
| `fpga/rtl/rx/frame_builder.sv`, `slot_fifo.sv` | 15 checks on layout, overrun and drop behaviour |
| `fpga/rtl/rx/qspi_master.sv` | speaks the ESP32-S3 slave-HD protocol; checked against `fpga/sim/qspi_hd_monitor.sv`, a model of the slave built from the ESP-IDF source |
| `fpga/rtl/rx/rx_top.sv` | 46 checks: both sources, QSPI decoded the way the ESP latches it, backpressure, counters |
| `fpga/boards/chain_arty_s7_25_top.sv` + `.xdc` | the whole board simulated (14 checks, jumpers modelled); synthesised (`QSPI_DIV=6`, WNS +0.497 ns) and **run on hardware**: emulator -> jumpers -> receiver -> QSPI -> ESP -> Wi-Fi -> Mac bit-exact at 2, 4 and 8 kS/s |
| `fpga/boards/rx_arty_s7_top.sv` + `rx_boolean.xdc` | synthesised and smoke-tested on a Boolean (UART, ID, internal pattern), before the QSPI protocol fix |
| `esp/` firmware | runs on an ESP32-S3 DevKitC-1 clone: synthetic source to the Mac, and the QSPI receiver (through the GPIO matrix, see `esp/README.md`) |

The split is deliberate: `rx_top` takes an already-buffered clock and contains no
vendor primitives, so the entire receiver simulates in plain Icarus. Only the
~100-line board wrapper is outside that.

## Layout

```
CONTRACTS.md          the frame, datagram, pattern and register definitions
tools/                generators: contract -> C header + Python; pattern -> RTL vectors

fpga/rtl/gen/         pattern_gen         the test pattern, in hardware
fpga/rtl/rx/          spi_master          RHD2164 sequencer + 4x oversample capture
                      miso_phase_select   Intan's phase-select trick, from Rhythm
                      frame_builder       header + payload into a datagram slot
                      slot_fifo, dgram_mem
                      qspi_master         quad-SPI egress, ready-gated, patches t_tx
                      uart_rx/tx, ctrl_regs
                      rx_top              the receiver, board-independent
fpga/boards/          chain_arty_s7_25_top.sv + .xdc   emulator + receiver, one Arty S7-25
                      rx_arty_s7_top.sv + rx_*.xdc     receiver alone (Arty S7-50, Boolean)
                      emu_arty_s7_25.xdc               emulator alone (rhd2164_top_se)
fpga/sim/             testbenches, run_sim.sh, golden vectors
                      qspi_hd_monitor     the ESP32-S3 slave-HD receiver, as a model
                      unisim_stubs        IBUF/BUFG/MMCM stand-ins so board tops simulate
fpga/vivado/          build.tcl, program.tcl (non-project batch flow; `make bit-*`)
fpga/upstream/        vendored RHD2164 emulator RTL + memories (stimulus; never edited)

esp/                  ESP-IDF project: dumb DMA-to-socket relay
host/chain/           pattern, frame, metrics, ctrl, receiver, sweep, report
docs/results/         one JSON per measurement run, committed
```

## The idea in one paragraph

The FPGA builds the **complete UDP payload** -- five frames, 1380 bytes -- and
hands it to the ESP in one QSPI transfer. The ESP appends a 12-byte trailer and
calls `sendto`. It never reframes, never parses a frame, never allocates. That
is a measurement decision: whatever throughput comes out, nobody can argue the
ESP was doing something clever, and the ESP firmware stays about 150 lines.

Every frame carries two FPGA clock ticks (`t_cap`, `t_tx`) and every datagram
carries two ESP microsecond stamps, so three of the four latency legs are exact
and only the air hop needs estimating. Accuracy is scored against a closed-form
pattern that the FPGA and the host generate independently from the same
specification -- no data files to keep in step.

## The single-board bench

One Arty S7-25 carries both designs. The receiver's SPI master leaves on Pmod
JA and comes back into the emulator on Pmod JB through five jumpers, so the
bus crosses real pads and wire; the QSPI to the ESP leaves on Pmod JC. JA and
JB are the Arty S7's high-speed Pmods (no series resistors); JC is standard
(200 ohm). Clocks: one MMCM makes 96 MHz for the receiver and 192 MHz for the
emulator.

| Jumper | Signal | | Arty | Signal | ESP32-S3 |
|---|---|---|---|---|---|
| JA1 -> JB1 | cs_n | | JC1 | qspi_cs_n | GPIO10 |
| JA2 -> JB2 | sclk | | JC2 | qspi_sclk | GPIO12 |
| JA3 -> JB3 | mosi | | JC3 | qspi_dq[0] | GPIO11 |
| JB4 -> JA4 | miso0 | | JC4 | qspi_dq[1] | GPIO13 |
| JB7 -> JA7 | miso1 | | JC7 | qspi_dq[2] | GPIO14 |
| | | | JC8 | qspi_dq[3] | GPIO9 |
| | | | JC9 | esp_ready (in) | GPIO21 |
| | | | JC5/JC11 | GND | GND |

Never connect a 3V3 pin (Pmod pins 6 and 12) between boards. The SPI loop
needs no ground jumper; the ESP does.

LEDs: LD2 receiver running, LD3 heartbeat, LD4 QSPI activity, LD5 the
emulator sees chip select (dark while LD2 is lit means a JA/JB jumper is off).

### Build and program

```bash
# on a machine with Vivado (source <Vivado>/settings64.sh first)
QSPI_DIV=6 make bit-chain     # -> fpga/bitstreams/chain_arty_s7_25_div6.bit, QSPI at 8 MHz

# on the Mac, with the Arty on USB (brew install openfpgaloader)
openFPGALoader -b arty_s7_25 fpga/bitstreams/chain_arty_s7_25_div6.bit
```

Start at `QSPI_DIV=6`: 8 MHz over jumper wires proves the protocol before
speed is pushed. `make bit-chain` alone builds 24 MHz.

### Bring-up order

Each step is verifiable before the next one is wired.

1. **ESP synthetic source -> PC.** No FPGA (`-DBENCH_INGRESS=synth`). Gives the
   radio ceiling.
2. **Board alone.** Program it, `R 1f` must answer `1f=1c100001`. With the ESP
   unplugged, `W 03 00000004` then `W 00 00000001`: `FRAMES` climbs, `STALLS`
   climbs and `DGRAMS` stays 0 (esp_ready is pulled down).
3. **Internal pattern -> QSPI -> ESP -> PC.** Wire JC to the ESP, flash
   `-DBENCH_INGRESS=qspi`. Debugs the QSPI link with SPI out of the way
   (`MODE` bit 2): `python3 -m chain.sweep --serial <arty uart> --rates 2000`.
   The host's CHECK pattern must show zero bit errors.
4. **Emulator through the jumpers.** `MODE` bit 2 clear:
   `python3 -m chain.sweep --serial <arty uart> --phase-scan --phase-rate 2000` scores every frame
   against the emulator's playback memory, so a phase either recovers all 128
   channels exactly or shows bit errors. Take the middle of the clean run. On
   the bench the window is PHASE_SELECT 5-6 (simulation: 3-4; the extra is pad
   and jumper delay). Then `--source emulator --rates ...` for throughput.
5. **Codec on.** Not written yet; the flag and the frame type exist for it.

## Driving the board by hand

The control interface is text on the Arty's onboard USB-UART, so it works from a
terminal before any host tool is involved. The board ends a command on line
feed only and never echoes, so in `screen` type blind and press **Ctrl-J**, not
Enter (Enter sends a carriage return, which the board drops). Anything typed
before stays in the board's line buffer until a line feed: if the first reply
is `err`, press Ctrl-J once and retry.

```
screen /dev/cu.usbserial-XXXXXXXX1 115200    # the Arty's second port, on a Mac
R 1f            -> 1f=1c100001      which bitstream is loaded
W 01 0000bb80   -> ok               sample period, ticks at 96 MHz
W 03 00000004   -> ok               MODE: use the internal pattern
W 00 00000001   -> ok               run
R 13            -> 13=00000000      overruns
```

## FPGA behaviour the host works around

Both are in the RTL as built; the host's scoring copes, and a later RTL change
can remove them.

- **The pattern does not restart with seq.** `clear_counters` zeroes the frame
  counter, but the pattern generator carries on, so a run after the first
  carries pattern index seq + (samples generated before). `metrics` finds that
  offset per run (`accuracy.pattern_offset`) and holds every frame to it.
- **A partly built datagram survives stop/start.** The first datagram of a run
  can carry the previous run's last frames ahead of this run's first.
  `metrics` drops them and reports `loss.stale_frames`.

## Known ceilings in the current configuration

One datagram on the QSPI link is a WRDMA (16 command/address bits, 8 dummy
cycles, 2760 data nibbles), a WR_DONE (24 SCLKs) and a 10 us gap for the ESP
to re-arm its DMA. At 96 MHz:

| `QSPI_DIV` | SCLK | clocks per datagram | QSPI ceiling |
|---|---|---|---|
| 6 | 8 MHz | ~34700 | **~13.8 kS/s per channel** |
| 2 | 24 MHz | ~12200 | ~39 kS/s per channel |

One SPI sweep is 2800 clocks, about **34 kS/s**. So at `QSPI_DIV = 6` the QSPI
link is the first ceiling, and at 2 the RHD2164 sequencer is -- worth knowing
before blaming the radio.

## Licence note

`fpga/rtl/rx/miso_phase_select.sv` reimplements the mechanism in Intan's
`MISO_phase_selector.v`, whose licence grants use "for any applications that use
Intan Technologies integrated circuits". That is fine here and worth remembering
if the design is ever retargeted to a non-Intan front end.
