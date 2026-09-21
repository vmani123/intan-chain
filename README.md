# intan-chain

A minimal, measurable prototype of one pipeline:

```
FPGA pattern/emulator  ->  FPGA receiver  ->  QSPI  ->  ESP32-S3  ->  Wi-Fi UDP  ->  PC
   Arty S7-25              Arty S7-50                                   MacBook
```

It exists to produce three numbers honestly: **throughput**, **latency** and
**accuracy**. Everything else was left out on purpose.

## Run everything that does not need hardware

```bash
make check
```

That is the contract check, the host Python tests, and five FPGA testbenches
under Icarus Verilog. It needs `iverilog`, `python3` and `pytest`, and nothing
else. On a Mac: `brew install icarus-verilog`.

## What is proven, and what is not

| Part | Status |
|---|---|
| `host/` Python: pattern, framing, metrics, control, sweep | 38 pytest tests pass |
| `fpga/rtl/gen/pattern_gen.sv` | matches the host's pattern bit for bit |
| `fpga/rtl/rx/spi_master.sv` + `miso_phase_select.sv` | closed loop against the **real** RHD2164 emulator RTL; all 128 channels recovered |
| `fpga/rtl/rx/uart_rx/tx`, `ctrl_regs.sv` | 47 checks, including at the true 96 MHz / 115200 divisor |
| `fpga/rtl/rx/frame_builder.sv`, `slot_fifo.sv`, `qspi_master.sv` | 15 checks on layout, overrun and drop behaviour |
| `fpga/rtl/rx/rx_top.sv` | 41 checks: both sources, QSPI decoded back to frames, backpressure, counters |
| `fpga/boards/rx_arty_s7_top.sv` + `.xdc` | **never synthesised.** The MMCM and the pads are the only untested RTL |
| `esp/` firmware | **never compiled.** No ESP-IDF in the environment that wrote it; CI is the first build |

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
fpga/boards/          rx_arty_s7_top.sv + .xdc   MMCM and pads, Arty S7-50
fpga/sim/             testbenches, run_sim.sh, golden vectors
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

## Build order at the bench

Each step is verifiable before the next one is wired.

1. **ESP synthetic source -> PC.** No FPGA. Gives the radio ceiling.
2. **Internal pattern inside `rx_top` -> QSPI -> ESP.** One board. Debugs the
   QSPI link with the SPI link out of the way (`MODE` bit 2).
3. **Two boards.** The emulator drives the real Intan SPI (`MODE` bit 2 clear).
   Sweep `PHASE_SELECT` 0..11 and take the setting with zero bit errors.
4. **Codec on.** Not written yet; the flag and the frame type exist for it.

## Driving the board by hand

The control interface is text on the Arty's onboard USB-UART, so it works from a
terminal before any host tool is involved:

```
screen /dev/tty.usbserial-XXXXXXXB 115200
R 1f            -> 1f=1c100001      which bitstream is loaded
W 01 0000bb80   -> ok               sample period, ticks at 96 MHz
W 03 00000004   -> ok               MODE: use the internal pattern
W 00 00000001   -> ok               run
R 13            -> 13=00000000      overruns
```

## Known ceilings in the current configuration

At 96 MHz with `QSPI_DIV = 2` (SCLK 24 MHz), one datagram takes 11056 clocks,
so the QSPI egress tops out near **23 kS/s per channel**. One SPI sweep is 2800
clocks, about **34 kS/s**. So QSPI, not the RHD2164 sequencer, is the first
ceiling -- worth knowing before blaming the radio. Lower `QSPI_DIV` to 1 to
double it.

## Licence note

`fpga/rtl/rx/miso_phase_select.sv` reimplements the mechanism in Intan's
`MISO_phase_selector.v`, whose licence grants use "for any applications that use
Intan Technologies integrated circuits". That is fine here and worth remembering
if the design is ever retargeted to a non-Intan front end.
