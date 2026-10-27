# host

PC end of the chain: receive the UDP stream, drive the FPGA over UART, and
score throughput, loss, accuracy and latency. Standard library plus pyserial.

    pip install -r requirements.txt

## Modules

| File | What it is |
|---|---|
| `chain/contract.py` | GENERATED from `../CONTRACTS.md`. Never edit, never redefine. |
| `chain/pattern.py` | The golden test pattern (section 3). Must match the RTL bit for bit. |
| `chain/frame.py` | Frame and datagram encode/decode (sections 1 and 2). |
| `chain/metrics.py` | Parsed datagrams in, one results dict out. |
| `chain/ctrl.py` | UART text protocol (section 4), plus `FakeSerial`, the board side in memory. |
| `chain/receiver.py` | UDP capture and the echo probe. |
| `chain/sweep.py` | The measurement loop CLI. |
| `chain/report.py` | The JSON files in `docs/results/` as a markdown table. |
| `chain/rhd2164.py` | RHD2164 knowledge: command words, ROM, Rhythm's register recipe, MISO tap decode. |
| `chain/diag.py` | The FPGA's diagnostic command list over UART, host-side phase scan, status readers. |
| `chain/fakeboard.py` | The headstage FPGA build plus two RHD2164 models and their MISO wires, with faults. |
| `chain/bringup.py` | Staged bring-up of real RHD2164s with a cause and a check for every failure. |
| `chain/viewer.py` | Live 128-channel view (UDP or UART), or headless per-channel statistics. |

## Running

    cd host
    python3 -m chain.sweep --dry-run                      # no hardware, no network
    python3 -m chain.sweep --serial /dev/ttyUSB1 --target 192.168.4.1 \
                           --rates 10000 20000 30000 --seconds 10
    python3 -m chain.sweep --phase-scan --serial /dev/ttyUSB1 --target 192.168.4.1
    python3 -m chain.sweep --listen-only --rates 20000 --seconds 10   # stage 1: ESP alone
    python3 -m chain.report

`--listen-only` opens no control port: it captures whatever arrives on UDP 5001
and scores it. Use it for stage 1, where the ESP synthesises the stream, or
when the board is driven by hand from another terminal; `--rates` then only
labels the result. `--target` (the ESP's address, from its boot log) turns on
the echo probe for air latency; without it that column is empty.

`--source emulator` clears MODE bit 2, so the receiver reads the RHD2164 bus
(on the bench, the emulator over the JA-JB jumpers), and scores every frame
against the emulator's playback memory (`chain/emulator.py`, read from
`fpga/upstream/rhd2164-emulator-mem/`). The default, `pattern`, uses the FPGA's
internal test pattern. `--phase-scan` always uses the emulator: PHASE_SELECT
does nothing to the internal pattern.

`--phase-scan` sweeps `PHASE_SELECT` 0..15 at one rate and prints the bit error
count for each; pick a phase in the middle of a run of zeros, not at its edge.

Results land one JSON file per cell in `../docs/results/`.

## Tests

    python3 -m pytest host/tests -q     # from the repo root

No hardware, no network beyond loopback.

## Reading the numbers

- `throughput` is measured at the PC, between the first and last datagram seen.
- `loss` separates missing, reordered and duplicate frames: they have different
  causes and only the first one loses data.
- `accuracy` compares each frame against the pattern (or, with `--source
  emulator`, the emulator's memory) at its own `seq` plus one per-run offset, so
  lost frames cost only themselves and never shift the comparison. The offset
  is there because the FPGA's pattern generator and playback pointer do not
  restart with `seq`; it is found by majority over the first frames and
  reported as `pattern_offset` or `playback_offset`.
- `loss.stale_frames` counts frames a run's first datagram carried over from
  the run before (the FPGA finishes a partial datagram across stop/start); they
  are dropped, not counted as loss.
- `latency` has three legs. `fpga_us` and `esp_us` are exact: each is a
  difference of two timestamps from one device's own counter. `air_us` is half a
  round trip measured by the echo probe, so it assumes a symmetric path that a
  loaded Wi-Fi uplink does not have. Treat it as an estimate, and separately.

## Real RHD2164 bring-up

Two RHD2164s share CS, SCLK and MOSI; each has its own MISO (`miso0`, `miso1`),
128 channels in all. Install the extras once:

    python3 -m pip install pyserial matplotlib numpy

`numpy` and `matplotlib` are only needed to draw; bring-up and the headless
viewer are standard library plus pyserial. The Arty's FTDI shows up as two
ports: the first (`...960`) is JTAG and never answers, the UART is the second
(`...961`).

    cd host
    python3 -m chain.bringup --serial /dev/cu.usbserial-210352BF97961
    python3 -m chain.bringup --serial PORT --spi-div 1 --phase auto --rate 2000 \
                             --lower 1 --upper 500 --seconds 5 --json /tmp/b.json
    python3 -m chain.bringup --fake chip1_absent      # no hardware: see a failure report

Each stage prints PASS / WARN / FAIL / SKIP; anything but PASS comes with a
LIKELY CAUSE and a CHECK naming a physical thing. The run ends with a one-screen
summary and a JSON report in `../docs/results/bringup_<time>.json` (`--fake`
runs save only with `--json`). Exit status is 1 if any stage failed. Hard
failures stop the run; later stages are recorded as SKIP, never PASS.

| Stage | Checks | A failure means |
|---|---|---|
| S0 FPGA | port opens, `R 1f` answers, ID `0x1c100002`, CAPS taps 78, SPI_DIV read-back | no reply: not programmed (volatile), JTAG port instead of UART, baud. `0x1c100001`: the old emulator-chain bitstream. Taps: bitstream and host built from different contracts |
| S1 Self-test | MODE bit3 loopback onto the on-die emulator; ROM on 4 lanes, phase scan, FPGA decode == host decode | the FPGA or bitstream is wrong, not the wiring. Skipped if CAPS bit8 is clear or `--skip-loopback` |
| S2 Bus activity | SPI_CYCLES advances by exactly the list length; MISO_STAT per line | stuck low: wire off, converter unpowered, no common ground (the input has a pull-down). Stuck high: converter failsafe, so the chip is not driving: chip power, MISO pair open, CS/SCLK not reaching it |
| S3 Identity + eye | READ 40-44, 59-63 with raw taps; every phase decoded on the host; FPGA decode cross-checked; PHASE_SELECT chosen (centre of the widest common window) | per lane: all 0 / all 1 (see S2); ROM only when inverted (MISO pair P/N swapped); A and B traded (SCLK pair inverted); ROM present past phase 15 (delay: S3 retries SPI_DIV 1, 3, 7 and says which works); not an RHD2164 (chip ID); lanes at different phases (unequal MISO delays); only one chip answering (that chip's power or MISO) |
| S4 Configure | WRITE 0-21, every echo `{ff,d}`; READ back 0-17 on A, 18-21 on B | echo wrong while the ROM reads fine: MOSI is not arriving intact (both chips: the shared pair). Echo right, read-back wrong: the chip lost its registers (supply) |
| S5 Calibrate | CALIBRATE + 9 dummies, then READ(63) answers normally | the window did not end where expected (a missed command) |
| S6 Convert snapshot | `--snapshots` CONVERT sweeps; mean/std/min/max per channel; 8 x 16 grid | a whole module or chip constant: its amplifier power registers or the chip. Stuck, railed or noisy single channels are WARN: floating inputs do that |
| S7 Streaming | SPI_DIV, PHASE_SELECT, SAMPLE_PERIOD, MODE 0, run `--seconds`; LINK_CHECKS = rate x time, LINK_ERR0/1 zero, FRAMES; DGRAMS/STALLS say whether an ESP drains QSPI | link errors on every sweep vs some of them (marginal eye); no frames; rate too high for the sweep at this SPI_DIV. No ESP is a note, not a failure |

Run `--leave-running` to keep streaming after S7 (for the viewer).

Register recipe (`chain/rhd2164.py`) follows Intan's Rhythm `Rhd2000Registers`
field by field, with two's complement output so CONVERT results match the
frame's int16 (0.195 uV/LSB). Defaults are for surface EMG: 2000 S/s per
channel, 1 Hz to 500 Hz analog band, DSP high-pass at about 1 Hz. The upper
band is never silently clamped to Nyquist: above 0.45 x rate S4 warns. The ADC
buffer and MUX biases are chosen for the rate commands are issued *inside* a
sweep (back to back, 32.6 kS/s-equivalent at SPI_DIV 0), not the average rate,
because they set how fast the ADC settles between consecutive conversions.

Every diagnostic list cycle costs UART round trips (three per cycle, nine with
raw taps), so a full bring-up takes a few hundred round trips plus about 100
per S6 snapshot. If the USB serial driver adds latency per read, lower
`--snapshots`.

### Viewer

    python3 -m chain.viewer --udp                          # board already streaming via the ESP
    python3 -m chain.viewer --udp --serial PORT --start    # configure (S0, S2-S5), then run
    python3 -m chain.viewer --uart --serial PORT           # no ESP: diagnostic snapshots
    python3 -m chain.viewer --udp --headless 10            # no display: status + per-channel stats

Top: an **activity map**, one tile per channel (8 x 16; two rows per module:
chip 0 A, chip 0 B, chip 1 A, chip 1 B), coloured by RMS over the window on a
log scale (1 µV to 10 mV) and labelled with the channel and its RMS. A red box
and RAIL mark a channel at the ADC rail; grey and DEAD one that never changed.

Bottom: four panels of 32 **lanes**. Each channel is centred on its median and
scaled to its own 99th-percentile excursion, then clipped to its lane, so no
trace can draw over another; the right edge says what a full lane means
(e.g. ±1.8 mV). Press `s` for one shared scale across all lanes (to compare
amplitudes; `--spacing UV` fixes it), space to freeze the display, and hover a
tile or a lane for that channel's RMS, mean, min and max.

The status line shows frame rate, seq gaps and frames lost, malformed
datagrams, and, with `--serial`, LINK_ERR per lane. `--uart` needs no ESP but
runs at about one snapshot per second: it shows which channels are alive and
how active, not EMG waveforms.
