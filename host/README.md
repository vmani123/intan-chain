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

`--phase-scan` sweeps `PHASE_SELECT` 0..11 at one rate and prints the bit error
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
