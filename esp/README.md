# esp/ - the ESP32-S3 node

Takes finished UDP payloads from the FPGA over QSPI, appends a 12-byte
trailer, and pushes them to a host over Wi-Fi. That is the whole job.

The ESP is deliberately dumb. It never reframes, never parses a frame, never
allocates in the data path. The FPGA builds the complete datagram; this
firmware writes 12 bytes at the end of it and calls `sendto()`. That is a
measurement decision, not laziness: it means the throughput number this rig
produces cannot be credited to, or blamed on, clever ESP-side work.

Everything about frame and datagram layout comes from `../CONTRACTS.md` via
the generated `main/chain_contract.h`. Do not hand-edit that header; change
`CONTRACTS.md`, run `make contract` at the repo root, and commit all three
generated artifacts together.


## Build status

Both ingress variants build with no warnings in `main/` against ESP-IDF
v6.0-dev (`v6.0-dev-2980-gab149384e1`), and every key in `sdkconfig.defaults`
applies. CI (`.github/workflows/esp.yml`) builds the same two variants in the
`espressif/idf:release-v5.4` image.

Both run on hardware (an ESP32-S3 DevKitC-1 clone with a CH343 USB bridge and
N8R8 module): `synth` streams to the Mac at up to ~21.5 Mbit/s on a 2.4 GHz
home network, and `qspi` receives from the Arty S7-25 bit-exact at 2-8 kS/s.

## On the bench: three things that had to change

1. **The QSPI slave runs through the GPIO matrix, not IOMUX.** On the direct
   IOMUX route the slave never decoded a command (`cmd_err` on every
   transaction, no WR_DONE ever seen), although all six lines were live. The
   same wires through the GPIO matrix deliver every datagram bit-exact.
   `slave_init()` forces the matrix with `SPICOMMON_BUSFLAG_GPIO_PINS`. ESP-IDF
   says the two routes behave the same below 80 MHz, so the cause is still open;
   the leading guess is ringing on SCLK or CS over the jumpers. Recheck at
   24 MHz (`QSPI_DIV=2`) and with a scope.
2. **The receiver re-arms on a timer, not only on completions.** If the buffer
   pool ran dry with nothing armed, nothing ever completed again: ready stayed
   low and the FPGA stalled for good (seen at 8 kS/s). The task now wakes every
   2 ms while short of armed buffers. An empty pool is a stall the FPGA counts,
   not a drop, so it uses `pipe_try_acquire()`, which does not count one.
3. **When nothing arrives, the log says why.** After a second without a
   segment, `qspi` prints what the slave decoded (last command, WR_DONE and
   cmd_err flags) and whether each QSPI pin is moving. A pin stuck high or low
   while the FPGA counts DGRAMS is a wire. `CONFIG_CHAIN_QSPI_PIN_SCAN`
   (menuconfig, off by default) goes further: it tries all 120 ways the five
   non-CS wires could be connected and runs on the one that delivers valid
   datagrams. It is how the matrix/IOMUX difference was found.

The 1 Hz `pipe` line divides by a nominal second; when the reporting task is
starved (saturated radio) its rates are wrong. Use the host's numbers.


## Build

```sh
cd esp
idf.py set-target esp32s3

idf.py -DBENCH_INGRESS=synth build      # Stage 0, default
idf.py -DBENCH_INGRESS=qspi  build      # Stage 2

idf.py menuconfig                       # -> "intan-chain benchmark node"
idf.py -p /dev/ttyACM0 flash monitor
```

Set at minimum the Wi-Fi SSID, the Wi-Fi password and the host IPv4 address
under `intan-chain benchmark node` in menuconfig, or the node will associate
with nothing and stream to nowhere.

`sdkconfig.defaults` carries the settings that move throughput (240 MHz CPU,
lwIP mailbox depth, Wi-Fi buffer counts, A-MPDU window, power save off). Each
one has a comment saying why it is there. Delete anything whose comment you
do not believe; nothing in that file is load-bearing for correctness.


## Selecting the ingress

`BENCH_INGRESS` is a CMake variable, passed on the `idf.py` command line, not
a menuconfig option. It sets `BENCH_INGRESS_SYNTH` or `BENCH_INGRESS_QSPI`,
and the only thing it changes is which source task `app_main()` starts.

Both `source_synth.c` and `source_qspi.c` are compiled in both variants. That
is deliberate: the QSPI file is the one nobody exercises until hardware
exists, so it should at least be type-checked by every build.

| | `synth` | `qspi` |
|---|---|---|
| stage | 0 | 2 |
| data comes from | the ESP itself | the FPGA, over QSPI |
| `t_cap` / `t_tx` | ESP microseconds | FPGA 96 MHz ticks |
| purpose | control arm: measures the radio path alone | the real thing |

Run `synth` first and record the number. It is the ceiling. Any `qspi` number
below it is the ingress, and that statement is only clean because both arms
share the pipe, the sink, the socket code, the task priorities and the core
assignment exactly.

### Timestamps differ between the two, and the host must know which it has

In `synth`, `t_cap` and `t_tx` are `esp_timer_get_time()` microseconds taken
on the ESP. They are not FPGA ticks, the units are wrong by a factor of ~96,
and `t_tx - t_cap` is near zero and means nothing. FPGA-internal latency is
undefined for any run made with the synthetic ingress.


## Architecture

```
  source_synth.c  ---\                                        core 1, prio 8
                      >-- pipe (16 x 1392 B, DMA) --> sink_udp.c --> Wi-Fi
  source_qspi.c   ---/       free / filled queues             core 0, prio 6

  echo.c    UDP 5002, core 0, prio 2   (must not perturb the data path)
  pipe.c    1 Hz report, core 0, prio 1
```

The source and the sink are on different cores with two queues between them,
so a Wi-Fi stall blocks only the sink. Ingress keeps running until the pool is
empty and then reports a drop. A drop that is counted is data; a stall that is
not counted is a lie about where the ceiling is.

Core 0 holds Wi-Fi, lwIP and the sink, so the pbuf hand-off stays off the
cross-core queue. Core 1 holds only the source.

### Where the number comes from

Bytes are counted in exactly one place: `pipe_note_sent()` in `pipe.c`, called
by the sink immediately after `sendto()` returns, that is once the buffer has
been handed to the socket layer. Nothing else in this firmware increments a
byte counter.

Moving that call changes what the number means. Before `sendto()` it measures
the source. In the source it measures `memcpy`. On the host it measures the
air plus the host receive path, which is a different and also useful number,
but not this one. See the header comment in `main/pipe.h`.

The 1 Hz log line reports datagrams/s, Mbit/s of UDP payload, filled-queue
high water, source drops and `sendto()` errors. Payload includes the trailer
and excludes IP, UDP and 802.11 headers, which is also what the host counts,
so the two agree without first having to agree on header overhead.


## Before first hardware bring-up

Read the header comment of `main/source_qspi.c` in full. Two things will bite:

1. **The bus protocol.** Command byte, address width, dummy cycles and line
   count for every phase are fixed by the slave hardware. They were read out
   of the ESP-IDF source (v5.4 and v6.0-dev) and the FPGA QSPI master matched
   to them: the confirmed table is in `main/source_qspi.c`, and
   `fpga/sim/qspi_hd_monitor.sv` models the slave in the FPGA testbenches. A
   one-cycle dummy mismatch does not fault; it silently rotates the data, and
   downstream that is indistinguishable from a radio problem. The CHECK
   pattern exists partly to catch this.

2. **The ready GPIO.** `PIN_QSPI_READY` (GPIO21) is asserted by the ESP only
   while a DMA receive buffer is armed, and the FPGA master must gate the next
   transaction on it. Without it the master overruns the slave at precisely
   the load being measured, and the resulting curve looks like a genuine radio
   ceiling. Cross-check against the FPGA `STALLS` register (0x14): a run with
   `STALLS` at zero and a flat curve means the line is not wired or not being
   sampled.

Pin assignments and their strapping/flash conflicts are all in
`main/pins_s3.h`.


## Verified / not verified

Verified on the machine where this was written, with no ESP-IDF present:

- `main/source_synth.c` compiles clean under `gcc -std=c11 -Wall -Wextra -O2`
  against minimal stub headers, in both pattern modes.
- Its output was compared **byte for byte** against an independent Python
  transcription of `CONTRACTS.md` section 3, over 3 datagrams / 15 frames /
  1920 channel-samples, in CHECK and in REALISTIC. Both match exactly,
  including every frame header field.
- The 16-bit LFSR has maximal period 65535 from seed 1, as a maximal
  polynomial must.
- `pipe.c`, `sink_udp.c`, `echo.c` and `app_main.c` pass `gcc -fsyntax-only
  -Wall -Wextra` against stub IDF headers (the socket stub is a shim onto the
  host's real BSD sockets, so those calls are genuinely checked).
- `app_main.c` builds under both `BENCH_INGRESS_SYNTH` and
  `BENCH_INGRESS_QSPI`, and `#error`s when neither is set.
- Every file is pure ASCII. `chain_contract.h`'s static asserts pass: header
  20 B, trailer 12 B, frame 276 B, datagram 1392 B.

Verified since, with a real toolchain:

- Both variants compile against ESP-IDF v6.0-dev with no warnings in `main/`,
  so the `spi_slave_hd` and `esp_wifi` calls in `source_qspi.c` and `wifi.c`
  exist with the signatures used.
- Every `sdkconfig.defaults` key applies (none reported as unknown).
- The QSPI bus protocol was checked against the ESP-IDF source, as above.

Not verified, and this is the honest list:

- The v5.4 build is CI's job; it has not been run locally.
- Why the IOMUX route fails (above).
- The ready-line race in `drive_ready()` is real: at 8 kS/s (Wi-Fi saturated)
  one datagram reached the host twice, from two ESP receives 13.7 ms apart,
  with identical FPGA `t_tx` stamps. That is a segment sent into no armed
  buffer, followed by a buffer completing with stale contents. The fix
  proposed in `drive_ready()`'s comment, driving the line from the driver's
  receive callbacks, has not been written yet.
- The 10 us gap the FPGA leaves after each WR_DONE, for the driver's ISR to
  re-arm the receive DMA, is an estimate of interrupt latency with Wi-Fi on
  the same core. Periodic seq gaps on the host with `STALLS` at zero would
  mean it is too short.
- `esp_wifi_sta_get_negotiated_phymode()` would be the nicer way to log the
  PHY mode; it is still left commented out in `wifi.c`.


## Open questions the contract does not settle

- **LFSR sample alignment.** `CONTRACTS.md` says each channel is seeded with
  `ch + 1` and advances once per sample, but not whether the value at sample
  `i` is the state before or after the `i`-th advance. This firmware reports
  the state *after `i` advances*, so sample 0 is the seed itself. That is the
  only reading under which the seed is observable. The FPGA and the host must
  use the same convention; if either disagrees, fix `CONTRACTS.md` and all
  three implementations, do not patch one side. See `main/pattern.h`.
- **`pipe_depth` units.** The trailer field is specified as "bytes queued in
  the ESP pipe". Implemented as filled-queue slots times `CHAIN_DGRAM_BYTES`,
  so it is quantised to 1392 and never counts a partially filled buffer.
- **QSPI clock rate.** Not in the contract. The IOMUX pins are used so the
  slave is not the limit, but the actual SCLK the FPGA drives is an FPGA-side
  decision.
- **Trailer timestamp width.** `esp_rx_us` and `esp_tx_us` are 32 bits, so
  they wrap every 4295 s. Fine for a bench run, and the host must not assume
  monotonicity across a wrap.
