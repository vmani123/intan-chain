"""Measurement loop: set a rate, capture, read the counters, write one JSON.

    python3 -m chain.sweep --serial /dev/ttyUSB1 --target 192.168.4.1
    python3 -m chain.sweep --phase-scan --serial /dev/ttyUSB1 --target 192.168.4.1
    python3 -m chain.sweep --listen-only --rates 20000 --seconds 10
    python3 -m chain.sweep --dry-run

--listen-only opens no control port: it captures whatever arrives and scores
it. That is stage 1, where the ESP synthesises the stream and there is no FPGA,
and any run where the board is driven by hand from another terminal. --rates
then only labels the result; the source sets the real rate.
"""
import argparse
import contextlib
import json
import pathlib
import time

from . import contract as C
from . import frame, metrics, pattern, receiver
from .ctrl import Ctrl, FakeSerial

RESULTS = pathlib.Path(__file__).resolve().parents[2] / "docs" / "results"

# Floor from CONTRACTS.md section 4: below this the sequencer free-runs and
# OVERRUNS climbs, which is a source limit and not a radio result.
MIN_PERIOD = C.CMDS_PER_SAMPLE * 16 * C.OVERSAMPLE
MAX_RATE = C.SYS_CLK_HZ // MIN_PERIOD


def period_for(rate_hz: int) -> int:
    return max(MIN_PERIOD, round(C.SYS_CLK_HZ / rate_hz))


def synth_capture(duration_s: float, rate_hz: int, realistic: bool,
                  drop_every: int = 0) -> list[tuple[float, bytes]]:
    """Traffic for --dry-run: real bytes, fake radio.

    Built in process rather than pushed through loopback so the dry run cannot
    fail on an unrelated socket; the socket path is covered by the receiver test.
    """
    n_dgram = max(1, int(duration_s * rate_hz / C.FRAMES_PER_DGRAM))
    stream = pattern.PatternStream(realistic=realistic)
    out = []
    t0 = time.monotonic()
    seq = 0
    for d in range(n_dgram):
        frames = []
        for _ in range(C.FRAMES_PER_DGRAM):
            t_cap = (seq * period_for(rate_hz)) & 0xFFFFFFFF
            frames.append(frame.build_frame(
                seq, stream.next_sample(), t_cap=t_cap, t_tx=(t_cap + 640) & 0xFFFFFFFF,
                flags=C.FLAG_REALISTIC if realistic else 0))
            seq += 1
        if drop_every and d % drop_every == 0:
            continue  # pretend the radio ate this one
        rx = d * 1000
        out.append((t0 + d * C.FRAMES_PER_DGRAM / rate_hz,
                    frame.build_datagram(frames, esp_rx_us=rx, esp_tx_us=rx + 250, pipe_depth=2)))
    return out


def run_cell(ctrl, args, rate_hz: int, phase: int | None = None) -> dict:
    """One measurement point: configure, capture, read back, score.

    ctrl is None in listen-only mode: nothing is configured or read back.
    """
    period = period_for(rate_hz)
    if ctrl is not None:
        mode = (C.FLAG_REALISTIC if args.realistic else 0) | 0x4  # bit2: internal pattern
        ctrl.write("SAMPLE_PERIOD", period)
        ctrl.write("MODE", mode)
        if phase is not None:
            ctrl.write("PHASE_SELECT", phase)
        ctrl.clear_counters()
        ctrl.set_run(True)
    try:
        if args.dry_run:
            raw = synth_capture(args.seconds, rate_hz, args.realistic, args.drop_every)
            air = [1800.0, 2100.0, 5400.0]  # plausible placeholders, no radio here
            # Keep the fake board's counters consistent with the fake traffic,
            # so the counter read path is exercised with numbers that mean
            # something rather than with zeros.
            built = max(1, int(args.seconds * rate_hz / C.FRAMES_PER_DGRAM))
            if ctrl is not None:
                ctrl.ser.regs[C.REG["DGRAMS"]] = built
                ctrl.ser.regs[C.REG["FRAMES"]] = built * C.FRAMES_PER_DGRAM
        else:
            th, air = None, []
            if args.target:
                th, air = receiver.probe_in_background(
                    args.target, port=args.echo_port,
                    count=max(1, int(args.seconds / 0.05)), interval=0.05)
            raw = receiver.capture(args.seconds, port=args.udp_port)
            if th is not None:
                th.join(timeout=2.0)  # let the last probe land before it is scored
    finally:
        if ctrl is not None:
            ctrl.set_run(False)
    counters = ctrl.counters() if ctrl is not None else None

    dgrams, malformed = [], 0
    for t, data in raw:
        try:
            frames, trailer = frame.parse_datagram(data)
        except frame.FrameError:
            malformed += 1
            continue
        dgrams.append((t, frames, trailer))
    if not dgrams:
        return {"rate_hz": rate_hz, "phase": phase, "sample_period": period,
                "seconds": args.seconds, "counters": counters, "error": "no valid datagrams",
                "malformed_datagrams": malformed}

    m = metrics.compute(dgrams, air_us=air, malformed=malformed)
    m.update({"rate_hz": rate_hz, "phase": phase, "sample_period": period,
              "realistic": args.realistic, "seconds": args.seconds,
              "dry_run": args.dry_run, "listen_only": args.listen_only,
              "counters": counters,
              "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S")})
    return m


def save(result: dict, outdir: pathlib.Path) -> pathlib.Path:
    outdir.mkdir(parents=True, exist_ok=True)
    tag = f"phase{result['phase']:02d}_" if result.get("phase") is not None else ""
    path = outdir / f"{tag}rate{result['rate_hz']}_{time.strftime('%Y%m%d-%H%M%S')}.json"
    n = 1
    while path.exists():  # a phase scan writes several cells inside one second
        path = path.with_name(path.stem + f"_{n}.json")
        n += 1
    path.write_text(json.dumps(result, indent=1) + "\n")
    return path


def one_line(r: dict) -> str:
    if "error" in r:
        return f"  rate {r['rate_hz']:>6} phase {r['phase']}: {r['error']}"
    t, l, a = r["throughput"], r["loss"], r["accuracy"]
    return (f"  rate {r['rate_hz']:>6} phase {str(r['phase']):>4}: "
            f"{t['mbit_per_s']:6.2f} Mbit/s  loss {l['missing_pct']:5.2f}%  "
            f"bit errors {a['bit_errors']}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="run the chain measurement sweep")
    ap.add_argument("--serial", default="/dev/ttyUSB1", help="FPGA control UART")
    ap.add_argument("--target", default=None,
                    help="ESP address for the echo probe; omit to skip air latency")
    ap.add_argument("--rates", type=int, nargs="+", default=[10000, 20000, 30000],
                    help="samples/s/channel to try")
    ap.add_argument("--seconds", type=float, default=10.0, help="capture per cell")
    ap.add_argument("--realistic", action="store_true", help="REALISTIC pattern instead of CHECK")
    ap.add_argument("--phase-scan", action="store_true",
                    help="sweep PHASE_SELECT 0..11 at --phase-rate and report bit errors")
    ap.add_argument("--phase-rate", type=int, default=20000)
    ap.add_argument("--udp-port", type=int, default=C.UDP_PORT)
    ap.add_argument("--echo-port", type=int, default=C.ECHO_PORT)
    ap.add_argument("--out", type=pathlib.Path, default=RESULTS)
    ap.add_argument("--dry-run", action="store_true",
                    help="FakeSerial plus synthetic traffic; no hardware, no network")
    ap.add_argument("--drop-every", type=int, default=0,
                    help="dry run only: drop every Nth datagram to exercise the loss path")
    ap.add_argument("--listen-only", action="store_true",
                    help="no control port: capture and score only (stage 1, or a board "
                         "driven by hand); --rates just labels the result")
    args = ap.parse_args(argv)
    if args.listen_only and args.phase_scan:
        ap.error("--phase-scan writes PHASE_SELECT, which needs the control port")

    if args.listen_only:
        ctrl = None
    else:
        ctrl = Ctrl(ser=FakeSerial()) if args.dry_run else Ctrl(args.serial)
    with (ctrl if ctrl is not None else contextlib.nullcontext()):
        if ctrl is not None:
            ctrl.check_id()
        cells = ([(args.phase_rate, p) for p in range(12)] if args.phase_scan
                 else [(r, None) for r in args.rates])
        results = []
        for rate, phase in cells:
            if rate > MAX_RATE:
                print(f"  rate {rate} exceeds the {MAX_RATE} Hz sequencer limit; clamping period")
            r = run_cell(ctrl, args, rate, phase)
            path = save(r, args.out)
            results.append(r)
            print(one_line(r) + f"  -> {path.name}")

    if args.phase_scan:
        print("\nphase scan (pick a phase with zero bit errors, in the middle of a run of them):")
        for r in results:
            errs = r.get("accuracy", {}).get("bit_errors", "n/a")
            print(f"  PHASE_SELECT {r['phase']:2d}: bit errors {errs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
