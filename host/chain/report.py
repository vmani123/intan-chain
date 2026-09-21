"""Print the JSON files in docs/results/ as one markdown table.

    python3 -m chain.report [--dir docs/results]
"""
import argparse
import json
import pathlib

RESULTS = pathlib.Path(__file__).resolve().parents[2] / "docs" / "results"

COLS = ["rate", "phase", "Mbit/s", "kS/s/ch", "loss %", "dup", "reord",
        "bit err", "BER", "fpga us", "esp us", "air us"]


def _leg(stats) -> str:
    """One latency leg as median / p99, the two numbers worth eyeballing."""
    if not stats:
        return "-"
    return f"{stats['median']:.0f}/{stats['p99']:.0f}"


def row(r: dict) -> list[str]:
    if "error" in r:
        return [str(r.get("rate_hz", "?")), str(r.get("phase", "-")), r["error"]] + ["-"] * 9
    t, l, a, lat = r["throughput"], r["loss"], r["accuracy"], r["latency"]
    return [
        str(r.get("rate_hz", "?")),
        "-" if r.get("phase") is None else str(r["phase"]),
        f"{t['mbit_per_s']:.2f}",
        f"{t['samples_per_s_per_ch'] / 1000:.2f}",
        f"{l['missing_pct']:.3f}",
        str(l["duplicates"]),
        str(l["reordered"]),
        str(a["bit_errors"]) if a["checked"] else "-",
        f"{a['ber']:.2e}" if a.get("ber") else ("0" if a["checked"] else "-"),
        _leg(lat["fpga_us"]),
        _leg(lat["esp_us"]),
        _leg(lat["air_us"]),
    ]


def table(results: list[dict]) -> str:
    rows = [row(r) for r in results]
    lines = ["| " + " | ".join(COLS) + " |",
             "|" + "|".join(["---"] * len(COLS)) + "|"]
    lines += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(lines)


def load(d: pathlib.Path) -> list[dict]:
    out = []
    for path in sorted(d.glob("*.json")):
        try:
            out.append(json.loads(path.read_text()))
        except (OSError, ValueError) as exc:
            print(f"skipping {path.name}: {exc}")
    # Sort by the two knobs, so a phase scan reads in phase order.
    return sorted(out, key=lambda r: (r.get("rate_hz", 0), r.get("phase") is not None,
                                      r.get("phase") or 0, r.get("timestamp", "")))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="tabulate the sweep results")
    ap.add_argument("--dir", type=pathlib.Path, default=RESULTS)
    args = ap.parse_args(argv)
    results = load(args.dir)
    if not results:
        print(f"no results in {args.dir}")
        return 1
    print(f"# chain results ({len(results)} cells from {args.dir})\n")
    print(table(results))
    print("\nLatency columns are median/p99 microseconds. fpga and esp are exact "
          "differences of one device's own counter; air is half a round trip and "
          "is an estimate.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
