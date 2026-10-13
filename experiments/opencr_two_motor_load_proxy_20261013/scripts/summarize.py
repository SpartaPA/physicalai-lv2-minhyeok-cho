#!/usr/bin/env python3
"""Generate a Markdown summary without pretending motor current = supply current."""
import argparse
import csv
from pathlib import Path
import json
from statistics import mean


def percentile(numbers, p):
    if not numbers: return float("nan")
    ordered = sorted(numbers)
    i = (len(ordered) - 1) * p
    low = int(i)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (i - low)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default="results/raw")
    ap.add_argument("--out", default="results/summary.md")
    args = ap.parse_args()
    root = Path(args.raw)
    trials = []
    for path in sorted(root.glob("*/telemetry.csv")):
        meta_path = path.parent / "metadata.json"
        if not meta_path.exists(): continue
        meta = json.loads(meta_path.read_text())
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        if not rows: continue
        current_proxy = [abs(float(x["steer_current_mA"])) + abs(float(x["drive_current_mA"])) for x in rows]
        steer = [abs(float(x["steer_current_mA"])) for x in rows]
        drive = [abs(float(x["drive_current_mA"])) for x in rows]
        volt = [min(float(x["steer_voltage_V"]), float(x["drive_voltage_V"])) for x in rows]
        temps = [max(float(x["steer_temp_C"]), float(x["drive_temp_C"])) for x in rows]
        elapsed = (int(rows[-1]["host_mono_ns"]) - int(rows[0]["host_mono_ns"])) / 1e9
        trials.append((meta, path.parent.name, len(rows), elapsed, mean(steer), mean(drive),
                       mean(current_proxy), percentile(current_proxy, .95), max(current_proxy),
                       min(volt), max(temps)))
    lines = [
        "# Two-motor current-proxy experiment — trial summary", "",
        "**Scope:** XM430 motor-reported `Present Current` values, **not** directly measured OpenCR",
        "or battery input current. `sum(abs(current))` is only a motor telemetry proxy, not a",
        "validated board amperage or wattage estimate. Unobserved current transients cannot be ruled out.",
        "This experiment cannot establish compliance with OpenCR's 4.5 A rating.", "",
        "| Trial | Mode | Samples | Span (s) | Mean steer abs (mA) | Mean drive abs (mA) | Mean proxy (mA) | P95 proxy (mA) | Max sampled proxy (mA) | Min V | Max temp °C | Disarm ACK |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for meta, name, n, elapsed, sm, dm, av, p95, pk, mv, mt in trials:
        lines.append(f"| `{name}` | {meta.get('mode','?')} | {n} | {elapsed:.1f} | {sm:.1f} | {dm:.1f} | {av:.1f} | {p95:.1f} | {pk:.1f} | {mv:.2f} | {mt:.0f} | {'YES' if meta.get('disarm_acknowledged') else 'NO'} |")
    if not trials: lines.append("_No valid telemetry found yet._")
    lines += ["", "## Interpretation prompts", "",
              "1. Does BOTH show a higher motor-current proxy than DRIVE-only and STEER-only?",
              "2. Is reported voltage lower during BOTH, and is any difference beyond run-to-run variability?",
              "3. Does the drive motor track its requested velocity when steering is active?",
              "4. Are there missed samples, firmware faults, or trials without a DISARM acknowledgement?",
              "5. Why do these observations **not** establish total OpenCR supply current or peak transients?",
              "", "## Next measurement required", "",
              "Measure actual input current with a suitable current-limited laboratory supply display",
              "or appropriately rated DC power meter/shunt/Hall sensor before establishing a 4.5 A safety case.", ""]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines))
    print(f"Wrote {out} ({len(trials)} trials)")

if __name__ == "__main__":
    main()
