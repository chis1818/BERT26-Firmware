#!/usr/bin/env python3
"""Find sustained tractive-system-voltage events in decoded VCU fast logs.

Scans every *_fast.csv in a directory and reports contiguous spans where
bus_voltage_v (tractive system voltage) stays strictly above 100 V for longer
than 45 seconds. t_ms is per-session (resets each file) but monotonic within a
file, so durations are computed file-by-file.
"""
import csv
import glob
import os
import sys

THRESHOLD_V = 100.0
MIN_DURATION_MS = 45_000

def fmt_ms(ms):
    s = ms / 1000.0
    m, s = divmod(s, 60)
    return f"{int(m)}m{s:05.2f}s"

def analyze(path):
    spans = []
    cur = None  # [start_ms, last_ms, n, vmax]
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                t = int(row["t_ms"])
                v = float(row["bus_voltage_v"])
            except (KeyError, ValueError):
                continue
            if v > THRESHOLD_V:
                if cur is None:
                    cur = [t, t, 1, v]
                else:
                    cur[1] = t
                    cur[2] += 1
                    cur[3] = max(cur[3], v)
            else:
                if cur is not None:
                    spans.append(cur)
                    cur = None
        if cur is not None:
            spans.append(cur)
    return [s for s in spans if (s[1] - s[0]) > MIN_DURATION_MS]

def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    files = sorted(glob.glob(os.path.join(d, "*_fast.csv")))
    hits = 0
    for path in files:
        spans = analyze(path)
        if not spans:
            continue
        name = os.path.basename(path)
        for s in spans:
            start, end, n, vmax = s
            dur = (end - start) / 1000.0
            print(f"{name}: TSV>100V for {dur:.1f}s "
                  f"(t={fmt_ms(start)}->{fmt_ms(end)}, "
                  f"peak={vmax:.1f}V, {n} samples)")
            hits += 1
    if not hits:
        print("No log had TSV>100V sustained longer than 45s.")
    else:
        print(f"\n{hits} qualifying span(s) found.")

if __name__ == "__main__":
    main()
