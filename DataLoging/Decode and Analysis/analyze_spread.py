#!/usr/bin/env python3
"""Quantify pack cell-voltage spread and test the bad-wire-bond hypothesis.

Spread = max_cell_mv - min_cell_mv (from the SLOW records).
A high-resistance interconnect (bad weld/bond) makes the affected cell sag under
discharge, so spread should grow with |bus current| and shrink at rest. A real
chemistry imbalance shows spread even at idle. We test this by:
  1. Spread distribution per session (median, 95th pct, idle vs loaded).
  2. Correlation of spread with bus current (joined from FAST records by time).
"""
import csv, glob, os, statistics, bisect

def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))

def fnum(x):
    try: return float(x)
    except: return None

def session_num(fast_path):
    base = os.path.basename(fast_path)
    return base.split("_")[0]

results = []
roots = ["tools", os.path.join("tools", "3rd set of runs"),
         os.path.join("tools", "4th Set of runs")]
for root in roots:
    for slow_path in sorted(glob.glob(os.path.join(root, "*_slow.csv"))):
        stem = slow_path[:-len("_slow.csv")]
        fast_path = stem + "_fast.csv"
        slow = read_csv(slow_path)
        # valid cell-voltage rows only
        rows = [r for r in slow if r.get("cell_v_valid") == "1"]
        spreads = []
        for r in rows:
            vmn, vmx = fnum(r["min_cell_mv"]), fnum(r["max_cell_mv"])
            if vmn and vmx and vmn > 0:
                spreads.append((fnum(r["t_ms"]), vmx - vmn, vmn, vmx, fnum(r["avg_cell_mv"])))
        if not spreads:
            continue

        # Join bus current from FAST by nearest earlier timestamp
        cur_t, cur_i = [], []
        if os.path.exists(fast_path):
            for fr in read_csv(fast_path):
                t, i = fnum(fr["t_ms"]), fnum(fr["bus_current_a"])
                if t is not None and i is not None:
                    cur_t.append(t); cur_i.append(i)

        def current_at(t):
            if not cur_t: return None
            j = bisect.bisect_right(cur_t, t) - 1
            if j < 0: j = 0
            return abs(cur_i[j])

        sp_vals = [s[1] for s in spreads]
        idle = [s[1] for s in spreads if current_at(s[0]) is not None and current_at(s[0]) < 5]
        loaded = [s[1] for s in spreads if current_at(s[0]) is not None and current_at(s[0]) > 60]

        # correlation spread vs |current|
        pairs = [(current_at(s[0]), s[1]) for s in spreads if current_at(s[0]) is not None]
        corr = None
        if len(pairs) > 5:
            xs = [p[0] for p in pairs]; ys = [p[1] for p in pairs]
            try: corr = statistics.correlation(xs, ys)
            except: corr = None

        results.append({
            "sess": session_num(slow_path),
            "n": len(sp_vals),
            "median": statistics.median(sp_vals),
            "p95": sorted(sp_vals)[int(0.95*len(sp_vals))-1],
            "max": max(sp_vals),
            "idle_med": statistics.median(idle) if idle else None,
            "load_med": statistics.median(loaded) if loaded else None,
            "corr": corr,
        })

print(f"{'sess':<10}{'n':>5}{'med':>6}{'p95':>6}{'max':>6}{'idle':>6}{'load':>6}{'corr':>7}")
for r in results:
    idle = f"{r['idle_med']:.0f}" if r['idle_med'] is not None else "-"
    load = f"{r['load_med']:.0f}" if r['load_med'] is not None else "-"
    corr = f"{r['corr']:+.2f}" if r['corr'] is not None else "-"
    print(f"{r['sess']:<10}{r['n']:>5}{r['median']:>6.0f}{r['p95']:>6.0f}{r['max']:>6.0f}{idle:>6}{load:>6}{corr:>7}")
