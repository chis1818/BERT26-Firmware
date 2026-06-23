#!/usr/bin/env python3
import csv, os, statistics, bisect

def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))
def fnum(x):
    try: return float(x)
    except: return None

for sess in ["LOG0063", "LOG0064", "LOG0065", "LOG0066"]:
    root = os.path.join("tools", "4th Set of runs")
    slow = read_csv(os.path.join(root, sess + "_slow.csv"))
    fast = read_csv(os.path.join(root, sess + "_fast.csv"))
    rows = [r for r in slow if r.get("cell_v_valid") == "1"]
    pts = []
    for r in rows:
        vmn, vmx, vav, t = fnum(r["min_cell_mv"]), fnum(r["max_cell_mv"]), fnum(r["avg_cell_mv"]), fnum(r["t_ms"])
        if vmn and vmx and vmn > 0:
            pts.append((t, vmx-vmn, vmn, vmx, vav))
    if not pts:
        print(sess, "no valid cell data"); continue

    cur_t, cur_i = [], []
    for fr in fast:
        t, i = fnum(fr["t_ms"]), fnum(fr["bus_current_a"])
        if t is not None and i is not None: cur_t.append(t); cur_i.append(i)
    def cur(t):
        if not cur_t: return None
        j = max(0, bisect.bisect_right(cur_t, t)-1); return cur_i[j]

    spreads = [p[1] for p in pts]
    low_dev = [p[4]-p[2] for p in pts]   # avg - min  (how far the lowest cell sits below mean)
    high_dev = [p[3]-p[4] for p in pts]  # max - avg
    # within-session trend: first 25% vs last 25% of time
    pts_sorted = sorted(pts)
    q = max(1, len(pts_sorted)//4)
    first = statistics.median([p[1] for p in pts_sorted[:q]])
    last  = statistics.median([p[1] for p in pts_sorted[-q:]])
    imax = max(abs(i) for i in cur_i) if cur_i else 0
    idle = [p[1] for p in pts if cur(p[0]) is not None and abs(cur(p[0]))<3]
    hi   = [p[1] for p in pts if cur(p[0]) is not None and abs(cur(p[0]))>100]

    print(f"== {sess} ==")
    print(f"  spread mV: median={statistics.median(spreads):.0f}  max={max(spreads):.0f}")
    print(f"  low-side dev (avg-min): median={statistics.median(low_dev):.0f}   high-side dev (max-avg): median={statistics.median(high_dev):.0f}")
    print(f"  within-run: first-quarter median={first:.0f} mV  ->  last-quarter median={last:.0f} mV")
    print(f"  bus current range: {min(cur_i):.0f}..{max(cur_i):.0f} A  (|max|={imax:.0f})")
    if idle: print(f"  spread @ idle (<3A):  median={statistics.median(idle):.0f} mV  (n={len(idle)})")
    if hi:   print(f"  spread @ load(>100A): median={statistics.median(hi):.0f} mV  (n={len(hi)})")
    print()
