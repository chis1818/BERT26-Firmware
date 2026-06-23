#!/usr/bin/env python3
"""Show context around BSPD-low events and re-check torque calc above 1000 rpm."""
import struct, sys

HEADER_FMT = "<8sIBBBB"
FAST_FMT = "<BIHHHHHhhhhH"
SIZES = {1: struct.calcsize(FAST_FMT), 2: struct.calcsize("<BIhhhhhhHhhBBBBHIIBB"),
         3: struct.calcsize("<BIHHHhhhHBB"), 4: struct.calcsize("<BIhhhhhhh")}
FLAGS = ["braking", "hard_braking", "bspc_active", "apps_plausible", "rtd_latched",
         "drive_mode_d", "bms_fault", "imd_fault", "pm100_fault", "bms_timeout",
         "imd_can_timeout", "contactors_closed", "bspd_gpio_ok", "imd_gpio_ok",
         "bms_gpio_ok"]

def load(path):
    data = open(path, "rb").read()
    off = struct.calcsize(HEADER_FMT)
    rows = []
    while off < len(data):
        rt = data[off]
        size = SIZES.get(rt)
        if size is None or off + size > len(data):
            break
        if rt == 1:
            (_, ms, a1, a2, b1, b2, hv, trq, rpm, busv, busi,
             flags) = struct.unpack_from(FAST_FMT, data, off)
            rows.append((ms, b1, b2, hv, trq / 10.0, rpm, busv / 10.0,
                         busi / 10.0, flags))
        off += size
    return rows

def show(path, t0, t1, step=1):
    rows = load(path)
    print(f"\n--- {path} {t0}..{t1}s ---")
    print("t_s     bse1  bse2  hvraw  cmdNm   rpm   busV   busA  flags")
    last = None
    for i, r in enumerate(rows):
        t = r[0] / 1000.0
        if t0 <= t <= t1 and i % step == 0:
            fl = [n for b, n in enumerate(FLAGS) if r[8] & (1 << b)]
            line = (f"{t:7.2f} {r[1]:5d} {r[2]:5d} {r[3]:6d} {r[4]:6.1f} "
                    f"{r[5]:5d} {r[6]:6.1f} {r[7]:6.1f}  {','.join(fl)}")
            print(line)

# torque recheck >=1000 rpm across the third set's power runs
SET = "3rd set of runs"
for f in ["LOG0010.BIN", "LOG0011.BIN", "LOG0012.BIN"]:
    rows = load(f"{SET}/{f}")
    best = max(((9549.3 * v * i / 1000.0 / rpm, ms, v * i / 1000.0, rpm, trq)
                for ms, b1, b2, hv, trq, rpm, v, i, fl in rows if rpm >= 1000),
               default=None)
    if best:
        print(f"{f}: max calc torque (rpm>=1000): {best[0]:.1f} Nm "
              f"({best[2]:.1f} kW @ {best[3]} rpm, cmd {best[4]:.1f} Nm, t={best[1]/1000:.1f}s)")

# context around real BSPD-low events found by analyze_runs in this set
show(f"{SET}/LOG0008.BIN", 50.5, 56.5, 4)   # longest stationary BSPD-low window
show(f"{SET}/LOG0011.BIN", 402.3, 402.9, 1) # brief BSPD-low at end of driving run
show(f"{SET}/LOG0012.BIN", 1069.0, 1069.6, 1)
