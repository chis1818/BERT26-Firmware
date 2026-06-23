#!/usr/bin/env python3
"""Analyze a folder of VCU LOGnnnn.BIN files:
- max power (medium records, and V*I from fast records)
- max realistic torque from power/rpm (T_nm = 9549 * kW / rpm) vs commanded
- BSPD fault windows (bspd_gpio_ok == 0) excluding initial startup
"""
import glob, os, struct, sys

MAGIC = b"VCULOG01"
HEADER_FMT = "<8sIBBBB"
FAST_FMT = "<BIHHHHHhhhhH"
MEDIUM_FMT = "<BIhhhhhhHhhBBBBHIIBB"
SLOW_FMT = "<BIHHHhhhHBB"
IMU_FMT = "<BIhhhhhhh"
BSPD_BIT = 12  # bspd_gpio_ok

def decode(path):
    data = open(path, "rb").read()
    hs = struct.calcsize(HEADER_FMT)
    magic, session, fs, ms_, ss, imus = struct.unpack_from(HEADER_FMT, data, 0)
    if magic != MAGIC:
        return None
    sizes = {1: fs, 2: ms_, 3: ss}
    # imus is 0 in logs from firmware without the MPU6050; newer logs interleave
    # IMU records (type 4) that this analysis ignores but must skip over.
    if imus:
        sizes[4] = imus
    fast, med = [], []
    off = hs
    while off < len(data):
        rt = data[off]
        size = sizes.get(rt)
        if size is None or off + size > len(data):
            break
        if rt == 1:
            (_, ms, a1, a2, b1, b2, hv, trq, rpm, busv, busi,
             flags) = struct.unpack_from(FAST_FMT, data, off)
            fast.append((ms, trq / 10.0, rpm, busv / 10.0, busi / 10.0, flags))
        elif rt == 2:
            (_, ms, ap1, ap2, ped, des, pwr, mph, glv, invt, mtrt, soc,
             batt, bmsf, code, warn, post, run, fan,
             rtd) = struct.unpack_from(MEDIUM_FMT, data, off)
            med.append((ms, des / 10.0, pwr / 10.0, rtd))
        off += size
    return session, fast, med

def fmt_t(ms):
    return f"{ms/1000.0:8.1f}s"

folder = sys.argv[1]
overall = []
for path in sorted(glob.glob(os.path.join(folder, "LOG*.BIN"))):
    name = os.path.basename(path)
    r = decode(path)
    if not r:
        print(f"{name}: bad magic, skipped")
        continue
    session, fast, med = r
    if not fast:
        print(f"{name}: no fast records")
        continue

    dur = fast[-1][0] / 1000.0
    # max power from medium records
    max_pwr = max((m[2], m[0]) for m in med) if med else (0, 0)
    # max V*I from fast records
    max_vi = max((v * i / 1000.0, ms, v, i, rpm, trq)
                 for ms, trq, rpm, v, i, flags in fast)
    # realistic torque: from medium power, need rpm at same time -> use fast V*I and rpm together
    best_tq = (0, 0, 0, 0, 0)  # (tq, ms, kw, rpm, cmd)
    for ms, trq, rpm, v, i, flags in fast:
        if rpm > 100:
            kw = v * i / 1000.0
            tq = 9549.3 * kw / rpm
            if tq > best_tq[0]:
                best_tq = (tq, ms, kw, rpm, trq)
    max_cmd = max((trq, ms, rpm) for ms, trq, rpm, v, i, flags in fast)
    max_rpm = max(rpm for ms, trq, rpm, v, i, flags in fast)

    # BSPD analysis: find windows where bspd_gpio_ok == 0
    windows = []
    cur = None
    for ms, trq, rpm, v, i, flags in fast:
        ok = bool(flags & (1 << BSPD_BIT))
        if not ok:
            if cur is None:
                cur = [ms, ms]
            else:
                cur[1] = ms
        else:
            if cur is not None:
                windows.append(tuple(cur))
                cur = None
    if cur is not None:
        windows.append((cur[0], cur[1], "until end"))

    first_ms = fast[0][0]
    print(f"\n=== {name} (session {session}, {dur:.0f}s, {len(fast)} fast rows) ===")
    print(f"  max power (medium log): {max_pwr[0]:.1f} kW at t={max_pwr[1]/1000:.1f}s")
    print(f"  max bus V*I (fast log): {max_vi[0]:.1f} kW at t={max_vi[1]/1000:.1f}s "
          f"({max_vi[2]:.1f}V x {max_vi[3]:.1f}A, rpm={max_vi[4]}, cmd={max_vi[5]:.1f}Nm)")
    if best_tq[0]:
        print(f"  max calc torque: {best_tq[0]:.1f} Nm at t={best_tq[1]/1000:.1f}s "
              f"({best_tq[2]:.1f} kW @ {best_tq[3]} rpm, commanded {best_tq[4]:.1f} Nm)")
    print(f"  max commanded torque: {max_cmd[0]:.1f} Nm at t={max_cmd[1]/1000:.1f}s (rpm={max_cmd[2]})")
    print(f"  max rpm: {max_rpm}")
    if windows:
        for w in windows:
            startup = " [startup]" if w[0] <= first_ms + 3000 else ""
            end = w[1] / 1000.0
            print(f"  BSPD low: {w[0]/1000:.1f}s -> {end:.1f}s ({(w[1]-w[0])/1000:.1f}s){startup}")
    else:
        print("  BSPD: never low")
    overall.append((name, max_pwr[0], max_vi[0], best_tq, max_cmd[0]))

print("\n==== OVERALL ====")
best_p = max(overall, key=lambda x: max(x[1], x[2]))
best_t = max(overall, key=lambda x: x[3][0])
print(f"Max power: {max(best_p[1], best_p[2]):.1f} kW in {best_p[0]}")
bt = best_t[3]
print(f"Max calc torque: {bt[0]:.1f} Nm ({bt[2]:.1f} kW @ {bt[3]} rpm, cmd {bt[4]:.1f} Nm) in {best_t[0]}")
