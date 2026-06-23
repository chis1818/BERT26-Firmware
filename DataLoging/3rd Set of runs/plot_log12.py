"""
LOG0012 half-endurance pack analysis.
Chart 1: pack voltage spread (mV) vs time (s)
Chart 2: min / max / avg cell voltage with a change-point detector to estimate
         when the low cell (suspected failed wirebond, ~60 mV low) appeared.

numpy + matplotlib only (no pandas/scipy on this machine).
"""
import csv
import numpy as np
import matplotlib.pyplot as plt

CSV = "LOG0012_slow.csv"

# ---- load ----------------------------------------------------------------
t_ms, vmin, vmax, spread, vavg = [], [], [], [], []
with open(CSV, newline="") as fh:
    reader = csv.DictReader(fh)
    for row in reader:
        try:
            if int(row["cell_v_valid"]) != 1:
                continue
            t = float(row["t_ms"])
            lo = float(row["min_cell_mv"])
            hi = float(row["max_cell_mv"])
            av = float(row["avg_cell_mv"])
        except (TypeError, ValueError):
            continue  # garbage / trailing blank line
        if lo <= 0 or hi <= 0:
            continue  # not yet measuring real voltages
        t_ms.append(t)
        vmin.append(lo)
        vmax.append(hi)
        vavg.append(av)
        spread.append(hi - lo)

t_ms = np.asarray(t_ms)
vmin = np.asarray(vmin)
vmax = np.asarray(vmax)
vavg = np.asarray(vavg)
spread = np.asarray(spread)

# x axis in seconds, elapsed from first valid sample
t_s = (t_ms - t_ms[0]) / 1000.0
print(f"valid samples: {len(t_s)}   duration: {t_s[-1]:.0f} s "
      f"({t_s[-1]/60:.1f} min)")
print(f"spread  min/median/max: {spread.min():.0f} / "
      f"{np.median(spread):.0f} / {spread.max():.0f} mV")

# ---- Chart 1: spread vs time --------------------------------------------
fig1, ax1 = plt.subplots(figsize=(12, 5))
ax1.plot(t_s, spread, color="tab:red", lw=1.0)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Cell voltage spread, max - min (mV)")
ax1.set_title("Pack Cell-Voltage Spread vs Time — Half Endurance Simulation (LOG0012)")
ax1.grid(True, alpha=0.3)
ax1.margins(x=0)
fig1.tight_layout()
fig1.savefig("LOG0012_spread_vs_time.png", dpi=150)
print("wrote LOG0012_spread_vs_time.png")

# ---- weak-cell statistics ------------------------------------------------
# A failing wirebond shows up as the MIN cell sagging under load (I*R drop),
# so the diagnostic signal is "avg - min" (how far the weak cell is below the
# pack) and, more importantly, how the *envelope* of that sag trends in time.
low_gap = vavg - vmin                      # weak-cell gap below average (mV)

# rolling envelope (max) and rolling 95th pct of the sag, ~30 s windows
def rolling(fn, x, w):
    out = np.empty_like(x, dtype=float)
    half = w // 2
    for i in range(len(x)):
        a, b = max(0, i - half), min(len(x), i + half + 1)
        out[i] = fn(x[a:b])
    return out

W = 15                                       # ~30 s window (1 sample/s)
gap_env = rolling(np.max, low_gap, W)        # worst-case sag envelope
gap_p95 = rolling(lambda v: np.percentile(v, 95), low_gap, W)

# drive start = where spread first lifts off its idle baseline
idle = np.median(spread[:20])
drive = np.where(spread > idle + 5)[0]
drive_start = int(drive[0]) if len(drive) else 0
ds_t = t_s[drive_start]

# trend of the sag envelope across the driving portion -> worsening or stable?
m = slice(drive_start, len(t_s))
coef = np.polyfit(t_s[m], gap_env[m], 1)     # slope (mV/s), intercept
trend = np.poly1d(coef)
slope_per_min = coef[0] * 60.0

# peak sag event
pk = int(np.argmax(low_gap))
pk_t, pk_v = t_s[pk], low_gap[pk]

print(f"drive start (load begins): t = {ds_t:.0f} s")
print(f"peak weak-cell sag: {pk_v:.0f} mV below avg at t = {pk_t:.0f} s")
print(f"sag-envelope trend: {slope_per_min:+.2f} mV/min "
      f"({'worsening' if slope_per_min > 0.2 else 'stable across run'})")

# ---- Chart 2: statistical view ------------------------------------------
fig2, (axA, axB) = plt.subplots(
    2, 1, figsize=(12, 9), sharex=True,
    gridspec_kw={"height_ratios": [2, 1.2]})

# top: min / avg / max band
axA.fill_between(t_s, vmin, vmax, color="tab:blue", alpha=0.12,
                 label="min-max band")
axA.plot(t_s, vmax, color="tab:green", lw=1.0, label="max cell (mV)")
axA.plot(t_s, vavg, color="tab:blue", lw=1.2, label="avg cell (mV)")
axA.plot(t_s, vmin, color="tab:red", lw=1.0, label="min cell (mV)")
axA.axvline(ds_t, color="k", ls="--", lw=1.0)
axA.plot(pk_t, vmin[pk], "v", color="darkred", ms=10, zorder=5)
axA.annotate(f"deepest sag\n{pk_v:.0f} mV below avg\nt ≈ {pk_t:.0f} s",
             xy=(pk_t, vmin[pk]), xytext=(0.55, 0.12),
             textcoords="axes fraction",
             arrowprops=dict(arrowstyle="->", color="darkred"),
             fontsize=9, ha="left",
             bbox=dict(boxstyle="round", fc="wheat", alpha=0.85))
axA.set_ylabel("Cell voltage (mV)")
axA.set_title("Min / Avg / Max Cell Voltage — Weak-Cell Diagnosis — "
              "Half Endurance Simulation (LOG0012)")
axA.legend(loc="lower right", ncol=2, fontsize=8)
axA.grid(True, alpha=0.3)
axA.margins(x=0)

# bottom: weak-cell sag, its rolling envelope, and the trend line
axB.plot(t_s, low_gap, color="tab:red", lw=0.7, alpha=0.5,
         label="avg - min  (weak-cell sag, mV)")
axB.plot(t_s, gap_p95, color="tab:orange", lw=1.3,
         label="rolling 95th pct (≈30 s)")
axB.plot(t_s, gap_env, color="maroon", lw=1.0, alpha=0.6,
         label="rolling max envelope")
axB.plot(t_s[m], trend(t_s[m]), color="black", ls="-.", lw=1.4,
         label=f"envelope trend {slope_per_min:+.2f} mV/min")
axB.axvline(ds_t, color="k", ls="--", lw=1.0,
            label=f"drive start t≈{ds_t:.0f}s")
axB.set_xlabel("Time (s)")
axB.set_ylabel("avg - min (mV)")
axB.legend(loc="upper right", fontsize=8, ncol=2)
axB.grid(True, alpha=0.3)
axB.margins(x=0)

fig2.tight_layout()
fig2.savefig("LOG0012_cell_stats.png", dpi=150)
print("wrote LOG0012_cell_stats.png")
