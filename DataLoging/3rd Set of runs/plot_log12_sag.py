"""
LOG0012 weak-cell sag model — measured vs expected (DCIR) sag.

Joins bus current (fast log, 100 Hz) with min/max/avg cell voltages
(slow log, 1 Hz) and overlays the expected I*DCIR sag on the measured min
and max cell voltages.

Pack: 4 cells in PARALLEL  ->  cell current = bus_current / 4
DCIR = 13 mOhm per cell    ->  sag = (I_bus/4) * 13 mOhm

Model:  V_cell(t) = OCV_soc(t) + rest_offset - (I_bus(t)/N_par) * DCIR
  - OCV_soc(t): no-load pack voltage, interpolated from low-current samples
  - rest_offset: each cell's static balance offset at rest
A healthy cell tracks its prediction; the failed-wirebond cell sags BELOW it.

NOTE: the 1 Hz BMS voltages are slow/held (a single min value can persist for
several seconds), so the current is averaged over a +/-0.5 s window to match
the voltage cadence rather than using an instantaneous sample.
numpy + matplotlib only.
"""
import csv
import numpy as np
import matplotlib.pyplot as plt

SLOW = "LOG0012_slow.csv"
FAST = "LOG0012_fast.csv"
DCIR_OHM = 0.013          # 13 mOhm per cell
N_PAR = 4                 # 4 cells in parallel
LOW_I = 3.0               # |A| considered "no load" for OCV estimate
WIN_MS = 500              # +/- window for current averaging

# ---- slow log: cell voltages --------------------------------------------
st, vmin, vmax, vavg = [], [], [], []
with open(SLOW, newline="") as fh:
    for row in csv.DictReader(fh):
        try:
            if int(row["cell_v_valid"]) != 1:
                continue
            lo, hi, av = (float(row["min_cell_mv"]), float(row["max_cell_mv"]),
                          float(row["avg_cell_mv"]))
            t = float(row["t_ms"])
        except (TypeError, ValueError):
            continue
        if lo <= 0 or hi <= 0:
            continue
        st.append(t); vmin.append(lo); vmax.append(hi); vavg.append(av)
st = np.asarray(st); vmin = np.asarray(vmin)
vmax = np.asarray(vmax); vavg = np.asarray(vavg)

# ---- fast log: bus current (cols t_ms=0, bus_current_a=9) ----------------
fast = np.genfromtxt(FAST, delimiter=",", skip_header=1, usecols=(0, 9),
                     invalid_raise=False)
fast = fast[~np.isnan(fast).any(axis=1)]
ft, fi = fast[:, 0], fast[:, 1]

# bus current averaged over a +/- WIN_MS window centred on each slow sample
lo_idx = np.searchsorted(ft, st - WIN_MS, side="left")
hi_idx = np.searchsorted(ft, st + WIN_MS, side="right")
I_bus = np.array([fi[a:b].mean() if b > a else 0.0
                  for a, b in zip(lo_idx, hi_idx)])
I_cell = I_bus / N_PAR                 # per-cell current, amps

t_s = (st - st[0]) / 1000.0
sag_mv = I_cell * DCIR_OHM * 1000.0    # expected per-cell sag, mV

# ---- OCV(SoC) from low-current samples -----------------------------------
rest = np.abs(I_bus) < LOW_I
ocv = np.interp(t_s, t_s[rest], vavg[rest])      # no-load pack voltage
off_min = np.median((vmin - vavg)[rest])         # static balance offsets
off_max = np.median((vmax - vavg)[rest])

exp_min = ocv + off_min - sag_mv
exp_max = ocv + off_max - sag_mv

# ---- implied per-cell DCIR (robust LSQ through the load points) ----------
load = I_bus > 30.0              # only meaningful current, avoids OCV artefacts
R_min = np.sum(((ocv + off_min - vmin)[load]) * I_cell[load]) / \
        np.sum(I_cell[load] ** 2) / 1000.0
R_max = np.sum(((ocv + off_max - vmax)[load]) * I_cell[load]) / \
        np.sum(I_cell[load] ** 2) / 1000.0

# best-fit expected curves using the data-implied resistance
fit_min = ocv + off_min - I_cell * R_min * 1000.0
fit_max = ocv + off_max - I_cell * R_max * 1000.0

resid_min = (vmin - exp_min)[load]   # measured below 13 mOhm model, load only
wb_local = int(np.argmin(resid_min))
wb = np.where(load)[0][wb_local]
print(f"samples: {len(t_s)}   peak bus current: {I_bus.max():.0f} A  "
      f"(=> {I_bus.max()/N_PAR:.0f} A/cell)")
print(f"peak expected sag: {sag_mv.max():.0f} mV per cell")
print(f"static offsets  min:{off_min:+.0f} mV  max:{off_max:+.0f} mV")
print(f"implied per-cell DCIR (load > 20 A)  min-cell: {R_min*1000:.1f} mOhm   "
      f"max-cell: {R_max*1000:.1f} mOhm   (nominal 13)")
print(f"worst excess sag on min cell: {resid_min[wb_local]:.0f} mV below "
      f"13 mOhm model at t = {t_s[wb]:.0f} s  (I_bus = {I_bus[wb]:.0f} A)")

# ---- plot: two stacked panels -------------------------------------------
fig, (axN, axX) = plt.subplots(2, 1, figsize=(13, 9), sharex=True)

# MIN cell
axN.plot(t_s, vmin, color="tab:red", lw=0.9, label="min cell — measured")
axN.plot(t_s, exp_min, color="black", lw=1.0, ls="--",
         label="expected: OCV − (I/4)·13 mΩ (nominal)")
axN.plot(t_s, fit_min, color="tab:blue", lw=1.0, ls=":",
         label=f"best-fit: OCV − (I/4)·{R_min*1000:.1f} mΩ")
axN.plot(t_s, ocv, color="gray", lw=0.8, alpha=0.6, label="OCV (no-load est.)")
axN.set_ylabel("Min cell voltage (mV)")
axN.set_title("Measured vs Expected (13 mΩ DCIR, 4P) Sag — Min Cell — "
              "Half Endurance Simulation (LOG0012)")
axN.legend(loc="lower left", fontsize=8, ncol=2)
axN.grid(True, alpha=0.3); axN.margins(x=0)
axN.annotate(f"implied min-cell DCIR ≈ {R_min*1000:.0f} mΩ (nominal 13)\n"
             f"worst excess sag {resid_min[wb]:.0f} mV @ t≈{t_s[wb]:.0f}s, "
             f"I_bus≈{I_bus[wb]:.0f} A",
             xy=(0.015, 0.06), xycoords="axes fraction", fontsize=9,
             bbox=dict(boxstyle="round", fc="wheat", alpha=0.85))

# MAX cell
axX.plot(t_s, vmax, color="tab:green", lw=0.9, label="max cell — measured")
axX.plot(t_s, exp_max, color="black", lw=1.0, ls="--",
         label="expected: OCV − (I/4)·13 mΩ (nominal)")
axX.plot(t_s, fit_max, color="tab:blue", lw=1.0, ls=":",
         label=f"best-fit: OCV − (I/4)·{R_max*1000:.1f} mΩ")
axX.plot(t_s, ocv, color="gray", lw=0.8, alpha=0.6, label="OCV (no-load est.)")
axX.set_ylabel("Max cell voltage (mV)")
axX.set_xlabel("Time (s)")
axX.set_title(f"Measured vs Expected (13 mΩ DCIR, 4P) Sag — Max Cell "
              f"(implied DCIR ≈ {R_max*1000:.0f} mΩ)")
axX.legend(loc="lower left", fontsize=8, ncol=2)
axX.grid(True, alpha=0.3); axX.margins(x=0)

fig.tight_layout()
fig.savefig("LOG0012_sag_model.png", dpi=150)
print("wrote LOG0012_sag_model.png")
