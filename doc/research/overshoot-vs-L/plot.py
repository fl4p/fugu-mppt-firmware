# plot.py SWEEP.txt OUT.png [title note]
import sys
import matplotlib
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt

src, png = sys.argv[1], sys.argv[2]
note = sys.argv[3] if len(sys.argv) > 3 else ""
d = pd.read_csv(src, sep=r"\s+", header=None, names=["C", "L", "hz", "ph", "trip", "t", "pk"], engine="python")
d = d[d.trip != "REJECT"].dropna(subset=["pk"])
d["pk"] = d.pk.astype(float)
nph = d.groupby(["C", "hz", "L"]).size().max()
g = d.groupby(["C", "hz", "L"]).pk.agg(["min", "mean", "max"]).reset_index()
col = {560: "#2a78d6", 470: "#eb6834"}
ink, ink2, surf = "#0b0b0b", "#52514e", "#fcfcfb"
fig, axs = plt.subplots(1, 3, figsize=(13, 4.8), sharey=True, facecolor=surf)
for ax, hz, ttl in zip(axs, [200, 450, 5000], ["200 Hz (fmetal expected_hz)", "450 Hz", "5 kHz (ADC-monitor trip)"]):
    ax.set_facecolor(surf)
    for C in [560, 470]:
        s = g[(g.C == C) & (g.hz == hz)]
        ax.fill_between(s.L, s["min"], s["max"], color=col[C], alpha=0.15, lw=0)
        ax.plot(s.L, s["max"], color=col[C], lw=2, marker="o", ms=5, label=f"Cout {C} µF, worst phase")
        ax.plot(s.L, s["mean"], color=col[C], lw=1, ls="--", label=f"Cout {C} µF, mean over phase")
    ax.axhline(60, color=ink2, lw=1, ls=":")
    ax.text(100, 60.6, "60 V: backflow FETs Q5–Q7", ha="right", va="bottom", fontsize=8, color=ink2)
    ax.axhline(29.87, color=ink2, lw=1, ls=":")
    ax.text(100, 30.4, "OV trip 1.03×29 V", ha="right", va="bottom", fontsize=8, color=ink2)
    ax.set_title(f"Vout sampled at {ttl}", fontsize=10, color=ink, loc="left")
    ax.set_xlabel("coil inductance L0 [µH] (DC-biased)", color=ink2)
    ax.grid(color="#e4e3df", lw=0.6)
    ax.set_xlim(26, 102)
    ax.set_ylim(28, 70)
    for sp in ["top", "right"]: ax.spines[sp].set_visible(False)
    for sp in ["left", "bottom"]: ax.spines[sp].set_color("#c9c8c2")
    ax.tick_params(colors=ink2)
axs[0].set_ylabel("peak Vout after battery opens [V]", color=ink2)
axs[0].legend(loc="center right", fontsize=8, frameon=False)
fig.suptitle(f"Battery disconnect at ~28 A (8S LFP, Vin 64 V, 39 kHz), mppt-sim, band = min…max over {nph} sample phases"
             + (f", {note}" if note else ""), fontsize=11, color=ink, x=0.01, ha="left")
fig.tight_layout()
fig.savefig(png, dpi=130, facecolor=surf)
print(g.pivot_table(index="L", columns=["hz", "C"], values="max").round(1).to_string())
print(g.pivot_table(index="L", columns=["hz", "C"], values="mean").round(1).to_string())
