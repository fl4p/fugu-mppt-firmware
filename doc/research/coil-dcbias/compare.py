"""Compare KDM Sendust 125u/60u DC-bias laws: printed rows, dsdig readings, maglib, Kool Mu, Micrometals MS.

Run: /Users/fab/dev/pv/ee/datasheet-chart-digitizer/.venv/bin/python -I compare.py  (from this directory)
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(sys.argv[0]).resolve().parent
D = HERE / "dsdig"
HS = [5, 10, 20, 30, 50, 100]


def law(H, b, c, a=1.0):
    return 100.0 / (a + b * np.asarray(H, float) ** c)


def load(tag):
    return json.loads((D / f"{tag}-dcbias.json").read_text())


vec = load("ks184-vector")
ljf = load("ljf_ks-raster")
cat = load("catalog_p14-raster")
perms = vec["perms_left_to_right"]

laws = {
    125: {
        "KDM printed (KS184 sheet, Ljf_KS, catalog p.14)": (1.34e-3, 1.780),
        "maglib KDM_SendustKS_125 (current)": (1.34e-3, 1.78),
        "Magnetics Kool Mu 125u (p.63: a=.01 b=2.209e-5 c=1.636)": (2.209e-3, 1.636),
        "Micrometals MS-125 (2021 p.29: a=.01 b=7.88e-6 c=1.88)": (7.88e-4, 1.88),
    },
    60: {
        "KDM printed (KS184 sheet, Ljf_KS, catalog p.14)": (3.57e-4, 1.748),
        "maglib KDM_SendustKS_60 (current)": (3.57e-4, 1.748),
        "Magnetics Kool Mu 60u (p.63: a=.01 b=2.142e-6 c=1.855)": (2.142e-4, 1.855),
        "Micrometals MS-60 (2021 p.29: a=.01 b=2.15e-6 c=1.84)": (2.15e-4, 1.84),
    },
}

out = []
for mu in (125, 60):
    i = perms.index(float(mu))
    poly = np.array(vec["vector_polylines_HP"][i])
    o = np.argsort(poly[:, 0])
    lh, pp = np.log10(poly[o, 0]), poly[o, 1]
    vec_at = [float(np.interp(math.log10(h), lh, pp)) for h in HS]
    rows = [("dsdig vector trace (KS184 sheet, drawn curve)", vec_at)]
    for name, src in (("dsdig free fit, vector KS184", vec), ("dsdig free fit, raster Ljf_KS", ljf),
                      ("dsdig free fit, raster catalog p.14", cat)):
        ff = src["curves"][i]["free_fit"]
        rows.append((f"{name} (b={ff['b']:.3e}, c={ff['c']:.4f}, rms {ff['rms_pct']:.2f} %-pt)",
                     law(HS, ff["b"], ff["c"]).tolist()))
    for name, (b, c) in laws[mu].items():
        rows.append((name, law(HS, b, c).tolist()))
    out.append(f"\n### {mu}u  (% of initial permeability; H in Oe)\n")
    out.append("| source | " + " | ".join(f"{h} Oe" for h in HS) + " |")
    out.append("|---|" + "---|" * len(HS))
    for name, vals in rows:
        out.append(f"| {name} | " + " | ".join(f"{v:.1f}" for v in vals) + " |")
    pr = vec["curves"][i]["printed_resid_pct"]
    out.append(f"\nKS184 vector points vs printed row: rms {pr['rms']:.2f}, max {pr['max_abs']:.2f} %-pt "
               f"({len(vec['curves'][i]['points'])} levels 5..95 %).")
    out.append("\nVector points (H Oe, %mu): " + ", ".join(f"({h:.2f}, {p:g})" for h, p in vec["curves"][i]["points"]))

# Spec-table anchor on the KS184-125A sheet: 344.4 uH min at 50 Oe, 913.0 uH +-8 % at 0 A
r = 344.4 / 913.0
out.append(f"\nKS184-125A spec row: L(50 Oe) min 344.4 uH / L0 913.0 uH = {100 * r:.1f} %; "
           f"/ (0.92 * 913.0) = {100 * 344.4 / (0.92 * 913.0):.1f} %; printed law at 50 Oe = {law(50, 1.34e-3, 1.78):.1f} %.")
print("\n".join(out))
