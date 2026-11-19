#!/usr/bin/env python3
# Uncapped sibling-B theta against the period-mean noise A measures on the same wx-low run (diff-sigma:
# consecutive period differences / sqrt 2, includes stepping): effective k = theta / (sqrt2 * sigma).
#   keff.py <main results dir> <diag results dir>
import sys
import pandas as pd

M, G = sys.argv[1:3]
o = pd.concat([pd.read_csv(f"{M}/results-{w}.tsv", sep="\t") for w in ("main", "lowtail")])
o = o[(o["var"] == "A") & (o.scenario == "wx-low")].copy()
o["nz"] = o.noise.str.replace(r"@\d+$", "", regex=True).str.replace(r"seed\d+", "gauss", regex=True)
p = pd.read_csv(f"{G}/per-run.tsv", sep="\t")
p = p[(p["var"] == "BsN") & (p.scenario == "wx-low")]
m = o.merge(p[["noise", "th_fast", "th_slow"]], on="noise", suffixes=("_A", "_Bs"))
m["k_fast"] = m.th_fast_Bs / (2 ** .5 * m.ds_fast)
m["k_slow"] = m.th_slow_Bs / (2 ** .5 * m.ds_slow)
print(m.groupby("nz")[["ds_fast", "th_fast_A", "th_fast_Bs", "k_fast", "ds_slow", "th_slow_A", "th_slow_Bs", "k_slow"]]
      .mean().round(2).to_string())
