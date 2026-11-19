#!/usr/bin/env python3
# Diagnosis sweep: sibling B (BsN), capped (BsCap), Huber 4x (BsH4), span 64 (BsS64) against A, paired.
#   analyse3.py <main results dir> <diag results dir>
import sys
import pandas as pd

M, G = sys.argv[1:3]
old = pd.concat([pd.read_csv(f"{M}/results-{w}.tsv", sep="\t") for w in ("main", "lowtail", "nt")])
new = pd.concat([pd.read_csv(f"{G}/results-{w}.tsv", sep="\t") for w in ("diag", "diag2") if
                 __import__("os").path.exists(f"{G}/results-{w}.tsv")])
keys = set(zip(new.scenario, new.noise))
a = old[(old["var"] == "A").values & __import__("numpy").array([k in keys for k in zip(old.scenario, old.noise)])]
bs = old[(old["var"] == "Bs")].set_index(["scenario", "noise"]).harvest
bn = new[new["var"] == "BsN"].set_index(["scenario", "noise"]).harvest
common = bn.index.intersection(bs.index)
print(f"identity BsN (diag binary) vs Bs (earlier binary): {(bn[common] == bs[common]).sum()} of {len(common)} equal,"
      f" max |diff| {(bn[common] - bs[common]).abs().max():.3f}")
d = pd.concat([a, new]).reset_index(drop=True)
d["nz"] = d.noise.str.replace(r"@\d+(?=@|$)", "", regex=True).str.replace(r"seed\d+", "gauss", regex=True)
d["case"] = d.scenario + " " + d.nz
ah = a.set_index(["scenario", "noise"]).harvest
d["dh"] = d.harvest.values - ah.loc[list(zip(d.scenario, d.noise))].values
d["run"] = d.noise
V = [v for v in ["A", "BsN", "BsCap", "BsH4", "BsS64", "BsH4S64"] if v in set(d["var"])]
pd.set_option("display.width", 300), pd.set_option("display.max_rows", 1000), pd.set_option("display.max_columns", 60)
rows = []
for case, g in d.groupby("case"):
    r = {"case": case, "n": int((g["var"] == "A").sum()), "A %": round(g[g["var"] == "A"].harvest.mean(), 2)}
    for v in V[1:]:
        x = g[g["var"] == v]
        w = x.loc[x.dh.idxmin()]
        r[v] = f"{x.dh.mean():+.2f} / {w.dh:+.2f} ({w.run})"
    rows.append(r)
print("\nharvest vs A, pp: mean / worst run (run)")
print(pd.DataFrame(rows).set_index("case").to_string())
print("\ntheta/P % per case, mean [min..max] over runs, fast | slow")
rows = []
for case, g in d.groupby("case"):
    r = {"case": case}
    for v in V:
        x = g[g["var"] == v]
        r[v] = (f"{x.th_fast.mean():.2f} [{x.th_fast.min():.2f}..{x.th_fast.max():.2f}] | "
                f"{x.th_slow.mean():.2f} [{x.th_slow.min():.2f}..{x.th_slow.max():.2f}]")
    rows.append(r)
print(pd.DataFrame(rows).set_index("case").to_string())
print("\nestimator diagnostics (BsN), per case means: fixed-rule share %, samples/period, samples/s, early share %,"
      " rms early/late")
x = d[d["var"].isin(["BsN", "BsCap", "BsH4", "BsS64"])]
print(x.groupby(["case", "var"])[["fix_fast", "fix_slow", "nps_fast", "nps_slow", "nrate_fast", "nrate_slow",
                                   "early_fast", "early_slow", "rmsr_fast", "rmsr_slow"]].mean().round(2)
      .loc[[c for c in sorted(set(x.case)) if c.startswith("wx-low")]].to_string())
grp = d.case.map(lambda c: "wx-low" if c.startswith("wx-low") else "dawn/dusk" if c.startswith(("wx-dawn", "wx-dusk"))
                 else "sweep-rise" if "sweep-rise" in c else "wob" if "wob" in c else "shade")
print("\ngroups: mean / worst run (pp vs A)")
print(d[d["var"] != "A"].groupby([grp, "var"]).dh.agg(["mean", "min", "count"]).round(2).unstack(0).to_string())
for v in V[1:]:
    w = d[d["var"] == v].nsmallest(4, "dh")
    print(f"{v} worst:", "; ".join(f"{r.scenario} {r.noise} {r.dh:+.2f} (theta/P fast {r.th_fast:.1f} %)" for r in w.itertuples()))
cols = ["var", "scenario", "noise", "harvest", "dh", "th_fast", "th_slow", "fix_fast", "fix_slow", "nrate_fast",
        "nrate_slow", "early_fast", "rmsr_fast", "rmsr_slow", "revmin"]
d.sort_values(["scenario", "noise", "var"])[cols].to_csv(f"{G}/per-run.tsv", sep="\t", index=False, float_format="%.3f")
print(f"\nper-run table: {G}/per-run.tsv")
