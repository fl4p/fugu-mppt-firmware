#!/usr/bin/env python3
# k scan on the sibling's estimator, uncapped vs capped, against A (paired by scenario and noise realisation).
#   analyse4.py <main results dir> <diag results dir>
import sys
import numpy as np
import pandas as pd

M, G = sys.argv[1:3]
new = pd.read_csv(f"{G}/results-kscan.tsv", sep="\t")
old = pd.concat([pd.read_csv(f"{M}/results-{w}.tsv", sep="\t") for w in ("main", "lowtail")] +
                [pd.read_csv(f"{G}/results-cloudA.tsv", sep="\t")])
keys = set(zip(new.scenario, new.noise))
a = old[(old["var"] == "A").values & np.array([k in keys for k in zip(old.scenario, old.noise)])]
a = a.drop_duplicates(["scenario", "noise"])
d = pd.concat([a, new]).reset_index(drop=True)
d["nz"] = d.noise.str.replace(r"@\d+(?=@|$)", "", regex=True).str.replace(r"seed\d+", "gauss", regex=True)
d["case"] = d.scenario + " " + d.nz
ah = a.set_index(["scenario", "noise"]).harvest
d["dh"] = d.harvest.values - ah.loc[list(zip(d.scenario, d.noise))].values
V = ["A", "BsK0.7", "BsK1", "BsK1.4", "BsN", "BsCapK1", "BsCap"]
NAME = {"A": "A", "BsK0.7": "k0.7", "BsK1": "k1.0", "BsK1.4": "k1.4", "BsN": "k2.5", "BsCapK1": "cap k1.0",
        "BsCap": "cap k2.5"}
pd.set_option("display.width", 400), pd.set_option("display.max_rows", 1000), pd.set_option("display.max_columns", 60)


def table(f):
    rows = []
    for case, g in d.groupby("case"):
        r = {"case": case, "n": int((g["var"] == "A").sum())}
        for v in V:
            x = g[g["var"] == v]
            r[NAME[v]] = f(x) if len(x) else "-"
        rows.append(r)
    return pd.DataFrame(rows).set_index("case").to_string()


print("harvest: A in %, others mean / worst run (run) pp vs A")
print(table(lambda x: f"{x.harvest.mean():.2f}" if x["var"].iloc[0] == "A" else
            f"{x.dh.mean():+.2f} / {x.dh.min():+.2f} ({x.loc[x.dh.idxmin()].noise})"))
print("\nreversals/min")
print(table(lambda x: f"{x.revmin.mean():.0f}"))
print("\ntheta/P %, fast | slow (mean over runs)")
print(table(lambda x: f"{x.th_fast.mean():.2f} | {x.th_slow.mean():.2f}"))
grp = d.case.map(lambda c: "low" if c.startswith("wx-low") else "dawn/dusk" if c.startswith(("wx-dawn", "wx-dusk"))
                 else "cloud-g3" if "flat-g3" in c else "cloud-g1" if "flat-g1" in c else "sweep-rise" if "rise" in c
                 else "wob" if "wob" in c else "shade")
s = d[d["var"] != "A"].groupby([grp, "var"]).dh.agg(["mean", "min"]).round(2)
s.index = s.index.set_levels([s.index.levels[0], [NAME[v] for v in s.index.levels[1]]])
print("\ngroups: mean / worst run, pp vs A")
print(s.unstack(0).to_string())
for v in V[1:]:
    w = d[d["var"] == v].nsmallest(3, "dh")
    print(f"{NAME[v]} worst:", "; ".join(f"{r.scenario} {r.noise} {r.dh:+.2f}" for r in w.itertuples()))
d.sort_values(["scenario", "noise", "var"])[["var", "scenario", "noise", "harvest", "dh", "revmin", "th_fast",
                                            "th_slow"]].to_csv(f"{G}/per-run-kscan.tsv", sep="\t", index=False,
                                                                float_format="%.3f")
