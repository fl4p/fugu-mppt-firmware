#!/usr/bin/env python3
# k 1.4 / 2.0 / 2.5 (uncapped, sibling's estimator) vs A, incl. 120 Hz ADS-like cases; identity check vs the k scan.
#   analyse5.py <k2 results dir> <diag results dir (k scan)>
import sys
import numpy as np
import pandas as pd

K, G = sys.argv[1:3]
d = pd.concat([pd.read_csv(f"{K}/results-k2A.tsv", sep="\t"), pd.read_csv(f"{K}/results-k2.tsv", sep="\t")])
d = d.reset_index(drop=True)
d["nz"] = d.noise.str.replace(r"@\d+(?=@|$)", "", regex=True).str.replace(r"seed\d+", "gauss", regex=True)
d["case"] = d.scenario + " " + d.nz
ah = d[d["var"] == "A"].set_index(["scenario", "noise"]).harvest
d["dh"] = d.harvest.values - ah.loc[list(zip(d.scenario, d.noise))].values
V = ["A", "K1.4", "K2", "K2.5"]
NAME = {"A": "A", "K1.4": "k1.4", "K2": "k2.0", "K2.5": "k2.5"}
pd.set_option("display.width", 400), pd.set_option("display.max_rows", 1000), pd.set_option("display.max_columns", 60)

# identity: refreshed binary with explicit k vs the k scan's binary; A vs the earlier A runs
try:
    o = pd.read_csv(f"{G}/per-run-kscan.tsv", sep="\t")
    for new, old in (("K1.4", "BsK1.4"), ("K2.5", "BsN"), ("A", "A")):
        a = d[d["var"] == new].set_index(["scenario", "noise"]).harvest
        b = o[o["var"] == old].set_index(["scenario", "noise"]).harvest
        j = a.to_frame("n").join(b.to_frame("o"), how="inner")
        print(f"identity {new} vs k-scan {old}: {int((np.abs(j.n - j.o) < 1e-3).sum())}/{len(j)} equal")
except FileNotFoundError:
    pass


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
grp = d.case.map(lambda c: "120Hz dusk" if "fry120" in c and "dusk" in c else "120Hz low" if "fry120" in c
                 else "low" if c.startswith("wx-low") else "dawn/dusk" if c.startswith(("wx-dawn", "wx-dusk"))
                 else "cloud-g3" if "flat-g3" in c else "cloud-g1" if "flat-g1" in c else "sweep-rise" if "rise" in c
                 else "wob" if "wob" in c else "shade")
d["grp"] = grp
s = d[d["var"] != "A"].groupby(["grp", "var"]).dh.agg(["mean", "min", "count"]).round(2)
s.index = s.index.set_levels([s.index.levels[0], [NAME[v] for v in s.index.levels[1]]])
print("\ngroups: mean / worst run, pp vs A")
print(s.unstack(0).to_string())
print("\ngroup A harvest %:", d[d["var"] == "A"].groupby("grp").harvest.mean().round(2).to_dict())
for v in V[1:]:
    x = d[d["var"] == v]
    for gname in ("low", "120Hz dusk", "120Hz low", "cloud-g3"):
        g = x[x.grp == gname]
        w = g.loc[g.dh.idxmin()]
        print(f"{NAME[v]} {gname}: mean {g.dh.mean():+.2f} se {g.dh.std() / np.sqrt(len(g)):.2f} worst {w.scenario} "
              f"{w.noise} {w.dh:+.2f} (n={len(g)})")
    w = x.nsmallest(3, "dh")
    print(f"{NAME[v]} worst overall:", "; ".join(f"{r.scenario} {r.noise} {r.dh:+.2f}" for r in w.itertuples()))
d.sort_values(["scenario", "noise", "var"])[["var", "scenario", "noise", "harvest", "dh", "revmin", "th_fast",
                                            "th_slow"]].to_csv(f"{K}/per-run-k2.tsv", sep="\t", index=False,
                                                               float_format="%.3f")
