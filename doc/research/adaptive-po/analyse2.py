#!/usr/bin/env python3
# All variants against A, paired by (scenario, noise realisation): per-case mean and worst run, groups, worst runs.
#   analyse2.py <dir with results-main.tsv, results-fast.tsv, results-nt.tsv, results-lowtail.tsv>
import sys
import pandas as pd

D = sys.argv[1]
d = pd.concat([pd.read_csv(f"{D}/results-{w}.tsv", sep="\t") for w in ("main", "fast", "nt", "lowtail")])
d = d[d.harvest.notna()].drop_duplicates(["var", "scenario", "noise"])
a = d[d["var"] == "A"].set_index(["scenario", "noise"])
d = d[d["var"] != "A"].copy()
d["dh"] = d.harvest.values - a.loc[list(zip(d.scenario, d.noise))].harvest.values
d["dtrips"] = d.trips.values - a.loc[list(zip(d.scenario, d.noise))].trips.values
d["nz"] = d.noise.str.replace(r"@\d+(?=@|$)", "", regex=True).str.replace(r"seed\d+", "gauss", regex=True)
d["case"] = d.scenario + " " + d.nz
d["grp"] = d.case.map(lambda c: "wx-low*" if c.startswith("wx-low") else "dawn/dusk" if c.startswith(("wx-dawn", "wx-dusk"))
                      else "shade/sweep" if "shade" in c or "sweep" in c else "rest")
V = [v for v in ["B2", "C2", "D2", "AF.33", "AF.2", "AF.1", "Bs", "BsD", "BsR1", "BsR2", "BsR4", "BsR2D"] if v in set(d["var"])]
pd.set_option("display.width", 300), pd.set_option("display.max_rows", 500), pd.set_option("display.max_columns", 50)
mean = d.pivot_table(index="case", columns="var", values="dh", aggfunc="mean")[V]
mn = d.pivot_table(index="case", columns="var", values="dh", aggfunc="min")[V]
print("per case: mean / worst run, pp vs A")
print(pd.DataFrame({v: mean[v].map("{:+.2f}".format) + "/" + mn[v].map("{:+.2f}".format) for v in V}).to_string())
print("\nA harvest per case:")
a2 = a.reset_index()
a2["nz"] = a2.noise.str.replace(r"@\d+(?=@|$)", "", regex=True).str.replace(r"seed\d+", "gauss", regex=True)
print((a2.groupby(a2.scenario + " " + a2.nz).harvest.agg(["mean", "min", "max", "count"])).round(2).to_string())
print("\ngroups (runs): mean / min / max")
g = d.groupby(["var", "grp"]).dh.agg(["mean", "min", "max", "count"]).round(3)
print(g.loc[V].to_string())
print("\nmean of case means:", mean.mean().round(3).to_dict())
print("cases with mean < -0.1 pp:", {v: [f"{c} {x:+.2f}" for c, x in mean[v].items() if x < -0.1] for v in V})
for v in V:
    w = d[d["var"] == v].nsmallest(4, "dh")
    print(f"{v} worst runs:", "; ".join(f"{r.scenario} {r.noise} {r.dh:+.2f}" for r in w.itertuples()))
print("\nextra trips vs A (sum):", d.groupby("var").dtrips.sum().to_dict())
rv = d.pivot_table(index="case", columns="var", values="revmin", aggfunc="mean")[V]
rv.insert(0, "A", a2.groupby(a2.scenario + " " + a2.nz).revmin.mean())
print("\nreversals/min")
print(rv.round(1).to_string())
