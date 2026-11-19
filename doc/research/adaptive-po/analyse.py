#!/usr/bin/env python3
# Paired comparison of the adaptive P&O variants against A (same scenario and noise realisation).
#   analyse.py results-main.tsv
import sys
import pandas as pd

d = pd.read_csv(sys.argv[1], sep="\t")
d = d[d.harvest.notna()]
d["nz"] = d.noise.str.replace(r"@\d+(?=@|$)", "", regex=True).str.replace(r"seed\d+", "gauss", regex=True)
d["case"] = d.scenario + " " + d.nz
base = d[d["var"] == "A"].set_index(["scenario", "noise"])
rows = []
for (case, var), g in d.groupby(["case", "var"]):
    g = g.set_index(["scenario", "noise"])
    b = base.loc[g.index]
    dh = g.harvest - b.harvest
    rows.append(dict(case=case, var=var, n=len(g), harvest=g.harvest.mean(), d_mean=dh.mean(),
                     d_se=dh.std(ddof=1) / len(g) ** .5 if len(g) > 1 else float("nan"), d_min=dh.min(),
                     d_max=dh.max(), worse=int((dh < -0.1).sum()), rev=g.revmin.mean(), slow=g.slow.mean(),
                     wrong=g.wrongpeak_s.mean() - b.wrongpeak_s.mean(), trips=g.trips.sum()))
r = pd.DataFrame(rows)
pd.set_option("display.width", 250), pd.set_option("display.max_rows", 500)
piv = r.pivot(index="case", columns="var", values="d_mean").round(3)
piv.insert(0, "A_harvest", r[r["var"] == "A"].set_index("case").harvest.round(3))
piv["n"] = r[r["var"] == "A"].set_index("case").n
print(piv.to_string())
print()
for v in sorted(set(r["var"]) - {"A"}):
    s = r[r["var"] == v]
    print(f"{v}: mean of case deltas {s.d_mean.mean():+.3f} pp; cases worse by >2 SE and >0.1 pp: "
          + ", ".join(f"{c} {m:+.2f} ({lo:+.2f}..{hi:+.2f})" for c, m, se, lo, hi in
                      zip(s.case, s.d_mean, s.d_se, s.d_min, s.d_max) if m < -0.1 and (se != se or m < -2 * se)))
print()
print(r.pivot(index="case", columns="var", values="rev").round(1).to_string())
print()
r.to_csv(sys.argv[1].replace(".tsv", "-paired.tsv"), sep="\t", index=False, float_format="%.4f")
