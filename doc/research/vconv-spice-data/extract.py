#!/usr/bin/env python3
"""Extract the SPICE rows vconv-spice-check.cpp uses from /tmp/vconv-spice/results into small CSVs, and compare them
with the constants embedded in the check (exit 1 on a mismatch).
usage (repo root): doc/research/vconv-spice-data/extract.py doc/research/vconv-spice-data test/host-stub/vconv-spice-check.cpp"""
import json, os, re, sys

R = "/tmp/vconv-spice/results"
out, cpp = sys.argv[1], open(sys.argv[2]).read()
os.makedirs(out, exist_ok=True)
bad = 0


def close(a, b, tol):
    global bad
    ok = abs(a - b) <= tol
    if not ok:
        bad += 1
    return ok


# dawn: ripple amp, period, falling share, eta 1-3 W, eta 8-30 W for B and B-rp
ds = json.load(open(f"{R}/dawn_summary.json"))
rows = {}
for x in ds:
    if x["var"] in ("B", "B-rp"):
        rows.setdefault(x["set"], {})[x["var"]] = x
with open(f"{out}/dawn.csv", "w") as f:
    f.write("set,var,ripple_amp,ripple_period,frac_falling,eta_1_3W,eta_8_30W\n")
    for s in sorted(rows):
        for v in ("B", "B-rp"):
            if v in rows[s]:
                x = rows[s][v]
                g = lambda k, n: "" if x.get(k) is None else f"{x[k]:.{n}f}"
                f.write(f"{s},{v},{g('ripple_amp', 4)},{g('ripple_period', 0)},{g('frac_neg_slope', 3)},"
                        f"{g('eta_1_3W', 4)},{g('eta_8_30W', 4)}\n")
for m in re.finditer(r'\{"(fry|flat)", (\d+), ([^}]*)\}', cpp.split("kDawnRef[]")[1].split("};")[0]):
    s = f"{m.group(1)}-{m.group(2)}"
    v = [float(t.strip().rstrip("f")) if "NAN" not in t else None for t in m.group(3).split(",")]
    b, rp = rows[s]["B"], rows[s].get("B-rp")
    ref = [b["ripple_amp"], b["ripple_period"], b["frac_neg_slope"], b["eta_1_3W"], b["eta_8_30W"]]
    ref += [rp["ripple_amp"], rp["frac_neg_slope"], rp["eta_1_3W"], rp["eta_8_30W"]] if rp else [None] * 4
    for k, (e, r) in enumerate(zip(v, ref)):
        if e is None or r is None or r != r:
            if (e is None) != (r is None or r != r):
                print("dawn", s, k, "embedded", e, "json", r)
                bad += 1
            continue
        if not close(e, r, 0.006 if k != 1 else 30):
            print("dawn", s, k, "embedded", e, "json", r)

# near boundary and P2: SPICE B Pout per HS:LS
db = json.load(open(f"{R}/dawn_b.json"))
near = [(x["board"], x["hs"], x["ls"], x["pout"]) for x in db if x["var"] == "B" and x["vin"] == 30]
nmap = {(b, h, l): p for b, h, l, p in near}
emb = re.findall(r'\{"(fry|flat)", (\d+), (\d+), ([\d.]+)f\}', cpp.split("kNearRef[]")[1].split("};")[0])
with open(f"{out}/near_boundary.csv", "w") as f:
    f.write("board,vin,vout,hs,ls,pout_W\n")
    for b, h, l, p in emb:
        r = nmap[(b, int(h), int(l))]
        f.write(f"{b},30,26.5,{h},{l},{r:.3f}\n")
        if not close(float(p), r, 0.002):
            print("near", b, h, l, p, r)
p2 = json.load(open(f"{R}/p2.json"))["b"]
pmap = {(x["board"], x["hs"], x["ls"]): x["pout"] for x in p2}
emb = re.findall(r'\{"(fry|flat)", (\d+), (\d+), (\d+), ([\d.]+)f\}', cpp.split("kP2Ref[]")[1].split("};")[0])
with open(f"{out}/p2.csv", "w") as f:
    f.write("board,vin,vout,hs,ls,pout_W\n")
    for b, v, h, l, p in emb:
        r = pmap[(b, int(h), int(l))]
        f.write(f"{b},{v},26.5,{h},{l},{r:.2f}\n")
        if not close(float(p), r, 0.01):
            print("p2", b, h, l, p, r)

# CCM: eta B and the avalanche loss (W) where the table lists it among the dominant terms
with open(f"{out}/ccm.csv", "w") as f:
    f.write("point,vin,iout_A,pout_W,eta_B_pct,avalanche_W\n")
    for ln in open(f"{R}/ccm_table.md"):
        c = [t.strip() for t in ln.strip().strip("|").split("|")]
        if len(c) < 12 or not c[1].isdigit():
            continue
        av = re.search(r"avalanche ([\d.]+)", c[12])
        f.write(f"{c[0]},{c[1]},{c[2]},{c[3]},{c[7]},{av.group(1) if av else ''}\n")
print("mismatches:", bad)
sys.exit(1 if bad else 0)
