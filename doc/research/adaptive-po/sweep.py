#!/usr/bin/env python3
# Adaptive P&O study sweep: every (variant, scenario, noise realisation) through `offload run` pinned to one host.
#   sweep.py <binary> <outdir> <host> <tag> [main|thscale] [jobs]
import itertools, os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

BIN, OUT, HOST, TAG = sys.argv[1:5]
WHICH = sys.argv[5] if len(sys.argv) > 5 else "main"
JOBS = int(sys.argv[6]) if len(sys.argv) > 6 else 14
OFS = [str(o) for o in range(0, 18, 2)]  # 9 offsets into the measured noise capture [s]
SEEDS = [str(s) for s in range(1, 10)]

VARIANTS = {
    "A": {"TK_VARIANT": "0"},
    "B2": {"TK_VARIANT": "1", "TK_K": "2"},
    "B3": {"TK_VARIANT": "1", "TK_K": "3"},
    "C2": {"TK_VARIANT": "2", "TK_K": "2"},
    "C3": {"TK_VARIANT": "2", "TK_K": "3"},
    "D2": {"TK_VARIANT": "3", "TK_K": "2"},
    "D3": {"TK_VARIANT": "3", "TK_K": "3"},
    # control: baseline with only the fast-mode thresholds scaled (A1 = identity re-run)
    "A1": {"TK_VARIANT": "0", "TK_FASTSCALE": "1"},
    "AF.33": {"TK_VARIANT": "0", "TK_FASTSCALE": "0.33"},
    "AF.2": {"TK_VARIANT": "0", "TK_FASTSCALE": "0.2"},
    "AF.1": {"TK_VARIANT": "0", "TK_FASTSCALE": "0.1"},
    # sibling's B (rev_k 2.5, rev_floor_w 0.05, AR(1)-corrected sigma) + this study's D / relative floor (nt binary)
    "Bs": {}, "BsD": {"TK_D": "1"}, "BsR1": {"TK_RELFLOOR": "0.01"}, "BsR2": {"TK_RELFLOOR": "0.02"},
    "BsR4": {"TK_RELFLOOR": "0.04"}, "BsR2D": {"TK_RELFLOOR": "0.02", "TK_D": "1"},
}
MAIN = ["A", "B2", "B3", "C2", "C3", "D2", "D3"]
FAST = ["A1", "AF.33", "AF.2", "AF.1"]
NT = ["Bs", "BsD", "BsR1", "BsR2", "BsR4", "BsR2D"]


def keys_lowtail():
    # sibling's tail case: wx-low with flat noise at gain 0.3
    return [("wx-low", f"flat-g0.3@{o}", {"SIM_NOISE": "flat", "SIM_NOISE_GAIN": "0.3", "SIM_NOISE_OFS": o})
            for o in ["0", "100", "200"] + [str(x) for x in range(2, 18, 2)]]


def measured(sc, prof, extra=None):
    return [(sc, f"{prof}@{o}", {"SIM_NOISE": prof, "SIM_NOISE_OFS": o, **(extra or {})}) for o in OFS]


def gauss(sc):
    return [(sc, f"seed{s}", {"SIM_SEED": s}) for s in SEEDS]


def nf(sc, extra=None):
    return [(sc, "nf", dict(extra or {}))]


def keys_main():
    k = []
    k += nf("wx-sun-fry") + measured("wx-sun-fry", "fry") + measured("wx-sun-fry", "fry-wander")
    k += [(sc, n + "@200W", {**e, "SIM_P": "200"}) for sc, n, e in measured("wx-sun-fry", "fry")]
    k += measured("wx-sun-flat", "flat")
    k += nf("wx-wob-fry") + measured("wx-wob-fry", "fry") + measured("wx-wob-fry", "fry-wander")
    k += measured("wx-wob-flat", "flat") + measured("wx-wob-flat", "flat-wander")
    k += measured("wx-ramp-fry", "fry")
    for sc in ("wx-dusk", "wx-dawn"):
        k += nf(sc) + measured(sc, "fry") + measured(sc, "flat")
    for sc in ("wx-low", "wx-low-slow", "wx-mid"):
        k += gauss(sc)
    k += measured("wx-low", "fry")
    k += nf("wx-cloud") + measured("wx-cloud", "fry") + measured("wx-cloud-cv", "fry")
    for sc in ("wx-sweep-fall", "wx-sweep-rise", "wx-sweep-shade", "wx-shade2", "wx-shade3", "wx-shade2-move",
               "wx-shade3-move", "wx-shade-cloud"):
        k += nf(sc) + measured(sc, "fry")
    return k


def keys_thscale():
    k = nf("wx-sun-fry") + measured("wx-sun-fry", "fry")
    k += [(sc, n + "@200W", {**e, "SIM_P": "200"}) for sc, n, e in nf("wx-sun-fry") + measured("wx-sun-fry", "fry")]
    k += [(sc, n + "@20W", {**e, "SIM_P": "20"}) for sc, n, e in nf("wx-sun-fry") + measured("wx-sun-fry", "fry")]
    out = []
    for s in ("0.25", "0.5", "1", "2", "4"):
        for sc, n, e in k:
            out.append((f"Ax{s}", sc, n, {**e, "TK_THSCALE": s}))
    return out


NUM = r"([-\d.]+)"
PAT = {
    "harvest": r"\| harvest " + NUM + " %",
    "trackloss": r"\| tracking loss " + NUM + " %",
    "revmin": r"\| " + NUM + " rev/min",
    "slow": r"slow " + NUM + " %,",
    "slowloss": r"slow-tracking loss " + NUM,
    "th_fast": r"theta % fast " + NUM, "th_slow": r"theta % fast [-\d.]+ slow " + NUM,
    "sg_fast": r"sigma-hat % fast " + NUM, "sg_slow": r"sigma-hat % fast [-\d.]+ slow " + NUM,
    "ds_fast": r"diff-sigma % fast " + NUM, "ds_slow": r"diff-sigma % fast [-\d.]+ slow " + NUM,
    "td": r"Td mean " + NUM,
    # diag sweep (sibling B + diagnosis knobs): "== ADAPT ... fixed rule" and "== NOISE" fields
    "fix_fast": r"fixed rule % fast " + NUM, "fix_slow": r"fixed rule % fast [-\d.]+ slow " + NUM,
    "nps_fast": r"samples/period fast " + NUM, "nps_slow": r"samples/period fast [-\d.]+ slow " + NUM,
    "nrate_fast": r"samples/s fast " + NUM, "nrate_slow": r"samples/s fast [-\d.]+ slow " + NUM,
    "early_fast": r"early % fast " + NUM, "early_slow": r"early % fast [-\d.]+ slow " + NUM,
    "rmsr_fast": r"rms early/late fast " + NUM, "rmsr_slow": r"rms early/late fast [-\d.]+ slow " + NUM,
}
DIAG = ["BsN", "BsCap", "BsH4", "BsS64"]
VARIANTS.update({"BsN": {}, "BsCap": {"TK_CAP": "1"}, "BsH4": {"TK_HUBER": "4"}, "BsS64": {"TK_SPAN": "64"},
                 "BsH4S64": {"TK_HUBER": "4", "TK_SPAN": "64"}})


KSCAN = ["BsN", "BsCap", "BsK0.7", "BsK1", "BsK1.4", "BsCapK1"]
VARIANTS.update({"BsK0.7": {"TK_K": "0.7"}, "BsK1": {"TK_K": "1.0"}, "BsK1.4": {"TK_K": "1.4"},
                 "BsCapK1": {"TK_K": "1.0", "TK_CAP": "1"}})


def keys_cloud():
    # high-noise cloud: flat noise x3 (x1 as reference)
    return [("wx-cloud", f"flat-g{g}@{o}", {"SIM_NOISE": "flat", "SIM_NOISE_GAIN": g, "SIM_NOISE_OFS": o})
            for g in ("1", "3") for o in OFS]


K2 = ["K1.4", "K2", "K2.5"]
VARIANTS.update({"K1.4": {"TK_K": "1.4"}, "K2": {"TK_K": "2.0"}, "K2.5": {"TK_K": "2.5"}})


def keys_120():
    # ADS1015-like 120 Hz tick, fry noise at every whole-second offset of the 18 s capture
    return [(sc, f"fry120@{o}", {"SIM_NOISE": "fry", "SIM_ADC_HZ": "120", "SIM_NOISE_OFS": str(o)})
            for sc in ("wx-dusk", "wx-low") for o in range(18)]


def keys_diag():
    k = keys_lowtail() + gauss("wx-low") + measured("wx-low", "fry")
    for sc in ("wx-dawn", "wx-dusk"):
        k += nf(sc) + measured(sc, "fry") + measured(sc, "flat")
    for sc in ("wx-sweep-rise", "wx-shade2", "wx-shade3", "wx-shade2-move", "wx-shade3-move", "wx-shade-cloud"):
        k += nf(sc) + measured(sc, "fry")
    k += measured("wx-wob-flat", "flat") + measured("wx-wob-fry", "fry")
    return k


def run(job):
    var, sc, nkey, env = job
    fn = os.path.join(OUT, "raw", f"{var}.{sc}.{nkey}.txt")
    if not (os.path.exists(fn) and "== ADAPT" in open(fn).read()):
        e = {**os.environ, "OFFLOAD_TAG": TAG, **env, **VARIANTS.get(var, {})}
        with open(fn, "w") as f:
            r = subprocess.run(["offload", "run", "--host", HOST, "--tag", TAG, "--", BIN, sc], env=e, stdout=f,
                               stderr=subprocess.STDOUT)
        if r.returncode:
            return f"{var}\t{sc}\t{nkey}\tERROR rc={r.returncode}"
    txt = open(fn).read()
    a = next((l for l in txt.splitlines() if l.startswith("== ADAPT")), "") + " " + \
        next((l for l in txt.splitlines() if l.startswith("== NOISE")), "")
    s = next((l for l in txt.splitlines() if l.startswith("== SUMMARY")), "")
    vals = []
    for k, p in PAT.items():
        m = re.search(p, a)
        vals.append(m.group(1) if m else "nan")
    wp = re.search(r"wrong-peak +([\d.]+) s", s)
    sw = re.search(r"sweeps +(\d+)", s)
    tr = re.search(r"trips (\d+)", s)
    vals += [wp.group(1) if wp else "nan", sw.group(1) if sw else "nan", tr.group(1) if tr else "nan"]
    return "\t".join([var, sc, nkey] + vals)


if __name__ == "__main__":
    os.makedirs(os.path.join(OUT, "raw"), exist_ok=True)
    if WHICH in ("main", "fast"):
        jobs = [(v, *x) for v, x in itertools.product(MAIN if WHICH == "main" else FAST, keys_main())]
    elif WHICH == "nt":
        jobs = [(v, *x) for v, x in itertools.product(NT, keys_main() + keys_lowtail())]
    elif WHICH in ("diag", "diag2"):
        jobs = [(v, *x) for v, x in itertools.product(DIAG if WHICH == "diag" else ["BsH4S64"], keys_diag())]
    elif WHICH == "kscan":
        jobs = [(v, *x) for v, x in itertools.product(KSCAN, keys_diag() + keys_cloud())]
    elif WHICH in ("k2", "k2A"):
        jobs = [(v, *x) for v, x in itertools.product(K2 if WHICH == "k2" else ["A"],
                                                       keys_diag() + keys_cloud() + keys_120())]
    elif WHICH == "cloudA":
        jobs = [("A", *x) for x in keys_cloud()]
    elif WHICH == "lowtail":
        jobs = [(v, *x) for v, x in itertools.product(["A", "AF.33"], keys_lowtail())]
    else:
        jobs = keys_thscale()
    hdr = "var\tscenario\tnoise\t" + "\t".join(PAT) + "\twrongpeak_s\tsweeps\ttrips"
    with ThreadPoolExecutor(JOBS) as ex, open(os.path.join(OUT, f"results-{WHICH}.tsv"), "w") as f:
        f.write(hdr + "\n")
        for i, line in enumerate(ex.map(run, jobs)):
            f.write(line + "\n")
            f.flush()
            if "ERROR" in line or i % 100 == 0:
                print(i, len(jobs), line, flush=True)
    print("done", len(jobs))
