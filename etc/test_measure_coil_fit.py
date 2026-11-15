#!/usr/bin/env python3
"""Host test of the coil-inductance estimator: etc/measure_coil.py fit_l0() against its on-device copy
src/selftest/coil_fit.h, on synthetic DCM sweeps with a switch-node ring term.

Each dataset is a 48-step sweep (or 10 steps, the ring-unresolved fallback) of a buck at known L:
Iout = DCM transfer * (1 + ripple(t)), t = (1 - D*Vin/Vout)/fsw the dead time, ripple a zero-mean
Kepler-warped (sawtooth-like) damped sinusoid of 200..400 kHz with +-10..25 % amplitude, plus sensor noise and a
sagging Vin, an Iout zero uncertainty and an on-time offset bound per dataset. Checks: Python and C++ agree on L,
the ring frequency, the stated errors (noise, systematic) and the L(Ipk) split (slope, its noise and offset bound,
bins); the fit is within 3 % of the true L where the median of the points is not, and the actual error is of the
order of the stated one (rms ratio 0.5-2).

    .venv/bin/python3 etc/test_measure_coil_fit.py      # builds /tmp/coil-fit-test with clang++
"""
import math
import os
import random
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from measure_coil import fit_l0  # noqa: E402

FSW, PMAX = 39000.0, 4103


def dataset(rng, steps, inject=False):
    L = rng.uniform(40e-6, 100e-6)
    fr = rng.uniform(200e3, 400e3)
    amp, warp, ph = rng.uniform(0.1, 0.25), rng.uniform(0.0, 0.9), rng.uniform(0, 2 * math.pi)
    voc, vout = rng.uniform(62, 80), rng.uniform(25.5, 28.5)
    sag = rng.choice([0.0, 0.15])  # Vin drop over the band: stiff / low-light source
    zsig, ton = rng.uniform(0.002, 0.01), rng.uniform(30e-9, 200e-9)  # stated: zero uncertainty, on-time bound
    zoff, delta, gain = rng.gauss(0, zsig), rng.uniform(-ton, ton), rng.uniform(-0.01, 0.01)  # actual, within them
    if not inject:
        zoff = delta = gain = 0.0
    m = vout / voc
    lo, hi = max(2, round(0.25 * m * PMAX)), round(0.9 * m * PMAX)
    step = max(1, (hi - lo) // (steps - 1))
    pts = []
    for h in range(lo, hi + 1, step):
        d = h / PMAX
        vin = voc * (1 - sag * (h - lo) / (hi - lo))
        if d * vin / vout > 0.97:
            break
        t = (1 - d * vin / vout) / FSW
        # Kepler-warped ring (E - warp*sin E = w), the steady state of a ring whose zero crossing the
        # body diode delays: sin E is zero-mean over w, as the ring's effect on Iout
        w = 2 * math.pi * fr * t + ph
        e = w
        for _ in range(8):
            e -= (e - warp * math.sin(e) - w) / (1 - warp * math.cos(e))
        r = amp * math.exp(-t * 4e4) * math.sin(e)
        de = d + delta * FSW
        io = (vin - vout) * vin * de * de / (2 * vout * FSW * L) * (1 + r)
        io = io * (1 + gain + 0.005 * rng.gauss(0, 1)) + 0.003 * rng.gauss(0, 1) + zoff
        if io < 0.15:
            continue
        pts.append((d, vin + 0.03 * rng.gauss(0, 1), vout + 0.01 * rng.gauss(0, 1), io))
        if io > 2.0:
            break
    return L, fr, pts, (zsig, ton, inject)


def main():
    exe = "/tmp/coil-fit-test"
    src = [os.path.join(ROOT, "test/host-stub/coil-fit-test.cpp"), os.path.join(ROOT, "src/sim/vconv.cpp")]
    subprocess.run(["clang++", "-std=gnu++20", "-O2", "-I", os.path.join(ROOT, "src"), *src, "-o", exe], check=True)
    rng = random.Random(7)
    sets = [dataset(rng, 48) for _ in range(40)] + [dataset(rng, 10) for _ in range(10)] + \
           [dataset(rng, 48, True) for _ in range(60)]
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        for _, _, pts, (zsig, ton, _) in sets:
            f.write(f"{FSW} {zsig:.9g} {ton:.9g}\n")
            for p in pts:
                f.write(" ".join(f"{v:.9g}" for v in p) + "\n")
            f.write("\n")
        path = f.name
    out = subprocess.run([exe, "--fit", path], check=True, capture_output=True, text=True).stdout.split("\n")
    os.unlink(path)
    fails, e_fit, e_med, ratio = 0, [], [], []
    n_split = 0
    rel = lambda a, b, tol: abs(a / b - 1) <= tol if b == b else a != a
    for (L, fr, pts, (zsig, ton, inject)), line in zip(sets, out):
        py = fit_l0(pts, FSW, zsig, ton)
        v = [float(x) for x in line.split()]
        cl, cf, cmed, cse, csys, ck, ckse, cksys, cnb = v[:9]
        cb = [v[9 + 5 * b:14 + 5 * b] for b in range(int(cnb))]
        bad = abs(cl / py["L"] - 1) > 1e-3 or (py["f"] > 0) != (cf > 0) or (cf > 0 and abs(cf / py["f"] - 1) > 2e-3)
        bad |= not (rel(cse, py["se"], 0.05) and rel(csys, py["sys"], 0.05))  # stated errors 5 %
        pb = py["periods"] * 1.25  # bin count limit: float and double may round an integer edge differently
        edge = abs(pb - round(pb)) < 2e-3 and abs(len(cb) - len(py["bins"])) == 1
        if py["k"] == py["k"]:  # split: slope 0.05 %/A + 5 % of its se, se and offset bound 5 %, same bins
            n_split += 1
            bad |= (abs(ck - py["k"]) > 5e-4 + 0.05 * py["kse"] or abs(ckse / py["kse"] - 1) > 0.05
                    or abs(cksys / py["ksys"] - 1) > 0.05 or (len(cb) != len(py["bins"]) and not edge)
                    or any(abs(c[2] / q[2] - 1) > 1e-3 or abs(c[0] - q[0]) > 1e-3 or abs(c[3] / q[3] - 1) > 0.05
                           or abs(c[4] / q[4] - 1) > 0.05 for c, q in zip(cb, py["bins"]) if not edge))
        elif ck == ck:
            bad = True
        if bad:
            fails += 1
            print(f"  PARITY FAIL: py L {py['L'] * 1e6:.3f} f {py['f']:.0f} se {py['se']:.5f} sys {py['sys']:.5f} "
                  f"k {py['k']:.5f}+-{py['kse']:.5f}/{py['ksys']:.5f} bins {len(py['bins'])}  c++ L {cl * 1e6:.3f} f {cf:.0f} "
                  f"se {cse:.5f} sys {csys:.5f} k {ck:.5f}+-{ckse:.5f}/{cksys:.5f} bins {len(cb)}")
        if inject:
            if py["se"] == py["se"]:
                ratio.append((py["L"] / L - 1) / math.hypot(py["se"], py["sys"]))
        elif len(pts) >= 30:
            e_fit.append(py["L"] / L - 1)
            e_med.append(py["Lmed"] / L - 1)
    rms = lambda e: math.sqrt(sum(x * x for x in e) / len(e))
    worst = max(abs(x) for x in e_fit)
    print(f"parity: {len(sets) - fails}/{len(sets)} datasets agree (L 0.1 %, f 0.2 %; L(Ipk) split on {n_split})")
    print(f"accuracy over {len(e_fit)} 48-step sweeps: median of points rms {rms(e_med) * 100:.1f} % "
          f"(worst {max(abs(x) for x in e_med) * 100:.1f} %), ring fit rms {rms(e_fit) * 100:.2f} % (worst {worst * 100:.1f} %)")
    print(f"stated error: actual/stated rms {rms(ratio):.2f} over {len(ratio)} sweeps with zero, on-time (30-200 ns) and"
          f" gain errors drawn within the stated bounds (no node charge)")
    ok = fails == 0 and worst < 0.03 and rms(e_fit) < rms(e_med) and 0.5 < rms(ratio) < 2
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
