#!/usr/bin/env python3
"""Host test of the coil-inductance estimator: etc/measure_coil.py fit_l0() against its on-device copy
src/selftest/coil_fit.h, on synthetic DCM sweeps with a switch-node ring term.

Each dataset is a 48-step sweep (or 10 steps, the ring-unresolved fallback) of a buck at known L:
Iout = DCM transfer * (1 + ripple(t)), t = (1 - D*Vin/Vout)/fsw the dead time, ripple a zero-mean
Kepler-warped (sawtooth-like) damped sinusoid of 200..400 kHz with +-10..25 % amplitude, plus sensor noise and a
sagging Vin. Checks: Python and C++ agree on L and the ring frequency; the fit is within 3 % of the
true L where the median of the points is not.

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


def dataset(rng, steps):
    L = rng.uniform(40e-6, 100e-6)
    fr = rng.uniform(200e3, 400e3)
    amp, warp, ph = rng.uniform(0.1, 0.25), rng.uniform(0.0, 0.9), rng.uniform(0, 2 * math.pi)
    voc, vout = rng.uniform(62, 80), rng.uniform(25.5, 28.5)
    sag = rng.choice([0.0, 0.15])  # Vin drop over the band: stiff / low-light source
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
        io = (vin - vout) * vin * d * d / (2 * vout * FSW * L) * (1 + r)
        io = io * (1 + 0.005 * rng.gauss(0, 1)) + 0.003 * rng.gauss(0, 1)
        if io < 0.15:
            continue
        pts.append((d, vin + 0.03 * rng.gauss(0, 1), vout + 0.01 * rng.gauss(0, 1), io))
        if io > 2.0:
            break
    return L, fr, pts


def main():
    exe = "/tmp/coil-fit-test"
    src = [os.path.join(ROOT, "test/host-stub/coil-fit-test.cpp"), os.path.join(ROOT, "src/sim/vconv.cpp")]
    subprocess.run(["clang++", "-std=gnu++20", "-O2", "-I", os.path.join(ROOT, "src"), *src, "-o", exe], check=True)
    rng = random.Random(7)
    sets = [dataset(rng, 48) for _ in range(40)] + [dataset(rng, 10) for _ in range(10)]
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        for _, _, pts in sets:
            f.write(f"{FSW}\n")
            for p in pts:
                f.write(" ".join(f"{v:.9g}" for v in p) + "\n")
            f.write("\n")
        path = f.name
    out = subprocess.run([exe, "--fit", path], check=True, capture_output=True, text=True).stdout.split("\n")
    os.unlink(path)
    fails, e_fit, e_med = 0, [], []
    for (L, fr, pts), line in zip(sets, out):
        py = fit_l0(pts, FSW)
        cl, cf, cmed = (float(x) for x in line.split())
        if abs(cl / py["L"] - 1) > 1e-3 or (py["f"] > 0) != (cf > 0) or (cf > 0 and abs(cf / py["f"] - 1) > 2e-3):
            fails += 1
            print(f"  PARITY FAIL: py L {py['L'] * 1e6:.3f} f {py['f']:.0f}  c++ L {cl * 1e6:.3f} f {cf:.0f}")
        if len(pts) >= 30:
            e_fit.append(py["L"] / L - 1)
            e_med.append(py["Lmed"] / L - 1)
    rms = lambda e: math.sqrt(sum(x * x for x in e) / len(e))
    worst = max(abs(x) for x in e_fit)
    print(f"parity: {len(sets) - fails}/{len(sets)} datasets agree (L 0.1 %, f 0.2 %)")
    print(f"accuracy over {len(e_fit)} 48-step sweeps: median of points rms {rms(e_med) * 100:.1f} % "
          f"(worst {max(abs(x) for x in e_med) * 100:.1f} %), ring fit rms {rms(e_fit) * 100:.2f} % (worst {worst * 100:.1f} %)")
    ok = fails == 0 and worst < 0.03 and rms(e_fit) < rms(e_med)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
