import importlib.util
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("measure_coil", ROOT / "etc/measure_coil.py")
mc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mc)

# Ideal DCM buck at fixed HS on-time, in timer counts (160 MHz tick, the MCPWM resolution).
VIN, VO, L, FSW, IPK, VF = 60.0, 27.0, 83e-6, 39e3, 1.5, 0.8
PWM_MAX = 4103
TICK = FSW * PWM_MAX
T_ON = L * IPK / (VIN - VO)
T_ID = L * IPK / VO
IDEAL = T_ID * TICK


def iout(ls):
    t = ls / TICK
    q1 = 0.5 * IPK * T_ON
    if t <= T_ID:
        i1 = IPK - VO / L * t
        return (q1 + IPK * t - 0.5 * VO / L * t * t + 0.5 * i1 * i1 * L / (VO + VF)) * FSW
    tau = t - T_ID
    ineg = VO / L * tau
    return (q1 + 0.5 * IPK * T_ID - 0.5 * ineg * tau - 0.5 * ineg * ineg * L / (VIN + VF - VO)) * FSW


def sweep(noise=0.0, rng=None):
    """The tool's default sweep: 0.5..1.4 x ideal in 24 steps, stopping 20 % below the peak."""
    lo, hi = round(0.5 * IDEAL), round(1.4 * IDEAL)
    step = (hi - lo) // 23
    xs, ys, peak = [], [], -1e9
    for ls in range(lo, hi + 1, step):
        io = iout(ls) + (rng.gauss(0, noise) if noise else 0.0)
        xs.append(ls)
        ys.append(io)
        peak = max(peak, io)
        if io < 0.8 * peak and ls > IDEAL:
            break
    return xs, ys


def test_noiseless_peak_and_curvature():
    xs, ys = sweep()
    x0, _, a, b = mc.asym_peak_fit(xs, ys)
    assert abs(x0 - IDEAL) < 2          # a single parabola lands ~160 counts early here
    assert 0 < a < b
    lc = VO * VIN / (2 * b * FSW * PWM_MAX ** 2 * (VIN - VO))
    assert abs(lc / L - 1) < 0.02       # V_f is neglected in the curvature formula


def test_noisy_sweeps_stay_near_the_peak():
    rng = random.Random(1)
    errs = []
    for _ in range(60):
        xs, ys = sweep(0.001, rng)      # 1 mA on 0.245 A
        fit = mc.asym_peak_fit(xs, ys)
        assert fit is not None
        errs.append(fit[0] - IDEAL)
    mean = sum(errs) / len(errs)
    assert abs(mean) < 10
    assert max(abs(e) for e in errs) < 60


def test_too_few_points():
    assert mc.asym_peak_fit([1, 2, 3, 4], [1, 2, 1, 0]) is None
