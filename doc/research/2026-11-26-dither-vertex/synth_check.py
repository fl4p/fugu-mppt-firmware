"""Known-offset check of the window fit: synthetic 48 Hz P&O dither on s = 1 - c*x^2 with an irradiance ramp and
noise; the fit must recover the planted offset. Usage: synth_check.py"""
import numpy as np

import dither_slope as D

rng = np.random.default_rng(3)
c, vmpp, pmax = 15.7, 57.0, 400.0
for off in (-2.0, 0.0, 1.0, 2.0, 3.0):
    est = []
    for _ in range(40):
        n = 2880
        tt = np.arange(n) * 20.8
        duty = np.repeat(2000 + np.cumsum(rng.choice([-2, 2], n // 40)), 40)[:n]
        v0 = vmpp - off
        v = v0 + (duty - duty.mean()) * 0.05 + rng.normal(0, 0.05, n)
        ramp = 1 + 0.01 * (tt / tt[-1] - 0.5)  # 1 % irradiance drift over the window
        p = pmax * ramp * (1 - c * ((v - vmpp) / vmpp) ** 2) + rng.normal(0, 3.0, n)
        rows = [[t, a, b, d] for t, a, b, d in zip(tt, v, p, duty)]

        def fake_query(q, rows=rows):
            return [{'values': [[r[0], r[1], r[2], r[3]] for r in rows]}]

        D.query = fake_query
        f = D.window_fit('flat', 0)
        if f:
            est.append(f['dV'])
    print(f'planted offset {off:+.1f} V -> median {np.median(est):+.2f} V, IQR '
          f'{np.percentile(est, 25):+.2f}..{np.percentile(est, 75):+.2f} (n={len(est)})')
