"""Summarise dither_slope.py output: per series median and inverse-variance mean of the offset, day-bootstrap 90 % CI,
and the implied loss c*(dV/V)^2. Usage: aggregate.py FILE.jsonl..."""
import json
import sys

import numpy as np

C = {'fry': 16.0, 'flat': 15.7}


def summary(rows):
    g = np.array([r['g_pct_per_V'] for r in rows]) / 100
    e = np.array([max(r['g_err'], 0.05) for r in rows]) / 100
    v = np.array([r['V'] for r in rows])
    c = C[rows[0]['dev']]
    dv = g * v ** 2 / (2 * c)
    w = 1 / e ** 2
    days = np.array([r['t'][:10] for r in rows])
    ud = np.unique(days)
    rng = np.random.default_rng(0)
    boot = []
    for _ in range(2000):
        pick = rng.choice(ud, len(ud))
        idx = np.concatenate([np.flatnonzero(days == d) for d in pick])
        boot.append(np.sum(w[idx] * dv[idx]) / np.sum(w[idx]))
    m = np.sum(w * dv) / np.sum(w)
    lo, hi = np.percentile(boot, [5, 95])
    loss = c * (m / v.mean()) ** 2
    return dict(n=len(rows), days=len(ud), P_med=round(float(np.median([r['P'] for r in rows]))),
                dV_median=round(float(np.median(dv)), 2), dV_wmean=round(float(m), 2),
                ci90=(round(float(lo), 2), round(float(hi), 2)), frac_pos=round(float(np.mean(dv > 0)), 2),
                implied_loss_pct=round(100 * loss, 2))


for f in sys.argv[1:]:
    rows = [json.loads(ln) for ln in open(f) if ln.strip()]
    print(f, summary(rows) if rows else 'no rows')
