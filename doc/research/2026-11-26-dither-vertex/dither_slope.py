"""Operating-point offset from the P&O dither itself (no sweep curve): per 60 s window, average each pwm_duty
plateau, fit P = b0 + b1*(t - tc) + b2*(V - Vm), and convert the normalised slope g = b2/P into an offset from the
local maximum with the sweep-measured curvature c (s = 1 - c*x^2, x = dV/Vmpp): dV = g*V^2/(2c).
dV > 0: the maximum lies above the operating point (tracker sits below Vmpp).
Usage: dither_slope.py DEVICE DAYS [PMIN PMAX] [MAXWIN_PER_DAY]"""
import datetime as dt
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '2026-11-21-mppt-vs-fixed-voltage'))
from influx import query  # noqa: E402

C = {'fry': 16.0, 'flat': 15.7}  # mean of the +-5 % curvature rows (2026-11-21 study)


def iso(ms):
    return dt.datetime.fromtimestamp(ms / 1000, dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%fZ')


def bright_minutes(dev, day, pmin, pmax):
    t0, t1 = f'{day}T05:00:00Z', f'{day}T16:00:00Z'
    s = query(f"SELECT mean(P), stddev(P), max(mppt_state), max(cv_lim_idx), count(P) FROM mppt "
              f"WHERE device='{dev}' AND time >= '{t0}' AND time < '{t1}' GROUP BY time(1m) fill(none)")
    out = []
    for t, p, sd, st, cv, n in (s[0]['values'] if s else []):
        if p is None or n < 1500 or not (pmin <= p < pmax) or sd is None or sd / p > 0.03:
            continue
        if st is not None and st == 5:  # sweep
            continue
        if cv is not None:  # a limiter shapes P(V), not the panel
            continue
        out.append(t)
    return out


def window_fit(dev, t):
    s = query(f"SELECT Ui, P, pwm_duty FROM mppt WHERE device='{dev}' AND time >= '{iso(t)}' AND time < '{iso(t + 60000)}'")
    if not s:
        return None
    a = np.array([[r[0], r[1], r[2], r[3]] for r in s[0]['values'] if None not in r[:4]], float)
    if len(a) < 1500:
        return None
    tt, v, p, d = a.T
    runs, i = [], 0
    while i < len(d):
        j = i
        while j < len(d) and d[j] == d[i]:
            j += 1
        if j - i >= 8:  # drop 3 settling samples (~60 ms) at the start of each plateau
            k = slice(i + 3, j)
            runs.append((tt[k].mean(), v[k].mean(), p[k].mean(), j - i - 3))
        i = j
    if len(runs) < 6:
        return None
    r = np.array(runs)
    tc, vm, pm = r[:, 0].mean(), r[:, 1].mean(), r[:, 2].mean()
    span = r[:, 1].max() - r[:, 1].min()
    if span < 0.3:
        return None
    X = np.column_stack([np.ones(len(r)), (r[:, 0] - tc) / 1000, r[:, 1] - vm])
    w = np.sqrt(r[:, 3])
    beta, *_ = np.linalg.lstsq(X * w[:, None], r[:, 2] * w, rcond=None)
    res = r[:, 2] - X @ beta
    cov = np.linalg.pinv((X * w[:, None]).T @ (X * w[:, None])) * (np.sum((res * w) ** 2) / max(1, len(r) - 3))
    g, ge = beta[2] / pm, np.sqrt(cov[2, 2]) / pm
    dv = g * vm ** 2 / (2 * C[dev])
    return dict(t=iso(t)[:19], P=round(pm, 1), V=round(vm, 2), span=round(span, 2), plateaus=len(r),
                g_pct_per_V=round(100 * g, 3), g_err=round(100 * ge, 3), dV=round(dv, 2))


def main():
    dev, days = sys.argv[1], int(sys.argv[2])
    pmin = float(sys.argv[3]) if len(sys.argv) > 3 else 300
    pmax = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
    maxw = int(sys.argv[5]) if len(sys.argv) > 5 else 30
    today = dt.date.today()
    rng = np.random.default_rng(1)
    for k in range(days, 0, -1):
        day = (today - dt.timedelta(days=k - 1)).isoformat()
        mins = bright_minutes(dev, day, pmin, pmax)
        if len(mins) > maxw:
            mins = sorted(rng.choice(mins, maxw, replace=False).tolist())
        for t in mins:
            f = window_fit(dev, t)
            if f:
                print(json.dumps(dict(dev=dev, **f)), flush=True)


if __name__ == '__main__':
    main()
