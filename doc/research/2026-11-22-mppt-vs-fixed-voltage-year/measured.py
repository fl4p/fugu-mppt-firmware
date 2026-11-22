"""Collect the November measurements the year model is calibrated against, from the 11-21 study's code and
its $OUT pickles. Writes nov_measured.json here."""
import json, os, sys
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '2026-11-21-mppt-vs-fixed-voltage'))
from analyze import evaluate, ratios, VGRID, KGRID, s_at  # noqa: E402
from report import tracking, temperature  # noqa: E402

out = {}
for dev in ('fry', 'flat'):
    r = evaluate(dev)
    u = r['u']
    fv, fk, _, _ = ratios(r)
    fv_un, fk_un, _, _ = ratios(r, ~u.shaded.values)
    eta_op, _, sweep_cost, _ = tracking(r)
    t = temperature(r)
    cv = r['cv']
    ref = cv.pmax.quantile(0.95)
    c = cv[(cv.npk <= 1) & (cv.vmpp >= 0.75 * cv.voc) & (cv.pmax > 0.3 * ref)]
    # daily harvest, all bins (limited included), days with a full record
    b = r['b']
    day = b.P.groupby(b.index.normalize()).sum() * 10 / 3600
    lim = b.P[b.limited].groupby(b.index[b.limited].normalize()).sum() * 10 / 3600
    out[dev] = dict(
        vgrid=VGRID.tolist(), fv=fv.tolist(), fv_unshaded=fv_un.tolist(), kgrid=KGRID.tolist(), fk=fk.tolist(),
        fk_unshaded=fk_un.tolist(), eta=float(eta_op * (1 - sweep_cost)), eta_op=float(eta_op), sweep_cost=float(sweep_cost),
        bright=dict(n=int(t['n']), voc_mean=float(t['voc_mean']), voc_std=float(t['voc_std']), vmpp_mean=float(t['vmpp_mean']),
                    vmpp_std=float(t['vmpp_std']), k_mean=float(t['k_mean']), k_std=float(t['k_std']),
                    coef=[float(x) for x in t['coef']], r2=float(t['r2']), curv={str(k): float(v) for k, v in t['curv'].items()},
                    pmax_p95=float(ref), pmax_median=float(c.pmax.median())),
        daily_wh={str(k.date()): float(v) for k, v in day.items()},
        daily_limited_wh={str(k.date()): float(v) for k, v in lim.items()})
    i, j = int(np.argmax(fv)), int(np.argmax(fk))
    print(dev, 'best V', VGRID[i], round(fv[i], 4), 'best k', KGRID[j], round(fk[j], 4), 'eta', round(out[dev]['eta'], 4),
          'unshaded best', VGRID[int(np.argmax(fv_un))], round(fv_un.max(), 4))
json.dump(out, open(os.path.join(HERE, 'nov_measured.json'), 'w'), indent=1)
