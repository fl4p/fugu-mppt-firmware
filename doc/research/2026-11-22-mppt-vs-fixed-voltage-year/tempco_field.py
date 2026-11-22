"""Try to estimate the Voc temperature coefficient from the 30 November days: regress sweep Voc on the converter
NTC (board temperature, weak proxy for cell temperature) and ln(Pmax) (irradiance). Inputs: $OUT/curves_*.pkl,
bins_*.pkl from the 11-21 study."""
import os
import numpy as np
import pandas as pd

OUT = os.environ.get('OUT', '/tmp/mpptfix')
EXCL = {'flat': ('2026-11-12 19:30Z', '2026-11-13 08:50Z')}


def ols(X, y):
    A = np.c_[np.ones(len(y)), X]
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    r = y - A @ c
    s2 = (r ** 2).sum() / (len(y) - A.shape[1])
    se = np.sqrt(np.diag(s2 * np.linalg.inv(A.T @ A)))
    r2 = 1 - (r ** 2).sum() / ((y - y.mean()) ** 2).sum()
    return c, se, r2


for dev in ('fry', 'flat'):
    cv = pd.read_pickle(f'{OUT}/curves_{dev}.pkl')
    cv = cv[cv.valid].copy()
    if dev in EXCL:
        a, z = map(pd.Timestamp, EXCL[dev]); cv = cv[~((cv.t1 >= a) & (cv.t1 < z))]
    b = pd.read_pickle(f'{OUT}/bins_{dev}.pkl')
    ntc = b.ntc.dropna()
    # board temperature just before the sweep (mean of the 2 min before t0)
    cv['ntc'] = [ntc[(ntc.index >= t - pd.Timedelta('2min')) & (ntc.index < t)].mean() for t in cv.t0]
    cv['unsh'] = (cv.npk == 1) & (cv.vmpp >= 0.75 * cv.voc)
    p95 = cv.pmax.quantile(0.95)
    print(f'## {dev}: valid {len(cv)}, ntc range {cv.ntc.min():.1f}..{cv.ntc.max():.1f} C')
    for name, m in [('unshaded, all power', cv.unsh), ('unshaded, Pmax < 20 % P95 (low heating)', cv.unsh & (cv.pmax < 0.2 * p95)),
                    ('unshaded, Pmax > 30 % P95', cv.unsh & (cv.pmax > 0.3 * p95))]:
        c = cv[m & cv.ntc.notna()]
        if len(c) < 15:
            continue
        voc_ref = c.voc.median()
        co, se, r2 = ols(np.c_[c.ntc, np.log(c.pmax)], c.voc.values)
        print(f'  {name}: n={len(c)}, Voc = {co[0]:.1f} {co[1]:+.3f}(±{se[1]:.3f})*ntc {co[2]:+.2f}(±{se[2]:.2f})*lnP, R2 {r2:.2f}'
              f' -> dVoc/dT_ntc = {100 * co[1] / voc_ref:+.3f} ± {100 * se[1] / voc_ref:.3f} %/K')
        co1, se1, r21 = ols(c[['ntc']].values, c.voc.values)
        print(f'      ntc only: {100 * co1[1] / voc_ref:+.3f} ± {100 * se1[1] / voc_ref:.3f} %/K, R2 {r21:.2f}; corr(ntc, lnP) {np.corrcoef(c.ntc, np.log(c.pmax))[0, 1]:.2f}')
