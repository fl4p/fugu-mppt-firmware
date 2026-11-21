"""MPPT vs fixed input voltage, from measured sweep curves (Influx ~50 Hz) weighted by 10 s energy bins.
Inputs in $OUT (default /tmp/mpptfix): bins_<dev>.pkl (fetch_bins.py), curves_<dev>.pkl (curves.py),
loglines.txt (grep of the console log on havan). Self-produced pickles only."""
import os, re, sys
import numpy as np
import pandas as pd

OUT = os.environ.get('OUT', '/tmp/mpptfix')
VGRID = np.arange(35, 80.01, 0.5)
KGRID = np.round(np.arange(0.66, 0.90, 0.01), 2)
MAX_AGE = pd.Timedelta('40min')
# flat ran a simulated converter overnight (constant Voc 50.0 V, 108 W around the clock)
EXCLUDE = {'flat': [('2026-11-12 19:30Z', '2026-11-13 08:50Z')]}


def _drop(df, dev, col=None):
    t = df.index if col is None else pd.DatetimeIndex(df[col])
    keep = np.ones(len(df), bool)
    for a, z in EXCLUDE.get(dev, []):
        keep &= ~((t >= pd.Timestamp(a)) & (t < pd.Timestamp(z)))
    return df[keep]


def s_at(c, V):
    """normalised power of curve c at voltage(s) V; 0 above Voc, NaN below the swept range"""
    V = np.asarray(V, float)
    r = np.interp(V, c.v, c.s, left=np.nan, right=0.0)
    r = np.where(V >= c.voc, 0.0, r)
    return r


def load(dev):
    b = pd.read_pickle(f'{OUT}/bins_{dev}.pkl')
    cv = pd.read_pickle(f'{OUT}/curves_{dev}.pkl')
    cv = _drop(cv[cv.valid], dev, 't1').sort_values('t1').reset_index(drop=True)
    b = _drop(b, dev)
    b = b[(b.P > 1) & b.Ui.notna() & (b.index.hour >= 6) & (b.index.hour < 19)].copy()
    b['lim_frac'] = b.lim_n.fillna(0) / b.n
    # battery/output-limited: a Vout/Iout/power limiter sampled in >=10 % of the bin's points, or CV/CC/CP mode seen
    b['limited'] = (b.lim_frac >= 0.1) & (b.lim_max.fillna(0) >= 1) | (b.st_min.fillna(4).between(1, 3) & (b.st_mean < 3.9))
    return b, cv


def assign(b, cv):
    """latest valid curve that ended before the bin (<= 40 min), else the next one within 15 min"""
    bi = b.reset_index().rename(columns={'time': 't'})
    c = cv[['t1']].copy(); c['ci'] = c.index
    back = pd.merge_asof(bi[['t']], c, left_on='t', right_on='t1', direction='backward', tolerance=MAX_AGE)
    fwd = pd.merge_asof(bi[['t']], c, left_on='t', right_on='t1', direction='forward', tolerance=pd.Timedelta('15min'))
    ci = back.ci.fillna(fwd.ci)
    b['ci'] = ci.values
    return b


_SKY = None


def sky():
    """Day class from both converters' summed power (same site): level = daily peak of 30-min means vs a
    15-day rolling max; roughness = sum |2nd difference| / sum of 5-min means, 08-16 UTC."""
    global _SKY
    if _SKY is None:
        t30 = t5 = None
        for dev in ('fry', 'flat'):
            b = _drop(pd.read_pickle(f'{OUT}/bins_{dev}.pkl'), dev)
            p = b.P.clip(lower=0)
            a30, a5 = p.resample('30min').mean().fillna(0), p.resample('5min').mean().fillna(0)
            t30 = a30 if t30 is None else t30.add(a30, fill_value=0)
            t5 = a5 if t5 is None else t5.add(a5, fill_value=0)
        t30 = t30[(t30.index.hour >= 6) & (t30.index.hour < 19)]
        t5 = t5[(t5.index.hour >= 8) & (t5.index.hour < 16)]
        g30, g5 = t30.index.normalize(), t5.index.normalize()
        d = pd.DataFrame({'peak': t30.groupby(g30).max()})
        d['level'] = d.peak / d.peak.rolling(15, center=True, min_periods=1).max()
        d['rough'] = (t5.shift(-1) - 2 * t5 + t5.shift(1)).abs().groupby(g5).sum() / t5.groupby(g5).sum()
        d['cls'] = np.where(d.level < 0.4, 'overcast', np.where((d.level >= 0.7) & (d.rough <= 0.25), 'clear', 'mixed'))
        _SKY = d
    return _SKY


def evaluate(dev):
    b, cv = load(dev)
    b = assign(b, cv)
    E_all = b.P.sum() * 10 / 3600
    E_lim = b.P[b.limited].sum() * 10 / 3600
    u = b[~b.limited]
    E_free = u.P.sum() * 10 / 3600
    E_nocurve = u.P[u.ci.isna()].sum() * 10 / 3600
    u = u[u.ci.notna()].copy()
    u['ci'] = u.ci.astype(int)
    # dawn/dusk: first/last 90 min of each day's production (P > 2 W)
    day = u.index.normalize()
    prod = b[b.P > 2]
    first = prod.index.to_series().groupby(prod.index.normalize()).min()
    last = prod.index.to_series().groupby(prod.index.normalize()).max()
    u['dawndusk'] = (u.index < day.map(first) + pd.Timedelta('90min')) | (u.index > day.map(last) - pd.Timedelta('90min'))
    u['dayclass'] = day.map(sky().cls).astype(str)

    n = len(u)
    S = np.full((n, len(VGRID)), np.nan)
    s_op = np.full(n, np.nan)
    voc = np.full(n, np.nan); vmpp = np.full(n, np.nan); npk = np.zeros(n, int)
    SK = np.full((n, len(KGRID)), np.nan)
    for ci, idx in u.groupby('ci').indices.items():
        c = cv.loc[ci]
        S[idx] = s_at(c, VGRID)[None, :]
        SK[idx] = s_at(c, KGRID * c.voc)[None, :]
        s_op[idx] = s_at(c, u.Ui.values[idx])
        voc[idx] = c.voc; vmpp[idx] = c.vmpp; npk[idx] = c.npk
    u['s_op'] = s_op; u['voc'] = voc; u['vmpp'] = vmpp; u['npk'] = npk
    t1 = pd.DatetimeIndex(cv.t1.values[u.ci.values])
    t1 = t1.tz_localize('UTC') if t1.tz is None else t1
    u['age'] = (u.index - t1).total_seconds() / 60
    # partial shading: a second peak in the curve, or the global peak moved to the lower substrings
    u['shaded'] = (u.npk > 1) | (u.vmpp < 0.75 * u.voc)
    # MPP power estimate: harvested power over the curve value at the operating point (floored, stale curves)
    u['pmpp'] = u.P / np.clip(np.nan_to_num(s_op, nan=1.0), 0.8, 1.0)
    w = u.pmpp.values
    S0 = np.nan_to_num(S, nan=0.0); SK0 = np.nan_to_num(SK, nan=0.0)
    return dict(dev=dev, b=b, cv=cv, u=u, S=S0, SK=SK0, w=w, E_all=E_all, E_lim=E_lim, E_free=E_free,
                E_nocurve=E_nocurve)


def ratios(r, mask=None):
    """E(fixed)/E(ideal MPP) for each V of VGRID and k of KGRID, plus tracking efficiency"""
    u, S, SK, w = r['u'], r['S'], r['SK'], r['w']
    m = np.ones(len(u), bool) if mask is None else np.asarray(mask)
    W = w[m].sum()
    fv = (S[m] * w[m, None]).sum(0) / W
    fk = (SK[m] * w[m, None]).sum(0) / W
    eta = u.P.values[m].sum() / W
    return fv, fk, eta, W


if __name__ == '__main__':
    for dev in ('fry', 'flat'):
        r = evaluate(dev)
        fv, fk, eta, W = ratios(r)
        i = int(np.argmax(fv)); j = int(np.argmax(fk))
        print(dev, 'best V %.1f ideal %.3f actual %.3f | best k %.2f ideal %.3f | eta %.3f' %
              (VGRID[i], fv[i], fv[i] / eta, KGRID[j], fk[j], eta))
