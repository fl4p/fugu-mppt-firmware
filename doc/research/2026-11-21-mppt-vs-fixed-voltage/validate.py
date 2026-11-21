"""Check the Influx-derived sweep curves against (a) the device's own binned sweep curves (`sc:` lines) and
(b) the static two-point ratios of the periodic peak scan (`scan:` lines)."""
import re
import numpy as np
import pandas as pd
from analyze import OUT, s_at

LINE = re.compile(r'^(\S+ \S+) +(fry|flat): (.*)$')


def parse_log():
    sc, scans = [], []
    cur = None
    for ln in open(f'{OUT}/loglines.txt', errors='replace'):
        m = LINE.match(ln.rstrip('\n'))
        if not m:
            continue
        ts, dev, msg = m.groups()
        if msg.startswith('sc: t='):
            g = re.match(r'sc: t=(\d+) n=(\d+) mpp=([\d.]+),([\d.]+) sweeping=(\d)', msg)
            cur = dict(dev=dev, t=int(g[1]), n=int(g[2]), pmpp=float(g[3]), vmpp=float(g[4]), sweeping=int(g[5]), pts=[])
        elif msg.startswith('sc: end'):
            if cur and cur['dev'] == dev and not cur['sweeping'] and cur['t'] > 0:
                sc.append(cur)
            cur = None
        elif msg.startswith('sc: ') and cur and cur['dev'] == dev:
            xs = [float(x) for x in msg[4:].split()]
            cur['pts'] += list(zip(xs[0::2], xs[1::2]))
        g = re.search(r'scan: P ([\d.]+)->([\d.]+) W at ([\d.]+)V, (\d+) probes, best x([\d.]+) at ([\d.]+)V', msg)
        if g:
            scans.append(dict(dev=dev, time=pd.Timestamp(ts.replace(',', '.'), tz='UTC'), p0=float(g[1]),
                              p1=float(g[2]), v0=float(g[3]), n=int(g[4]), g=float(g[5]), v2=float(g[6])))
    sc = {(d['dev'], d['t']): d for d in sc}  # the dashboard re-polls the same curve
    return list(sc.values()), pd.DataFrame(scans)


def main():
    sc, scans = parse_log()
    out = {}
    for dev in ('fry', 'flat'):
        cv = pd.read_pickle(f'{OUT}/curves_{dev}.pkl')
        cv = cv[cv.valid].sort_values('t1').reset_index(drop=True)
        t1 = cv.t1.astype('int64') // 10**9 if cv.t1.dt.tz is not None else cv.t1.astype('int64') // 10**9
        # (a) sc: curves
        rows = []
        for d in (x for x in sc if x['dev'] == dev and x['n'] >= 20):
            k = int(np.argmin(np.abs(t1.values - d['t'])))
            if abs(t1.values[k] - d['t']) > 30:
                continue
            c = cv.loc[k]
            v = np.array([p[0] for p in d['pts']]); p = np.array([p[1] for p in d['pts']])
            ok = np.isfinite(p)
            s_dev = p[ok] / d['pmpp']
            s_inf = s_at(c, v[ok])
            near = np.abs(v[ok] - d['vmpp']) < 8  # +-8 V around the MPP
            rows.append(dict(t=d['t'], pmpp_dev=d['pmpp'], pmpp_inf=c.pmax, vmpp_dev=d['vmpp'], vmpp_inf=c.vmpp,
                             mad_all=np.nanmean(np.abs(s_inf - s_dev)), mad_near=np.nanmean(np.abs(s_inf - s_dev)[near])))
        a = pd.DataFrame(rows)
        # (b) peak-scan two-point ratios
        sv = scans[scans.dev == dev].copy()
        pred, age = [], []
        for _, s in sv.iterrows():
            prev = cv[cv.t1 <= s.time]
            if prev.empty:
                pred.append(np.nan); age.append(np.nan); continue
            c = prev.iloc[-1]
            a0, a2 = s_at(c, [s.v0, s.v2])
            pred.append(a2 / a0 if a0 > 0 else np.nan)
            age.append((s.time - c.t1).total_seconds() / 60)
        sv['pred'] = pred; sv['age'] = age
        sv['steady'] = (sv.p1 - sv.p0).abs() < 0.05 * sv.p0
        out[dev] = (a, sv)
    return out


if __name__ == '__main__':
    for dev, (a, sv) in main().items():
        print(dev, 'sc curves matched', len(a))
        if len(a):
            print(a.assign(dP=(a.pmpp_inf / a.pmpp_dev - 1), dV=a.vmpp_inf - a.vmpp_dev)[['dP', 'dV', 'mad_all', 'mad_near']].describe().round(3))
        f = sv[(sv.age < 30) & sv.steady & sv.pred.notna()]
        print(dev, 'scans', len(sv), 'usable (curve <30 min, steady P)', len(f))
        if len(f):
            e = f.pred - f.g
            print('  measured x mean %.3f, predicted %.3f, mean err %+.3f, MAE %.3f, corr %.2f' %
                  (f.g.mean(), f.pred.mean(), e.mean(), e.abs().mean(), np.corrcoef(f.g, f.pred)[0, 1]))
