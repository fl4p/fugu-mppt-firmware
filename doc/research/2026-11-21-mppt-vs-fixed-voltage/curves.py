"""Turn raw sweep windows into binned P-V curves. Writes $OUT/curves_<dev>.pkl (DataFrame, one row per sweep)."""
import os, pickle
import numpy as np
import pandas as pd

OUT = os.environ.get('OUT', '/tmp/mpptfix')
BIN = 0.5  # V


def extract(sw):
    df = sw['df'].dropna(subset=['Ui', 'P', 'pwm_duty']).reset_index(drop=True)
    if len(df) < 40:
        return None
    d = df.pwm_duty.values
    # ramp start: lowest duty within the first part of the window
    head = slice(0, max(10, len(df) // 3))
    i0 = int(np.argmin(d[head]))
    i1 = i0 + int(np.argmax(d[i0:]))
    r = df.iloc[i0:i1 + 1]
    if len(r) < 40 or (r.time.iloc[-1] - r.time.iloc[0]) < 3000:
        return None
    v, p = r.Ui.values, r.P.values
    # the input settles to Voc within ~0.1 s after the converter stops; take the highest Ui early in the ramp
    voc = float(np.max(v[:max(5, len(v) // 3)]))
    edges = np.arange(np.floor(v.min() / BIN) * BIN, v.max() + BIN, BIN)
    idx = np.digitize(v, edges)
    vb, pb = [], []
    for k in np.unique(idx):
        m = idx == k
        vb.append(np.median(v[m])); pb.append(np.median(p[m]))
    vb, pb = np.array(vb), np.array(pb)
    o = np.argsort(vb); vb, pb = vb[o], pb[o]
    ps = pd.Series(pb).rolling(3, center=True, min_periods=1).median().values
    k = int(np.argmax(ps)); pmax = float(ps[k])
    return dict(t0=pd.to_datetime(r.time.iloc[0], unit='ms', utc=True),
                t1=pd.to_datetime(r.time.iloc[-1], unit='ms', utc=True),
                duty0=int(d[i0]), voc=voc, vmin=float(vb[0]), vmpp=float(vb[k]), pmax=pmax,
                edge=bool(k <= 1), v=vb, s=np.clip(ps / pmax, 0, None) if pmax > 0 else ps, npts=len(r))


def peaks(v, s, prom=0.06, hmin=0.3):
    """local maxima of the normalised curve with prominence >= prom and height >= hmin"""
    out = []
    n = len(s)
    for i in range(n):
        if s[i] < hmin or (i > 0 and s[i] < s[i - 1]) or (i < n - 1 and s[i] < s[i + 1]):
            continue
        lo = s[:i][::-1]; hi = s[i + 1:]
        # prominence: drop to the lowest point before reaching a higher point on each side
        def drop(side):
            m = s[i]
            for x in side:
                if x > s[i]:
                    break
                m = min(m, x)
            else:
                return s[i] - m if len(side) else s[i]
            return s[i] - m
        pr = min(drop(lo), drop(hi))
        if pr >= prom:
            out.append((float(v[i]), float(s[i])))
    # merge peaks closer than 2 V
    merged = []
    for pk in out:
        if merged and pk[0] - merged[-1][0] < 2:
            if pk[1] > merged[-1][1]:
                merged[-1] = pk
        else:
            merged.append(pk)
    return merged


if __name__ == '__main__':
    for dev in ('fry', 'flat'):
        sweeps = pickle.load(open(f'{OUT}/sweeps_{dev}.pkl', 'rb'))
        rows = [c for c in map(extract, sweeps) if c]
        cv = pd.DataFrame(rows)
        cv['npk'] = [len(peaks(a, b)) for a, b in zip(cv.v, cv.s)]
        # valid: real power, full ramp from duty 0 down to below 0.65 Voc
        cv['valid'] = (cv.pmax > 3) & (cv.vmin < 0.65 * cv.voc) & (cv.duty0 == 0)
        cv.to_pickle(f'{OUT}/curves_{dev}.pkl')
        print(dev, len(sweeps), 'windows', len(cv), 'curves', int(cv.valid.sum()), 'valid',
              'multi-peak', int((cv.valid & (cv.npk > 1)).sum()))
