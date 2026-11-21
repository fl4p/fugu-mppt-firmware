"""Print the tables used in ../2026-11-21-mppt-vs-fixed-voltage.md and save ratio_vs_v.png next to this script."""
import os
import numpy as np
import pandas as pd
from analyze import evaluate, ratios, sky, VGRID, KGRID, OUT, s_at
import validate

HERE = os.path.dirname(os.path.abspath(__file__))


def tracking(r):
    """operating-point efficiency on fresh curves (1-10 min after the sweep, outside sweeps) and sweep dip cost"""
    u = r['u']
    nosw = u.st_max.fillna(4) < 5
    f = u[(u.age >= 1) & (u.age < 10) & nosw & u.s_op.notna()]
    eta_op = (f.P * f.s_op).sum() / f.P.sum()  # P-weighted mean of P/Pmpp at the operating point
    eta_op_med = f.s_op.median()
    # sweep cost: power in sweep bins vs the median of the neighbouring non-sweep bins
    b = r['b']
    P = b.P
    ref = P.where(b.st_max.fillna(4) < 5).rolling(7, center=True, min_periods=1).median()
    sw = b.st_max.fillna(4) >= 5
    dip = (ref[sw] - P[sw]).clip(lower=0).sum()
    sweep_cost = dip / (P.sum() + dip)
    return eta_op, eta_op_med, sweep_cost, len(f)


def best_daily(r):
    u, S, w = r['u'], r['S'], r['w']
    num = 0.0
    for _, idx in u.groupby(u.index.normalize()).indices.items():
        num += (S[idx] * w[idx, None]).sum(0).max()
    return num / w.sum()


def split(r, cats, ibest, jbest):
    u, S, SK, w = r['u'], r['S'], r['SK'], r['w']
    loss = w * (1 - S[:, ibest]); lossk = w * (1 - SK[:, jbest])
    rows = []
    for name, m in cats:
        m = np.asarray(m)
        if m.sum() == 0:
            continue
        fv, fk, _, W = ratios(r, m)
        rows.append(dict(category=name, E_share=W / w.sum(), ratio_bestV=fv[ibest], ratio_own_bestV=fv.max(),
                         own_bestV=VGRID[int(np.argmax(fv))], loss_share=loss[m].sum() / loss.sum(),
                         ratio_kVoc=fk[jbest], lossk_share=lossk[m].sum() / lossk.sum()))
    return pd.DataFrame(rows)


def temperature(r):
    """unshaded, bright, core-day curves: Vmpp drift and what a fixed V loses on them"""
    cv = r['cv'].copy()
    t1 = pd.DatetimeIndex(cv.t1)
    cv['hour'] = t1.hour
    ref = cv.pmax.quantile(0.95)
    m = (cv.npk <= 1) & (cv.vmpp >= 0.75 * cv.voc) & (cv.pmax > 0.3 * ref)
    c = cv[m].copy()
    # NTC (converter heatsink) is no cell temperature; Voc is the temperature-sensitive quantity we have
    A = np.c_[np.ones(len(c)), c.voc, np.log(c.pmax)]
    coef, *_ = np.linalg.lstsq(A, c.vmpp, rcond=None)
    pred = A @ coef
    r2 = 1 - ((c.vmpp - pred) ** 2).sum() / ((c.vmpp - c.vmpp.mean()) ** 2).sum()
    # curvature around the MPP
    def s_rel(x):
        return np.nanmean([s_at(row, row.vmpp * (1 + x)) for _, row in c.iterrows()])
    curv = {x: s_rel(x) for x in (-0.10, -0.05, 0.05, 0.10)}
    wk = c.groupby(pd.DatetimeIndex(c.t1).to_period('W')).agg(voc=('voc', 'median'), vmpp=('vmpp', 'median'), n=('voc', 'size'))
    return dict(n=len(c), vmpp_mean=c.vmpp.mean(), vmpp_std=c.vmpp.std(), voc_mean=c.voc.mean(), voc_std=c.voc.std(),
                k_mean=(c.vmpp / c.voc).mean(), k_std=(c.vmpp / c.voc).std(), coef=coef, r2=r2, curv=curv, weekly=wk)


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(11, 4))
    val = validate.main()
    for ax, dev in zip(axs, ('fry', 'flat')):
        r = evaluate(dev)
        u, cv = r['u'], r['cv']
        fv, fk, eta_all, W = ratios(r)
        i, j = int(np.argmax(fv)), int(np.argmax(fk))
        eta_op, eta_med, sweep_cost, nf = tracking(r)
        eta = eta_op * (1 - sweep_cost)
        big = cv[cv.pmax > 0.5 * cv.pmax.quantile(0.95)]
        voc_nom = big.voc.median()
        print(f'\n## {dev}')
        print(f'bins {r["b"].index.min()} .. {r["b"].index.max()}; curves valid {len(cv)}, multi-peak {(cv.npk > 1).sum()}, '
              f'shaded(npk>1 or Vmpp<0.75Voc) {((cv.npk > 1) | (cv.vmpp < 0.75 * cv.voc)).sum()}')
        print(f'E all {r["E_all"] / 1e3:.2f} kWh, limited {r["E_lim"] / 1e3:.2f} ({r["E_lim"] / r["E_all"]:.1%}), '
              f'time limited {r["b"].limited.mean():.1%}, no curve {r["E_nocurve"] / 1e3:.2f}, analysed {u.P.sum() * 10 / 3.6e6:.2f} kWh')
        print(f'Voc_nom (median Voc of curves >50% of P95) {voc_nom:.1f} V, Vmpp median {big.vmpp.median():.1f}')
        print(f'eta_op (fresh, n={nf}) {eta_op:.4f} (median s_op {eta_med:.3f}), sweep cost {sweep_cost:.4f}, '
              f'eta {eta:.4f}; eta incl. stale curves {eta_all:.4f}')
        rows = [('best fixed V (hindsight)', f'{VGRID[i]:.1f} V', fv[i]),
                ('per-day best fixed V', 'varies', best_daily(r))]
        for k in (0.76, 0.78, 0.80):
            V = k * voc_nom
            rows.append((f'{k:.2f} x Voc_nom', f'{V:.1f} V', np.interp(V, VGRID, fv)))
        rows.append(('k*Voc, best k', f'k={KGRID[j]:.2f}', fk[j]))
        for k in (0.70, 0.75, 0.80, 0.85):
            rows.append((f'k*Voc', f'k={k:.2f}', fk[int(np.argmin(np.abs(KGRID - k)))]))
        print('| strategy | setting | E/E(ideal MPP) | E/E(MPPT harvested) |')
        print('|---|---|---|---|')
        print(f'| MPPT (this firmware) | | {eta:.3f} | 1.000 |')
        for a, b_, x in rows:
            print(f'| {a} | {b_} | {x:.3f} | {x / eta:.3f} |')
        fresh = (u.age < 10).values
        fvf, fkf, _, _ = ratios(r, fresh)
        print(f'sensitivity, fresh curves only (age<10 min, {fresh.mean():.0%} of bins): best V {VGRID[int(np.argmax(fvf))]:.1f} '
              f'{fvf.max():.3f}, at global best V {fvf[i]:.3f}, kVoc best {fkf.max():.3f} (k={KGRID[int(np.argmax(fkf))]:.2f})')
        # splits
        hours = u.index.hour
        cats = [('clear days', u.dayclass == 'clear'), ('mixed days', u.dayclass == 'mixed'),
                ('overcast days', u.dayclass == 'overcast'),
                ('unshaded curve', ~u.shaded), ('shaded curve', u.shaded),
                ('dawn/dusk (first/last 90 min)', u.dawndusk), ('core day', ~u.dawndusk),
                ('before 13 UTC', hours < 13), ('13 UTC and later', hours >= 13),
                ('unshaded, core day, clear', (~u.shaded) & (~u.dawndusk) & (u.dayclass == 'clear')),
                ('unshaded, core day, all skies', (~u.shaded) & (~u.dawndusk))]
        sp = split(r, cats, i, j)
        print(sp.round(3).to_string(index=False))
        t = temperature(r)
        print(f"temperature subset n={t['n']}: Vmpp {t['vmpp_mean']:.1f}+-{t['vmpp_std']:.2f} V, Voc {t['voc_mean']:.1f}+-{t['voc_std']:.2f} V, "
              f"Vmpp/Voc {t['k_mean']:.3f}+-{t['k_std']:.3f}; Vmpp = {t['coef'][0]:.1f} + {t['coef'][1]:.2f}*Voc + {t['coef'][2]:.2f}*ln(Pmax), R2 {t['r2']:.2f}")
        print('curvature s(Vmpp*(1+x)):', {k: round(v, 4) for k, v in t['curv'].items()})
        print(t['weekly'].round(1).to_string())
        # where does the peak sit, by hour, for shaded curves (fry's afternoon)
        cvh = cv.assign(h=pd.DatetimeIndex(cv.t1).hour)
        print(cvh[cvh.pmax > 10].groupby('h').agg(n=('vmpp', 'size'), vmpp=('vmpp', 'median'), voc=('voc', 'median'),
                                                   shaded=('npk', lambda x: 0)).assign(
            shaded=cvh[cvh.pmax > 10].assign(sh=(cvh.npk > 1) | (cvh.vmpp < 0.75 * cvh.voc)).groupby('h').sh.mean()).round(2).T.to_string())
        a, sv = val[dev]
        f = sv[(sv.age < 30) & sv.steady & sv.pred.notna()]
        print(f'validation: sc curves {len(a)}, |dPmax| median {np.median(np.abs(a.pmpp_inf / a.pmpp_dev - 1)):.3f}, '
              f'|dVmpp| median {np.median(np.abs(a.vmpp_inf - a.vmpp_dev)):.2f} V, MAD(s) median {a.mad_all.median():.3f}; '
              f'scans {len(f)}: measured {f.g.mean():.3f} predicted {f.pred.mean():.3f}, bias {(f.pred - f.g).mean():+.3f}, MAE {(f.pred - f.g).abs().mean():.3f}')
        ax.plot(VGRID, fv / eta, label='fixed V')
        ax.axhline(1, color='k', lw=0.6)
        ax.set_ylim(0.5, 1.05); ax.set_xlim(40, 78)
        ax.set_title(f'{dev}: E(fixed V)/E(MPPT harvested)'); ax.set_xlabel('fixed input voltage [V]'); ax.grid(alpha=0.3)
        ax2 = ax.twiny(); ax2.plot(KGRID, fk / eta, color='C1', label='k*Voc'); ax2.set_xlabel('k (k*Voc tracker)', color='C1')
    plt.tight_layout()
    plt.savefig(os.path.join(HERE, 'ratio_vs_v.png'), dpi=90)


if __name__ == '__main__':
    main()
