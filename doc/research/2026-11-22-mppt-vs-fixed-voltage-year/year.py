"""Yearly energy of fixed-V, k*Voc and MPPT strategies vs a perfect MPP tracker, per device and scenario.
Prints the tables used in ../2026-11-22-mppt-vs-fixed-voltage-year.md (year_output.txt)."""
import itertools
import numpy as np
import pandas as pd
from model import (NOV, UGRID, KGRID, weather, calibrate, curves, in_window, gamma_pmp, bright, s_rel)

VOC_NOM = {'fry': 78.2, 'flat': 69.9}  # 11-21 study: median Voc of curves with Pmax > 50 % of P95
BETAS = (-0.0028, -0.0032, -0.0035)
MOUNTS = ('open_rack_glass_polymer', 'close_mount_glass_glass', 'insulated_back_glass_polymer')
MAIN = dict(site='lisbon', tilt=0.0, mount='close_mount_glass_glass', beta=-0.0032)
CAP_X = (1.0, 1.5)  # daily cap = factor x the largest measured November day (a guess)


def meas(dev):
    m = NOV[dev]
    fv, fk = np.array(m['fv']), np.array(m['fk'])
    return dict(fix=fv.max(), vbest=m['vgrid'][int(fv.argmax())], k=fk.max(), kbest=m['kgrid'][int(fk.argmax())],
                fix_un=np.max(m['fv_unshaded']), k_un=np.max(m['fk_unshaded']), vbest_un=m['vgrid'][int(np.argmax(m['fv_unshaded']))], eta=m['eta'],
                cap=max(m['daily_wh'].values()))


def ratio(E, S):
    return (E[:, None] * S).sum(0) / E.sum()


def run(dev, site, tilt, mount, beta, penalty='total', const_tc=None, rs_fix=None):
    """penalty: 'total' = field Nov total / model Nov clean (reproduces the field month), 'shading' = field total /
    field unshaded, None = clean model. const_tc: hold the cell temperature (irradiance-only drift)."""
    d = weather(site, tilt, mount)
    cal = calibrate(dev, d, beta, rs_fix)
    if const_tc is not None:
        d = d.assign(tc=const_tc)
    x, vref = cal['x'], cal['voc_ref']
    c = curves(x, beta, d.g.values, d.tc.values)
    nov = in_window(d.index)
    M = meas(dev)
    # absolute scale (only used for the daily cap): largest model day in the November window = largest field day
    E = c['pmp'] * M['cap'] / pd.Series(c['pmp'][nov]).groupby(d.index[nov].normalize()).sum().max()
    fv_nov, fk_nov = ratio(E[nov], c['S'][nov]), ratio(E[nov], c['SK'][nov])
    # non-temperature losses (shading, dawn/dusk, transients) from the field month, carried over as a factor
    pen_fix = {'total': M['fix'] / fv_nov.max(), 'shading': M['fix'] / M['fix_un'], None: 1.0}[penalty]
    pen_k = {'total': M['k'] / fk_nov.max(), 'shading': M['k'] / M['k_un'], None: 1.0}[penalty]
    fv, fk = ratio(E, c['S']), ratio(E, c['SK'])
    i, j = int(fv.argmax()), int(fk.argmax())
    month = d.index.month
    mon = []
    num_m = 0.0
    for mo in range(1, 13):
        mm = month == mo
        fvm = ratio(E[mm], c['S'][mm]); im = int(fvm.argmax()); num_m += fvm[im] * E[mm].sum()
        w = E[mm] / E[mm].sum()
        mon.append(dict(month=mo, E_kwh=E[mm].sum() / 1e3, tc=(w * d.tc.values[mm]).sum(), tamb=(w * d.tamb.values[mm]).sum(),
                        vmpp=(w * c['ump'][mm]).sum() * vref, voc=(w * c['uoc'][mm]).sum() * vref,
                        fix_year=fvm[i] * pen_fix, vbest_m=UGRID[im] * vref, fix_m=fvm[im] * pen_fix,
                        k_year=ratio(E[mm], c['SK'][mm])[j] * pen_k))
    r = dict(dev=dev, site=site, tilt=tilt, mount=mount, beta=beta, x=x, vref=vref, resid=cal['resid'],
             gamma=gamma_pmp(x, beta), eta=M['eta'], pen_fix=pen_fix, pen_k=pen_k,
             nov_fix=fv_nov.max(), nov_vbest=UGRID[int(fv_nov.argmax())] * vref, nov_k=fk_nov.max(),
             nov_kbest=KGRID[int(fk_nov.argmax())],
             fix=fv[i] * pen_fix, vbest=UGRID[i] * vref, fix_monthly=num_m / E.sum() * pen_fix,
             kbest=KGRID[j], k=fk[j] * pen_k, mon=pd.DataFrame(mon), E_year=E.sum() / 1e3,
             at_nov_v=fv[int(np.argmin(abs(UGRID * vref - M['vbest'])))] * pen_fix)
    for kk in (0.76, 0.80):
        r[f'k{kk:.2f}'] = fk[int(np.argmin(abs(KGRID - kk)))] * pen_k
        u = kk * VOC_NOM[dev] / vref
        r[f'nom{kk:.2f}'] = fv[int(np.argmin(abs(UGRID - u)))] * pen_fix
    # battery-limited sensitivity: daily delivered energy capped, per strategy
    day = d.index.normalize()
    for cx in CAP_X:
        cap = cx * M['cap']
        def capped(e):
            return pd.Series(e).groupby(day).sum().clip(upper=cap).sum()
        ideal = capped(E)
        # best fixed V and best k re-chosen under the cap (hindsight)
        fx = max((capped(E * c['S'][:, ii] * pen_fix), UGRID[ii] * vref) for ii in range(max(0, i - 60), min(len(UGRID), i + 121), 5))
        kx = max((capped(E * c['SK'][:, jj] * pen_k), KGRID[jj]) for jj in range(len(KGRID)))
        r[f'cap{cx}'] = dict(cap=cap, ideal_capped_share=ideal / E.sum(), fix=fx[0] / ideal, vfix=fx[1], k=kx[0] / ideal, kk=kx[1],
                            mppt=capped(E * M['eta']) / ideal,
                            days_capped=int((pd.Series(E).groupby(day).sum() > cap).sum()))
    r['_c'], r['_d'], r['_E'] = c, d, E
    return r


def validation(r):
    """model November window (TMY) vs the field month"""
    dev, d, c, vref = r['dev'], r['_d'], r['_c'], r['vref']
    b = bright(d, r['x'], r['beta'])
    k, cv, pmp, ump, uoc = s_rel(r['x'], r['beta'], b.g.values, b.tc.values)
    V, O = ump * vref, uoc * vref
    A = np.c_[np.ones(len(V)), O, np.log(pmp)]
    co, *_ = np.linalg.lstsq(A, V, rcond=None)
    m = NOV[dev]['bright']
    nov = in_window(d.index)
    days = pd.Series(r['_E'][nov]).groupby(d.index[nov].normalize()).sum()
    meas_days = pd.Series(NOV[dev]['daily_wh'])
    return dict(model=dict(vmpp_mean=V.mean(), vmpp_std=V.std(), voc_mean=O.mean(), voc_std=O.std(), k_mean=(V / O).mean(),
                           k_std=(V / O).std(), b=co[1], c=co[2], tc_med=np.median(b.tc), g_med=np.median(b.g),
                           day_mean_kwh=days.mean() / 1e3, day_max_kwh=days.max() / 1e3),
                meas=dict(vmpp_mean=m['vmpp_mean'], vmpp_std=m['vmpp_std'], voc_mean=m['voc_mean'], voc_std=m['voc_std'],
                          k_mean=m['k_mean'], k_std=m['k_std'], b=m['coef'][1], c=m['coef'][2],
                          day_mean_kwh=meas_days[meas_days > 100].mean() / 1e3, day_max_kwh=meas_days.max() / 1e3))


def known_case(dev):
    """constant 25 C cell, constant 800 W/m2: one curve all year, fixed V at its Vmpp must lose nothing"""
    d = weather('lisbon')
    cal = calibrate(dev, d, MAIN['beta'])
    n = 500
    c = curves(cal['x'], MAIN['beta'], np.full(n, 800.0), np.full(n, 25.0))
    fv = ratio(c['pmp'], c['S'])
    fine = curves(cal['x'], MAIN['beta'], np.array([800.0]), np.array([25.0]), ugrid=np.array([c['ump'][0]]))
    return fv.max(), UGRID[int(fv.argmax())] * cal['voc_ref'], c['ump'][0] * cal['voc_ref'], fine['S'][0, 0]


def rs_scan(dev, rss=(0.05, 0.08, 0.11, 0.14, 0.17, None)):
    """structural check: pin rs, refit the rest; compare the Nov irradiance slope c and Vmpp spread with the field.
    Returns rs interpolated to the field c."""
    print('rs | fit rms | Nov c | Nov Vmpp std | Nov Voc std | year best fixed | year k*Voc best')
    cs, rv = [], []
    for rs in rss:
        r = run(dev, **MAIN, rs_fix=rs)
        v = validation(r)['model']
        rms = np.sqrt(np.mean(np.asarray(r['resid']) ** 2))
        print(f'{r["x"][1]:.3f}{"" if rs is not None else " (free)"} | {rms:.4f} | {v["c"]:.2f} | {v["vmpp_std"]:.2f} | {v["voc_std"]:.2f} | '
              f'{r["fix"]:.4f} @ {r["vbest"]:.1f} V | {r["k"]:.4f} @ {r["kbest"]}')
        cs.append(v['c']); rv.append(r['x'][1])
    o = np.argsort(cs)
    return float(np.interp(NOV[dev]['bright']['coef'][2], np.array(cs)[o], np.array(rv)[o]))


def fmt(v):
    return f'{v:.3f}'


def main():
    pd.set_option('display.width', 200)
    res = {}
    for dev in ('fry', 'flat'):
        M = meas(dev)
        print(f'\n# {dev}  (field Nov: fixed {M["fix"]:.4f} @ {M["vbest"]} V, k*Voc {M["k"]:.4f} @ k={M["kbest"]}, '
              f'unshaded fixed {M["fix_un"]:.4f} @ {M["vbest_un"]} V, eta {M["eta"]:.4f}, max day {M["cap"]:.0f} Wh)')
        print(f'known case (25 C, 800 W/m2 constant): best fixed ratio {known_case(dev)}')
        grid = {}
        for mount, beta in itertools.product(MOUNTS, BETAS):
            grid[(mount, beta)] = run(dev, 'lisbon', 0.0, mount, beta)
        main_r = grid[(MAIN['mount'], MAIN['beta'])]
        res[dev] = main_r
        # calibration
        print('\n## calibration (Lisbon, horizontal)')
        print('mount | beta %/K | a_ref | rs | log10 rsh | Voc_ref V | fit resid (k, s-10,-5,+5,+10) | gamma_Pmp, beta_Vmp %/K')
        for (mount, beta), r in grid.items():
            print(f'{mount} | {100 * beta:.2f} | {r["x"][0]:.4f} | {r["x"][1]:.3f} | {r["x"][2]:.2f} | {r["vref"]:.1f} | '
                  f'{np.round(r["resid"], 3)} | {r["gamma"][0]:.3f}, {r["gamma"][1]:.3f}')
        # validation
        print('\n## validation: November window, model (TMY) vs field, bright unshaded')
        print('mount | beta | Vmpp mean/std | Voc mean/std | k mean/std | b, c (Vmpp=a+b*Voc+c*lnP) | Tc, G median | '
              'model Nov clean fixed ratio @V | field unshaded | Nov k*Voc model | day mean/max kWh')
        for (mount, beta), r in grid.items():
            v = validation(r); mo, me = v['model'], v['meas']
            print(f'{mount} | {100 * beta:.2f} | {mo["vmpp_mean"]:.1f}/{mo["vmpp_std"]:.2f} | {mo["voc_mean"]:.1f}/{mo["voc_std"]:.2f} | '
                  f'{mo["k_mean"]:.3f}/{mo["k_std"]:.3f} | {mo["b"]:.2f}, {mo["c"]:.2f} | {mo["tc_med"]:.1f}, {mo["g_med"]:.0f} | '
                  f'{r["nov_fix"]:.4f} @ {r["nov_vbest"]:.1f} | {M["fix_un"]:.4f} @ {M["vbest_un"]} | {r["nov_k"]:.4f} @ {r["nov_kbest"]} | '
                  f'{mo["day_mean_kwh"]:.2f}/{mo["day_max_kwh"]:.2f}')
        print(f'field | | {me["vmpp_mean"]:.1f}/{me["vmpp_std"]:.2f} | {me["voc_mean"]:.1f}/{me["voc_std"]:.2f} | '
              f'{me["k_mean"]:.3f}/{me["k_std"]:.3f} | {me["b"]:.2f}, {me["c"]:.2f} | | | | | {me["day_mean_kwh"]:.2f}/{me["day_max_kwh"]:.2f}')
        # yearly results across the mount x beta grid
        print('\n## year, Lisbon horizontal: E/E(ideal MPP), penalty from the field month applied')
        print('mount | beta | pen_fix | pen_k | best fixed V | Nov field best V | monthly fixed V | 0.80 Voc_nom | 0.76 Voc_nom | best k | k=0.80 | k=0.76 | MPPT')
        for (mount, beta), r in grid.items():
            print(f'{mount} | {100 * beta:.2f} | {r["pen_fix"]:.4f} | {r["pen_k"]:.4f} | {r["fix"]:.4f} @ {r["vbest"]:.1f} V | {r["at_nov_v"]:.4f} | '
                  f'{r["fix_monthly"]:.4f} | {r["nom0.80"]:.4f} | {r["nom0.76"]:.4f} | {r["k"]:.4f} @ {r["kbest"]} | '
                  f'{r["k0.80"]:.4f} | {r["k0.76"]:.4f} | {r["eta"]:.4f}')
        vals = {key: [r[key] for r in grid.values()] for key in ('fix', 'at_nov_v', 'fix_monthly', 'k', 'k0.80', 'k0.76', 'nom0.80', 'nom0.76', 'vbest')}
        print('band over mount x beta:', {k: (round(min(v), 4), round(max(v), 4)) for k, v in vals.items()})
        for pm in (None, 'shading'):
            r0 = run(dev, **MAIN, penalty=pm)
            print(f'main case, penalty {pm}: pen_fix {r0["pen_fix"]:.4f} pen_k {r0["pen_k"]:.4f}: fixed {r0["fix"]:.4f} @ {r0["vbest"]:.1f} V, '
                  f'at Nov V {r0["at_nov_v"]:.4f}, monthly {r0["fix_monthly"]:.4f}, '
                  f'k {r0["k"]:.4f} @ {r0["kbest"]}, k0.80 {r0["k0.80"]:.4f}, k0.76 {r0["k0.76"]:.4f}')
        # monthly, main case
        print('\n## monthly, main case (Lisbon, horizontal, close mount, -0.32 %/K)')
        mon = main_r['mon'].copy()
        print(mon.round(dict(E_kwh=1, tc=1, tamb=1, vmpp=1, voc=1, fix_year=4, vbest_m=1, fix_m=4, k_year=4)).to_string(index=False))
        # sensitivities, one at a time around the main case
        cases = [('main', MAIN), ('Evora', dict(MAIN, site='evora')), ('tilt 30 S', dict(MAIN, tilt=30.0)),
                 ('Evora, tilt 30 S', dict(MAIN, site='evora', tilt=30.0)),
                 ('open rack', dict(MAIN, mount='open_rack_glass_polymer')), ('insulated back', dict(MAIN, mount='insulated_back_glass_polymer')),
                 ('Evora, insulated, -0.35', dict(MAIN, site='evora', mount='insulated_back_glass_polymer', beta=-0.0035)),
                 ('cell temp held at 34 C (irradiance-only drift)', dict(MAIN, const_tc=34.0)),
                 ('rs pinned 0.05 (physical module Rs, worse curvature fit)', dict(MAIN, rs_fix=0.05))]
        print('\n## structural: effective series resistance')
        rs_c = rs_scan(dev)
        print(f'rs matching the field irradiance slope c: {rs_c:.3f}')
        cases.append((f'rs pinned {rs_c:.3f} (matches field c)', dict(MAIN, rs_fix=rs_c)))
        print('\n## sensitivity (one change from the main case)')
        print('case | best fixed V | ratio | at Nov field V | monthly fixed | best k | k*Voc | k=0.80 | E_year kWh')
        for name, kw in cases:
            r = run(dev, **kw)
            print(f'{name} | {r["vbest"]:.1f} | {r["fix"]:.4f} | {r["at_nov_v"]:.4f} | {r["fix_monthly"]:.4f} | {r["kbest"]} | {r["k"]:.4f} | {r["k0.80"]:.4f} | {r["E_year"]:.0f}')
        # battery cap, main case
        print('\n## daily energy cap, main case (cap = x * largest measured Nov day)')
        print('x | cap Wh | days capped | ideal kept | best fixed V | k*Voc, best k | MPPT')
        for cx in CAP_X:
            q = main_r[f'cap{cx}']
            print(f'{cx} | {q["cap"]:.0f} | {q["days_capped"]} | {q["ideal_capped_share"]:.3f} | {q["fix"]:.4f} @ {q["vfix"]:.1f} V | {q["k"]:.4f} @ {q["kk"]} | {q["mppt"]:.4f}')
    return res


if __name__ == '__main__':
    main()
