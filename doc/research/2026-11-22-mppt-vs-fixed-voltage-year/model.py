"""Year model of the fry/flat panel strings: PVGIS TMY -> plane of array -> SAPM cell temperature -> single-diode
P-V curve per hour, calibrated to the November field curves (nov_measured.json).

Voltages are normalised to Voc_ref = Voc at 25 C, 1000 W/m2 (u = V / Voc_ref), the photocurrent to 1 A at
1000 W/m2. The diode parameters are relative too; Voc_ref in volts is set at the end so that the mean Voc of the
bright November hours equals the measured one."""
import json, os
import numpy as np
import pandas as pd
import pvlib
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
NOV = json.load(open(os.path.join(HERE, 'nov_measured.json')))
SITES = {'lisbon': (38.72, -9.14, 68.0), 'evora': (38.57, -7.91, 281.0)}
MOUNTS = pvlib.temperature.TEMPERATURE_MODEL_PARAMETERS['sapm']
UGRID = np.round(np.arange(0.45, 1.0801, 0.001), 3)
KGRID = np.round(np.arange(0.66, 0.9001, 0.005), 3)
ALPHA = 0.0005  # Isc tempco, 1/K (typical c-Si)
WIN = ('10-22', '11-21')  # November field window, month-day (inclusive)
XCURV = (-0.10, -0.05, 0.05, 0.10)


def weather(site, tilt=0.0, mount='close_mount_glass_glass'):
    lat, lon, alt = SITES[site]
    w = pd.read_csv(os.path.join(HERE, f'tmy_{site}.csv.gz'), parse_dates=['time']).set_index('time')
    # PVGIS hourly radiation is instantaneous at HH:10 (irradiance_time_offset 0.168 h)
    sp = pvlib.solarposition.get_solarposition(w.index + pd.Timedelta('10min'), lat, lon, alt)
    sp.index = w.index
    if tilt == 0:
        cosz = np.cos(np.radians(sp.apparent_zenith)).clip(lower=0)
        beam, diff = w.dni * cosz, w.dhi
        aoi = sp.apparent_zenith
    else:
        dni_extra = pvlib.irradiance.get_extra_radiation(w.index)
        am = pvlib.atmosphere.get_relative_airmass(sp.apparent_zenith)
        poa = pvlib.irradiance.get_total_irradiance(tilt, 180, sp.apparent_zenith, sp.azimuth, w.dni, w.ghi, w.dhi,
                                                    dni_extra=dni_extra, airmass=am, model='perez')
        beam, diff = poa.poa_direct, poa.poa_diffuse
        aoi = pvlib.irradiance.aoi(tilt, 180, sp.apparent_zenith, sp.azimuth)
    poa_global = (beam + diff).fillna(0).clip(lower=0)
    geff = (beam * pvlib.iam.physical(aoi) + diff).fillna(0).clip(lower=0)  # beam reflection loss only
    p = MOUNTS[mount]
    tc = pvlib.temperature.sapm_cell(poa_global, w.tamb, w.wind, p['a'], p['b'], p['deltaT'])
    d = pd.DataFrame({'g': geff, 'tc': tc, 'tamb': w.tamb})
    return d[d.g > 5]


def params(x, beta, g, tc):
    """single-diode parameters for irradiance g (W/m2) and cell temperature tc (C); x = (a_ref, rs, log10 rsh_ref)"""
    a_ref, rs, lrsh = x
    rsh_ref = 10 ** lrsh
    a = a_ref * (tc + 273.15) / 298.15
    voc1000 = 1 + beta * (tc - 25)  # Voc at 1000 W/m2 follows the chosen tempco exactly
    il1000 = 1 + ALPHA * (tc - 25)
    i0 = (il1000 - voc1000 / rsh_ref) / np.expm1(voc1000 / a)
    il = g / 1000 * il1000
    return il, i0, rs, rsh_ref * 1000 / g, a


def curves(x, beta, g, tc, ugrid=UGRID, kgrid=KGRID):
    """per hour: pmp, ump, uoc, s(ugrid) and s(k*uoc)"""
    g, tc = np.asarray(g, float), np.asarray(tc, float)
    il, i0, rs, rsh, a = params(x, beta, g, tc)
    mpp = pvlib.pvsystem.max_power_point(il, i0, rs, rsh, a, method='newton')
    pmp, ump = np.asarray(mpp['p_mp']), np.asarray(mpp['v_mp'])
    uoc = np.asarray(pvlib.pvsystem.v_from_i(0.0, il, i0, rs, rsh, a))

    def s_of(U):
        col = lambda z: np.asarray(z)[:, None]
        I = pvlib.pvsystem.i_from_v(U, col(il), col(i0), rs, col(rsh), col(a))
        return np.where(U < col(uoc), np.clip(U * I, 0, None), 0.0) / col(pmp)

    S = s_of(np.broadcast_to(ugrid[None, :], (len(g), len(ugrid))))
    SK = s_of(kgrid[None, :] * uoc[:, None])
    return dict(pmp=pmp, ump=ump, uoc=uoc, S=S, SK=SK)


def s_rel(x, beta, g, tc, xs=XCURV):
    """mean P/Pmax at Vmp*(1+x), and mean Vmp/Voc, like the field curvature table"""
    il, i0, rs, rsh, a = params(x, beta, g, tc)
    mpp = pvlib.pvsystem.max_power_point(il, i0, rs, rsh, a, method='newton')
    pmp, ump = np.asarray(mpp['p_mp']), np.asarray(mpp['v_mp'])
    uoc = np.asarray(pvlib.pvsystem.v_from_i(0.0, il, i0, rs, rsh, a))
    out = []
    for xx in xs:
        U = ump * (1 + xx)
        I = pvlib.pvsystem.i_from_v(U, il, i0, rs, rsh, a)
        out.append(np.mean(np.where(U < uoc, np.clip(U * I, 0, None), 0) / pmp))
    return np.mean(ump / uoc), np.array(out), pmp, ump, uoc


def in_window(idx):
    md = idx.strftime('%m-%d')
    return (md >= WIN[0]) & (md <= WIN[1])


def bright(d, x, beta):
    """November-window hours whose Pmp is above 30 % of the window's P95 (the field 'bright' subset)"""
    nov = d[in_window(d.index)]
    il, i0, rs, rsh, a = params(x, beta, nov.g.values, nov.tc.values)
    pmp = np.asarray(pvlib.pvsystem.max_power_point(il, i0, rs, rsh, a, method='newton')['p_mp'])
    return nov[pmp > 0.3 * np.quantile(pmp, 0.95)]


def calibrate(dev, d, beta, rs_fix=None):
    """fit (a_ref, rs, log10 rsh_ref) to the measured Vmp/Voc and curvature table on the bright November hours;
    rs_fix pins rs (structural sensitivity)"""
    m = NOV[dev]['bright']
    target = np.r_[m['k_mean'], [m['curv'][str(xx)] for xx in XCURV]]
    lo, hi, xs = np.array([0.01, 0.0, 0.0]), np.array([0.2, 0.6, 5.0]), np.array([0.01, 0.05, 0.5])
    free = np.ones(3, bool) if rs_fix is None else np.array([True, False, True])
    x = np.array([0.03, 0.1 if rs_fix is None else rs_fix, 2.0])
    for _ in range(2):  # the bright subset depends weakly on x
        b = bright(d, x, beta)

        def res(xf):
            xx = x.copy(); xx[free] = xf
            k, c, *_ = s_rel(xx, beta, b.g.values, b.tc.values)
            return np.r_[k, c] - target
        fit = least_squares(res, x[free], bounds=(lo[free], hi[free]), x_scale=xs[free])
        x[free] = fit.x
    k, c, pmp, ump, uoc = s_rel(x, beta, b.g.values, b.tc.values)
    voc_ref = m['voc_mean'] / np.mean(uoc)
    return dict(x=x, voc_ref=voc_ref, resid=fit.fun, k=k, curv=c, n_bright=len(b))


def gamma_pmp(x, beta):
    """Pmp and Vmp tempco at 1000 W/m2 between 25 and 50 C, %/K"""
    il, i0, rs, rsh, a = params(x, beta, np.array([1000.0, 1000.0]), np.array([25.0, 50.0]))
    mpp = pvlib.pvsystem.max_power_point(il, i0, rs, rsh, a, method='newton')
    p, v = np.asarray(mpp['p_mp']), np.asarray(mpp['v_mp'])
    return 100 * (p[1] / p[0] - 1) / 25, 100 * (v[1] / v[0] - 1) / 25
