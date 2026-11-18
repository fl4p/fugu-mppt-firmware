"""How much of P's 0.3..15 Hz content is explained by Vin (the tracker's own dither moves Vin; load/irradiance
wander at a fixed operating point does not). Band-pass P and Vin to 0.3..15 Hz, regress P on Vin and Vin^2."""
import sys, numpy as np
sys.path.insert(0, "doc/research/filter-chain")
from chain_study import *
from scipy import signal

for name, i, v, fs in caps(ROOT):
    d = os.path.join(ROOT, name)
    vin, fvin = load_ch(d, "vin")
    if vin is None: continue
    n = min(len(i), len(v)); p = np.abs(i[:n]) * v[:n]
    t = np.arange(n) / fs
    vi = np.interp(t, np.arange(len(vin)) / fvin, vin)
    b, a = signal.butter(2, [0.3 / (fs / 2), 15 / (fs / 2)], "band")
    pb = signal.filtfilt(b, a, p); vb = signal.filtfilt(b, a, vi)
    vq = signal.filtfilt(b, a, (vi - vi.mean()) ** 2)
    X = np.c_[vb, vq, np.ones(n)]
    k = slice(int(2 * fs), n - int(2 * fs))
    coef, *_ = np.linalg.lstsq(X[k], pb[k], rcond=None)
    r2 = 1 - (pb[k] - X[k] @ coef).var() / pb[k].var()
    print(f"{name[:55]:55s} P(0.3-15Hz) sd {pb[k].std() / p.mean() * 100:5.2f}%  Vin-explained {r2 * 100:5.1f}%  "
          f"left {np.sqrt(1 - r2) * pb[k].std() / p.mean() * 100:5.2f}%  Vin sd {vb[k].std() / vi.mean() * 100:.2f}%")
