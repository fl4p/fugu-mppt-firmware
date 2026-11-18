"""Share of the power residual's variance per band (Welch PSD of P = Iout*Vout after the 0.3 Hz high-pass)."""
import sys, numpy as np
sys.path.insert(0, "doc/research/filter-chain")
from chain_study import *
from scipy import signal

print(f"{'capture':55s} {'sd%':>6s} {'.3-2Hz':>7s} {'2-15Hz':>7s} {'15-60':>7s} {'lines':>6s} {'rest':>6s}")
for name, i, v, fs in caps(ROOT):
    n = min(len(i), len(v)); p = np.abs(i[:n]) * v[:n]
    pr = residual(p, fs); x = pr - pr.mean()
    f, P = signal.welch(x, fs=fs, nperseg=min(8192, len(x)))
    df = f[1]; tot = P.sum()
    band = lambda a, b: P[(f >= a) & (f < b)].sum() / tot
    hi = (f >= 60)
    # lines: bins >10x the local median in >=60 Hz
    med = signal.medfilt(P, 31)
    lines = P[hi & (P > 10 * med)].sum() / tot
    rest = P[hi].sum() / tot - lines
    print(f"{name[:55]:55s} {x.std() / pr.mean() * 100:6.2f} {band(.3, 2) * 100:6.1f}% {band(2, 15) * 100:6.1f}% "
          f"{band(15, 60) * 100:6.1f}% {lines * 100:5.1f}% {rest * 100:5.1f}%")
