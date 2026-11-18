"""Equal-latency comparison: firmware chain (notch>med5>ewma2, then period mean) vs a boxcar of the same
step latency. For each capture: t95 = time for the chain's period mean to reach 95 % of a step;
sd = std of a period/window mean around the residual's mean, relative to mean P (noise of P_n - P_ref is ~sqrt2*sd)."""
import sys, numpy as np
sys.path.insert(0, "doc/research/filter-chain")
from chain_study import *


def t95_periods(fn, L, n=3000):
    x = np.zeros(n); s0 = int(20 * L); x[s0:] = 1.0
    pm = period_means(fn(x), L)
    k = np.argmax(pm[20:] >= 0.95)
    return k + 1  # periods after the step until the first period mean >= 95 %


for name, i, v, fs in caps(ROOT):
    n = min(len(i), len(v)); i, v = np.abs(i[:n]), v[:n]
    p = i * v; f0n, _ = ripple_f0(p); pr = residual(p, fs)
    Lt = fs / TRK_HZ
    fa = lambda x: ewma2(med5(notch(x, f0n)))
    kA = t95_periods(fa, Lt)
    TA = kA * Lt  # samples from step to the first >=95 % decision
    sdA = period_means(fa(pr), Lt)[3:].std() / pr.mean()
    out = [f"{name:55s} t95={kA} periods ({TA / fs * 1000:4.0f} ms)  A sd={sdA * 100:6.3f}%"]
    for lab, fn in (("notch>med5", lambda x: med5(notch(x, f0n))), ("notch>despike", lambda x: despike(notch(x, f0n))),
                    ("raw", lambda x: x)):
        y = fn(pr)
        for frac in (1.0, 0.5):
            Lw = max(1, round(frac * TA * f0n)) / f0n  # whole ripple periods, latency <= TA
            sd = period_means(y, Lw)[3:].std() / pr.mean()
            out.append(f"   box[{lab}] {Lw / fs * 1000:4.0f} ms sd={sd * 100:6.3f}% ({sd / sdA:4.2f}x)")
    print("\n".join(out), flush=True)
