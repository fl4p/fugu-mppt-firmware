"""Firmware sensor filter chain vs alternatives on stored adcscope captures (output-side P = Iout*Vout).

For each capture: noise residual (raw - 0.3 Hz zero-phase LP + mean), then per variant
  sd   = std of successive period-mean differences dP, relative to mean P   (tracker decision noise)
  g1   = fraction of a unit step at a period boundary that shows in the next dP (signal gain)
  snr  = g1 / sd  (higher = a duty step is easier to tell from noise)
  bias = mean(filtered) / mean(raw) - 1  (median under-read on pulsed loads)
"""
import glob, os, re, sys
import numpy as np
from scipy import signal

ROOT = sys.argv[1] if len(sys.argv) > 1 else "etc/adcscope/data"
TRK_HZ = 30.0
SPAN = 20


def load_ch(d, name):
    fs = [p for p in glob.glob(f"{d}/{name}*") if re.match(rf"{name}(_\d+(hz)?)?\.(npz|csv)$", os.path.basename(p))]
    if not fs:
        return None, None
    p = fs[0]
    m = re.search(r"_(\d+)(hz)?\.", os.path.basename(p))
    rate = float(m.group(1)) if m else 454.0
    if p.endswith(".npz"):
        v = np.load(p)["v"].astype(float)
    else:
        v = np.array([float(x) for x in open(p).read().split()[1:] if x.strip()], float)
    return v, rate


def notch_coeffs(f, gain_db=-30, Q=20):
    amp = np.sqrt(10 ** (gain_db / 20))
    w = 2 * np.pi * f
    s, c = np.sin(w), np.cos(w)
    al = s / (2 * Q); a0 = 1 + al
    b = np.array([(1 + al * amp), -2 * c, (1 - al * amp)]) / a0
    a = np.array([1, -2 * c / a0, (1 - al) / a0])
    return b, a


def notch(x, f0n):
    b, a = notch_coeffs(f0n)
    zi = signal.lfilter_zi(b, a) * x[0]
    return signal.lfilter(b, a, x, zi=zi)[0]


def med5(x):
    y = np.copy(x)
    for i in range(len(x)):
        w = x[max(0, i - 4):i + 1]
        y[i] = np.median(w)
    return y


def despike(x, k=8.0):
    m = med5(x); y = np.copy(x); sc = 0.0; trips = 0
    for i in range(len(x)):
        dev = abs(x[i] - m[i])
        if sc > 0 and dev > k * sc:
            y[i] = m[i]; trips += 1
        sc = sc + 0.095 * (dev - sc) if sc > 0 else dev
    return y


def ewma2(x, span=SPAN):
    al = 2 / (span + 1)
    b, a = [al], [1, -(1 - al)]
    y = signal.lfilter(b, a, x, zi=[x[0] * (1 - al)])[0]
    return signal.lfilter(b, a, y, zi=[y[0] * (1 - al)])[0]


def period_means(x, L, start=0.0):
    """means over consecutive windows of fractional length L samples (endpoint samples weighted)."""
    c = np.concatenate([[0], np.cumsum(x)])
    def S(t):  # integral of the sample-and-hold sequence up to t
        i = int(np.floor(t)); f = t - i
        return c[i] + (x[i] * f if i < len(x) else 0)
    out = []; t = start
    while t + L < len(x) - 1:
        out.append((S(t + L) - S(t)) / L); t += L
    return np.array(out)


def ripple_f0(x):
    f, p = signal.welch(x - x.mean(), fs=1.0, nperseg=min(4096, len(x)))
    band = (f > 0.12) & (f < 0.34)
    i = np.argmax(np.where(band, p, 0))
    return f[i], p[i] / np.median(p[band])


def variants(fs, f0n):
    Lt = fs / TRK_HZ
    k = max(1, round(Lt * f0n))
    Lr = k / f0n  # integer ripple periods, fractional samples
    return {
        "A fw now: notch>med5>ewma2>33ms": (lambda x: ewma2(med5(notch(x, f0n))), Lt),
        "B notch>med5>box(k ripple)": (lambda x: med5(notch(x, f0n)), Lr),
        "C med5>box(k ripple)": (lambda x: med5(x), Lr),
        "D raw>box(k ripple)": (lambda x: x, Lr),
        "E notch>despike8>ewma2>33ms": (lambda x: ewma2(despike(notch(x, f0n))), Lt),
        "F notch>despike8>box(k ripple)": (lambda x: despike(notch(x, f0n)), Lr),
        "G despike8>box(k ripple)": (lambda x: despike(x), Lr),
    }


def step_gain(fn, L, n=4000):
    """dP seen one period after a unit step placed on a period boundary, averaged over phases."""
    gs = []
    for ph in np.linspace(0, L, 5, endpoint=False):
        x = np.zeros(n); s0 = int(np.ceil(10 * L + ph)); x[s0:] = 1.0
        y = fn(x)
        pm = period_means(y, L, start=s0 - 10 * L)  # boundary at s0 (period 10)
        gs.append(pm[10] - pm[9])
    return float(np.mean(gs))


def residual(x, fs, fc=0.3):
    b, a = signal.butter(2, fc / (fs / 2))
    return x - signal.filtfilt(b, a, x) + x.mean()


def caps(root):
    out = []
    for d in sorted(set(os.path.dirname(p) for p in glob.glob(f"{root}/**/iout*", recursive=True))):
        i, fi = load_ch(d, "iout"); v, fv = load_ch(d, "vout")
        if i is None or v is None or len(i) < 2000:
            continue
        out.append((os.path.relpath(d, root), i, v, fi))
    return out


def main():
    rows = []
    for name, i, v, fs in caps(ROOT):
        n = min(len(i), len(v)); i, v = np.abs(i[:n]), v[:n]
        if i.mean() < 1e-6:
            continue
        p = i * v
        f0n, snr0 = ripple_f0(p)
        pr = residual(p, fs)
        kurt = float(((pr - pr.mean()) ** 4).mean() / pr.var() ** 2)
        print(f"\n## {name}  n={n} fs~{fs:.0f}  ripple f0={f0n:.4f} cyc/sample (line/median {snr0:.0f})  "
              f"raw sd {pr.std() / pr.mean() * 100:.2f}% kurt {kurt:.1f}")
        for vname, (fn, L) in variants(fs, f0n).items():
            y = fn(pr)
            pm = period_means(y, L)[3:]
            dp = np.diff(pm)
            sd = dp.std() / pr.mean()
            g = step_gain(fn, L)
            bias = fn(p)[50:].mean() / p[50:].mean() - 1
            rows.append((name, vname, sd, g, g / sd if sd > 0 else np.inf, bias, L))
            print(f"  {vname:34s} L={L:5.1f}  sd(dP)={sd * 100:6.3f}%  g1={g:4.2f}  snr={g / sd:7.1f}  bias={bias * 100:+.2f}%")
    return rows


if __name__ == "__main__":
    main()
