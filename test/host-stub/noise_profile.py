"""Sensor-noise profile for the host sim (SIM_NOISE) from adcscope captures of a running converter.

  noise_profile.py OUT.fnz CAPDIR[:TELEMETRY.json[:T0[:T1]]] ...

Each CAPDIR holds the adcscope npz files (vin_*, vout_*, iout_*) and capture.json. The residual of a
channel is the capture minus its zero-phase 0.3 Hz low-pass (drift and the tracker's own slow moves
leave, the 100/200 Hz ripple, aliased lines and the 0.5..5 Hz fluctuation stay). Scales: Iout from
the INA226 current LSB (35 A / 2^15, scope sends |raw| / 3); Vin and Vout by the telemetry mean
(InfluxDB `mppt` points over the capture; JSON as returned by /query with P, Ui, Uo, I) or, without
telemetry, by --vin/--vout scope-unit sizes. T0..T1 [s] cut a capture (leave out the tracker's
duty jumps: their power steps are plant behaviour, the sim makes its own). Several captures are
concatenated, but a join between two is a step in the ripple's phase and frequency that the adaptive
notch passes as a power transient: one continuous capture is the clean profile. The sim loops it; the
loop length is a whole number of ripple periods and its last second fades into the first.
--keep-slow keeps Iout's content below fc too (only its linear trend is removed): the slow wander of the
current, which at a steady duty is light or measurement, not the tracker (pick a capture whose Vin hardly moves).

File (little endian): 'FNZ1', float iRef [A], then per channel vin, vout, iout: float fs [Hz],
float lsb, uint32 n, int16[n] (value = n * lsb).
"""
import argparse, glob, json, os, struct
import numpy as np
from scipy import signal

I_LSB = 35 / 32768 * 3
CH = ("vin", "vout", "iout")


def load(d):
    meta = json.load(open(f"{d}/capture.json"))
    out = {}
    for name in CH:
        f, = [p for p in glob.glob(f"{d}/{name}_*.npz") if os.path.basename(p).rsplit("_", 1)[0] == name]
        out[name] = (np.load(f)["v"].astype(float), meta["channels"][name]["rate_hz"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("caps", nargs="+")
    ap.add_argument("--vin", type=float, help="V per scope unit (no telemetry)")
    ap.add_argument("--vout", type=float, help="V per scope unit (no telemetry)")
    ap.add_argument("--fc", type=float, default=0.3)
    ap.add_argument("--keep-slow", action="store_true")
    a = ap.parse_args()
    res = {c: [] for c in CH}
    fs = {}
    irefs = []
    for spec in a.caps:
        d, tel, t0, t1 = (spec.split(":") + ["", "", ""])[:4]
        raw = load(d)
        k = {"iout": I_LSB, "vin": a.vin, "vout": a.vout}
        if tel:
            s = json.load(open(tel))["results"][0]["series"][0]
            col = {c: i for i, c in enumerate(s["columns"])}
            m = lambda c: np.nanmean([r[col[c]] for r in s["values"] if r[col[c]] is not None])
            k["vin"] = m("Ui") / raw["vin"][0].mean()
            k["vout"] = m("Uo") / raw["vout"][0].mean()
            irefs.append(m("I"))
        dur = min(x.size / f for x, f in raw.values())
        t0, t1 = float(t0 or 0.5), min(float(t1 or dur), dur)  # the first samples can be stale
        for c in CH:
            x, f = raw[c]
            x = x[:int(dur * f)] * k[c]
            sos = signal.butter(2, a.fc, "lowpass", fs=f, output="sos")
            seg = slice(int(t0 * f), int(t1 * f))
            res[c].append(signal.detrend(x[seg]) if a.keep_slow and c == "iout" else (x - signal.sosfiltfilt(sos, x))[seg])
            fs.setdefault(c, []).append(f)
            if c == "iout" and not tel:
                irefs.append(x.mean())
    # loop: n samples at the Iout rate, a whole number of ripple periods, the last nx fade into the first
    fi = np.mean(fs["iout"])
    x = np.concatenate(res["vout"])
    f, X = signal.periodogram(x, fi, window="hann")
    m = (f > 80) & (f < 140)
    f0 = f[m][np.argmax(X[m])]
    nx = int(fi)
    cand = np.arange(x.size - 2 * nx, x.size - nx)
    n = cand[np.argmax(np.cos(2 * np.pi * f0 * cand / fi))]
    print(f"ripple {f0:.2f} Hz, loop {n / fi:.2f} s")
    with open(a.out, "wb") as fo:
        fo.write(b"FNZ1" + struct.pack("<f", float(np.mean(irefs))))
        for c in CH:
            r = np.concatenate(res[c])
            k, kx = int(round(n * np.mean(fs[c]) / fi)), int(round(nx * np.mean(fs[c]) / fi))
            w = np.linspace(0, 1, kx)
            r = np.concatenate([r[:kx] * w + r[k:k + kx] * (1 - w), r[kx:k]])
            lsb = float(np.abs(r).max() / 30000)
            fo.write(struct.pack("<ffI", float(np.mean(fs[c])), lsb, r.size))
            fo.write(np.round(r / lsb).astype("<i2").tobytes())
            print(f"{c}: fs {np.mean(fs[c]):.1f} Hz, {r.size} samples ({r.size / np.mean(fs[c]):.1f} s), rms {r.std() * 1e3:.2f} m, lsb {lsb * 1e6:.1f} u")
    print(f"iRef {np.mean(irefs):.3f} A -> {a.out} ({os.path.getsize(a.out)} B)")


if __name__ == "__main__":
    main()
