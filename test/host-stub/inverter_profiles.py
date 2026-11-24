"""SIM_NOISE profiles of an inverter on the DC bus, from the adcscope captures in etc/adcscope/data.

  inverter_profiles.py [DATA_DIR]      # writes test/host-stub/noise/<name>.fnz

The scope only estimates each channel's sample rate (fry's INA226 ran at ~512 sps, labelled 646). An inverter
at 50 Hz mains draws its ripple at 100 Hz, so each channel's rate is set to put its ripple line at 100 Hz
(fs = 100 Hz / line in cycles per sample); a channel without a visible line keeps NOMINAL. The captures carry
no telemetry, so Vin and Vout are scaled to VIN / VOUT volts at their mean (the residual keeps its relative
size); Iout uses the scope's INA226 current LSB as in noise_profile.py. The profiles are then made by
noise_profile.py from a temporary capture layout.
"""
import json, os, subprocess, sys, tempfile
import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
VIN, VOUT, F_RIPPLE = 60.0, 26.5, 100.0
NOMINAL = {"vin": 400.0, "vout": 451.0, "iout": 451.0}
CAPS = {  # name: (dir, {channel: file})
    "fry-inv": ("fry/2000w-inverter-midday-20260606T131106Z",
                {"vin": "vin_583.npz", "vout": "vout_646.npz", "iout": "iout_646.npz"}),
    "var-inv": ("var-inverter", {"vin": "vin_402hz.csv", "vout": "vout_452hz.csv", "iout": "iout_452hz.csv"}),
    "china-inv": ("china-inverter-1kw-load", {"vin": "vin.csv", "vout": "vout.csv", "iout": "iout.csv"}),
    "pv8a-inv": ("pv8A-inv500W", {"vin": "vin.csv", "vout": "vout.csv", "iout": "iout.csv"}),
    "heater-inv": ("heater500w-off-on", {"vin": "vin_399hz.csv", "vout": "vout_453hz.csv", "iout": "iout_454hz.csv"}),
}


def load(p):
    x = np.load(p)["v"] if p.endswith(".npz") else np.loadtxt(p)
    return x.astype(float)[5:]  # the first samples can be stale


def ripple_line(x):
    """2x-line in cycles/sample: strongest line in 0.15..0.33 (100 Hz at 300..670 sps), None if not distinct."""
    f, p = signal.welch(x - x.mean(), 1.0, nperseg=min(8192, x.size // 2))
    m = (f > .15) & (f < .33)
    k = np.argmax(p[m])
    return f[m][k] if p[m][k] > 30 * np.median(p[m]) else None


def main():
    data = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "../../etc/adcscope/data")
    for name, (d, files) in CAPS.items():
        tmp = tempfile.mkdtemp(prefix=f"invcap-{name}-")
        meta = {"channels": {}}
        x = {c: load(os.path.join(data, d, f)) for c, f in files.items()}
        line = {c: ripple_line(v) for c, v in x.items()}
        # Vout and Iout come from one INA226: one rate, from the cleaner Vout line
        if line["vout"]: line["iout"] = line["vout"]
        for c, v in x.items():
            fs = F_RIPPLE / line[c] if line[c] else NOMINAL[c]
            meta["channels"][c] = {"rate_hz": fs}
            np.savez(os.path.join(tmp, f"{c}_0.npz"), v=v)
            print(f"{name} {c}: line {line[c] or 0:.4f} cyc/sample -> fs {fs:.1f} Hz, mean {v.mean():.0f}")
        json.dump(meta, open(os.path.join(tmp, "capture.json"), "w"))
        out = os.path.join(HERE, "noise", f"{name}.fnz")
        subprocess.run([sys.executable, os.path.join(HERE, "noise_profile.py"), out, tmp,
                        "--vin", str(VIN / x["vin"].mean()), "--vout", str(VOUT / x["vout"].mean())], check=True)


if __name__ == "__main__":
    main()
