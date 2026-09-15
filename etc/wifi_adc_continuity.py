#!/usr/bin/env python3
"""dry_int (real internal continuous ADC) vs WiFi bring-up.
Polls `status` ~3/s, tracks N-deltas -> sample rate, lag, and any
ADC-stall/backoff/latency/ADC-error across a wifi off->on->off cycle.
Native-USB serial (no auto-reset)."""
import re, sys, time
sys.path.insert(0, "etc")
from fugu.transport import SerialTransport

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem1301"
N    = re.compile(r"\bN=(\d+)")
LAG  = re.compile(r"lag=(\d+)")
RSSI = re.compile(r"rssi=(-?\d+)")
PWR  = re.compile(r"([\d.]+)W ")
ALERT= re.compile(r"No samples|shutdown|backoff|ADC error|no ADC|latency high|stall", re.I)

t = SerialTransport(PORT, 115200)
t.open(); t.ser.timeout = 0.4; time.sleep(0.4)

def send(c): print(f">>> {c}", flush=True); t.write((c + "\r\n").encode())

def phase(sec, lbl):
    end = time.time() + sec
    samples = []          # (t, N)
    lags = []; alerts = []; rssi = "?"; pwr = "?"
    send("status")
    last_poll = time.time()
    while time.time() < end:
        raw = t.read()
        if raw:
            ln = raw.decode("utf-8", "replace").strip()
            if ln:
                m = N.search(ln)
                if m:
                    samples.append((time.time(), int(m.group(1))))
                    r = RSSI.search(ln); p = PWR.search(ln); l = LAG.search(ln)
                    if r: rssi = r.group(1)
                    if p: pwr = p.group(1)
                    if l: lags.append(int(l.group(1)))
                if ALERT.search(ln):
                    alerts.append(ln); print(f"[{lbl}] !! {ln}", flush=True)
        if time.time() - last_poll > 0.33:
            send("status"); last_poll = time.time()
    # sample rate from first/last N over elapsed wall time
    sps = 0.0
    if len(samples) >= 2:
        (t0, n0), (t1, n1) = samples[0], samples[-1]
        if t1 > t0: sps = (n1 - n0) / (t1 - t0)
    mlag = max(lags) if lags else 0
    print(f"[{lbl}] sps~={sps:8.0f}/s  N_samples={len(samples)}  maxlag={mlag}us  "
          f"rssi={rssi} pwr={pwr}W alerts={len(alerts)}", flush=True)
    return dict(sps=sps, mlag=mlag, alerts=alerts, rssi=rssi, pwr=pwr)

R = {}
R['base']  = phase(8,  "BASE wifi-down")
send("reset-lag"); time.sleep(0.2)
send("wifi on")
R['oning'] = phase(30, "WIFI-ON")
send("wifi off 60")   # keep ssid; just disconnect
R['after'] = phase(8,  "AFTER wifi-off")

print("\n===== SUMMARY (dry_int / esp32adc1 real continuous ADC) =====")
for k in ('base','oning','after'):
    r = R[k]
    print(f"{k:16s} sps~={r['sps']:8.0f}/s maxlag={r['mlag']:>6}us "
          f"rssi={r['rssi']} pwr={r['pwr']}W alerts={len(r['alerts'])}")
allA = sum((R[k]['alerts'] for k in R), [])
print("ADC-stall/backoff/latency/ADC-error events:", len(allA))
for a in allA: print("  ", a)
drop = (R['oning']['sps']/R['base']['sps']*100) if R['base']['sps'] else 0
print(f"sample-rate retention during wifi-on: {drop:.0f}% of baseline")
print("VERDICT:", "SAFE (sampler continuous, no watchdog trip)" if not allA else "UNSAFE")
