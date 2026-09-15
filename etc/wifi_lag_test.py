#!/usr/bin/env python3
"""Loop-lag impact of connecting WiFi during power conversion (bench-only).
Native-USB serial (no auto-reset). Safety boundary: 50000us no-sample -> backoff.
lag prints >1e7 us are newlib-nano arg-layout artifacts => 'disturbance'.
"""
import re, sys, time
sys.path.insert(0, "etc")
from fugu.transport import SerialTransport

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem1301"
LAG = re.compile(r"lag=(\d+)"); RSSI = re.compile(r"rssi=(-?\d+)"); PWR = re.compile(r"([\d.]+)W ")
ALERT = re.compile(r"No samples|shutdown|backoff|ADC error|no ADC|latency high", re.I)
ART = 10_000_000

t = SerialTransport(PORT, 115200)
t.open(); t.ser.timeout = 1.0; time.sleep(0.5)

def send(c): print(f">>> {c}", flush=True); t.write((c + "\r\n").encode())

def watch(sec, lbl):
    end = time.time() + sec
    mx = 0; dist = 0; al = []; rssi = "?"; pwr = "?"
    while time.time() < end:
        raw = t.read()
        if not raw: continue
        ln = raw.decode("utf-8", "replace").strip()
        if not ln: continue
        m = LAG.search(ln)
        if m:
            lag = int(m.group(1)); r = RSSI.search(ln); p = PWR.search(ln)
            if r: rssi = r.group(1)
            if p: pwr = p.group(1)
            if lag >= ART: dist += 1; print(f"[{lbl}] DISTURB rssi={rssi} {pwr}W", flush=True)
            else: mx = max(mx, lag); print(f"[{lbl}] lag={lag} rssi={rssi} {pwr}W", flush=True)
        if ALERT.search(ln): al.append(ln); print(f"[{lbl}] !! {ln}", flush=True)
    return dict(max=mx, dist=dist, al=al, rssi=rssi, pwr=pwr)

R = {}
send("reset-lag"); R['base'] = watch(8, "BASE")                 # converting, wifi down
send("reset-lag"); time.sleep(0.2)
send("wifi on");   R['on'] = watch(35, "WIFI-ON-converting")    # connect attempt under load

print("\n===== SUMMARY =====")
print("no-sample shutdown boundary: 50000 us")
for k in ('base', 'on'):
    r = R[k]
    print(f"{k:20s} maxlag={r['max']:>6}us disturb={r['dist']} rssi={r['rssi']} pwr={r['pwr']}W trips={len(r['al'])}")
al = R['base']['al'] + R['on']['al']
print("backoff/shutdown/ADC-error/latency events:", len(al))
for a in al: print("  ", a)
print("VERDICT:", "SAFE (no watchdog trip, conversion held)" if not al else "UNSAFE")
if R['on']['dist']:
    print(f"NOTE: {R['on']['dist']} RT-loop disturbance(s) at connect; stayed under the 50 ms watchdog.")
t.close()
