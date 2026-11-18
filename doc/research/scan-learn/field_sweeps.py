"""Field sweeps for the substring learner replay (test/host-stub/scan-learn-replay.cpp).

fetch: sweep end lines ("Stop sweep ... MPP=(P W,duty,V V)") from the console log, telemetry window per sweep from
InfluxDB (ha_van/mppt, ~48 Hz Ui, P, pwm_duty; needs INFLUX_HA_VAN_USER/_PASS) -> pickle.
export: per sweep the boot (log time minus uptime at its first sweep) and the duty ramp of each sweep (last zero-duty sample to the duty maximum), P as med3 + EWM (alpha 0.3, about
the sweep's power_smooth), Vin falling -> replay text on stdout.
  python3 field_sweeps.py fetch <sweep-lines.txt> <out.pkl>
  python3 field_sweeps.py export <out.pkl> > sweeps.txt
"""
import concurrent.futures as cf
import datetime as dt
import json
import os
import pickle
import re
import sys
import urllib.parse
import urllib.request

import numpy as np


def fetch(lines, out):
    rx = re.compile(r'^(\S+ \S+),\d+\s+(fry|flat): I \((\d+)\) .*Stop sweep ([\d.]+)s.*MPP=\(([\d.]+)W,(\d+),([\d.]+)V\)')
    items = []
    for l in open(lines, errors='replace'):
        m = rx.match(l)
        if m:  # log time = telemetry time (UTC): the logged MPP matches the telemetry at offset 0, not at 1 or 2 h
            t = dt.datetime.strptime(m[1], '%Y-%m-%d %H:%M:%S')
            items.append([m[2], t, float(m[4]), float(m[5]), float(m[7]), int(m[3])])
    items.sort(key=lambda x: (x[0], x[1]))
    dev = boot = None
    for x in items:  # boot id: log time minus uptime, a new boot when that moves by more than 2 minutes
        b = x[1] - dt.timedelta(milliseconds=x[5])
        if dev != x[0] or abs((b - boot).total_seconds()) > 120:
            dev, boot = x[0], b
        x[5] = boot
    u, p = os.environ['INFLUX_HA_VAN_USER'], os.environ['INFLUX_HA_VAN_PASS']

    def q(it):
        dev, t, dur, pm, vm, boot = it
        a = (t - dt.timedelta(seconds=dur + 4)).strftime('%Y-%m-%dT%H:%M:%SZ')
        b = (t + dt.timedelta(seconds=2)).strftime('%Y-%m-%dT%H:%M:%SZ')
        qs = f"SELECT Ui,P,pwm_duty FROM mppt WHERE device='{dev}' AND time >= '{a}' AND time <= '{b}'"
        url = 'http://tm.fabi.me:8086/query?' + urllib.parse.urlencode(
            {'db': 'ha_van', 'u': u, 'p': p, 'q': qs, 'epoch': 'ms'})
        s = json.load(urllib.request.urlopen(url, timeout=60))['results'][0].get('series')
        return dict(dev=dev, t=t, dur=dur, pm=pm, vm=vm, boot=boot, rows=s[0]['values'] if s else [])

    with cf.ThreadPoolExecutor(4) as ex:
        pickle.dump(list(ex.map(q, items)), open(out, 'wb'))


def export(pkl):
    for o in sorted(pickle.load(open(pkl, 'rb')), key=lambda o: (o['dev'], o['t'])):
        r = np.array([[np.nan if x is None else x for x in row] for row in o['rows']], float)
        if len(r) < 50:
            continue
        r = r[~np.isnan(r[:, 3])]
        z = np.where(r[:, 3] <= 5)[0]
        if not len(z) or r[z[-1]:, 3].max() < 1000:
            continue  # no full sweep in the window (log/telemetry gap)
        i0 = z[-1]
        i1 = i0 + int(np.argmax(r[i0:, 3]))
        if i1 - i0 < 100:
            continue
        ui, p = r[i0:i1 + 1, 1], r[i0:i1 + 1, 2]
        pm = np.array([np.median(p[max(0, i - 1):i + 2]) for i in range(len(p))])
        s, ps = pm[0], []
        for x in pm:
            s += 0.3 * (x - s)
            ps.append(s)
        k = int(np.argmax(ps))  # the MPP from the same smoothed P (the firmware compares power_smooth with itself)
        print(f"S {o['dev']} {o['boot']:%Y%m%d%H%M} {o['t']:%H:%M:%S} {ui[0]:.2f} {ui[k]:.2f} {ps[k]:.1f}")
        for v, x in zip(ui, ps):
            print(f"{v:.2f} {x:.2f}")
        print("E")


if __name__ == '__main__':
    fetch(sys.argv[2], sys.argv[3]) if sys.argv[1] == 'fetch' else export(sys.argv[2])
