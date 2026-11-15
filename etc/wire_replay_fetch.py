#!/usr/bin/env python3
"""Fetch converter + BMS data from InfluxDB for test/host-stub/wire-guard-replay.cpp.

    set -a; . .claude/memory/secrets.env; set +a
    .venv/bin/python3 etc/wire_replay_fetch.py --out /tmp/wirereplay/data --start 2026-11-04 [--end ...] [--device fry]

Writes mppt1s.csv (1 s means of Uo/I), bms.csv (pack V and A frames), sweeps.csv (raw samples of every
sweep, seg = sweep start in ms), tele_r.csv (Ro/Rc/Rb) and tele_wf.csv (the device's own wf/Pw).
Queries go in 6 h windows; the credentials come from $INFLUX_HA_VAN_USER/_PASS and are never printed.
Only fry has valid wire data so far: flat's Vout sense lacks its filter cap, and from 2026-11-12 ~19:00
to 2026-11-13 ~09:00 UTC a host simulator wrote points tagged device=flat (Ro ~50.01, power at night).
"""
import argparse
import datetime as dt
import json
import os
import urllib.parse
import urllib.request

URL = 'http://tm.fabi.me:8086/query'


def query(q):
    p = urllib.parse.urlencode({'db': 'ha_van', 'u': os.environ['INFLUX_HA_VAN_USER'],
                                'p': os.environ['INFLUX_HA_VAN_PASS'], 'q': q, 'epoch': 'ms'})
    for attempt in range(3):
        try:
            with urllib.request.urlopen(URL + '?' + p, timeout=120) as r:
                res = json.load(r)['results'][0]
            if 'error' in res:
                raise RuntimeError(res['error'])
            return res['series'][0]['values'] if 'series' in res else []
        except OSError:
            if attempt == 2:
                raise


def utc(s):
    d = dt.datetime.fromisoformat(s)
    return d if d.tzinfo else d.replace(tzinfo=dt.timezone.utc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--start', required=True)
    ap.add_argument('--end')
    ap.add_argument('--device', default='fry')
    a = ap.parse_args()
    start, end = utc(a.start), utc(a.end) if a.end else dt.datetime.now(dt.timezone.utc)
    dev = a.device
    os.makedirs(a.out, exist_ok=True)

    def tr(x, y):
        return f"time >= '{x.isoformat()}' AND time < '{y.isoformat()}'"

    fm = open(f'{a.out}/mppt1s.csv', 'w'); fm.write('t_ms,Uo,I\n')
    fb = open(f'{a.out}/bms.csv', 'w'); fb.write('t_ms,kind,value\n')
    fs = open(f'{a.out}/sweeps.csv', 'w'); fs.write('seg,t_ms,Uo,I,pwm\n')
    fr = open(f'{a.out}/tele_r.csv', 'w'); fr.write('t_ms,Ro,Rc,Rb\n')
    fw = open(f'{a.out}/tele_wf.csv', 'w'); fw.write('t_ms,wf,Pw\n')
    nseg = 0
    d = start
    while d < end:
        e = min(d + dt.timedelta(hours=6), end)
        w = tr(d, e)
        for t, uo, i in query(f"SELECT mean(Uo), mean(I) FROM mppt WHERE device='{dev}' AND {w} GROUP BY time(1s) fill(none)"):
            fm.write(f'{t},{uo},{i}\n')
        for t, v in query(f"SELECT value FROM \"V\" WHERE entity_id='bat_caravan_voltage' AND {w}"):
            fb.write(f'{t},V,{v}\n')
        for t, v in query(f"SELECT value FROM \"A\" WHERE entity_id='bat_caravan_current' AND {w}"):
            fb.write(f'{t},A,{v}\n')
        for t, ro, rc, rb in query(f"SELECT Ro, Rc, Rb FROM mppt WHERE device='{dev}' AND (Ro > 0 OR Rc > -1000) AND {w}"):
            fr.write(f'{t},{ro},{rc},{rb}\n')
        for t, wf, pw in query(f"SELECT wf, Pw FROM mppt WHERE device='{dev}' AND wf > -10 AND {w}"):
            fw.write(f'{t},{wf},{"" if pw is None else pw}\n')
        segs = []
        for t, _ in query(f"SELECT mppt_state FROM mppt WHERE device='{dev}' AND mppt_state = 5 AND {w}"):
            if segs and t - segs[-1][1] <= 2000:
                segs[-1][1] = t
            else:
                segs.append([t, t])
        for s0, s1 in segs:
            ta = dt.datetime.fromtimestamp((s0 - 1500) / 1000, dt.timezone.utc)
            tb = dt.datetime.fromtimestamp((s1 + 1500) / 1000, dt.timezone.utc)
            for t, uo, i, p in query(f"SELECT Uo, I, pwm_duty FROM mppt WHERE device='{dev}' AND {tr(ta, tb)}"):
                fs.write(f'{s0},{t},{"" if uo is None else uo},{"" if i is None else i},{"" if p is None else p}\n')
        nseg += len(segs)
        print(d.isoformat(), 'sweeps', nseg, flush=True)
        d = e
    for f in (fm, fb, fs, fr, fw):
        f.close()
    print('done')


if __name__ == '__main__':
    main()
