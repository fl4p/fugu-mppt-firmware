"""Fetch 10 s aggregates (energy, mode, limiter, temperature) per device/day into $OUT/bins_<dev>.pkl."""
import os, sys, datetime as dt
import pandas as pd
from influx import query

OUT = os.environ.get('OUT', '/tmp/mpptfix')
start = dt.date(2026, 10, 22); end = dt.date(2026, 11, 22)
for dev in ('fry', 'flat'):
    frames = []
    d = start
    while d < end:
        t0, t1 = d.isoformat() + 'T00:00:00Z', (d + dt.timedelta(days=1)).isoformat() + 'T00:00:00Z'
        q = (f"SELECT mean(P) AS P, mean(Ui) AS Ui, mean(Uo) AS Uo, count(P) AS n, "
             f"mean(mppt_state) AS st_mean, min(mppt_state) AS st_min, max(mppt_state) AS st_max, count(mppt_state) AS st_n, "
             f"count(cv_lim_idx) AS lim_n, min(cv_lim_idx) AS lim_min, max(cv_lim_idx) AS lim_max, "
             f"mean(ntc_temp) AS ntc, max(P_max_today) AS pmax_today "
             f"FROM mppt WHERE device='{dev}' AND time >= '{t0}' AND time < '{t1}' GROUP BY time(10s) fill(none)")
        for s in query(q):
            frames.append(pd.DataFrame(s['values'], columns=s['columns']))
        print(dev, d, file=sys.stderr)
        d += dt.timedelta(days=1)
    df = pd.concat(frames)
    df['time'] = pd.to_datetime(df['time'], unit='ms', utc=True)
    df.set_index('time').sort_index().to_pickle(f'{OUT}/bins_{dev}.pkl')
