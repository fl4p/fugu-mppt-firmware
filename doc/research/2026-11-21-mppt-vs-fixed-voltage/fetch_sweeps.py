"""Find sweeps (mppt_state==5) per device and fetch the raw ~50 Hz Ui/P/pwm_duty samples of each.
Writes $OUT/sweeps_<dev>.pkl: list of dicts {t0, t1, df}."""
import os, sys, pickle, datetime as dt
import pandas as pd
from influx import query, query_multi

OUT = os.environ.get('OUT', '/tmp/mpptfix')
start = dt.date(2026, 10, 22); end = dt.date(2026, 11, 22)

def iso(ms):
    return dt.datetime.fromtimestamp(ms / 1000, dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%fZ')

for dev in ('fry', 'flat'):
    sweeps = []
    d = start
    while d < end:
        t0, t1 = d.isoformat() + 'T00:00:00Z', (d + dt.timedelta(days=1)).isoformat() + 'T00:00:00Z'
        s = query(f"SELECT mppt_state FROM mppt WHERE device='{dev}' AND mppt_state=5 AND time >= '{t0}' AND time < '{t1}'")
        ts = [v[0] for v in s[0]['values']] if s else []
        wins = []
        for t in ts:
            if wins and t - wins[-1][1] < 4000:
                wins[-1][1] = t
            else:
                wins.append([t, t])
        # batch raw fetches, 20 statements per request
        for i in range(0, len(wins), 20):
            chunk = wins[i:i + 20]
            qs = [f"SELECT Ui, P, Uo, pwm_duty FROM mppt WHERE device='{dev}' AND time >= '{iso(a - 2500)}' AND time <= '{iso(b + 2500)}'"
                  for a, b in chunk]
            for (a, b), res in zip(chunk, query_multi(qs)):
                if res:
                    w = pd.DataFrame(res[0]['values'], columns=res[0]['columns'])
                    sweeps.append(dict(t0=a, t1=b, df=w))
        print(dev, d, len(wins), file=sys.stderr)
        d += dt.timedelta(days=1)
    with open(f'{OUT}/sweeps_{dev}.pkl', 'wb') as f:
        pickle.dump(sweeps, f)
