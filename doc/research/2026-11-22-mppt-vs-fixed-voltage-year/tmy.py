"""Convert PVGIS TMY JSON (downloaded with fetch_tmy.sh into a separate dir) to slim tmy_<site>.csv.gz here.
Usage: python3 -I tmy.py <download dir>"""
import json, os, sys
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
src = sys.argv[1]
for site in ('lisbon', 'evora'):
    d = json.load(open(os.path.join(src, f'tmy_{site}.json')))
    h = pd.DataFrame(d['outputs']['tmy_hourly'])
    h['time'] = pd.to_datetime(h['time(UTC)'], format='%Y%m%d:%H%M', utc=True)
    h = h.rename(columns={'G(h)': 'ghi', 'Gb(n)': 'dni', 'Gd(h)': 'dhi', 'T2m': 'tamb', 'WS10m': 'wind'})
    h[['time', 'ghi', 'dni', 'dhi', 'tamb', 'wind']].to_csv(os.path.join(HERE, f'tmy_{site}.csv.gz'), index=False)
    loc = d['inputs']['location']
    print(site, loc, 'months', [(m['month'], m['year']) for m in d['outputs']['months_selected']], len(h))
