import json, os, pathlib, urllib.parse, urllib.request
out = pathlib.Path(__file__).parent
for dev in ('fry', 'flat'):
    q = f"SELECT * FROM mppt WHERE device='{dev}' AND time >= '2026-11-25T10:00:00Z' AND time < '2026-11-25T12:00:00Z' ORDER BY time ASC LIMIT 2048"
    p = urllib.parse.urlencode(dict(db='ha_van', u=os.environ['INFLUX_HA_VAN_USER'], p=os.environ['INFLUX_HA_VAN_PASS'], q=q, epoch='ms'))
    try:
        with urllib.request.urlopen('http://tm.fabi.me:8086/query?' + p, timeout=60) as r:
            data = json.load(r)
    except Exception as e:
        raise SystemExit(type(e).__name__ + ': database fetch failed (URL withheld)')
    if not data['results'][0].get('series'): raise SystemExit('No series: '+dev)
    (out / (dev+'.json')).write_text(json.dumps({'query': q, 'response': data}))
    s=data['results'][0]['series'][0]
    print(dev, len(s['values']), 'rows', s['columns'])
