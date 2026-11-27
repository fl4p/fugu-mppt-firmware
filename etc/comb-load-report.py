#!/usr/bin/env python3
"""Daily fry/flat yield ratio with and without inverter load (ripple comb field check, issue #37).

Per minute: load A = (P_fry + P_flat) / Uo - I_bat. Minutes where either device spends > 5 % of its points on the
Vout limiter (cv_lim_idx=1, of the points carrying the field) are dropped; cv_lim_idx is the interval minimum, so a
Vin-limiter touch in the same interval hides a 1 and some capped minutes can stay in. Reports ratio(load > 30 A) / ratio(load < 5 A, same hours); below 1 means
fry loses against flat while the inverter runs. Prints one JSON line per day.

  comb-load-report.py [--day YYYY-MM-DD] [--days N]     (default: yesterday, UTC)
Env: INFLUX_HA_VAN_USER/_PASS (tm.fabi.me ha_van), BATMON_CONF (json with influxdb_host/_username/_password/_database).
"""
import argparse, datetime as dt, json, os, ssl, urllib.parse, urllib.request
from collections import defaultdict

LOAD_ON, LOAD_OFF, CAP_FRAC, MIN_LOAD_MIN = 30.0, 5.0, 0.05, 20


def query(base, db, u, p, ql):
    url = f"{base}/query?" + urllib.parse.urlencode({"db": db, "u": u, "p": p, "q": ql, "epoch": "s"})
    ctx = None
    if base.startswith("https"):
        try:
            import certifi
            ctx = ssl.create_default_context(cafile=certifi.where())
        except ImportError:
            ctx = ssl.create_default_context()
    res = json.loads(urllib.request.urlopen(url, timeout=120, context=ctx).read())["results"]
    if any("error" in r for r in res):
        raise RuntimeError(res)
    return res


def series(res):
    """{(tag device or name): {epoch: row dict}} from one statement result."""
    out = defaultdict(dict)
    for s in res.get("series", []):
        key = s.get("tags", {}).get("device", s["name"])
        for v in s["values"]:
            out[key][v[0]] = dict(zip(s["columns"], v))
    return out


def day_report(day):
    t0 = f"'{day}T00:00:00Z'"
    t1 = f"'{day + dt.timedelta(days=1)}T00:00:00Z'"
    w = f"time >= {t0} AND time < {t1}"
    r = query("http://tm.fabi.me:8086", "ha_van", os.environ["INFLUX_HA_VAN_USER"], os.environ["INFLUX_HA_VAN_PASS"],
              f"SELECT mean(P) AS P, mean(Uo) AS U, count(P) AS n, count(cv_lim_idx) AS nl FROM mppt WHERE {w} GROUP BY time(1m), device fill(none);"
              f"SELECT count(cv_lim_idx) AS c FROM mppt WHERE {w} AND cv_lim_idx = 1 GROUP BY time(1m), device fill(none)")
    m, cap = series(r[0]), series(r[1])
    bc = json.load(open(os.environ.get("BATMON_CONF", os.path.expanduser("~/pv/comb-report/batmon.json"))))
    b = series(query(f"https://{bc['influxdb_host']}:8086", bc["influxdb_database"], bc["influxdb_username"],
                     bc["influxdb_password"],
                     f"SELECT mean(current) AS I FROM batmon WHERE device='bat_caravan' AND {w} GROUP BY time(1m) fill(none)")[0])
    bat = next(iter(b.values()), {})
    fry, flat = m.get("fry", {}), m.get("flat", {})
    mins, ncap = [], 0
    for t, a in fry.items():
        f, i = flat.get(t), bat.get(t)
        if not f or not i or a["P"] is None or f["P"] is None or i["I"] is None or not a["U"]:
            continue
        if a["P"] <= 3 and f["P"] <= 3:
            continue
        if any(cap.get(d, {}).get(t, {}).get("c", 0) > CAP_FRAC * (x["nl"] or x["n"]) for d, x in (("fry", a), ("flat", f))):
            ncap += 1
            continue
        mins.append((t, a["P"], f["P"], (a["P"] + f["P"]) / a["U"] - i["I"]))
    on = [x for x in mins if x[3] > LOAD_ON]
    hours = {dt.datetime.fromtimestamp(x[0], dt.timezone.utc).hour for x in on}
    off = [x for x in mins if x[3] < LOAD_OFF and dt.datetime.fromtimestamp(x[0], dt.timezone.utc).hour in hours]

    def ratio(xs):
        s = sum(x[2] for x in xs)
        return sum(x[1] for x in xs) / s if s > 0 else None

    ron, roff = ratio(on), ratio(off)
    rel = ron / roff if ron and roff and len(on) >= MIN_LOAD_MIN and len(off) >= MIN_LOAD_MIN else None
    return {"day": str(day), "day_min": len(mins), "cap_min_dropped": ncap, "load_min": len(on), "noload_min": len(off),
            "ratio_load": ron and round(ron, 3), "ratio_noload": roff and round(roff, 3),
            "load_over_noload": rel and round(rel, 3), "hours": sorted(hours),
            "verdict": "ok" if rel else "too few load minutes"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--day")
    ap.add_argument("--days", type=int, default=1)
    a = ap.parse_args()
    end = dt.date.fromisoformat(a.day) if a.day else dt.datetime.now(dt.timezone.utc).date() - dt.timedelta(days=1)
    for k in range(a.days - 1, -1, -1):
        print(json.dumps(day_report(end - dt.timedelta(days=k))), flush=True)


if __name__ == "__main__":
    main()
