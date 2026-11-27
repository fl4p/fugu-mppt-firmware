"""Power gained by each global sweep: operating point before the sweep vs after it (and vs the sweep's MPP).

Sweeps come from InfluxDB `sweep` (collector, full resolution) and `sweep_ascii` (log backfill). For each sweep:
  start  = last mppt_state=4 point before the sweep's first state-5 point (T = sweep end / chart time)
  pre    = median P, Ui over [start-31 s, start-1 s]
  post   = median P, Ui over [T+20 s, T+50 s]
  lim    = any cv_lim_idx in pre or post window (limiter-held power is not a tracking loss)
  mppt_* = window entirely in state 4 (MPPT)
Writes a CSV; prints a per-device/day summary. Usage: sweep_gap.py [--since 2026-11-13] [--out sweep_gap.csv]
"""
import argparse, csv, json, os, sys, urllib.parse, urllib.request
from datetime import datetime, timezone

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
URL = "http://tm.fabi.me:8086/query"


U, P = os.environ["INFLUX_HA_VAN_USER"], os.environ["INFLUX_HA_VAN_PASS"]  # source secrets.env


def query(q):
    qs = urllib.parse.urlencode({"db": "ha_van", "u": U, "p": P, "epoch": "ms", "q": q})
    with urllib.request.urlopen(URL + "?" + qs, timeout=120) as r:
        return json.load(r)["results"]


def series_rows(res):
    out = []
    for s in res.get("series", []) or []:
        cols = s["columns"]
        for v in s["values"]:
            out.append(dict(zip(cols, v), **(s.get("tags") or {})))
    return out


def sweeps(since):
    got = {}
    for meas in ("sweep_ascii", "sweep"):
        res = query(f"SELECT mpp_P, mpp_V FROM {meas} WHERE time >= '{since}' AND mpp_P > 0 GROUP BY device")[0]
        for r in series_rows(res):
            key = (r["device"], r["time"] // 60000)  # same sweep in both: keep `sweep`
            got[key] = dict(device=r["device"], T=r["time"], mpp_P=r["mpp_P"], mpp_V=r["mpp_V"], src=meas)
    return sorted(got.values(), key=lambda s: (s["device"], s["T"]))


def iso(ms):
    return datetime.fromtimestamp(ms / 1000, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def measure(batch):
    qs = []
    for s in batch:
        d, T = s["device"], s["T"]
        qs.append(f"SELECT first(mppt_state) FROM mppt WHERE device='{d}' AND mppt_state=5 "
                  f"AND time >= '{iso(T - 60000)}' AND time <= '{iso(T)}'")
    res = query(";".join(qs))
    for s, r in zip(batch, res):
        rows = series_rows(r)
        s["s5"] = rows[0]["time"] if rows else None
    qs = []
    for s in batch:
        s5 = s["s5"] or s["T"] - 15000
        qs.append(f"SELECT last(mppt_state) FROM mppt WHERE device='{s['device']}' AND mppt_state=4 "
                  f"AND time >= '{iso(s5 - 300000)}' AND time < '{iso(s5)}'")
    res = query(";".join(qs))
    for s, r in zip(batch, res):
        rows = series_rows(r)
        s["start"] = rows[0]["time"] + 1 if rows else None
    qs = []
    for s in batch:
        d, T, st = s["device"], s["T"], s["start"] or s["T"] - 15000
        for a, b in ((st - 31000, st - 1000), (T + 20000, T + 50000)):
            qs.append(f"SELECT median(P), median(Ui), count(P), count(cv_lim_idx), min(mppt_state), max(mppt_state) FROM mppt WHERE device='{d}' "
                      f"AND time >= '{iso(a)}' AND time < '{iso(b)}'")
    res = query(";".join(qs))
    for k, s in enumerate(batch):
        for tag, r in (("pre", res[2 * k]), ("post", res[2 * k + 1])):
            rows = series_rows(r)
            row = rows[0] if rows else {}
            s[f"P_{tag}"] = row.get("median")
            s[f"V_{tag}"] = row.get("median_1")
            s[f"n_{tag}"] = row.get("count") or 0
            s[f"lim_{tag}"] = row.get("count_1") or 0
            s[f"mppt_{tag}"] = int(row.get("min") == 4 and row.get("max") == 4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="2026-11-13T00:00:00Z")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "sweep_gap.csv"))
    a = ap.parse_args()
    sw = sweeps(a.since)
    for i in range(0, len(sw), 40):
        measure(sw[i:i + 40])
        print(f"\r{min(i + 40, len(sw))}/{len(sw)}", end="", file=sys.stderr)
    print(file=sys.stderr)
    prev = {}
    for s in sw:
        s["dt_prev_s"] = round((s["T"] - prev[s["device"]]) / 1000) if s["device"] in prev else None
        prev[s["device"]] = s["T"]
        s["time"] = iso(s["T"])[:19]
    cols = ["device", "time", "src", "dt_prev_s", "start", "mpp_P", "mpp_V", "P_pre", "V_pre", "n_pre", "lim_pre", "mppt_pre",
            "P_post", "V_post", "n_post", "lim_post", "mppt_post"]
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(sw)
    print(f"{len(sw)} sweeps -> {a.out}")


if __name__ == "__main__":
    main()
