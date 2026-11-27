#!/usr/bin/env python3
"""Store every MPPT sweep curve the devices publish into InfluxDB (measurement `sweep`).

Listens on pv/log/+. When a device logs `Stop sweep` or `sweep verified|flagged:`, it sends the
read-only console command `sweep-curve` to pv/log/<host>/cmd (nothing else is ever sent) and parses
the `sc:` reply. Curves seen on the log without a request (e.g. a dashboard fetch) are stored too.

Each curve is first appended to a daily JSONL archive (<archive>/<UTC date>.jsonl, raw lines
included), then written to InfluxDB: one point per (V, P) pair, time = sweep end (s) + index (ms),
tags device + src=fw, fields i, V, P; point 0 also mpp_V, mpp_P, n and, once the sweep has a
verdict, flag ("" clean, else the reason) and land. Re-writing the same curve overwrites the same
points, so `--replay FILE.jsonl` re-imports an archive idempotently.

Config (environment or env files, see README.md): MQTT_USER, MQTT_PASS, MQTT_PORT,
MQTT_COLLECTOR_HOST (default localhost), INFLUX_URL, INFLUX_DB, INFLUX_USER, INFLUX_PASS.
"""
import argparse
import datetime
import json
import logging
import math
import os
import queue
import re
import socket
import time
import urllib.error
import urllib.parse
import urllib.request
from logging.handlers import RotatingFileHandler

TOPIC = "pv/log/+"
ROOT = "pv/log/"
STOP_DELAY_S = 8.0     # after `Stop sweep`: the verdict line usually follows within ~2 s
VERDICT_DELAY_S = 2.0
ATTEMPT_TIMEOUT_S = 5.0
RETRIES = 2            # attempts after the first
SWEEPING_RETRY_S = 5.0
SWEEPING_TRIES = 6
WRITE_RETRY_S = 60.0
MAX_PENDING_WRITES = 200

log = logging.getLogger("sweep")

# ---------------------------------------------------------------------------------------------
# Parser (mirrors etc/config-tool/teledec.js parseScLine / SweepCurveParser)
# ---------------------------------------------------------------------------------------------
ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
SC_RE = re.compile(r"(?:^|[^A-Za-z0-9_])sc:\s*(.*?)\s*$")
NUM_RE = re.compile(r"^[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?$|^[-+]?(?:inf|nan)$", re.I)


def sc_num(tok):
    t = tok.lower()
    if t.lstrip("+-") == "inf":
        return -math.inf if t[0] == "-" else math.inf
    if t.endswith("nan"):
        return math.nan
    return float(tok)


def _js_number(s):
    """Number(s) for the header fields: nan for anything non-numeric."""
    try:
        return float(s) if s is not None and NUM_RE.match(s) else math.nan
    except ValueError:
        return math.nan


def parse_sc_line(line):
    """None for a non-`sc:` line, else a dict with kind header|data|end|none|invalid."""
    m = SC_RE.search(ANSI_RE.sub("", line))
    if not m:
        return None
    body = m.group(1)
    if body in ("end", "none"):
        return {"kind": body}
    if re.search(r"(?:^|\s)t=", body):
        kv = {}
        for tok in body.split():
            e = tok.find("=")
            if e > 0:
                kv[tok[:e]] = tok[e + 1:]
        mpp = kv.get("mpp", "").split(",")
        t, n = _js_number(kv.get("t")), _js_number(kv.get("n"))
        if (not math.isfinite(t) or not math.isfinite(n) or n != int(n) or n < 0 or len(mpp) != 2
                or not NUM_RE.match(mpp[0]) or not NUM_RE.match(mpp[1])):
            return {"kind": "invalid", "text": body}
        h = {"kind": "header", "t": int(t), "n": int(n), "mpp_P": sc_num(mpp[0]), "mpp_V": sc_num(mpp[1]),
             "sweeping": kv.get("sweeping") == "1"}
        if "flag" in kv:  # sweep verdict: '' clean, else the reason
            h["flag"] = "" if kv["flag"] == "0" else kv["flag"]
            land = kv.get("land", "")
            h["land"] = sc_num(land) if NUM_RE.match(land) else math.nan
        return h
    vals, truncated = [], False
    for tok in body.split() if body else []:
        if not NUM_RE.match(tok):
            truncated = True
            break
        vals.append(sc_num(tok))
    if len(vals) % 2:
        truncated = True
    pairs = [(vals[i], vals[i + 1]) for i in range(0, len(vals) - 1, 2)]
    if not pairs and not truncated:
        return {"kind": "invalid", "text": body}
    return {"kind": "data", "pairs": pairs, "truncated": truncated}


class SweepCurveParser:
    """Assembles one curve from header, data and `end` lines.

    feed() returns None until a curve finishes, then {complete, curve, reason}; `sc: none` gives
    {complete: True, curve: None}. curve["points"] holds (i, V, P) with i the pair's position in the
    reply; inf/nan pairs are dropped but keep their position. curve["raw"] holds the sc: lines.
    """

    def __init__(self):
        self.cur = None

    def _finish(self, reason=None):
        c, self.cur = self.cur, None
        if not reason and len(c["points"]) + c["dropped"] != c["n"]:
            reason = "short"
        if not reason and c["truncated"]:
            reason = "truncated"
        return {"complete": not reason, "curve": c, "reason": reason}

    def feed(self, line):
        p = parse_sc_line(line)
        if p is None:
            return None
        k = p["kind"]
        if k == "header":
            prev = self._finish("restart") if self.cur else None
            self.cur = {key: v for key, v in p.items() if key != "kind"}
            self.cur.update(points=[], dropped=0, truncated=False, raw=[line.rstrip("\r\n")])
            return prev
        if k == "data":
            c = self.cur
            if c:
                c["raw"].append(line.rstrip("\r\n"))
                for V, P in p["pairs"]:
                    idx = len(c["points"]) + c["dropped"]
                    if math.isfinite(V) and math.isfinite(P):
                        c["points"].append((idx, V, P))
                    else:
                        c["dropped"] += 1
                c["truncated"] |= p["truncated"]
            return None
        if k == "end":
            if not self.cur:
                return None
            self.cur["raw"].append(line.rstrip("\r\n"))
            return self._finish()
        if k == "none":
            self.cur = None
            return {"complete": True, "curve": None, "reason": None}
        return None


# ---------------------------------------------------------------------------------------------
# Influx line protocol
# ---------------------------------------------------------------------------------------------
def _tag(s):
    return re.sub(r"([,= \\])", r"\\\1", s)


def _f(x):
    return repr(float(x))


def curve_lines(device, curve, recv_ts):
    """Line-protocol lines (ms precision) for a complete curve."""
    base = (curve["t"] or int(recv_ts)) * 1000
    tags = f"sweep,device={_tag(device)},src=fw"
    out = []
    for j, (i, V, P) in enumerate(curve["points"]):
        f = f"i={i}i,V={_f(V)},P={_f(P)}"
        if j == 0:
            f += f",n={curve['n']}i"
            for k in ("mpp_V", "mpp_P", "land"):
                if k in curve and math.isfinite(curve[k]):
                    f += f",{k}={_f(curve[k])}"
            if "flag" in curve:
                f += ',flag="' + curve["flag"].replace("\\", "\\\\").replace('"', '\\"') + '"'
        out.append(f"{tags} {f} {base + i}")
    return out


class Influx:
    def __init__(self, url, db, user, password, timeout=15):
        self.url, self.db, self.user, self.password, self.timeout = url.rstrip("/"), db, user, password, timeout

    def write(self, lines):
        q = urllib.parse.urlencode({"db": self.db, "u": self.user, "p": self.password, "precision": "ms"})
        req = urllib.request.Request(f"{self.url}/write?{q}", data="\n".join(lines).encode(), method="POST")
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            if r.status // 100 != 2:
                raise IOError(f"influx write HTTP {r.status}")


# ---------------------------------------------------------------------------------------------
# Archive
# ---------------------------------------------------------------------------------------------
def _num(x):
    return x if math.isfinite(x) else None


def archive_record(device, curve, recv_ts, complete=True, reason=None):
    r = {"device": device, "recv_ts": round(recv_ts, 3), "t": curve["t"], "n": curve["n"],
         "mpp_V": _num(curve["mpp_V"]), "mpp_P": _num(curve["mpp_P"]), "sweeping": curve["sweeping"],
         "flag": curve.get("flag"), "land": _num(curve["land"]) if "land" in curve else None,
         "points": [[V, P] for _, V, P in curve["points"]], "raw": curve["raw"]}
    if not complete:
        r.update(complete=False, reason=reason)
    return r


def dedup_key(device, curve):
    """Identity of a stored curve; t=0 (no clock) falls back to the content."""
    if curve["t"]:
        return device, curve["t"]
    return device, 0, curve["n"], tuple(curve["points"][:3])


class Archive:
    def __init__(self, path):
        self.path = path
        os.makedirs(path, exist_ok=True)

    def file_for(self, ts):
        return os.path.join(self.path, datetime.datetime.fromtimestamp(ts, datetime.timezone.utc)
                            .strftime("%Y-%m-%d") + ".jsonl")

    def append(self, rec):
        with open(self.file_for(rec["recv_ts"]), "a") as f:
            f.write(json.dumps(rec, separators=(",", ":")) + "\n")
            f.flush()
            os.fsync(f.fileno())

    def recent_keys(self, now):
        """{dedup key: has verdict} of the complete curves archived today and yesterday."""
        keys = {}
        for ts in (now - 86400, now):
            fn = self.file_for(ts)
            if not os.path.exists(fn):
                continue
            for rec in read_jsonl(fn):
                c = curve_from_record(rec)
                if c:
                    k = dedup_key(rec["device"], c)
                    keys[k] = keys.get(k, False) or "flag" in c
        return keys


def read_jsonl(fn):
    with open(fn) as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                try:
                    yield json.loads(ln)
                except ValueError:
                    log.warning("%s: bad line skipped", fn)


def curve_from_record(rec):
    """Re-parse an archived record's raw lines; None unless it is a complete, finished curve."""
    if rec.get("complete") is False:
        return None
    p = SweepCurveParser()
    for ln in rec.get("raw", []):
        r = p.feed(ln)
        if r and r["complete"] and r["curve"] and not r["curve"]["sweeping"]:
            return r["curve"]
    return None


# ---------------------------------------------------------------------------------------------
# Collector
# ---------------------------------------------------------------------------------------------
class Device:
    def __init__(self, name):
        self.name = name
        self.parser = SweepCurveParser()
        self.due = None        # time to send the next attempt
        self.tries = 0         # attempts of the current request; 0 = no request open
        self.attempt_end = None


class Collector:
    def __init__(self, influx, archive, send, clock=time.time):
        self.influx, self.archive, self.send, self.clock = influx, archive, send, clock
        self.devs = {}
        self.stored = archive.recent_keys(clock()) if archive else {}
        self.pending = []  # (device, lines) whose Influx write failed
        self.next_write_retry = 0

    def dev(self, name):
        if name not in self.devs:
            self.devs[name] = Device(name)
        return self.devs[name]

    def schedule(self, d, delay):
        """Start a new request; a reply to an older attempt then counts as unrequested."""
        d.due, d.tries, d.attempt_end = self.clock() + delay, 0, None

    def on_line(self, host, line):
        d = self.dev(host)
        plain = ANSI_RE.sub("", line)
        if "Stop sweep" in plain:
            self.schedule(d, STOP_DELAY_S)
        elif re.search(r"sweep (?:verified|flagged):", plain):
            self.schedule(d, VERDICT_DELAY_S)
        r = d.parser.feed(line)
        if r:
            self.on_curve(d, r)

    def on_curve(self, d, r):
        now = self.clock()
        c = r["curve"]
        if r["complete"]:
            if c is None:  # `sc: none`
                if d.tries:
                    self._done(d)
                return
            if c["sweeping"]:
                if d.tries and d.tries < SWEEPING_TRIES:  # a sweep is running: ask again later
                    d.due, d.attempt_end = now + SWEEPING_RETRY_S, None
                elif d.tries:
                    self._done(d)
                return
            if d.tries:
                self._done(d)
            self.store(d.name, c, now)
        elif r["reason"] != "restart":
            log.warning("%s: incomplete curve (%s, %d/%d points)", d.name, r["reason"],
                        len(c["points"]) + c["dropped"], c["n"])
            if d.tries and d.tries <= RETRIES:
                d.due, d.attempt_end = now, None
            elif not c["sweeping"]:
                self.archive.append(archive_record(d.name, c, now, False, r["reason"]))
                if d.tries:
                    self._done(d)

    def _done(self, d):
        d.due = d.attempt_end = None
        d.tries = 0

    def store(self, device, c, recv_ts):
        key = dedup_key(device, c)
        had = self.stored.get(key)
        if had is not None and (had or "flag" not in c):
            log.info("%s: curve t=%d already stored", device, c["t"])
            return
        self.archive.append(archive_record(device, c, recv_ts))
        self.stored[key] = "flag" in c
        lines = curve_lines(device, c, recv_ts)
        if had is not None:  # verdict arrived after the first fetch: update point 0 only
            lines = lines[:1]
        log.info("%s: curve t=%d n=%d points=%d mpp=%.1fW@%.2fV flag=%s", device, c["t"], c["n"],
                 len(c["points"]), c["mpp_P"], c["mpp_V"], c.get("flag", "-"))
        self.write(device, lines)

    def write(self, device, lines):
        try:
            self.influx.write(lines)
        except Exception as e:
            log.error("%s: influx write failed (%s), will retry", device, e)
            self.pending.append((device, lines))
            del self.pending[:-MAX_PENDING_WRITES]
            self.next_write_retry = self.clock() + WRITE_RETRY_S

    def tick(self):
        now = self.clock()
        for d in self.devs.values():
            if d.attempt_end is not None and now >= d.attempt_end:
                d.attempt_end = None
                if d.tries > RETRIES:
                    log.warning("%s: no sweep-curve reply after %d attempts", d.name, d.tries)
                    self._done(d)
                    d.parser.cur = None
                else:
                    d.due = now
            if d.due is not None and now >= d.due:
                d.due = None
                d.tries += 1
                d.attempt_end = now + ATTEMPT_TIMEOUT_S
                log.info("%s: sweep-curve (attempt %d)", d.name, d.tries)
                try:
                    self.send(d.name)
                except Exception as e:
                    log.error("%s: publish failed: %s", d.name, e)
        if self.pending and now >= self.next_write_retry:
            todo, self.pending = self.pending, []
            for device, lines in todo:
                self.write(device, lines)
            if not self.pending:
                log.info("pending influx writes flushed")


def load_env(path):
    if not path or not os.path.exists(path):
        return
    for ln in open(path):
        ln = ln.strip()
        if ln and not ln.startswith("#") and "=" in ln:
            k, v = ln.split("=", 1)
            os.environ.setdefault(k.replace("export ", "").strip(), v.strip().strip("'\""))


def influx_from_env():
    return Influx(os.environ.get("INFLUX_URL", "http://tm.fabi.me:8086"), os.environ.get("INFLUX_DB", "ha_van"),
                  os.environ["INFLUX_USER"], os.environ["INFLUX_PASS"])


def replay(files):
    influx = influx_from_env()
    total = 0
    for fn in files:
        for rec in read_jsonl(fn):
            c = curve_from_record(rec)
            if not c:
                continue
            influx.write(curve_lines(rec["device"], c, rec["recv_ts"]))
            total += 1
    print(f"replayed {total} curves")


def run(args):
    import paho.mqtt.client as mqtt
    from paho.mqtt.enums import CallbackAPIVersion

    rx = queue.Queue()
    client = mqtt.Client(CallbackAPIVersion.VERSION2, client_id=f"fugu-sweep-collector-{socket.gethostname()}")
    client.username_pw_set(os.environ.get("MQTT_USER", "pv"), os.environ["MQTT_PASS"])
    client.reconnect_delay_set(1, 60)

    def on_connect(c, _u, _f, rc, _p=None):
        log.info("connected: %s", rc)
        if not rc.is_failure:
            c.subscribe(TOPIC)

    def on_disconnect(_c, _u, _f, rc, _p=None):
        log.warning("disconnected: %s", rc)

    def on_message(_c, _u, msg):
        parts = msg.topic.split("/")
        if len(parts) == 3:
            for ln in msg.payload.decode("utf-8", "replace").splitlines():
                rx.put((parts[2], ln))

    def send(host):
        client.publish(f"{ROOT}{host}/cmd", "sweep-curve", qos=1)

    client.on_connect, client.on_disconnect, client.on_message = on_connect, on_disconnect, on_message
    col = Collector(influx_from_env(), Archive(args.archive), send)
    host, port = os.environ.get("MQTT_COLLECTOR_HOST", "localhost"), int(os.environ.get("MQTT_PORT", "1882"))
    log.info("start: broker %s:%d, archive %s, %d recent curves known", host, port, args.archive, len(col.stored))
    client.connect_async(host, port, 60)
    client.loop_start()
    while True:
        try:
            h, ln = rx.get(timeout=0.25)
            col.on_line(h, ln)
        except queue.Empty:
            pass
        except Exception:
            log.exception("line handling failed")
        try:
            col.tick()
        except Exception:
            log.exception("tick failed")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--env", action="append", default=[], help="env file(s) to load (repeatable)")
    ap.add_argument("--archive", default=os.path.expanduser("~/pv/sweeps"), help="JSONL archive directory")
    ap.add_argument("--log", default="sweep_collector.log", help="log file ('-' = stderr)")
    ap.add_argument("--replay", nargs="+", metavar="FILE", help="re-import archived JSONL files and exit")
    args = ap.parse_args()
    for e in args.env:
        load_env(os.path.expanduser(e))
    h = logging.StreamHandler() if args.log == "-" or args.replay else \
        RotatingFileHandler(args.log, maxBytes=1_000_000, backupCount=3)
    h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    log.addHandler(h)
    log.setLevel(logging.INFO)
    if args.replay:
        replay(args.replay)
    else:
        run(args)


if __name__ == "__main__":
    main()
