#!/usr/bin/env python3
"""Store every MPPT sweep curve the devices publish into InfluxDB (measurement `sweep`).

Listens on pv/log/+. When a device logs `Stop sweep` or `sweep verified|flagged:`, it sends the
read-only console command `sweep-curve` to pv/log/<host>/cmd (nothing else is ever sent) and parses
the `sc:` reply. Curves seen on the log without a request (e.g. a dashboard fetch) are stored too.

Each curve is first appended to a daily JSONL archive (<archive>/<UTC date>.jsonl, raw lines
included), then queued for a writer thread that writes it to InfluxDB and appends the record id
to <UTC date>.written (ok, or rejected on a 4xx); a failed write is retried every 60 s, and records
of the last 7 days without a mark are re-queued at start. In InfluxDB: one point per (V, P) pair, time = sweep end (s) + index (ms),
tags device + src=fw, fields i, V, P; point 0 also mpp_V, mpp_P, n and, once the sweep has a
verdict, flag ("" clean, else the reason) and land. Re-writing the same curve overwrites the same
points, so `--replay FILE.jsonl` re-imports an archive idempotently. One collector per archive
directory (flock); test runs: --archive <dir> --no-send --no-influx --log -.

Config (environment or env files, see README.md): MQTT_USER, MQTT_PASS, MQTT_PORT,
MQTT_COLLECTOR_HOST (default localhost), INFLUX_URL, INFLUX_DB, INFLUX_USER, INFLUX_PASS.
"""
import argparse
import collections
import datetime
import fcntl
import json
import logging
import math
import os
import queue
import re
import socket
import threading
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
MAX_PENDING_WRITES = 5000  # in memory; older unwritten records stay in the archive
KEEP_KEYS_S = 2 * 86400    # dedup memory
RX_MAX = 20000

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


def curve_lines(device, curve, base_s):
    """Line-protocol lines (ms precision) for a complete curve; point i at base_s + i ms."""
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
        out.append(f"{tags} {f} {base_s * 1000 + i}")
    return out


SKIP = "skip"  # write() result: not written and not marked (--no-influx)


class InfluxRejected(Exception):
    """Influx answered 4xx: retrying the same lines cannot succeed."""


class Influx:
    def __init__(self, url, db, user, password, timeout=15):
        self.url, self.db, self.user, self.password, self.timeout = url.rstrip("/"), db, user, password, timeout

    def write(self, lines):
        q = urllib.parse.urlencode({"db": self.db, "u": self.user, "p": self.password, "precision": "ms"})
        req = urllib.request.Request(f"{self.url}/write?{q}", data="\n".join(lines).encode(), method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                if r.status // 100 != 2:
                    raise IOError(f"influx write HTTP {r.status}")
        except urllib.error.HTTPError as e:
            body = e.read()[:200].decode("utf-8", "replace")
            if e.code in (400, 422):  # bad data; 404 (no db), 413 etc. may heal: retry
                raise InfluxRejected(f"HTTP {e.code}: {body}") from None
            raise IOError(f"HTTP {e.code}: {body}") from None


# ---------------------------------------------------------------------------------------------
# Archive: <day>.jsonl holds the curves, <day>.written the ids of the records Influx has (or rejected)
# ---------------------------------------------------------------------------------------------
def _num(x):
    return x if math.isfinite(x) else None


def archive_record(device, curve, recv_ts, base_s=None, complete=True, reason=None):
    r = {"device": device, "recv_ts": round(recv_ts, 3), "t": curve["t"], "n": curve["n"],
         "mpp_V": _num(curve["mpp_V"]), "mpp_P": _num(curve["mpp_P"]), "sweeping": curve["sweeping"],
         "flag": curve.get("flag"), "land": _num(curve["land"]) if "land" in curve else None,
         "points": [[V, P] for _, V, P in curve["points"]], "raw": curve["raw"]}
    if base_s is not None and base_s != curve["t"]:
        r["base_s"] = base_s  # t=0: timestamp base of the first fetch of this curve
    if not complete:
        r.update(complete=False, reason=reason)
    return r


def record_id(rec):
    return f"{rec['device']}:{rec['t']}:{round(rec['recv_ts'] * 1000)}"


def record_base(rec):
    return rec.get("base_s") or rec["t"] or int(rec["recv_ts"])


def dedup_key(device, curve):
    """Identity of a stored curve; t=0 (no clock) falls back to the content."""
    if curve["t"]:
        return device, curve["t"]
    return device, 0, curve["n"], tuple(curve["points"][:3])


def _day(ts):
    return datetime.datetime.fromtimestamp(ts, datetime.timezone.utc).strftime("%Y-%m-%d")


class Archive:
    def __init__(self, path):
        self.path = path
        os.makedirs(path, exist_ok=True)

    def file_for(self, ts, ext=".jsonl"):
        return os.path.join(self.path, _day(ts) + ext)

    @staticmethod
    def _append(fn, text):
        with open(fn, "a") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())

    def append(self, rec):
        self._append(self.file_for(rec["recv_ts"]), json.dumps(rec, separators=(",", ":")) + "\n")

    def mark(self, rec, status):
        """Record that Influx has rec (status ok) or refused it for good (rejected ...)."""
        self._append(self.file_for(rec["recv_ts"], ".written"), f"{record_id(rec)} {status}\n")

    def days(self, now, n):
        """Archive files of the last n UTC days, oldest first."""
        out = []
        for k in range(n - 1, -1, -1):
            fn = self.file_for(now - k * 86400)
            if os.path.exists(fn):
                out.append(fn)
        return out

    def scan(self, now, key_days=2, backfill_days=7):
        """({dedup key: (has verdict, base_s, recv_ts)} of the last key_days, unwritten records of
        the last backfill_days)."""
        keys, unwritten = {}, []
        files = self.days(now, max(key_days, backfill_days))
        for fn in files:
            wfn = fn[:-len(".jsonl")] + ".written"
            done = set()
            if os.path.exists(wfn):
                done = {ln.split(" ", 1)[0] for ln in open(wfn) if ln.strip()}
            recent = fn in files[-key_days:]
            for rec in read_jsonl(fn):
                c = curve_from_record(rec)
                if not c:
                    continue
                if recent:
                    k = dedup_key(rec["device"], c)
                    had = keys.get(k)
                    keys[k] = (bool(had and had[0]) or "flag" in c, had[1] if had else record_base(rec),
                               rec["recv_ts"])
                if record_id(rec) not in done:
                    unwritten.append(rec)
        return keys, unwritten


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


class Writer:
    """Writes archived records to Influx in its own thread, in order; a failed write is retried
    after a backoff before anything newer, a rejected (4xx) one is marked and dropped."""

    def __init__(self, influx, archive, backoff_s=WRITE_RETRY_S, max_queue=MAX_PENDING_WRITES):
        self.influx, self.archive, self.backoff_s, self.max_queue = influx, archive, backoff_s, max_queue
        self.q = collections.deque()
        self.cv = threading.Condition()
        self.stopped = False
        self.thread = None

    def submit(self, rec, lines):
        with self.cv:
            self.q.append((rec, lines))
            if len(self.q) > self.max_queue:
                old, _ = self.q.popleft()
                log.error("write queue full: %s dropped from memory, stays unwritten in the archive "
                          "(re-queued on a start within 7 days, else --replay)", record_id(old))
            self.cv.notify()

    def pending(self):
        with self.cv:
            return len(self.q)

    def start(self):
        self.thread = threading.Thread(target=self._run, name="influx-writer", daemon=True)
        self.thread.start()

    def stop(self):
        with self.cv:
            self.stopped = True
            self.cv.notify()

    def _run(self):
        failing = False
        while True:
            with self.cv:
                while not self.q and not self.stopped:
                    self.cv.wait()
                if self.stopped:
                    return
                rec, lines = self.q[0]
            try:
                if self.influx.write(lines) == SKIP:
                    with self.cv:
                        if self.q and self.q[0][0] is rec:
                            self.q.popleft()
                    continue
                status = "ok"
            except InfluxRejected as e:
                log.error("%s: influx rejected the curve, not retried: %s", record_id(rec), e)
                status = "rejected"
            except Exception as e:
                if not failing:
                    log.error("influx write failed (%s), retrying every %g s; %d queued", e, self.backoff_s,
                              self.pending())
                failing = True
                until = time.monotonic() + self.backoff_s
                with self.cv:
                    while not self.stopped and time.monotonic() < until:
                        self.cv.wait(until - time.monotonic())
                continue
            if failing:
                log.info("influx writes recovered")
                failing = False
            try:
                self.archive.mark(rec, status)
            except OSError as e:
                log.error("%s: cannot mark written: %s", record_id(rec), e)
            with self.cv:
                if self.q and self.q[0][0] is rec:
                    self.q.popleft()


# ---------------------------------------------------------------------------------------------
# Collector
# ---------------------------------------------------------------------------------------------
class Device:
    def __init__(self, name):
        self.name = name
        self.parser = SweepCurveParser()
        self.due = None        # time to send the next attempt
        self.tries = 0         # sends of the current request; 0 = no request open
        self.fails = 0         # of these, timeouts and incomplete replies
        self.busy = 0          # of these, sweeping=1 replies
        self.attempt_end = None


class Collector:
    def __init__(self, writer, archive, send, clock=time.time):
        self.writer, self.archive, self.send, self.clock = writer, archive, send, clock
        self.devs = {}
        self.stored, unwritten = archive.scan(clock())
        for rec in unwritten:
            c = curve_from_record(rec)
            log.info("%s: archived but not written, queued", record_id(rec))
            writer.submit(rec, curve_lines(rec["device"], c, record_base(rec)))
        self.next_prune = clock() + 3600

    def dev(self, name):
        if name not in self.devs:
            self.devs[name] = Device(name)
        return self.devs[name]

    def schedule(self, d, delay):
        """Start a new request; a reply to an older attempt then counts as unrequested."""
        d.due, d.attempt_end = self.clock() + delay, None
        d.tries = d.fails = d.busy = 0

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
                if d.tries:
                    d.busy += 1
                    if d.busy < SWEEPING_TRIES:  # a sweep is running: ask again later
                        d.due, d.attempt_end = now + SWEEPING_RETRY_S, None
                    else:
                        log.warning("%s: still sweeping after %d asks", d.name, d.busy)
                        self._done(d)
                return
            if d.tries:
                self._done(d)
            self.store(d.name, c, now)
        elif r["reason"] != "restart":
            log.warning("%s: incomplete curve (%s, %d/%d points)", d.name, r["reason"],
                        len(c["points"]) + c["dropped"], c["n"])
            if d.tries and d.fails < RETRIES:
                d.fails += 1
                d.due, d.attempt_end = now, None
            elif not c["sweeping"]:
                self.archive.append(archive_record(d.name, c, now, complete=False, reason=r["reason"]))
                if d.tries:
                    self._done(d)

    def _done(self, d):
        d.due = d.attempt_end = None
        d.tries = d.fails = d.busy = 0

    def store(self, device, c, recv_ts):
        key = dedup_key(device, c)
        had = self.stored.get(key)
        if had is not None and (had[0] or "flag" not in c):
            log.info("%s: curve t=%d already stored", device, c["t"])
            return
        base = had[1] if had else (c["t"] or int(recv_ts))
        rec = archive_record(device, c, recv_ts, base)
        self.archive.append(rec)
        self.stored[key] = ("flag" in c, base, recv_ts)
        lines = curve_lines(device, c, base)
        if had is not None:  # verdict arrived after the first fetch: update point 0 only
            lines = lines[:1]
        log.info("%s: curve t=%d n=%d points=%d mpp=%.1fW@%.2fV flag=%s", device, c["t"], c["n"],
                 len(c["points"]), c["mpp_P"], c["mpp_V"], c.get("flag", "-"))
        self.writer.submit(rec, lines)

    def tick(self):
        now = self.clock()
        for d in self.devs.values():
            if d.attempt_end is not None and now >= d.attempt_end:
                d.attempt_end = None
                if d.fails >= RETRIES:
                    log.warning("%s: no sweep-curve reply after %d attempts", d.name, d.tries)
                    self._done(d)
                    d.parser.cur = None
                else:
                    d.fails += 1
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
        if now >= self.next_prune:
            self.next_prune = now + 3600
            self.stored = {k: v for k, v in self.stored.items() if v[2] > now - KEEP_KEYS_S}


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


def replay(files, archive=None):
    """Re-import archived records (idempotent: same timestamps); marks them when the archive is given."""
    influx = influx_from_env()
    total = 0
    for fn in files:
        for rec in read_jsonl(fn):
            c = curve_from_record(rec)
            if not c:
                continue
            try:
                influx.write(curve_lines(rec["device"], c, record_base(rec)))
                status = "ok"
            except InfluxRejected as e:
                print(f"{record_id(rec)}: rejected: {e}")
                status = "rejected"
            if archive and os.path.dirname(os.path.abspath(fn)) == os.path.abspath(archive.path):
                archive.mark(rec, status)
            total += 1
    print(f"replayed {total} curves")


class _NoInflux:
    def write(self, lines):
        log.info("no-influx: %d lines not written", len(lines))
        return SKIP


def lock_archive(path):
    """Exclusive lock on the archive directory: a second collector on it refuses to start."""
    f = open(os.path.join(path, ".lock"), "w")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        raise SystemExit(f"another sweep collector holds {path}/.lock; for a test run use "
                         f"--archive <other dir> --no-send --no-influx --log -")
    return f


def run(args):
    import paho.mqtt.client as mqtt
    from paho.mqtt.enums import CallbackAPIVersion

    archive = Archive(args.archive)
    lock = lock_archive(args.archive)  # noqa: F841 (held for the process lifetime)
    rx = queue.Queue(maxsize=RX_MAX)
    dropped = [0]
    client = mqtt.Client(CallbackAPIVersion.VERSION2,
                         client_id=f"fugu-sweep-collector-{socket.gethostname()}-{os.urandom(3).hex()}")
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
                try:
                    rx.put_nowait((parts[2], ln))
                except queue.Full:
                    dropped[0] += 1

    def send(host):
        if args.no_send:
            log.info("no-send: sweep-curve to %s not sent", host)
        else:
            client.publish(f"{ROOT}{host}/cmd", "sweep-curve", qos=1)

    client.on_connect, client.on_disconnect, client.on_message = on_connect, on_disconnect, on_message
    writer = Writer(_NoInflux() if args.no_influx else influx_from_env(), archive)
    col = Collector(writer, archive, send)
    writer.start()
    host, port = os.environ.get("MQTT_COLLECTOR_HOST", "localhost"), int(os.environ.get("MQTT_PORT", "1882"))
    log.info("start: broker %s:%d, archive %s, %d recent curves known, %d unwritten queued", host, port,
             args.archive, len(col.stored), writer.pending())
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
        if dropped[0]:
            log.error("rx queue full: %d lines dropped", dropped[0])
            dropped[0] = 0
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
    ap.add_argument("--no-send", action="store_true", help="test run: never publish sweep-curve")
    ap.add_argument("--no-influx", action="store_true", help="test run: archive only, no Influx writes")
    args = ap.parse_args()
    for e in args.env:
        load_env(os.path.expanduser(e))
    h = logging.StreamHandler() if args.log == "-" or args.replay else \
        RotatingFileHandler(args.log, maxBytes=1_000_000, backupCount=3)
    h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    log.addHandler(h)
    log.setLevel(logging.INFO)
    if args.replay:
        replay(args.replay, Archive(args.archive))
    else:
        run(args)


if __name__ == "__main__":
    main()
