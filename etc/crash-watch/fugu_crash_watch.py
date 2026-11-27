#!/usr/bin/env python3
"""Follow the fleet console log and report crashes, critical errors and silence to GlitchTip.

Runs on the host that writes the log (mqtt_console.py). On a new core dump it pulls it over MQTT
(`coredump get`, read-only) so a later crash cannot overwrite it. Events also go to a JSONL file.
Env: FCW_LOG, FCW_DEVICES, FCW_DSN, FCW_DIR, FCW_SILENCE_S, FCW_CONSOLE (fugu_console.py path).
"""
import collections, datetime, json, os, re, subprocess, sys, threading, time, urllib.request, uuid

LOG = os.environ.get("FCW_LOG", os.path.expanduser("~/pv/fugu_console.log"))
DEVICES = os.environ.get("FCW_DEVICES", "fry,flat").split(",")
DSN = os.environ.get("FCW_DSN", "")
DIR = os.environ.get("FCW_DIR", os.path.expanduser("~/pv/crash-watch"))
SILENCE_S = float(os.environ.get("FCW_SILENCE_S", "300"))
CONSOLE = os.environ.get("FCW_CONSOLE", os.path.join(DIR, "etc", "fugu_console.py"))
DEDUP_S = 600

LINE = re.compile(r"^(\S+ \S+)\s+([\w-]+): (.*)$")
VERSION = re.compile(r"app fugu-firmware v\S+ (\S+)")
BENIGN_RESET = {"SW"}
SIGS = [  # (signature, level, regex)
    ("reset", None, re.compile(r"reset reason: (\S+)")),
    ("coredump", "error", re.compile(r"coredump: new dump")),
    ("adc-stall", "error", re.compile(r"ADC stall \d+ ?ms")),
    ("adc-nosample", "error", re.compile(r"Never got a sample")),
    ("ina-timeout", "error", re.compile(r"ina22x: \d+ timeout!")),
    ("loop-latency", "error", re.compile(r"Loop latency high")),
    ("abort", "error", re.compile(r"abort\(\) was called|Guru Meditation|assert failed|pbuf_free: p->ref")),
]


def now():
    return datetime.datetime.now(datetime.timezone.utc)


class Watch:
    def __init__(self):
        self.ctx = {d: collections.deque(maxlen=60) for d in DEVICES}
        self.version = {d: "?" for d in DEVICES}
        self.last_line = {d: time.time() for d in DEVICES}
        self.silent = {d: False for d in DEVICES}
        self.sent = {}
        self.lock = threading.Lock()
        os.makedirs(os.path.join(DIR, "coredumps"), exist_ok=True)

    def emit(self, dev, sig, level, msg, extra=None):
        key = (dev, sig)
        t = time.time()
        with self.lock:
            if t - self.sent.get(key, 0) < DEDUP_S:
                return
            self.sent[key] = t
        ev = {"time": now().isoformat(), "device": dev, "sig": sig, "level": level, "msg": msg,
              "version": self.version.get(dev, "?"), **(extra or {})}
        with open(os.path.join(DIR, "events.jsonl"), "a") as f:
            f.write(json.dumps(ev) + "\n")
        print(f"{ev['time']} {level} {dev} {sig}: {msg}", flush=True)
        if DSN:
            glitchtip(ev, list(self.ctx.get(dev, ())))

    def line(self, raw):
        m = LINE.match(raw)
        if not m or m.group(2) not in self.ctx:
            return
        dev, msg = m.group(2), m.group(3)
        self.ctx[dev].append(raw.rstrip())
        self.last_line[dev] = time.time()
        if self.silent[dev]:
            self.silent[dev] = False
            self.emit(dev, "back", "info", "log lines again")
        v = VERSION.search(msg)
        if v:
            self.version[dev] = v.group(1)
        for sig, level, rx in SIGS:
            r = rx.search(msg)
            if not r:
                continue
            if sig == "reset":
                reason = r.group(1)
                if reason in BENIGN_RESET:
                    continue
                level = "warning" if reason in ("POWERON", "EXT") else "error"
                self.emit(dev, f"reset-{reason}", level, msg)
            else:
                self.emit(dev, sig, level, msg)
                if sig == "coredump":
                    threading.Thread(target=self.pull, args=(dev,), daemon=True).start()

    def pull(self, dev):
        time.sleep(60)
        out = os.path.join(DIR, "coredumps", f"{dev}-{now():%Y%m%dT%H%M%SZ}")
        os.makedirs(out)
        try:
            p = subprocess.run([sys.executable, CONSOLE, "--mqtt", "--name", dev, "--coredump", "get"],
                               cwd=out, capture_output=True, text=True, timeout=600)
            ok = os.path.exists(os.path.join(out, "coredump.bin"))
            tail = (p.stdout + p.stderr)[-1500:]
        except Exception as e:  # noqa: BLE001
            ok, tail = False, repr(e)
        with open(os.path.join(out, "pull.log"), "w") as f:
            f.write(tail)
        self.sent.pop((dev, "coredump-pull"), None)
        self.emit(dev, "coredump-pull", "info" if ok else "error",
                  f"coredump {'saved' if ok else 'pull FAILED'}: {out}", {"pull_log": tail[-600:]})

    def silence_loop(self):
        while True:
            time.sleep(30)
            for d in DEVICES:
                age = time.time() - self.last_line[d]
                if age > SILENCE_S and not self.silent[d]:
                    self.silent[d] = True
                    self.emit(d, "silent", "warning", f"no log line for {age:.0f} s (BLE backup link?)")


def glitchtip(ev, ctx):
    key, rest = DSN.split("//", 1)[1].split("@", 1)
    host, pid = rest.rsplit("/", 1)
    eid = uuid.uuid4().hex
    body = {"event_id": eid, "timestamp": ev["time"], "level": ev["level"], "platform": "other",
            "logger": "fugu-crash-watch", "server_name": ev["device"], "release": ev["version"],
            "message": {"formatted": f"{ev['device']}: {ev['sig']}: {ev['msg']}"[:500]},
            "fingerprint": ["fugu", ev["device"], ev["sig"]],
            "tags": {"device": ev["device"], "sig": ev["sig"], "fw": ev["version"]},
            "extra": {"context": "\n".join(ctx[-40:]), **{k: v for k, v in ev.items() if k == "pull_log"}}}
    env = "\n".join([json.dumps({"event_id": eid, "sent_at": ev["time"]}), json.dumps({"type": "event"}),
                     json.dumps(body)]) + "\n"
    url = f"https://{host}/api/{pid}/envelope/?sentry_key={key}&sentry_version=7"
    try:
        req = urllib.request.Request(url, env.encode(), {"Content-Type": "application/x-sentry-envelope"})
        urllib.request.urlopen(req, timeout=15).read()
    except Exception as e:  # noqa: BLE001
        print(f"glitchtip send failed: {e!r}", flush=True)


def main():
    w = Watch()
    if len(sys.argv) > 1 and sys.argv[1] == "--test":  # feed lines from stdin, no silence loop
        for raw in sys.stdin:
            w.line(raw)
        return
    threading.Thread(target=w.silence_loop, daemon=True).start()
    p = subprocess.Popen(["tail", "-F", "-n", "0", LOG], stdout=subprocess.PIPE, text=True, errors="replace")
    for raw in p.stdout:
        w.line(raw)
    sys.exit(f"tail exited {p.wait()}")


if __name__ == "__main__":
    main()
