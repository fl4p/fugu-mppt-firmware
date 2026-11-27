#!/usr/bin/env python3
"""Archive every PeakScan near-point probe line the devices log, before the console log rotates.

Listens on pv/log/+ (the same MQTT log mirror `fugu-sweep-collector` reads) and appends each
`scan near:` line to <archive>/<UTC date>.jsonl, one JSON object per probe:

  {"device": "flat", "recv_ts": "2026-11-27T15:12:34Z", "v0": 57.4, "minus": 0.939, "plus": 1.003,
   "ref_before": 366.4, "ref_after": 368.6, "drift": false, "near_only": true, "raw": "..."}

Why: `tracker.conf::scan_near_v` probes are log-only — nothing puts them in InfluxDB — so the only
other copy is ~/pv/fugu_console.log on havan, which rotates. That is why the >=300 W sample count
stays at one or two a day and a conclusion drawn from one day's file changes the next.

It sends no commands (read-only) and writes nothing but its archive and its log.

Config (environment or env files): MQTT_USER, MQTT_PASS, MQTT_PORT, MQTT_COLLECTOR_HOST.
"""
import argparse
import datetime
import fcntl
import json
import logging
import os
import queue
import re
import socket
import sys
import time
from logging.handlers import RotatingFileHandler

TOPIC = "pv/log/+"
RX_MAX = 20000
NEAR_RE = re.compile(r"scan near: v0 ([\d.]+) V, -2\.0V (?:x(\S+)|n/a), \+2\.0V (?:x(\S+)|n/a), "
                     r"ref ([\d.]+)->([\d.]+) W(.*)")

log = logging.getLogger("near-collector")


def num(s):
    """a ratio, or None for the `n/a` the firmware writes when a probe was out of range"""
    if s is None:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def parse_near_line(ln):
    """the probe's fields, or None if this is not a `scan near:` line"""
    m = NEAR_RE.search(ln)
    if not m:
        return None
    v0, minus, plus, p0, p1, rest = m.groups()
    return {"v0": float(v0), "minus": num(minus), "plus": num(plus),
            "ref_before": float(p0), "ref_after": float(p1),
            "drift": "drift" in rest, "near_only": "near only" in rest}


def load_env(path):
    if not path or not os.path.exists(path):
        return
    for ln in open(path):
        ln = ln.strip()
        if ln and not ln.startswith("#") and "=" in ln:
            k, v = ln.split("=", 1)
            os.environ.setdefault(k.replace("export ", "").strip(), v.strip().strip("'\""))


def lock_archive(d):
    """one collector per archive directory: an flock the kernel drops when the holder dies"""
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, ".lock")
    try:
        f = open(path, "w")
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        raise SystemExit(f"another near collector holds {path}; for a test run use "
                         f"--archive <other dir> --log -")
    return f


def append(archive, device, rec, raw, now=None):
    ts = now or datetime.datetime.now(datetime.timezone.utc)
    day = ts.strftime("%Y-%m-%d")
    out = dict(rec, device=device, recv_ts=ts.strftime("%Y-%m-%dT%H:%M:%SZ"), raw=raw)
    with open(os.path.join(archive, day + ".jsonl"), "a") as fh:
        fh.write(json.dumps(out, sort_keys=True) + "\n")
    return out


def run(args):
    import paho.mqtt.client as mqtt
    from paho.mqtt.enums import CallbackAPIVersion

    lock_archive(args.archive)  # held for the process lifetime
    rx = queue.Queue(maxsize=RX_MAX)
    dropped = [0]
    seen, stored = [0], [0]
    client = mqtt.Client(CallbackAPIVersion.VERSION2,
                         client_id=f"fugu-near-collector-{socket.gethostname()}-{os.urandom(3).hex()}")
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

    client.on_connect, client.on_disconnect, client.on_message = on_connect, on_disconnect, on_message
    host, port = os.environ.get("MQTT_COLLECTOR_HOST", "localhost"), int(os.environ.get("MQTT_PORT", "1882"))
    log.info("start: broker %s:%d, archive %s", host, port, args.archive)
    client.connect_async(host, port, 60)
    client.loop_start()
    t_report = time.monotonic()
    while True:
        try:
            h, ln = rx.get(timeout=0.25)
            seen[0] += 1
            rec = parse_near_line(ln)
            if rec:
                append(args.archive, h, rec, ln)
                stored[0] += 1
                log.info("%s: v0 %.1f, -2V %s, +2V %s, ref %.1f->%.1f%s%s", h, rec["v0"],
                         rec["minus"], rec["plus"], rec["ref_before"], rec["ref_after"],
                         ", drift" if rec["drift"] else "", ", near only" if rec["near_only"] else "")
        except queue.Empty:
            pass
        except Exception:
            log.exception("line handling failed")
        if dropped[0]:
            log.error("rx queue full: %d lines dropped", dropped[0])
            dropped[0] = 0
        if time.monotonic() - t_report > 1800:
            log.info("stats: %d log lines seen, %d probes stored", seen[0], stored[0])
            t_report = time.monotonic()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", action="append", default=[], help="env file with MQTT credentials")
    ap.add_argument("--archive", default=os.path.expanduser("~/pv/near"))
    ap.add_argument("--log", default=None)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    for e in args.env:
        load_env(e)
    handlers = [logging.StreamHandler(sys.stdout)] if args.log in (None, "-") else \
               [RotatingFileHandler(args.log, maxBytes=1 << 20, backupCount=3)]
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s", handlers=handlers)
    if "MQTT_PASS" not in os.environ:
        raise SystemExit("MQTT_PASS is not set (see --env)")
    run(args)


if __name__ == "__main__":
    main()
