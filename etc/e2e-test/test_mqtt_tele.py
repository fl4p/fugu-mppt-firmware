#!/usr/bin/env python3
"""Exercise the MQTT telemetry lease (`tele-mqtt`, CONFIG_FUGU_WITH_TELE_MQTT) over the broker only.

  1. subscribe pv/tele/<host>, publish `tele-mqtt 30` to pv/log/<host>/cmd (needs cmd_input=1)
  2. the first message carries a symbol-table frame, and >=10 points with I/Ui/P arrive within 5 s
  3. `tele-mqtt 0` -> no further message after 2 s

The device clock must be set (SNTP or `set-time`); with the UDP `tele` service running,
tele.conf needs binary=1.

Usage:
    python etc/e2e-test/test_mqtt_tele.py --name <hostname>
Broker is read from $MQTT_HOST/$MQTT_PORT/$MQTT_USER/$MQTT_PASS or etc/mqtt.env.
"""
import argparse
import os
import sys
import threading
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt required: pip install paho-mqtt")

ETC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ETC_DIR)
from fugu.teledec import Reader, decode_payload
from _harness import Results, wait_for
from test_mqtt_cmd_input import load_env


def frame_types(payload):
    r, out = Reader(payload), []
    while not r.eof():
        n = r.varint()
        out.append(r.take(n)[0])
    return out


class TeleTap:
    def __init__(self, host, port, user, pw, hostname):
        self.topic = f"pv/tele/{hostname}"
        self.cmd_topic = f"pv/log/{hostname}/cmd"
        self.msgs = []   # (arrival monotonic, bytes)
        self._lock = threading.Lock()
        try:
            self._c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except (AttributeError, TypeError):   # paho < 2.0
            self._c = mqtt.Client()
        if user:
            self._c.username_pw_set(user, pw)
        self._c.on_connect = lambda c, *a: c.subscribe(self.topic)
        self._c.on_message = self._on_message
        self._c.connect(host, port, keepalive=30)
        self._c.loop_start()

    def _on_message(self, client, userdata, msg):
        with self._lock:
            self.msgs.append((time.monotonic(), bytes(msg.payload)))

    def since(self, cursor):
        with self._lock:
            return [m for _, m in self.msgs[cursor:]]

    def mark(self):
        with self._lock:
            return len(self.msgs)

    def send(self, cmd):
        self._c.publish(self.cmd_topic, cmd, qos=0)

    def close(self):
        self._c.loop_stop()
        self._c.disconnect()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True, help="device hostname (topic suffix)")
    ap.add_argument("--mqtt", default=None, help="broker host (default $MQTT_HOST)")
    ap.add_argument("--mqtt-port", type=int, default=None)
    args = ap.parse_args()

    load_env(os.path.join(ETC_DIR, "mqtt.env"))
    host = args.mqtt or os.environ.get("MQTT_HOST")
    port = args.mqtt_port or int(os.environ.get("MQTT_PORT", "1883"))
    if not host:
        sys.exit("no broker: set $MQTT_HOST / etc/mqtt.env or pass --mqtt")

    tap = TeleTap(host, port, os.environ.get("MQTT_USER"), os.environ.get("MQTT_PASS"), args.name)
    res = Results()
    try:
        time.sleep(1.0)   # subscription settles
        cur = tap.mark()
        tap.send("tele-mqtt 30")
        tab, points = {}, []

        def enough():
            for m in tap.since(cur)[len(enough.seen):]:
                enough.seen.append(m)
                points.extend(l for l in decode_payload(m[0], m[1:], tab)
                              if all(f"{k}=" in l for k in ("I", "Ui", "P")))
            return len(points) >= 10
        enough.seen = []

        got = wait_for(enough, 5)
        msgs = enough.seen
        res.check("first message starts with a table frame",
                  msgs and msgs[0][0] == 0 and 2 in frame_types(msgs[0][1:]),
                  f"{len(msgs)} msgs" if msgs else "no message")
        res.check(">=10 points with I/Ui/P within 5 s", got, f"{len(points)} points")

        tap.send("tele-mqtt 0")
        time.sleep(2.0)
        cur = tap.mark()
        time.sleep(2.0)
        res.check("tele-mqtt 0 -> silence", tap.mark() == cur, f"{tap.mark() - cur} msgs after stop")
    finally:
        tap.send("tele-mqtt 0")
        tap.close()

    print("\n" + ("ALL CHECKS PASSED" if res.ok() else "SOME CHECKS FAILED"))
    return 0 if res.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
