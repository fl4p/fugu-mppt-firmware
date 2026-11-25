#!/usr/bin/env python3
"""Host unit tests for the MQTT console: broker-echo delivery check and the read-only resend.

Run: `python3 etc/test_mqtt_console.py` or pytest.
"""
import os
import sys
import threading
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fugu_console as fc  # noqa: E402
from fugu.transport import MqttTransport  # noqa: E402


class _FakeClient:
    """paho stand-in: publish() optionally routes the message back like a real broker."""

    def __init__(self, transport, echo=True, rc=0):
        self.t, self.echo, self.rc, self.sent = transport, echo, rc, []

    def publish(self, topic, payload, qos=0):
        self.sent.append((topic, payload, qos))
        if self.echo and self.rc == 0:
            msg = types.SimpleNamespace(topic=topic, payload=payload.encode())
            threading.Thread(target=self.t._on_cmd_echo, args=(None, None, msg)).start()
        return types.SimpleNamespace(rc=self.rc)


def _transport(**kw):
    t = MqttTransport("broker", device="fry")
    t.hostname = "fry"
    t._client = _FakeClient(t, **kw)
    return t


def test_write_confirmed_by_broker_echo():
    t = _transport()
    t.write(b"status\n")
    assert t._client.sent == [("pv/log/fry/cmd", "status", 0)]


def test_write_raises_when_broker_never_echoes():
    t = _transport(echo=False)
    t._echo_timeout = 0.2
    try:
        t.write(b"status\n")
    except ConnectionError:
        return
    raise AssertionError("a command the broker never routed back must raise")


def test_write_raises_on_publish_error():
    t = _transport(rc=4)
    try:
        t.write(b"status\n")
    except ConnectionError:
        return
    raise AssertionError("a failed publish must raise ConnectionError")


def test_other_device_echo_does_not_confirm():
    t = _transport(echo=False)
    t._echo_timeout = 0.2
    msg = types.SimpleNamespace(topic="pv/log/flat/cmd", payload=b"status")
    threading.Timer(0.05, t._on_cmd_echo, args=(None, None, msg)).start()
    try:
        t.write(b"status\n")
    except ConnectionError:
        return
    raise AssertionError("an echo on another device's topic must not confirm")


def test_echo_of_another_command_does_not_confirm():
    t = _transport(echo=False)
    t._echo_timeout = 0.2
    msg = types.SimpleNamespace(topic="pv/log/fry/cmd", payload=b"heap")
    threading.Timer(0.05, t._on_cmd_echo, args=(None, None, msg)).start()
    try:
        t.write(b"status\n")
    except ConnectionError:
        return
    raise AssertionError("an echo of a different command must not confirm")


class _FakeMqtt(MqttTransport):
    def __init__(self):
        pass


class _FakeCon:
    def __init__(self, replies):
        self.transport, self.replies, self.sent = _FakeMqtt(), list(replies), []

    def command(self, cmd, timeout, echo=None):
        self.sent.append((cmd, timeout))
        return self.replies.pop(0)


def test_resends_lost_read_only_command_once():
    lost = fc.Reply(["V=1/2 status line"], timed_out=True)
    con = _FakeCon([lost, fc.Reply(["balance due 1/2"], ok=True)])
    assert fc.dispatch_command(con, "balance", None).ok
    assert con.sent == [("balance", fc.MQTT_REPLY_TIMEOUT)] * 2


def test_no_resend_when_received_or_state_changing():
    slow = fc.Reply(["I (1) main: received serial command: 'balance'"], timed_out=True)
    con = _FakeCon([slow])
    assert fc.dispatch_command(con, "balance", None).timed_out and len(con.sent) == 1
    con = _FakeCon([fc.Reply([], timed_out=True)])
    assert fc.dispatch_command(con, "restart", None).timed_out and len(con.sent) == 1


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    fails = 0
    for t in tests:
        try:
            t()
            print(f"  [PASS] {t.__name__}")
        except Exception as e:
            fails += 1
            print(f"  [FAIL] {t.__name__}: {e!r}")
    print(f"\n{len(tests) - fails}/{len(tests)} passed")
    sys.exit(1 if fails else 0)
