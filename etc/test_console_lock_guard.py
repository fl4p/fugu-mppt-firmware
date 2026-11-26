#!/usr/bin/env python3
"""Host tests for fugu_console's lock guard (command classifier, guard decision, refusal before
sending, attribution log); no device, a fake lock.sh. Run: .venv/bin/python3 etc/test_console_lock_guard.py"""
import json
import os
import queue
import sys
import tempfile
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.modules.setdefault("serial", types.ModuleType("serial"))

import fugu_console as F  # noqa: E402
from fugu.transport import Transport  # noqa: E402

E, K = F.load_ro_list(), F.load_known_verbs()

RO = ["status", "bootinfo", "balance", "sensor", "sensor avg", "rt-stats", "get-config tele.conf",
      "getc mqtt.conf host", "help", "hostname", "hn", "ip", "heap", "heap tasks", "tasks", "svc list",
      "svc", "coredump info", "coredump get", "peek 0x3fc00000 4", "peek g_app", "sym loop",
      "peek-struct g_app", "pwr", "pwr locks", "uptime", "mem", "rect-cal status", "conf-hash",
      "status;;bootinfo"]
RW = ["restart", "reset", "reboot", "dc 0", "dc 100 50", "mppt", "psu 13.5", "psu off", "pv off", "sweep",
      "+5", "-20", "set-config tele.conf log_level info", "setc a b c", "del-config a b", "ota http://h/f",
      "ota-ble begin", "bf 1", "panel 0", "fan 1", "sync on", "svc on mqtt", "svc off tele",
      "svc restart mqtt", "rect-cal", "rect-cal force", "rect-cal abort", "measure-coil l0", "selftest bflow",
      "selftest abort", "balance now", "balance skip", "hn newname", "coredump erase", "pwr cpu 80",
      "wifi off", "wifi-add a:b", "log main info", "status;;restart", "status ;; dc 0"]


class Classifier(unittest.TestCase):
    def test_firmware_list_parsed(self):
        self.assertIn("status", E)
        self.assertIn("restart", K)
        self.assertIn("set-config", K)

    def test_read_only(self):
        for c in RO:
            self.assertEqual(F.classify(c, E, K)[0], "ro", c)

    def test_state_changing(self):
        for c in RW:
            self.assertEqual(F.classify(c, E, K)[0], "rw", c)

    def test_unknown_fails_closed(self):
        kind, why = F.classify("frobnicate 3", E, K)
        self.assertEqual(kind, "unknown")
        self.assertIn("not a known command", why)

    def test_no_firmware_list_means_everything_guarded(self):
        self.assertEqual(F.classify("status", list(F._RO_HOST), K)[0], "rw")


class FakeRun:
    """Stands in for subprocess.run on lock.sh: `mine` answers from `mine`, `status` from `holder`."""

    def __init__(self, mine=(), holder="HELD fry by pid 4242, session other-1234", held=()):
        self.mine, self.holder, self.held, self.calls = set(mine), holder, dict.fromkeys(held), []

    def __call__(self, argv, **kw):
        self.calls.append(argv[1:])
        verb, name = argv[1], argv[2]
        if verb == "mine":
            return types.SimpleNamespace(returncode=0 if name in self.mine else 1, stdout="", stderr="")
        if name.startswith("ota-"):  # OTA locks are free unless listed in `held`
            out = (f"HELD {name} by pid 77, session ota-1\nnote=OTA by other\n" if name in self.held
                   else f"FREE {name}\n")
            return types.SimpleNamespace(returncode=1 if name in self.held else 0, stdout=out, stderr="")
        return types.SimpleNamespace(returncode=0, stdout=self.holder, stderr="")


def guard(run=None, **kw):
    return F.LockGuard(protected={"fry", "flat"}, run=run or FakeRun(), who="test (00000000)",
                       log_path=kw.pop("log_path", os.devnull), **kw)


class Decision(unittest.TestCase):
    def test_read_only_never_asks_lock_sh(self):
        run = FakeRun()
        g = guard(run)
        for c in RO:
            self.assertTrue(g.allowed(c, "fry"))
        self.assertEqual(run.calls, [])

    def test_refused_without_lock(self):
        run = FakeRun()
        self.assertFalse(guard(run).allowed("dc 0", "fry"))
        self.assertIn(["status", "fry", "--brief"], run.calls)

    def test_allowed_with_device_lock(self):
        self.assertTrue(guard(FakeRun(mine={"flat"})).allowed("restart", "flat"))

    def test_ota_lock_counts_only_for_ota(self):
        g = guard(FakeRun(mine={"ota-fry"}))
        self.assertTrue(g.allowed("ota http://h/f.bin", "fry"))
        self.assertFalse(g.allowed("dc 0", "fry"))

    def test_other_device_lock_does_not_count(self):
        self.assertFalse(guard(FakeRun(mine={"flat"})).allowed("mppt", "fry"))

    def test_bench_device_not_guarded(self):
        run = FakeRun()
        g = guard(run)
        for h in ("fugu-esp32s3-4028", "bench139c", "fboost", "fbuck"):
            self.assertTrue(g.allowed("restart", h))
        self.assertEqual(run.calls, [])

    def test_hostname_case_insensitive(self):
        self.assertFalse(guard().allowed("restart", "FRY"))

    def test_unconfirmed_hostname_refused(self):
        self.assertFalse(guard().allowed("restart", None))

    def test_no_lock_guard_allows(self):
        self.assertTrue(guard(enabled=False).allowed("restart", "fry"))
        self.assertTrue(guard(enabled=False).allowed("restart", None))

    def test_name_hint(self):
        g = guard()
        self.assertEqual(g.protected_host("fry"), "fry")
        self.assertEqual(g.protected_host("fugu-flat"), "flat")
        self.assertEqual(g.protected_host("fl"), "flat")  # MQTT --name is a substring filter
        self.assertIsNone(g.protected_host("fugu"))
        self.assertIsNone(g.protected_host("fboost"))
        self.assertIsNone(g.protected_host(None))

    def test_take_lock(self):
        g = guard(take=True, note="test")
        with mock.patch.object(g.locks, "take", return_value=True) as take:
            self.assertTrue(g.allowed("dc 0", "fry"))
        take.assert_called_once_with("fry", "test")

    def test_running_ota_blocks_other_commands(self):
        # 2026-11-26: a sweep nearly went to flat mid-OTA; the guard checked only the device lock.
        g = guard(FakeRun(mine={"flat"}, held={"ota-flat"}))
        self.assertFalse(g.allowed("sweep", "flat"))
        self.assertTrue(g.allowed("status", "flat"))
        self.assertIn("OTA by other", g.ota_busy("flat"))
        self.assertFalse(guard(FakeRun(mine={"flat"}, held={"ota-flat"})).allowed("ota http://h/x.bin", "flat"))

    def test_flagged_held_ota_lock_still_blocks(self):
        run = FakeRun(mine={"flat"}, held={"ota-flat"})
        orig = run.__call__
        def flagged(argv, **kw):
            r = orig(argv, **kw)
            if argv[1] == "status" and argv[2] == "ota-flat":
                r.stdout = "FLAGGED ota-flat: bench note\n" + r.stdout
            return r
        self.assertFalse(guard(flagged).allowed("sweep", "flat"))

    def test_own_ota_lock_does_not_block(self):
        g = guard(FakeRun(mine={"flat", "ota-flat"}, held={"ota-flat"}))
        self.assertTrue(g.allowed("sweep", "flat"))
        self.assertTrue(g.allowed("ota http://h/f.bin", "flat"))

    def test_take_lock_busy_refuses(self):
        g = guard(take=True)
        with mock.patch.object(g.locks, "take", return_value=False):
            self.assertFalse(g.allowed("dc 0", "fry"))


class Record(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(tempfile.mkdtemp(), "elf-archive", "console-commands.jsonl")

    def rows(self):
        with open(self.path) as f:
            return [json.loads(ln) for ln in f]

    def test_masks_secret_and_skips_reads(self):
        g = guard(FakeRun(mine={"fry"}), log_path=self.path, transport="mqtt")
        g.record("status", "fry")
        g.record("restart", "fugu-esp32s3-4028")
        g.record("set-config wifi.conf psk hunter2", "fry")
        (r,) = self.rows()
        self.assertEqual(r["device"], "fry")
        self.assertEqual(r["command"], "set-config wifi.conf psk ***")
        self.assertEqual((r["transport"], r["who"], r["lock_held"], r["guard"]),
                         ("mqtt", "test (00000000)", True, True))

    def test_unlocked_send_recorded(self):
        g = guard(log_path=self.path, enabled=False)
        g.record("dc 0", "flat")
        self.assertEqual(self.rows()[0]["lock_held"], False)


class FakeTransport(Transport):
    """Answers `hn` with a hostname and everything else with OK; records what was written."""

    def __init__(self, hostname="fry"):
        self.hostname, self.rx, self.sent = hostname, queue.Queue(), []

    def open(self):
        pass

    def is_alive(self):
        return True

    def write(self, data):
        for cmd in filter(None, (c.strip() for c in data.decode().splitlines())):
            self.sent.append(cmd)
            if cmd == "hn":
                self.rx.put(f"Hostname: {self.hostname}\n".encode())
            self.rx.put(f"OK: {cmd}\n".encode())

    def read(self):
        try:
            return self.rx.get(timeout=0.05)
        except queue.Empty:
            return b""

    def close(self):
        pass


class Main(unittest.TestCase):
    """main() end to end with a fake transport and a fake lock.sh: what reaches the device."""

    def run_main(self, argv, hostname="fry", mine=()):
        t = FakeTransport(hostname)
        made = []
        tmp = tempfile.mkdtemp()
        lock = os.path.join(tmp, "lock.sh")
        with open(lock, "w") as f:
            f.write("#!/bin/sh\n"
                    f"echo \"$@\" >> {tmp}/calls\n"
                    "case \"$1\" in\n"
                    f"  mine) case \" {' '.join(mine)} \" in *\" $2 \"*) exit 0;; esac; exit 1;;\n"
                    "  status) case \"$2\" in ota-*) echo \"FREE $2\";; *) echo \"HELD $2 by pid 4242, session other-1234\";; esac;;\n"
                    "  newsession) echo console-abc;;\n"
                    "esac\nexit 0\n")
        os.chmod(lock, 0o755)
        log = os.path.join(tmp, "log.jsonl")

        def mk(args):
            made.append(args)
            return t
        env = dict(FUGU_PROTECTED="fry,flat", FUGU_CONSOLE_LOG=log, OTA_OWNER="test (00000000)",
                   MQTT_HOST="broker.invalid")
        with mock.patch.dict(os.environ, env), mock.patch.object(F, "LOCK_SH", lock), \
                mock.patch.object(F, "make_transport", mk), mock.patch.object(sys, "argv", ["fc"] + argv), \
                mock.patch.object(F.atexit, "register"):
            rc = F.main()
        calls, rows = [], []
        if os.path.exists(f"{tmp}/calls"):
            with open(f"{tmp}/calls") as f:
                calls = f.read().splitlines()
        if os.path.exists(log):
            with open(log) as f:
                rows = [json.loads(ln) for ln in f]
        return rc, made, t.sent, calls, rows

    def test_read_only_unchanged(self):
        rc, made, sent, calls, rows = self.run_main(["--mqtt", "--name", "fry", "-c", "hostname", "-c", "status"])
        self.assertEqual((rc, sent, calls, rows), (0, ["hostname", "status"], [], []))

    def test_refused_before_connecting(self):
        rc, made, sent, calls, rows = self.run_main(["--mqtt", "--name", "fry", "-c", "status", "-c", "dc 0"])
        self.assertEqual((rc, made, sent, rows), (1, [], [], []))
        self.assertIn("mine fry --quiet", calls)

    def test_refused_after_hostname_without_sending(self):
        # telnet gives no name hint: the guard asks `hn` (read-only), then refuses
        rc, made, sent, calls, rows = self.run_main(["--ip", "10.0.0.9", "-c", "status", "-c", "restart"],
                                                    hostname="flat")
        self.assertEqual((rc, sent, rows), (1, ["hn"], []))

    def test_allowed_with_lock_and_recorded(self):
        rc, made, sent, calls, rows = self.run_main(["--ip", "10.0.0.9", "-c", "mppt"], mine=("fry",))
        self.assertEqual((rc, sent), (0, ["hn", "mppt"]))
        self.assertEqual([(r["device"], r["command"], r["transport"], r["lock_held"]) for r in rows],
                         [("fry", "mppt", "telnet", True)])

    def test_take_lock_acquires_and_releases(self):
        rc, made, sent, calls, rows = self.run_main(
            ["--mqtt", "--name", "fry", "--take-lock", "--lock-note", "why", "-c", "mppt"])
        self.assertEqual((rc, sent), (0, ["hn", "mppt"]))
        self.assertIn("acquire fry why --session console-abc", calls)
        self.assertEqual(calls[-1], "release fry --session console-abc")
        self.assertTrue(rows[0]["lock_held"])

    def test_bench_device_unguarded(self):
        rc, made, sent, calls, rows = self.run_main(["--ip", "10.0.0.9", "-c", "restart"],
                                                    hostname="fugu-esp32s3-4028")
        self.assertEqual((rc, sent, calls, rows), (0, ["hn", "restart"], [], []))

    def test_no_lock_guard_sends_and_records(self):
        rc, made, sent, calls, rows = self.run_main(["--mqtt", "--name", "flat", "--no-lock-guard", "-c", "dc 0"],
                                                    hostname="flat")
        self.assertEqual((rc, sent), (0, ["hn", "dc 0"]))
        self.assertEqual((rows[0]["lock_held"], rows[0]["guard"]), (False, False))


if __name__ == "__main__":
    unittest.main()
