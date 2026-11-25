#!/usr/bin/env python3
"""Host tests for etc/ota_mqtt.py and fugu_console's app-OTA handling; no device.
Run: .venv/bin/python3 etc/test_ota_mqtt.py"""
import os
import queue
import re
import sys
import tempfile
import threading
import time
import types
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.modules.setdefault("serial", types.ModuleType("serial"))

import ota_mqtt as M  # noqa: E402
import fugu_console as F  # noqa: E402
from etc.fugu.console import Console  # noqa: E402
from etc.fugu.transport import Transport  # noqa: E402

BOOTINFO = """\
reset reason: SW
running: ota_1 @0x1f0000 (1920KB)  rollback=VALID
heap: free=100 min-ever=50 largest=40
uptime: 7 s, app fugu-firmware v1.0 fry-brk1-120-geecc43a (built Nov 12 2026 10:00:00, IDF v6.0.3)
bootloader: idf v6.0.3 v1, Nov 12 2026, len 1, sha 00
""".splitlines()


class Bootinfo(unittest.TestCase):
    def test_parse(self):
        b = M.parse_bootinfo(BOOTINFO)
        self.assertEqual(b, dict(uptime=7, project='fugu-firmware', version='fry-brk1-120-geecc43a',
                                 built='Nov 12 2026 10:00:00', idf='v6.0.3',
                                 partition='ota_1', rollback='VALID'))
        self.assertIsNone(M.parse_bootinfo(['reset reason: SW', 'OK: bootinfo']))

    def test_judge(self):
        before = dict(uptime=5000, version='old', partition='ota_0')
        new = 'new'
        cases = [  # after, elapsed, verdict
            (dict(uptime=20, version='new', partition='ota_1'), 60, 'ok'),
            (dict(uptime=20, version='new'), 60, 'ok'),                       # slot unknown
            (dict(uptime=20, version='old', partition='ota_0'), 60, 'old-image'),
            (dict(uptime=20, version='new', partition='ota_0'), 60, 'old-image'),  # same slot
            (dict(uptime=5060, version='old', partition='ota_0'), 60, 'no-reboot'),
            (dict(uptime=5060, version='new', partition='ota_1'), 60, 'no-reboot'),
            (dict(uptime=70, version='new', partition='ota_1'), 60, 'no-reboot'),  # booted before send
        ]
        for after, elapsed, want in cases:
            self.assertEqual(M.judge(before, after, new, elapsed)[0], want, after)

    def test_forced_same_version_needs_slot_change(self):
        before = dict(uptime=900, version='v', partition='ota_0')
        self.assertEqual(M.judge(before, dict(uptime=3, version='v', partition='ota_1'), 'v', 50)[0], 'ok')
        self.assertEqual(M.judge(before, dict(uptime=3, version='v', partition='ota_0'), 'v', 50)[0],
                         'old-image')


class Guards(unittest.TestCase):
    def test_unconfirmed_running_image_refused_non_interactively(self):
        a = NS(bin='/nonexistent/fugu-firmware.bin', no_driver_check=True, dry_run=False, yes=False)
        img = dict(version='v1')
        stdin = sys.stdin
        sys.stdin = open(os.devnull)
        try:
            self.assertFalse(M.guards(a, None, img, dict(rollback='PENDING_VERIFY')))
            self.assertTrue(M.guards(a, None, img, dict(rollback='VALID')))
            a.dry_run = True
            self.assertTrue(M.guards(a, None, img, dict(rollback='PENDING_VERIFY')))
            self.assertTrue(M.guards(a, None, dict(version='v1-dirty'), {}))
        finally:
            sys.stdin.close()
            sys.stdin = stdin


class FakeConfCon:
    """get-config replies from a {file: text} map; a file mapped to None never completes."""
    def __init__(self, files):
        self.files, self.sent = files, []

    def command(self, cmd, timeout=None):
        self.sent.append(cmd)
        f = cmd.split()[1]
        text = self.files.get(f, f"W (1) conf: cannot read ConfFile /littlefs/conf/{f}")
        return Dropped() if text is None else Ok(text.splitlines())


class Ok(list):
    ok = True


class Dropped(list):
    ok = False


class ConfKeys(unittest.TestCase):
    def run_check(self, files, image=b"adc_fake_freq was renamed to adc_freq\0", yes=False):
        with tempfile.NamedTemporaryFile(suffix=".bin") as f:
            f.write(image)
            f.flush()
            a = NS(bin=f.name, dry_run=False, yes=yes, would_refuse=False)
            con = FakeConfCon(files)
            stdin = sys.stdin
            sys.stdin = open(os.devnull)
            try:
                return M.conf_keys_check(a, con, "dev"), a
            finally:
                sys.stdin.close()
                sys.stdin = stdin

    def test_leftover_key_refused_even_with_yes(self):
        ok, a = self.run_check({"board.conf": "I (1) main: Conf '/littlefs/conf/board.conf:adc_fake_freq' = '1'"},
                               yes=True)
        self.assertFalse(ok)
        self.assertTrue(a.would_refuse)

    def test_clean_and_missing_files_pass(self):
        ok, a = self.run_check({"board.conf": "I (1) main: Conf '/littlefs/conf/board.conf:adc_freq' = '1'"})
        self.assertTrue(ok)
        self.assertFalse(a.would_refuse)

    def test_unread_file_is_unverified_not_clean(self):
        ok, a = self.run_check({"sensor.conf": None})
        self.assertFalse(ok)
        ok, a = self.run_check({"sensor.conf": None}, yes=True)
        self.assertTrue(ok)


def havan_log(path):
    """(t, line)s of a pv/fugu_console.log excerpt ('YYYY-MM-DD HH:MM:SS,mmm  <dev>: <line>')."""
    out = []
    for raw in open(path, encoding='utf-8'):
        if m := re.match(r'\S+ (\d+):(\d+):(\d+),(\d+)\s+[\w.-]+: (.*)$', raw.rstrip('\n')):
            out.append((int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3]) + int(m[4]) / 1000, m[5]))
    return out


class BootWatch(unittest.TestCase):
    """Post-OTA fault watch, calibrated on fry's real OTA boot 2026-11-23 10:46 (a clean boot)."""
    LOG = havan_log(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testdata',
                                 'ota_boot_fry_20261123.txt'))

    def test_fixture_has_the_lines_that_were_flagged(self):
        text = [l for _, l in self.LOG]
        for want in ('Rebooting', 'I (9992) mppt: backoff 30s [sweep-no-mpp]'):
            self.assertIn(want, text)
        self.assertTrue(any(l.startswith('ina22x: i2c_hs_freq') for l in text))
        self.assertTrue(any('START blocked: backoff' in l for l in text))

    def test_clean_boot_has_no_faults(self):
        self.assertEqual(M.boot_faults(self.LOG), [])
        self.assertTrue(M.new_boot(self.LOG)[0][1].startswith('ina22x: i2c_hs_freq'))

    def test_faults_after_the_boot(self):
        t = self.LOG[-1][0]
        bad = ['E (30000) ina22x: 3 timeout!', 'W (31000) mppt: backoff 5s [Vout-OV]',
               'E (32000) main: ADC stall 250 ms, shutdown (nSamples=1)', 'reset reason: TASK_WDT',
               'W (33000) mppt: PSU escalated backoff 60s [supply-UV] (trip 3)',
               'E (34000) main: Loop latency high (10<50 Hz), shutdown! (nSamples=1 D=0 rt=1.0s)',
               "Guru Meditation Error: Core  1 panic'ed (LoadProhibited)", 'Rebooting']
        ok = ['I (35000) mppt: backoff 30s [sweep-no-mpp]', 'reset reason: SW',
              'I (36000) mppt: START blocked: backoff (Vin=1 Vout=2 ntc=3 mcu=4)',
              'netguard: mqtt down 0 s, re-inits 0, guard reboots 0 since power-on']
        lines = self.LOG + [(t + 1 + k * 0.1, l) for k, l in enumerate(bad + ok)]
        self.assertEqual(M.boot_faults(lines), bad)

    def test_pre_boot_lines_ignored_and_fallback(self):
        k = next(i for i, (_, l) in enumerate(self.LOG) if 'OTA Succeed' in l)
        t = self.LOG[k][0]
        lines = self.LOG[:k + 1] + [(t, 'W (2116950) mppt: backoff 5s [Vout-OV]')] + self.LOG[k + 1:]
        self.assertEqual(M.boot_faults(lines), [])
        no_succeed = [x for x in self.LOG if 'OTA Succeed' not in x[1]]
        self.assertIsNone(M.new_boot(no_succeed))
        boot_t = next(x[0] for x in self.LOG if x[1].startswith('ina22x')) - 1
        self.assertEqual(M.boot_faults(no_succeed, boot_t), [])
        self.assertEqual(M.boot_faults(no_succeed, None), ['no new boot found in the log (unverified)'])

    def test_summary_names_devices_not_run(self):
        self.assertEqual(M.summary(['fry', 'flat'], {'fry': 1}), '\nsummary: fry FAILED, flat not run')
        self.assertEqual(M.summary(['fry', 'flat'], {'fry': 0, 'flat': 0}), '\nsummary: fry ok, flat ok')


class OtaLines(unittest.TestCase):
    cmd = 'ota http://192.168.1.200:9000/fugu-firmware.bin'

    def test_is_app_ota(self):
        self.assertTrue(F.is_app_ota(self.cmd))
        self.assertFalse(F.is_app_ota('ota bootloader http://x/b.bin'))
        self.assertFalse(F.is_app_ota('ota'))
        self.assertFalse(F.is_app_ota('otab begin 1 2'))

    def test_classify(self):
        c = lambda l: F.classify_ota_line(l, self.cmd)
        self.assertEqual(c('I (123) ota: OTA Succeed, Rebooting...'), 'succeeded')
        self.assertEqual(c('E (123) ota: Firmware upgrade failed'), 'failed')
        self.assertEqual(c('W (123) main: ota: update failed'), 'failed')
        self.assertEqual(c('W (1) main: ota: RT shutdown transition timed out'), 'failed')
        self.assertEqual(c('E (9) ota: Ota timeout after 300.0 seconds'), 'failed')
        self.assertEqual(c('ERR: ' + self.cmd), 'failed')
        for benign in ('I (1) ota: Download Progress:   42 %', 'I (1) ota: Watchdog task started',
                       'App: fugu-firmware v1.0 old (built x, IDF y)', 'Uptime: 5 s',
                       'I (1) ota: Attempting to download update from http://x'):
            self.assertIsNone(c(benign), benign)

    def test_relevant_filter(self):
        self.assertTrue(M.is_relevant('I (1) ota: Download Progress:  3 %'))
        self.assertFalse(M.is_relevant('Vin 40.1V Vout 27.2V P 120W'))


class FakeTransport(Transport):
    """Replays scripted lines after the first write; optionally goes dead."""

    def __init__(self, script, die=False):
        self.script, self.die, self.rx, self.sent, self.alive = script, die, queue.Queue(), [], True

    def open(self):
        pass

    def is_alive(self):
        return self.alive

    def write(self, data):
        self.sent.append(data.decode())

        def play():
            for delay, line in self.script:
                time.sleep(delay)
                self.rx.put((line + '\n').encode())
            if self.die:
                self.alive = False
        threading.Thread(target=play, daemon=True).start()

    def read(self):
        try:
            return self.rx.get(timeout=0.05)
        except queue.Empty:
            return b''

    def close(self):
        pass


class RunOta(unittest.TestCase):
    cmd = 'ota http://h/f.bin'

    def run_script(self, script, **kw):
        t = FakeTransport(script, die=kw.pop('die', False))
        con = Console(t)
        try:
            return F.run_ota(con, self.cmd, **kw), t
        finally:
            con.close()

    def test_success_returns_at_succeed_line(self):
        t0 = time.monotonic()
        (out, lines, pct), t = self.run_script(
            [(0, 'App: old'), (0, 'I (1) ota: Download Progress:  99 %'),
             (0, 'I (1) ota: Download Progress: 100 %'), (0, 'I (2) ota: OTA Succeed, Rebooting...')],
            timeout=10)
        self.assertEqual((out, pct), ('succeeded', 100))
        self.assertLess(time.monotonic() - t0, 3)
        self.assertEqual(t.sent, [self.cmd + '\r\n'])

    def test_failure(self):
        (out, _, pct), _ = self.run_script(
            [(0, 'I (1) ota: Download Progress:   1 %'), (0, 'E (2) ota: Firmware upgrade failed')],
            timeout=10)
        self.assertEqual((out, pct), ('failed', 1))

    def test_silence_and_timeout_and_drop(self):
        self.assertEqual(self.run_script([(0, 'x')], timeout=10, silence=1)[0][0], 'silent')
        self.assertEqual(self.run_script([], timeout=1)[0][0], 'timeout')
        self.assertEqual(self.run_script([(0, 'x')], timeout=10, die=True)[0][0], 'dropped')


class EnvAndFilter(unittest.TestCase):
    def test_etc_file_prefers_local(self):
        self.assertEqual(F.etc_file('ota_mqtt.py'), os.path.join(os.path.dirname(F.__file__), 'ota_mqtt.py'))

    def test_load_env_shell_wins(self):
        with tempfile.NamedTemporaryFile('w', suffix='.env', delete=False) as f:
            f.write('# c\nZZ_OTA_A=file\nZZ_OTA_B=file\n')
        os.environ['ZZ_OTA_A'] = 'shell'
        try:
            F.load_env_file(f.name)
            self.assertEqual((os.environ['ZZ_OTA_A'], os.environ['ZZ_OTA_B']), ('shell', 'file'))
        finally:
            os.unlink(f.name)
            for k in ('ZZ_OTA_A', 'ZZ_OTA_B'):
                os.environ.pop(k, None)

    def test_exact_hostname_match(self):
        t = M.ExactMqtt('h', device='flat')
        for topic in ('pv/log/flat2', 'pv/log/xflat', 'pv/log/flat/cmd', 'pv/log/flat'):
            t._on_message(None, None, NS(topic=topic, payload=b'x\n'))
        self.assertEqual(t.hostname, 'flat')
        self.assertEqual(t._rx.qsize(), 1)

    def test_same_24(self):
        self.assertTrue(M.same_24('192.168.1.204', '192.168.1.200'))
        self.assertFalse(M.same_24('100.64.0.3', '192.168.1.200'))
        self.assertFalse(M.same_24(None, '192.168.1.200'))


class RunDeviceTest(unittest.TestCase):
    def test_lost_link_is_a_failure_not_a_traceback(self):
        def boom(*a):
            raise ConnectionError("MQTT broker did not acknowledge command 'get-config sensor.conf'")
        orig, M.ota_device = M.ota_device, boom
        try:
            self.assertEqual(2, M.run_device(NS(), 'fry', None, None, None, None))
        finally:
            M.ota_device = orig

    def test_lost_link_message_says_nothing_was_sent(self):
        import io, contextlib
        def boom(*a):
            raise ConnectionError('x')
        orig, M.ota_device = M.ota_device, boom
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                M.run_device(NS(), 'fry', None, None, None, None)
        finally:
            M.ota_device = orig
        self.assertIn('before `ota` was sent', buf.getvalue())

    def test_other_errors_still_raise(self):
        def boom(*a):
            raise ValueError('bug')
        orig, M.ota_device = M.ota_device, boom
        try:
            self.assertRaises(ValueError, M.run_device, NS(), 'fry', None, None, None, None)
        finally:
            M.ota_device = orig


if __name__ == '__main__':
    unittest.main()
