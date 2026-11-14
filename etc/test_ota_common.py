#!/usr/bin/env python3
"""Host tests for etc/ota_common.py (the steps shared by ota.py, ota_mqtt.py, ota_ble.py); no device.
Run: .venv/bin/python3 etc/test_ota_common.py"""
import builtins
import contextlib
import hashlib
import io
import os
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ota_common as C  # noqa: E402


def app_image(version, project='fugu-firmware', at=0x20):
    desc = bytearray(0x100)
    desc[0:4] = struct.pack('<I', C.APP_DESC_MAGIC)
    for off, s in ((0x10, version), (0x30, project), (0x50, '10:00:00'), (0x60, 'Nov 12 2026'),
                   (0x70, 'v6.0.3')):
        desc[off:off + len(s)] = s.encode()
    return bytes(at) + bytes(desc)


def write_tmp(data):
    f = tempfile.NamedTemporaryFile(delete=False, suffix='.bin')
    f.write(data)
    f.close()
    return f.name


class LocalImage(unittest.TestCase):
    def test_app_desc(self):
        p = write_tmp(app_image('fry-brk1-120-geecc43a'))
        try:
            self.assertEqual(C.read_local_app_desc(p), dict(
                version='fry-brk1-120-geecc43a', project='fugu-firmware', time='10:00:00',
                date='Nov 12 2026', idf='v6.0.3'))
        finally:
            os.unlink(p)

    def test_app_desc_missing_or_garbage(self):
        self.assertIsNone(C.read_local_app_desc('/nonexistent/fugu-firmware.bin'))
        p = write_tmp(bytes(0x400))
        try:
            self.assertIsNone(C.read_local_app_desc(p))
        finally:
            os.unlink(p)

    def bootloader(self, corrupt=False, hashed=True):
        d = bytearray(200)
        d[0], d[23], d[32] = 0xE9, 1 if hashed else 0, 0x50
        d[36:40] = struct.pack('<I', 1)
        d[40:46] = b'v6.0.3'
        d[72:83] = b'Nov 12 2026'
        sha = hashlib.sha256(bytes(d)).digest()
        if corrupt:
            d[100] ^= 1
        return write_tmp(bytes(d) + sha), sha[:8].hex()

    def test_bootloader(self):
        p, sha = self.bootloader()
        try:
            self.assertEqual(C.read_local_bootloader(p), dict(
                text=f'idf v6.0.3 v1, Nov 12 2026, len 232, sha {sha}', sha=sha))
        finally:
            os.unlink(p)

    def test_bootloader_rejects_bad_sha_and_unhashed(self):
        for kw in (dict(corrupt=True), dict(hashed=False)):
            p, _ = self.bootloader(**kw)
            try:
                self.assertIsNone(C.read_local_bootloader(p), kw)
            finally:
                os.unlink(p)
        self.assertIsNone(C.read_local_bootloader('/nonexistent/bootloader.bin'))


UPTIME = ['Uptime: 6789 s', 'App: fugu-firmware v1.0 fry-brk1-120-geecc43a (built Nov 12 2026 10:00:00, IDF v6.0.3)']
BOOTINFO = """\
reset reason: SW
running: ota_1 @0x1f0000 (1920KB)  rollback=PENDING_VERIFY
uptime: 7 s, app fugu-firmware v1.0 v2 (built Nov 12 2026 10:00:00, IDF v6.0.3)
bootloader: idf v6.0.3 v1, Nov 12 2026, len 1, sha 00
""".splitlines()


class DeviceReplies(unittest.TestCase):
    def test_uptime_reply(self):
        self.assertEqual(C.parse_uptime(UPTIME), 6789)
        self.assertEqual(C.parse_device_app(UPTIME), dict(
            project='fugu-firmware', version='fry-brk1-120-geecc43a',
            built='Nov 12 2026 10:00:00', idf='v6.0.3'))
        self.assertEqual(C.parse_device_version(UPTIME), 'fry-brk1-120-geecc43a')
        for f in (C.parse_uptime, C.parse_device_app, C.parse_device_version):
            self.assertIsNone(f(['OK: uptime', 'Vin 40 V']), f.__name__)

    def test_bootinfo(self):
        b = C.parse_bootinfo(BOOTINFO)
        self.assertEqual((b['uptime'], b['version'], b['partition'], b['rollback']),
                         (7, 'v2', 'ota_1', 'PENDING_VERIFY'))
        self.assertTrue(C.is_pending_verify(b))
        self.assertFalse(C.is_pending_verify(dict(b, rollback='VALID')))
        self.assertFalse(C.is_pending_verify({}))
        self.assertEqual(C.parse_bootloader_line(BOOTINFO), 'idf v6.0.3 v1, Nov 12 2026, len 1, sha 00')
        self.assertIsNone(C.parse_bootloader_line(['uptime: 7 s']))

    def test_is_current(self):
        self.assertTrue(C.is_current('v2', 'v2'))
        self.assertFalse(C.is_current('v2', 'v1'))
        self.assertFalse(C.is_current('v2', None))
        self.assertFalse(C.is_current(None, None))  # no image version: never "already current"
        self.assertFalse(C.is_current('', ''))

    def test_dirty_and_label(self):
        self.assertTrue(C.image_is_dirty('v1-3-gabc-dirty'))
        self.assertFalse(C.image_is_dirty('v1-3-gabc'))
        self.assertFalse(C.image_is_dirty(None))
        self.assertEqual([C.device_driver_label(d) for d in ('mcpwm', 'other', None)],
                         ['mcpwm', 'ledc/vconv', 'unknown'])


@contextlib.contextmanager
def stdin(tty, answer=None):
    asked = []
    with patch.object(sys, 'stdin', types.SimpleNamespace(isatty=lambda: tty)), \
            patch.object(builtins, 'input', lambda p='': (asked.append(p), answer)[1]), \
            contextlib.redirect_stdout(io.StringIO()) as out:
        yield asked, out


class Confirm(unittest.TestCase):
    def test_refused_without_tty_even_with_yes_answer(self):
        with stdin(False, 'y') as (asked, out):
            self.assertFalse(C.confirm('go? ', 'refused', aborted='aborted'))
        self.assertEqual(asked, [])
        self.assertEqual(out.getvalue(), 'refused\n')

    def test_accept_skips_prompt(self):
        with stdin(False) as (asked, out):
            self.assertTrue(C.confirm('go? ', 'refused', accept='accepted (--yes)'))
        self.assertEqual((asked, out.getvalue()), ([], 'accepted (--yes)\n'))

    def test_false_accept_still_asks(self):
        with stdin(True, 'n') as (asked, out):
            self.assertFalse(C.confirm('go? ', 'refused', accept=False, aborted='aborted'))
        self.assertEqual((asked, out.getvalue()), (['go? '], 'aborted\n'))

    def test_answers(self):
        for answer, want in (('y', True), (' YES ', True), ('yes', True), ('', False), ('n', False),
                             ('yep', False)):
            with stdin(True, answer) as (_, out):
                self.assertEqual(C.confirm('go? ', 'refused'), want, answer)
            self.assertEqual(out.getvalue(), '', answer)  # no `aborted` message given


class Verify(unittest.TestCase):
    def test_judge(self):
        before = dict(uptime=5000, version='old', partition='ota_0')
        for after, elapsed, want in [
            (dict(uptime=20, version='new', partition='ota_1'), 60, 'ok'),
            (dict(uptime=20, version='new'), 60, 'ok'),
            (dict(uptime=20, version='old', partition='ota_1'), 60, 'old-image'),
            (dict(uptime=20, version='new', partition='ota_0'), 60, 'old-image'),
            (dict(uptime=66, version='new', partition='ota_1'), 60, 'no-reboot'),
            (dict(uptime=5060, version='new', partition='ota_1'), 60, 'no-reboot'),
        ]:
            self.assertEqual(C.judge(before, after, 'new', elapsed)[0], want, after)

    def test_verify_versions(self):
        after = dict(a=dict(version='v2'), b=dict(version='v1'), c=None)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            got = C.verify_versions(['a', 'b', 'c', 'd'], after, 'v2')
        self.assertEqual(got, dict(a=True, b=False, c=False, d=False))
        self.assertEqual(out.getvalue().splitlines(), [
            'verify:', '  a: ✅ v2', "  b: ❌ got 'v1', expected 'v2'",
            "  c: ❌ got None, expected 'v2'", "  d: ❌ got None, expected 'v2'"])


class Output(unittest.TestCase):
    def test_fmt_uptime(self):
        self.assertEqual(C.fmt_uptime(6789), '1.89h')
        self.assertEqual(C.fmt_uptime(None), '-')

    def test_bootinfo_row(self):
        self.assertEqual(C.bootinfo_row('after', None),
                         dict(state='after', uptime='-', version='-', slot='-', rollback='-'))
        self.assertEqual(C.bootinfo_row('before', C.parse_bootinfo(BOOTINFO)),
                         dict(state='before', uptime='7 s', version='v2', slot='ota_1',
                              rollback='PENDING_VERIFY'))


class Archive(unittest.TestCase):
    def fake(self, raise_=None):
        calls = []

        def archive(device, **kw):
            calls.append((device, kw))
            if raise_:
                raise raise_
        return calls, types.SimpleNamespace(archive=archive)

    def test_args(self):
        calls, mod = self.fake()
        with patch.object(C, 'require_elf_archive', lambda: mod):
            C.archive_elf('fry', 'ota', 'v2')
            C.archive_elf('flat', 'ota-mqtt', 'v3', '/x/build-a/fugu-firmware.bin')
        self.assertEqual(calls, [
            ('fry', dict(method='ota', version='v2')),
            ('flat', dict(method='ota-mqtt', version='v3', bin='/x/build-a/fugu-firmware.bin',
                          build_dir='/x/build-a'))])

    def test_failure_only_warns(self):
        _, mod = self.fake(OSError('disk full'))
        with patch.object(C, 'require_elf_archive', lambda: mod), \
                contextlib.redirect_stdout(io.StringIO()) as out:
            C.archive_elf('fry', 'ota', 'v2', indent='  ')
        self.assertEqual(out.getvalue(), '  WARN: ELF archive failed for fry: disk full\n')

    def test_missing_module_only_warns(self):
        def missing():
            raise ImportError('No module named elf_archive')
        with patch.object(C, 'require_elf_archive', missing), \
                contextlib.redirect_stdout(io.StringIO()) as out:
            C.archive_elf('fry', 'ota', 'v2')
        self.assertIn('WARN: ELF archive failed for fry', out.getvalue())


if __name__ == '__main__':
    unittest.main()
