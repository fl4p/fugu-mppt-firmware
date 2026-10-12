"""Pins the `conf-hash` definition (src/conf_hash.h) via its host reference etc/conf_hash.py.

The firmware and the host must agree byte for byte, so the expected digests here are written out
from the definition by hand, not taken from conf_hash.py's own output.
"""
import hashlib
import io
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "etc"))
import conf_hash  # noqa: E402


def _fixture(d: Path):
    # Names chosen so bytewise order differs from naive orders: uppercase sorts before lowercase,
    # "a.conf" before "a_b.conf" ('.'=0x2e < '_'=0x5f), "sub/x.conf" after "sub.conf" ('.' < '/').
    (d / "conf").mkdir()
    c = d / "conf"
    (c / "board.conf").write_bytes(b"mcu=esp32s3\n")
    (c / "a_b.conf").write_bytes(b"k=1\n")
    (c / "a.conf").write_bytes(b"")
    (c / "Z.conf").write_bytes(b"\x00\xff raw\r\n")
    (c / "sub.conf").write_bytes(b"s=1\n")
    (c / "sub").mkdir()
    (c / "sub" / "x.conf").write_bytes(b"x=2\n")
    (c / "wifi.conf").write_bytes(b"ssid=lab\npsk=secret\n")
    return c


EXPECTED_ORDER = ["Z.conf", "a.conf", "a_b.conf", "board.conf", "sub.conf", "sub/x.conf"]


def _by_hand(c: Path):
    h = hashlib.sha256()
    for rel in EXPECTED_ORDER:
        data = (c / rel).read_bytes()
        h.update(rel.encode() + b"\0" + str(len(data)).encode() + b"\0" + data)
    return h.hexdigest()


class ConfHashTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        self.conf = _fixture(self.dir)

    def tearDown(self):
        self._tmp.cleanup()

    def test_order_and_exclusion(self):
        self.assertEqual(conf_hash.list_files(self.conf), EXPECTED_ORDER)

    def test_digest_matches_definition(self):
        sha, files, total = conf_hash.conf_hash(self.conf)
        self.assertEqual(sha, _by_hand(self.conf))
        # Frozen value: a change here is a change of the wire definition shared with the firmware.
        self.assertEqual(sha, "280be720036dbc75cd7420b7c96ea303e097c576eca66ba6f27d439b411417df")
        self.assertEqual(total, sum(len((self.conf / r).read_bytes()) for r in EXPECTED_ORDER))
        self.assertEqual([f[0] for f in files], EXPECTED_ORDER)
        self.assertEqual(dict((r, s) for r, s, _ in files)["board.conf"],
                         hashlib.sha256(b"mcu=esp32s3\n").hexdigest())

    def test_wifi_conf_does_not_affect_hash(self):
        before = conf_hash.conf_hash(self.conf)[0]
        (self.conf / "wifi.conf").write_bytes(b"psk=other\n")
        self.assertEqual(conf_hash.conf_hash(self.conf)[0], before)
        (self.conf / "wifi.conf").unlink()
        self.assertEqual(conf_hash.conf_hash(self.conf)[0], before)

    def test_only_top_level_wifi_conf_is_excluded(self):
        before = conf_hash.conf_hash(self.conf)[0]
        (self.conf / "sub" / "wifi.conf").write_bytes(b"x\n")
        self.assertIn("sub/wifi.conf", conf_hash.list_files(self.conf))
        self.assertNotEqual(conf_hash.conf_hash(self.conf)[0], before)

    def test_content_and_rename_change_hash(self):
        before = conf_hash.conf_hash(self.conf)[0]
        (self.conf / "board.conf").write_bytes(b"mcu=esp32\n")
        changed = conf_hash.conf_hash(self.conf)[0]
        self.assertNotEqual(changed, before)
        (self.conf / "board.conf").rename(self.conf / "board2.conf")
        self.assertNotEqual(conf_hash.conf_hash(self.conf)[0], changed)

    def test_length_prefix_separates_boundaries(self):
        # Moving bytes across a file boundary must change the hash.
        a = self.dir / "a"; b = self.dir / "b"
        a.mkdir(); b.mkdir()
        (a / "1.conf").write_bytes(b"xy"); (a / "2.conf").write_bytes(b"z")
        (b / "1.conf").write_bytes(b"x"); (b / "2.conf").write_bytes(b"yz")
        self.assertNotEqual(conf_hash.conf_hash(a)[0], conf_hash.conf_hash(b)[0])

    def test_empty_set_digest(self):
        e = self.dir / "empty"
        e.mkdir()
        (e / "wifi.conf").write_bytes(b"psk=x\n")
        self.assertEqual(conf_hash.conf_hash(e), (hashlib.sha256(b"").hexdigest(), [], 0))

    def test_resolve_profile_dir_and_cli_lines(self):
        self.assertEqual(conf_hash.resolve_conf_dir(self.dir), self.conf)
        self.assertEqual(conf_hash.resolve_conf_dir(self.conf), self.conf)
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(conf_hash.main([str(self.dir)]), 0)
        lines = out.getvalue().splitlines()
        sha = _by_hand(self.conf)
        self.assertRegex(lines[0], r"^conf sha256=[0-9a-f]{64} files=6 bytes=\d+ excluded=wifi\.conf$")
        self.assertIn(f"sha256={sha} ", lines[0])
        self.assertEqual(lines[1], "conf-file Z.conf sha256=%s bytes=8"
                         % hashlib.sha256(b"\x00\xff raw\r\n").hexdigest()[:16])
        self.assertEqual(len(lines), 1 + len(EXPECTED_ORDER))

    def test_missing_dir_is_an_error_not_an_empty_hash(self):
        err = io.StringIO()
        from contextlib import redirect_stderr
        with redirect_stderr(err), redirect_stdout(io.StringIO()) as out:
            self.assertEqual(conf_hash.main([str(self.dir / "nope")]), 1)
        self.assertEqual(out.getvalue(), "")
        self.assertIn("sha256=unknown", err.getvalue())

    def test_real_profiles_have_no_subdirectories(self):
        # The firmware recurses anyway; this records that today's profiles are flat.
        for conf in ROOT.glob("config/**/conf"):
            self.assertEqual([p for p in conf.iterdir() if p.is_dir()], [], conf)


class NvsHashTest(unittest.TestCase):
    def test_definition(self):
        entries = [("wifi_ssid", "str", "lab"), ("hostname", "str", "dev1"), ("n", "u16", 0x1234)]
        h = hashlib.sha256()
        h.update(b"hostname\0str\x004\0dev1")
        h.update(b"n\0u16\x002\0" + struct.pack("<H", 0x1234))
        h.update(b"wifi_ssid\0str\x003\0lab")
        self.assertEqual(conf_hash.nvs_hash(entries), h.hexdigest())

    def test_empty_string_and_empty_namespace(self):
        self.assertEqual(conf_hash.nvs_hash([]), hashlib.sha256(b"").hexdigest())
        self.assertEqual(conf_hash.nvs_hash([("wsync_arm", "str", "")]),
                         hashlib.sha256(b"wsync_arm\0str\x000\0").hexdigest())


if __name__ == "__main__":
    unittest.main()
