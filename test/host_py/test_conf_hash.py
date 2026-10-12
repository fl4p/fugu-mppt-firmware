"""Pins the `conf-hash` definition (src/conf_hash.h) via its host reference etc/conf_hash.py.

The firmware and the host must agree byte for byte. Expected digests here are written out from the
definition by hand, not taken from conf_hash.py's own output, and the frozen values (FROZEN_*) are
shared with test/host-stub/conf-hash-test.cpp, which runs the firmware's src/conf_hash.cpp.
"""
import hashlib
import io
import os
import struct
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "etc"))
import conf_hash  # noqa: E402

# Shared with test/host-stub/conf-hash-test.cpp -- change both or neither.
FROZEN_BASIC = "280be720036dbc75cd7420b7c96ea303e097c576eca66ba6f27d439b411417df"


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


EXPECTED_ORDER = [b"Z.conf", b"a.conf", b"a_b.conf", b"board.conf", b"sub.conf", b"sub/x.conf"]


def _by_hand(c: Path):
    h = hashlib.sha256()
    for rel in EXPECTED_ORDER:
        data = (c / rel.decode()).read_bytes()
        h.update(rel + b"\0" + str(len(data)).encode() + b"\0" + data)
    return h.hexdigest()


# Redaction vectors (input line(s) -> expected). Shared with conf-hash-test.cpp.
REDACT_VECTORS = [
    (b"ble_passkey=123456\n", b"ble_passkey=<redacted>\n", 1),
    (b" " * 243 + b"ble_passkey=123456\n", b" " * 243 + b"ble_passkey=<redacted>\n", 1),
    (b"password=" + b"a" * 246 + b"XYZ", b"password=<redacted>", 1),
    (b"# ble_passkey=old\n", b"# ble_passkey=<redacted>\n", 1),
    (b"  #\t# password = x # c\r\n", b"  #\t# password =<redacted>\n", 1),
    (b"ftp_pass=\n", b"ftp_pass=<redacted>\n", 1),
    (b"Wifi_PSK=x\n", b"Wifi_PSK=<redacted>\n", 1),
    (b"cloud_key=k\n", b"cloud_key=<redacted>\n", 1),
    (b"ble_security=passkey\n", b"ble_security=passkey\n", 0),
    (b"username=bob\n", b"username=bob\n", 0),
    (b"broker_uri=mqtt://u:p@host:1883\n", b"broker_uri=mqtt://<redacted>@host:1883\n", 1),
    (b"broker_uri=mqtt://host:1883\n", b"broker_uri=mqtt://host:1883\n", 0),
    (b"u=http://host/a@b?c@d\n", b"u=http://host/a@b?c@d\n", 0),
    (b"u=mqtt://a@b@host/x@y\n", b"u=mqtt://<redacted>@host/x@y\n", 1),
    (b"# old=mqtt://u:p@h#x\n", b"# old=mqtt://<redacted>@h#x\n", 1),
    (b"u=a://x@h b://y@g\n", b"u=a://<redacted>@h b://<redacted>@g\n", 2),
    (b"u=file:///p@q\n", b"u=file:///p@q\n", 0),
    (b"u=mqtt://h#frag@x\n", b"u=mqtt://h#frag@x\n", 0),
    (b"u=a://longuserinfo1234@h b://y@g\n", b"u=a://<redacted>@h b://<redacted>@g\n", 2),
    (b"no equals password here\n", b"no equals password here\n", 0),
    (b"x=1\npsk=2", b"x=1\npsk=<redacted>", 1),
]

SECRET_DIR_FILES = {
    "ble.conf": b"# ble_passkey=111111\nble_security=passkey\nble_passkey = 123456 \r\n",
    "mqtt.conf": b"broker_uri=mqtt://user:pw@10.0.0.1:1883\nusername=u\npassword=hunter2 # c\n",
    "ftp.conf": b"enabled=1\nftp_pass=\n",
}
SECRET_DIR_EXPECTED = {
    "ble.conf": b"# ble_passkey=<redacted>\nble_security=passkey\nble_passkey =<redacted>\n",
    "mqtt.conf": b"broker_uri=mqtt://<redacted>@10.0.0.1:1883\nusername=u\npassword=<redacted>\n",
    "ftp.conf": b"enabled=1\nftp_pass=<redacted>\n",
}

NVS_VECTOR = [("hostname", "str", "dev1"), ("wifi_psk", "str", "pw"), ("n", "u16", 0x1234),
              ("uri", "str", "mqtt://a:b@h/x"), ("ftp_pass", "str", "x")]


def _secret_by_hand():
    h = hashlib.sha256()
    for name in sorted(SECRET_DIR_EXPECTED):
        data = SECRET_DIR_EXPECTED[name]
        h.update(name.encode() + b"\0" + str(len(data)).encode() + b"\0" + data)
    return h.hexdigest()


def _nvs_by_hand():
    h = hashlib.sha256()
    h.update(b"ftp_pass\0redacted")
    h.update(b"hostname\0str\x004\0dev1")
    h.update(b"n\0u16\x002\0" + struct.pack("<H", 0x1234))
    h.update(b"uri\0str\x0021\0mqtt://<redacted>@h/x")
    h.update(b"wifi_psk\0redacted")
    return h.hexdigest()


FROZEN_SECRET = "4af599a6bf072297df11aea5ab7b65e578982e70c75c607db3cb408318ef0c75"
FROZEN_NVS = "20ea769cd207a86263d5bc5cbc4fbb1be9890794b82765ece790fb039b95d9e5"


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
        sha, files, total, redacted = conf_hash.conf_hash(self.conf)
        self.assertEqual(redacted, 0)
        self.assertEqual(sha, _by_hand(self.conf))
        self.assertEqual(sha, FROZEN_BASIC)
        self.assertEqual([f[0] for f in files], EXPECTED_ORDER)
        self.assertEqual(dict((r, s) for r, s, _, _ in files)[b"board.conf"],
                         hashlib.sha256(b"mcu=esp32s3\n").hexdigest())

    def test_wifi_conf_does_not_affect_hash(self):
        before = conf_hash.conf_hash(self.conf)[0]
        (self.conf / "wifi.conf").write_bytes(b"psk=other\n")
        self.assertEqual(conf_hash.conf_hash(self.conf)[0], before)
        (self.conf / "wifi.conf").unlink()
        self.assertEqual(conf_hash.conf_hash(self.conf)[0], before)

    def test_only_top_level_wifi_conf_is_excluded(self):
        (self.conf / "sub" / "wifi.conf").write_bytes(b"x\n")
        self.assertIn(b"sub/wifi.conf", conf_hash.list_files(self.conf))

    def test_content_and_rename_change_hash(self):
        before = conf_hash.conf_hash(self.conf)[0]
        (self.conf / "board.conf").write_bytes(b"mcu=esp32\n")
        changed = conf_hash.conf_hash(self.conf)[0]
        self.assertNotEqual(changed, before)
        (self.conf / "board.conf").rename(self.conf / "board2.conf")
        self.assertNotEqual(conf_hash.conf_hash(self.conf)[0], changed)

    def test_length_prefix_separates_boundaries(self):
        a = self.dir / "a"; b = self.dir / "b"
        a.mkdir(); b.mkdir()
        (a / "1.conf").write_bytes(b"xy"); (a / "2.conf").write_bytes(b"z")
        (b / "1.conf").write_bytes(b"x"); (b / "2.conf").write_bytes(b"yz")
        self.assertNotEqual(conf_hash.conf_hash(a)[0], conf_hash.conf_hash(b)[0])

    def test_empty_set_digest(self):
        e = self.dir / "empty"
        e.mkdir()
        (e / "wifi.conf").write_bytes(b"psk=x\n")
        self.assertEqual(conf_hash.conf_hash(e), (hashlib.sha256(b"").hexdigest(), [], 0, 0))

    def test_cli_lines(self):
        out = io.StringIO()
        with redirect_stdout(out), redirect_stderr(io.StringIO()):
            self.assertEqual(conf_hash.main([str(self.dir)]), 0)
        lines = out.getvalue().splitlines()
        self.assertEqual(lines[0], f"conf sha256={FROZEN_BASIC} files=6 bytes=32 redacted=0 excluded=wifi.conf")
        self.assertEqual(lines[1], "conf-file Z.conf sha256=%s bytes=8 redacted=0"
                         % hashlib.sha256(b"\x00\xff raw\r\n").hexdigest()[:16])
        self.assertEqual(len(lines), 1 + len(EXPECTED_ORDER))

    def test_errors_print_unknown_and_exit_1(self):
        cases = []
        cases.append(self.dir / "nope")                                  # missing
        (self.conf / "board.conf.new").write_bytes(b"x=1\n")             # staging
        cases.append(self.conf)
        for path in cases:
            out = io.StringIO()
            with redirect_stdout(out), redirect_stderr(io.StringIO()):
                self.assertEqual(conf_hash.main([str(path)]), 1)
            self.assertRegex(out.getvalue(), r"^conf sha256=unknown excluded=wifi\.conf error=.+\n$")
        self.assertIn("staging file present: board.conf.new", out.getvalue())

    def test_symlink_is_an_error_not_followed(self):
        os.symlink(self.conf, self.conf / "sub" / "loop")
        with self.assertRaisesRegex(conf_hash.ConfHashError, "not a regular file"):
            conf_hash.conf_hash(self.conf)

    def test_max_depth(self):
        d = self.dir / "deep"
        p = d
        for i in range(8):
            p = p / f"d{i}"
        p.mkdir(parents=True)
        (p / "x.conf").write_bytes(b"x=1\n")
        self.assertEqual(conf_hash.list_files(d), [b"d0/d1/d2/d3/d4/d5/d6/d7/x.conf"])
        (p / "d8").mkdir()
        with self.assertRaisesRegex(conf_hash.ConfHashError, "too deep"):
            conf_hash.list_files(d)

    @unittest.skipUnless(sys.platform != "darwin", "APFS rejects non-UTF-8 file names")
    def test_non_utf8_filename(self):
        (Path(os.fsdecode(os.fsencode(str(self.conf)) + b"/\xff.conf"))).write_bytes(b"a=1\n")
        self.assertIn(b"\xff.conf", conf_hash.list_files(self.conf))

    def test_non_utf8_filename_bytes_path_logic(self):
        # Platform-independent part: relpaths are bytes end to end and sort bytewise.
        self.assertTrue(all(isinstance(p, bytes) for p in conf_hash.list_files(self.conf)))
        self.assertEqual(conf_hash.format_lines("0" * 64, [(b"\xff.conf", "ab" * 32, 1, 0)], 1, 0)[1],
                         "conf-file \\xff.conf sha256=abababababababab bytes=1 redacted=0")

    def test_resolve_conf_dir(self):
        self.assertEqual(conf_hash.resolve_conf_dir(self.dir), self.conf)   # profile root: descend
        self.assertEqual(conf_hash.resolve_conf_dir(self.conf), self.conf)
        mixed = self.dir / "mixed"
        (mixed / "conf").mkdir(parents=True)
        (mixed / "top.conf").write_bytes(b"a=1\n")
        self.assertEqual(conf_hash.resolve_conf_dir(mixed), mixed)          # has *.conf: hash as is
        err = io.StringIO()
        with redirect_stdout(io.StringIO()), redirect_stderr(err):
            conf_hash.main([str(self.dir)])
        self.assertEqual(err.getvalue(), f"# hashing {self.conf}\n")

    def test_real_profiles_have_no_subdirectories(self):
        for conf in ROOT.glob("config/**/conf"):
            self.assertEqual([p for p in conf.iterdir() if p.is_dir()], [], conf)


class RedactionTest(unittest.TestCase):
    def test_vectors(self):
        for src, exp, n in REDACT_VECTORS:
            self.assertEqual(conf_hash.redact(src), (exp, n), src)

    def test_secret_keys(self):
        for k in ["ble_passkey", "password", "ftp_pass", "wifi_psk", "API_TOKEN", "my_secret", "cloud_key",
                  "Password", "api_Key"]:
            self.assertTrue(conf_hash.is_secret_key(k), k)
        for k in ["ftp_user", "username", "ble_security", "hostname", "broker_uri", "keys", "key_x",
                  "vout_max", "api_\u212aey"]:  # U+212A KELVIN SIGN lowercases to 'k' only outside ASCII
            self.assertFalse(conf_hash.is_secret_key(k), k)

    def test_long_line_is_an_error(self):
        conf_hash.redact(b"a" * 4096 + b"\n")
        with self.assertRaises(conf_hash.ConfHashError):
            conf_hash.redact(b"a" * 4097 + b"\n")

    def _dir(self, root, files):
        root.mkdir()
        for k, v in files.items():
            (root / k).write_bytes(v)
        return root

    def test_secret_dir_frozen_and_secret_values_do_not_matter(self):
        with tempfile.TemporaryDirectory() as t:
            a = self._dir(Path(t) / "a", SECRET_DIR_FILES)
            other = {k: v.replace(b"123456", b"999999").replace(b"hunter2", b"x").replace(b"user:pw", b"q")
                         .replace(b"ftp_pass=\n", b"ftp_pass=s3cret\n").replace(b"111111", b"222222")
                     for k, v in SECRET_DIR_FILES.items()}
            b = self._dir(Path(t) / "b", other)
            ha, hb = conf_hash.conf_hash(a), conf_hash.conf_hash(b)
            self.assertEqual(ha, hb)
            self.assertEqual(ha[0], _secret_by_hand())
            self.assertEqual(ha[0], FROZEN_SECRET)
            self.assertEqual(ha[3], 5)
            self.assertEqual(ha[2], sum(len(v) for v in SECRET_DIR_EXPECTED.values()))
            c = self._dir(Path(t) / "c", {**SECRET_DIR_FILES, "ftp.conf": b"enabled=0\nftp_pass=\n"})
            self.assertNotEqual(conf_hash.conf_hash(c)[0], ha[0])  # a non-secret change still counts


class NvsHashTest(unittest.TestCase):
    def test_vector(self):
        digest, n = conf_hash.nvs_hash_counted(NVS_VECTOR)
        self.assertEqual(digest, _nvs_by_hand())
        self.assertEqual(digest, FROZEN_NVS)
        self.assertEqual(n, 3)

    def test_secret_values_do_not_matter(self):
        other = [(k, t, ("zz" if k in ("wifi_psk", "ftp_pass") else v)) for k, t, v in NVS_VECTOR]
        other = [(k, t, "mqtt://other@h/x" if k == "uri" else v) for k, t, v in other]
        self.assertEqual(conf_hash.nvs_hash(other), FROZEN_NVS)
        self.assertNotEqual(conf_hash.nvs_hash([("hostname", "str", "dev2")] + NVS_VECTOR[1:]), FROZEN_NVS)

    def test_ascii_lowercasing(self):
        self.assertEqual(conf_hash.nvs_hash([("api_Key", "str", "a")]),
                         conf_hash.nvs_hash([("api_Key", "str", "b")]))

    def test_empty(self):
        self.assertEqual(conf_hash.nvs_hash([]), hashlib.sha256(b"").hexdigest())


@unittest.skipUnless(subprocess.run(["which", "c++"], capture_output=True).returncode == 0, "no C++ compiler")
class FirmwareCodeTest(unittest.TestCase):
    """Builds and runs test/host-stub/conf-hash-test.cpp (the firmware's src/conf_hash.cpp)."""

    def test_host_cpp(self):
        with tempfile.TemporaryDirectory() as t:
            exe = Path(t) / "conf-hash-test"
            subprocess.run(["c++", "-std=gnu++17", "-Wall", "-I", str(ROOT / "test/host-stub"),
                            "-I", str(ROOT / "src"), "-o", str(exe),
                            str(ROOT / "test/host-stub/conf-hash-test.cpp")], check=True)
            r = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)


if __name__ == "__main__":
    unittest.main()
