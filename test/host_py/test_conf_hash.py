"""Pins `conf-hash`: etc/conf_hash.py against a hand-built digest, and against the firmware's
src/conf_hash.cpp (host build of test/host-stub/conf-hash-test.cpp) on every config/ profile.

Run: .venv/bin/python3 test/host_py/test_conf_hash.py -v
"""
import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "etc"))
import conf_hash  # noqa: E402

REDACT_VECTORS = [
    (b"ble_passkey=123456\n", b"ble_passkey=<redacted>\n"),
    (b"# ble_passkey=old\n", b"# ble_passkey=<redacted>\n"),
    (b"  #\t# password = x # c\r\n", b"  #\t# password =<redacted>\n"),
    (b"ftp_pass=\n", b"ftp_pass=<redacted>\n"),
    (b"Wifi_PSK=x\n", b"Wifi_PSK=<redacted>\n"),
    (b"cloud_key=k\n", b"cloud_key=<redacted>\n"),
    (b"x=api_KEY\t=v\n", b"x=api_KEY\t=<redacted>\n"),
    (b"note=about passwords=here\n", b"note=about passwords=<redacted>\n"),
    (b"a=" + b"x" * 200 + b"password=hunter2\n", b"a=" + b"x" * 200 + b"password=<redacted>\n"),
    (b"a=" + b"x" * 253 + b"password=hunter2\n", b"<redacted>\n"),
    (b"x" * 255 + b"\n" + b"y" * 256, b"x" * 255 + b"\n<redacted>"),
    (b"ble_security=passkey\n", b"ble_security=passkey\n"),
    (b"no equals password here\n", b"no equals password here\n"),
    (b"username=bob\n", b"username=<redacted>\n"),
    (b"ftp_user=bob\n", b"ftp_user=<redacted>\n"),
    (b"broker_uri=mqtt://u:my_token=abc@host\n", b"broker_uri=mqtt://<redacted>@host\n"),
    (b"broker_uri=mqtt://u:pw@host # token=x\n", b"broker_uri=mqtt://<redacted>@host # token=<redacted>\n"),
    (b"broker_uri=mqtts://u:pw@host/?psk=1\n", b"broker_uri=mqtts://<redacted>@host/?psk=<redacted>\n"),
    (b"broker_uri=mqtt://u:p@host:1883\n", b"broker_uri=mqtt://<redacted>@host:1883\n"),
    (b"broker_uri=mqtt://u:pa/ss@host\n", b"broker_uri=mqtt://<redacted>@host\n"),
    (b"broker_uri=mqtt://host:1883\n", b"broker_uri=mqtt://host:1883\n"),
    (b"u=a://x@h b://y@g\n", b"u=a://<redacted>@g\n"),
    (b"u=x@y a://h\n", b"u=x@y a://h\n"),
    (b"u=a://x@h\nv=b://y@g", b"u=a://<redacted>@h\nv=b://<redacted>@g"),
    (b"x=1\npsk=2", b"x=1\npsk=<redacted>"),
    (b"", b""),
]


def _fixture(d: Path) -> Path:
    c = d / "conf"
    c.mkdir()
    (c / "board.conf").write_bytes(b"mcu=esp32s3\n")
    (c / "a_b.conf").write_bytes(b"k=1\n")
    (c / "a.conf").write_bytes(b"")
    (c / "Z.conf").write_bytes(b"\x00\xff raw\r\n")
    (c / "mqtt.conf").write_bytes(b"broker_uri=mqtt://u:pw@h:1883\npassword=hunter2\n")
    (c / "wifi.conf").write_bytes(b"ssid=lab\npsk=secret\n")
    (c / "board.conf.cft").write_bytes(b"staged\n")
    (c / "notes.txt").write_bytes(b"x\n")
    return c


def _by_hand(c: Path) -> str:
    h = hashlib.sha256()
    for name in ["Z.conf", "a.conf", "a_b.conf", "board.conf"]:
        h.update(name.encode() + b"\0" + (c / name).read_bytes() + b"\0")
    h.update(b"mqtt.conf\0broker_uri=mqtt://<redacted>@h:1883\npassword=<redacted>\n\0")
    return h.hexdigest()


class ConfHashTest(unittest.TestCase):
    def test_redact(self):
        for src, exp in REDACT_VECTORS:
            self.assertEqual(conf_hash.redact(src), exp, src)

    def test_digest(self):
        with tempfile.TemporaryDirectory() as t:
            c = _fixture(Path(t))
            sha, files = conf_hash.conf_hash(c)
            self.assertEqual(sha, _by_hand(c))
            self.assertEqual([n for n, _ in files], [b"Z.conf", b"a.conf", b"a_b.conf", b"board.conf", b"mqtt.conf"])
            self.assertEqual(conf_hash.resolve_conf_dir(t), str(c))


@unittest.skipUnless(subprocess.run(["which", "c++"], capture_output=True).returncode == 0, "no C++ compiler")
class FirmwareCrossCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.exe = str(Path(cls._tmp.name) / "conf-hash-test")
        subprocess.run(["c++", "-std=gnu++17", "-Wall", "-Werror", "-I", str(ROOT / "test/host-stub"),
                        "-I", str(ROOT / "src"), "-o", cls.exe,
                        str(ROOT / "test/host-stub/conf-hash-test.cpp")], check=True)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def _same(self, d):
        cpp = subprocess.run([self.exe, str(d)], capture_output=True, check=True).stdout.decode()
        py = subprocess.run([sys.executable, str(ROOT / "etc/conf_hash.py"), str(d)],
                            capture_output=True, check=True).stdout.decode()
        self.assertEqual(cpp, py, d)
        return cpp

    def test_redact(self):
        for src, exp in REDACT_VECTORS:
            out = subprocess.run([self.exe, "-r"], input=src, capture_output=True, check=True).stdout
            self.assertEqual(out, exp, src)

    def test_fixture(self):
        with tempfile.TemporaryDirectory() as t:
            c = _fixture(Path(t))
            self.assertIn(f"conf sha256={_by_hand(c)} files=5", self._same(c))

    def test_config_profiles(self):
        dirs = sorted(p for p in (ROOT / "config").rglob("conf") if p.is_dir())
        self.assertGreater(len(dirs), 10)
        for d in dirs:
            self._same(d)


if __name__ == "__main__":
    unittest.main()
