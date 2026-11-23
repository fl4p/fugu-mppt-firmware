"""Host-side ports of the firmware secret masking (src/etc/con_guard.h secretCut) agree with it.

Vectors: test/host-stub/con-guard-test.cpp. Ports: etc/fugu/console.py mask_secrets, conf-editor.html maskSecrets."""
import json, re, shutil, subprocess, sys, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from etc.fugu.console import mask_secrets

VECTORS = [
    ("set-config mqtt.conf password hunter2", "set-config mqtt.conf password ***"),
    ("setc tlscon.conf psk 00112233445566778899aabbccddeeff", "setc tlscon.conf psk ***"),
    ("set-config wifi.conf ssid_home_psk my secret pw", "set-config wifi.conf ssid_home_psk ***"),
    ("set-config ftp.conf ftp_pass x", "set-config ftp.conf ftp_pass ***"),
    ("set-config ble.conf ble_passkey 123456", "set-config ble.conf ble_passkey ***"),
    ("SET-CONFIG mqtt.conf PASSWORD x", "SET-CONFIG mqtt.conf PASSWORD ***"),
    ("wifi-add home:hunter2", "wifi-add home:***"),
    ("status;;wifi-add home:pw;;mem", "status;;wifi-add home:***"),
    ("status;;set-config mqtt.conf password pw", "status;;set-config mqtt.conf password ***"),
    ("script-set s set-config mqtt.conf password pw; restart", "script-set s set-config mqtt.conf password ***"),
    ("curl -d token=abc http://x", "curl -d token=***"),
    ("password=hunter2", "password=***"),
    ("# ble_passkey=123456", "# ble_passkey=***"),
    ("ssid_home_psk = pw", "ssid_home_psk =***"),
    ("broker_uri=mqtt://u:pw@host:1883", "broker_uri=mqtt://***"),
    ("set-config mqtt.conf broker_uri mqtts://u:pw@host", "set-config mqtt.conf broker_uri mqtts://***"),
    ("ota http://10.0.0.2:9000/fw.bin", "ota http://10.0.0.2:9000/fw.bin"),
    ("set-config coil.conf L0 50", "set-config coil.conf L0 50"),
    ("get-config mqtt.conf password", "get-config mqtt.conf password"),
    ("del-config tlscon.conf psk", "del-config tlscon.conf psk"),
    ("broker_uri=mqtt://1.2.3.4", "broker_uri=mqtt://1.2.3.4"),
    ("ssid_home=home", "ssid_home=home"),
    ("status", "status"),
    ("wifi-add nocolon", "wifi-add nocolon"),
]


class MaskSecrets(unittest.TestCase):
    def test_python(self):
        for line, want in VECTORS:
            self.assertEqual(mask_secrets(line), want, line)

    @unittest.skipUnless(shutil.which("node"), "node not installed")
    def test_conf_editor(self):
        html = (ROOT / "etc/config-tool/conf-editor.html").read_text()
        a = html.index("const isSecretKey")
        b = html.index("\n}\n", html.index("function maskSecrets")) + 3
        js = html[a:b] + "\nconsole.log(JSON.stringify(%s.map(maskSecrets)));" % json.dumps([v[0] for v in VECTORS])
        got = json.loads(subprocess.check_output(["node", "-e", js], text=True))
        self.assertEqual(got, [v[1] for v in VECTORS])


if __name__ == "__main__":
    unittest.main()
