import json
import os.path
import re
import subprocess
import sys


def read_firmware_version():
    version = None
    with open('src/version.h', 'r') as fh:
        for l in fh.readlines():
            m = re.match(r'\s*#define\s+FIRMWARE_VERSION\s+"(.+)"', l)
            if m:
                if version:
                    raise Exception('multiple matching lines')
                version = m.group(1)
    return version


fw_ver = read_firmware_version()

envs = ["e2_esp32dev", "e1_esp32s3dev"]

for env in envs:
    bin_path = f'web-flash/{env}.bin'
    if os.path.exists(bin_path):
        print('removing', bin_path)
        os.remove(bin_path)

print('Building FW ver %s...' % fw_ver)

p = subprocess.Popen(["bash", "build-for-webflash.sh"], stdout=sys.stdout, stderr=sys.stderr)
out, _ = p.communicate()
if p.returncode != 0:
    raise Exception('Shell script failed')

for env in envs:
    bin_path = f'web-flash/{env}.bin'
    bin_path_ver = f'web-flash/{env}_v{fw_ver}.bin'
    os.rename(bin_path, bin_path_ver)

manifest_json = {
    "name": "Fugu MPPT",
    "version": fw_ver,
    "home_assistant_domain": "esphome",
    "new_install_prompt_erase": False,
    "builds": [
        {
            "chipFamily": "ESP32",
            "parts": [{"path": f"e2_esp32dev_v{fw_ver}.bin", "offset": 0}]
        },
        {
            "chipFamily": "ESP32-S3",
            "parts": [{"path": f"e1_esp32s3dev_v{fw_ver}.bin", "offset": 0}]
        }
    ]
}

with open('web-flash/manifest.json', 'w') as fh:
    json.dump(manifest_json, fh, indent=4)

print("")
print("")
print('Wrote manifest.json, you can now upload web-flash/*')
