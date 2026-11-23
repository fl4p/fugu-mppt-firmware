"""Run test/main_bl_ota.cpp under QEMU (esp32s3): refusal cases, then a real bootloader install, then a
second boot from the new bootloader.

  MAIN_SRC=../test/main_bl_ota.cpp idf.py -B build/blqemu -DSDKCONFIG=build/blqemu/sdkconfig build
  (QEMU's flash has no QIO: set CONFIG_ESPTOOLPY_FLASHMODE_DIO=y in build/blqemu/sdkconfig first)
  python3 test/bl_ota_qemu.py build/blqemu

The candidate is the build's own bootloader with its descriptor date replaced (checksum and SHA-256
fixed up), so it boots like the original but is distinguishable in `bootloader:` lines.
"""
import hashlib
import os
import re
import shutil
import subprocess
import sys

BL_SRC = 0x3F8000
MARK = b'BLOTA-TEST-CANDIDATE'


def candidate(bl: bytes) -> bytes:
    img = bytearray(bl[:-32])
    assert img[0] == 0xE9 and img[32] == 0x50, 'no bootloader descriptor'
    date = slice(32 + 40, 32 + 40 + len(MARK))
    delta = 0
    for a, b in zip(img[date], MARK):
        delta ^= a ^ b
    img[date] = MARK
    img[-1] ^= delta  # 8-bit XOR checksum is the last byte before the digest
    return bytes(img) + hashlib.sha256(img).digest()


def run_qemu(flash, efuse, timeout=90):
    qemu = shutil.which('qemu-system-xtensa') or sys.exit('qemu-system-xtensa not on PATH (idf_tools.py install qemu-xtensa)')
    args = [qemu, '-M', 'esp32s3', '-m', '32M', '-nographic', '-no-reboot',
            '-drive', f'file={flash},if=mtd,format=raw',
            '-drive', f'file={efuse},if=none,format=raw,id=efuse',
            '-global', 'driver=nvram.esp32s3.efuse,property=drive,value=efuse',
            '-global', 'driver=timer.esp32s3.timg,property=wdt_disable,value=true']
    p = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = []
    try:
        for line in iter(p.stdout.readline, b''):
            s = line.decode('utf-8', 'replace').rstrip()
            out.append(s)
            print(s)
            if 'BLTEST DONE' in s:
                break
    finally:
        p.kill()
        p.wait()
    return out


def main(build):
    flash = os.path.join(build, 'qemu_flash.bin')
    efuse = os.path.join(build, 'qemu_efuse.bin')
    subprocess.check_call([sys.executable, '-m', 'esptool', '--chip=esp32s3', 'merge-bin', '--output=qemu_flash.bin',
                           '--pad-to-size=4MB', '@flash_args'], cwd=build)
    with open(os.path.join(build, 'bootloader', 'bootloader.bin'), 'rb') as f:
        cand = candidate(f.read())
    with open(flash, 'r+b') as f:
        f.seek(BL_SRC)
        f.write(cand)
    sys.path.insert(0, os.path.join(os.environ['IDF_PATH'], 'tools'))
    from idf_py_actions.qemu_ext import QEMU_TARGETS
    with open(efuse, 'wb') as f:
        f.write(QEMU_TARGETS['esp32s3'].default_efuse)

    ok = True
    for boot in (1, 2):
        out = run_qemu(flash, efuse)
        fails = [l for l in out if 'BLTEST FAIL' in l]
        done = [l for l in out if 'BLTEST DONE fails=0' in l]
        cur = next((l for l in out if 'BLTEST bootloader:' in l), '')
        want_cur = boot == 2
        good = bool(done) and not fails and ((MARK.decode() in cur) == want_cur)
        if boot == 1:
            good = good and any(MARK.decode() in l for l in out if 'bootloader now' in l)
        print(f'== boot {boot}: {"OK" if good else "FAILED"}')
        ok &= good
    return ok


if __name__ == '__main__':
    sys.exit(0 if main(sys.argv[1] if len(sys.argv) > 1 else 'build/blqemu') else 1)
