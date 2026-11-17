"""Build the Unity suite (RUN_TESTS=1) and run it in QEMU (esp32s3), no board needed.

  . ./idf-export.sh
  python3 test/unity_qemu.py [-B build-qemu] [--no-build] [--timeout 1200] [--log FILE]

QEMU's flash has no QIO, so the build dir's sdkconfig is switched to DIO (sdkconfig.defaults untouched).
Tests that need real pins or peripherals detect QEMU at runtime (test/qemu.h) and report as Ignored.
Exit 0 only when the Unity summary shows 0 failures; 1 on failures, 2 on abort/reset/timeout.
"""
import argparse
import os
import re
import select
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUMMARY = re.compile(r'^(\d+) Tests (\d+) Failures (\d+) Ignored')
CRASH = ('abort() was called', 'Guru Meditation', 'Rebooting...')


def set_dio(sdkconfig):
    with open(sdkconfig) as f:
        s = f.read()
    t = re.sub(r'^CONFIG_ESPTOOLPY_FLASHMODE_(QIO|QOUT|DOUT)=y$', r'# CONFIG_ESPTOOLPY_FLASHMODE_\1 is not set', s, flags=re.M)
    t = re.sub(r'^# CONFIG_ESPTOOLPY_FLASHMODE_DIO is not set$', 'CONFIG_ESPTOOLPY_FLASHMODE_DIO=y', t, flags=re.M)
    if t != s:
        with open(sdkconfig, 'w') as f:
            f.write(t)


def build(bdir):
    env = dict(os.environ, RUN_TESTS='1')
    idf = ['idf.py', '-B', bdir, f'-DSDKCONFIG={bdir}/sdkconfig']
    if not os.path.exists(os.path.join(bdir, 'sdkconfig')):
        subprocess.check_call(idf + ['reconfigure'], cwd=ROOT, env=env)
    set_dio(os.path.join(bdir, 'sdkconfig'))
    subprocess.check_call(idf + ['build'], cwd=ROOT, env=env)


def images(bdir):
    flash = os.path.join(bdir, 'qemu_flash.bin')
    efuse = os.path.join(bdir, 'qemu_efuse.bin')
    subprocess.check_call([sys.executable, '-m', 'esptool', '--chip=esp32s3', 'merge-bin', '--output=qemu_flash.bin',
                           '--pad-to-size=4MB', '@flash_args'], cwd=bdir)
    sys.path.insert(0, os.path.join(os.environ['IDF_PATH'], 'tools'))
    from idf_py_actions.qemu_ext import QEMU_TARGETS
    with open(efuse, 'wb') as f:
        f.write(QEMU_TARGETS['esp32s3'].default_efuse)  # blank factory MAC: test/qemu.h keys on it
    return flash, efuse


def run(flash, efuse, timeout, log):
    qemu = shutil.which('qemu-system-xtensa') or sys.exit('qemu-system-xtensa not on PATH (idf_tools.py install qemu-xtensa)')
    args = [qemu, '-M', 'esp32s3', '-m', '32M', '-nographic', '-no-reboot',
            '-drive', f'file={flash},if=mtd,format=raw',
            '-drive', f'file={efuse},if=none,format=raw,id=efuse',
            '-global', 'driver=nvram.esp32s3.efuse,property=drive,value=efuse',
            '-global', 'driver=timer.esp32s3.timg,property=wdt_disable,value=true']
    t0 = time.time()
    p = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    summary, crash, buf = None, None, b''
    try:
        with open(log, 'w') as f:
            while summary is None:  # -no-reboot: a panic/reset ends QEMU, after the backtrace
                if time.time() - t0 > timeout:
                    crash = crash or f'timeout after {timeout:.0f} s'
                    break
                if not select.select([p.stdout], [], [], 1.0)[0]:
                    if p.poll() is not None:
                        crash = crash or f'qemu exited ({p.returncode})'
                        break
                    continue
                chunk = os.read(p.stdout.fileno(), 4096)
                if not chunk:
                    crash = crash or 'qemu output closed'
                    break
                buf += chunk
                while b'\n' in buf and summary is None:
                    line, buf = buf.split(b'\n', 1)
                    s = line.decode('utf-8', 'replace').rstrip('\r')
                    f.write(s + '\n')
                    print(s, flush=True)
                    if SUMMARY.match(s.strip()):
                        summary = s.strip()
                    elif crash is None and any(c in s for c in CRASH):
                        crash = s.strip()
    finally:
        p.kill()
        p.wait()
    return summary, crash, time.time() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-B', '--build-dir', default='build-qemu')
    ap.add_argument('--no-build', action='store_true')
    ap.add_argument('--timeout', type=float, default=1200)
    ap.add_argument('--log', default=None, help='default: <build-dir>/unity_qemu.log')
    a = ap.parse_args()
    bdir = os.path.abspath(os.path.join(ROOT, a.build_dir) if not os.path.isabs(a.build_dir) else a.build_dir)
    if 'IDF_PATH' not in os.environ:
        sys.exit('IDF_PATH not set: source ./idf-export.sh first')
    if not a.no_build:
        build(bdir)
    flash, efuse = images(bdir)
    summary, crash, dt = run(flash, efuse, a.timeout, a.log or os.path.join(bdir, 'unity_qemu.log'))
    if summary is None:
        print(f'\nNO UNITY SUMMARY: {crash} ({dt:.0f} s)')
        return 2
    fails = int(SUMMARY.match(summary).group(2))
    print(f'\n{summary} ({dt:.0f} s)')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
