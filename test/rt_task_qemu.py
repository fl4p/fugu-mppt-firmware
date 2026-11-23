"""Run test/main_rt_task.cpp (rtLoopRunning()) under QEMU (esp32s3).

  MAIN_SRC=../test/main_rt_task.cpp idf.py -B build/rtqemu -DSDKCONFIG=build/rtqemu/sdkconfig build
  (QEMU's flash has no QIO: set CONFIG_ESPTOOLPY_FLASHMODE_DIO=y in build/rtqemu/sdkconfig first)
  python3 test/rt_task_qemu.py build/rtqemu
"""
import os
import shutil
import subprocess
import sys


def main(build):
    flash = os.path.join(build, 'qemu_flash.bin')
    efuse = os.path.join(build, 'qemu_efuse.bin')
    subprocess.check_call([sys.executable, '-m', 'esptool', '--chip=esp32s3', 'merge-bin', '--output=qemu_flash.bin',
                           '--pad-to-size=4MB', '@flash_args'], cwd=build)
    sys.path.insert(0, os.path.join(os.environ['IDF_PATH'], 'tools'))
    from idf_py_actions.qemu_ext import QEMU_TARGETS
    with open(efuse, 'wb') as f:
        f.write(QEMU_TARGETS['esp32s3'].default_efuse)
    qemu = shutil.which('qemu-system-xtensa') or sys.exit('qemu-system-xtensa not on PATH')
    p = subprocess.Popen([qemu, '-M', 'esp32s3', '-m', '32M', '-nographic', '-no-reboot',
                          '-drive', f'file={flash},if=mtd,format=raw',
                          '-drive', f'file={efuse},if=none,format=raw,id=efuse',
                          '-global', 'driver=nvram.esp32s3.efuse,property=drive,value=efuse',
                          '-global', 'driver=timer.esp32s3.timg,property=wdt_disable,value=true'],
                         stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = []
    try:
        for line in iter(p.stdout.readline, b''):
            s = line.decode('utf-8', 'replace').rstrip()
            out.append(s)
            print(s)
            if 'RTTEST done' in s:
                break
    finally:
        p.kill()
        p.wait()
    return any('RTTEST done 0 failures' in l for l in out) and not any('RTTEST FAIL' in l for l in out)


if __name__ == '__main__':
    sys.exit(0 if main(sys.argv[1] if len(sys.argv) > 1 else 'build/rtqemu') else 1)
