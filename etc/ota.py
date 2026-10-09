import datetime

import requests

fallback_hosts = [
    # (ip, port, name),
    # ('192.168.4.3', 0, 'flat'),
    # ('192.168.4.2', 0, 'fry'), # 192.168.4.3
]

"""

1. build (idf.py build)
2. start web server to serve firmware binary
    python3 -m http.server 9000upd
3. discover hosts
4 iterate hosts
    > ota http://192.168.1.161:9000/build/fugu-firmware.bin

idf.py build

Safety guards (read from the build's sdkconfig.json, see ota_build_flags.py):
- a no-network build (CONFIG_FUGU_WITH_NETW=n) warns; devices lose Wi-Fi after OTA.
- a plant-sim build (CONFIG_FUGU_WITH_VCONV=y) warns hard: VCONV swaps the real PWM driver for a
  simulator (src/buck.h) so a real converter makes 0W while looking alive.
- an uncommitted build (git-describe version ends in '-dirty') warns: the image maps to no commit.
- a gate-driver change (image MCPWM vs device LEDC/VCONV or the reverse, probed with a no-arg
  `pwm-freq`) warns; so does a device whose driver can't be established.
  All four require an interactive y/N and are refused non-interactively (the driver check unless
  --allow-driver-change).

"""""
import sys
sys.path.insert(0, ".")
import argparse
import asyncio
import atexit
import datetime
import json
import os
import re
import socket
import struct
import subprocess
import sys
import time

from rich.console import Console as RichConsole
from rich.table import Table

from etc.fugu.console import Console
from etc.fugu.discover import discover_scope_servers
from etc.fugu.fugu import FuguDevice
from etc.fugu.transport import SocketTransport
from etc.fugu_console import scan_nat_async

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'idf-devtools'))
import elf_archive  # vendored submodule (github.com/fl4p/idf-devtools)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ota_build_flags import (build_has_networking, build_is_plant_sim, build_gate_driver,
                             parse_device_gate_driver, gate_driver_verdict)

# CLion / PyCharm Run consoles report as a TTY but don't render ANSI escapes —
# disable color/style so the boxes don't come out wrapped in raw \x1b[...m codes.
# Also fix width to a wide value: with force_terminal=False rich would default to
# 80 cols and squash the table, but JB's run console happily prints long lines.
_in_jb_run = bool(os.environ.get('PYCHARM_HOSTED'))
_rich_console = RichConsole(color_system=None if _in_jb_run else 'auto',
                            force_terminal=False if _in_jb_run else None,
                            highlight=not _in_jb_run,
                            width=200 if _in_jb_run else None)

HTTP_PORT = 9000
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_BIN = os.path.join(REPO_ROOT, 'build', 'fugu-firmware.bin')
APP_DESC_MAGIC = 0xABCD5432

argp = argparse.ArgumentParser(description='Push an OTA build to all discovered devices.')
argp.add_argument('-f', '--force', action='store_true',
                  help='update even when the device already runs the local build')
argp.add_argument('-n', '--dry-run', action='store_true',
                  help='only print what would be updated, do not send `ota`')
argp.add_argument('-m', '--match', metavar='REGEX',
                  help='only act on devices whose name matches REGEX (re.search)')
argp.add_argument('--allow-driver-change', action='store_true',
                  help='push even if the image\'s gate driver differs from the device\'s or is unverified')
# args/hosts are module globals populated by cli() (under __main__) so importing this module for
# its helpers doesn't parse argv or hit the network.


def read_local_app_desc(bin_path):
    """Parse esp_app_desc_t from the start of a firmware .bin: {project, version, date, time, idf}."""
    try:
        with open(bin_path, 'rb') as f:
            data = f.read(0x200)
    except FileNotFoundError:
        return None
    off = data.find(struct.pack('<I', APP_DESC_MAGIC))
    if off < 0:
        return None
    def s(rel, n):
        return data[off+rel:off+rel+n].split(b'\x00', 1)[0].decode('utf-8', 'replace')
    return dict(version=s(0x10, 32), project=s(0x30, 32),
                time=s(0x50, 16), date=s(0x60, 16), idf=s(0x70, 32))


# build_has_networking() / build_is_plant_sim() (read sdkconfig.json) live in ota_build_flags.py
# so they stay host-unit-testable without this module's network/argparse import side effects.


RE_APP_LINE = re.compile(r'App:\s+(\S+)\s+v\S+\s+(\S+)\s+\(built (.+),\s+IDF\s+(\S+)\)')


def parse_device_app(lines):
    """Pull {project, version, built, idf} from the App: line of an `uptime` reply, or None."""
    for l in lines:
        m = RE_APP_LINE.search(l)
        if m:
            return dict(project=m.group(1), version=m.group(2),
                        built=m.group(3), idf=m.group(4))
    return None


def ensure_http_server():
    """Spawn `python3 -m http.server` serving the repo root if port is free."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        if s.connect_ex(('127.0.0.1', HTTP_PORT)) == 0:
            return  # something already listening
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    print(f'starting http.server on :{HTTP_PORT} (cwd={repo_root})')
    proc = subprocess.Popen([sys.executable, '-m', 'http.server', str(HTTP_PORT)],
                            cwd=repo_root, stderr=subprocess.DEVNULL)
    atexit.register(proc.terminate)
    time.sleep(.5)


async def fetch_version(addr, port, name, retry=False):
    """Run `uptime` over a fresh telnet console; return {uptime, app, error}.
    With `retry`, one extra attempt is made after a short backoff if the first fails."""
    def do_once():
        try:
            st = SocketTransport(addr, port=port or SocketTransport.DEFAULT_PORT, timeout=5)
            con = Console(st, eol='\n', wait_banner=True)
        except Exception as e:
            return dict(uptime=None, app=None, error=f'{type(e).__name__}: {e}')
        try:
            reply = con.command('uptime', timeout=5)
            if reply.timed_out:
                return dict(uptime=None, app=None, error='timeout')
            up = None
            for l in reply:
                if m := re.search(r'Uptime:\s*(\d+)', l):
                    up = int(m.group(1)); break
            app = parse_device_app(reply)
            if up is None and app is None:
                return dict(uptime=None, app=None, error='no uptime info in reply')
            return dict(uptime=up, app=app, error=None)
        except Exception as e:
            return dict(uptime=None, app=None, error=f'{type(e).__name__}: {e}')
        finally:
            con.close()
    def do():
        out = do_once()
        if retry and out['error']:
            time.sleep(2)
            return do_once()
        return out
    return await asyncio.to_thread(do)


async def fetch_gate_driver(addr, port):
    """Running gate driver of a device via a no-arg (read-only) `pwm-freq`: 'mcpwm', 'other' or None."""
    def do():
        try:
            st = SocketTransport(addr, port=port or SocketTransport.DEFAULT_PORT, timeout=5)
            con = Console(st, eol='\n', wait_banner=True)
        except Exception:
            return None
        try:
            r = con.command('pwm-freq', timeout=5)
            return parse_device_gate_driver(r, r.ok, r.rejected)
        except Exception:
            return None
        finally:
            con.close()
    return await asyncio.to_thread(do)


def fmt_uptime(seconds):
    """Seconds → human-readable hours (e.g. 6789 → '1.89h'); None → '-'.
    Uses datetime.timedelta as the canonical parse."""
    if seconds is None:
        return '-'
    td = datetime.timedelta(seconds=int(seconds))
    return f'{td.total_seconds() / 3600:.2f}h'


def print_table(rows, cols, title=None):
    """rows: list of dicts. cols: list of (key, header, kw) tuples;
    kw is per-column kwargs for Table.add_column (no_wrap, min_width, etc.)."""
    t = Table(title=title, show_header=True, header_style='bold')
    for col in cols:
        k, h, kw = (*col, {})[:3] if len(col) == 2 else col
        t.add_column(h, **kw)
    for r in rows:
        t.add_row(*(str(r.get(k, '')) for k, *_ in cols))
    _rich_console.print(t)


async def print_versions(hosts, title='installed firmware versions:', settle=0.0, retry=False):
    """Print a table of each host's uptime/version/build; return {name: parsed_app_or_None}.
    `settle` waits before querying (lets a just-rebooted device's telnet come up)."""
    if settle:
        await asyncio.sleep(settle)
    results = await asyncio.gather(*[fetch_version(ip, port, name, retry=retry) for ip, port, name in hosts])
    rows, parsed = [], {}
    for (ip, port, name), info in zip(hosts, results):
        app, up, err = info['app'], info['uptime'], info['error']
        rows.append(dict(
            name=name,
            addr=f'{ip}:{port or SocketTransport.DEFAULT_PORT}',
            uptime=fmt_uptime(up),
            version=(app['version'] if app else None) or err or '?',
            built=(app['built'] if app else '') or '',
        ))
        parsed[name] = app
    print_table(rows, [
        ('name',    'NAME',    {'no_wrap': True, 'min_width': 14}),
        ('addr',    'ADDRESS', {'no_wrap': True, 'min_width': 18}),
        ('uptime',  'UPTIME',  {'no_wrap': True, 'justify': 'right', 'min_width': 7}),
        ('version', 'VERSION', {'no_wrap': True, 'min_width': 32}),
        ('built',   'BUILT',   {'no_wrap': True, 'min_width': 20}),
    ], title=title)
    return parsed


async def send_ota_command(addr, port, name):
    print('\n', name)
    st = SocketTransport(addr, port=port or SocketTransport.DEFAULT_PORT, timeout=10)
    fd = FuguDevice(st, block=True, prefix=name)
    fd.verbose = True
    ota_progress = 0
    success = False

    def on_message(rx):
        nonlocal ota_progress, success
        m = re.match(r'.*ota: Download Progress:\s*([0-9.]+)\s*%.*', rx)
        if m:
            ota_progress = float(m[1])
        if 'OTA Succeed' in rx:
            if ota_progress != 100:
                print('OTA Succeed msg but progress ', ota_progress, '!= 100')
                ota_progress = 101
            else:
                success = True

    fd.on_message = on_message

    private_ip, *_ = st.sock.getsockname()
    # fd.wait_for_pwm_state()
    fd.write(f"ping\n")  # clear command buffer on device
    time.sleep(.1)

    url = f"http://{private_ip}:9000/build/fugu-firmware.bin"
    import email.utils
    print('firmware mtime', email.utils.parsedate_to_datetime(requests.head(url).headers['Last-Modified']).astimezone().isoformat())
    fd.write(f"ota {url}\n")
    # fd.write(f"reset\n")
    # print(fd.prefix, fd.pwm_state)
    while ota_progress < 100:
        if not st.check_connection():
            print(fd.prefix, 'connection to device unexpectedly closed @ota_progress=', ota_progress)
            return False
        await asyncio.sleep(.3)

    if ota_progress != 100:
        return False

    print(fd.prefix, 'waiting for device to close the connection..')
    t_wait_start = time.time()
    while st.check_connection():
        if time.time() - t_wait_start > 30:
            # device may have rebooted without sending FIN; TCP keepalive should have
            # tripped check_connection by now — bail out instead of spinning forever
            print(fd.prefix, 'still "connected" after 30s, forcing close')
            st.close()
            break
        time.sleep(.2)
    print(fd.prefix, 'closed the connection')
    fd.close()

    if not success:
        print(fd.prefix, 'didnt send a success msg')
        return False

    # now try to re-connect
    print(fd.prefix, 'waiting for device to come online again')
    for _ in range(10):
        time.sleep(1)
        st = SocketTransport(addr, port=port or SocketTransport.DEFAULT_PORT)
        try:
            fd = FuguDevice(st, block=True, prefix=name)
        except (ConnectionRefusedError, TimeoutError):
            continue
        print(fd.prefix, 'device back online! OTA successful (probably TODO check ver)')
        fd.close()
        break
    else:
        print(fd.prefix, 'device didnt come online in time')
        return False

    return True

async def send_ota_command_try(addr, port, name):
    try:
        return await send_ota_command(addr,port,name)
    except Exception as e:
        print('err', addr, name, e)
        return False



async def main():
    local = read_local_app_desc(FIRMWARE_BIN)
    if local:
        print(f"local build: {local['project']} {local['version']} "
              f"(built {local['date']} {local['time']}, IDF {local['idf']})")
    else:
        print(f'WARN: could not parse app_desc from {FIRMWARE_BIN}; --force assumed')

    before = await print_versions(hosts)

    if args.force or not local:
        to_update = list(hosts)
    else:
        to_update = []
        for h in hosts:
            dev = before.get(h[2])
            if dev and dev.get('version') == local['version']:
                print(f" ☑️ skip {h[2]}: already at {local['version']}")
            else:
                to_update.append(h)
    if not to_update:
        print('all devices up to date')
        return True

    netw = build_has_networking(FIRMWARE_BIN)
    if netw is False:
        print('⚠️  this build has networking DISABLED (CONFIG_FUGU_WITH_NETW=n) — '
              'devices will lose Wi-Fi after OTA and can only be recovered by serial.')

    sim = build_is_plant_sim(FIRMWARE_BIN)
    if sim:
        print('⛔  this is a PLANT-SIMULATION build (CONFIG_FUGU_WITH_VCONV=y) — the real PWM '
              'driver is replaced by a simulator (src/buck.h), so the half-bridge never switches. '
              'Flashing it to a real converter yields 0W (Vin pinned at Voc) while the device '
              'still looks alive. This is a bench/sim image; do NOT push it to fry/flat.')

    # 'dirty' in the git-describe version => built from a working tree with uncommitted changes.
    # The natural version-match skip already keeps a rebuild of the *same* dirty version off devices;
    # we only reach here when this image would actually land (--force, or a device on another
    # version), so warn loudly: uncommitted firmware can't be reproduced or bisected later.
    dirty = bool(local) and 'dirty' in local['version']
    if dirty:
        print(f'⚠️  UNCOMMITTED BUILD: local image is "{local["version"]}" (git-describe --dirty) — '
              'built from a working tree with uncommitted changes. Commit or stash first so the '
              'firmware on the device maps to a real commit.')

    img_drv = build_gate_driver(FIRMWARE_BIN)
    dev_drv = await asyncio.gather(*[fetch_gate_driver(ip, port) for ip, port, _ in to_update])
    drv_bad = []
    for (_, _, name), dev in zip(to_update, dev_drv):
        verdict = gate_driver_verdict(img_drv, dev)
        if verdict != 'ok':
            drv_bad.append(name)
            dev_s = {'mcpwm': 'mcpwm', 'other': 'ledc/vconv'}.get(dev, 'unknown')
            print(f'⚠️  gate driver {verdict.upper()} on {name}: image {img_drv or "unknown"}, '
                  f'device {dev_s} (Kconfig FUGU_GATE_DRIVER; a wrong driver switches the '
                  'half-bridge with the wrong timing and dead-time model).')

    if args.dry_run:
        print('dry-run, would update:')
        for _, _, name in to_update:
            print(f'  - {name}')
        return True

    if netw is False:
        if not sys.stdin.isatty():
            print('aborting: no-network image, refusing to OTA non-interactively')
            return False
        if input('proceed anyway? [y/N] ').strip().lower() not in ('y', 'yes'):
            print('aborted')
            return False

    if sim:
        if not sys.stdin.isatty():
            print('aborting: plant-sim (VCONV) image, refusing to OTA non-interactively')
            return False
        if input('really OTA a plant-sim build to these devices? [y/N] ').strip().lower() not in ('y', 'yes'):
            print('aborted')
            return False

    if drv_bad and not args.allow_driver_change:
        if not sys.stdin.isatty():
            print('aborting: gate driver changes or is unverified on ' + ', '.join(drv_bad)
                  + '; refusing non-interactively (--allow-driver-change to override)')
            return False
        if input('really OTA across a gate-driver change? [y/N] ').strip().lower() not in ('y', 'yes'):
            print('aborted')
            return False

    if dirty:
        if not sys.stdin.isatty():
            print('aborting: uncommitted (dirty) image, refusing to OTA non-interactively')
            return False
        if input('really OTA an uncommitted (dirty) build? [y/N] ').strip().lower() not in ('y', 'yes'):
            print('aborted')
            return False

    ensure_http_server()
    time.sleep(1)
    res = await asyncio.gather(*[send_ota_command_try(ip, port, name) for ip, port, name in to_update])
    res = dict(zip((name for _, _, name in to_update), res))
    for name, ok in res.items():
        print('%20s: %s' % (name, '✅' if ok else '❌'))

    after = await print_versions(hosts, title='installed firmware versions (after):',
                                 settle=3.0, retry=True)

    if local:
        print('verify:')
        for _, _, name in to_update:
            dev = after.get(name)
            got = dev.get('version') if dev else None
            if got == local['version']:
                print(f'  {name}: ✅ {got}')
            else:
                print(f'  {name}: ❌ got {got!r}, expected {local["version"]!r}')
                res[name] = False

    # archive the flashed ELF per device so a later coredump can be symbolicated
    for name, ok in res.items():
        if ok:
            try:
                elf_archive.archive(name, method='ota',
                                    version=local['version'] if local else None)
            except Exception as e:
                print(f'  WARN: ELF archive failed for {name}: {e}')

    return all(res.values())


def discover_scope_hosts(match=None, attempts=4, timeout=3, min_attempts=2):
    """mDNS _scope._tcp browse, retried + accumulated. A power-saved/weak/flapping node
    (e.g. flat, which lives directly on the LAN and is only findable via mDNS) routinely
    misses a single 1 s window, so give it several wider ones. Stops early once a --match
    target shows up, or — no match — once results go stable after min_attempts.
    Returns [(host, 23, name)] (telnet port, matching the rest of ota.py)."""
    pat = re.compile(match) if match else None
    seen = {}
    for i in range(attempts):
        before = len(seen)
        for h, _port, name in discover_scope_servers(timeout=timeout):
            name = name.rstrip('.')                       # mDNS server name → bare hostname
            if name.endswith('.local'):
                name = name[:-len('.local')]
            seen[(h, name)] = (h, 23, name)
        if pat and any(pat.search(n) for _, _, n in seen.values()):
            break
        if not pat and i + 1 >= min_attempts and len(seen) == before and seen:
            break
    return list(seen.values())


def cli():
    """Parse argv, discover + match hosts, run the OTA. Sets the module globals main() reads."""
    global args, hosts
    args = argp.parse_args()
    hosts = discover_scope_hosts(match=args.match)
    hosts += asyncio.run(scan_nat_async(reachable_only=True))
    hosts = hosts or fallback_hosts
    if not hosts:
        print('no hosts discovered!')
        sys.exit(1)
    if args.match:
        pat = re.compile(args.match)
        hosts = [h for h in hosts if pat.search(h[2])]
        if not hosts:
            print(f'no devices match {args.match!r}')
            sys.exit(1)
    print(hosts)
    sys.exit(0 if asyncio.run(main()) else 1)


if __name__ == '__main__':
    cli()
