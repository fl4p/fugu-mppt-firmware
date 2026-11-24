#!/usr/bin/env python3
"""OTA devices through their MQTT consoles, one after the other, and verify each.

    etc/fleet-build.sh                    # fry/flat image from a clean worktree (sdkconfig.d/fleet, IDF 6.0.3)
    etc/ota_mqtt.py --name fry,flat --bin ../fugu-fleet-wt/build/fleet/fugu-firmware.bin -n
    etc/ota_mqtt.py --name fry,flat --bin ... [--check wire --check "rect-cal status"]

Setup: broker and credentials from etc/mqtt.env (relay/ssh settings from etc/nat.env; a worktree
falls back to the main checkout's files); an ssh tunnel when the broker is not directly reachable.
Per device: lock onto the exact hostname, read `bootinfo`, run the pre-flight checks (image flags,
-dirty image, rect-cal/sweep/balance activity, RTC no-init layout vs the running build's archived
ELF, the device's previous OTA record, the --check commands). Then, holding the system locks
ota-<device> and ota-relay (lock.sh): serve the image (locally on the broker LAN, else staged on
the relay with etc/ota-via-relay.sh), raise the NAT gateway's Wi-Fi TX power, send `ota <url>`,
follow the download and the reboot into the new image (failed downloads are retried with
backoff). --verify then waits for rollback=VALID, watches the log for faults, compares power and
the --check outputs, and every OTA is recorded in elf-archive/ota-owners.jsonl. A device that
fails stops the rest. `--dry-run` does everything up to sending `ota` (no locks).
Run from the repo root with .venv/bin/python3.
"""
import argparse
import atexit
import datetime
import difflib
import getpass
import glob
import hashlib
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import signal
import time
import urllib.error
import urllib.request

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from etc.fugu.console import Console  # noqa: E402
from etc.fugu.transport import MqttTransport  # noqa: E402
from etc.fugu_console import (broker_endpoint, etc_file, free_port, load_env_file,  # noqa: E402
                              relay_ssh_dest, run_ota, tcp_ok)
from ota_common import (archive_elf, bootinfo_row, build_gate_driver,  # noqa: E402
                        build_has_networking, build_is_plant_sim, device_driver_label,
                        gate_driver_verdict, image_is_dirty, is_current, is_pending_verify, judge,
                        parse_bootinfo, parse_device_gate_driver, print_table, read_local_app_desc,
                        require_elf_archive)
from ota_common import confirm as yes_no  # noqa: E402
import rich  # noqa: E402,F401  (result table; fail before the OTA, not after it)

elf_archive = require_elf_archive()  # vendored submodule (etc/idf-devtools); fail before any OTA

RE_HOSTNAME = re.compile(r'Hostname:\s*(\S+)')
RE_OTA_URL = re.compile(r'^ota\s+(https?://\S+)\s*$', re.M)
RE_TOKEN = re.compile(r'^token\s+(\S+)\s*$', re.M)
RE_STATUS = re.compile(r'V=\s*([\d.]+)/\s*([\d.]+) I=\s*([\d.]+)/\s*([\d.]+)A\s+([-\d.]+)W.*?st=\s*([^,\s]+)')
RE_DL_TOOK = re.compile(r'Download took (\d+) ms \((\d+) KB/s\)')
# fault lines of a new boot (src/: main.cpp ADC/latency watchdogs, ina22x, mppt.h shutdownDcdc); the
# sweep-no-mpp backoff, `START blocked: backoff` and the ina22x self-test line are normal at boot
RE_FAULT = re.compile(r'ADC stall|Never got a sample|ina22x.*timeout|timeout!|Loop latency high|ADC error|'
                      r'mppt: (PSU escalated )?backoff \d+s \[(?!sweep-no-mpp\])|PSU latch|'
                      r'Guru Meditation|panic|abort\(\)|Backtrace:|assert failed|Rebooting|'
                      r'reset reason: (?!SW\b)', re.I)
RE_LOGTS = re.compile(r'^[EWIDV] \((\d+)\) ')
RE_LOGPFX = re.compile(r'^([EWI]) \(\d+\) ')
OTA_BUDGET = 330.0  # firmware OTA watchdog (300 s) + margin
LIVE = ('fry', 'flat')  # live converters: no -dirty firmware without --allow-dirty
FW_INPUTS = ('src/', 'main/', 'components/', 'lib/', 'sdkconfig', 'CMakeLists.txt', 'partitions.csv')
LOCK_SH = os.path.expanduser('~/bin/lock.sh')
OWNERS = os.path.join(elf_archive.ARCHIVE_DIR, 'ota-owners.jsonl')


def is_relevant(line):
    """Log lines worth echoing during an OTA (the status stream is noise here)."""
    return bool(re.search(r'\bota\b|OTA|Download|HTTP_|main: ota', line, re.I))


def route_src(host):
    """Local address this host would use to reach `host`, or None."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as u:
            u.connect((host, 9))
            return u.getsockname()[0]
    except OSError:
        return None


def same_24(a, b):
    return bool(a and b) and a.split('.')[:3] == b.split('.')[:3]


class LocalServer:
    """http.server on this host's LAN address, serving the image's directory."""

    def __init__(self, bin_path, addr):
        self.bin, self.addr, self.proc = bin_path, addr, None

    def stage(self):
        if self.proc is None or self.proc.poll() is not None:
            port = free_port(9000)
            self.proc = subprocess.Popen(
                [sys.executable, '-m', 'http.server', str(port), '--bind', self.addr,
                 '--directory', os.path.dirname(self.bin)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.url = f'http://{self.addr}:{port}/{os.path.basename(self.bin)}'
            time.sleep(0.7)
        with urllib.request.urlopen(urllib.request.Request(self.url, method='HEAD'), timeout=5) as r:
            if int(r.headers.get('Content-Length', -1)) != os.path.getsize(self.bin):
                raise RuntimeError(f'{self.url}: size mismatch')
        return self.url

    def status(self):
        return 'up' if self.proc and self.proc.poll() is None else 'down'

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()


class Relay:
    """Stage and serve the image on the relay host with etc/ota-via-relay.sh (own dir + port)."""

    def __init__(self, bin_path, env):
        self.bin, self.env, self.token, self.url = bin_path, env, None, None

    def _run(self, *a):
        return subprocess.run([os.path.join(REPO_ROOT, 'etc', 'ota-via-relay.sh'), *a],
                              env=self.env, capture_output=True, text=True, timeout=180)

    def status(self):
        if not self.token:
            return 'not staged'
        return (self._run('status', self.token).stdout.strip().splitlines() or ['?'])[-1]

    def stage(self):
        if self.token and self.status().startswith('up'):
            return self.url
        self.stop()
        r = self._run(self.bin)
        m, t = RE_OTA_URL.search(r.stdout), RE_TOKEN.search(r.stdout)
        self.token = t and t[1]
        if r.returncode or not m:
            raise RuntimeError(f'relay staging failed (exit {r.returncode}): '
                               f'{(r.stderr or r.stdout).strip()[-300:]}')
        self.url = m[1]
        return self.url

    def stop(self):
        if self.token:
            self._run('stop', self.token)
            self.token = None


def default_gateway():
    """$OTA_GW, else the host of the first $NAT_TELNET entry, else 192.168.1.231."""
    nat = (os.environ.get('NAT_TELNET') or '').split(',')[0].strip()
    return os.environ.get('OTA_GW') or nat.rsplit(':', 1)[0] or '192.168.1.231'


class GatewayTx:
    """Raise the Wi-Fi TX power of the NAT gateway (nimble-ble-proxy) for a transfer as a timed
    boost and cancel it afterwards (a plain set of the persisted level). Talks HTTP directly when
    port 80 is reachable, else runs curl on `ssh_dest`. Failures only warn."""

    def __init__(self, host, ssh_dest, dbm, hold_s):
        self.host, self.ssh_dest, self.dbm, self.hold_s = host, ssh_dest, dbm, min(int(hold_s), 900)
        self.direct = tcp_ok(host, 80, 1.5)
        self.orig = None

    def _call(self, method, query=''):
        url = f'http://{self.host}/txpower' + (f'?{query}' if query else '')
        if self.direct:
            req = urllib.request.Request(url, method=method,
                                         headers={'X-NBP-Confirm': 'yes'} if method == 'POST' else {})
            try:
                with urllib.request.urlopen(req, timeout=6) as r:
                    return r.status, r.read().decode(errors='replace')
            except urllib.error.HTTPError as e:
                return e.code, e.read().decode(errors='replace')
        cmd = ['curl', '-sS', '-m', '6', '-w', r'\n%{http_code}']
        if method == 'POST':
            cmd += ['-X', 'POST', '-H', 'X-NBP-Confirm: yes']
        # own session: a second Ctrl-C must not kill the restore
        r = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', self.ssh_dest,
                            ' '.join(f"'{a}'" for a in cmd + [url])],
                           stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=25,
                           start_new_session=True)
        body, _, code = r.stdout.rpartition('\n')
        if r.returncode or not code.isdigit():
            raise RuntimeError(f'ssh/curl exit {r.returncode}: {(r.stderr or r.stdout).strip()[-200:]}')
        return int(code), body.strip()

    def get(self):
        code, body = self._call('GET')
        if code != 200:
            raise RuntimeError(f'GET /txpower: HTTP {code} {body[:120]}')
        return json.loads(body)

    def boost(self):
        try:
            cur = self.get()
            wifi, cfg = cur.get('wifi'), cur.get('wifi_cfg', cur.get('wifi'))
            if not isinstance(wifi, int) or wifi < 2:
                print(f'gw {self.host}: tx boost skipped, wifi={wifi!r} ({cur})')
                return
            if cur.get('revert_s'):
                print(f'gw {self.host}: another boost is active ({cur}), leaving it')
                return
            if wifi >= self.dbm:
                print(f'gw {self.host}: tx already {wifi} dBm, no boost')
                return
            timed = 'revert_s' in cur
            q = f'wifi={self.dbm}' + (f'&for={self.hold_s}' if timed else '')
            code, body = self._call('POST', q)
            got = self.get()
            if got.get('wifi') != self.dbm:
                raise RuntimeError(f'POST /txpower?{q}: HTTP {code} {body[:120]}, GET {got}')
            self.orig = cfg
            print(f'gw {self.host}: tx boost {wifi} -> {self.dbm} dBm ({q}), GET {got}')
        except Exception as e:
            print(f'⚠️  gw {self.host}: tx boost failed ({type(e).__name__}: {e}); continuing')

    def restore(self):
        if self.orig is None:
            return
        try:
            # a plain set of the persisted level cancels the timed boost
            code, body = self._call('POST', f'wifi={self.orig}')
            got = self.get()
            if got.get('revert_s'):  # older gateway firmware: replace it by a 1 s boost at the old level
                self._call('POST', f'wifi={self.orig}&for=1')
                time.sleep(1.5)
                got = self.get()
            if got.get('wifi') != self.orig or got.get('revert_s'):
                raise RuntimeError(f'HTTP {code} {body[:80]}, GET {got}')
            print(f'gw {self.host}: tx restored, boost cancelled: GET {got}')
            self.orig = None
        except Exception as e:
            print(f'⚠️  gw {self.host}: tx restore to {self.orig} dBm FAILED '
                  f'({type(e).__name__}: {e}); set it by hand: POST /txpower?wifi={self.orig}')


class Locks:
    """System-wide lock.sh locks owned by this process (stale, so takeable, once it dies)."""

    def __init__(self, wait):
        self.wait, self.held, self.sid = wait, [], None
        self.env = dict(os.environ, AGENT_LOCK_OWNER_PID=str(os.getpid()))

    def _run(self, *a):
        return subprocess.run([LOCK_SH, *a], env=self.env, capture_output=True, text=True,
                              timeout=self.wait + 60)

    def take(self, name, note):
        if name in self.held:
            return True
        if self.sid is None:
            r = self._run('newsession', 'ota')
            if r.returncode or not r.stdout.strip():
                print(f'⛔ lock.sh newsession failed (exit {r.returncode}): {r.stderr.strip()}')
                return False
            self.sid = r.stdout.strip()
        if self._run('resolve', name).returncode:
            r = self._run('acquire', name, note, '--new', '--session', self.sid)
        elif self.wait:
            print(f'waiting up to {self.wait:.0f} s for lock {name} ...')
            r = self._run('wait', name, note, str(int(self.wait)), '--session', self.sid)
        else:
            r = self._run('acquire', name, note, '--session', self.sid)
        if r.returncode == 0:
            self.held.append(name)
            return True
        print(f'⛔ lock {name} not taken (exit {r.returncode}; --lock-wait S to queue):\n'
              f'{(r.stdout + r.stderr).strip()}')
        return False

    def release(self, *names):
        for n in [n for n in reversed(self.held) if not names or n in names]:
            r = self._run('release', n, '--session', self.sid)
            if r.returncode:
                print(f'⚠️  lock {n} not released (exit {r.returncode}): {(r.stdout + r.stderr).strip()}')
            self.held.remove(n)


class ExactMqtt(MqttTransport):
    """MqttTransport that only accepts `pv/log/<device>` exactly, not a substring match."""

    def _on_message(self, client, userdata, msg):
        parts = msg.topic.split('/')
        if len(parts) == 3 and parts[2] == self.device:
            super()._on_message(client, userdata, msg)


class Tap(list):
    """Every console line with its monotonic receive time."""

    def __call__(self, line):
        self.append((time.monotonic(), line))

    def __bool__(self):  # Console tests on_line for truth
        return True

    def status(self, since=0.0):
        """Fields of the newest status line received after `since`, or None."""
        for t, l in reversed(self):
            if t < since:
                break
            if m := RE_STATUS.search(l):
                return dict(vin=float(m[1]), vout=float(m[2]), iin=float(m[3]), iout=float(m[4]),
                            p=float(m[5]), st=m[6])
        return None


def fmt_status(s):
    return (f"{s['p']:.0f} W, Vin {s['vin']:.1f} V, Vout {s['vout']:.2f} V, Iout {s['iout']:.2f} A, "
            f"st={s['st']}") if s else 'no status line'


def ask(con, cmd, parse, budget):
    """Repeat `cmd` until `parse(reply)` is truthy or `budget` s pass (a busy console, e.g. one
    blocked in another client's `sleep`, answers late or drops the command)."""
    deadline = time.monotonic() + budget
    while True:
        if out := parse(con.command(cmd, timeout=8)):
            return out
        if time.monotonic() > deadline:
            return None


def parse_hostname(lines):
    return next((m[1] for l in lines if (m := RE_HOSTNAME.search(l))), None)


def confirm(prompt, args):
    if args.dry_run:
        return True
    return yes_no(f'{prompt} — proceed? [y/N] ', f'aborting: {prompt} (refused non-interactively)',
                  accept=args.yes and f'{prompt}: accepted (--yes)')


def git(*a, cwd=REPO_ROOT):
    r = subprocess.run(['git', '-C', cwd, *a], capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else None


def firmware_dirt(bin_path):
    """Uncommitted firmware inputs of the git tree bin_path was built in; None if not in a tree."""
    top = git('rev-parse', '--show-toplevel', cwd=os.path.dirname(bin_path))
    if top is None:
        return None
    out = git('status', '--porcelain', '--ignore-submodules=none', cwd=top.strip()) or ''
    return [p for p in (l[3:] for l in out.splitlines()) if p.startswith(FW_INPUTS)]


def image_git_sha(version):
    m = re.search(r'-g([0-9a-f]{7,})', version or '')
    return m[1] if m else None


def owner():
    """Who runs this OTA: $OTA_OWNER, else the Claude session name, else user@host."""
    if v := os.environ.get('OTA_OWNER'):
        return v
    sid = os.environ.get('CLAUDE_CODE_SESSION_ID')
    finder = os.path.expanduser('~/dev/bin/session-find')
    if sid and os.access(finder, os.X_OK):
        try:
            out = subprocess.run([finder, sid], capture_output=True, text=True, timeout=10).stdout.split()
            if out:
                return f'{out[0]} ({sid[:8]})'
        except Exception:
            pass
    return f'{getpass.getuser()}@{socket.gethostname().split(".")[0]}'


def last_record(device):
    """Newest ota-owners.jsonl record of `device`, else its newest ELF-archive flash entry."""
    for path in (OWNERS, elf_archive.INDEX):
        try:
            with open(path) as f:
                rows = [r for r in map(json.loads, filter(str.strip, f)) if r.get('device') == device]
        except (OSError, ValueError):
            rows = []
        if rows:
            return rows[-1]
    return None


def record(entry):
    try:
        with open(OWNERS, 'a') as f:
            f.write(json.dumps(entry) + '\n')
    except OSError as e:
        print(f'⚠️  owner record not written ({e})')


def find_objdump():
    if p := shutil.which('xtensa-esp32s3-elf-objdump'):
        return p
    c = sorted(glob.glob(os.path.expanduser(
        '~/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin/xtensa-esp32s3-elf-objdump')))
    return c[-1] if c else None


def rtc_symbols(od, elf):
    """{name: (section, addr, size)} of the data objects in .rtc* sections."""
    out = {}
    for l in subprocess.run([od, '-t', elf], capture_output=True, text=True, check=True).stdout.splitlines():
        p = l.split()
        if len(p) >= 5 and p[-3].startswith('.rtc') and p[-3] != '.rtc.text' and 'O' in p[1:-3]:
            name = p[-1] if p[-1] not in out else f'{p[-1]}#{len(out)}'
            out[name] = (p[-3], int(p[0], 16), int(p[-2], 16))
    return out


def rtc_layout_check(device, running_version, bin_path):
    """Warn when RTC symbols (no-init state survives the OTA reboot) moved or changed size between
    the running build (from the ELF archive) and the image's ELF. Returns a list of warnings."""
    new_elf, _ = elf_archive.default_artifacts(os.path.dirname(bin_path))
    od = find_objdump()
    rows = [r for r in elf_archive._read_index() if r.get('version') == running_version]
    rows.sort(key=lambda r: (r.get('device') == device, r['ts']))
    shas = {r['sha'] for r in rows}
    if not od or not new_elf or not os.path.exists(new_elf) or not rows:
        print(f'rtc layout: skipped ({"no objdump" if not od else "no ELF next to the image" if not new_elf or not os.path.exists(new_elf) else f"running {running_version} not in the ELF archive"})')
        return []
    try:
        old_elf = elf_archive.extract(rows[-1]['sha'])
    except Exception as e:
        print(f'rtc layout: skipped (archived ELF {rows[-1]["sha"][:8]}: {e})')
        return []
    try:
        old, new = rtc_symbols(od, old_elf), rtc_symbols(od, new_elf)
    finally:
        os.unlink(old_elf)
    note = f', {len(shas)} archived builds share this version, used the newest' if len(shas) > 1 else ''
    warns = []
    for n in sorted(old.keys() | new.keys()):
        a, b = old.get(n), new.get(n)
        if a != b:
            fa = f'{a[0]} 0x{a[1]:08x}+{a[2]}' if a else 'absent'
            fb = f'{b[0]} 0x{b[1]:08x}+{b[2]}' if b else 'absent'
            warns.append(f'{n}: {fa} -> {fb}')
    print(f'rtc layout vs running build {rows[-1]["sha"][:8]}{note}: '
          + (f'{len(warns)} RTC symbol(s) moved/resized/added/removed' if warns else
             f'{len(new)} RTC symbols unchanged'))
    for w in warns:
        print(f'  ⚠️  {w}')
    return warns


def norm(lines):
    return [RE_LOGPFX.sub(r'\1 ', l) for l in lines
            if not RE_STATUS.search(l) and 'received serial command' not in l]


def run_checks(con, cmds):
    out = {}
    for c in cmds:
        r = con.command(c, timeout=10)
        out[c] = norm(r) + ([] if r.ok else ['<rejected>' if r.rejected else '<no reply>'])
    return out


def print_checks(before, after):
    for c, a in before.items():
        b = after.get(c, [])
        if a == b:
            print(f'check `{c}`: unchanged')
            for l in a:
                print(f'  {l}')
        else:
            print(f'check `{c}`: before -> after')
            for l in list(difflib.unified_diff(a, b, lineterm='', n=99))[2:]:
                print(f'  {l}')


def guards(args, con, image, before, name=''):
    """Image safety checks plus the device's rollback state; False aborts (a dry run prints all)."""
    ok = True
    if is_pending_verify(before):
        print('⚠️  the running image is not confirmed yet (PENDING_VERIFY): this OTA overwrites the '
              'other slot, which holds the last confirmed image (the rollback target)')
        ok = confirm('unconfirmed running image', args) and ok
    if build_has_networking(args.bin) is False:
        print('⚠️  image has networking DISABLED (CONFIG_FUGU_WITH_NETW=n): the device loses Wi-Fi')
        ok = confirm('no-network image', args) and ok
    if build_is_plant_sim(args.bin):
        print('⛔  PLANT-SIMULATION image (CONFIG_FUGU_WITH_VCONV=y): a real converter makes 0 W')
        ok = confirm('plant-sim image', args) and ok
    if image_is_dirty(image['version']):
        fw = getattr(args, 'dirt', None)
        if fw == [] and name in LIVE:
            print(f"note: {image['version']} is -dirty, but the tree it was built in has no "
                  f"uncommitted firmware inputs (as of now)")
        elif name in LIVE and not getattr(args, 'allow_dirty', False):
            print(f"⛔  {image['version']} is a -dirty build; refused for {name} without --allow-dirty. "
                  + ('Uncommitted firmware inputs: ' + ' '.join(fw) if fw else
                     'Build it clean: etc/fleet-build.sh'))
            args.would_refuse = True
            ok = False
        else:
            print(f"⚠️  uncommitted build {image['version']}")
    if not args.no_driver_check:
        r = con.command('pwm-freq', timeout=6)
        dev = parse_device_gate_driver(r, r.ok, r.rejected)
        img = build_gate_driver(args.bin)
        verdict = gate_driver_verdict(img, dev)
        if verdict != 'ok':
            print(f'⚠️  gate driver {verdict.upper()}: image {img or "unknown"}, device {device_driver_label(dev)}')
            ok = (args.allow_driver_change or confirm('gate-driver change', args)) and ok
    return ok


def preflight(args, con, tap, name, image, before):
    """guards() plus device activity, RTC layout and the last OTA record. Returns False to stop."""
    ok = guards(args, con, image, before, name)
    busy = []
    rc = ' '.join(con.command('rect-cal status', timeout=8))
    if m := re.search(r'state=(running|fitting)', rc):
        busy.append(f'rect-cal {m[1]}')
    if 'unsaved=1' in rc:
        busy.append('rect-cal result not saved yet (unsaved=1)')
    if 'running' in ' '.join(con.command('balance', timeout=8)):
        busy.append('top-balance run in progress')
    st = tap.status(time.monotonic() - 30)
    if st and 'SWEEP' in st['st']:
        busy.append('MPPT sweep in progress')
    print(f'{name}: now {fmt_status(st)}')
    for b in busy:
        print(f'⚠️  {b}: the reboot interrupts it')
    if busy:
        ok = confirm(', '.join(busy), args) and ok
    rtc_layout_check(name, before['version'], args.bin)
    prev = last_record(name)
    if prev:
        print(f"{name}: last OTA record {prev.get('ts')} {prev.get('version')} by "
              f"{prev.get('who') or prev.get('method')} ({prev.get('result', 'flashed')})")
    return ok


def attempt(args, con, tap, url, image, before):
    """Send `ota <url>` and wait for the device. Returns ('ok'|'failed'|'lost', text, bootinfo, t_send)."""
    cmd = f'ota {url}'
    print(f'> {cmd}')
    t_send = time.monotonic()
    outcome, lines, pct = run_ota(con, cmd, timeout=OTA_BUDGET, silence=args.silence,
                                  echo=lambda l: is_relevant(l) and print(f'  {args.cur}: {l}'))
    dt = time.monotonic() - t_send
    m = next((m for l in lines if (m := RE_DL_TOOK.search(l))), None)
    rate = f'{m[2]} KB/s (device)' if m else (
        f'~{os.path.getsize(args.bin) * max(pct, 0) / 100 / 1024 / dt:.1f} KB/s' if pct > 0 else '-')
    print(f'download: {outcome} after {dt:.0f} s, last progress {pct if pct >= 0 else "-"} %, {rate}')
    if any('Ota timeout' in l for l in lines):
        print('  the firmware OTA watchdog (300 s) gave up and restarts the device')
    # a device that is still downloading cannot answer (commands queue behind `ota`), so any
    # bootinfo reply means the OTA is over; poll until it answers or the reboot budget is spent
    budget = args.reboot_timeout + (0 if outcome in ('succeeded', 'failed') else
                                    max(0.0, OTA_BUDGET - dt))
    deadline = time.monotonic() + budget
    time.sleep(3 if outcome == 'succeeded' else 0)
    while time.monotonic() < deadline:
        info = parse_bootinfo(con.command('bootinfo', timeout=5))
        if info:
            verdict, text = judge(before, info, image['version'], time.monotonic() - t_send)
            return ('ok' if verdict == 'ok' else 'failed'), text, info, t_send
        time.sleep(3)
    return 'lost', f'no bootinfo reply within {budget:.0f} s after `ota` ({outcome})', None, t_send


def new_boot(lines):
    """The (t, line)s logged by the new boot after an `OTA Succeed`: they start at the first line
    whose ESP log time is below that line's, or after a 2 s silence (the reboot plus Wi-Fi/MQTT
    reconnect take longer). None without an `OTA Succeed` line or a boot."""
    i = next((k for k, (_, l) in enumerate(lines) if 'OTA Succeed' in l), None)
    if i is None or not (m := RE_LOGTS.match(lines[i][1])):
        return None
    ts0, prev = int(m[1]), lines[i][0]
    for k in range(i + 1, len(lines)):
        t, l = lines[k]
        m = RE_LOGTS.match(l)
        if (m and int(m[1]) < ts0) or t - prev >= 2.0:
            return lines[k:]
        prev = t
    return None


def boot_faults(lines, boot_t=None):
    """Fault lines of the new boot in `lines` ((t, line)s since `ota` was sent); without a boundary
    in the log, those received after `boot_t` (the boot time estimated from bootinfo)."""
    nb = new_boot(lines)
    if nb is None:
        if boot_t is None:
            return ['no new boot found in the log (unverified)']
        nb = [x for x in lines if x[0] > boot_t]
    return [l for _, l in nb if RE_FAULT.search(l)]


def verify(args, con, tap, image, t_ota):
    """Wait for rollback=VALID on the image's version, then watch the log. Returns (ok, text)."""
    deadline, info, boot_t = time.monotonic() + args.valid_timeout, None, None
    while time.monotonic() < deadline:
        info = parse_bootinfo(con.command('bootinfo', timeout=6)) or info
        if info and boot_t is None:
            boot_t = time.monotonic() - info['uptime']
        if info and info['version'] != image['version']:
            return False, f"running {info['version']}, not {image['version']} (rolled back?)"
        if info and ' '.join(info['built'].split()) != ' '.join(f"{image['date']} {image['time']}".split()):
            return False, f"running {info['version']} built {info['built']}, not this image's build"
        if info and info.get('rollback') == 'VALID':
            break
        time.sleep(5)
    else:
        return False, f"still {info.get('rollback') if info else 'silent'} after {args.valid_timeout:.0f} s"
    print(f"{args.cur}: rollback=VALID after uptime {info['uptime']} s")
    t0, up0 = time.monotonic(), info['uptime']
    if args.watch > 0:
        print(f'watching the log for {args.watch:.0f} s ...')
        time.sleep(args.watch)
    faults = boot_faults([x for x in tap if x[0] > t_ota], boot_t)
    end = parse_bootinfo(ask(con, 'bootinfo', lambda r: r if parse_bootinfo(r) else None, 30) or [])
    if not end or end['uptime'] + 5 < up0 + (time.monotonic() - t0):
        faults.append(f'device restarted during the watch (uptime {end and end["uptime"]} s)')
    for f in faults:
        print(f'  ❌ {f}')
    return not faults, (f'{len(faults)} fault line(s) after the OTA' if faults else
                        f'VALID, clean log since the new boot ({args.watch:.0f} s watch)')


def ota_device(args, name, image, server, gw, locks):
    """One device: 0 ok / current / dry-run passed, 1 failed or refused, 2 setup error."""
    args.cur = name
    print(f'\n━━ {name} ━━')
    tap = Tap()
    try:
        con = Console(ExactMqtt(args.host, port=args.mport, username=os.environ.get('MQTT_USER'),
                                password=os.environ.get('MQTT_PASS'), device=name,
                                discover_timeout=args.wait), on_line=tap)
    except Exception as e:
        print(f'MQTT connect failed: {type(e).__name__}: {e}')
        return 2
    try:
        hn = ask(con, 'hostname', parse_hostname, args.wait)
        if hn != name:
            print(f'hostname check failed: device answered {hn or "nothing"}, expected {name!r}')
            return 2
        before = ask(con, 'bootinfo', parse_bootinfo, args.wait)
        if not before:
            print('no bootinfo reply')
            return 2
        print(f"{name}: {before['version']}, up {before['uptime']} s, "
              f"slot {before.get('partition', '?')} ({before.get('rollback', '?')})")
        if before['project'] != image['project']:
            print(f"project mismatch: device {before['project']}, image {image['project']}")
            return 2
        if is_current(image['version'], before['version']) and not args.force:
            print(f"already at {image['version']} (-f to push anyway)")
            return 0
        ok = preflight(args, con, tap, name, image, before)
        checks = run_checks(con, args.check)
        for c, out in checks.items():
            print(f'check `{c}`:')
            for l in out:
                print(f'  {l}')
        if not ok:
            return 1
        rows = [bootinfo_row('before', before)]

        def table(*extra):
            print_table(rows + list(extra) + [dict(state='image', version=image['version'])],
                        [('state', 'STATE'), ('uptime', 'UPTIME', {'justify': 'right'}),
                         ('version', 'VERSION', {'no_wrap': True}), ('slot', 'SLOT'),
                         ('rollback', 'ROLLBACK')], title=f'{name}:')

        if args.dry_run:
            try:
                url = server.stage()
            except Exception as e:
                print(f'staging failed: {e}')
                return 2
            table()
            print(f'dry-run: would send `ota {url}` to {name}')
            return 0

        note = f"OTA {image['version']} by {args.who}"
        if not (locks.take(f'ota-{name}', note) and locks.take('ota-relay', f'{note} -> {name}')):
            return 1
        st_before = tap.status(time.monotonic() - 30)
        entry = dict(ts=datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
                     device=name, who=args.who, method='ota-mqtt', from_version=before['version'],
                     version=image['version'], bin_sha256=args.bin_sha, git=image_git_sha(image['version']),
                     dirty=args.dirt, result=None)
        result, text, after, t_ota = 'failed', '', None, 0.0
        tries = 1 + max(0, args.retries)
        for n in range(tries):
            try:
                url = server.stage()
            except Exception as e:
                print(f'staging failed: {e}')
                entry['result'] = 'staging failed'
                record(entry)
                return 2
            print(f'serving {url}')
            try:
                if gw:
                    gw.boost()
                result, text, after, t_ota = attempt(args, con, tap, url, image, before)
            finally:
                if gw:
                    gw.restore()
            print(f'attempt {n + 1}/{tries}: {result}: {text}')
            if result != 'failed' or n + 1 == tries:
                break
            if after:
                rb = after['uptime'] < time.monotonic() - t_ota + 5
                print(f"  device {'REBOOTED' if rb else 'did not reboot'} (uptime {after['uptime']} s, "
                      f"{after['version']}); image server: {server.status()}")
                before = after
            pause = args.retry_backoff[min(n, len(args.retry_backoff) - 1)]
            print(f'retrying in {pause:.0f} s ...')
            time.sleep(pause)
        locks.release('ota-relay')
        table(bootinfo_row('after', after))
        if result != 'ok':
            print(f'❌ {name}: {text}')
            entry['result'] = f'{result}: {text}'
            record(entry)
            return 1
        print(f'✅ {name}: {text}')
        archive_elf(name, 'ota-mqtt', image['version'], args.bin)
        if args.verify:
            vok, vtext = verify(args, con, tap, image, t_ota)
            print(f"{name}: power before {fmt_status(st_before)}")
            print(f"{name}: power after  {fmt_status(tap.status(time.monotonic() - 30))}")
            if args.check:
                print_checks(checks, run_checks(con, args.check))
            entry['result'] = ('ok, ' if vok else 'VERIFY FAILED, ') + vtext
            record(entry)
            print(f"{'✅' if vok else '❌'} {name}: {vtext}")
            return 0 if vok else 1
        entry['result'] = 'ok, not verified'
        record(entry)
        return 0
    finally:
        locks.release(f'ota-{name}', 'ota-relay')
        con.close()


def summary(names, results):
    return '\nsummary: ' + ', '.join(
        f"{n} {'not run' if n not in results else 'ok' if results[n] == 0 else 'FAILED'}" for n in names)


def main():
    load_env_file(etc_file('mqtt.env'))
    load_env_file(etc_file('nat.env'))
    ap = argparse.ArgumentParser(
        description='OTA devices through their MQTT consoles, one after the other, and verify each. '
                    'Build the fry/flat image with etc/fleet-build.sh [rev] (clean cached worktree '
                    '../fugu-fleet-wt, sdkconfig.d/fleet, IDF 6.0.3) and pass its .bin. '
                    'Broker and credentials come from etc/mqtt.env; an ssh tunnel (via '
                    '$OTA_RELAY_SSH / $NAT_TUNNEL_HOST / havan-ts) is opened when the broker is '
                    'not directly reachable, and the image is staged on that relay host '
                    '(etc/ota-via-relay.sh, $OTA_RELAY_IP default $MQTT_HOST) when this host '
                    'cannot serve it to the device LAN. Real OTAs hold the lock.sh locks '
                    'ota-<device> and ota-relay and are recorded in elf-archive/ota-owners.jsonl.',
        epilog='Exit status: 0 updated / already current / dry-run passed, 1 OTA or verification '
               'failed or refused, 2 setup error.')
    ap.add_argument('--name', required=True,
                    help='exact device hostname(s), comma-separated: updated in this order, '
                         'stopping at the first failure')
    ap.add_argument('--bin', default=os.path.join(REPO_ROOT, 'build', 'fugu-firmware.bin'),
                    help='app image (default build/fugu-firmware.bin; for fry/flat use etc/fleet-build.sh)')
    ap.add_argument('-n', '--dry-run', action='store_true',
                    help='connect, read bootinfo, run the checks and stage the image, '
                         'but do not send `ota`')
    ap.add_argument('-f', '--force', action='store_true',
                    help='update even when the device already runs the image version')
    ap.add_argument('--check', action='append', default=[], metavar='CMD',
                    help='read-only console command run before the OTA and after verification, '
                         'shown as a diff (repeatable)')
    ap.add_argument('--no-verify', dest='verify', action='store_false',
                    help='skip waiting for rollback=VALID and the log watch')
    ap.add_argument('--valid-timeout', type=float, default=90.0,
                    help='s to wait for rollback=VALID after the reboot (90)')
    ap.add_argument('--watch', type=float, default=120.0,
                    help='s to watch the log for faults once VALID (120)')
    ap.add_argument('--allow-dirty', action='store_true',
                    help=f'push a -dirty image with uncommitted firmware inputs to {"/".join(LIVE)}')
    ap.add_argument('--lock-wait', type=float, default=0.0,
                    help='s to wait for a held ota-<device>/ota-relay lock (default 0: refuse)')
    ap.add_argument('--relay', choices=('auto', 'always', 'never'), default='auto',
                    help='serve via the relay host: auto (when this host is not on the broker LAN)')
    ap.add_argument('--broker', default=os.environ.get('MQTT_HOST'), help='default $MQTT_HOST')
    ap.add_argument('--port', type=int, default=int(os.environ.get('MQTT_PORT', '1883')),
                    help='default $MQTT_PORT')
    ap.add_argument('--wait', type=float, default=90.0,
                    help='s to wait for the device to show up and answer before the OTA (90)')
    ap.add_argument('--retries', type=int, default=2, help='re-tries after a failed download (2)')
    ap.add_argument('--retry-backoff', type=lambda s: [float(x) for x in s.split(',')],
                    default=[30.0, 90.0], help='s to wait before each retry, comma-separated (30,90)')
    ap.add_argument('--silence', type=float, default=60.0,
                    help='s without any device line before a download counts as silent (60)')
    ap.add_argument('--reboot-timeout', type=float, default=120.0,
                    help='s to wait for bootinfo after the download ended (120)')
    ap.add_argument('--no-driver-check', action='store_true',
                    help='skip the gate-driver probe (a no-arg `pwm-freq`)')
    ap.add_argument('--yes', action='store_true',
                    help='accept the guard prompts (plant-sim image, unconfirmed running image, '
                         'rect-cal/sweep/balance in progress, ...)')
    ap.add_argument('--allow-driver-change', action='store_true',
                    help='push even if the gate driver differs or is unverified')
    ap.add_argument('--gw', default=default_gateway(),
                    help='NAT gateway whose Wi-Fi TX power is raised during the transfer '
                         '(default $OTA_GW, else the $NAT_TELNET host, else 192.168.1.231)')
    ap.add_argument('--gw-tx-dbm', type=int, default=20,
                    help='gateway Wi-Fi TX power during the transfer, dBm (20)')
    ap.add_argument('--no-gw-boost', action='store_true',
                    help='leave the gateway TX power alone')
    args = ap.parse_args()
    for s in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, lambda *_: sys.exit(143))  # run the finally blocks and atexit
    args.bin = os.path.abspath(args.bin)
    args.would_refuse = False
    sys.stdout.reconfigure(line_buffering=True)
    names = [n.strip() for n in args.name.split(',') if n.strip()]

    if not args.broker:
        print('no broker: set MQTT_HOST in etc/mqtt.env or pass --broker')
        return 2
    image = read_local_app_desc(args.bin)
    if not image:
        print(f'{args.bin}: missing or no app descriptor')
        return 2
    with open(args.bin, 'rb') as f:
        args.bin_sha = hashlib.sha256(f.read()).hexdigest()
    args.dirt = firmware_dirt(args.bin)
    args.who = owner()
    print(f"image: {image['project']} {image['version']} (built {image['date']} {image['time']}, "
          f"IDF {image['idf']}), sha256 {args.bin_sha[:16]}, {args.bin}")
    print(f'owner: {args.who}')

    ssh_dest = relay_ssh_dest()
    try:
        args.host, args.mport, tun = broker_endpoint(args.broker, args.port)
    except RuntimeError as e:
        print(e)
        return 2
    src = route_src(args.broker) if tun is None else None
    use_relay = args.relay == 'always' or (args.relay == 'auto' and not same_24(src, args.broker))
    if use_relay:
        env = dict(os.environ)
        env.setdefault('OTA_RELAY_IP', args.broker)
        env.setdefault('OTA_RELAY_SSH', ssh_dest)
        env.setdefault('OTA_RELAY_TTL', str(int(OTA_BUDGET + args.reboot_timeout + 300)))
        server = Relay(args.bin, env)
    else:
        server = LocalServer(args.bin, src)
    locks = Locks(args.lock_wait)
    atexit.register(server.stop)
    atexit.register(locks.release)
    gw = None if args.no_gw_boost or args.dry_run else GatewayTx(
        args.gw, ssh_dest, args.gw_tx_dbm, OTA_BUDGET + args.reboot_timeout + 60)

    results = {}
    for name in names:
        results[name] = ota_device(args, name, image, server, gw, locks)
        if results[name] and not args.dry_run:
            break
    if len(names) > 1:
        print(summary(names, results))
    return max(results.values(), default=2) if not args.would_refuse else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
