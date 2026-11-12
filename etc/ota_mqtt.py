#!/usr/bin/env python3
"""OTA one device through its MQTT console and verify the result.

    etc/ota_mqtt.py --name flat [--bin build/fugu-firmware.bin] [-n] [-f]

Steps: load the broker and credentials from etc/mqtt.env (relay/ssh settings from etc/nat.env; a
worktree falls back to the main checkout's files), open an ssh tunnel when the broker is not
directly reachable, lock onto the device by exact hostname, read `bootinfo`, serve the image
(locally when this host and the broker share a LAN, otherwise staged on the relay with
etc/ota-via-relay.sh), send `ota <url>`, watch the log for `OTA Succeed`, a failure line or
silence, then poll `bootinfo` until the device answers and check that it runs the image's version
from another slot with a reset uptime. A failed or silent download is retried once, re-staged.
`--dry-run` does all of this except sending `ota`. Run from the repo root with .venv/bin/python3.
"""
import argparse
import atexit
import os
import re
import socket
import subprocess
import sys
import time
import urllib.request

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from etc.fugu.console import Console  # noqa: E402
from etc.fugu.transport import MqttTransport  # noqa: E402
from etc.fugu_console import etc_file, load_env_file, run_ota  # noqa: E402
from ota_build_flags import (build_has_networking, build_is_plant_sim, build_gate_driver,  # noqa: E402
                             parse_device_gate_driver, gate_driver_verdict)

RE_BOOT_UPTIME = re.compile(r'uptime:\s*(\d+)\s*s,\s*app\s+(\S+)\s+v\S+\s+(\S+)\s+\(built (.+?),\s*IDF\s+(\S+)\)')
RE_BOOT_RUNNING = re.compile(r'running:\s*(\S+)\s+@\S+\s+\(\S+\)\s+rollback=(\S+)')
RE_HOSTNAME = re.compile(r'Hostname:\s*(\S+)')
RE_OTA_URL = re.compile(r'^ota\s+(https?://\S+)\s*$', re.M)
OTA_BUDGET = 330.0  # firmware OTA watchdog (300 s) + margin
REBOOT_SLACK = 5.0


def parse_bootinfo(lines):
    """{uptime, project, version, built, idf, partition, rollback} from a `bootinfo` reply, or None
    without the uptime line."""
    out = {}
    for l in lines:
        if m := RE_BOOT_UPTIME.search(l):
            out.update(uptime=int(m[1]), project=m[2], version=m[3], built=m[4], idf=m[5])
        elif m := RE_BOOT_RUNNING.search(l):
            out.update(partition=m[1], rollback=m[2])
    return out if 'uptime' in out else None


def judge(before, after, image_version, elapsed):
    """('ok'|'old-image'|'no-reboot', text) for a `bootinfo` read `elapsed` s after `ota` was sent."""
    rebooted = after['uptime'] < elapsed + REBOOT_SLACK
    slot = after.get('partition'), before.get('partition')
    if not rebooted:
        return 'no-reboot', f"still running {after['version']} (uptime {after['uptime']} s, no reboot)"
    if after['version'] != image_version:
        return 'old-image', f"rebooted into {after['version']}, not {image_version}"
    if None not in slot and slot[0] == slot[1]:
        return 'old-image', f'rebooted but still on slot {slot[0]} (rollback?)'
    return 'ok', f"running {after['version']} from {after.get('partition', '?')}"


def is_relevant(line):
    """Log lines worth echoing during an OTA (the status stream is noise here)."""
    return bool(re.search(r'\bota\b|OTA|Download|HTTP_|main: ota', line, re.I))


def tcp_ok(host, port, timeout=2.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


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


def free_port(start=None):
    for p in ([start] if start else []) + [0]:
        with socket.socket() as s:
            try:
                s.bind(('', p))
                return s.getsockname()[1]
            except OSError:
                continue


class Tunnel:
    """`ssh -N -L <free port>:host:port dest`, owned and closed by this process."""

    def __init__(self, dest, host, port, timeout=20.0):
        self.port = free_port()
        self.proc = subprocess.Popen(
            ['ssh', '-N', '-o', 'ExitOnForwardFailure=yes', '-o', 'BatchMode=yes',
             '-L', f'{self.port}:{host}:{port}', dest], stdin=subprocess.DEVNULL)
        atexit.register(self.close)
        deadline = time.monotonic() + timeout
        while not tcp_ok('127.0.0.1', self.port, 1.0):
            if self.proc.poll() is not None or time.monotonic() > deadline:
                self.close()
                raise RuntimeError(f'ssh tunnel to {host}:{port} via {dest} did not come up')
            time.sleep(0.3)

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


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
            atexit.register(self.stop)
            self.url = f'http://{self.addr}:{port}/{os.path.basename(self.bin)}'
            time.sleep(0.7)
        with urllib.request.urlopen(urllib.request.Request(self.url, method='HEAD'), timeout=5) as r:
            if int(r.headers.get('Content-Length', -1)) != os.path.getsize(self.bin):
                raise RuntimeError(f'{self.url}: size mismatch')
        return self.url

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()


class Relay:
    """Stage and serve the image on the relay host with etc/ota-via-relay.sh."""

    def __init__(self, bin_path, env):
        self.bin, self.env, self.staged = bin_path, env, False

    def _run(self, *a):
        return subprocess.run([os.path.join(REPO_ROOT, 'etc', 'ota-via-relay.sh'), *a],
                              env=self.env, capture_output=True, text=True, timeout=180)

    def stage(self):
        r = self._run(self.bin)
        self.staged = True
        m = RE_OTA_URL.search(r.stdout)
        if r.returncode or not m:
            raise RuntimeError(f'relay staging failed (exit {r.returncode}): '
                               f'{(r.stderr or r.stdout).strip()[-300:]}')
        return m[1]

    def stop(self):
        if self.staged:
            self._run('stop')
            self.staged = False


class ExactMqtt(MqttTransport):
    """MqttTransport that only accepts `pv/log/<device>` exactly, not a substring match."""

    def _on_message(self, client, userdata, msg):
        parts = msg.topic.split('/')
        if len(parts) == 3 and parts[2] == self.device:
            super()._on_message(client, userdata, msg)


def read_app_desc(bin_path):
    from ota import read_local_app_desc  # heavy import (rich, requests); only when run
    return read_local_app_desc(bin_path)


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
    if args.yes:
        print(f'{prompt}: accepted (--yes)')
        return True
    if not sys.stdin.isatty():
        print(f'aborting: {prompt} (refused non-interactively)')
        return False
    return input(f'{prompt} — proceed? [y/N] ').strip().lower() in ('y', 'yes')


def guards(args, con, image, before):
    """ota.py's image safety checks plus the device's rollback state; False aborts."""
    ok = True
    if before.get('rollback') == 'PENDING_VERIFY':
        print('⚠️  the running image is not confirmed yet (PENDING_VERIFY): this OTA overwrites the '
              'other slot, which holds the last confirmed image (the rollback target)')
        ok = ok and confirm('unconfirmed running image', args)
    if build_has_networking(args.bin) is False:
        print('⚠️  image has networking DISABLED (CONFIG_FUGU_WITH_NETW=n): the device loses Wi-Fi')
        ok = ok and confirm('no-network image', args)
    if build_is_plant_sim(args.bin):
        print('⛔  PLANT-SIMULATION image (CONFIG_FUGU_WITH_VCONV=y): a real converter makes 0 W')
        ok = ok and confirm('plant-sim image', args)
    if 'dirty' in image['version']:
        print(f"⚠️  uncommitted build {image['version']}")
        ok = ok and confirm('dirty image', args)
    if not args.no_driver_check:
        r = con.command('pwm-freq', timeout=6)
        dev = parse_device_gate_driver(r, r.ok, r.rejected)
        img = build_gate_driver(args.bin)
        verdict = gate_driver_verdict(img, dev)
        if verdict != 'ok':
            print(f'⚠️  gate driver {verdict.upper()}: image {img or "unknown"}, device {dev or "unknown"}')
            ok = ok and (args.allow_driver_change or confirm('gate-driver change', args))
    return ok


def attempt(con, url, image_version, before):
    """Send `ota <url>` and verify. Returns ('ok'|'failed'|'lost', text, after_bootinfo)."""
    cmd = f'ota {url}'
    print(f'> {cmd}')
    t_send = time.monotonic()
    outcome, _, pct = run_ota(con, cmd, timeout=OTA_BUDGET, silence=args.silence,
                              echo=lambda l: is_relevant(l) and print(f'  {args.name}: {l}'))
    print(f'download: {outcome} (last progress {pct if pct >= 0 else "-"} %)')
    # a device that is still downloading cannot answer (commands queue behind `ota`), so any
    # bootinfo reply means the OTA is over; poll until it answers or the reboot budget is spent
    budget = args.reboot_timeout + (0 if outcome in ('succeeded', 'failed') else
                                    max(0.0, OTA_BUDGET - (time.monotonic() - t_send)))
    deadline = time.monotonic() + budget
    time.sleep(3 if outcome == 'succeeded' else 0)
    while time.monotonic() < deadline:
        r = con.command('bootinfo', timeout=5)
        info = parse_bootinfo(r)
        if info:
            verdict, text = judge(before, info, image_version, time.monotonic() - t_send)
            return ('ok' if verdict == 'ok' else 'failed'), text, info
        time.sleep(3)
    return 'lost', f'no bootinfo reply within {budget:.0f} s after `ota` ({outcome})', None


def row(label, info):
    if not info:
        return dict(state=label, uptime='-', version='-', slot='-', rollback='-')
    return dict(state=label, uptime=f"{info['uptime']} s" if 'uptime' in info else '',
                version=info.get('version', ''), slot=info.get('partition', ''),
                rollback=info.get('rollback', ''))


def main():
    global args
    load_env_file(etc_file('mqtt.env'))
    load_env_file(etc_file('nat.env'))
    ap = argparse.ArgumentParser(
        description='OTA one device through its MQTT console and verify the running version. '
                    'Broker and credentials come from etc/mqtt.env; an ssh tunnel (via '
                    '$OTA_RELAY_SSH / $NAT_TUNNEL_HOST / havan-ts) is opened when the broker is '
                    'not directly reachable, and the image is staged on that relay host '
                    '(etc/ota-via-relay.sh, $OTA_RELAY_IP default $MQTT_HOST) when this host '
                    'cannot serve it to the device LAN.',
        epilog='Exit status: 0 updated / already current / dry-run passed, 1 OTA failed, '
               '2 setup error.')
    ap.add_argument('--name', required=True, help='exact device hostname (pv/log/<name>)')
    ap.add_argument('--bin', default=os.path.join(REPO_ROOT, 'build', 'fugu-firmware.bin'),
                    help='app image (default build/fugu-firmware.bin)')
    ap.add_argument('-n', '--dry-run', action='store_true',
                    help='connect, read bootinfo, run the checks and stage the image, '
                         'but do not send `ota`')
    ap.add_argument('-f', '--force', action='store_true',
                    help='update even when the device already runs the image version')
    ap.add_argument('--relay', choices=('auto', 'always', 'never'), default='auto',
                    help='serve via the relay host: auto (when this host is not on the broker LAN)')
    ap.add_argument('--broker', default=os.environ.get('MQTT_HOST'), help='default $MQTT_HOST')
    ap.add_argument('--port', type=int, default=int(os.environ.get('MQTT_PORT', '1883')),
                    help='default $MQTT_PORT')
    ap.add_argument('--wait', type=float, default=90.0,
                    help='s to wait for the device to show up and answer before the OTA (90)')
    ap.add_argument('--retries', type=int, default=1, help='re-tries after a failed download (1)')
    ap.add_argument('--silence', type=float, default=60.0,
                    help='s without any device line before a download counts as silent (60)')
    ap.add_argument('--reboot-timeout', type=float, default=120.0,
                    help='s to wait for bootinfo after the download ended (120)')
    ap.add_argument('--no-driver-check', action='store_true',
                    help='skip the gate-driver probe (a no-arg `pwm-freq`)')
    ap.add_argument('--yes', action='store_true',
                    help='accept the guard prompts (plant-sim image, unconfirmed running image, ...)')
    ap.add_argument('--allow-driver-change', action='store_true',
                    help='push even if the gate driver differs or is unverified')
    args = ap.parse_args()
    args.bin = os.path.abspath(args.bin)
    sys.stdout.reconfigure(line_buffering=True)

    if not args.broker:
        print('no broker: set MQTT_HOST in etc/mqtt.env or pass --broker')
        return 2
    image = read_app_desc(args.bin)
    if not image:
        print(f'{args.bin}: missing or no app descriptor')
        return 2
    print(f"image: {image['project']} {image['version']} (built {image['date']} {image['time']}, "
          f"IDF {image['idf']})")

    ssh_dest = (os.environ.get('OTA_RELAY_SSH') or os.environ.get('NAT_TUNNEL_HOST')
                or 'havan-ts')
    host, port = args.broker, args.port
    direct = tcp_ok(host, port)
    if not direct:
        print(f'broker {host}:{port} not reachable, tunnelling via {ssh_dest}')
        try:
            tun = Tunnel(ssh_dest, host, port)
        except RuntimeError as e:
            print(e)
            return 2
        host, port = '127.0.0.1', tun.port

    src = route_src(args.broker) if direct else None
    use_relay = args.relay == 'always' or (args.relay == 'auto' and not same_24(src, args.broker))
    if use_relay:
        env = dict(os.environ)
        env.setdefault('OTA_RELAY_IP', args.broker)
        env.setdefault('OTA_RELAY_SSH', ssh_dest)
        env.setdefault('OTA_RELAY_TTL', str(int(OTA_BUDGET + args.reboot_timeout + 300)))
        server = Relay(args.bin, env)
    else:
        server = LocalServer(args.bin, src)
    atexit.register(server.stop)

    try:
        con = Console(ExactMqtt(host, port=port, username=os.environ.get('MQTT_USER'),
                                password=os.environ.get('MQTT_PASS'), device=args.name,
                                discover_timeout=args.wait))
    except Exception as e:
        print(f'MQTT connect failed: {type(e).__name__}: {e}')
        return 2
    try:
        hn = ask(con, 'hostname', parse_hostname, args.wait)
        if hn != args.name:
            print(f'hostname check failed: device answered {hn or "nothing"}, expected {args.name!r}')
            return 2
        before = ask(con, 'bootinfo', parse_bootinfo, args.wait)
        if not before:
            print('no bootinfo reply')
            return 2
        print(f"{args.name}: {before['version']}, up {before['uptime']} s, "
              f"slot {before.get('partition', '?')} ({before.get('rollback', '?')})")
        if before['project'] != image['project']:
            print(f"project mismatch: device {before['project']}, image {image['project']}")
            return 2
        if before['version'] == image['version'] and not args.force:
            print(f"already at {image['version']} (-f to push anyway)")
            return 0
        if not guards(args, con, image, before):
            return 1

        rows = [row('before', before)]

        def table(*extra):
            from ota import print_table
            print_table(rows + list(extra) + [dict(state='image', version=image['version'])],
                        [('state', 'STATE'), ('uptime', 'UPTIME', {'justify': 'right'}),
                         ('version', 'VERSION', {'no_wrap': True}), ('slot', 'SLOT'),
                         ('rollback', 'ROLLBACK')], title=f'{args.name}:')

        result, text, after = 'failed', '', None
        for n in range(1 + max(0, args.retries)):
            try:
                url = server.stage()
            except Exception as e:
                print(f'staging failed: {e}')
                return 2
            print(f'serving {url}' + (' (relay)' if use_relay else ''))
            if args.dry_run:
                table()
                print(f'dry-run: would send `ota {url}` to {args.name}')
                return 0
            result, text, after = attempt(con, url, image['version'], before)
            print(f"attempt {n + 1}: {result}: {text}")
            if result != 'failed':
                break
            before = after or before
        table(row('after', after))
        if result == 'ok':
            print(f'✅ {args.name}: {text}')
            try:
                sys.path.insert(0, os.path.join(REPO_ROOT, 'etc', 'idf-devtools'))
                import elf_archive
                elf_archive.archive(args.name, method='ota-mqtt', version=image['version'],
                                    bin=args.bin, build_dir=os.path.dirname(args.bin))
            except Exception as e:
                print(f'WARN: ELF archive failed: {e}')
            return 0
        print(f'❌ {args.name}: {text}')
        return 1
    finally:
        con.close()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
