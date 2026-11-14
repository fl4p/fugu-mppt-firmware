"""Steps shared by the OTA clients (etc/ota.py, etc/ota_mqtt.py, etc/ota_ble.py): local image
inspection, build-flag safety checks, device reply parsing, the y/N gate, verification after the
update, the result table and ELF archiving. Stdlib only at import time (rich and elf_archive load on
first use), so it stays host-unit-testable (test_ota_common.py, test_ota_build_flags.py) without
the clients' network/argparse side effects."""
import datetime
import hashlib
import json
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
APP_DESC_MAGIC = 0xABCD5432


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


def read_local_bootloader(bin_path):
    """Describe a bootloader .bin like the device's `bootinfo` line; None if it is not a valid one."""
    try:
        with open(bin_path, 'rb') as f:
            d = f.read()
    except FileNotFoundError:
        return None
    if len(d) < 32 + 80 + 32 or d[0] != 0xE9 or not d[23]:
        return None  # no magic or no appended SHA-256
    if hashlib.sha256(d[:-32]).digest() != d[-32:]:
        return None
    desc = d[32:112]
    if desc[0] == 0x50:
        s = lambda a, n: desc[a:a + n].split(b'\x00', 1)[0].decode('utf-8', 'replace')
        text = f"idf {s(8, 32)} v{struct.unpack('<I', desc[4:8])[0]}, {s(40, 24)}"
    else:
        text = 'no descriptor (old IDF)'
    sha = d[-32:-24].hex()
    return dict(text=f'{text}, len {len(d)}, sha {sha}', sha=sha)


def _build_flag(bin_path, key):
    """Value of the sdkconfig key (CONFIG_<key> without the CONFIG_ prefix) from the sdkconfig.json
    next to bin_path's build dir, as bool; None if the config can't be read/parsed. None means
    'unknown' — callers must treat that as not-safe, never as the flag being off."""
    cfg = os.path.join(os.path.dirname(bin_path), 'config', 'sdkconfig.json')
    try:
        with open(cfg) as f:
            return bool(json.load(f).get(key))
    except (OSError, ValueError):
        return None


def build_has_networking(bin_path):
    """True/False if the build that produced bin_path has CONFIG_FUGU_WITH_NETW, None if unknown."""
    return _build_flag(bin_path, 'FUGU_WITH_NETW')


def build_is_plant_sim(bin_path):
    """True/False if the build that produced bin_path has CONFIG_FUGU_WITH_VCONV, None if unknown.
    With VCONV the real LEDC PwmDriver is swapped for a plant simulator (src/buck.h), so the
    half-bridge never switches: flashing it to a real converter yields 0W while the device still
    samples real sensors and looks alive."""
    return _build_flag(bin_path, 'FUGU_WITH_VCONV')


def build_gate_driver(bin_path):
    """Gate driver the build that produced bin_path compiles in: 'mcpwm', 'ledc' or 'vconv' (Kconfig
    choice FUGU_GATE_DRIVER), None if the config can't be read or names none of them."""
    for key, name in (('FUGU_WITH_MCPWM', 'mcpwm'), ('FUGU_GATE_LEDC', 'ledc'), ('FUGU_WITH_VCONV', 'vconv')):
        v = _build_flag(bin_path, key)
        if v is None:
            return None
        if v:
            return name
    return None


def image_is_dirty(version):
    """True for a git-describe --dirty version: built from a tree with uncommitted changes."""
    return bool(version) and 'dirty' in version


RE_PWM_FREQ_MCPWM = re.compile(r'pwm-freq [0-9.]+ Hz period_ticks=\d+')


def parse_device_gate_driver(lines, ok, rejected):
    """Running gate driver from a no-arg `pwm-freq` reply: 'mcpwm' (reports its timer period),
    'other' (LEDC or VCONV, which answer 'n/a'), None if the reply proves neither (timeout, unknown
    command on old firmware, garbled) — callers must treat None as unverified, never as a match."""
    if ok and any(RE_PWM_FREQ_MCPWM.search(l) for l in lines):
        return 'mcpwm'
    if rejected and any('pwm-freq: n/a' in l for l in lines):
        return 'other'
    return None


def gate_driver_verdict(image, device):
    """'ok', 'mismatch' or 'unverified' for pushing an image with gate driver `image` (see
    build_gate_driver) to a device running `device` (see parse_device_gate_driver)."""
    if image is None or device is None:
        return 'unverified'
    if (image == 'mcpwm') == (device == 'mcpwm'):
        return 'ok'
    return 'mismatch'


def device_driver_label(device):
    """Display name of a parse_device_gate_driver() result."""
    return {'mcpwm': 'mcpwm', 'other': 'ledc/vconv'}.get(device, 'unknown')


RE_APP_LINE = re.compile(r'App:\s+(\S+)\s+v\S+\s+(\S+)\s+\(built (.+),\s+IDF\s+(\S+)\)')
RE_UPTIME = re.compile(r'Uptime:\s*(\d+)')


def parse_device_app(lines):
    """{project, version, built, idf} from the App: line of an `uptime` reply, or None."""
    for l in lines:
        m = RE_APP_LINE.search(l)
        if m:
            return dict(project=m.group(1), version=m.group(2),
                        built=m.group(3), idf=m.group(4))
    return None


def parse_device_version(lines):
    """Version from the App: line of an `uptime` reply, or None."""
    app = parse_device_app(lines)
    return app['version'] if app else None


def parse_uptime(lines):
    """Seconds from the Uptime: line of an `uptime` reply, or None."""
    for l in lines:
        if m := RE_UPTIME.search(l):
            return int(m.group(1))
    return None


RE_BOOT_UPTIME = re.compile(r'uptime:\s*(\d+)\s*s,\s*app\s+(\S+)\s+v\S+\s+(\S+)\s+\(built (.+?),\s*IDF\s+(\S+)\)')
RE_BOOT_RUNNING = re.compile(r'running:\s*(\S+)\s+@\S+\s+\(\S+\)\s+rollback=(\S+)')
RE_BOOTLOADER_LINE = re.compile(r'bootloader:\s+(.*?)\s*$')


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


def parse_bootloader_line(lines):
    """Text of the `bootloader:` line of a `bootinfo` reply, or None."""
    for l in lines:
        if m := RE_BOOTLOADER_LINE.search(l):
            return m.group(1)
    return None


def is_pending_verify(bootinfo):
    """The running image is not confirmed yet; the other slot holds the rollback target."""
    return bootinfo.get('rollback') == 'PENDING_VERIFY'


def is_current(image_version, device_version):
    """The device already runs the image's version (nothing to update without --force)."""
    return bool(image_version) and device_version == image_version


def confirm(prompt, refuse, accept=None, aborted=None):
    """y/N gate. `accept` (an override flag's message) approves without asking. Without a TTY,
    prints `refuse` and refuses. Otherwise asks `prompt`; on anything but y/yes prints `aborted`."""
    if accept:
        print(accept)
        return True
    if not sys.stdin.isatty():
        print(refuse)
        return False
    if input(prompt).strip().lower() in ('y', 'yes'):
        return True
    if aborted:
        print(aborted)
    return False


REBOOT_SLACK = 5.0


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


def verify_versions(names, after, expected):
    """Print a verify: block comparing each device's version (after = {name: app or None}) with
    `expected`; return {name: matched}."""
    print('verify:')
    out = {}
    for name in names:
        dev = after.get(name)
        got = dev.get('version') if dev else None
        out[name] = got == expected
        if out[name]:
            print(f'  {name}: ✅ {got}')
        else:
            print(f'  {name}: ❌ got {got!r}, expected {expected!r}')
    return out


def fmt_uptime(seconds):
    """Seconds → human-readable hours (e.g. 6789 → '1.89h'); None → '-'."""
    if seconds is None:
        return '-'
    td = datetime.timedelta(seconds=int(seconds))
    return f'{td.total_seconds() / 3600:.2f}h'


def bootinfo_row(label, info):
    """Table row (state, uptime, version, slot, rollback) of a parse_bootinfo() result."""
    if not info:
        return dict(state=label, uptime='-', version='-', slot='-', rollback='-')
    return dict(state=label, uptime=f"{info['uptime']} s" if 'uptime' in info else '',
                version=info.get('version', ''), slot=info.get('partition', ''),
                rollback=info.get('rollback', ''))


_rich_console = None


def print_table(rows, cols, title=None):
    """rows: list of dicts. cols: list of (key, header[, kw]) tuples;
    kw is per-column kwargs for Table.add_column (no_wrap, min_width, etc.)."""
    global _rich_console
    from rich.console import Console as RichConsole
    from rich.table import Table
    if _rich_console is None:
        # CLion / PyCharm Run consoles report as a TTY but don't render ANSI escapes, and rich
        # would squash the table to 80 cols there: no color, fixed wide width.
        jb = bool(os.environ.get('PYCHARM_HOSTED'))
        _rich_console = RichConsole(color_system=None if jb else 'auto',
                                    force_terminal=False if jb else None,
                                    highlight=not jb, width=200 if jb else None)
    t = Table(title=title, show_header=True, header_style='bold')
    for col in cols:
        k, h, kw = (*col, {})[:3] if len(col) == 2 else col
        t.add_column(h, **kw)
    for r in rows:
        t.add_row(*(str(r.get(k, '')) for k, *_ in cols))
    _rich_console.print(t)


def require_elf_archive():
    """The vendored elf_archive module (etc/idf-devtools submodule); ImportError if it is missing."""
    path = os.path.join(HERE, 'idf-devtools')
    if path not in sys.path:
        sys.path.insert(0, path)
    import elf_archive
    return elf_archive


def archive_elf(device, method, version, bin_path=None, indent=''):
    """Archive the build ELF of a verified update for later coredump symbolication; a failure only
    warns. Without bin_path, elf_archive picks the image from ./build."""
    try:
        kw = dict(bin=bin_path, build_dir=os.path.dirname(os.path.abspath(bin_path))) if bin_path else {}
        require_elf_archive().archive(device, method=method, version=version, **kw)
    except Exception as e:
        print(f'{indent}WARN: ELF archive failed for {device}: {e}')
