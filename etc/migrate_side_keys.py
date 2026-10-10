#!/usr/bin/env python3
"""Migrate sensor.conf and limits.conf from role keys to HV/LV side keys.

Firmware from the sensor-hv-lv change on reads only side keys (src/conv_side.h). A sensor.conf or
limits.conf that still has a role key (vin_ch, iout_factor, vin_max, ...) fails setup at boot, so
migrate every config BEFORE it meets the new firmware.

Mapping, by converter.conf topo (absent = buck, as the firmware reads it):
  buck:  vin_/iin_ -> hv_v_/hv_i_   vout_/iout_ -> lv_v_/lv_i_   vin_max -> hv_max  iout_max -> lv_i_max ...
  boost: vin_/iin_ -> lv_v_/lv_i_   vout_/iout_ -> hv_v_/hv_i_   vin_max -> lv_max  iout_max -> hv_i_max ...
Side current factors are positive HV->LV, so a boost's current *_factor changes sign. Every other
key (ntc_*, vin_min, iout_short, p_max, charger.conf vout_max, ...) is left alone.

Offline, rewrite conf directories in place (comments, order and inline comments are kept; already
migrated files are left unchanged; a role key next to its own side key is refused):
  etc/migrate_side_keys.py dir config/lab/buck_bench [more dirs...] [--topo buck|boost] [--check]

Live board, still on the old firmware: capture the configs, then print the console commands:
  get-config converter.conf / get-config sensor.conf / get-config limits.conf  -> save to board.log
  etc/migrate_side_keys.py live board.log [--topo buck|boost] [--rollback]
Run the printed set-config/del-config lines over BLE or telnet, then OTA straight away: the old
firmware keeps its sensors only until the next reboot. --rollback prints the inverse sequence.
"""
import argparse
import re
import sys
from pathlib import Path

ROLE_CHANNELS = ("vin", "vout", "iin", "iout")
SENSOR_SUFFIXES = ("adc", "ch", "rh", "rl", "factor", "midpoint", "filt_len")  # conv_side.h SensorKeySuffixes
LIMIT_KEYS = ("vin_max", "vout_max", "iin_max", "iout_max")


class MigrationError(Exception):
    pass


def side_of(input_role, boost):
    return "hv" if input_role != boost else "lv"


def side_key(file, key, boost):
    """Side key replacing role key `key` in `file`, or None if `key` is not a mapped role key."""
    if file == "sensor.conf":
        role, _, sfx = key.partition("_")
        if role in ROLE_CHANNELS and sfx in SENSOR_SUFFIXES:
            return f"{side_of(role[1] == 'i', boost)}_{role[0]}_{sfx}"
    elif file == "limits.conf" and key in LIMIT_KEYS:
        return side_of(key[1] == "i", boost) + ("_i_max" if key[0] == "i" else "_max")
    return None


def flips_sign(file, key, boost):
    return boost and file == "sensor.conf" and key in ("iin_factor", "iout_factor")


def negate(value):
    v = value.strip()
    float(v)  # ValueError on a non-number
    if v.startswith("-"):
        return v[1:]
    return "-" + (v[1:] if v.startswith("+") else v)


def topo_of(converter_kv, forced):
    """boost? from converter.conf's key/values (None = file absent) and an optional --topo."""
    topo = None if converter_kv is None else converter_kv.get("topo", "buck")
    if topo is None and forced is None:
        raise MigrationError("converter.conf not given: pass --topo buck|boost")
    if topo is not None and forced is not None and topo != forced:
        raise MigrationError(f"converter.conf topo={topo} contradicts --topo {forced}")
    topo = topo or forced
    if topo not in ("buck", "boost"):
        raise MigrationError(f"converter.conf: topo must be buck|boost, got '{topo}'")
    return topo == "boost"


def plan(file, kv, boost):
    """[(role_key, side_key, old_value, new_value)] for the role keys in `kv` (key -> value)."""
    out = []
    for key, val in kv.items():
        new = side_key(file, key, boost)
        if new is None:
            continue
        if new in kv:
            raise MigrationError(f"{file}: has both {key} and {new}; delete the one that is wrong, then rerun")
        try:
            nval = negate(val) if flips_sign(file, key, boost) else val
        except ValueError:
            raise MigrationError(f"{file}: {key}={val!r} is not a number, cannot flip its sign for topo=boost")
        out.append((key, new, val, nval))
    return out


# ---- offline: conf directories

LINE = re.compile(r"^(?P<pre>\s*(?:#\s*)?)(?P<key>[^\s=#]+)(?P<eq>\s*=\s*)(?P<val>[^#]*?)(?P<post>\s*(?:#.*)?)$")


def parse_conf(text):
    """key -> value as ConfFile reads it: '#' starts a comment, the last duplicate wins."""
    kv = {}
    for line in text.splitlines():
        code = line.split("#", 1)[0].strip()
        if "=" in code:
            k, v = code.split("=", 1)
            kv[k.strip()] = v.strip()
    return kv


def rewrite(file, text, boost):
    """(new text, plan) with every role key line renamed, commented-out lines included."""
    changes = plan(file, parse_conf(text), boost)
    out = []
    for line in text.splitlines(keepends=True):
        body = line.rstrip("\r\n")
        m = LINE.match(body)
        new = m and side_key(file, m["key"], boost)
        if new:
            val = m["val"]
            if flips_sign(file, m["key"], boost):
                try:
                    val = negate(val)
                except ValueError:
                    if not m["pre"].lstrip().startswith("#"):
                        raise  # plan() has already refused an active one
            line = m["pre"] + new + m["eq"] + val + m["post"] + line[len(body):]
        out.append(line)
    return "".join(out), changes


def conf_dir(path):
    p = Path(path)
    return p / "conf" if not (p / "sensor.conf").exists() and (p / "conf").is_dir() else p


def migrate_dir(path, forced_topo=None, write=True):
    """Rewrite sensor.conf and limits.conf of one conf dir. Returns [(file, role, side, old, new)]."""
    d = conf_dir(path)
    conv = d / "converter.conf"
    conv_kv = parse_conf(conv.read_text()) if conv.exists() else None
    if conv_kv is None and forced_topo is None:
        conv_kv = {}  # no converter.conf on the board either: the firmware runs it as a buck
    boost = topo_of(conv_kv, forced_topo)
    done, texts = [], {}
    for file in ("sensor.conf", "limits.conf"):
        f = d / file
        if not f.exists():
            continue
        text, changes = rewrite(file, f.read_text(), boost)
        texts[f] = text
        done += [(file,) + c for c in changes]
    if write:
        for f, text in texts.items():
            if f.read_text() != text:
                f.write_text(text)
    return done


# ---- live: get-config output -> console commands

CONF_LINE = re.compile(r"Conf '(?:[^':]*/)?(?P<file>[\w.-]+\.conf):(?P<key>[^']*)' = '(?P<val>.*)'")
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def parse_get_config(text):
    files = {}
    for line in text.splitlines():
        m = CONF_LINE.search(ANSI.sub("", line).rstrip())
        if m:
            files.setdefault(m["file"], {})[m["key"]] = m["val"]
    return files


def live_commands(text, forced_topo=None, rollback=False):
    files = parse_get_config(text)
    if "sensor.conf" not in files and "limits.conf" not in files:
        raise MigrationError("no sensor.conf or limits.conf lines in the get-config output")
    boost = topo_of(files.get("converter.conf"), forced_topo)
    sets, dels = [], []
    for file in ("sensor.conf", "limits.conf"):
        for role, side, old, new in plan(file, files.get(file, {}), boost):
            if rollback:
                sets.append(f"set-config {file} {role} {old}")
                dels.append(f"del-config {file} {side}")
            else:
                sets.append(f"set-config {file} {side} {new}")
                dels.append(f"del-config {file} {role}")
    # all sets first: an interrupted run leaves the old keys, which the old firmware still reads
    return sets + dels


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    a = sub.add_parser("dir", help="rewrite conf directories in place")
    a.add_argument("dirs", nargs="+", help="conf dir, or a profile dir with conf/")
    a.add_argument("--topo", choices=("buck", "boost"), help="topo when converter.conf is absent (default buck, "
                   "as the firmware); else must match it")
    a.add_argument("--check", action="store_true", help="write nothing, exit 1 if anything needs migrating")
    b = sub.add_parser("live", help="print console commands from a get-config capture")
    b.add_argument("log", nargs="?", default="-", help="get-config output (default stdin)")
    b.add_argument("--topo", choices=("buck", "boost"), help="needed when converter.conf was not captured")
    b.add_argument("--rollback", action="store_true", help="print the inverse sequence (side -> role keys)")
    args = ap.parse_args(argv)

    try:
        if args.mode == "live":
            text = sys.stdin.read() if args.log == "-" else Path(args.log).read_text()
            cmds = live_commands(text, args.topo, args.rollback)
            print("\n".join(cmds) if cmds else "# nothing to migrate", flush=True)
            return 0
        pending = False
        for d in args.dirs:
            changes = migrate_dir(d, args.topo, write=not args.check)
            for file, role, side, old, new in changes:
                flip = f" ({old} -> {new})" if old != new else ""
                print(f"{d}: {file} {role} -> {side}{flip}")
            pending |= bool(changes)
        return 1 if args.check and pending else 0
    except MigrationError as e:
        print(f"migrate_side_keys: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
