#!/usr/bin/env python3
"""Migrate sensor.conf and limits.conf from role keys to HV/LV side keys, move the INA226
measurement keys (ina22x_resistor, ina22x_conv_time_us, ina22x_ov) from board.conf to sensor.conf,
and rename the board.conf backflow switch pins panel_en/panel_sd to bflow_en/bflow_sd.

Firmware from the sensor-hv-lv change on reads only side keys (src/conv_side.h). A sensor.conf or
limits.conf that still has a role key (vin_ch, iout_factor, vin_max, ...) fails setup at boot, so
migrate every config BEFORE it meets the new firmware.

Mapping, by converter.conf topo (absent = buck, as the firmware reads it):
  buck:  vin_/iin_ -> hv_v_/hv_i_   vout_/iout_ -> lv_v_/lv_i_   vin_max -> hv_max  iout_max -> lv_i_max ...
  boost: vin_/iin_ -> lv_v_/lv_i_   vout_/iout_ -> hv_v_/hv_i_   vin_max -> lv_max  iout_max -> hv_i_max ...
Side current factors are positive HV->LV, so a boost's current *_factor changes sign. Every other
key (ntc_*, vin_min, iout_short, p_max, charger.conf vout_max, ...) is left alone. Firmware that reads
the INA226 keys from sensor.conf fails setup while board.conf still has one of them, unless
sensor.conf holds the same value (then it only warns); earlier firmware reads them only from
board.conf. Firmware that reads bflow_en/bflow_sd fails setup on a board.conf with panel_en/panel_sd
but not its replacement; with both it warns and uses the new key.

Offline, rewrite conf directories in place (comments, order and inline comments are kept; already
migrated files are left unchanged; a role key next to its own side key is refused; moved INA226 lines
are appended to sensor.conf with their comments; panel_en/panel_sd lines are renamed in place, or
dropped when the new key already holds the same value):
  etc/migrate_side_keys.py dir config/lab/buck_bench [more dirs...] [--topo buck|boost] [--check]

Live board, still on the old firmware: capture the configs, then print the console commands:
  get-config converter.conf / get-config sensor.conf / get-config limits.conf / get-config board.conf
  -> save to board.log
  etc/migrate_side_keys.py live board.log [--topo buck|boost] [--rollback]
Run the printed set-config/del-config lines over BLE or telnet, then OTA straight away: after role
key deletes the old firmware keeps its sensors only until the next reboot (no config suits both
firmwares there). The INA226 keys and panel_en/panel_sd get two steps: step 1, before the OTA, only
adds the sensor.conf INA226 copies and the bflow_* keys, which the old firmware ignores; step 2,
once the board runs the new image and bootinfo shows it VALID, deletes the board.conf INA226 keys
and panel_*. Between the steps either firmware boots. An INA226 key whose sensor.conf value differs
from board.conf is flagged (the new firmware would fail setup); step 1 overwrites it with the
board.conf value, the one the old firmware uses. --rollback prints the inverse sequence for the
same capture.
"""
import argparse
import re
import sys
from pathlib import Path

ROLE_CHANNELS = ("vin", "vout", "iin", "iout")
SENSOR_SUFFIXES = ("adc", "ch", "rh", "rl", "factor", "midpoint", "filt_len", "gain", "offset")  # conv_side.h SensorKeySuffixes
LIMIT_KEYS = ("vin_max", "vout_max", "iin_max", "iout_max")
MOVED_KEYS = ("ina22x_resistor", "ina22x_conv_time_us", "ina22x_ov")  # board.conf -> sensor.conf
RENAMED_KEYS = {"panel_en": "bflow_en", "panel_sd": "bflow_sd"}  # board.conf, src/pwm/backflow.h


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


def same_value(a, b):
    """Equal as the firmware compares the INA226 keys (getFloat), e.g. 0.5e-3 == 0.0005."""
    try:
        return float(a) == float(b)
    except ValueError:
        return a == b


def moves(board_kv, sensor_kv):
    """[(key, board value, sensor value or None)] for the MOVED_KEYS in board.conf."""
    return [(k, board_kv[k], sensor_kv.get(k)) for k in MOVED_KEYS if k in board_kv]


def differ_msg(k, bv, sv):
    return f"board.conf {k}={bv} and sensor.conf {k}={sv} differ"


def move_lines(board_text, sensor_text):
    """(board text, sensor text, [(key, value)]) with the MOVED_KEYS lines, commented-out ones too,
    cut from board.conf and appended to sensor.conf unless sensor.conf already has the key."""
    active = parse_conf(sensor_text)
    changes = []
    for k, bv, sv in moves(parse_conf(board_text), active):
        if sv is not None and not same_value(bv, sv):
            raise MigrationError(differ_msg(k, bv, sv) + "; delete the wrong one, then rerun")
        changes.append((k, bv))
    listed = {m["key"] for m in map(LINE.match, sensor_text.splitlines()) if m}
    keep, moved, cut = [], [], False
    for line in board_text.splitlines(keepends=True):
        m = LINE.match(line.rstrip("\r\n"))
        if m and m["key"] in MOVED_KEYS:
            cut = True
            if m["key"] not in (listed if "#" in m["pre"] else active):
                moved.append(line if line.endswith("\n") else line + "\n")
        else:
            keep.append(line)
    if not cut:
        return board_text, sensor_text, changes
    if moved:
        sep = "" if not sensor_text or sensor_text.endswith("\n") else "\n"
        sensor_text += sep + ("\n" if sensor_text else "") + "# INA226\n" + "".join(moved)
    return "".join(keep), sensor_text, changes


def renames(board_kv):
    """[(old, new, value)] for the renamed board.conf keys in `board_kv`; new == old value is fine."""
    out = []
    for old, new in RENAMED_KEYS.items():
        if old not in board_kv:
            continue
        if new in board_kv and board_kv[new] != board_kv[old]:
            raise MigrationError(f"board.conf {old}={board_kv[old]} and {new}={board_kv[new]} differ; "
                                 "delete the wrong one, then rerun")
        out.append((old, new, board_kv[old]))
    return out


def rename_lines(board_text):
    """(board text, [(old, new, value)]) with panel_en/panel_sd lines, commented-out ones too, renamed
    in place; an active old line is dropped when the new key is already set."""
    active = parse_conf(board_text)
    changes = renames(active)
    out = []
    for line in board_text.splitlines(keepends=True):
        body = line.rstrip("\r\n")
        m = LINE.match(body)
        new = m and RENAMED_KEYS.get(m["key"])
        if new:
            if "#" not in m["pre"] and new in active:
                continue
            line = m["pre"] + new + m["eq"] + m["val"] + m["post"] + line[len(body):]
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
    board = d / "board.conf"
    if board.exists():
        sensor = d / "sensor.conf"
        orig = board.read_text()
        btext, stext, changes = move_lines(orig, texts.get(sensor, sensor.read_text() if sensor.exists() else ""))
        if btext != orig:
            texts[sensor] = stext
        done += [("board.conf", k, "sensor.conf", v, v) for k, v in changes]
        btext, changes = rename_lines(btext)
        if btext != orig:
            texts[board] = btext
        done += [("board.conf", o, n, v, v) for o, n, v in changes]
    if write:
        for f, text in texts.items():
            if not f.exists() or f.read_text() != text:
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


def live_steps(text, forced_topo=None, rollback=False, warnings=None):
    """(step 1: before the OTA, step 2: after it) console commands. Notes needing attention are
    appended to `warnings` (a list) if given."""
    warn = warnings.append if warnings is not None else lambda _: None
    files = parse_get_config(text)
    if not {"sensor.conf", "limits.conf", "board.conf"} & files.keys():
        raise MigrationError("no sensor.conf, limits.conf or board.conf lines in the get-config output")
    sets, dels, after = [], [], []
    board = files.get("board.conf", {})
    for old, new, v in renames(board):
        if rollback:
            sets.append(f"set-config board.conf {old} {v}")
            if new not in board:
                after.append(f"del-config board.conf {new}")
        else:
            if new not in board:
                sets.append(f"set-config board.conf {new} {v}")
            after.append(f"del-config board.conf {old}")
    # INA226 keys: the old firmware reads board.conf only, the new one sensor.conf and tolerates an
    # equal board.conf copy. Deletes wait for step 2 so either firmware boots in between.
    sens = files.get("sensor.conf", {})
    for k, v, sv in moves(board, sens):
        differs = sv is not None and not same_value(v, sv)
        if rollback:
            sets.append(f"set-config board.conf {k} {v}")
            if sv is None:
                after.append(f"del-config sensor.conf {k}")
            elif differs:
                after.append(f"set-config sensor.conf {k} {sv}")
        else:
            if differs:
                warn(f"{differ_msg(k, v, sv)}: the new firmware would fail setup. The old firmware uses "
                     f"board.conf's {v}; step 1 sets sensor.conf to it. Check which value is right first.")
            if sv is None or differs:
                sets.append(f"set-config sensor.conf {k} {v}")
            after.append(f"del-config board.conf {k}")
    roles = [f for f in ("sensor.conf", "limits.conf") if f in files]
    boost = topo_of(files.get("converter.conf"), forced_topo) if roles else False
    for file in roles:
        for role, side, old, new in plan(file, files.get(file, {}), boost):
            if rollback:
                sets.append(f"set-config {file} {role} {old}")
                dels.append(f"del-config {file} {side}")
            else:
                sets.append(f"set-config {file} {side} {new}")
                dels.append(f"del-config {file} {role}")
    # all sets first: an interrupted run leaves the old keys, which the old firmware still reads
    return sets + dels, after


def live_commands(text, forced_topo=None, rollback=False):
    now, after = live_steps(text, forced_topo, rollback)
    return now + after


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
    b.add_argument("--rollback", action="store_true", help="print the inverse sequence (side -> role keys, bflow_* -> panel_*, "
                   "INA226 keys back to board.conf)")
    args = ap.parse_args(argv)

    try:
        if args.mode == "live":
            text = sys.stdin.read() if args.log == "-" else Path(args.log).read_text()
            warnings = []
            now, after = live_steps(text, args.topo, args.rollback, warnings)
            if after:
                ota = "the downgrade" if args.rollback else "the OTA"
                now = ([f"# step 1: now, before {ota}"] + (now or ["# (nothing)"]) +
                       [f"# step 2: after {ota}, once bootinfo shows that image VALID"] + after)
            for w in warnings:
                print(f"migrate_side_keys: WARNING: {w}", file=sys.stderr)
            print("\n".join([f"# WARNING: {w}" for w in warnings] + now) if now else "# nothing to migrate", flush=True)
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
