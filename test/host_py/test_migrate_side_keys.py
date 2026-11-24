"""Host tests for etc/migrate_side_keys.py and the side-keyed profiles under config/.

The reference is every profile as it was before side keys (BASE, role keys only), read the way that
firmware read it. Both the migrated BASE profiles and today's config/ must resolve to the same Vin,
Vout, Iin and Iout channels (ADC, channel, divider, factor incl. sign, midpoint, filt_len) and the
same four limits, read the way the current firmware (src/conv_side.h) reads them.

Run: python3 -m pytest test/host_py/test_migrate_side_keys.py
"""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "etc"))
import migrate_side_keys as m  # noqa: E402

BASE = "861d51d5af052b111047b89359e26f55bbd80b8c"  # origin/main before the side keys
CONFS = ("converter.conf", "sensor.conf", "limits.conf")

# Profiles whose resolved values changed on purpose since BASE (none: fmetal's live boost wiring
# went to the new profile lab/fmetal_boost, which has no BASE version).
EXPECTED_CHANGES = {}


def git_show(path):
    r = subprocess.run(["git", "-C", str(ROOT), "show", f"{BASE}:{path}"], capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else None


def base_profiles():
    r = subprocess.run(["git", "-C", str(ROOT), "ls-tree", "-r", "--name-only", BASE, "config"],
                       capture_output=True, text=True)
    if r.returncode:
        raise unittest.SkipTest(f"BASE commit {BASE[:8]} not in this clone")
    return sorted({str(Path(p).parent) for p in r.stdout.split() if p.endswith("/sensor.conf")})


def tracked(suffix):
    """Paths under config/ in git ending in `suffix` (untracked device captures and backups excluded)."""
    r = subprocess.run(["git", "-C", str(ROOT), "ls-files", "config"], capture_output=True, text=True, check=True)
    return sorted(ROOT / p for p in r.stdout.split() if p.endswith(suffix))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return v


def resolve(files, side_keys):
    """Role-resolved channels and limits. side_keys=False reads role keys as the BASE firmware did,
    True reads side keys by topo as the current firmware does and fails on a role key."""
    conv = files.get("converter.conf")
    boost = m.topo_of({} if conv is None else m.parse_conf(conv), None)
    s = m.parse_conf(files.get("sensor.conf") or "")
    lim = m.parse_conf(files.get("limits.conf") or "")
    if side_keys:
        for f, kv in (("sensor.conf", s), ("limits.conf", lim)):
            role = [k for k in kv if m.side_key(f, k, boost)]
            assert not role, f"{f}: role keys {role} would fail setup"
    out = {}
    for chn in ("vin", "iin", "iout", "vout"):
        key = f"{m.side_of(chn[1] == 'i', boost)}_{chn[0]}" if side_keys else chn
        g = lambda sfx, d=None: s.get(f"{key}_{sfx}", d)  # noqa: E731
        r = {"ch": num(g("ch", "255")), "filt_len": num(g("filt_len", "10"))}
        if r["ch"] != 255:
            r["adc"] = g("adc", s.get("adc", ""))
            if chn[0] == "v":
                r.update(rh=num(g("rh")), rl=num(g("rl")))
            else:
                f = num(g("factor", "1"))
                r.update(factor=-f if side_keys and boost else f, midpoint=num(g("midpoint", "0")))
        out[chn] = r
    for role in m.LIMIT_KEYS:
        out[role] = num(lim.get(m.side_key("limits.conf", role, boost) if side_keys else role))
    return out


def apply_console(kv_by_file, cmds):
    for c in cmds:
        verb, file, key, *val = c.split(" ")
        kv = kv_by_file.setdefault(file, {})
        if verb == "set-config":
            kv[key] = " ".join(val)
        else:
            assert verb == "del-config" and key in kv, c
            del kv[key]


def to_text(kv):
    return "".join(f"{k}={v}\n" for k, v in kv.items())


class ProfilesTest(unittest.TestCase):
    def test_current_profiles_resolve_like_base(self):
        for p in base_profiles():
            with self.subTest(profile=p):
                base = {f: git_show(f"{p}/{f}") for f in CONFS}
                now = {f: (ROOT / p / f).read_text() if (ROOT / p / f).exists() else None for f in CONFS}
                want = resolve(base, False)
                want.update(EXPECTED_CHANGES.get(p, {}))
                self.assertEqual(resolve(now, True), want)

    def test_no_profile_has_a_role_key(self):
        for d in sorted({f.parent for f in tracked("/conf/board.conf")}):
            with self.subTest(profile=str(d.relative_to(ROOT))):
                self.assertEqual(m.migrate_dir(d, write=False), [])

    def test_fmetal_boost_current_limits(self):
        lim = m.parse_conf((ROOT / "config/lab/fmetal_boost/conf/limits.conf").read_text())
        self.assertEqual((lim["hv_i_max"], lim["lv_i_max"]), ("30", "32"))


class OfflineTest(unittest.TestCase):
    def test_round_trip_on_base_profiles(self):
        for p in base_profiles():
            with self.subTest(profile=p), tempfile.TemporaryDirectory() as tmp:
                base = {f: git_show(f"{p}/{f}") for f in CONFS}
                for f, t in base.items():
                    if t is not None:
                        (Path(tmp) / f).write_text(t)
                m.migrate_dir(tmp)
                got = {f: (Path(tmp) / f).read_text() if (Path(tmp) / f).exists() else None for f in CONFS}
                self.assertEqual(resolve(got, True), resolve(base, False))
                again = dict(got)
                self.assertEqual(m.migrate_dir(tmp), [])  # idempotent
                self.assertEqual({f: (Path(tmp) / f).read_text() if again[f] is not None else None
                                  for f in CONFS}, again)

    def test_keeps_comments_and_flips_boost_current_sign(self):
        text = "# head\nvin_ch=0   # LV\n# iin_ch=255  # none\niin_factor=-1.0\niout_factor=+2\nntc_ch=7\n"
        new, _ = m.rewrite("sensor.conf", text, True)
        self.assertEqual(new, "# head\nlv_v_ch=0   # LV\n# lv_i_ch=255  # none\nlv_i_factor=1.0\n"
                              "hv_i_factor=-2\nntc_ch=7\n")

    def test_refusals(self):
        with self.assertRaisesRegex(m.MigrationError, "both vin_ch and hv_v_ch"):
            m.rewrite("sensor.conf", "vin_ch=1\nhv_v_ch=2\n", False)
        with self.assertRaisesRegex(m.MigrationError, "not a number"):
            m.rewrite("sensor.conf", "iout_factor=x\n", True)
        with self.assertRaisesRegex(m.MigrationError, "topo must be"):
            m.topo_of({"topo": "sepic"}, None)
        with self.assertRaisesRegex(m.MigrationError, "contradicts"):
            m.topo_of({"topo": "buck"}, "boost")
        self.assertEqual(m.rewrite("limits.conf", "vin_min=8\niout_short=6\np_max=800\n", False)[1], [])

    def test_moves_ina226_keys_to_sensor_conf(self):
        # move_lines runs after rename_lines, so it sees the new names
        board = "# INA226\nina226_alert=41\nina226_resistor=0.5e-3\nina226_conv_time_us=332 # fast\n#ina226_ov=30\n"
        b, s, ch = m.move_lines(board, "ntc_ch=7")
        self.assertEqual(b, "# INA226\nina226_alert=41\n")
        self.assertEqual(s, "ntc_ch=7\n\n# INA226\nina226_resistor=0.5e-3\nina226_conv_time_us=332 # fast\n#ina226_ov=30\n")
        self.assertEqual(ch, [("ina226_resistor", "0.5e-3"), ("ina226_conv_time_us", "332")])
        self.assertEqual(m.move_lines(b, s), (b, s, []))
        b, s, ch = m.move_lines("ina226_ov=30\n", "#ina226_ov=1\nina226_ov=30\n")  # equal: drop from board
        self.assertEqual((b, s, ch), ("", "#ina226_ov=1\nina226_ov=30\n", [("ina226_ov", "30")]))
        with self.assertRaisesRegex(m.MigrationError, "differ"):
            m.move_lines("ina226_ov=30\n", "ina226_ov=31\n")

    def test_offline_renames_then_moves_ina226_keys(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "converter.conf").write_text("topo=buck\n")
            (Path(tmp) / "board.conf").write_text("ina22x_alert=41\nina22x_addr=0x40\nina22x_resistor=0.5e-3\n#ina22x_ov=30\n")
            (Path(tmp) / "sensor.conf").write_text("ntc_ch=7\n")
            m.migrate_dir(tmp)
            # renamed in board.conf, and the moved key lands in sensor.conf under its new name
            self.assertEqual((Path(tmp) / "board.conf").read_text(), "ina226_alert=41\nina226_addr=0x40\n")
            self.assertEqual((Path(tmp) / "sensor.conf").read_text(),
                             "ntc_ch=7\n\n# INA226\nina226_resistor=0.5e-3\n#ina226_ov=30\n")
            self.assertEqual(m.migrate_dir(tmp, write=False), [])  # idempotent

    def test_renames_ina226_keys(self):
        new, ch = m.rename_lines("board.conf", "ina22x_alert=41\nina22x_addr=0x40\n#ina22x_ov=30\n")
        self.assertEqual(new, "ina226_alert=41\nina226_addr=0x40\n#ina226_ov=30\n")
        self.assertEqual(ch, [("ina22x_alert", "ina226_alert", "41"), ("ina22x_addr", "ina226_addr", "0x40")])
        self.assertEqual(m.rename_lines("sensor.conf", "ina22x_resistor=0.5e-3\n"),
                         ("ina226_resistor=0.5e-3\n", [("ina22x_resistor", "ina226_resistor", "0.5e-3")]))
        self.assertEqual(m.rename_lines("sensor.conf", "ina226_resistor=0.5e-3\n"),
                         ("ina226_resistor=0.5e-3\n", []))
        with self.assertRaisesRegex(m.MigrationError, "differ"):
            m.rename_lines("sensor.conf", "ina22x_ov=30\nina226_ov=31\n")

    def test_renames_backflow_pins(self):
        text = "panel_sd=40   # SD\n# panel_en=8 # none\nbflow_bat=1\n"
        new, ch = m.rename_lines("board.conf", text)
        self.assertEqual(new, "bflow_sd=40   # SD\n# bflow_en=8 # none\nbflow_bat=1\n")
        self.assertEqual(ch, [("panel_sd", "bflow_sd", "40")])
        self.assertEqual(m.rename_lines("board.conf", new), (new, []))
        self.assertEqual(m.rename_lines("board.conf", "panel_sd=40\nbflow_sd=40\n"),
                         ("bflow_sd=40\n", [("panel_sd", "bflow_sd", "40")]))
        with self.assertRaisesRegex(m.MigrationError, "differ"):
            m.rename_lines("board.conf", "panel_sd=40\nbflow_sd=41\n")

    def test_no_profile_has_old_keys(self):
        for suffix, file in (("/board.conf", "board.conf"), ("/sensor.conf", "sensor.conf")):
            for f in tracked(suffix):
                with self.subTest(profile=str(f.relative_to(ROOT))):
                    self.assertFalse(set(m.RENAMED_KEYS[file]) & m.parse_conf(f.read_text()).keys())

    def test_no_profile_has_ina226_keys_in_board_conf(self):
        for f in tracked("/board.conf"):
            with self.subTest(profile=str(f.relative_to(ROOT))):
                self.assertFalse(set(m.MOVED_KEYS) & m.parse_conf(f.read_text()).keys())


class LiveTest(unittest.TestCase):
    @staticmethod
    def capture(files):
        lines = []
        for f, text in files.items():
            if text is not None:
                lines += [f"I (123) main: Conf '/littlefs/conf/{f}:{k}' = '{v}'\r"
                          for k, v in m.parse_conf(text).items()]
        return "\n".join(lines)

    def test_console_sequence_on_base_profiles(self):
        for p in base_profiles():
            with self.subTest(profile=p):
                base = {f: git_show(f"{p}/{f}") for f in CONFS}
                kv = {f: m.parse_conf(t) for f, t in base.items() if t is not None}
                # a board without converter.conf prints nothing for it: the user passes --topo buck
                topo = None if "converter.conf" in kv else "buck"
                cap = self.capture({f: to_text(v) for f, v in kv.items()})
                cmds = m.live_commands(cap, topo)
                apply_console(kv, cmds)
                got = {f: to_text(v) for f, v in kv.items()}
                self.assertEqual(resolve(got, True), resolve(base, False))
                self.assertEqual(m.live_commands(self.capture(got), topo), [])

    def test_rollback_restores(self):
        kv = {"converter.conf": {"topo": "boost"}, "sensor.conf": {"iin_ch": "1", "iin_factor": "-1.0"},
              "limits.conf": {"vin_max": "60", "vin_min": "10"}}
        cap = self.capture({f: to_text(v) for f, v in kv.items()})
        self.assertEqual(m.live_commands(cap), [
            "set-config sensor.conf lv_i_ch 1", "set-config sensor.conf lv_i_factor 1.0",
            "set-config limits.conf lv_max 60",
            "del-config sensor.conf iin_ch", "del-config sensor.conf iin_factor", "del-config limits.conf vin_max"])
        new = {f: dict(v) for f, v in kv.items()}
        apply_console(new, m.live_commands(cap))
        rb = m.live_commands(cap, rollback=True)
        apply_console(new, rb)
        self.assertEqual(new, kv)

    def test_needs_topo_without_converter_capture(self):
        cap = "Conf '/littlefs/conf/sensor.conf:vin_ch' = '3'"
        with self.assertRaisesRegex(m.MigrationError, "--topo"):
            m.live_commands(cap)
        self.assertEqual(m.live_commands(cap, "buck"),
                         ["set-config sensor.conf hv_v_ch 3", "del-config sensor.conf vin_ch"])

    def test_ina226_keys_board_only_capture(self):
        cap = ("Conf '/littlefs/conf/board.conf:ina22x_resistor' = '0.5e-3'\n"
               "Conf '/littlefs/conf/board.conf:ina22x_ov' = '30'\n"
               "Conf '/littlefs/conf/board.conf:ina22x_alert' = '41'")
        # the old firmware reads board.conf: its copy is only deleted after the OTA. The new name is
        # set in board.conf too, so step 2's delete of it finds the key.
        self.assertEqual(m.live_steps(cap), (
            ["set-config board.conf ina226_alert 41", "set-config board.conf ina226_resistor 0.5e-3",
             "set-config board.conf ina226_ov 30",
             "set-config sensor.conf ina226_resistor 0.5e-3", "set-config sensor.conf ina226_ov 30"],
            ["del-config board.conf ina22x_alert", "del-config board.conf ina22x_resistor",
             "del-config board.conf ina22x_ov",
             "del-config board.conf ina226_resistor", "del-config board.conf ina226_ov"]))
        self.assertEqual(m.live_steps(cap, rollback=True), (
            ["set-config board.conf ina22x_alert 41", "set-config board.conf ina22x_resistor 0.5e-3",
             "set-config board.conf ina22x_ov 30",
             "set-config board.conf ina226_resistor 0.5e-3", "set-config board.conf ina226_ov 30"],
            ["del-config board.conf ina226_alert", "del-config board.conf ina226_resistor",
             "del-config board.conf ina226_ov",
             "del-config sensor.conf ina226_resistor", "del-config sensor.conf ina226_ov"]))

    def test_ina226_keys_same_in_both(self):  # fry 2026-11-23
        cap = ("Conf '/littlefs/conf/converter.conf:topo' = 'buck'\n"
               "Conf '/littlefs/conf/board.conf:ina22x_resistor' = '0.5e-3'\n"
               "Conf '/littlefs/conf/board.conf:ina22x_ov' = '0'\n"
               "Conf '/littlefs/conf/sensor.conf:ina22x_resistor' = '0.0005'\n"
               "Conf '/littlefs/conf/sensor.conf:ina22x_ov' = '0'")
        w = []
        self.assertEqual(m.live_steps(cap, warnings=w), (
            ["set-config board.conf ina226_resistor 0.5e-3", "set-config board.conf ina226_ov 0",
             "set-config sensor.conf ina226_resistor 0.0005", "set-config sensor.conf ina226_ov 0"],
            ["del-config board.conf ina22x_resistor", "del-config board.conf ina22x_ov",
             "del-config sensor.conf ina22x_resistor", "del-config sensor.conf ina22x_ov",
             "del-config board.conf ina226_resistor", "del-config board.conf ina226_ov"]))
        self.assertEqual(w, [])
        self.assertEqual(m.live_steps(cap, rollback=True), (
            ["set-config board.conf ina22x_resistor 0.5e-3", "set-config board.conf ina22x_ov 0",
             "set-config sensor.conf ina22x_resistor 0.0005", "set-config sensor.conf ina22x_ov 0",
             "set-config board.conf ina226_resistor 0.5e-3", "set-config board.conf ina226_ov 0"],
            ["del-config board.conf ina226_resistor", "del-config board.conf ina226_ov",
             "del-config sensor.conf ina226_resistor", "del-config sensor.conf ina226_ov"]))

    def test_ina226_keys_differ(self):
        cap = ("Conf '/littlefs/conf/converter.conf:topo' = 'buck'\n"
               "Conf '/littlefs/conf/board.conf:ina22x_resistor' = '0.5e-3'\n"
               "Conf '/littlefs/conf/sensor.conf:ina22x_resistor' = '1e-3'")
        w = []
        self.assertEqual(m.live_steps(cap, warnings=w), (
            ["set-config board.conf ina226_resistor 0.5e-3", "set-config sensor.conf ina226_resistor 1e-3",
             "set-config sensor.conf ina226_resistor 0.5e-3"],
            ["del-config board.conf ina22x_resistor", "del-config sensor.conf ina22x_resistor",
             "del-config board.conf ina226_resistor"]))
        self.assertEqual(len(w), 1)
        self.assertIn("would fail setup", w[0])
        self.assertIn("board.conf's 0.5e-3", w[0])
        # the rollback restores sensor.conf's value under its old name (the rename step), so it must
        # not also re-create it under the new one: that copy would outlive the rollback
        self.assertEqual(m.live_steps(cap, rollback=True), (
            ["set-config board.conf ina22x_resistor 0.5e-3", "set-config sensor.conf ina22x_resistor 1e-3",
             "set-config board.conf ina226_resistor 0.5e-3"],
            ["del-config board.conf ina226_resistor", "del-config sensor.conf ina226_resistor"]))

    def test_ina226_keys_already_migrated(self):
        cap = "Conf '/littlefs/conf/converter.conf:topo' = 'buck'\n" \
            "Conf '/littlefs/conf/sensor.conf:ina22x_resistor' = '0.5e-3'\nConf '/littlefs/conf/board.conf:i2c_port' = '0'"
        self.assertEqual(m.live_steps(cap),
                         (["set-config sensor.conf ina226_resistor 0.5e-3"],
                          ["del-config sensor.conf ina22x_resistor"]))

    def test_step1_never_deletes_a_key_the_old_firmware_reads(self):
        cap = ("Conf '/littlefs/conf/converter.conf:topo' = 'buck'\n"
               "Conf '/littlefs/conf/board.conf:ina22x_resistor' = '0.5e-3'\n"
               "Conf '/littlefs/conf/board.conf:panel_sd' = '40'\n"
               "Conf '/littlefs/conf/sensor.conf:ina22x_resistor' = '0.5e-3'")
        now, _ = m.live_steps(cap)
        self.assertFalse([c for c in now if c.startswith("del-config board.conf")])

    def test_equal_spellings_move_offline(self):
        b, s, ch = m.move_lines("ina226_resistor=0.5e-3\n", "ina226_resistor=0.0005\n")
        self.assertEqual((b, s, ch), ("", "ina226_resistor=0.0005\n", [("ina226_resistor", "0.5e-3")]))

    def test_backflow_rename_in_two_steps(self):
        cap = ("Conf '/littlefs/conf/board.conf:panel_sd' = '40'\n"
               "Conf '/littlefs/conf/board.conf:bflow_bat' = '1'")
        self.assertEqual(m.live_steps(cap), (["set-config board.conf bflow_sd 40"], ["del-config board.conf panel_sd"]))
        self.assertEqual(m.live_steps(cap, rollback=True),
                         (["set-config board.conf panel_sd 40"], ["del-config board.conf bflow_sd"]))
        both = cap + "\nConf '/littlefs/conf/board.conf:bflow_sd' = '40'"
        self.assertEqual(m.live_steps(both), ([], ["del-config board.conf panel_sd"]))
        with self.assertRaisesRegex(m.MigrationError, "differ"):
            m.live_steps(cap + "\nConf '/littlefs/conf/board.conf:bflow_sd' = '41'")


if __name__ == "__main__":
    unittest.main()
