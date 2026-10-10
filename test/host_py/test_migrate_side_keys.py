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
        for d in sorted(ROOT.glob("config/**/conf")):
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


if __name__ == "__main__":
    unittest.main()
