"""Host tests for etc/config-tool/scrape_conf_keys.py: getter receiver resolution, key following, and
that anything it cannot resolve fails the run instead of being skipped.

Run: .venv/bin/python3 -m pytest test/host_py/test_scrape_conf_keys.py
"""
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "etc" / "config-tool"))
import scrape_conf_keys as sck  # noqa: E402


@pytest.fixture
def bare(monkeypatch):
    """Synthetic sources: no hand tables, so only what the code says counts."""
    for name, empty in (("VAR_TO_CONF", {}), ("FILE_VAR_OVERRIDES", {}), ("LAMBDA_ARG_CONFS", {}),
                        ("DYNAMIC_KEY_SITES", {}), ("MIGRATION_GUARD_READS", {}), ("INDIRECT_KEYS", []),
                        ("IGNORED_CONFS", set())):
        monkeypatch.setattr(sck, name, empty)

    def scan(files):
        return sck.scan_sources({rel: sck.Source(rel, txt) for rel, txt in files.items()})
    return scan


def test_same_name_resolves_per_function(bare):
    s = bare({"src/a.cpp": """
        void f() { ConfFile c{"/littlefs/conf/bsync.conf"}; c.getFloat("kp", 5e-6f); }
        void g() { ConfFile c{"/littlefs/conf/tlscon.conf", true}; c.getString("psk", ""); }
        void h() { ConfFile conf = loadConfSafe("/littlefs/conf/tele.conf"); conf.getLong("binary", 0); }
    """})
    assert not s.errors
    assert s.file_keys == {"bsync.conf": {"kp"}, "tlscon.conf": {"psk"}, "tele.conf": {"binary"}}
    assert s.defaults["kp"] == "5e-06"


def test_parameter_traced_to_callers(bare):
    s = bare({
        "src/guard.cpp": """
            static void wireGuardConf(const ConfFile &c) { c.getFloat("wire_p_warn", 2.f); }
            void setup() { ConfFile limitsConf{"/littlefs/conf/limits.conf"}; wireGuardConf(limitsConf); }
        """,
        "src/pg.h": """
            class LvPgood { public: void init(const ConfFile &board) { board.getByte("lv_pgood", 255); } };
            class Other { public: void init(const ConfFile &x) { } };
        """,
        "src/m.cpp": """
            LvPgood lvPgood{};
            Other other{};
            void begin() {
                ConfFile b{"/littlefs/conf/board.conf"};
                ConfFile t{"/littlefs/conf/tracker.conf"};
                lvPgood.init(b);
                other.init(t);
            }
        """,
    })
    assert not s.errors
    assert s.file_keys == {"limits.conf": {"wire_p_warn"}, "board.conf": {"lv_pgood"}}
    assert s.defaults["wire_p_warn"] == "2.0"


def test_constructor_init_list_and_delegation(bare):
    s = bare({"src/l.h": """
        struct Limits {
            float a, b;
            Limits(const ConfFile &limits, bool boost) : a(limits.getFloat("vin_min")), b(limits.getFloat("p_max", 1.f)) {}
            explicit Limits(const ConfFile &limits) : Limits(limits, true) {}
            Limits(const Limits &lim) = default;
            Limits &operator=(const Limits &right) { new(this) Limits(right); return *this; }
        };
        void s() { Limits l{ConfFile{"/littlefs/conf/limits.conf"}}; }
    """})
    assert not s.errors, s.errors
    assert s.file_keys == {"limits.conf": {"vin_min", "p_max"}}
    assert "vin_min" not in s.defaults                      # required: no default


def test_service_conf_path_and_base(bare):
    s = bare({
        "src/service.h": """
            class Service {
                const char *_confPath;
                void loadConf() { ConfFile c{_confPath, true}; c.getByte("enabled", _enabled ? 1 : 0);
                                  c.getString("log_level", "info"); }
            };
        """,
        "src/bsync.h": """
            class BeaconSyncService : public Service {
                BeaconSyncService() : Service("bsync", "/littlefs/conf/bsync.conf", false) {}
                void onStart();
            };
            class LcdService : public Service {
                LcdService() : Service("lcd", "/littlefs/conf/lcd.conf", false) {}
                bool onStart() { return ConfFile{_confPath, true}.getByte("addr", 0); }
            };
        """,
        "src/bsync.cpp": """
            void BeaconSyncService::onStart() { ConfFile c{_confPath, true}; c.getString("bssid", ""); }
        """,
    })
    assert not s.errors, s.errors
    assert s.file_keys["bsync.conf"] == {"bssid", "enabled", "log_level"}
    assert s.file_keys["lcd.conf"] == {"addr", "enabled", "log_level"}
    assert "enabled" not in s.defaults and s.defaults["log_level"] == '"info"'


def test_temporaries(bare):
    s = bare({"src/t.cpp": """
        void a() { ConfFile{"/littlefs/conf/tele.conf"}.getLong("adv_ms", 500); }
        void b() { loadConfSafe("/littlefs/conf/ble.conf", false, true).getLong("idle_adv_ms", 1000); }
        bool add() { auto confPath = "/littlefs/conf/wifi.conf"; ConfFile w{confPath, true};
                     return w.getLong("improv_offline_s", 600); }
    """})
    assert not s.errors, s.errors
    assert s.file_keys == {"tele.conf": {"adv_ms"}, "ble.conf": {"idle_adv_ms"}, "wifi.conf": {"improv_offline_s"}}


def test_key_followed_through_helper_and_locals(bare):
    s = bare({"src/b.h": """
        void init(const ConfFile &boardConf, bool isBoost) {
            auto parseDt = [&](const char *key, bool hl) -> uint32_t { return boardConf.getFloat(key, 0.f); };
            parseDt("pwm_deadtime_hl_ns", true);
            parseDt("pwm_deadtime_lh_ns", false);
            auto pn = isBoost ? "pwm_en" : "pwm_in";
            boardConf.getByte(pn);
            std::string k = "led_WS2812";
            boardConf.getByte(k, 255);
        }
        void setup() { ConfFile boardConf{"/littlefs/conf/board.conf"}; init(boardConf, false); }
    """})
    assert not s.errors, s.errors
    assert s.file_keys["board.conf"] == {"pwm_deadtime_hl_ns", "pwm_deadtime_lh_ns", "pwm_en", "pwm_in",
                                         "led_WS2812"}


def test_mutated_local_is_not_a_literal_key(bare):
    s = bare({"src/w.cpp": """
        void f() { ConfFile w{"/littlefs/conf/wifi.conf"}; std::string label = "ssid_"; label += "x";
                   w.getString(label, ""); }
    """})
    assert any("computed at runtime" in e for e in s.errors)
    assert "ssid_" not in s.file_keys.get("wifi.conf", set())


def test_unresolved_parameter_fails(bare):
    s = bare({"src/p.h": """
        void load(const ConfFile &c) { c.getFloat("cv_eoc", 3.5f); }
    """})
    assert any("unresolved getter receiver `c`" in e and "no call site" in e for e in s.errors)


def test_conflicting_callers_fail(bare):
    s = bare({"src/p.cpp": """
        void load(const ConfFile &c) { c.getFloat("x", 1.f); }
        void a() { ConfFile p{"/littlefs/conf/a.conf"}; load(p); }
        void b() { ConfFile q{"/littlefs/conf/b.conf"}; load(q); }
    """})
    assert any("callers pass different files" in e for e in s.errors)


def test_non_literal_conf_path_fails(bare):
    s = bare({"src/p.cpp": """
        void f(const char *fn) { ConfFile conf{fn}; conf.getString("username", ""); }
    """})
    assert any("not a literal /littlefs/conf/" in e for e in s.errors)


def test_dynamic_key_needs_acknowledgement(bare, monkeypatch):
    files = {"src/d.cpp": """
        void f(const std::string &key) { ConfFile c{"/littlefs/conf/sensor.conf"}; c.getByte(key + "_ch", 255); }
    """}
    assert any("computed at runtime" in e for e in bare(files).errors)
    monkeypatch.setattr(sck, "DYNAMIC_KEY_SITES", {("src/d.cpp", 'key + "_ch"'): "pattern"})
    assert not bare(files).errors


def test_stale_hand_entries_fail(bare, monkeypatch):
    monkeypatch.setattr(sck, "FILE_VAR_OVERRIDES", {"src/gone.cpp": {"c": "x.conf"}})
    monkeypatch.setattr(sck, "DYNAMIC_KEY_SITES", {("src/gone.cpp", "k"): "moved"})
    s = bare({"src/a.cpp": 'void f() { ConfFile{"/littlefs/conf/a.conf"}.getByte("k", 0); }'})
    assert any("FILE_VAR_OVERRIDES" in e or "override [" in e for e in s.errors)
    assert any("DYNAMIC_KEY_SITES entry" in e for e in s.errors)


def test_disagreeing_defaults_left_out(bare):
    s = bare({"src/a.cpp": """
        void a() { ConfFile{"/littlefs/conf/http.conf"}.getLong("port", 80); }
        void b() { ConfFile{"/littlefs/conf/tlscon.conf"}.getLong("port", 992); }
        void c() { ConfFile{"/littlefs/conf/coil.conf"}.getFloat("L0"); ConfFile{"/littlefs/conf/coil.conf"}.f("L0", 50e-6f); }
    """})
    assert not s.errors
    assert "port" not in s.defaults and "L0" not in s.defaults
    assert any("'port'" in w for w in s.warnings) and any("'L0'" in w for w in s.warnings)


def test_main_exits_nonzero_on_unresolved(bare, tmp_path, capsys):
    (tmp_path / "src").mkdir()
    (tmp_path / "main").mkdir()
    (tmp_path / "src" / "p.h").write_text('void load(const ConfFile &c) { c.getFloat("cv_eoc", 3.5f); }')
    html = tmp_path / "e.html"
    html.write_text("// SCRAPER:BEGIN FILE_KEYS\nconst FILE_KEYS = {};\n// SCRAPER:END FILE_KEYS\n")
    before = html.read_text()
    assert sck.main(["--repo", str(tmp_path), "--html", str(html), "--write"]) == 2
    assert sck.main(["--repo", str(tmp_path), "--html", str(html), "--check"]) == 2
    assert html.read_text() == before
    assert "unresolved getter receiver" in capsys.readouterr().err


# ---- the real firmware tree

@pytest.fixture(scope="module")
def repo_scan():
    return sck.scan_repo(ROOT)


def test_repo_resolves_everything(repo_scan):
    assert not repo_scan.errors, "\n".join(repo_scan.errors)


@pytest.mark.parametrize("conf,key", [
    ("converter.conf", "ctrl_vin_kp"), ("converter.conf", "ctrl_power_td"),
    ("bsync.conf", "bssid"), ("bsync.conf", "hw_only"), ("bsync.conf", "enabled"),
    ("tele.conf", "adv_ms"), ("tele.conf", "binary"), ("tele.conf", "ble"), ("tele.conf", "idle_interval_ms"),
    ("ble.conf", "ble_passkey"), ("ble.conf", "idle_adv_ms"), ("wifi.conf", "improv_offline_s"),
    ("tlscon.conf", "psk"), ("tlscon.conf", "psk_id"), ("board.conf", "pwm_deadtime_hl_ns"),
    ("board.conf", "lv_pgood"), ("board.conf", "lv_pgood_v"), ("limits.conf", "wire_p_warn"),
    ("limits.conf", "hv_max"), ("board.conf", "pwm_hi"), ("board.conf", "pwm_en"), ("http.conf", "port"),
    ("tracker.conf", "pv_substrings"),
])
def test_repo_keys_read_indirectly_are_kept(repo_scan, conf, key):
    assert key in repo_scan.file_keys.get(conf, set())


def test_repo_ignored_and_guard_reads_stay_out(repo_scan):
    assert "vconv.conf" not in repo_scan.file_keys
    assert "ntc_ch" not in repo_scan.file_keys["board.conf"]
    assert "pwm_driver" not in repo_scan.file_keys["converter.conf"]


def test_write_roundtrip_on_copy(repo_scan, tmp_path):
    html = tmp_path / "conf-editor.html"
    shutil.copy(ROOT / "etc" / "config-tool" / "conf-editor.html", html)
    assert sck.main(["--repo", str(ROOT), "--html", str(html), "--write"]) == 0
    fk, tk, df = sck.parse_tables(html.read_text())
    assert fk == repo_scan.file_keys
    assert tk == {t: repo_scan.type_keys.get(t, set()) for t in ("byte", "long", "float", "string")}
    assert set(df) == set(repo_scan.defaults)
