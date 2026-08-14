from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _vout(profile):
    text = (profile / "conf" / "charger.conf").read_text()
    return float(next(line.split("=", 1)[1] for line in text.splitlines()
                      if line.startswith("vout_max=")))


def test_battery_profile_keeps_the_29v_protection_reference():
    assert _vout(ROOT / "config/lab/fbuck_lab_bench") == 29.0


def test_open_output_profile_is_complete_and_changes_only_the_charger_limit():
    battery = ROOT / "config/lab/fbuck_lab_bench"
    open_output = ROOT / "config/lab/fbuck_lab_bench_open_output"
    expected = {
        "board.conf", "charger.conf", "coil.conf", "converter.conf",
        "limits.conf", "sensor.conf", "tele.conf", "tracker.conf", "wifi.conf",
    }

    assert {path.name for path in (battery / "conf").iterdir()} == expected
    assert {path.name for path in (open_output / "conf").iterdir()} == expected
    assert not (open_output / "extends").exists()
    assert _vout(battery) == 29.0
    assert _vout(open_output) == 60.0

    for name in expected - {"charger.conf"}:
        open_text = (open_output / "conf" / name).read_text().splitlines()
        battery_text = (battery / "conf" / name).read_text().splitlines()
        assert open_text == battery_text
