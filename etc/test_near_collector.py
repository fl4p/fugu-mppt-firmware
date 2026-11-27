#!/usr/bin/env python3
"""Host tests for etc/havan/near_collector.py: the probe-line parser and the daily archive.

Run: `.venv/bin/python3 -m pytest etc/test_near_collector.py`.
"""
import datetime
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "havan"))

import near_collector as nc  # noqa: E402

# as the MQTT log mirror carries them, with other device output interleaved
NOISE = "V=63.9/27.72 I=7.83/17.53A 501.0W st=MPPT,1 lag=2119"


def test_parses_a_real_probe():
    rec = nc.parse_near_line("I (328017) mppt: scan near: v0 57.4 V, -2.0V x0.939, "
                             "+2.0V x1.003, ref 366.4->368.6 W, near only")
    assert rec == {"v0": 57.4, "minus": 0.939, "plus": 1.003, "ref_before": 366.4,
                   "ref_after": 368.6, "drift": False, "near_only": True}


def test_drift_flag():
    rec = nc.parse_near_line("I (1) mppt: scan near: v0 52.7 V, -2.0V x0.960, +2.0V x1.040, "
                             "ref 413.4->405.9 W, drift, near only")
    assert rec["drift"] is True and rec["near_only"] is True


def test_na_ratio_is_none():
    rec = nc.parse_near_line("I (1) mppt: scan near: v0 74.7 V, -2.0V x1.003, +2.0V n/a, "
                             "ref 77.9->78.1 W")
    assert rec["plus"] is None and rec["minus"] == 1.003 and rec["drift"] is False


def test_ignores_every_other_line():
    for ln in (NOISE,
               "I (1) mppt: scan: 5 bypass-diode substrings learned from the sweep",
               "I (1) mppt: Stop sweep 8.24s mode=CV (lim=-1 tgt=nan act=nan) MPP=(137.9W,1853,61.3V)",
               "I (1) mppt: scan: P 166.6->167.5 W at 61.7V, 1 probes, best x0.812 at 46.6V",
               ""):
        assert nc.parse_near_line(ln) is None


def test_append_writes_daily_jsonl_and_never_rewrites(tmp_path):
    rec = nc.parse_near_line("I (1) mppt: scan near: v0 57.4 V, -2.0V x0.939, +2.0V x1.003, "
                             "ref 366.4->368.6 W")
    when = datetime.datetime(2026, 11, 27, 15, 12, 34, tzinfo=datetime.timezone.utc)
    out = nc.append(str(tmp_path), "flat", rec, "the raw line", now=when)
    assert out["device"] == "flat" and out["recv_ts"] == "2026-11-27T15:12:34Z"
    assert out["raw"] == "the raw line"
    lines = (tmp_path / "2026-11-27.jsonl").read_text().strip().splitlines()
    assert len(lines) == 1 and json.loads(lines[0]) == out
    nc.append(str(tmp_path), "fry", rec, "second", now=when)
    assert len((tmp_path / "2026-11-27.jsonl").read_text().strip().splitlines()) == 2
    # a different UTC day is a different file
    nc.append(str(tmp_path), "flat", rec, "next day", now=when + datetime.timedelta(days=1))
    assert (tmp_path / "2026-11-28.jsonl").exists()
