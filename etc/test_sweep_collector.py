#!/usr/bin/env python3
"""Host tests for etc/havan/sweep_collector.py: sc: parser, line protocol, request flow, archive.

Run: `.venv/bin/python3 -m pytest etc/test_sweep_collector.py`.
"""
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "havan"))

import sweep_collector as sc  # noqa: E402

FRY = open(os.path.join(HERE, "testdata", "sweep_curve_fry_20261127.txt")).read().splitlines()
# as the MQTT log mirror carries them, with other device output interleaved
NOISE = "V=63.9/27.72 I=7.83/17.53A 501.0W st=MPPT,1 lag=2119"


def feed_all(lines):
    p = sc.SweepCurveParser()
    out = [r for ln in lines if (r := p.feed(ln))]
    return out


def test_real_fry_reply():
    (r,) = feed_all([NOISE, "I (4919852) main: received serial command: 'sweep-curve'"] + FRY + [NOISE])
    assert r["complete"]
    c = r["curve"]
    assert (c["t"], c["n"], c["mpp_P"], c["mpp_V"], c["sweeping"]) == (1795775998, 60, 484.3, 63.6, False)
    assert c["flag"] == "" and c["land"] == 482.1
    assert len(c["points"]) == 60 and c["points"][0] == (0, 30.57, 269.8) and c["points"][-1] == (59, 77.52, 0.0)
    assert c["raw"] == FRY


def test_flag_light_and_no_verdict():
    h = sc.parse_sc_line("sc: t=5 n=1 mpp=1.0,2.0 sweeping=0 flag=light land=3.5")
    assert h["flag"] == "light" and h["land"] == 3.5
    h = sc.parse_sc_line("\x1b[0;32msc: t=5 n=1 mpp=1.0,2.0 sweeping=0\x1b[0m")
    assert h["kind"] == "header" and "flag" not in h


def test_inf_nan_dropped_keep_index():
    (r,) = feed_all(["sc: t=9 n=3 mpp=5.0,2.0 sweeping=0", "sc: 1.00 1.0 inf 2.0 3.00 nan", "sc: end"])
    assert r["complete"] and r["curve"]["points"] == [(0, 1.0, 1.0)] and r["curve"]["dropped"] == 2


def test_short_and_truncated():
    (r,) = feed_all(["sc: t=9 n=3 mpp=5.0,2.0 sweeping=0", "sc: 1.00 1.0 2.00 2.0", "sc: end"])
    assert not r["complete"] and r["reason"] == "short"
    (r,) = feed_all(["sc: t=9 n=2 mpp=5.0,2.0 sweeping=0", "sc: 1.00 1.0 2.00 2.0 3.0", "sc: end"])
    assert not r["complete"] and r["reason"] == "truncated"
    (r,) = feed_all(["sc: t=9 n=2 mpp=5.0,2.0 sweeping=0", "sc: 1.00 1.0", "sc: t=10 n=0 mpp=0,0 sweeping=1"])
    assert r["reason"] == "restart"


def test_invalid_and_none():
    assert sc.parse_sc_line("hello") is None
    assert sc.parse_sc_line("sc: t=x n=1 mpp=1,2")["kind"] == "invalid"
    assert sc.parse_sc_line("sc: t=1 n=1 mpp=1")["kind"] == "invalid"
    assert sc.parse_sc_line("disc: 1 2") is None
    assert feed_all(["sc: none"]) == [{"complete": True, "curve": None, "reason": None}]


def test_line_protocol():
    (r,) = feed_all(FRY)
    lines = sc.curve_lines("fry", r["curve"], r["curve"]["t"])
    assert len(lines) == 60
    assert lines[0] == ('sweep,device=fry,src=fw i=0i,V=30.57,P=269.8,n=60i,mpp_V=63.6,mpp_P=484.3,'
                        'land=482.1,flag="" 1795775998000')
    assert lines[59] == "sweep,device=fry,src=fw i=59i,V=77.52,P=0.0 1795775998059"


class SyncWriter:
    """Writer stand-in: records what the collector submits."""

    def __init__(self):
        self.writes, self.recs = [], []

    def submit(self, rec, lines):
        self.recs.append(rec)
        self.writes.append(lines)

    def pending(self):
        return 0


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make(tmp_path, writer=None):
    clk, sent = Clock(), []
    col = sc.Collector(writer or SyncWriter(), sc.Archive(str(tmp_path)), sent.append, clock=clk)
    return col, clk, sent


def archived(tmp_path):
    return [json.loads(x) for fn in sorted(tmp_path.glob("*.jsonl")) for x in open(fn)]


def test_request_flow_dedup_and_archive(tmp_path):
    col, clk, sent = make(tmp_path)
    col.on_line("fry", "I (1) mppt: Stop sweep 10.75s mode=CV PWM=3782 MPP=(484.3W,1845,63.6V)")
    clk.t += 1; col.tick()
    assert sent == []
    col.on_line("fry", "I (2) mppt: sweep verified: landed 482 W, off by -0 %")
    clk.t += 2.1; col.tick()
    assert sent == ["fry"]
    for ln in FRY:
        col.on_line("fry", ln)
    clk.t += 10; col.tick()
    assert sent == ["fry"] and len(col.writer.writes) == 1 and len(col.writer.writes[0]) == 60
    # a second reply (dashboard) and a restarted collector both skip it
    for ln in FRY:
        col.on_line("fry", ln)
    assert len(col.writer.writes) == 1
    sc.Archive(str(tmp_path)).mark(col.writer.recs[0], "ok")
    col2, _, _ = make(tmp_path)
    for ln in FRY:
        col2.on_line("fry", ln)
    assert col2.writer.writes == []
    (rec,) = archived(tmp_path)
    assert rec["device"] == "fry" and rec["n"] == 60 and len(rec["points"]) == 60 and rec["raw"] == FRY


def test_interleaved_devices(tmp_path):
    col, clk, sent = make(tmp_path)
    other = [ln.replace("t=1795775998", "t=1795775999") for ln in FRY]
    for a, b in zip(FRY, other):
        col.on_line("fry", a)
        col.on_line("flat", NOISE)
        col.on_line("flat", b)
    assert [r["device"] for r in col.writer.recs] == ["fry", "flat"]
    assert all(len(w) == 60 for w in col.writer.writes)


def test_two_replies_interleaved_on_one_device(tmp_path):
    col, clk, sent = make(tmp_path)
    # second reply's header arrives inside the first: the first is abandoned, the second kept
    for ln in FRY[:3] + FRY + FRY[3:]:
        col.on_line("fry", ln)
    assert len(col.writer.writes) == 1 and len(col.writer.writes[0]) == 60
    assert col.writer.recs[0]["raw"] == FRY


def test_verdict_after_first_fetch_updates_point0(tmp_path):
    col, clk, sent = make(tmp_path)
    noflag = [FRY[0].replace(" flag=0 land=482.1", "")] + FRY[1:]
    for ln in noflag:
        col.on_line("fry", ln)
    for ln in FRY:
        col.on_line("fry", ln)
    assert [len(w) for w in col.writer.writes] == [60, 1]
    assert 'flag=""' in col.writer.writes[1][0] and 'flag' not in col.writer.writes[0][0]


def test_t0_verdict_update_keeps_first_base(tmp_path):
    col, clk, _ = make(tmp_path)
    curve = ["sc: t=0 n=1 mpp=1.0,2.0 sweeping=0", "sc: 1.00 1.0", "sc: end"]
    for ln in curve:
        col.on_line("bench", ln)
    clk.t += 7
    for ln in [curve[0] + " flag=0 land=1.0"] + curve[1:]:
        col.on_line("bench", ln)
    w = col.writer.writes
    assert w[0][0].endswith(" 1000000") and w[1][0].endswith(" 1000000") and "flag" in w[1][0]
    recs = archived(tmp_path)
    assert [sc.record_base(r) for r in recs] == [1000, 1000]
    assert sc.curve_lines("bench", sc.curve_from_record(recs[1]), sc.record_base(recs[1]))[0] == w[1][0]


def test_retries_then_gives_up(tmp_path):
    col, clk, sent = make(tmp_path)
    col.on_line("flat", "I (1) mppt: Stop sweep 12.55s")
    for _ in range(40):
        clk.t += 1; col.tick()
    assert sent == ["flat"] * (1 + sc.RETRIES)
    assert col.devs["flat"].tries == 0 and col.devs["flat"].due is None


def test_sweeping_reply_retries_counted_apart(tmp_path):
    col, clk, sent = make(tmp_path)
    col.on_line("flat", "I (1) mppt: sweep flagged: landed 279 W, off by -30 %")
    busy = ["sc: t=0 n=1 mpp=1.0,2.0 sweeping=1", "sc: 1.00 1.0", "sc: end"]
    for _ in range(sc.SWEEPING_TRIES + 2):
        clk.t += 3; col.tick()
        clk.t += 4; col.tick()  # one timeout, too
        for ln in busy:
            col.on_line("flat", ln)
    d = col.devs["flat"]
    assert col.writer.writes == [] and d.tries == 0  # gave up after SWEEPING_TRIES busy replies
    assert len(sent) > sc.RETRIES + 1


class FakeInflux:
    def __init__(self, fail=False):
        self.writes, self.fail, self.calls = [], fail, 0

    def write(self, lines):
        self.calls += 1
        if self.fail == "reject":
            raise sc.InfluxRejected("HTTP 400")
        if self.fail:
            raise IOError("down")
        self.writes.append(lines)


class BlackholeInflux:
    def __init__(self):
        self.release = threading.Event()

    def write(self, lines):
        self.release.wait(10)
        raise IOError("timeout")


def wait_for(cond, s=3.0):
    end = time.time() + s
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_blackholed_influx_does_not_block_fetches(tmp_path):
    bh = BlackholeInflux()
    arch = sc.Archive(str(tmp_path))
    w = sc.Writer(bh, arch, backoff_s=0.01)
    w.start()
    col, clk, sent = make(tmp_path, w)
    t0 = time.time()
    for ln in FRY:
        col.on_line("fry", ln)
    col.on_line("flat", "I (1) mppt: sweep verified: landed 1 W, off by -0 %")
    clk.t += 3; col.tick()
    assert sent == ["flat"] and time.time() - t0 < 1
    assert len(archived(tmp_path)) == 1 and w.pending() == 1
    bh.release.set(); w.stop()


def test_writer_retries_marks_and_rejects(tmp_path):
    arch = sc.Archive(str(tmp_path))
    inf = FakeInflux(fail=True)
    w = sc.Writer(inf, arch, backoff_s=0.02)
    w.start()
    col, clk, _ = make(tmp_path, w)
    for ln in FRY:
        col.on_line("fry", ln)
    assert wait_for(lambda: inf.calls >= 2) and w.pending() == 1 and not list(tmp_path.glob("*.written"))
    inf.fail = False
    assert wait_for(lambda: w.pending() == 0)
    (wfn,) = tmp_path.glob("*.written")
    assert open(wfn).read().split()[1] == "ok"
    inf.fail = "reject"
    for ln in [FRY[0].replace("t=1795775998", "t=1795775000")] + FRY[1:]:
        col.on_line("fry", ln)
    assert wait_for(lambda: w.pending() == 0) and inf.calls >= 4
    assert open(wfn).read().split()[-1] == "rejected"
    w.stop()


def test_restart_requeues_unwritten(tmp_path):
    col, clk, _ = make(tmp_path)
    for ln in FRY:
        col.on_line("fry", ln)
    for ln in [FRY[0].replace("t=1795775998", "t=1795775000")] + FRY[1:]:
        col.on_line("fry", ln)
    sc.Archive(str(tmp_path)).mark(col.writer.recs[0], "ok")  # only the first one was written
    col2, _, _ = make(tmp_path)
    assert [r["t"] for r in col2.writer.recs] == [1795775000] and len(col2.writer.writes[0]) == 60
    assert len(col2.stored) == 2


def test_replay_marks(tmp_path, monkeypatch):
    col, clk, _ = make(tmp_path)
    for ln in FRY:
        col.on_line("fry", ln)
    good = FakeInflux()
    monkeypatch.setattr(sc, "influx_from_env", lambda: good)
    (fn,) = tmp_path.glob("*.jsonl")
    sc.replay([str(fn)], sc.Archive(str(tmp_path)))
    assert good.writes == col.writer.writes
    col2, _, _ = make(tmp_path)
    assert col2.writer.recs == []


def test_unknown_time_uses_receive_time(tmp_path):
    col, clk, _ = make(tmp_path)
    for ln in ["sc: t=0 n=1 mpp=1.0,2.0 sweeping=0", "sc: 1.00 1.0", "sc: end"]:
        col.on_line("bench", ln)
    assert col.writer.writes[0][0].endswith(" 1000000")
    assert math.isclose(archived(tmp_path)[0]["recv_ts"], 1000.0)


def test_stored_pruned(tmp_path):
    col, clk, _ = make(tmp_path)
    for ln in FRY:
        col.on_line("fry", ln)
    clk.t += sc.KEEP_KEYS_S + 3700; col.tick()
    assert col.stored == {}


def test_second_instance_refused(tmp_path):
    lk = sc.lock_archive(str(tmp_path))
    try:
        sc.lock_archive(str(tmp_path))
        assert False, "second lock taken"
    except SystemExit as e:
        assert "--no-send" in str(e)
    lk.close()
