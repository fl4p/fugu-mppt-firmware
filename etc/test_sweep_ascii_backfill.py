#!/usr/bin/env python3
"""Unit test of etc/sweep_ascii_backfill.py on two real fry sweep charts from the havan console log (2026-11-27,
with interleaved flat/bench139c lines): a 3.5 W dawn sweep whose bottom row is labelled -1 (one column wider than
the others) and a 91 W sweep with a verdict line.

    .venv/bin/python3 -m pytest etc/test_sweep_ascii_backfill.py
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sweep_ascii_backfill as b  # noqa: E402

FIX = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testdata', 'sweep_ascii_fry_1127.txt')


@pytest.fixture(scope='module')
def sweeps():
    return b.parse(b.read_lines(FIX))


def test_two_sweeps_accepted(sweeps):
    assert [(s.dev, b.utc(s.ts), s.reason) for s in sweeps] == [
        ('fry', '2026-11-27 07:31:26', None), ('fry', '2026-11-27 08:42:48', None)]


def test_negative_label_chart(sweeps):
    s = sweeps[0]
    assert s.mpp == (3.5, 54.6)
    assert s.V[0] == pytest.approx(31.26) and s.V[1] - s.V[0] == pytest.approx((66.52 - 31.26) / 100)
    assert abs(s.P.max() - 3.5) <= s.step
    assert 0.1 < s.step < 0.3
    # the -1 row (one column wider) holds the last bins: lowest value, right end of the curve
    assert s.P[98] == pytest.approx(s.P.min()) and s.P.min() < 0
    assert s.P[0] == pytest.approx(s.P[1]) and 1.8 < s.P[0] < 2.6  # bin 0 on the '2 ┼─╯' row
    assert s.land == (52.7, 3.2) and s.verdict is None


def test_large_chart_and_verdict(sweeps):
    s = sweeps[1]
    assert s.mpp == (90.9, 47.9)
    assert s.step == pytest.approx((91 - 17) / 16, abs=0.3)
    i = int(np.argmax(s.P))
    assert abs(s.P[i] - 90.9) <= s.step and abs(s.V[i] - 47.9) < 2 * (78.95 - 30.05) / 100
    assert (s.verdict, s.off, s.land) == ('verified', 1.0, (46.9, 91.9))


def test_device_filter(sweeps):
    assert b.parse(b.read_lines(FIX), {'flat'}) == []
    assert len(b.parse(b.read_lines(FIX), {'fry'})) == 2


def test_fit_labels_negative():
    labels = [3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, -1]
    mn, s = b.fit_labels(labels)
    k = np.arange(len(labels))
    assert np.all(np.abs(mn + k * s - np.array(labels[::-1])) <= 0.5 + 1e-9)
    assert b.fit_labels([5, 3, 4]) is None


def test_lost_row_rejected():
    lines = b.read_lines(FIX)
    drop = next(i for i, l in enumerate(lines) if ' 63 ' in b.SGR.sub('', l) and '┤' in l)
    s = b.parse(lines[:drop] + lines[drop + 1:])[1]
    assert s.reason


def test_line_protocol(sweeps):
    s = sweeps[1]
    lines = list(b.to_lines(s))
    assert len(lines) == b.BINS
    ts = [int(l.rsplit(' ', 1)[1]) for l in lines]
    assert ts == list(range(s.ts, s.ts + b.BINS))
    assert lines[0].startswith('sweep_ascii,device=fry i=0i,V=30.050,')
    assert 'mpp_V=47.9,mpp_P=90.9,' in lines[0] and 'verdict="verified"' in lines[0] and 'curve="mpp"' in lines[0]
    assert 'mpp_P' not in lines[1] and lines[99].split(' ')[1].startswith('i=99i,')
