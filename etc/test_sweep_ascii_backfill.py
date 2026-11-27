#!/usr/bin/env python3
"""Unit test of etc/sweep_ascii_backfill.py: two real fry sweep charts from the havan console log (2026-11-27, with
interleaved flat/bench139c lines), and charts rendered by `chart()`, a port of src/asciichart/ascii.h
Asciichart::Plot, decoded against their known series.

    .venv/bin/python3 -m pytest etc/test_sweep_ascii_backfill.py
"""
import lzma
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sweep_ascii_backfill as b  # noqa: E402

FIX = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testdata', 'sweep_ascii_fry_1127.txt')
X = np.linspace(0, 1, 100)


def _lround(x):
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


def chart(series, height=16, offset=3):
    """port of Asciichart::Plot (one series, float32), SGR-free rows"""
    s = np.float32(series)
    mn, mx = s.min(), s.max()
    rng = (mx - mn) or np.float32(1)
    label_w = max(len('%d' % int(mn)), len('%d' % int(mx)))
    ratio = np.float32(height) / rng
    min2, max2 = _lround(mn * ratio), _lround(mx * ratio)
    rows = (max2 - min2) or 1
    glyph = {1: '┼', 2: '┤', 3: '─', 4: '╰', 5: '╭', 6: '╮', 7: '╯', 8: '│'}
    out = []
    for r in range(rows + 1):
        g = [0] * (len(s) + offset)
        y_idx = rows - r
        g[offset - 1] = 1 if y_idx + min2 == 0 else 2
        if rows - (_lround(s[0] * ratio) - min2) == r:
            g[offset - 1] = 1
        for i in range(len(s) - 1):
            y0, y1 = _lround(s[i] * ratio) - min2, _lround(s[i + 1] * ratio) - min2
            sy0, sy1, c = rows - y0, rows - y1, i + offset
            if y0 == y1:
                g[c] = 3 if sy0 == r else g[c]
            elif sy1 == r:
                g[c] = 4 if y0 > y1 else 5
            elif sy0 == r:
                g[c] = 6 if y0 > y1 else 7
            elif min(sy0, sy1) < r < max(sy0, sy1):
                g[c] = 8
        lab = _lround(mn + np.float32(y_idx) * rng / np.float32(rows))
        out.append('%*d' % (label_w, lab) + ' ' * (offset - 2) + ''.join(glyph.get(v, ' ') for v in g[offset - 1:]))
    return out


def pv_curve(pk, vf=.6, low=.6, drop=.3, off=0.):
    return (pk * np.where(X < vf, low + (1 - low) * X / vf, np.clip(1 - (X - vf) / drop, 0, 1)) + off).astype(
        np.float32)


def decode(p, g0=True):
    return b.decode_chart(chart(p), round(float(p[0]), 2) if g0 else None)


@pytest.fixture(scope='module')
def sweeps():
    return b.parse(b.read_lines(FIX))


def test_real_charts(sweeps):
    assert [(s.dev, b.utc(s.ts)) for s in sweeps] == [('fry', '2026-11-27 07:31:26'), ('fry', '2026-11-27 08:42:48')]
    dawn, s = sweeps
    # 3.5 W, labels 3..-1, the '-1' row one column wider: zero row and labels bound the scale
    assert dawn.reason is None and dawn.land == (52.7, 3.2) and dawn.verdict is None
    assert 0.2 < dawn.step < 0.3 and 0.5 * dawn.step < dawn.err <= b.ERR_MAX * dawn.step
    assert abs(dawn.P.max() - 3.5) <= dawn.err and dawn.P[0] == pytest.approx(2.29, abs=dawn.err)
    assert dawn.P[98] == pytest.approx(dawn.P.min()) and dawn.P.min() < 0
    assert s.reason is None and s.mpp == (90.9, 47.9) and s.g0 == 69.02
    assert s.step == pytest.approx((91 - 17) / 16, abs=0.3) and s.err <= s.step
    i = int(np.argmax(s.P))
    assert abs(s.P[i] - 90.9) <= s.step and abs(s.V[i] - 47.9) < 2 * (78.95 - 30.05) / 100
    assert s.P[0] == pytest.approx(69.02, abs=s.err)
    assert (s.verdict, s.off, s.land) == ('verified', 1.0, (46.9, 91.9))


def test_device_filter():
    assert b.parse(b.read_lines(FIX), {'flat'}) == []
    assert len(b.parse(b.read_lines(FIX), {'fry'})) == 2


@pytest.mark.parametrize('pk', [91, 20, 8])
def test_ground_truth(pk):
    rnd = np.random.default_rng(pk)
    for _ in range(20):
        p = pv_curve(pk * rnd.uniform(.9, 1.1), rnd.uniform(.3, .8), drop=rnd.uniform(.02, .4)) + \
            rnd.normal(0, pk * .01, 100).astype(np.float32)
        P, step, err = decode(p)
        if P is None:
            assert pk < 20, step
            continue
        assert step == pytest.approx((p.max() - p.min()) / 16, rel=.1)
        assert np.max(np.abs(P - p)) <= err + 1e-6 and err <= b.ERR_MAX * step + 1e-9


def test_negative_label_wider_row():
    p = np.float32(np.where(X < .6, 4 + 5 * X / .6, 9 - 9.6 * (X - .6) / .4))  # -0.6 .. 9 W: '-1' row one column wider
    rows = chart(p)
    assert rows[-1].startswith('-1 ┤') and rows[-2].startswith('0 ┼')
    P, step, err = decode(p)
    assert P is not None and np.max(np.abs(P - p)) <= step and P.min() < 0


def test_sub_2w_ramp_rejected():
    # 0.25 -> 0.5 -> 0.25 W, MPP 0.5 W at bin 59: labels 1,0,...,0 pin nothing
    p = np.float32(np.interp(np.arange(100), [0, 59, 99], [.25, .5, .25]))
    for g0 in (True, False):
        P, step, err = decode(p, g0)
        assert P is None or np.max(np.abs(P - p)) <= (p.max() - p.min()) / 16, (step, err)
    assert decode(p, False)[0] is None


def test_lost_vertical_row_rejected():
    rnd = np.random.default_rng(1)
    tried = 0
    for pk in (1.5, 3, 5, 20, 91):
        p = pv_curve(pk, drop=.02) + rnd.normal(0, pk * .01, 100).astype(np.float32)
        rows = chart(p)
        for k, r in enumerate(rows):
            if any(c in b.NEW + b.OLD for c in b.ROW.match(r).group(3)):
                continue
            tried += 1
            assert b.decode_chart(rows[:k] + rows[k + 1:], round(float(p[0]), 2))[0] is None
            assert b.decode_chart(rows[:k + 1] + rows[k:], round(float(p[0]), 2))[0] is None  # doubled
    assert tried > 10


def test_time_zone():
    utc = b.ms_of('2026-11-27 08:42:48', 236, b.timezone.utc)
    assert b.ms_of('2026-11-27 08:42:48', 236) == utc  # WET = UTC in winter
    assert b.ms_of('2026-07-01 12:00:00', 0) == b.ms_of('2026-07-01 11:00:00', 0, b.timezone.utc)  # WEST
    s = b.parse(b.read_lines(FIX))[1]
    assert s.ts == b.ms_of('2026-11-27 08:42:48', 391, b.timezone.utc)


def test_no_footer():
    lines = b.read_lines(FIX)
    foot = [i for i, l in enumerate(lines) if 'P|V' in l]
    eof = b.parse(lines[:foot[1]])  # log ends inside the second chart
    assert eof[-1].reason == 'no P|V footer (end of log)' and eof[-1].ts == b.ms_of('2026-11-27 08:42:48', 236)
    cut = lines[:foot[0]] + lines[foot[0] + 1:]  # first footer lost, next sweep's Grouping closes it
    assert [s.reason or '' for s in b.parse(cut)][0] == 'no P|V footer'


def test_verdict_time_order():
    lines = b.read_lines(FIX)
    v = next(l for l in lines if 'sweep verified' in l)
    early = v.replace('08:42:49,929', '08:42:40,000')
    s = b.parse([l for l in lines if l != v] + [early])[1]
    assert s.verdict is None


def test_xz_input(tmp_path):
    f = tmp_path / 'fugu_console.log.1.xz'
    with open(FIX, 'rb') as src, lzma.open(f, 'wb') as dst:
        dst.write(src.read())
    assert [s.ts for s in b.parse(b.read_lines(str(f)))] == [s.ts for s in b.parse(b.read_lines(FIX))]
    with pytest.raises(SystemExit):
        b.read_lines(str(tmp_path / 'x.log.gz'))


def test_line_protocol(sweeps):
    s = sweeps[1]
    lines = list(b.to_lines(s))
    assert len(lines) == b.BINS
    ts = [int(l.rsplit(' ', 1)[1]) for l in lines]
    assert ts == list(range(s.ts, s.ts + b.BINS))
    assert lines[0].startswith('sweep_ascii,device=fry i=0i,V=30.050,')
    assert 'mpp_V=47.9,mpp_P=90.9,' in lines[0] and 'verdict="verified"' in lines[0] and 'curve="mpp"' in lines[0]
    assert 'p_err=' in lines[0] and 'mpp_P' not in lines[1] and lines[99].split(' ')[1].startswith('i=99i,')
    assert b.esc_tag('a b,c=d') == r'a\ b\,c\=d'


@pytest.mark.parametrize('off', [0.2, 0.5, 1.0, 2.0])
def test_g0_off_bin0(off):
    # bin 0 averaging several points: the Grouping line's first power is not bin 0, the bound must still hold
    rnd = np.random.default_rng(int(off * 10))
    for _ in range(15):
        pk = float(rnd.choice([3.5, 8, 20, 91]))
        p = pv_curve(pk, rnd.uniform(.3, .8), drop=rnd.uniform(.02, .4)) + rnd.normal(0, pk * .01, 100).astype(np.float32)
        g0 = round(float(p[0] + rnd.choice([-1, 1]) * off * (p.max() - p.min()) / 16), 2)
        P, step, err = b.decode_chart(chart(p), g0)
        assert P is None or np.max(np.abs(P - p)) <= err + 1e-6
