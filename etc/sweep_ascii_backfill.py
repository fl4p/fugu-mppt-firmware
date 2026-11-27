#!/usr/bin/env python3
"""Backfill MPPT sweep curves (P vs Vin) from the ASCII charts in the havan console log into InfluxDB.

usage: etc/sweep_ascii_backfill.py [-n] [--devices fry,flat] [--env-file F] LOG...

LOG: console log copies (`fugu_console.log*`, `.xz`, `.br` via the brotli CLI), any order; they are read in time order
and a sweep found in two files counts once. Copy them first, e.g. `rsync -a havan-ts:'pv/fugu_console.log*'
havan-ts:'pv/archive/*.xz' /tmp/logs/`. Log times are havan's local time (Python logging asctime, Europe/Lisbon,
--tz) and are converted to UTC.

Each sweep prints `Stop sweep ... MPP=(P W,pwm,V V)`, `Grouping n V points (x0,y0)~(x1,y1) into 100 bins`, a
16-row asciichart (src/asciichart/ascii.h) of the 100 bin averages and the footer `P|V x0 .. x1`. Bin i sits at
Vin = x0 + i (x1 - x0) / 100 (src/etc/plot.h). Chart column i draws the segment bin i -> i+1: '─╭╰' mark the row of
bin i+1, '─╮╯' that of bin i. With mn, mx the series min and max, r = 16 / (mx - mn), a bin of power v sits on row
lround(v r) - lround(mn r) (from the bottom), rows = lround(mx r) - lround(mn r), row k has the label
lround(mn + k (mx - mn) / rows), the axis shows '┼' on the zero row and on bin 0's row. (mn, mx) is solved from all
of these plus the Grouping line's first power (bin 0 normally holds only that point); P is the middle of the values
every consistent (mn, mx) allows, `p_err` its half width (W), `row_step` = (mx - mn) / 16 (W). A chart is rejected
when no (mn, mx) fits (lost or doubled rows, bad labels), p_err exceeds ERR_MAX row steps (under-constrained,
typically a sub-2 W chart with near-constant integer labels), a column has no glyph or two neighbouring columns
disagree, or the curve maximum misses the logged MPP by more than one row step in P or 2 bins in Vin.

Influx (v1, db ha_van, measurement sweep_ascii, tag device): one point per bin with fields i, V, P; point 0 also
mpp_V, mpp_P, row_step, p_err, curve and, when logged, verdict (verified|flagged|unverified), off_pct, land_V, land_P (the last
`sweep landed` line). Time = log time of the footer (converted to UTC) + i ms, so a re-run overwrites instead of duplicating.
curve: 'mpp' when the chart plots the MPP's own quantities (EWM Vin, power without the input caps' discharge,
c4805eb8), 'raw' before (med3 Vin, power with the discharge: on fast or dim sweeps the chart peak sits above and
left of the MPP, so more of those fail the MPP check). The switch is the first fleet OTA carrying c4805eb8
(elf-archive/ota-owners.jsonl, times in UTC), CURVE_MPP_FROM.
Credentials: $INFLUX_HA_VAN_USER / $INFLUX_HA_VAN_PASS, or --env-file (default: the main checkout's
.claude/memory/secrets.env).
"""
import argparse
import base64
import collections
import lzma
import os
import re
import subprocess
import sys
import urllib.request
from datetime import datetime, timezone
from zoneinfo import ZoneInfo

import numpy as np

BINS = 100
HEIGHT = 16
SGR = re.compile(r'\x1b\[[0-9;]*m')
LINE = re.compile(r'^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d),(\d{3})\s+(\S+): (.*)$')
STOP = re.compile(r'mppt: Stop sweep .*MPP=\(([-\d.]+)W,(?:PWM=)?\d+,([-\d.]+)V\)')
GROUP = re.compile(r'mppt: Grouping \d+ V points \(([-\d.]+),([-\d.naif]+)\)~\(([-\d.]+),[-\d.naif]+\) into (\d+) bins')
ROW = re.compile(r'^\s*(-?\d+) ([┤┼])(.*)$')
FOOT = re.compile(r'^\s*P\|V\s')
START = re.compile(r'mppt: Start sweep')
LANDED = re.compile(r'mppt: sweep landed ([-\d.]+) V ([-\d.]+) W')
VERDICT = re.compile(r'mppt: sweep (verified|flagged): landed [-\d.]+ W, off by ([-+\d.]+) %')
UNVER = re.compile(r'mppt: sweep not verified')
NEW, OLD = '─╭╰', '─╮╯'
CURVE_MPP_FROM = {'fry': '2026-11-24 15:21:09', 'flat': '2026-11-24 15:25:00'}  # UTC, from ota-owners.jsonl
ERR_MAX = 1.0  # p_err limit, row steps
LOG_TZ = ZoneInfo('Europe/Lisbon')


def ms_of(day_time, ms, tz=None):
    """'YYYY-mm-dd HH:MM:SS' in tz (default LOG_TZ) -> epoch ms"""
    d = datetime.strptime(day_time, '%Y-%m-%d %H:%M:%S').replace(tzinfo=tz or LOG_TZ)
    return int(d.timestamp()) * 1000 + int(ms)


def lround(x):
    return np.where(x >= 0, np.floor(x + np.float32(.5)), -np.floor(-x + np.float32(.5))).astype(np.int64)


def decode_chart(rows, g0=None):
    """chart rows (SGR-free, top first), Grouping's first power g0 -> (P[100], row step W, p_err W) or (None, reason)"""
    parsed = []
    for r in rows:
        m = ROW.match(r)
        if not m:
            return None, 'bad row', None
        parsed.append((int(m.group(1)), m.group(2), m.group(3)))
    R = len(parsed) - 1
    if R < 2:
        return None, f'{R + 1} rows', None
    new, old = [None] * BINS, [None] * BINS
    for c in range(BINS - 1):
        for k, (_, _, cells) in enumerate(parsed):
            ch = cells[c] if c < len(cells) else ' '
            if ch in NEW:
                if new[c] is not None:
                    return None, f'col {c}: two rows', None
                new[c] = k
            if ch in OLD:
                if old[c] is not None:
                    return None, f'col {c}: two rows', None
                old[c] = k
    for c in range(BINS - 1):
        if new[c] is None or old[c] is None:
            return None, f'col {c}: no glyph', None
        if c and old[c] != new[c - 1]:
            return None, f'col {c}: discontinuous', None
    yb = R - np.array([old[0]] + new[:BINS - 1])  # row of each bin, from the bottom
    if yb.min() != 0 or yb.max() != R:
        return None, 'top or bottom row empty', None
    plus = {R - k for k, (_, ax, _) in enumerate(parsed) if ax == '┼'}
    if int(yb[0]) not in plus:
        return None, 'no first-sample marker', None
    extra = plus - {int(yb[0])}
    if len(extra) > 1:
        return None, 'two zero rows', None
    z = extra.pop() if extra else None
    L = np.array([p[0] for p in parsed[::-1]], float)
    k = np.arange(R + 1)
    d, dk = L[None, :] - L[:, None], k[None, :] - k[:, None]
    up = dk > 0
    s_lo, s_hi = max(np.max((d[up] - 1) / dk[up]), 1e-6), np.min((d[up] + 1) / dk[up])
    if s_lo > s_hi:
        return None, 'labels inconsistent', None
    # candidate (mn, label pitch): a grid over the label-feasible set plus lines through the zero row and g0
    S, M = [], []
    for s in np.linspace(s_lo, s_hi, 121) if s_hi - s_lo > 1e-9 else [s_lo]:
        lo, hi = np.max(L - .5 - k * s), np.min(L + .5 - k * s)
        if lo > hi:
            continue
        q = s * R / 16  # row step
        y0 = -z if z is not None else -yb[0]
        ms = [np.linspace(lo, hi, 121), np.linspace((y0 - .5) * q, (y0 + .5) * q, 41)]
        if g0 is not None:
            ms.append(np.linspace(g0 - (yb[0] + .5) * q, g0 - (yb[0] - .5) * q, 41))
        mm = np.concatenate(ms)
        mm = mm[(mm >= lo) & (mm <= hi)]
        S.append(np.full(len(mm), s))
        M.append(mm)
    if not S:
        return None, 'labels inconsistent', None
    mn = np.concatenate(M).astype(np.float32)
    rng = (np.concatenate(S) * R).astype(np.float32)
    mx = mn + rng
    ratio = np.float32(16) / rng
    min2 = lround(mn * ratio)
    ok = lround(mx * ratio) - min2 == R
    for v in (mn * ratio, mx * ratio):  # a rounding tie: float noise could pass a lost or doubled row
        ok &= np.abs(np.abs(v - np.floor(v)) - .5) > 1e-3
    lab = lround(mn[:, None] + k[None, :].astype(np.float32) * rng[:, None] / np.float32(R))
    ok &= np.all(lab == L[None, :], axis=1)
    if z is not None:
        ok &= min2 == -z
    else:
        ok &= (min2 == -yb[0]) | (min2 > 0) | (min2 + R < 0)
    if not ok.any():
        return None, 'no consistent scale (lost or doubled row?)', None
    if g0 is not None:
        tol = np.float32(.006)
        okg = ok & (np.abs(g0 * ratio - min2 - yb[0]) <= .5 + tol * ratio) & (mn <= g0 + tol) & (mx >= g0 - tol)
        if okg.any():
            ok = okg
    mn, mx, ratio, min2 = mn[ok, None], mx[ok, None], ratio[ok, None], min2[ok, None]
    lo = np.maximum(mn, (yb[None, :] + min2 - .5) / ratio).min(axis=0)
    hi = np.minimum(mx, (yb[None, :] + min2 + .5) / ratio).max(axis=0)
    P, err = (lo + hi) / 2, float(np.max(hi - lo) / 2)
    step = float(np.median(1 / ratio))
    if err > ERR_MAX * step:
        return None, f'under-constrained scale (err {err:.3f} W, step {step:.3f})', None
    return P.astype(float), step, err


class Sweep:
    __slots__ = ('dev', 'ts', 'x0', 'x1', 'g0', 'rows', 'mpp', 'P', 'V', 'step', 'err', 'reason', 'land', 'verdict',
                 'off')

    def __init__(self, dev, x0, x1, mpp, g0=None, ts=None):
        self.dev, self.x0, self.x1, self.mpp, self.g0, self.ts, self.rows = dev, x0, x1, mpp, g0, ts, []
        self.P = self.V = self.step = self.err = self.reason = self.land = self.verdict = self.off = None

    def finish(self, ts):
        self.ts = ts
        if not self.mpp:
            self.reason = 'no Stop sweep line'
            return
        P, step, err = decode_chart(self.rows, self.g0)
        if P is None:
            self.reason = step
            return
        self.V = self.x0 + np.arange(BINS) * (self.x1 - self.x0) / BINS
        self.P, self.step, self.err = P, step, err
        mpp_p, mpp_v = self.mpp
        pmax = P.max()
        if abs(pmax - mpp_p) > step:
            self.reason = f'max {pmax:.1f} W vs MPP {mpp_p:.1f} W (step {step:.2f})'
            return
        dv = np.abs(self.V[P >= pmax - 1e-9] - mpp_v).min()
        if dv > 2 * (self.x1 - self.x0) / BINS:
            self.reason = f'max at {dv:.2f} V from MPP {mpp_v:.1f} V'


def read_lines(path):
    if path.endswith('.br'):
        data = subprocess.run(['brotli', '-d', '-c', path], capture_output=True, check=True).stdout
    elif path.endswith('.xz'):
        with lzma.open(path) as f:
            data = f.read()
    elif re.search(r'\.(gz|bz2|zst|lz4|zip|7z)$', path):
        raise SystemExit(f'{path}: unsupported compression')
    else:
        with open(path, 'rb') as f:
            data = f.read()
    return data.decode('utf-8', 'replace').splitlines()


def parse(lines_iter, devices=None):
    """log lines in time order -> list of Sweep (accepted and rejected)"""
    out, chart, stop, last = [], {}, {}, {}
    for raw in lines_iter:
        m = LINE.match(SGR.sub('', raw))
        if not m:
            continue
        dt, ms, dev, msg = m.groups()
        if devices and dev not in devices:
            continue
        t = ms_of(dt, ms)
        sw = chart.get(dev)
        if sw is not None:
            if ROW.match(msg):
                sw.rows.append(msg)
                continue
            if FOOT.match(msg):
                sw.finish(t)
                out.append(sw)
                last[dev] = sw
                del chart[dev]
                continue
            if 'Grouping' in msg or START.search(msg):
                sw.reason, sw.ts = 'no P|V footer', t
                out.append(sw)
                del chart[dev]
            # other lines of this device interleave: keep collecting rows
        if (m2 := STOP.search(msg)):
            stop[dev] = (t, float(m2.group(1)), float(m2.group(2)))
        elif (m2 := GROUP.search(msg)):
            st = stop.pop(dev, None)
            mpp = (st[1], st[2]) if st and t - st[0] < 5000 else None
            g0 = float(m2.group(2)) if re.fullmatch(r'-?\d+(\.\d*)?', m2.group(2)) else None
            chart[dev] = Sweep(dev, float(m2.group(1)), float(m2.group(3)), mpp, g0, t)
            last.pop(dev, None)
        elif START.search(msg):
            last.pop(dev, None)
        elif (sw := last.get(dev)) is not None and 0 <= t - sw.ts < 120000:
            if (m2 := LANDED.search(msg)):
                sw.land = (float(m2.group(1)), float(m2.group(2)))
            elif (m2 := VERDICT.search(msg)):
                sw.verdict, sw.off = m2.group(1), float(m2.group(2))
                last.pop(dev)
            elif UNVER.search(msg):
                sw.verdict = 'unverified'
                last.pop(dev)
    for sw in chart.values():  # still open at the end of the input
        sw.reason = 'no P|V footer (end of log)'
        out.append(sw)
    return out


def ordered_lines(paths):
    def first_ts(p):
        for l in read_lines(p)[:50]:
            m = LINE.match(SGR.sub('', l))
            if m:
                return m.group(1)
        return ''

    for p in sorted(paths, key=first_ts):
        yield from read_lines(p)


def dedupe(sweeps):
    seen, out = set(), []
    for s in sweeps:
        k = (s.dev, s.ts)
        if k not in seen:
            seen.add(k)
            out.append(s)
    return out


def curve(s):
    t = CURVE_MPP_FROM.get(s.dev)
    return 'mpp' if t and s.ts >= ms_of(t, 0, timezone.utc) else 'raw'


def utc(ms):
    return datetime.fromtimestamp(ms / 1000, timezone.utc).strftime('%Y-%m-%d %H:%M:%S')


def esc_tag(v):
    return re.sub(r'([,= \\])', r'\\\1', v)


def esc_str(v):
    return '"' + v.replace('\\', '\\\\').replace('"', '\\"') + '"'


def to_lines(s, measurement='sweep_ascii'):
    for i in range(BINS):
        f = [f'i={i}i', f'V={s.V[i]:.3f}', f'P={s.P[i]:.3f}']
        if i == 0:
            f += [f'mpp_V={s.mpp[1]}', f'mpp_P={s.mpp[0]}', f'row_step={s.step:.4f}', f'p_err={s.err:.4f}',
                  f'curve="{curve(s)}"']
            if s.verdict:
                f.append(f'verdict={esc_str(s.verdict)}')
            if s.off is not None:
                f.append(f'off_pct={s.off}')
            if s.land:
                f += [f'land_V={s.land[0]}', f'land_P={s.land[1]}']
        yield f'{esc_tag(measurement)},device={esc_tag(s.dev)} {",".join(f)} {s.ts + i}'


def load_env(path):
    if path and os.path.exists(path):
        for l in open(path):
            m = re.match(r'\s*(?:export\s+)?(\w+)=(.*)$', l)
            if m and m.group(1) not in os.environ:
                os.environ[m.group(1)] = m.group(2).strip().strip('"\'')


def write(lines, url, db, batch=5000):
    auth = base64.b64encode(f'{os.environ["INFLUX_HA_VAN_USER"]}:{os.environ["INFLUX_HA_VAN_PASS"]}'.encode()).decode()
    for k in range(0, len(lines), batch):
        req = urllib.request.Request(f'{url}/write?db={db}&precision=ms', data='\n'.join(lines[k:k + batch]).encode(),
                                     headers={'Authorization': 'Basic ' + auth}, method='POST')
        with urllib.request.urlopen(req, timeout=60) as r:
            if r.status != 204:
                raise SystemExit(f'influx write: HTTP {r.status} {r.read()[:200]}')


def main():
    common = subprocess.run(['git', 'rev-parse', '--path-format=absolute', '--git-common-dir'], capture_output=True,
                            text=True, cwd=os.path.dirname(os.path.abspath(__file__))).stdout.strip()
    main_checkout = os.path.dirname(common) if common else '.'
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('-n', '--dry-run', action='store_true')
    ap.add_argument('--devices', default='fry,flat', help='comma list, empty: all')
    ap.add_argument('--url', default='http://tm.fabi.me:8086')
    ap.add_argument('--db', default='ha_van')
    ap.add_argument('--measurement', default='sweep_ascii')
    ap.add_argument('--env-file', default=os.path.join(main_checkout, '.claude/memory/secrets.env'))
    ap.add_argument('--tz', default='Europe/Lisbon', help="the log's time zone")
    ap.add_argument('-v', '--verbose', action='store_true', help='list each rejected sweep')
    a = ap.parse_args()
    global LOG_TZ
    LOG_TZ = ZoneInfo(a.tz)
    devices = set(filter(None, a.devices.split(',')))
    sweeps = dedupe(parse(ordered_lines(a.logs), devices))
    ok = [s for s in sweeps if not s.reason]
    stats = collections.defaultdict(lambda: [0, 0, None, None])
    for s in sweeps:
        for key in (s.dev, f'{s.dev}/{curve(s)}'):
            st = stats[key]
            st[0 if not s.reason else 1] += 1
            day = utc(s.ts)[:16]
            st[2] = min(st[2] or day, day)
            st[3] = max(st[3] or day, day)
    for dev, (n_ok, n_bad, t0, t1) in sorted(stats.items()):
        print(f'{dev:9} found {n_ok + n_bad:4}  imported {n_ok:4}  rejected {n_bad:3}  '
              f'pass {100 * n_ok / max(1, n_ok + n_bad):.1f} %  {t0} .. {t1} UTC')
    reasons = collections.Counter(re.sub(r'-?\d+(\.\d+)?', '#', s.reason) for s in sweeps if s.reason)
    for r, n in reasons.most_common():
        print(f'  rejected {n:3}: {r}')
    if a.verbose:
        for s in sweeps:
            if s.reason:
                print(f'  {s.dev} {utc(s.ts)}: {s.reason}')
    verd = collections.Counter(s.verdict or '-' for s in ok)
    print('verdicts of imported sweeps:', dict(verd))
    lines = [l for s in ok for l in to_lines(s, a.measurement)]
    print(f'{len(lines)} points')
    if a.dry_run or not lines:
        return
    load_env(a.env_file)
    if not os.environ.get('INFLUX_HA_VAN_USER') or not os.environ.get('INFLUX_HA_VAN_PASS'):
        raise SystemExit(f'no $INFLUX_HA_VAN_USER/_PASS (env or {a.env_file})')
    write(lines, a.url, a.db)
    print(f'written to {a.db}.{a.measurement}')


if __name__ == '__main__':
    main()
