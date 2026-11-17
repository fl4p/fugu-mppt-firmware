#!/usr/bin/env python3
"""Pair mppt-sim runs base vs guard (same board, scenario, seed) from out/<variant>/<board>/<scenario>-s<seed>.txt.
Prints a per-scenario table over all boards and seeds and the near-unity detail per board."""
import collections
import os
import re
import sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/minpulse-batch/out'
NEAR = ('wx-dawn', 'wx-dusk', 'wx-low', 'wx-low-slow', 'wx-mp-ramp', 'wx-mp-psu', 'wx-mp-psu-nosink')


def parse(path):
    t = open(path, errors='replace').read()
    r = {}
    m = re.search(r'== PLANT TRUTH: PV energy ([\d.]+) Wh = ([\d.]+) %.*output energy (-?[\d.]+) Wh', t)
    if not m:
        return None
    r['pv'], r['harv'], r['out'] = float(m[1]), float(m[2]), float(m[3])
    m = re.search(r'== SUMMARY .*limiter\s+([\d.]+) %.*trips (\d+) \[(.*)\]', t)
    r['lim'], r['trips'], r['tl'] = float(m[1]), int(m[2]), m[3]
    r['revI'] = sum(int(x) for x in re.findall(r'revI[-\w]* x(\d+)', r['tl']))
    m = re.search(r'== REVERSE .*mean min (-?[\d.]+) A, ([\d.]+) s below -0.05 A, ([\d.]+) C back \| starts (\d+), '
                  r'minpulse-M stops (\d+) \| on at plant M > ([\d.]+) \(1 - t_min/T\) ([\d.]+) s', t)
    r['imin'], r['tneg'], r['qneg'], r['starts'], r['stops'], r['mc'], r['above'] = (
        float(m[1]), float(m[2]), float(m[3]), int(m[4]), int(m[5]), float(m[6]), float(m[7]))
    return r


runs = collections.defaultdict(dict)  # (board, sc, seed) -> {variant: r}
for v in ('base', 'wt'):
    for b in sorted(os.listdir(os.path.join(ROOT, v))):
        for f in os.listdir(os.path.join(ROOT, v, b)):
            m = re.match(r'(.*)-s(\d+)\.txt$', f)
            r = parse(os.path.join(ROOT, v, b, f))
            if m and r:
                runs[(b, m[1], int(m[2]))][v] = r

pairs = {k: d for k, d in runs.items() if 'base' in d and 'wt' in d}
print(f'{len(pairs)} paired runs ({len(runs)} keys)')

by = collections.defaultdict(list)
for (b, sc, s), d in pairs.items():
    by[sc].append(d)


def agg(rows, sel, f=sum):
    return f(sel(r) for r in rows)


print('\n| scenario | runs | harvest base / guard (mean %) | dPV mean / min [Wh] | minpulse-M stops | starts base / guard |'
      ' revI trips base / guard | on above M_c base / guard [s] | Iout min base / guard [A] | t(I<-0.05 A) base / guard [s] |'
      ' limiter base / guard [%] |')
print('|---|---|---|---|---|---|---|---|---|---|---|')
for sc in sorted(by):
    ds = by[sc]
    n = len(ds)
    hb, hw = agg(ds, lambda d: d['base']['harv']) / n, agg(ds, lambda d: d['wt']['harv']) / n
    dpv = [d['wt']['pv'] - d['base']['pv'] for d in ds]
    print(f"| {sc} | {n} | {hb:.2f} / {hw:.2f} | {sum(dpv) / n:+.3f} / {min(dpv):+.3f} | {agg(ds, lambda d: d['wt']['stops'])} |"
          f" {agg(ds, lambda d: d['base']['starts'])} / {agg(ds, lambda d: d['wt']['starts'])} |"
          f" {agg(ds, lambda d: d['base']['revI'])} / {agg(ds, lambda d: d['wt']['revI'])} |"
          f" {agg(ds, lambda d: d['base']['above']):.1f} / {agg(ds, lambda d: d['wt']['above']):.1f} |"
          f" {agg(ds, lambda d: d['base']['imin'], min):.3f} / {agg(ds, lambda d: d['wt']['imin'], min):.3f} |"
          f" {agg(ds, lambda d: d['base']['tneg']):.1f} / {agg(ds, lambda d: d['wt']['tneg']):.1f} |"
          f" {agg(ds, lambda d: d['base']['lim']) / n:.1f} / {agg(ds, lambda d: d['wt']['lim']) / n:.1f} |")

print('\nnear-unity scenarios per board (sum over seeds):')
print('| scenario | board | seeds | PV base / guard [Wh] | out base / guard [Wh] | revI base / guard | stops | starts base / guard |'
      ' Iout min base / guard [A] | t(I<-0.05) base / guard [s] | charge back base / guard [C] |')
print('|---|---|---|---|---|---|---|---|---|---|---|')
for sc in NEAR:
    for b in ('fry', 'fry-hq', 'flat', 'flat-hq'):
        ds = [d for (bb, s2, _), d in pairs.items() if bb == b and s2 == sc]
        if not ds:
            continue
        g = lambda k, v: sum(d[v][k] for d in ds)
        print(f"| {sc} | {b} | {len(ds)} | {g('pv', 'base'):.2f} / {g('pv', 'wt'):.2f} | {g('out', 'base'):.2f} / {g('out', 'wt'):.2f} |"
              f" {g('revI', 'base')} / {g('revI', 'wt')} | {g('stops', 'wt')} | {g('starts', 'base')} / {g('starts', 'wt')} |"
              f" {min(d['base']['imin'] for d in ds):.3f} / {min(d['wt']['imin'] for d in ds):.3f} |"
              f" {g('tneg', 'base'):.1f} / {g('tneg', 'wt'):.1f} | {g('qneg', 'base'):.1f} / {g('qneg', 'wt'):.1f} |")
