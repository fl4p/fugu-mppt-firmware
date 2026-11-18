#!/usr/bin/env python3
"""Tabulate one or two trk-noise-sweep.sh result files: harvest, tracking loss, reversals/min, slow share.
   trk-sweep-diff.py BASE.txt [NEW.txt]"""
import re, sys

def load(path):
    runs, key = {}, None
    for line in open(path):
        m = re.match(r'### (\S+) noise=(\S+) gain=(\S+)(?: ofs=(\S+))?', line)
        if m: key = m.groups()[:3] + (m.group(4) or '0',); runs[key] = {}; continue
        if key is None: continue
        for name, rx in (('harvest', r'harvest\s+([\d.]+) %'), ('loss', r'tracking loss ([\d.]+) %'),
                         ('rev', r'([\d.]+) reversals/min'), ('slow', r'slow\s+([\d.]+) % of the time'),
                         ('trips', r'trips (\d+)')):
            m = re.search(rx, line)
            if m and name not in runs[key]: runs[key][name] = float(m.group(1))
    return runs

base = load(sys.argv[1]); new = load(sys.argv[2]) if len(sys.argv) > 2 else None
cols = ('harvest', 'loss', 'rev', 'slow', 'trips')
hdr = f"{'scenario':12} {'noise':5} {'gain':>4} {'ofs':>4} | " + ' '.join(f"{c:>8}" for c in cols)
if new: hdr += ' | ' + ' '.join(f"{'d'+c:>8}" for c in cols)
print(hdr)
for k in sorted(base, key=lambda k: (k[0], k[1], float(k[2]), float(k[3]))):
    b = base[k]; row = f"{k[0]:12} {k[1]:5} {k[2]:>4} {k[3]:>4} | " + ' '.join(f"{b.get(c, float('nan')):8.2f}" for c in cols)
    if new and k in new:
        n = new[k]; row += ' | ' + ' '.join(f"{n.get(c, float('nan')) - b.get(c, float('nan')):+8.2f}" for c in cols)
    print(row)
if new:
    h = [new[k]['harvest'] - base[k]['harvest'] for k in base if k in new and 'harvest' in new[k] and 'harvest' in base[k]]
    print(f"harvest delta: mean {sum(h)/len(h):+.2f} pp, min {min(h):+.2f}, max {max(h):+.2f} over {len(h)} runs")
