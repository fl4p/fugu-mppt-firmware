#!/usr/bin/env python3
"""Peak-scan field summary from the havan console log: per device and UTC day, scans, moves, near misses.

usage: etc/scan-field-report.py [--days N] [--host havan-ts] [--log pv/fugu_console.log]
A near miss is a scan whose best probe read x1.00..x1.03 (under the move margin) without moving; heat counts scans
whose upward probes used the heating rule (`, heat`, firmware from 2026-11-26).
"""
import argparse, collections, re, subprocess

ap = argparse.ArgumentParser()
ap.add_argument('--days', type=int, default=3)
ap.add_argument('--host', default='havan-ts')
ap.add_argument('--log', default='pv/fugu_console.log')
a = ap.parse_args()

out = subprocess.run(['ssh', a.host, f'grep -a "mppt: scan: P" {a.log}'], capture_output=True, text=True, timeout=120)
if out.returncode not in (0, 1):
    raise SystemExit(f'ssh/grep failed ({out.returncode}): {out.stderr.strip()}')
pat = re.compile(r'^(\d{4}-\d\d-\d\d) (\d\d:\d\d):\S+\s+(\w+): .*scan: P ([\d.]+)->([\d.]+) W at ([\d.]+)V, (\d+) probes, '
                 r'best x([\d.]+) at ([\d.]+)V(, moving)?(?: \(light changed\))?(, heat)?')
days = collections.defaultdict(lambda: collections.defaultdict(list))
for line in out.stdout.splitlines():
    m = pat.match(line)
    if m: days[m.group(1)][m.group(3)].append(m)
for d in sorted(days)[-a.days:]:
    for dev in sorted(days[d]):
        s = days[d][dev]
        mv = [m for m in s if m.group(10)]
        near = [m for m in s if not m.group(10) and 1.0 <= float(m.group(8)) < 1.03]
        heat = [m for m in s if m.group(11)]
        print(f'{d} {dev:5} scans {len(s):3}  moved {len(mv):2}  near-miss {len(near):2}  heat {len(heat):2}')
        for m in sorted(set(mv + near + heat), key=lambda m: m.group(2)):
            tag = ('move' if m.group(10) else 'near' if m in near else 'scan') + (' heat' if m.group(11) else '')
            print(f'    {m.group(2)} {tag} {m.group(6)}V -> {m.group(9)}V x{m.group(8)} at {m.group(4)} W')
