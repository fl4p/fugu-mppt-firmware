#!/usr/bin/env python3
"""Peak-scan and sweep-verdict field summary from the havan console log, per device and UTC day.

usage: etc/scan-field-report.py [--days N] [--host havan-ts] [--log pv/fugu_console.log ...]
--log repeats; default is the current log plus pv/fugu_console.log.1 and .2 (each holds about a day; missing files are
skipped). Lines found in several files count once.
Scans: a near miss is a scan whose best probe read x1.00..x1.03 (under the move margin) without moving; heat counts
scans whose upward probes used the heating rule (`, heat`, firmware from 2026-11-26).
Sweeps: the firmware's post-sweep verdicts (`sweep verified|flagged: landed W, off by %`, flagged beyond
tracker.conf::sweep_verify, default 10 %; low-power sweeps get none). off = landing vs the sweep's MPP: negative is a
loss, positive means the light rose during the sweep. Under passing clouds a large loss is
usually light, not a wrong peak: compare landed V with MPP V on the `sweep landed` line. rescan = `, scan now`. Each flagged sweep gets a detail line.
"""
import argparse, collections, re, statistics, subprocess

ap = argparse.ArgumentParser()
ap.add_argument('--days', type=int, default=3)
ap.add_argument('--host', default='havan-ts')
ap.add_argument('--log', action='append')
a = ap.parse_args()
logs = ' '.join(a.log or ['pv/fugu_console.log', 'pv/fugu_console.log.1', 'pv/fugu_console.log.2'])

out = subprocess.run(['ssh', a.host, f'L=$(ls {logs} 2>/dev/null) || [ -n "$L" ] || {{ echo "no log found: {logs}" >&2; exit 2; }}; '
                      f'head -qn1 $L; grep -ahE "mppt: (scan: P|sweep (verified|flagged): )" $L'],
                     capture_output=True, text=True, timeout=120)
if out.returncode not in (0, 1):
    raise SystemExit(f'ssh/grep failed ({out.returncode}): {out.stderr.strip()}')
pat = re.compile(r'^(\d{4}-\d\d-\d\d) (\d\d:\d\d):\S+\s+(\w+): .*scan: P ([\d.]+)->([\d.]+) W at ([\d.]+)V, (\d+) probes, '
                 r'best x([\d.]+) at ([\d.]+)V(, moving)?(?: \(light changed\))?(, heat)?')
svp = re.compile(r'^(\d{4}-\d\d-\d\d) (\d\d:\d\d):\S+\s+(\w+): .*sweep (verified|flagged): landed (\d+) W, '
                 r'off by ([+-]\d+) %(, scan now)?')
days = collections.defaultdict(lambda: collections.defaultdict(lambda: ([], [])))
lines = sorted(set(out.stdout.splitlines()))
first = next((l for l in lines if re.match(r'\d{4}-\d\d-\d\d ', l)), None)
if first: print(f'log from {first[:16]}')
for line in lines:
    for i, p in enumerate((pat, svp)):
        m = p.match(line)
        if m: days[m.group(1)][m.group(3)][i].append(m)
for d in sorted(days)[-a.days:]:
    for dev in sorted(days[d]):
        s, v = days[d][dev]
        mv = [m for m in s if m.group(10)]
        near = [m for m in s if not m.group(10) and 1.0 <= float(m.group(8)) < 1.03]
        heat = [m for m in s if m.group(11)]
        fl = [m for m in v if m.group(4) == 'flagged']
        off = [int(m.group(6)) for m in v]
        hdr = f'{d} {dev:5}'
        if s: hdr += f' scans {len(s):3}  moved {len(mv):2}  near-miss {len(near):2}  heat {len(heat):2}'
        if v: hdr += (f' {"  " if s else " " * 45}sweeps {len(v):3}  verified {len(v) - len(fl):2}  flagged {len(fl):2}'
                      f'  rescan {sum(1 for m in fl if m.group(7)):2}  off med {statistics.median(off):+.0f} %'
                      f'  worst loss {min(off) if min(off) < 0 else "-"} %  worst gain {f"+{max(off)}" if max(off) > 0 else "-"} %')
        print(hdr)
        for m in sorted(set(mv + near + heat) | set(fl), key=lambda m: m.group(2)):
            if m.re is svp:
                print(f'    {m.group(2)} flagged landed {m.group(5)} W off {m.group(6)} %  rescan {"yes" if m.group(7) else "no"}')
                continue
            tag = ('move' if m.group(10) else 'near' if m in near else 'scan') + (' heat' if m.group(11) else '')
            print(f'    {m.group(2)} {tag} {m.group(6)}V -> {m.group(9)}V x{m.group(8)} at {m.group(4)} W')
