#!/usr/bin/env python3
# Interleaved die-temperature sweep over `pwr` knob settings (needs a CONFIG_FUGU_WITH_PWR_KNOBS build).
# Holds the port open, applies each point's commands, soaks, and averages `T=` from the `pwr` reply
# over the last --avg seconds of the soak.
#
#   pwr_sweep.py -p /dev/cu.usbmodem1101 --soak 240 --avg 90 --csv /tmp/sweep.csv \
#       --point "a=pwr cpu 160" --point "b=pwr cpu 80" --order a,b,a,b,a
#
# Several commands per point: separate with ';'. Repeat a label in --order to get an error bar;
# each segment is also compared with the mean of its neighbours to cancel ambient drift.
import argparse, re, statistics, sys, time
import serial

ap = argparse.ArgumentParser()
ap.add_argument('-p', '--port', required=True)
ap.add_argument('--point', action='append', required=True, help='label=cmd[;cmd...]')
ap.add_argument('--order', required=True, help='comma-separated labels')
ap.add_argument('--soak', type=float, default=240)
ap.add_argument('--avg', type=float, default=90)
ap.add_argument('--poll', type=float, default=5)
ap.add_argument('--csv')
args = ap.parse_args()

points = dict(p.split('=', 1) for p in args.point)
order = args.order.split(',')
missing = [o for o in order if o not in points]
if missing:
    sys.exit(f'unknown labels in --order: {missing}')

pat = re.compile(r'pwr: .*\bT=(-?[\d.]+)')
s = serial.Serial(args.port, 115200, timeout=0.2)
time.sleep(1.0)
s.write(b'\n')  # the first bytes after open can arrive corrupted
time.sleep(0.5)
s.reset_input_buffer()
buf = b''


def send(cmd):
    s.write((cmd + '\n').encode())
    s.flush()


def read_lines(dur):
    global buf
    end = time.time() + dur
    out = []
    while time.time() < end:
        buf += s.read(512)
        while b'\n' in buf:
            raw, buf = buf.split(b'\n', 1)
            out.append(raw.decode('utf-8', 'replace').strip())
    return out


csv = open(args.csv, 'w') if args.csv else None
if csv:
    csv.write('t,seg,label,T,line\n')
t0 = time.time()
segs = []
for i, label in enumerate(order):
    for cmd in points[label].split(';'):
        send(cmd.strip())
        for ln in read_lines(1.0):
            if 'pwr' in ln or 'ERR' in ln or 'usage' in ln.lower():
                print(f'   {ln[:150]}', flush=True)
    seg_t0 = time.time()
    temps, last = [], ''
    while time.time() - seg_t0 < args.soak:
        send('pwr')
        for ln in read_lines(args.poll):
            m = pat.search(ln)
            if m:
                temps.append((time.time() - seg_t0, float(m.group(1))))
                last = ln
                if csv:
                    csv.write(f'{time.time() - t0:.1f},{i},{label},{m.group(1)},"{ln}"\n')
                    csv.flush()
    tail = [T for t, T in temps if t >= args.soak - args.avg]
    if len(tail) < 3:
        print(f'[{i}] {label}: only {len(tail)} samples in the averaging window -> UNEVALUABLE', flush=True)
        segs.append((label, None, None))
        continue
    mean = statistics.fmean(tail)
    slope = tail[-1] - tail[0]
    segs.append((label, mean, slope))
    print(f'[{i}] {label:12s} T={mean:6.2f}  n={len(tail)}  drift-in-window={slope:+.2f}  | {last[:110]}', flush=True)

print('\nper label:')
for label in points:
    v = [m for l, m, _ in segs if l == label and m is not None]
    if v:
        sd = statistics.stdev(v) if len(v) > 1 else float('nan')
        print(f'  {label:12s} mean={statistics.fmean(v):6.2f}  sd={sd:.2f}  n={len(v)}')
    else:
        print(f'  {label:12s} no valid segments')
print('\nvs. neighbour mean (drift-cancelled):')
for i in range(1, len(segs) - 1):
    l, m, _ = segs[i]
    a, b = segs[i - 1], segs[i + 1]
    if m is None or a[1] is None or b[1] is None or a[0] != b[0] or a[0] == l:
        continue
    print(f'  [{i}] {l} - {a[0]} = {m - (a[1] + b[1]) / 2:+.2f} K')
s.close()
