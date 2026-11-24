"""Per-case gains / losses of scan-static variants vs base: static_cmp.py static.txt [variant ...] [--cases].
A case gains (loses) when the maximum it lands on (stays or moves to) is > 0.2 % better (worse) relative to the global
than under base. Tables by n, Voc drift and kp; --cases lists every loss (and the gains with --gains)."""
import re, sys, collections

rx = re.compile(r'STATIC n (\d) kp (\S+) drift (\S+) k (\d) k2 (\d) s (\S+) op (\d)/(\d) (\S+)V m0 (\d) gT (\S+) var (\S+)\s+'
                r'sh (\S+) \| G dV (\S+)% read (\S+) \| (\S+) land (\S+)V x(\S+)')
args = [a for a in sys.argv[1:] if not a.startswith('--')]
R = collections.defaultdict(dict)
for ln in open(args[0]):
    m = rx.match(ln)
    if not m:
        continue
    n, kp, dr, k, k2, s, op, np_, vin, m0, gT, var, sh, dv, rd, cls, lv, lx = m.groups()
    key = (int(n), kp, dr, int(k), int(k2), s, f'{op}/{np_}')
    R[var][key] = dict(cls=cls, x=float(lx), gT=float(gT), dv=float(dv), rd=float(rd), m0=int(m0), vin=float(vin))
vars_ = args[1:] or [v for v in R if v != 'base']
base = R['base']
TOL = 0.002


def cmp(v, keys):
    g = l = 0
    dx = 0.0
    for c in keys:
        d = R[v][c]['x'] - base[c]['x']
        dx += d
        g += d > TOL
        l += d < -TOL
    return g, l, dx / max(1, len(keys))


def table(title, field_idx, values, fmt):
    print(f'\n### {title}\n')
    print('| variant | ' + ' | '.join(fmt(x) for x in values) + ' | all |')
    print('|---' * (len(values) + 2) + '|')
    for v in vars_:
        cells = []
        for x in values:
            ks = [c for c in base if c[field_idx] == x and c in R[v]]
            g, l, _ = cmp(v, ks)
            cells.append(f'+{g} / -{l}')
        g, l, dx = cmp(v, [c for c in base if c in R[v]])
        print(f'| {v} | ' + ' | '.join(cells) + f' | +{g} / -{l} |')


ns = sorted({c[0] for c in base})
drs = sorted({c[2] for c in base}, key=float, reverse=True)
kps = sorted({c[1] for c in base})
print(f'cases per variant: {len(base)} (n {ns}); gain / loss = landed maximum > {100 * TOL:.1f} % of the global better / worse than base')
table('by n', 0, ns, lambda x: f'n = {x}')
table('by Voc drift since the sweep', 2, drs, lambda x: f'{float(x) * 100:+.0f} %')
table('by kp (module Vmpp/Voc)', 1, kps, lambda x: f'kp {x}')
# heated cells: drift <= -8 %, per n
print('\n### heated (drift -8 / -12 %), per n\n')
print('| variant | ' + ' | '.join(f'n = {n}, {float(d) * 100:+.0f} %' for n in ns for d in drs if float(d) <= -0.08) + ' |')
print('|---' * (1 + len(ns) * sum(float(d) <= -0.08 for d in drs)) + '|')
for v in vars_:
    cells = []
    for n in ns:
        for d in drs:
            if float(d) > -0.08:
                continue
            g, l, _ = cmp(v, [c for c in base if c[0] == n and c[2] == d and c in R[v]])
            cells.append(f'+{g} / -{l}')
    print(f'| {v} | ' + ' | '.join(cells) + ' |')
if '--cases' in sys.argv:
    for v in vars_:
        print(f'\n### {v}: per-case losses' + (' and gains' if '--gains' in sys.argv else ''))
        for c in base:
            if c not in R[v]:
                continue
            b, x = base[c], R[v][c]
            d = x['x'] - b['x']
            if d < -TOL or ('--gains' in sys.argv and d > TOL):
                print(f"  {'LOSS' if d < 0 else 'gain'} n {c[0]} kp {c[1]} drift {c[2]} k {c[3]} k2 {c[4]} s {c[5]} op {c[6]}"
                      f" m0 {b['m0']} gT {b['gT']:.3f}: {b['cls']} x{b['x']:.3f} -> {x['cls']} x{x['x']:.3f}"
                      f" (G probe dV {b['dv']:+.1f} -> {x['dv']:+.1f} %, read {b['rd']:.3f} -> {x['rd']:.3f})")
