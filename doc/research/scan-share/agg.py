import re, sys, collections, statistics as st
R = {}
for ln in open(sys.argv[1]):
    if not ln.strip() or ln.startswith('exit'):
        continue
    f = ln.split()
    key = f[0]
    parts = key.split('.')
    ofs, board, var = parts[-1], parts[-2], parts[-3]
    cell = '.'.join(parts[:-3])
    d = dict(x.split('=', 1) for x in f[1:])
    try:
        d['h2'] = float(d['h2'])
    except ValueError:
        continue
    R[(cell, var, board, ofs)] = d
cells = list(dict.fromkeys(k[0] for k in R))
vars_ = ['base'] + sorted({k[1] for k in R} - {'base'})  # 2026-11-24 round: base D6 D4 D2 A B94 C75


def probes(d):
    m = re.findall(r'([a-z]+)(\d+)', d['probes'])
    return {k: int(v) for k, v in m}


print('| cell | base harvest % | ' + ' | '.join(f'{v} - base' for v in vars_[1:]) + ' |')
print('|---' * (2 + len(vars_) - 1) + '|')
tot = collections.defaultdict(list)
for c in cells:
    base = [(k, d) for k, d in R.items() if k[0] == c and k[1] == 'base']
    hb = [d['h2'] for k, d in base]
    row = [c, f"{st.mean(hb):.2f} ({min(hb):.2f}..{max(hb):.2f}) n={len(hb)}"]
    for v in vars_[1:]:
        ds = [R[(c, v, k[2], k[3])]['h2'] - d['h2'] for k, d in base if (c, v, k[2], k[3]) in R]
        if not ds:
            row.append('-'); continue
        tot[v] += ds
        se = st.stdev(ds) / len(ds) ** .5 if len(ds) > 1 else 0
        row.append(f"{st.mean(ds):+.2f} ({min(ds):+.2f}..{max(ds):+.2f}) se {se:.2f}")
    print('| ' + ' | '.join(row) + ' |')
print('| ALL mean | | ' + ' | '.join(f"{st.mean(tot[v]):+.3f}" if tot[v] else '-' for v in vars_[1:]) + ' |')

print('\n| cell | variant | wrong-peak s mean | scans | moved | hit | miss-place | miss-noise | wrong | partial | light | G-probe |dV| % | read/true gain |')
print('|---' * 13 + '|')
for c in cells:
    for v in vars_:
        ds = [d for k, d in R.items() if k[0] == c and k[1] == v]
        if not ds:
            continue
        P = collections.Counter()
        for d in ds:
            P.update(probes(d))
        mean = lambda key: st.mean(float(d[key]) for d in ds if d.get(key))
        dv = [float(d['dv']) for d in ds if float(d['dv']) > 0]
        rg = [float(d['readgain']) for d in ds if float(d['readgain']) > 0]
        print(f"| {c} | {v} | {mean('wrong'):.0f} | {mean('scans'):.1f} | {mean('moved'):.1f} | {P['hit']} | {P['missplace']} |"
              f" {P['missnoise']} | {P['wrong']} | {P['partial']} | {P['light']} | {st.mean(dv) if dv else 0:.1f} |"
              f" {st.mean(rg) if rg else 0:.3f} |")
