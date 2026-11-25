import sys, re, statistics as st, collections
d = {}
for l in open(sys.argv[1]):
    k, *f = l.split()
    c, v, b, o = k.rsplit('.', 3) if k.count('.') == 3 else (None,)*4
    kv = dict(x.split('=', 1) for x in f)
    d[(c, v, b, o)] = float(kv['h2'])
cells = list(dict.fromkeys(k[0] for k in d))
pairs = [('t0', 'base', '1.015 vs 0.97'), ('u1015', 't0', 'heat .96'), ('t93', 't0', 'heat .93')]
print('%-16s' % 'cell', ' '.join('%22s' % p[2] for p in pairs), '  abs t0  abs u1015')
tot = collections.defaultdict(list)
for c in cells:
    row = []
    for a, b_, _ in pairs:
        ds = [d[(c, a, b, o)] - d[(c, b_, b, o)] for b in ('fry', 'flat') for o in ('0', '13', '29', '41', '57')]
        m = st.mean(ds); se = st.stdev(ds) / len(ds) ** .5
        tot[a + b_].append(m)
        row.append('%+6.2f (%.2f) min%+6.2f' % (m, se, min(ds)))
    at = st.mean(d[(c, 't0', b, o)] for b in ('fry', 'flat') for o in ('0', '13', '29', '41', '57'))
    au = st.mean(d[(c, 'u1015', b, o)] for b in ('fry', 'flat') for o in ('0', '13', '29', '41', '57'))
    print('%-16s' % c, ' '.join('%22s' % r for r in row), '  %6.2f  %6.2f' % (at, au))
print('%-16s' % 'MEAN', ' '.join('%22s' % ('%+6.3f' % st.mean(tot[a + b_])) for a, b_, _ in pairs))
