import re, sys, glob, os, statistics as st
# cmp.py DIR REF TREE... : per tree, mean of (u1015 - base) and its paired difference vs REF; subset of the round 2/3 offsets
d = sys.argv[1]
def h(f):
    for ln in open(f):
        m = re.search(r'^== TRACKER.*harvest ([0-9.]+) %', ln)
        if m: return float(m.group(1))
def delta(t):
    R = {}
    for f in glob.glob(f'{d}/{t}.h4.u1015.*.txt'):
        p = os.path.basename(f)[len(t) + 10:-4]
        g = f'{d}/{t}.h4.base.{p}.txt'
        if os.path.exists(g) and h(f) is not None and h(g) is not None: R[p] = (h(f) - h(g), h(g))
    return R
ref = delta(sys.argv[2])
old = {f'{b}.{o}' for b in ('fry', 'flat') for o in (0, 13, 29, 41, 57)}
for t in sys.argv[2:]:
    R = delta(t)
    ks = sorted(set(R) & set(ref))
    if not ks: print(t, '-'); continue
    x = [R[k][0] for k in ks]; dd = [R[k][0] - ref[k][0] for k in ks]; db = [R[k][1] - ref[k][1] for k in ks]
    xo = [R[k][0] for k in ks if k in old]
    se = lambda L: st.stdev(L) / len(L) ** .5 if len(L) > 1 else 0
    print(f'{t:12s} n {len(ks):2d} d {st.mean(x):+.2f} (se {se(x):.2f}) | 10 old ofs {st.mean(xo):+.2f} | vs {sys.argv[2]}: '
          f'{st.mean(dd):+.2f} (se {se(dd):.2f}), base {st.mean(db):+.2f}')
