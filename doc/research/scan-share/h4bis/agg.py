import re, sys, glob, os, statistics as st, subprocess, collections
# agg.py DIR TREE... : per tree, h4 1.015 - 0.97 paired, by kVmp learned at the boot sweep (kvoc > 67)
d = sys.argv[1]
def parse(f):
    h = kv = None; pr = ''
    for ln in open(f):
        m = re.search(r'^== TRACKER.*harvest ([0-9.]+) %', ln)
        if m and h is None: h = float(m.group(1))
        if ln.startswith('== PROBES'): pr = ln.split('|')[0]
        if ln.startswith('== PROBE t=14') and kv is None:
            v0 = float(re.search(r'v0 ([0-9.]+)', ln).group(1))
            ps = [float(x) for x in re.findall(r'([0-9.]+)V@', ln)]
            kv = 6 * (2 * ps[0] / (0.97 * 3) - v0 / 6)
    mp = re.search(r'miss-place (\d+)', pr); mn = re.search(r'miss-noise (\d+)', pr)
    return h, kv, int(mp.group(1)) if mp else 0, int(mn.group(1)) if mn else 0
for t in sys.argv[2:]:
    R = {}
    for f in glob.glob(f'{d}/{t}.h4.*.txt'):
        k = os.path.basename(f)[len(t) + 4:-4].split('.', 1)  # var, board.ofs
        try: R[(k[0], k[1])] = parse(f)
        except Exception: pass
    ds, dl, du, hb, nl, mp = [], [], [], [], 0, 0
    for (v, p), (h, kv, a, b) in R.items():
        if v != 'u1015' or ('base', p) not in R or h is None or R[('base', p)][0] is None: continue
        hbase = R[('base', p)][0]; x = h - hbase
        ds.append(x); hb.append(hbase); mp += a
        if kv is not None and kv > 67: dl.append(x); nl += 1
        else: du.append(x)
    if not ds: print(t, 'no pairs'); continue
    se = st.stdev(ds) / len(ds) ** .5 if len(ds) > 1 else 0
    f = lambda L: f'{st.mean(L):+.2f} (n {len(L)})' if L else '-'
    print(f'{t:12s} pairs {len(ds):2d} base {st.mean(hb):.2f} d {st.mean(ds):+.2f} se {se:.2f} min {min(ds):+.2f} max {max(ds):+.2f}'
          f' | learned {f(dl)} | default {f(du)} | miss-place {mp}')
