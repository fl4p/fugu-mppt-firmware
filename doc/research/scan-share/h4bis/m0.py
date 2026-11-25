import re, sys, glob, os, collections, statistics as st
# m0.py DIR TREE...: u1015 scans after the 720 s step while on the wrong peak (G x > 1.03): count m0 5 (one probe
# above G) vs m0 4 (two probes), outcome per class, v0 mean; and the first such scan's time
d = sys.argv[1]
var = os.environ.get('VAR', 'u1015')
for t in sys.argv[2:]:
    C = collections.Counter(); V = collections.defaultdict(list); first = []; nscan = []; learned = 0; runs = 0
    for f in glob.glob(f'{d}/{t}.h4.{var}.*.txt'):
        runs += 1; n = 0; fi = None
        for ln in open(f):
            m = re.match(r'== PROBE t=(\d+) v0 ([0-9.]+) op ([0-9.]+)V/[0-9.]+W G ([0-9.]+)V/[0-9.]+W x([0-9.]+) \|(.*)\| landed.*\| (\S+)', ln)
            if not m or int(m.group(1)) < 720: continue
            if fi is None: fi = int(m.group(1)); first.append(fi)
            if float(m.group(5)) < 1.03: continue
            n += 1
            np_ = len(re.findall(r'V@', m.group(6)))
            cls = f'{np_} probe(s)'
            C[(cls, m.group(7))] += 1; V[cls].append(float(m.group(2)))
        nscan.append(n)
    print(f'{t}: {runs} runs, wrong-peak scans after 720 s: {sum(nscan)} (max {max(nscan) if nscan else 0} in one run), first post-step scan t mean {st.mean(first) if first else 0:.0f}')
    for (cls, o), k in sorted(C.items()): print(f'   {cls:12s} {o:12s} {k}')
    for cls, L in V.items(): print(f'   {cls}: v0 mean {st.mean(L):.2f} min {min(L):.1f} max {max(L):.1f}')
