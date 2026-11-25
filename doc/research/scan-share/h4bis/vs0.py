import re,sys
# derive vs0 = kVmp*voc/n from the first PROBE line (t~146): lowest probe m=3 at 0.97*3*sh (n=6, m0=6)
for f in sys.argv[1:]:
    for ln in open(f):
        if ln.startswith('== PROBE t=14'):
            v0=float(re.search(r'v0 ([0-9.]+)',ln).group(1))
            ps=[float(x) for x in re.findall(r'\| ([0-9.]+)V@|% x[0-9.]+ ([0-9.]+)V@',ln) for x in x if x] if False else [float(x) for x in re.findall(r'([0-9.]+)V@',ln)]
            sh=ps[0]/(0.97*3)
            vs0=2*sh-v0/6
            print(f.split('/')[-1][:24], 'v0 %.1f probes %s sh %.2f vs0 %.2f kVmp*voc %.1f'%(v0,ps,sh,vs0,6*vs0))
            break
