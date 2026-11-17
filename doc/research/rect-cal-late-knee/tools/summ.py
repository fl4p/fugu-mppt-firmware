import sys, csv, math, statistics as st
tick = 12.5  # ns per count (sim: 2047 @ 39 kHz)
def f(x):
    try: return float(x)
    except: return math.nan
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
by = {}
for r in rows:
    by.setdefault(r['board'], []).append(r)
print(f"{'board':8} {'runs':>4} {'done':>4} {'fail':>4} {'bound':>5} | truth ns mean [min..max] | learned-truth ns mean sd [min..max] | applied-truth ns mean sd [min..max] | applied ns [min..max] | learned-0 mean sd")
allr=[]
for b in ['fry','fry-hq','flat','flat-hq']:
    R = by.get(b, [])
    done = [r for r in R if '_done_' in r['status'] and not math.isnan(f(r['truth']))]
    fails = [r['status'].split('failed:_')[1].split('_after')[0] for r in R if 'failed' in r['status']]
    tr = [f(r['truth'])*tick for r in done]
    le = [(f(r['learned'])-f(r['truth']))*tick for r in done]
    ap = [(f(r['applied'])-f(r['truth']))*tick for r in done]
    apa = [f(r['applied'])*tick for r in done]
    l0 = [f(r['learned'])*tick for r in done]
    nb = sum(int(r['bound']) for r in done)
    s = lambda v: f"{st.mean(v):+5.0f} {st.pstdev(v):4.0f} [{min(v):+5.0f}..{max(v):+5.0f}]" if v else "-"
    print(f"{b:8} {len(R):4} {len(done):4} {len(fails):4} {nb:5} | {st.mean(tr):+5.0f} [{min(tr):+5.0f}..{max(tr):+5.0f}] | {s(le)} | {s(ap)} | [{min(apa):+5.0f}..{max(apa):+5.0f}] | {st.mean(l0):+5.0f} {st.pstdev(l0):4.0f}  fails: {','.join(sorted(set(fails)))} ({len(fails)})")
