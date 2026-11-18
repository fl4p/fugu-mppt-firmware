import sys, collections, statistics as st
# paired stats new - main per subset; loaded by final.py
rows = {}
for fn in sys.argv[1:]:
    for l in open(fn):
        p = l.split()
        if len(p) != 9: continue
        b, sc, board, g, vi, mode, k, h, tr = p
        rows[(b, sc, board, g, vi, mode, k)] = (float(h), int(tr))
def pairs(filt=lambda key: True, A='main', B='v2a'):
    out = []
    for key, (h, tr) in rows.items():
        if key[0] != A or not filt(key): continue
        kb = (B,) + key[1:]
        if kb in rows: out.append((key[1:], h, rows[kb][0], tr, rows[kb][1]))
    return out
def summ(name, ps, trap=96):
    if not ps: return
    d = [b - a for _, a, b, _, _ in ps]
    n = len(d); m = st.mean(d); sd = st.stdev(d) if n > 1 else 0
    ta = sum(a < trap for _, a, _, _, _ in ps); tb = sum(b < trap for _, _, b, _, _ in ps)
    trd = sum(x != y for *_, x, y in ps)
    w = min(ps, key=lambda p: p[2] - p[1]); bst = max(ps, key=lambda p: p[2] - p[1])
    print(f"{name:34s} n={n:3d} main mean {st.mean(a for _,a,_,_,_ in ps):5.2f} new {st.mean(b for _,_,b,_,_ in ps):5.2f} | diff mean {m:+.2f} (se {sd/n**.5:.2f}) median {st.median(d):+.2f} worst {min(d):+.1f} best {max(d):+.1f} | worse/better {sum(x<0 for x in d)}/{sum(x>0 for x in d)} | <{trap}%: {ta}->{tb} | trips differ {trd}")
    return w, bst
