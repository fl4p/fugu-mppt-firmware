import re, sys, collections
rx = re.compile(r'STATIC (?:n (\d) )?kp (\S+) drift (\S+) k (\d) k2 (\d) s (\S+) op (\d)/(\d) (\S+)V m0 (\d) gT (\S+) var (\S+)\s+sh (\S+) \| G dV (\S+)% read (\S+) \| (\S+)')
rows = []
for ln in open(sys.argv[1]):
    m = rx.match(ln)
    if m:
        n, kp, dr, k, k2, s, op, np_, vin, m0, gT, var, sh, dv, rd, cls = m.groups()
        rows.append(dict(n=int(n or 6), kp=float(kp), dr=float(dr), k=int(k), k2=int(k2), s=float(s), m0=int(m0), gT=float(gT), var=var,
                         dv=float(dv), rd=float(rd), cls=cls))
vars_ = list(dict.fromkeys(r['var'] for r in rows))
drs = sorted(set(r['dr'] for r in rows), reverse=True)


def hits(rs):
    return f"{sum(r['cls'] == 'HIT' for r in rs)}/{len(rs)}"


print('| variant | m0=n, gain 3-10 % | m0=n, >10 % | m0<n, 3-10 % | m0<n, >10 % | ' +
      ' | '.join(f'drift {d:+.2f}' for d in drs) + ' | wrong/partial (all) | G-probe dV med / worst, m0=n | m0<n | read min |')
print('|---' * (9 + len(drs)) + '|')
for v in vars_:
    R = [r for r in rows if r['var'] == v]
    mv = [r for r in R if r['gT'] > 1.03]
    a = [r for r in mv if r['m0'] == r['n'] and r['gT'] <= 1.10]
    b = [r for r in mv if r['m0'] == r['n'] and r['gT'] > 1.10]
    c = [r for r in mv if r['m0'] < r['n'] and r['gT'] <= 1.10]
    d = [r for r in mv if r['m0'] < r['n'] and r['gT'] > 1.10]
    bad = sum(r['cls'] in ('WRONG', 'PARTIAL') for r in R)
    wrong = sum(r['cls'] == 'WRONG' for r in R)

    def dvs(rs):
        x = sorted(r['dv'] for r in rs if r['dv'] == r['dv'])
        return f"{x[len(x) // 2]:+.1f} / {max(x, key=abs):+.1f}" if x else '-'
    rd = [r['rd'] for r in mv if r['rd'] > 0]
    print(f"| {v} | {hits(a)} | {hits(b)} | {hits(c)} | {hits(d)} | " +
          ' | '.join(hits([r for r in mv if r['dr'] == dd]) for dd in drs) +
          f" | {wrong}/{bad - wrong} | {dvs(a + b)} | {dvs(c + d)} | {min(rd):.3f} |")
