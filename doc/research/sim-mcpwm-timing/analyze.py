import sys, math, collections, re
R = {}
for line in open('/tmp/sim4103-jobs/results.txt'):
    p = line.split()
    if not p: continue
    d = dict(kv.split('=', 1) for kv in p[1:])
    R[p[0]] = d
J = [l.split() for l in open('/tmp/sim4103-jobs/jobs.txt') if l.strip()]
def key(j): return '.'.join(j)
def get(bin, drv, sc, board, rd, mode, k, x, f='h2'):
    d = R.get('.'.join(map(str, (bin, drv, sc, board, rd, mode, k, x))))
    if not d: return None
    v = d.get(f, '')
    if f == 'trips': m = re.match(r'(\d+)', v); return int(m.group(1)) if m else None
    try: return float(v)
    except ValueError: return None
def tripd(*a):
    d = R.get('.'.join(map(str, a)))
    if not d: return {}
    return {n: int(c) for n, c in re.findall(r'([A-Za-z-]+?)x(\d+)', d['trips'])}
def tripsx(*a, only=None):
    t = tripd(*a)
    return sum(c for n, c in t.items() if (n == only if only else n != 'sweep-no-mpp'))
def tripsl(*a):
    d = R.get('.'.join(map(str, a)))
    return d['trips'] if d else None
def stat(xs):
    xs = [x for x in xs if x is not None]
    n = len(xs)
    if not n: return (0, float('nan'), float('nan'), float('nan'), float('nan'))
    m = sum(xs) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1)) if n > 1 else 0
    return (n, m, sd / math.sqrt(n) if n > 1 else 0, min(xs), max(xs))
def fmt(s): return f"n={s[0]} mean {s[1]:+.2f} se {s[2]:.2f} [{s[3]:+.1f}..{s[4]:+.1f}]"

out = []
P = lambda *a: out.append(' '.join(str(x) for x in a))
N = [("clean", 0), ("gauss", 1), ("gauss", 2), ("meas", 0), ("meas", 8)]
G = "wx-dawn wx-dusk wx-cloud wx-cloud-cv wx-low wx-low-slow wx-mid wx-calfail wx-sweep-fall wx-sweep-rise wx-sweep-shade wx-sweep-gust wx-sweep-learn wx-sweep-learn-off wx-sweep-scan wx-shade2 wx-shade3 wx-shade2-move wx-shade3-move wx-shade-walk wx-shade-cloud wx-shade-learn wx-mp-ramp wx-mp-psu wx-mp-psu-nosink wx-mp-manual wx-mp-ccm".split()
B = [("wx-sun-fry", "-"), ("wx-wob-fry", "-"), ("wx-dusk-land-fry", "-"), ("wx-sun-flat", "-"), ("wx-wob-flat", "-")]
def cells():
    for sc in G:
        for b in ("fry", "flat"): yield sc, b
    for sc, b in B: yield sc, b
P("## S1 standard set: harvest % of GMPP (TRACKER line), 5 noise cases (noise-free, Gaussian 1-2, replay 0/8 s)")
P("| scenario | board | LEDC mean | MCPWM mean | MCPWM-LEDC mean (se) | worst..best | MCPWM offset 0 - LEDC | trips LEDC / MCPWM | sweep-no-mpp LEDC / MCPWM | onset s LEDC / MCPWM | flag |")
P("|---|---|---|---|---|---|---|---|---|---|---|")
for sc, b in cells():
    L = [get('main', 'ledc', sc, b, '1:1', m, k, '-') for m, k in N]
    M = [get('main', 'mcpwm', sc, b, '1:1', m, k, '-') for m, k in N]
    Z = [get('main', 'mcpwm0', sc, b, '1:1', m, k, '-') for m, k in N]
    if not any(x is not None for x in L + M): continue
    dd = stat([m - l for m, l in zip(M, L) if m is not None and l is not None])
    dz = stat([z - l for z, l in zip(Z, L) if z is not None and l is not None])
    tl = sum(tripsx('main', 'ledc', sc, b, '1:1', m, k, '-') for m, k in N)
    tm = sum(tripsx('main', 'mcpwm', sc, b, '1:1', m, k, '-') for m, k in N)
    nl = sum(tripsx('main', 'ledc', sc, b, '1:1', m, k, '-', only='sweep-no-mpp') for m, k in N)
    nm = sum(tripsx('main', 'mcpwm', sc, b, '1:1', m, k, '-', only='sweep-no-mpp') for m, k in N)
    names = sorted(set(n for drv in ('ledc', 'mcpwm') for m, k in N for n in tripd('main', drv, sc, b, '1:1', m, k, '-') if n != 'sweep-no-mpp'))
    ol = sum(get('main', 'ledc', sc, b, '1:1', m, k, '-', 'onset') or 0 for m, k in N)
    om = sum(get('main', 'mcpwm', sc, b, '1:1', m, k, '-', 'onset') or 0 for m, k in N)
    sl, sm = stat(L), stat(M)
    flag = 'BEYOND' if abs(dd[1]) > max(2 * dd[2], 0.1) and dd[0] > 1 else ''
    zs = f"{dz[1]:+.2f} ({dz[2]:.2f})" if dz[0] else ''
    P(f"| {sc} | {b} | {sl[1]:.2f} | {sm[1]:.2f} | {dd[1]:+.2f} ({dd[2]:.2f}) | {dd[3]:+.1f}..{dd[4]:+.1f} | {zs} | {tl} / {tm} {','.join(names)} | {nl} / {nm} | {ol:.0f} / {om:.0f} | {flag} |")

P("")
P("## S2 dusk trap: wx-dusk SIM_VBAT=28, 4 boards x 4 readings x 9 noise cases (noise-free, Gaussian 1-4, replay 0/4/8/12)")
RD = ["1:1", "0.983:1", "0.975:1", "1:1.02"]
N2 = [("clean", 0)] + [("gauss", k) for k in (1, 2, 3, 4)] + [("meas", k) for k in (0, 4, 8, 12)]
BO = ("fry", "fry-hq", "flat", "flat-hq")
P("| build | timing | n | mean harvest | trapped (< 96 %) | worst | trapped cells |")
P("|---|---|---|---|---|---|---|")
for bin in ("pretrap", "main", "nodrain"):
    for drv in ("ledc", "mcpwm", "mcpwm0"):
        hs = []; tr = []
        for b in BO:
            if drv == 'mcpwm0' and not b.startswith('fry'): continue
            for rd in RD:
                for m, k in N2:
                    if bin == 'nodrain' and (m, k) not in N: continue
                    h = get(bin, drv, 'wx-dusk', b, rd, m, k, 'v28')
                    if h is None: continue
                    hs.append(h)
                    if h < 96: tr.append(f"{b}/{rd}/{m}{k}:{h:.1f}")
        if not hs: continue
        s = stat(hs)
        P(f"| {bin} | {drv} | {s[0]} | {s[1]:.2f} | {len(tr)} | {s[3]:.1f} | {' '.join(tr[:12])} |")
P("")
P("Fix effect (main - pretrap), paired, per timing and board:")
P("| timing | board | n | diff mean (se) | worst..best | < 96 % pretrap -> main |")
P("|---|---|---|---|---|---|")
for drv in ("ledc", "mcpwm", "mcpwm0"):
    for b in BO + ('all',):
        if drv == 'mcpwm0' and not b.startswith('fry') and b != 'all': continue
        d = []; t0 = t1 = 0
        for bb in (BO if b == 'all' else (b,)):
            if drv == 'mcpwm0' and not bb.startswith('fry'): continue
            for rd in RD:
                for m, k in N2:
                    a = get('main', drv, 'wx-dusk', bb, rd, m, k, 'v28'); p = get('pretrap', drv, 'wx-dusk', bb, rd, m, k, 'v28')
                    if a is None or p is None: continue
                    d.append(a - p); t0 += p < 96; t1 += a < 96
        s = stat(d)
        if s[0]: P(f"| {drv} | {b} | {s[0]} | {s[1]:+.2f} ({s[2]:.2f}) | {s[3]:+.1f}..{s[4]:+.1f} | {t0} -> {t1} |")
P("")
P("Drain limit effect (main - nodrain), wx-dusk v28, 4 boards x 4 readings x 5 noise cases; wx-low fry/flat x 5:")
P("| timing | set | n | diff mean (se) | worst..best | trips main / nodrain | min Iout main / nodrain |")
P("|---|---|---|---|---|---|---|")
for drv in ("ledc", "mcpwm"):
    for sub, bl, rds in (("dusk exact", BO, ["1:1"]), ("dusk Vout 0.975", BO, ["0.975:1"]), ("dusk fry* misread", ("fry", "fry-hq"), RD[1:]), ("dusk all", BO, RD)):
        d = []; ta = tb = 0; ia = []; ib = []
        for b in bl:
            for rd in rds:
                for m, k in N:
                    a = get('main', drv, 'wx-dusk', b, rd, m, k, 'v28'); p = get('nodrain', drv, 'wx-dusk', b, rd, m, k, 'v28')
                    if a is None or p is None: continue
                    d.append(a - p); ta += tripsx('main', drv, 'wx-dusk', b, rd, m, k, 'v28'); tb += tripsx('nodrain', drv, 'wx-dusk', b, rd, m, k, 'v28')
                    ia.append(get('main', drv, 'wx-dusk', b, rd, m, k, 'v28', 'imin')); ib.append(get('nodrain', drv, 'wx-dusk', b, rd, m, k, 'v28', 'imin'))
        s = stat(d)
        if s[0]: P(f"| {drv} | {sub} | {s[0]} | {s[1]:+.2f} ({s[2]:.2f}) | {s[3]:+.1f}..{s[4]:+.1f} | {ta} / {tb} | {min(x for x in ia if x is not None):.3f} / {min(x for x in ib if x is not None):.3f} |")
    d = []
    for b in ("fry", "flat"):
        for m, k in N:
            a = get('main', drv, 'wx-low', b, '1:1', m, k, '-'); p = get('nodrain', drv, 'wx-low', b, '1:1', m, k, '-')
            if a is not None and p is not None: d.append(a - p)
    s = stat(d)
    if s[0]: P(f"| {drv} | wx-low | {s[0]} | {s[1]:+.2f} ({s[2]:.2f}) | {s[3]:+.1f}..{s[4]:+.1f} | | |")

P("")
P("## S3 min-pulse guard: main vs noguard, replay noise seeds 1-4 (SIM_NOISE_OFS 37 s steps), fry and flat")
P("| scenario | timing | harvest noguard / guard | revI trips noguard / guard | on above onset [s] noguard / guard | minpulse-M stops | starts noguard / guard | Iout min noguard / guard |")
P("|---|---|---|---|---|---|---|---|")
for sc in "wx-dawn wx-dusk wx-low wx-mp-psu wx-mp-psu-nosink wx-mp-ramp wx-mp-manual wx-mp-ccm".split():
    for drv in ("ledc", "mcpwm"):
        row = {}
        for bin in ("noguard", "main"):
            hs = []; rv = 0; on = 0; st = 0; ns = 0; im = []
            for b in ("fry", "flat"):
                for s in (1, 2, 3, 4):
                    k = (bin, drv, sc, b, '1:1', 'rep', s, '-')
                    d = R.get('.'.join(map(str, k)))
                    if not d: continue
                    hs.append(get(*k)); rv += sum(c for n, c in tripd(*k).items() if n.startswith('revI'))
                    on += float(d['onset'] or 0); st += int(d['stops'] or 0); ns += int(d['starts'] or 0); im.append(float(d['imin'] or 0))
            row[bin] = (stat(hs), rv, on, st, ns, min(im) if im else float('nan'))
        if row['main'][0][0] == 0: continue
        a, b_ = row['noguard'], row['main']
        P(f"| {sc} | {drv} | {a[0][1]:.2f} / {b_[0][1]:.2f} | {a[1]} / {b_[1]} | {a[2]:.0f} / {b_[2]:.0f} | {b_[3]} | {a[4]} / {b_[4]} | {a[5]:.3f} / {b_[5]:.3f} |")

P("")
P("## S5 peak scan: pv_substrings=6 minus default, 6 noise cases (Gaussian 1-3, replay 0/8/16)")
P("| scenario | board | LEDC: base / scan - base (se) | MCPWM: base / scan - base (se) | scans/run LEDC / MCPWM |")
P("|---|---|---|---|---|")
N5 = [("gauss", 1), ("gauss", 2), ("gauss", 3), ("meas", 0), ("meas", 8), ("meas", 16)]
for sc in "wx-cloud wx-shade-cloud wx-shade-walk wx-shade2-move wx-shade3-move wx-sweep-shade wx-shade2 wx-shade3".split():
    for b in ("fry", "flat"):
        cols = []; scn = []
        for drv in ("ledc", "mcpwm"):
            base = [get('main', drv, sc, b, '1:1', m, k, '-') for m, k in N5]
            scan = [get('main', drv, sc, b, '1:1', m, k, 'scan6') for m, k in N5]
            d = stat([s - x for s, x in zip(scan, base) if s is not None and x is not None])
            cols.append(f"{stat(base)[1]:.2f} / {d[1]:+.2f} ({d[2]:.2f})")
            scn.append(stat([get('main', drv, sc, b, '1:1', m, k, 'scan6', 'scans') for m, k in N5])[1])
        P(f"| {sc} | {b} | {cols[0]} | {cols[1]} | {scn[0]:.1f} / {scn[1]:.1f} |")

P("")
P("## S6 settle window in slow mode, slow power filter (filter length 60, 200 Hz): main - oldsettle (51b3fa8 rule)")
P("| scenario | board | noise | LEDC: old / main - old | MCPWM: old / main - old |")
P("|---|---|---|---|---|")
for sc, b in (("wx-wob-fry", "-"), ("wx-wob-flat", "-"), ("wx-dusk", "fry"), ("wx-dusk", "flat")):
    for sub, nl in (("noise-free", [("clean", 0)]), ("replay 0-12", [("meas", k) for k in (0, 4, 8, 12)])):
        cols = []
        for drv in ("ledc", "mcpwm"):
            o = [get('oldsettle', drv, sc, b, '1:1', m, k, 'slowf') for m, k in nl]
            a = [get('main', drv, sc, b, '1:1', m, k, 'slowf') for m, k in nl]
            d = stat([x - y for x, y in zip(a, o) if x is not None and y is not None])
            cols.append(f"{stat(o)[1]:.2f} / {d[1]:+.2f} ({d[2]:.2f}, {d[3]:+.1f}..{d[4]:+.1f})")
        P(f"| {sc} | {b} | {sub} | {cols[0]} | {cols[1]} |")
print('\n'.join(out))
P2 = []
for bin in ("main", "pretrap"):
  for b in BO:
    for drv in ("mcpwm", "mcpwm0"):
        if drv == 'mcpwm0' and not b.startswith('fry'): continue
        for sub, rds in (("exact", ["1:1"]), ("misread", RD[1:]), ("all", RD)):
            d = []
            for rd in rds:
                for m, k in N2:
                    a = get(bin, drv, 'wx-dusk', b, rd, m, k, 'v28'); l = get(bin, 'ledc', 'wx-dusk', b, rd, m, k, 'v28')
                    if a is not None and l is not None: d.append(a - l)
            s = stat(d)
            P2.append(f"| {bin} | {b} | {drv} | {sub} | {fmt(s)} |")
print("\n## S2b wx-dusk v28: timing - LEDC, paired"); print("| build | board | timing | readings | diff |"); print("|---|---|---|---|---|"); print('\n'.join(P2))
print("\n## drain per board (main - nodrain)")
for drv in ("ledc", "mcpwm"):
    for b in BO:
        d=[]; tn=collections.Counter()
        for rd in RD:
            for m, k in N:
                a = get('main', drv, 'wx-dusk', b, rd, m, k, 'v28'); p = get('nodrain', drv, 'wx-dusk', b, rd, m, k, 'v28')
                if a is None or p is None: continue
                d.append(a-p); tn.update(tripd('nodrain', drv, 'wx-dusk', b, rd, m, k, 'v28'))
        print(drv, b, fmt(stat(d)), dict(tn))
print("\n## drain at MCPWM offset 0 (main - nodrain)")
for b in ("fry", "fry-hq"):
    for sub, rds in (("exact", ["1:1"]), ("misread", RD[1:])):
        d=[]; tn=collections.Counter(); tm=collections.Counter()
        for rd in rds:
            for m, k in N:
                a = get('main', 'mcpwm0', 'wx-dusk', b, rd, m, k, 'v28'); p = get('nodrain', 'mcpwm0', 'wx-dusk', b, rd, m, k, 'v28')
                if a is None or p is None: continue
                d.append(a-p); tn.update(tripd('nodrain', 'mcpwm0', 'wx-dusk', b, rd, m, k, 'v28')); tm.update(tripd('main', 'mcpwm0', 'wx-dusk', b, rd, m, k, 'v28'))
        print(b, sub, fmt(stat(d)), 'nodrain', dict(tn), 'main', dict(tm))
