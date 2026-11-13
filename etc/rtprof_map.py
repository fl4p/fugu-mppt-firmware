#!/usr/bin/env python3
"""Map an `rtprof` dump (console log) to functions: samples, size, archive(object).

  etc/rtprof_map.py <log> <elf> [--map <map>] [--min N | --rate R] [--lf out.lf [--merge in.lf]]
  etc/rtprof_map.py --check <in.lf> - <elf>    # list (noflash) entries the linked ELF left in flash
"""
import argparse, bisect, collections, glob, hashlib, os, re, shutil, subprocess, sys

ap = argparse.ArgumentParser()
ap.add_argument('log')
ap.add_argument('elf')
ap.add_argument('--map')
ap.add_argument('--min', type=int, default=1, help='minimum samples per function')
ap.add_argument('--rate', type=float, help='minimum samples per second of profiling (overrides --min)')
ap.add_argument('--hz', type=float, default=2000, help='sampling rate of the dump (rtprofStart)')
ap.add_argument('--nm', default='xtensa-esp32s3-elf-nm')
ap.add_argument('--merge', action='append', default=[], help='existing .lf whose entries are kept in --lf output')
ap.add_argument('--lf', help='write ldgen mapping fragments placing these functions in IRAM (noflash)')
ap.add_argument('--skip-archive', action='append', default=[], help='archive basename to leave out of --lf')
ap.add_argument('--check', help='.lf to verify against the ELF instead of mapping a dump')
ap.add_argument('--unverified', action='store_true', help='accept a dump without the samples/entries/elf header')
a = ap.parse_args()

if not shutil.which(a.nm):
    c = sorted(glob.glob(os.path.expanduser(f'~/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin/{a.nm}')))
    if not c:
        sys.exit(f'{a.nm} not found: source idf-export.sh or pass --nm')
    a.nm = c[-1]

if a.check:
    addr = {}
    for l in subprocess.run([a.nm, '--defined-only', a.elf], capture_output=True, text=True, check=True).stdout.splitlines():
        p = l.split()
        if len(p) == 3:
            addr[p[2]] = int(p[0], 16)
    ents = re.findall(r'^\s+\S+:(\S+) \(noflash\)', open(a.check).read(), re.M)
    # esp32s3 internal RAM: IRAM for code, DRAM for (noflash) rodata such as vtables
    bad = [f for f in ents if not (0x3fc88000 <= addr.get(f, 0) < 0x403e0000)]
    for f in bad:
        print(f'{"missing" if f not in addr else "in flash":8s} {f}')
    objs = re.findall(r'^\s+([^:\s]+) \(noflash\)', open(a.check).read(), re.M)
    print(f'{len(ents)} symbol entries, {len(bad)} not in IRAM; {len(objs)} whole-object entries not checked', file=sys.stderr)
    if not ents and not objs:
        sys.exit(f'{a.check}: no (noflash) entries found')
    sys.exit(1 if bad else 0)

pcs = collections.Counter()
total = 0
headers = 0
declared = None
verified = False
for l in open(a.log, errors='replace'):
    m = re.search(r'rtprof samples=(\d+)', l)
    if m:
        headers += 1
        total = int(m.group(1))
        m = re.search(r'entries=(\d+)', l)
        declared = int(m.group(1)) if m else None
        m = re.search(r'elf=(\S*)', l)
        if m:
            if not re.fullmatch(r'[0-9a-f]{8,64}', m.group(1)):
                sys.exit(f'dump header has a malformed elf= field: {m.group(1)!r}')
            got = hashlib.sha256(open(a.elf, 'rb').read()).hexdigest()
            if not got.startswith(m.group(1)):
                sys.exit(f'{a.elf} is {got[:8]}, dump is from elf={m.group(1)}: '
                         f'etc/idf-devtools/elf_archive.py find --sha {m.group(1)} -o <elf>')
            verified = True
    i = l.find('rtp ')
    if i < 0:
        continue
    for m in re.finditer(r'([0-9a-f]{8}):(\d+)', l[i:]):
        pcs[int(m.group(1), 16)] += int(m.group(2))
if headers > 1:
    sys.exit(f'{headers} dumps in one log: split it, counts and totals would mix')
if declared is not None and declared != len(pcs):
    sys.exit(f'truncated dump: header says {declared} entries, parsed {len(pcs)}')
if not verified:
    if not a.unverified:
        sys.exit('dump has no header with an elf= sha: cannot tell that this ELF matches (--unverified to map anyway)')
    print('warning: ELF match unverified', file=sys.stderr)
if not pcs:
    sys.exit('no rtp lines')

syms = []
for l in subprocess.run([a.nm, '-S', '--defined-only', a.elf], capture_output=True, text=True, check=True).stdout.splitlines():
    p = l.split()
    if len(p) == 4 and p[2] in 'tTwW':
        syms.append((int(p[0], 16), int(p[1], 16), p[3]))
    elif len(p) == 3 and p[1] in 'tTwW' and not p[2].startswith('.'):  # assembly: no size
        syms.append((int(p[0], 16), 0, p[2]))
syms.sort()
starts = [s[0] for s in syms]
# a sizeless symbol runs up to the next one
syms = [(a_, sz or (syms[k + 1][0] - a_ if k + 1 < len(syms) else 1), n_) for k, (a_, sz, n_) in enumerate(syms)]

obj = {}
mp = a.map or os.path.splitext(a.elf)[0] + '.map'
if os.path.exists(mp):
    cur = None
    for l in open(mp, errors='replace'):
        m = re.match(r'^ \.(?:text|literal)\.(\S+)\s*$', l)
        if m:
            cur = m.group(1)
            continue
        # address 0 = discarded copy (COMDAT), not the one that was linked
        m = re.match(r'^ \.(?:text|literal)\.(\S+)\s+0x([0-9a-f]+)\s+0x[0-9a-f]+\s+(\S+)', l)
        if m:
            if int(m.group(2), 16):
                obj.setdefault(m.group(1), m.group(3))
            cur = None
            continue
        if cur:
            m = re.match(r'^\s+0x([0-9a-f]+)\s+0x[0-9a-f]+\s+(\S+)', l)
            if m and int(m.group(1), 16):
                obj.setdefault(cur, m.group(2))
            cur = None

fn = collections.Counter()
info = {}
unk = 0
for pc, n in pcs.items():
    # a return address points past the call; one inside the first instruction is an interrupted pc
    i = bisect.bisect_right(starts, pc) - 1
    q = pc if i >= 0 and pc - syms[i][0] < 3 else pc - 3
    i = bisect.bisect_right(starts, q) - 1
    if i < 0 or not (syms[i][0] <= q < syms[i][0] + max(syms[i][1], 1)):
        unk += n
        continue
    s = syms[i]
    if s[2] not in obj:  # the section is named after another alias of this address
        j = bisect.bisect_left(starts, s[0])
        while j < len(syms) and syms[j][0] == s[0]:
            if syms[j][2] in obj:
                s = syms[j]
                break
            j += 1
    fn[s[2]] += n
    info[s[2]] = s

if a.rate is not None:
    if not total:
        sys.exit('--rate needs the "rtprof samples=" header in the log')
    a.min = max(1, int(a.rate * total / a.hz))
secs = total / a.hz if total else 0
rows = [(n, info[f][1], f, obj.get(f, '?')) for f, n in fn.items() if n >= a.min]
rows.sort(key=lambda r: -r[0])
tot = sum(r[1] for r in rows)
print(f'{len(rows)} functions >= {a.min} samples, {tot} B ({tot / 1024:.1f} KB) text; unmapped samples {unk}'
      + (f'; {secs:.0f} s profiled, rate = samples/s on stack' if secs else ''))
for n, size, f, o in rows:
    print(f'{n:8d} {n / secs if secs else 0:8.1f} {size:6d}  {o}  {f}')

if a.lf:
    by = collections.defaultdict(set)
    for n, size, f, o in rows:
        m = re.match(r'(?:.*/)?(lib[^/(]+\.a)\(([^)]+)\)$', o)
        if not m or not re.fullmatch(r'[A-Za-z0-9_.]+', f):
            print(f'not placed (no object or unsupported name, use IRAM_ATTR): {n} {f}', file=sys.stderr)
            continue
        if m.group(1) in a.skip_archive:
            continue
        stem = re.sub(r'(\.(c|cpp|cc|S))?\.(obj|o)$', '', m.group(2))
        # ldgen has no symbol info for toolchain archives: place the whole (single-function) object
        by[m.group(1)].add((stem, f if o.startswith('esp-idf/') else None))
    for mf in a.merge:
        ar = None
        for l in open(mf):
            m = re.match(r'archive:\s*(\S+)', l)
            if m:
                ar = m.group(1)
            if ar in a.skip_archive:
                continue
            m = re.match(r'\s+([^:\s]+):(\S+) \(noflash\)', l)
            if m and ar:
                if m.group(2) in info or any(sy[2] == m.group(2) for sy in syms):
                    by[ar].add((m.group(1), m.group(2)))
                else:
                    print(f'dropped from {mf} (not in this ELF): {m.group(2)}', file=sys.stderr)
            m = re.match(r'\s+([^:\s]+) \(noflash\)', l)
            if m and ar:
                by[ar].add((m.group(1), None))
    with open(a.lf, 'w') as w:
        w.write('# generated by etc/rtprof_map.py --lf: RT hot path -> IRAM\n')
        for ar in sorted(by):
            w.write(f'\n[mapping:rtprof_{re.sub(r"[^A-Za-z0-9_]", "_", ar[:-2])}]\narchive: {ar}\nentries:\n')
            for stem, f in sorted(by[ar], key=lambda e: (e[0], e[1] or '')):
                w.write(f'    {stem}:{f} (noflash)\n' if f else f'    {stem} (noflash)\n')
    print(f'wrote {a.lf}: {sum(len(v) for v in by.values())} entries in {len(by)} archives', file=sys.stderr)
