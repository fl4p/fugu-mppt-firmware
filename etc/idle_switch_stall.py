#!/usr/bin/env python3
# RT-loop gap caused by the idle profile's CPU clock switch (needs a CONFIG_FUGU_WITH_PWR_KNOBS build).
# Forces the profile on and off with `pwr idle 1|0` and polls `pwr gap` (max RT loop period since the
# last poll, converter on or off). The poll interval that contains a clock change is the switch
# interval; its max gap is max(switch stall, normal loop period), so the steady maxima at both clocks
# and a no-switching baseline over the same duration are reported next to it.
#
#   idle_switch_stall.py -p /dev/cu.usbmodem1101 --cycles 20 [--setup "dc 0;pwr rt 1470"] [--csv /tmp/stall.csv]
#
# --setup runs first (default `dc 0`: manual duty 0, so an MPPT start on the mock cannot keep the
# profile off). The profile is left in `pwr idle auto` at the end.
import argparse, re, statistics, sys, time
import serial

ap = argparse.ArgumentParser()
ap.add_argument('-p', '--port', required=True)
ap.add_argument('--cycles', type=int, default=20)
ap.add_argument('--poll', type=float, default=0.25, help='seconds between `pwr gap` polls')
ap.add_argument('--setup', default='dc 0', help="commands before the run, ';'-separated")
ap.add_argument('--timeout', type=float, default=20, help='max seconds per enter/exit phase')
ap.add_argument('--baseline', type=float, help='baseline seconds (default: the cycles\' duration)')
ap.add_argument('--csv')
args = ap.parse_args()

gap_re = re.compile(r'pwr: gap max=(\d+) us dt=(\d+) ms sps=(\S+) cpu=(\d+)')
st_re = re.compile(r'pwr: cpu=(\d+) .*\bidle=(\d)/(\w+) icpu=(\S+) .*\brt=(\S+)')
s = serial.Serial(args.port, 115200, timeout=0.05)
time.sleep(1.0)
s.write(b'\n')  # the first bytes after open can arrive corrupted
time.sleep(0.5)
s.reset_input_buffer()
buf = b''
csv = open(args.csv, 'w') if args.csv else None
if csv:
    csv.write('t,phase,cycle,gap_us,dt_ms,sps,cpu,icpu,rt\n')
t0 = time.time()


def send(cmd):
    s.write((cmd + '\n').encode())
    s.flush()


def lines(dur):
    global buf
    end = time.time() + dur
    while time.time() < end:
        buf += s.read(512)
        while b'\n' in buf:
            raw, buf = buf.split(b'\n', 1)
            yield raw.decode('utf-8', 'replace').strip()


def poll(phase, cycle):
    """One `pwr gap`: (gap_us, sps, cpu, icpu, rt) or None."""
    send('pwr gap')
    g = None
    for ln in lines(args.poll):
        if 'idle profile' in ln or 'ERR' in ln or 'usage' in ln:
            print(f'   {ln[:150]}', flush=True)
        m = gap_re.search(ln)
        if m:
            g = [int(m.group(1)), int(m.group(2)), float(m.group(3)), int(m.group(4))]
            continue
        m = st_re.search(ln)
        if m and g:
            r = (g[0], g[2], int(m.group(1)), m.group(4), float(m.group(5)))
            if csv:
                csv.write(f'{time.time() - t0:.2f},{phase},{cycle},{g[0]},{g[1]},{g[2]},{r[2]},{r[3]},{r[4]}\n')
            return r
    return None


carry_cpu = None  # cpu= of the last poll, so a switch in the first poll after a command counts


def run(cmd, phase, cycle, done):
    """Send cmd, poll until done(icpu); split the gaps into the switch interval and the rest."""
    send(cmd)
    t = time.time()
    global carry_cpu
    prev_cpu, sw, rest, sps, last = carry_cpu, [], [], {}, None
    while time.time() - t < args.timeout:
        r = poll(phase, cycle)
        if not r:
            continue
        gap, rate, cpu, icpu, rt = r
        if prev_cpu is not None and cpu != prev_cpu:
            sw.append(gap)
        elif prev_cpu is not None:
            rest.append(gap)
            if rate == rate:
                sps.setdefault(cpu, []).append(rate)
        prev_cpu, last = cpu, r
        carry_cpu = cpu
        if done(icpu):
            return sw, rest, sps, last
    return sw, rest, sps, last


for c in filter(None, (x.strip() for x in args.setup.split(';'))):
    send(c)
    for ln in lines(1.0):
        if 'pwr' in ln or 'ERR' in ln:
            print(f'   {ln[:150]}', flush=True)

# start from the boot clock, settled
_, _, _, last = run('pwr idle 0', 'init', 0, lambda st: st in ('boot', 'latched', '-'))
if not last or last[3] == '-':
    sys.exit(f'no idle CPU part (icpu={last and last[3]}): board.conf idle_cpu_mhz 0 or same as boot')
if last[3] == 'latched':
    sys.exit('CPU part latched off by the guard since boot: reboot, lower the load (pwr rt) and retry')
boot = last[2]

enter, exit_, steady, rates, rt_at = [], [], {}, {}, {}
tc0 = time.time()
for i in range(args.cycles):
    sw, rest, sps, last = run('pwr idle 1', 'enter', i, lambda st: st in ('low', 'latched'))
    if not last or last[3] != 'low':
        print(f'[{i}] enter: icpu={last and last[3]} -> stop (guard tripped or blocked)', flush=True)
        break
    idle_mhz = last[2]
    rt_at[idle_mhz] = last[4]
    for k, v in sps.items():
        rates.setdefault(k, []).extend(v)
    steady.setdefault(idle_mhz, []).extend(rest)
    e = max(sw) if sw else None
    sw2, rest2, sps2, last2 = run('pwr idle 0', 'exit', i, lambda st: st == 'boot')
    if not last2 or last2[3] != 'boot':
        print(f'[{i}] exit: icpu={last2 and last2[3]} -> stop', flush=True)
        break
    rt_at[boot] = last2[4]
    for k, v in sps2.items():
        rates.setdefault(k, []).extend(v)
    steady.setdefault(boot, []).extend(rest2)
    x = max(sw2) if sw2 else None
    enter.append(e)
    exit_.append(x)
    print(f'[{i}] enter {idle_mhz} MHz: switch-interval gap {e} us (steady max {max(rest, default=0)} us) | '
          f'exit {boot} MHz: {x} us (steady max {max(rest2, default=0)} us)', flush=True)
dur = time.time() - tc0

base = []
send('pwr idle 0')
tb = time.time()
while time.time() - tb < (args.baseline or dur):
    r = poll('baseline', -1)
    if r:
        base.append(r[0])
send('pwr idle auto')
for _ in lines(0.5):
    pass


def summ(name, v):
    v = [x for x in v if x is not None]
    if not v:
        print(f'  {name:34s} none')
        return
    print(f'  {name:34s} n={len(v):4d} max={max(v):6d} us  median={statistics.median(v):8.0f} us')


print(f'\n{len(enter)} cycles in {dur:.0f} s, poll {args.poll} s')
summ('enter switch interval', enter)
summ('exit switch interval', exit_)
for mhz, v in sorted(steady.items()):
    summ(f'steady intervals at {mhz} MHz', v)
summ(f'baseline at {boot} MHz, no switching', base)
for mhz, v in sorted(rates.items()):
    print(f'  rate at {mhz} MHz: measured {statistics.median(v):.0f}/s (median of {len(v)} polls), rt={rt_at.get(mhz)}')
s.close()
