import re, datetime as dt, json, os, urllib.parse, urllib.request, collections
U=os.environ['INFLUX_HA_VAN_USER']; P=os.environ['INFLUX_HA_VAN_PASS']
def q(s):
    u='http://tm.fabi.me:8086/query?'+urllib.parse.urlencode({'db':'ha_van','u':U,'p':P,'q':s})
    r=json.load(urllib.request.urlopen(u,timeout=60))['results'][0]
    return r.get('series',[])
ts=lambda s: dt.datetime.strptime(s[:19],'%Y-%m-%d %H:%M:%S')
last_scan={}; last_sweep={}; rows=[]
for l in open('/tmp/scanlog.txt'):
    m=re.match(r'(\S+ \S+)\s+(\w+): .*mppt: (.*)',l)
    if not m: continue
    t,dev,msg=ts(m.group(1)),m.group(2),m.group(3)
    if msg.startswith('Start sweep'): last_sweep[dev]=t; continue
    s=re.match(r'scan: P ([\d.]+)->([\d.]+) W at ([\d.]+)V, \d+ probes, best x([\d.]+) at ([\d.]+)V(, moving)?',msg)
    if not s: continue
    if s.group(6):
        p0=float(s.group(1)); x=float(s.group(4))
        up=dt.timedelta(milliseconds=int(re.search(r'\((\d+)\)',l).group(1))); boot=t-up
        b=max(last_sweep.get(dev,boot),boot); a=max(last_scan.get(dev,b),b)
        rows.append((t,dev,p0,x,(t-a).total_seconds(),(t-b).total_seconds()))
    last_scan[dev]=t
def avgP(dev,t0,t1):
    f=lambda t:(t-dt.timedelta(hours=0)).strftime('%Y-%m-%dT%H:%M:%SZ')
    s=q(f"SELECT mean(P) FROM mppt WHERE device='{dev}' AND time>='{f(t0)}' AND time<'{f(t1)}'")
    return s[0]['values'][0][1] if s else None
day=collections.defaultdict(lambda:[0,0]); print('time              dev  P0    x     lo_min hi_min  P-2m  P+2..4m real  lossWh lo..hi')
for t,dev,p0,x,lo,hi in rows:
    pb=avgP(dev,t-dt.timedelta(minutes=2),t); pa=avgP(dev,t+dt.timedelta(minutes=2),t+dt.timedelta(minutes=4))
    g=(x-1)*p0/3600; d=t.date()
    day[(d,dev)][0]+=g*lo; day[(d,dev)][1]+=g*hi
    print(f'{t} {dev:4} {p0:5.0f} {x:.3f} {lo/60:6.1f} {hi/60:6.1f} {pb or 0:5.0f} {pa or 0:6.0f} {(pa/pb if pa and pb else 0):.2f} {g*lo:6.1f}..{g*hi:.1f}')
print()
for (d,dev),(lo,hi) in sorted(day.items()):
    s=q(f"SELECT integral(P,1h) FROM mppt WHERE device='{dev}' AND time>='{d}T00:00:00Z' AND time<'{d+dt.timedelta(days=1)}T00:00:00Z'")
    y=s[0]['values'][0][1] if s else 0
    print(f'{d} {dev:4} yield {y:6.0f} Wh  loss {lo:5.1f}..{hi:5.1f} Wh = {100*lo/y if y else 0:.2f}..{100*hi/y if y else 0:.2f} %')
