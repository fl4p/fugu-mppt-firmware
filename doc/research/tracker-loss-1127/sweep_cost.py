import re, datetime as dt, json, os, urllib.parse, urllib.request, collections
U=os.environ['INFLUX_HA_VAN_USER']; P=os.environ['INFLUX_HA_VAN_PASS']
def q(s):
    u='http://tm.fabi.me:8086/query?'+urllib.parse.urlencode({'db':'ha_van','u':U,'p':P,'q':s})
    r=json.load(urllib.request.urlopen(u,timeout=60))['results'][0]; return r.get('series',[])
f=lambda t:t.strftime('%Y-%m-%dT%H:%M:%SZ')
tot=collections.defaultdict(lambda:[0.,0]); 
for l in open('/tmp/scanlog.txt'):
    m=re.match(r'(\S+ \S+)\s+(\w+): .*mppt: Start sweep',l)
    if not m: continue
    t=dt.datetime.strptime(m.group(1)[:19],'%Y-%m-%d %H:%M:%S'); dev=m.group(2)
    s=(f"SELECT mean(P) FROM mppt WHERE device='{dev}' AND time>='{f(t-dt.timedelta(seconds=40))}' AND time<'{f(t-dt.timedelta(seconds=5))}';"
        f"SELECT integral(P,1h) FROM mppt WHERE device='{dev}' AND time>='{f(t)}' AND time<'{f(t+dt.timedelta(seconds=20))}';"
        f"SELECT mean(P) FROM mppt WHERE device='{dev}' AND time>='{f(t+dt.timedelta(seconds=25))}' AND time<'{f(t+dt.timedelta(seconds=60))}'")
    u='http://tm.fabi.me:8086/query?'+urllib.parse.urlencode({'db':'ha_van','u':U,'p':P,'q':s})
    try: rs=json.load(urllib.request.urlopen(u,timeout=60))['results']
    except Exception as e: print('err', dev, t, getattr(e,'read',lambda:b'')()[:120]); continue
    v=[r['series'][0]['values'][0][1] if r.get('series') else None for r in rs]
    if None in v: continue
    ref=(v[0]+v[2])/2; loss=ref*20/3600-v[1]
    tot[dev][0]+=loss; tot[dev][1]+=1
for dev,(l,n) in tot.items(): print(dev, n, 'sweeps', round(l,1), 'Wh', round(l/n*1000/1, 0), 'mWh/sweep')
