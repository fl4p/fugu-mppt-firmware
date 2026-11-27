#!/usr/bin/env python3
"""Native end-to-end replay of the saved 4096-point corpus; outputs JSON results.
Run with .venv/bin/python test/host_py/pmini_native_replay.py.
"""
import ctypes as C, json, pathlib, struct, subprocess, sys, time, hashlib
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'etc'))
from fugu.teledec import decode_packet, decode_payload
import tamp
B=ROOT/'doc/research/pmini-benchmark-2026-11-26';D=pathlib.Path('/tmp/fugu-pmini-native-replay');D.mkdir(exist_ok=True)
inc=ROOT/'components/tamp/tamp/_c_src/tamp'
for name in ('common','compressor'):
 subprocess.run(['cc','-O2','-fPIC','-I'+str(inc),'-c',str(inc/(name+'.c')),'-o',str(D/(name+'.o'))],check=True)
sources=[B/'bridge.cpp',ROOT/'test/host-stub/pmini-replay-bridge.cpp',*[ROOT/'src/tele'/(x+'.cpp') for x in ('pmini_codec','pmini_packet','tele_packet','tamp_compress')]]
subprocess.run(['c++','-std=c++17','-O2','-shared','-fPIC','-I'+str(ROOT),'-I'+str(ROOT/'src'),'-I'+str(inc),*[str(x) for x in sources],str(D/'common.o'),str(D/'compressor.o'),'-o',str(D/'bridge.dylib')],check=True)
l=C.CDLL(str(D/'bridge.dylib'));l.point.argtypes=[C.c_int64,C.c_char_p,C.c_int,C.POINTER(C.c_char_p),C.c_char_p,C.POINTER(C.c_double),C.c_void_p];l.point.restype=C.c_size_t
l.compress_packet.argtypes=[C.c_char_p,C.c_size_t,C.c_void_p];l.compress_packet.restype=C.c_size_t
l.encode_packets.argtypes=[C.c_char_p,C.c_size_t,C.c_void_p,C.c_size_t];l.encode_packets.restype=C.c_size_t
buf=C.create_string_buffer(65536);meta=json.loads((B/'results.json').read_text());results=[];fixtures=[]
for dev in ('fry','flat'):
 s=json.loads((B/(dev+'.json')).read_text())['response']['results'][0]['series'][0]
 rows=[dict(zip(s['columns'],r)) for r in s['values']];keys=meta['sources'][dev]['fields'];types=meta['sources'][dev]['fmt'][4:]
 assert len(keys)==len(types)
 l.reset_table();l.reset_packets();wire=[]
 for r in rows:
  ks=[k for k in keys if r.get(k) is not None];vs=[r[k] for k in ks];ts=r['time'];ns=(C.c_char_p*len(ks))(*(k.encode() for k in ks));vals=(C.c_double*len(ks))(*vs);typ=''.join(types[keys.index(k)] for k in ks).encode()
  n=l.point(ts,dev.encode(),len(ks),ns,typ,vals,buf);wire.append(buf.raw[:n])
 batches=[];start=0
 for i in range(len(rows)):
  if i>start and (rows[i]['time']-rows[start]['time']>=1800 or sum(map(len,wire[start:i+1]))>8192):batches.append(b''.join(wire[start:i]));start=i
 batches.append(b''.join(wire[start:]));encoded=baseline=0;counts={0:0,1:0,2:0};us=[];maximum=0
 for raw in batches:
  n=l.compress_packet(raw,len(raw),buf);assert n;baseline+=1+n
  t=time.perf_counter_ns();n=l.encode_packets(raw,len(raw),buf,len(buf));us.append((time.perf_counter_ns()-t)/1000);assert n
  allbytes=buf.raw[:n];pos=0;decoded=b''
  while pos<n:
   length=struct.unpack_from('<H',allbytes,pos)[0];pos+=2;p=allbytes[pos:pos+length];pos+=length
   assert length<=1440;maximum=max(maximum,length);encoded+=length;counts[p[0]]+=1
   restored=p[1:] if p[0]==0 else tamp.decompress(p[1:]) if p[0]==1 else decode_packet(p[1:])
   decoded+=restored
   if p[0]==2:fixtures.append(dict(packet=p.hex(),raw=restored.hex()))
  assert decoded==raw,'roundtrip mismatch'
 results.append(dict(device=dev,points=len(rows),batches=len(batches),tamp_bytes=baseline,adaptive_bytes=encoded,saving_pct=100*(1-encoded/baseline),codec_counts=counts,max_payload=maximum,host_median_us=sorted(us)[len(us)//2],host_max_us=max(us)))
 print(json.dumps(results[-1]),flush=True)
# Cross-language golden packets produced by actual firmware code, for the JS check.
(D/'fixtures.json').write_text(json.dumps(fixtures))
report=dict(results=results,fingerprints={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
(B/'native-results.json').write_text(json.dumps(report,indent=2)+'\n')
