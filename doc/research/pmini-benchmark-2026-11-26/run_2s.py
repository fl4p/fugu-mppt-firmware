#!/usr/bin/env python3
"""Replay real DB rows, not captured wire: see report for precision/schema limits."""
import array, ctypes as C, hashlib, io, json, pathlib, statistics, struct, subprocess, sys, time
sys.dont_write_bytecode=True
HERE=pathlib.Path(__file__).resolve().parent
ROOT=HERE.parents[2]
PMINI=pathlib.Path(sys.argv[1] if len(sys.argv)>1 else '/Users/fab/dev/esp/pmini')
sys.path.insert(0,str(PMINI)); sys.path.insert(0,str(ROOT/'etc'))
from pmini import pmini
from fugu.teledec import Reader, decode_payload, DT
import tamp
build=pathlib.Path('/tmp/fugu-pmini-benchmark'); build.mkdir(exist_ok=True)
cdir=ROOT/'components/tamp/tamp/_c_src/tamp'
for name in ('common','compressor'):
    subprocess.run(['cc','-O2','-fPIC','-I'+str(cdir),'-c',str(cdir/(name+'.c')),'-o',str(build/(name+'.o'))],check=True)
subprocess.run(['c++','-std=c++17','-O2','-shared','-fPIC','-I'+str(ROOT),'-I'+str(cdir),str(HERE/'bridge.cpp'),str(ROOT/'src/tele/tamp_compress.cpp'),str(build/'common.o'),str(build/'compressor.o'),'-o',str(build/'bridge.dylib')],check=True)
lib=C.CDLL(str(build/'bridge.dylib'))
lib.point.argtypes=[C.c_int64,C.c_char_p,C.c_int,C.POINTER(C.c_char_p),C.c_char_p,C.POINTER(C.c_double),C.c_void_p];lib.point.restype=C.c_size_t
lib.compress_packet.argtypes=[C.c_char_p,C.c_size_t,C.c_void_p];lib.compress_packet.restype=C.c_size_t
buf=C.create_string_buffer(1024*1024)
def compress(b):
    n=lib.compress_packet(b,len(b),buf)
    assert n
    return buf.raw[:n]
def varint(v):
    b=bytearray()
    while v>=128: b.append((v&127)|128);v>>=7
    b.append(v);return bytes(b)
def data_only(wire):
    r=Reader(wire);out=b''
    while not r.eof():
        frame=r.take(r.varint())
        if frame[0]==1:out+=varint(len(frame))+frame
    return out
ORDER='I Ui Uo P P_smooth E E_today pwm_dir_f mppt_state P_max_today Ro Rc Rb wf Pw hs_short bf_fail mcu_temp ntc_temp lag rect_off heap heap_min pwm_duty Vset pwm_ls_duty pwm_ls_max pwm_dcm P_filt P_prev dP dP_thres cv_lim_idx'.split()
INTS=set('mppt_state wf hs_short bf_fail lag rect_off heap heap_min pwm_duty pwm_ls_duty pwm_ls_max cv_lim_idx'.split())
results=[];sources={}
for dev in ('fry','flat'):
    path=HERE/(dev+'.json');data=json.loads(path.read_text());s=data['response']['results'][0]['series'][0]
    rows=[dict(zip(s['columns'],r)) for r in s['values']]
    keys=[k for k in ORDER if any(r.get(k) is not None for r in rows)]
    assert len(keys)<=32
    assert not ({k for r in rows for k,v in r.items() if v is not None}-set(keys)-{'time','device'})
    types=''.join('i' if k in INTS else 'B' if k=='pwm_dcm' else 'f' for k in keys)
    fmt='<III'+types # timestamp low/high, field presence, values in fixed schema order
    lib.reset_table();wire=[];steady=[];packed=[];decoded_table={}
    for r in rows:
        ks=[k for k in keys if r.get(k) is not None]
        vs=[r[k] for k in ks];ts=r['time']
        ns=(C.c_char_p*len(ks))(*(k.encode() for k in ks)); vals=(C.c_double*len(ks))(*vs)
        typ=''.join(types[keys.index(k)] for k in ks).encode()
        n=lib.point(ts,dev.encode(),len(ks),ns,typ,vals,buf);w=buf.raw[:n]
        assert len(list(decode_payload(0,w,decoded_table)))==1
        wire.append(w);steady.append(data_only(w))
        presence=sum(1<<i for i,k in enumerate(keys) if r.get(k) is not None)
        values=[(int(r[k]) if t in 'iB' else r[k]) if r.get(k) is not None else 0 for k,t in zip(keys,types)]
        packed.append(struct.pack(fmt,ts&0xffffffff,ts>>32,presence,*values))
        # Check that the C++ baseline and proposed rows carry identical fields/bits/time.
        rr=Reader(steady[-1]); frame=Reader(rr.take(rr.varint())); assert frame.byte()==1
        assert decoded_table[frame.varint()][0]=='mppt'
        while frame.b[frame.i]: frame.varint(); frame.varint()
        frame.byte(); actual={}
        while frame.b[frame.i]:
            sid=frame.varint(); name,dt=decoded_table[sid]; actual[name]=frame.take(DT[dt][1])
        frame.byte(); assert frame.varint()==ts and frame.eof() and rr.eof()
        expected={k:struct.pack('<'+t,v) for k,t,v in zip(keys,types,values) if r.get(k) is not None}
        assert actual==expected
    spans=[rows[i+1]['time']-rows[i]['time'] for i in range(len(rows)-1)]
    sources[dev]=dict(query=data['query'],sha256=hashlib.sha256(path.read_bytes()).hexdigest(),n=len(rows),start_ms=rows[0]['time'],end_ms=rows[-1]['time'],median_interval_ms=statistics.median(spans),fields=keys,fmt=fmt)
    policies=[('latency-1800ms',None,1800,None),('latency-2000ms',None,2000,None)]
    for label,count,ms,cap in policies:
        batches=[];start=0;size=0
        for i in range(len(rows)):
            if i>start and ((count and i-start>=count) or (ms and rows[i]['time']-rows[start]['time']>=ms) or (cap and ((label.startswith('ble-') and size>=cap) or (not label.startswith('ble-') and size+len(wire[i])>cap)))):
                batches.append((start,i));start=i;size=0
            size+=len(wire[i])
        if start<len(rows):batches.append((start,len(rows)))
        maxn=max(b-a for a,b in batches);ws=pmini.Workspace(fmt,maxn)
        totals=dict(tamp_wire=0,tamp_steady=0,pmini=0,tamp_columnar=0,raw_columnar=0)
        timings={k:[] for k in ('tamp','pmini')};pmax=0;roundtrips=0
        for a,b in batches:
            full=b''.join(wire[a:b]);st=b''.join(steady[a:b]);raw=b''.join(packed[a:b]);n=b-a
            t=time.perf_counter_ns();tc=compress(st);timings['tamp'].append((time.perf_counter_ns()-t)/1000)
            out=io.BytesIO();t=time.perf_counter_ns();pmini.encode(ws,io.BytesIO(raw),n,out.write);pc=out.getvalue();timings['pmini'].append((time.perf_counter_ns()-t)/1000)
            assert pmini.decode_frames(pc,fmt[1:])==raw
            assert tamp.decompress(tc)==st
            fc=compress(full)
            fs=struct.calcsize(fmt);offset=0;cols=[]
            for code in fmt[1:]:
                width=struct.calcsize('<'+code)
                cols.append(b''.join(raw[j*fs+offset:j*fs+offset+width] for j in range(n)))
                offset+=width
            columnar=b''.join(cols);cc=compress(columnar)
            assert tamp.decompress(fc)==full and tamp.decompress(cc)==columnar
            totals['tamp_wire']+=1+len(fc);totals['tamp_steady']+=1+len(tc)
            totals['pmini']+=2+len(pc) # proposed codec + fixed schema id
            totals['tamp_columnar']+=2+len(cc);totals['raw_columnar']+=2+len(raw)
            pmax=max(pmax,len(pc)+2);roundtrips+=1
        def bufferbytes(v):
            if isinstance(v,(bytes,bytearray,array.array)):return len(v)*(v.itemsize if isinstance(v,array.array) else 1)
            if isinstance(v,list):return sum(bufferbytes(x) for x in v)
            return 0
        result=dict(device=dev,policy=label,batches=len(batches),rows=len(rows),mean_rows=len(rows)/len(batches),**totals,pmini_saving_pct=100*(1-totals['pmini']/totals['tamp_steady']),pmini_max_packet=pmax,workspace_buffer_bytes=sum(bufferbytes(v) for v in vars(ws).values()),roundtrips=roundtrips,median_batch_span_ms=statistics.median(rows[b-1]['time']-rows[a]['time'] for a,b in batches),host_median_us={k:statistics.median(v) for k,v in timings.items()})
        results.append(result);print(dev,label,round(result['pmini_saving_pct'],1),totals,flush=True)
files=[ROOT/'src/tele/sym_line_protocol.h',ROOT/'src/tele/tamp_compress.cpp',HERE/'run_2s.py',HERE/'bridge.cpp',*cdir.glob('*.[ch]'),PMINI/'pmini/pmini.py',PMINI/'pmini/pmini_py.py']
(HERE/'results_2s.json').write_text(json.dumps(dict(sources=sources,results=results,fingerprints={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},python=sys.version),indent=2))
