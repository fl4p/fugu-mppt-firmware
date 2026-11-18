#!/usr/bin/env python3
"""Generate test/fixtures/tele_fixtures.json for the teledec.js tests.

Run with the repo venv (tamp 2.3.0):
  .venv/bin/python3 etc/config-tool/test/gen_tele_fixtures.py

Expected values come from tamp (Python) and etc/fugu/teledec.py, the reference decoder.
"""
import base64
import io
import json
import random
import struct
import subprocess
import sys
from pathlib import Path

import tamp

assert tamp.__version__ == '2.3.0', f'expected tamp 2.3.0, got {tamp.__version__}'

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]


def _find_teledec():
    p = REPO / 'etc' / 'fugu'
    if (p / 'teledec.py').exists():
        return p
    # a worktree without the fugu-py submodule: use the main checkout's
    common = subprocess.check_output(['git', 'rev-parse', '--path-format=absolute', '--git-common-dir'],
                                     cwd=REPO, text=True).strip()
    p = Path(common).parent / 'etc' / 'fugu'
    if (p / 'teledec.py').exists():
        return p
    sys.exit('etc/fugu/teledec.py not found (git submodule update --init etc/fugu)')


TELEDEC_DIR = _find_teledec()
sys.path.insert(0, str(TELEDEC_DIR))
import teledec  # noqa: E402

SEED = 20261117
rng = random.Random(SEED)


def b64(b):
    return base64.b64encode(bytes(b)).decode()


def tcomp(data):
    c = tamp.compress(bytes(data), window=10, literal=8, extended=False)
    assert c[0] == 0x58, hex(c[0])
    return c


# --- encoder mirroring src/tele/sym_line_protocol.h ------------------------------------------

STR, BOOL, I8, U8, I16, U16, I32, U32, F16, F32, F64 = range(11)
PACK = {BOOL: '<B', I8: '<b', U8: '<B', I16: '<h', U16: '<H', I32: '<i', U32: '<I', F16: '<H', F32: '<f',
        F64: '<d'}


def varint(v):
    out = bytearray()
    while v >= 0x80:
        out.append((v & 0x7F) | 0x80)
        v >>= 7
    out.append(v)
    return bytes(out)


def f32_to_f16(f):
    """Port of f32ToF16() (round half up, saturate to inf, flush tiny to 0)."""
    x = struct.unpack('<I', struct.pack('<f', f))[0]
    sign = (x >> 16) & 0x8000
    raw_e = (x >> 23) & 0xFF
    exp = raw_e - 127 + 15
    mant = x & 0x7FFFFF
    if raw_e == 0xFF:
        return sign | 0x7C00 | (0x200 if mant else 0)
    if exp >= 0x1F:
        return sign | 0x7C00
    if exp <= 0:
        if exp < -10:
            return sign
        mant |= 0x800000
        shift = 14 - exp
        h = (mant >> shift) & 0xFFFF
        if (mant >> (shift - 1)) & 1:
            h += 1
        return (sign | h) & 0xFFFF
    h = sign | (exp << 10) | (mant >> 13)
    if mant & 0x1000:
        h += 1
    return h & 0xFFFF


class SymbolTable:
    def __init__(self):
        self.ids, self.names, self.dts = {}, [], []

    def intern(self, s, dt):
        if s in self.ids:
            return self.ids[s]
        self.names.append(s)
        self.dts.append(dt)
        self.ids[s] = len(self.names)
        return self.ids[s]

    def full_table(self):
        out = bytearray([2])
        for sid in range(1, len(self.names) + 1):
            out += varint(sid) + self.names[sid - 1].encode() + b'\0' + bytes([self.dts[sid - 1]])
        out.append(0)
        return bytes(out)


def data_frame(tab, meas, tags, fields, ts):
    """fields: [(key, dt, value)]; value for F16 is a float (encoded via f32_to_f16) or ('raw', u16)."""
    body = bytearray(varint(tab.intern(meas, STR)))
    for k, v in tags:
        body += varint(tab.intern(k, STR)) + varint(tab.intern(v, STR))
    body.append(0)
    for k, dt, v in fields:
        body += varint(tab.intern(k, dt))
        if dt == F16:
            v = v[1] if isinstance(v, tuple) else f32_to_f16(v)
        elif dt == BOOL:
            v = 1 if v else 0
        body += struct.pack(PACK[dt], v)
    body.append(0)
    body += varint(ts)
    return bytes([1]) + bytes(body)


def frames(*fs):
    out = bytearray()
    for f in fs:
        out += varint(len(f)) + f
    return bytes(out)


def ref_decode(cid, payload, tab):
    lines, err = [], None
    try:
        for ln in teledec.decode_payload(cid, payload, tab):
            lines.append(ln)
    except Exception as e:  # noqa: BLE001 - mirror TeleStream: partial lines are kept
        err = type(e).__name__
    return lines, err


# --- tamp cases -------------------------------------------------------------------------------

WORDS = ('vin vout iout power mppt sweep tracker duty buck coil charger battery cell float '
         'absorb limit protect sensor notch median filter wifi mqtt ble telemetry').split()


def text_2k():
    s = []
    while sum(len(w) + 1 for w in s) < 2048:
        s.append(rng.choice(WORDS))
        if rng.random() < 0.1:
            s.append('\n')
    return ' '.join(s).encode()[:2048]


def tele_like():
    tab = SymbolTable()
    raw = bytearray()
    for i in range(12):
        f = data_frame(tab, 'mppt', [('device', 'fry')],
                       [('Ui', F32, 40.0 + i * 0.25), ('Uo', F32, 27.1), ('P', F32, 150.0 + i),
                        ('pwm_duty', U16, 1800 + i), ('mppt_state', U8, 2)], 1763400000000 + 1000 * i)
        raw += frames(tab.full_table(), f) if i == 0 else frames(f)
    return bytes(raw)


def flush_stream():
    f = io.BytesIO()
    c = tamp.Compressor(f, window=10, literal=8, extended=False)
    raw = bytearray()
    for chunk in (b'hello hello hello', b'~~~~~~~~', b'world world world world', b'x'):
        c.write(chunk)
        raw += chunk
        c.flush(write_token=True)
    c.write(b'tail tail tail')
    raw += b'tail tail tail'
    c.close()
    out = f.getvalue()
    assert out[0] == 0x58
    assert bytes(tamp.decompress(out)) == bytes(raw)
    return out, bytes(raw)


tamp_cases = []


def add_tamp(name, raw, comp=None):
    comp = tcomp(raw) if comp is None else comp
    dec = bytes(tamp.decompress(comp))
    assert dec == bytes(raw), name
    tamp_cases.append({'name': name, 'raw': b64(raw), 'compressed': b64(comp)})


add_tamp('empty', b'')
add_tamp('one_byte', b'A')
add_tamp('run_a_2000', b'a' * 2000)
add_tamp('run_mixed', b'ab' * 300 + b'\0' * 500 + b'xyz' * 200)
add_tamp('random_600', bytes(rng.getrandbits(8) for _ in range(600)))
add_tamp('tilde_laden', bytes(rng.choice((0x7E, 0x7E, 0x7D, 0x00, 0x01)) for _ in range(800)))
add_tamp('tele_like', tele_like())
add_tamp('text_2k', text_2k())
add_tamp('dictionary_words', b'<a href="index.html">tea</a>\n' * 20)
fc, fr = flush_stream()
add_tamp('flush_tokens', fr, fc)

ext = tamp.compress(b'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa' * 4 + b'abc' * 40, window=10, literal=8,
                    extended=True)
assert ext[0] & 0x02, hex(ext[0])
tamp_reject = [{'name': 'extended_header', 'compressed': b64(ext)}]

# truncated streams: Python stops at EOF mid-token, keeping the bytes decoded so far
tamp_truncated = []
for name, raw in (('text', text_2k()[:600]), ('runs', b'abc' * 100 + b'z' * 300)):
    comp = tcomp(raw)
    for cut in (1, len(comp) // 3, len(comp) // 2, len(comp) - 1):
        tamp_truncated.append({'name': f'{name}_cut{cut}', 'compressed': b64(comp[:cut]),
                               'raw': b64(tamp.decompress(comp[:cut]))})

# --- payload cases ----------------------------------------------------------------------------

payload_cases = []


def add_seq(name, payloads):
    """payloads: [(cid, raw_payload_bytes)]; one table shared across the sequence."""
    tab = {}
    seq = []
    for cid, raw in payloads:
        p = tcomp(raw) if cid == 1 else raw
        lines, err = ref_decode(cid, p, tab)
        seq.append({'cid': cid, 'payload': b64(p), 'lines': lines, 'error': err})
    payload_cases.append({'name': name, 'sequence': seq})


def all_dt_payloads(tab, ts0):
    pts = [
        data_frame(tab, 'dt', [('device', 'flat'), ('board', 'fugu2')], [
            ('b1', BOOL, True), ('b0', BOOL, False),
            ('i8', I8, -128), ('i8p', I8, 127), ('u8', U8, 255),
            ('i16', I16, -32768), ('u16', U16, 65535),
            ('i32', I32, -2147483648), ('i32p', I32, 2147483647),
            ('u32', U32, 4294967295), ('u32b', U32, 2147483649),
            ('h01', F16, 0.1), ('hmax', F16, 65504.0), ('hneg', F16, -2.5), ('hinf', F16, float('inf')),
            ('hninf', F16, float('-inf')), ('hnz', F16, -0.0), ('hsub', F16, ('raw', 0x0001)),
            ('hnan', F16, ('raw', 0x7E00)), ('hover', F16, 1e6),
            ('ftie', F32, 123456.5), ('f01', F32, 0.1), ('fbig', F32, 3.4028234663852886e38),
            ('fsmall', F32, 1.401298464324817e-45), ('fneg', F32, -1234.5678),
            ('dtie', F64, 123456.5), ('dtie7', F64, 1234565.0), ('d999', F64, 999999.5),
            ('dmin', F64, 5e-324), ('dmax', F64, 1.7976931348623157e308), ('dnz', F64, -0.0),
            ('dnan', F64, float('nan')), ('dinf', F64, float('-inf')), ('dsmall', F64, 0.0001),
            ('dsmall5', F64, 0.00001234565), ('d1e16', F64, 1e16), ('dint', F64, 100000.0),
            ('dint6', F64, 1000000.0), ('dfrac', F64, 2.5e-7), ('dtie_e', F64, 0.5),
            ('dtie2', F64, 2.5), ('dneg_tie', F64, -123456.5),
        ], ts0),
    ]
    return pts


tab = SymbolTable()
p0 = all_dt_payloads(tab, 4294967296123)  # ts > 2^32
first = frames(tab.full_table(), *p0)
second = frames(data_frame(tab, 'dt', [('device', 'flat'), ('board', 'fugu2')],
                           [('u32', U32, 3000000000), ('ftie', F32, -0.0)], 1763400000123))
add_seq('all_dt_raw', [(0, first), (0, second)])
add_seq('all_dt_tamp', [(1, first), (1, second)])

# several points per payload, tagless point, fieldless point, ts 0
tab = SymbolTable()
mp = [data_frame(tab, 'mppt', [('device', 'fry')], [('Ui', F32, 41.5), ('P', F32, 120.25)], 1763400000000),
      data_frame(tab, 'sys', [], [('heap', U32, 123456)], 1763400000500),
      data_frame(tab, 'mppt', [('device', 'fry'), ('phase', 'sweep')], [('Ui', F32, 38.0)], 1),
      data_frame(tab, 'empty', [], [], 0)]
add_seq('multi_point', [(0, frames(tab.full_table(), *mp)), (1, frames(*mp[:2]))])

# >127 symbols: multi-byte SID varints
tab = SymbolTable()
big = [data_frame(tab, 'big', [('device', 'fbuck')], [(f'k{j:03d}', U16, j) for j in range(i, i + 20)],
                  1763400000000 + i) for i in range(0, 140, 20)]
add_seq('big_table', [(0, frames(tab.full_table(), *big))])

# data-only payload with an unknown SID (table frame lost)
tab = SymbolTable()
d = data_frame(tab, 'mppt', [('device', 'fry')], [('Ui', F32, 40.0)], 1763400000000)
add_seq('unknown_sid', [(0, frames(d)), (0, frames(tab.full_table(), d)), (1, frames(d))])

# partial: first frame decodes, second references a SID only later defined
tab = SymbolTable()
a = data_frame(tab, 'mppt', [], [('Ui', F32, 40.0)], 10)
t1 = tab.full_table()
b = data_frame(tab, 'mppt', [], [('NEW', F32, 1.0)], 11)
add_seq('partial_then_error', [(0, frames(t1, a, b)), (0, frames(tab.full_table(), b))])

# data frames with an invalid field type (DT 11, DT 0 = Str) are skipped; the good frame decodes
tab = SymbolTable()
good = data_frame(tab, 'mppt', [], [('P', F32, 1.0)], 6)
sx, sm = tab.intern('X', 11), tab.ids['mppt']
bad11 = bytes([1]) + varint(sm) + b'\0' + varint(sx) + b'\x00\x00\x80\x3f' + b'\0' + varint(5)
bad0 = bytes([1]) + varint(sm) + b'\0' + varint(sm) + b'\0' + varint(4)
add_seq('bad_field_dt', [(0, frames(tab.full_table(), bad11, good)), (0, frames(bad0, good))])
assert [s['lines'] for s in payload_cases[-1]['sequence']] == [['mppt P=1 6']] * 2, payload_cases[-1]

# unknown frame type, unknown cid, truncated data frame
add_seq('bad_frame_type', [(0, frames(b'\x03abc'))])
add_seq('bad_cid', [(2, b'\x01\x01')])
tab = SymbolTable()
d = data_frame(tab, 'm', [], [('x', U32, 7)], 5)
add_seq('truncated_frame', [(0, frames(tab.full_table()) + varint(len(d)) + d[:-3])])

# --- framing streams --------------------------------------------------------------------------


def record(cid, raw):
    p = tcomp(raw) if cid == 1 else raw
    return b'\x7e' + varint(len(p) + 1) + bytes([cid]) + p


class _Counting(teledec.TeleStream):
    def __init__(self):
        self.resyncs, self.errors = 0, 0
        super().__init__(on_error=self._err)

    def _err(self, e):
        self.errors += 1

    def _resync(self):
        self.resyncs += 1
        super()._resync()


def ref_stream(stream):
    calls = [0]
    orig = teledec.decode_payload

    def counted(*a):
        calls[0] += 1
        return orig(*a)

    teledec.decode_payload = counted
    try:
        ts = _Counting()
        lines = ts.feed(stream)
        # chunked feeding must give the same result
        ts2 = _Counting()
        lines2 = []
        for i in range(0, len(stream), 7):
            lines2 += ts2.feed(stream[i:i + 7])
        assert lines2 == lines and ts2.errors == ts.errors, (len(lines), len(lines2), ts.errors, ts2.errors)
    finally:
        teledec.decode_payload = orig
    return lines, {'records': calls[0] // 2 - ts.errors, 'resyncs': ts.resyncs, 'errors': ts.errors,
                   'bytes': len(stream), 'pending': len(ts.buf)}


framing_cases = []


def add_stream(name, stream):
    lines, stats = ref_stream(bytes(stream))
    framing_cases.append({'name': name, 'stream': b64(stream), 'lines': lines, 'stats': stats})


def point_stream(n, tab, ts0=1763400000000):
    recs = []
    for i in range(n):
        f = data_frame(tab, 'mppt', [('device', 'flat')],
                       [('Ui', F32, 40 + rng.random()), ('Uo', F32, 27 + rng.random()),
                        ('P', F32, 300 * rng.random()), ('pwm_duty', U16, rng.randrange(4096)),
                        ('lag', F16, rng.random() * 5)], ts0 + 1000 * i)
        raw = frames(tab.full_table(), f) if i % 4 == 0 else frames(f)
        recs.append((rng.choice((0, 1, 1)), raw))
    return recs


tab = SymbolTable()
recs = point_stream(8, tab)
clean = b''.join(record(c, r) for c, r in recs)
add_stream('clean', clean)

add_stream('garbage_between', b'\x00\x13garbage' + b''.join(record(c, r) + b'\xff\x7d\x00' for c, r in recs))

bogus = bytearray()
for c, r in recs:
    bogus += b'\x7e\x00' + b'\x7e\x81\x40' + b'\x7e\x05\x07' + b'\x7e\xff\xff\xff\xff\x0f' + record(c, r)
add_stream('bogus_magic', bogus)

tr = bytearray()
for i, (c, r) in enumerate(recs):
    rec = record(c, r)
    tr += rec[:len(rec) // 2] if i in (2, 5) else rec
add_stream('truncated_then_good', tr)

tab2 = SymbolTable()
orphan = data_frame(tab2, 'mppt', [('device', 'fry')], [('Ui', F32, 40.0)], 1763400000000)
sem = record(0, frames(orphan)) + record(1, frames(orphan)) + clean + record(0, frames(b'\x09zz'))
add_stream('semantic_error_keeps_framing', sem + record(1, recs[1][1]))

tab = SymbolTable()
tilde_recs = []
for i in range(6):
    f = data_frame(tab, 'mppt', [('device', '~~~')], [('w', U16, 0x7E7E), ('x', U32, 0x7E017E7E + i)],
                   0x7E7E7E7E7E + i)
    tilde_recs.append(record(i % 2, frames(tab.full_table(), f)))
assert all(r[1:].count(0x7E) >= 1 for r in tilde_recs), [r[1:].count(0x7E) for r in tilde_recs]
add_stream('tilde_in_payload', b''.join(tilde_recs))

# long noisy stream: seeded byte flips / drops / inserts
tab = SymbolTable()
big = bytearray(b''.join(record(c, r) for c, r in point_stream(60, tab)))
noisy = bytearray()
for b in big:
    x = rng.random()
    if x < 0.002:
        continue
    if x < 0.004:
        noisy.append(rng.getrandbits(8))
        continue
    noisy.append(b)
    if x > 0.998:
        noisy.append(0x7E)
add_stream('noisy_long', noisy)
add_stream('incomplete_tail', clean + record(1, recs[0][1])[:-4])

# --- write ------------------------------------------------------------------------------------

out = {
    'meta': {'generator': 'test/gen_tele_fixtures.py', 'tamp_version': tamp.__version__, 'seed': SEED,
             'tamp_args': {'window': 10, 'literal': 8, 'extended': False}},
    'dictionary': b64(tamp.initialize_dictionary(1024, literal=8)),
    'tamp': tamp_cases,
    'tamp_reject': tamp_reject,
    'tamp_truncated': tamp_truncated,
    'payload': payload_cases,
    'framing': framing_cases,
}
path = HERE / 'fixtures' / 'tele_fixtures.json'
path.parent.mkdir(exist_ok=True)
path.write_text(json.dumps(out, indent=1) + '\n')
print(f'teledec reference: {TELEDEC_DIR / "teledec.py"}')
print(f'tamp {len(tamp_cases)} + reject {len(tamp_reject)} + truncated {len(tamp_truncated)}, '
      f'payload {len(payload_cases)} seqs, framing {len(framing_cases)} streams')
for c in framing_cases:
    print(f"  {c['name']}: {len(c['lines'])} lines {c['stats']}")
print(f'wrote {path} ({path.stat().st_size} bytes)')
