// teledec.js payload decoder vs etc/fugu/teledec.py (fixtures/tele_fixtures.json).
const test = require('node:test');
const assert = require('node:assert/strict');
const T = require('../teledec.js');
const FX = require('./fixtures/tele_fixtures.json');

const b64 = (s) => new Uint8Array(Buffer.from(s, 'base64'));

test('readVarint: single, multi-byte and > 2^32 values', () => {
  assert.deepEqual(T.readVarint(Uint8Array.of(0x05), 0), {value: 5, pos: 1});
  assert.deepEqual(T.readVarint(Uint8Array.of(0xFF, 0xAC, 0x02), 1), {value: 300, pos: 3});
  // 4294967296123 = 0x3E8_0000_007B
  const enc = [];
  for (let v = 4294967296123; ; v = Math.floor(v / 128)) {
    if (v < 128) { enc.push(v); break; }
    enc.push((v % 128) | 0x80);
  }
  assert.equal(T.readVarint(Uint8Array.from(enc), 0).value, 4294967296123);
  assert.throws(() => T.readVarint(Uint8Array.of(0x80), 0), RangeError);
});

test('cstr reads up to NUL and rejects a missing terminator', () => {
  assert.deepEqual(T.cstr(new TextEncoder().encode('xUi\0P'), 1), {value: 'Ui', pos: 4});
  assert.throws(() => T.cstr(Uint8Array.of(0x41, 0x42), 0), RangeError);
});

test('f16ToF32 edge values', () => {
  assert.equal(T.f16ToF32(0x3C00), 1);
  assert.equal(T.f16ToF32(0x7BFF), 65504);
  assert.equal(T.f16ToF32(0xC100), -2.5);
  assert.equal(T.f16ToF32(0x2E66), 0.0999755859375);
  assert.equal(T.f16ToF32(0x7C00), Infinity);
  assert.equal(T.f16ToF32(0xFC00), -Infinity);
  assert.ok(Number.isNaN(T.f16ToF32(0x7E00)));
  assert.ok(Object.is(T.f16ToF32(0x8000), -0));
  assert.equal(T.f16ToF32(0x0001), 2 ** -24);
  assert.equal(T.f16ToF32(0x0400), 2 ** -14);
});

test('fmtG matches Python %.6g', () => {
  const cases = [
    [123456.5, '123456'], [123457.5, '123458'], [-123456.5, '-123456'], [1234565, '1.23456e+06'],
    [0.5, '0.5'], [2.5, '2.5'], [999999.5, '1e+06'], [100000, '100000'], [1e6, '1e+06'],
    [0.0001, '0.0001'], [0.00001, '1e-05'], [1.5e-7, '1.5e-07'], [1e16, '1e+16'], [1e100, '1e+100'],
    [0.1, '0.1'], [-1234.5678, '-1234.57'], [5e-324, '4.94066e-324'], [1.7976931348623157e308, '1.79769e+308'],
    [0, '0'], [-0, '-0'], [NaN, 'nan'], [Infinity, 'inf'], [-Infinity, '-inf'], [42, '42'], [0.000123456, '0.000123456'],
    [99999.95, '99999.9'], [9.9999995, '10'], [1/3, '0.333333'],
  ];
  for (const [x, s] of cases) assert.equal(T.fmtG(x), s, String(x));
});

for (const seq of FX.payload) {
  test(`decodePayload sequence: ${seq.name}`, () => {
    const table = new Map();
    for (const [k, p] of seq.sequence.entries()) {
      const out = [];
      let err = null;
      try { T.decodePayload(p.cid, b64(p.payload), table, out); } catch (e) { err = e; }
      assert.equal(err !== null, p.error !== null, `payload ${k}: error ${err} vs ${p.error}`);
      assert.deepEqual(out.map(T.pointToLine), p.lines, `payload ${k}`);
    }
  });
}

test('decodePayload returns structured points', () => {
  const seq = FX.payload.find((s) => s.name === 'all_dt_raw').sequence;
  const [p] = T.decodePayload(seq[0].cid, b64(seq[0].payload), new Map());
  assert.equal(p.meas, 'dt');
  assert.deepEqual(p.tags, {device: 'flat', board: 'fugu2'});
  assert.equal(p.ts, 4294967296123);
  assert.equal(p.fields.b1, true);
  assert.equal(p.fields.i8, -128);
  assert.equal(p.fields.u32, 4294967295);
  assert.equal(p.fields.u32b, 2147483649);
  assert.equal(p.fields.i32, -2147483648);
  assert.equal(p.fields.ftie, 123456.5);
  assert.equal(p.fields.hmax, 65504);
  assert.ok(Object.is(p.fields.hnz, -0));
  assert.equal(p.types.u16, T.DT.U16);
  assert.equal(p.types.ftie, T.DT.F32);
  assert.equal(p.types.h01, T.DT.F16);
});

test('teledec.js loads as a browser script into window.TeleDec', () => {
  const vm = require('node:vm');
  const src = require('node:fs').readFileSync(require.resolve('../teledec.js'), 'utf8');
  const window = {TextDecoder};
  window.self = window;
  vm.runInNewContext(require('node:fs').readFileSync(require.resolve('../pmini.js'), 'utf8'), window);
  vm.runInNewContext(src, window);
  assert.equal(typeof window.TeleDec.decodePayload, 'function');
  assert.equal(window.TeleDec.fmtG(123456.5), '123456');
});

test('decodePayload returns a fresh array and rejects unknown cid', () => {
  const seq = FX.payload.find((s) => s.name === 'multi_point').sequence;
  const pts = T.decodePayload(0, b64(seq[0].payload), new Map());
  assert.equal(pts.length, 4);
  assert.throws(() => T.decodePayload(2, Uint8Array.of(1, 1), new Map()), /compressor/);
});

const varint = (n) => { const o = []; do { let b = n % 128; n = Math.floor(n / 128); if (n) b |= 0x80; o.push(b); } while (n); return o; };
const framed = (f) => [...varint(f.length), ...f];

test('symbol table is reset once it would exceed MAX_SYMBOLS', () => {
  const table = new Map();
  const tab = [2];
  for (let sid = 1; sid <= T.MAX_SYMBOLS; sid++) tab.push(...varint(sid), 0x6B, 0, T.DT.F32);
  tab.push(0);
  T.decodePayload(0, Uint8Array.from(framed(tab)), table);
  assert.equal(table.size, T.MAX_SYMBOLS);
  T.decodePayload(0, Uint8Array.from(framed([2, ...varint(1), 0x6B, 0, T.DT.F32, 0])), table);
  assert.equal(table.size, T.MAX_SYMBOLS, 'redefining a known SID keeps the table');
  T.decodePayload(0, Uint8Array.from(framed([2, ...varint(T.MAX_SYMBOLS + 1), 0x6D, 0, 0, 0])), table);
  assert.equal(table.size, 1);
});

test('tampDecompress refuses output beyond MAX_TAMP_OUT', () => {
  // header 0x58 + zero bits: each 12-bit token is a 2-byte match at offset 0
  const big = new Uint8Array(50000); big[0] = 0x58;
  assert.throws(() => T.tampDecompress(big), /64 KB/);
  const ok = new Uint8Array(40000); ok[0] = 0x58;
  assert.ok(T.tampDecompress(ok).length < T.MAX_TAMP_OUT);
});
