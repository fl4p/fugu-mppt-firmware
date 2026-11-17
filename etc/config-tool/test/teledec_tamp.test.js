// teledec.js tamp decompressor vs Python tamp 2.3.0 (fixtures/tele_fixtures.json).
const test = require('node:test');
const assert = require('node:assert/strict');
const T = require('../teledec.js');
const FX = require('./fixtures/tele_fixtures.json');

const b64 = (s) => new Uint8Array(Buffer.from(s, 'base64'));

function pack(bits) {
  const padded = bits.padEnd(Math.ceil(bits.length / 8) * 8, '0');
  const bytes = [0x58];
  for (let i = 0; i < padded.length; i += 8) bytes.push(parseInt(padded.slice(i, i + 8), 2));
  return Uint8Array.from(bytes);
}

test('fixture was generated with the firmware tamp settings', () => {
  assert.equal(FX.meta.tamp_version, '2.3.0');
  assert.deepEqual(FX.meta.tamp_args, {window: 10, literal: 8, extended: false});
});

test('tampInitDictionary matches tamp.initialize_dictionary(1024)', () => {
  assert.deepEqual(T.tampInitDictionary(1024), b64(FX.dictionary));
  assert.deepEqual(T.tampInitDictionary(), b64(FX.dictionary));
});

for (const c of FX.tamp) {
  test(`tampDecompress: ${c.name}`, () => {
    const comp = b64(c.compressed);
    assert.equal(comp[0], 0x58);
    assert.deepEqual(T.tampDecompress(comp), b64(c.raw));
  });
}

for (const c of FX.tamp_truncated) {
  test(`tampDecompress stops at EOF mid-token: ${c.name}`, () => {
    assert.deepEqual(T.tampDecompress(b64(c.compressed)), b64(c.raw));
  });
}

for (const c of FX.tamp_reject) {
  test(`tampDecompress rejects ${c.name}`, () => {
    assert.throws(() => T.tampDecompress(b64(c.compressed)), /extended/);
  });
}

test('tampDecompress rejects other headers and empty input', () => {
  assert.throws(() => T.tampDecompress(new Uint8Array(0)), /header/);
  assert.throws(() => T.tampDecompress(Uint8Array.of(0x5C)), /dictionary/);
  assert.throws(() => T.tampDecompress(Uint8Array.of(0x59, 0)), /header/);
});

test('tampDecompress rejects a match reaching past the window', () => {
  // match flag 0, symbol 0 (size 2), offset 1023: 1023 + 2 > 1024
  assert.throws(() => T.tampDecompress(pack('00' + '1111111111')), /window/);
  // offset 1022 + size 2 is in bounds and reads the dictionary tail
  assert.deepEqual(T.tampDecompress(pack('00' + '1111111110')), T.tampInitDictionary().slice(1022, 1024));
});

test('overlapping match copies the window as it was before the write', () => {
  // literal 'a' (window pos 0), then match offset 0, symbol 3 (size 5). Source [0,5) overlaps
  // the destination [1,6): the output is 'a' + dict[1..5), not a forward-copy run of 'a'.
  const out = T.tampDecompress(pack('1' + '01100001' + '0' + '1011' + '0000000000'));
  assert.deepEqual([...out], [0x61, 0x61, ...T.tampInitDictionary().slice(1, 5)]);
});

test('FLUSH token byte-aligns the bit reader', () => {
  // literal 'x', FLUSH (0 + 10101011), pad to byte, literal 'y'
  const bits = '1' + '01111000' + '0' + '10101011';
  const aligned = bits.padEnd(Math.ceil(bits.length / 8) * 8, '0') + '1' + '01111001';
  assert.deepEqual([...T.tampDecompress(pack(aligned))], [0x78, 0x79]);
});
