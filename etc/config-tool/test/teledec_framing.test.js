// teledec.js TeleStream (BLE 0x7E record framing) vs etc/fugu/teledec.py TeleStream.
const test = require('node:test');
const assert = require('node:assert/strict');
const T = require('../teledec.js');
const FX = require('./fixtures/tele_fixtures.json');

const b64 = (s) => new Uint8Array(Buffer.from(s, 'base64'));

function feedChunks(stream, cuts) {
  const ts = new T.TeleStream();
  const pts = [];
  let prev = 0;
  for (const c of [...cuts, stream.length]) {
    pts.push(...ts.feed(stream.subarray(prev, c)));
    prev = c;
  }
  return {lines: pts.map(T.pointToLine), ts};
}

for (const c of FX.framing) {
  const stream = b64(c.stream);

  test(`TeleStream whole feed: ${c.name}`, () => {
    const {lines, ts} = feedChunks(stream, []);
    assert.deepEqual(lines, c.lines);
    const {records, resyncs, errors, bytes} = c.stats;
    assert.deepEqual(ts.stats, {records, resyncs, errors, bytes});
    assert.equal(ts.pending, c.stats.pending);
  });

  test(`TeleStream split at every boundary: ${c.name}`, () => {
    for (let k = 0; k <= stream.length; k++) {
      const {lines, ts} = feedChunks(stream, [k]);
      assert.deepEqual(lines, c.lines, `split at ${k}`);
      assert.equal(ts.stats.errors, c.stats.errors, `split at ${k}`);
      assert.equal(ts.pending, c.stats.pending, `split at ${k}`);
    }
  });

  test(`TeleStream byte by byte and 20-byte notifies: ${c.name}`, () => {
    const all = Array.from({length: stream.length}, (_, i) => i);
    assert.deepEqual(feedChunks(stream, all).lines, c.lines);
    assert.deepEqual(feedChunks(stream, all.filter((i) => i && i % 20 === 0)).lines, c.lines);
  });
}

test('TeleStream accepts ArrayBuffer and plain arrays; onError gets the error', () => {
  const c = FX.framing.find((f) => f.name === 'semantic_error_keeps_framing');
  const errs = [];
  const ts = new T.TeleStream((e) => errs.push(e));
  const s = b64(c.stream);
  const pts = [...ts.feed(s.buffer.slice(s.byteOffset, s.byteOffset + 10)), ...ts.feed(Array.from(s.subarray(10)))];
  assert.deepEqual(pts.map(T.pointToLine), c.lines);
  assert.equal(errs.length, c.stats.errors);
  assert.ok(errs.every((e) => e instanceof Error));
});

test('TeleStream rejects bogus headers without waiting for data', () => {
  const ts = new T.TeleStream();
  // length 0, length > 4096, cid 2: each resyncs to the next 0x7E, then the buffer empties
  assert.deepEqual(ts.feed(Uint8Array.of(0x7E, 0x00, 0x7E, 0x81, 0x40, 0x7E, 0x03, 0x02, 0x00)), []);
  assert.equal(ts.pending, 0);
  assert.equal(ts.stats.resyncs, 3);
  // an incomplete varint length waits
  assert.deepEqual(ts.feed(Uint8Array.of(0x7E, 0x80)), []);
  assert.equal(ts.pending, 2);
});
