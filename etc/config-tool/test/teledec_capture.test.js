// Real BLE capture: raw notify chunks from a bench board, expected values from etc/fugu/teledec.py.
const test = require('node:test');
const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const T = require('../teledec.js');
const FX = require('./fixtures/ble_capture.json');

test('real BLE capture decodes like the Python reference', () => {
  const errs = [];
  const ts = new T.TeleStream((e) => errs.push(e));
  const pts = [];
  for (const b of FX.notifications) {
    const buf = Buffer.from(b, 'base64');
    pts.push(...ts.feed(new Uint8Array(buf.buffer, buf.byteOffset, buf.length)));
  }
  const lines = pts.map(T.pointToLine);
  assert.equal(errs.length, 0);
  assert.equal(ts.stats.records, FX.expected.records);
  assert.equal(lines.length, FX.expected.points);
  assert.equal(lines[0], FX.expected.first);
  assert.equal(lines[lines.length - 1], FX.expected.last);
  assert.equal(crypto.createHash('sha1').update(lines.join('\n')).digest('hex'), FX.expected.sha1);
});
