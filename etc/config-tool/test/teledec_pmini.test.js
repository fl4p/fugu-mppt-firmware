// Codec-2 PMA2 golden packet. The MTP2 shard was emitted by native pmini;
// expected bytes are the original length-prefixed sym_line_protocol frames.
const test = require('node:test');
const assert = require('node:assert/strict');
const T = require('../teledec.js');

const packet = Buffer.from('UE1BMgEEBQEBAgMAAQQHAB4dAgFtcHB0AAACZGV2aWNlAAADZmxhdAAABHAABwABAQFNVFAyAwQ6yLYAVRFMQyN1IIueiIjGXdZWD/UVaxQAAwAEAC/SP1WbJqJi', 'base64');
const raw = Buffer.from('1d02016d707074000002646576696365000003666c6174000004700007000c0101020300047856341200640c010102030004efbeadde0065100101020300043412000000e680808010', 'hex');

function crc32(b, end = b.length) {
  let c = 0xFFFFFFFF;
  for (let i = 0; i < end; i++) { c ^= b[i]; for (let j = 0; j < 8; j++) c = (c & 1) ? ((c >>> 1) ^ 0xEDB88320) : (c >>> 1); }
  return (c ^ 0xFFFFFFFF) >>> 0;
}
function validVariant(offset, value) {
  const b = Buffer.from(packet); b[offset] = value;
  b.writeUInt32LE(crc32(b, b.length - 4), b.length - 4);
  return new Uint8Array(b);
}

test('PMA2 native golden packet reconstructs exact original frames', () => {
  assert.deepEqual(Buffer.from(T.pma2Decode(packet)), raw);
  const table = new Map(), points = T.decodePayload(2, packet, table);
  assert.deepEqual(points.map(T.pointToLine), [
    'mppt,device=flat p=305419896i 100',
    'mppt,device=flat p=3735928559i 101',
    'mppt,device=flat p=4660i 4294967398',
  ]);
  assert.deepEqual(table.get(1), ['mppt', 0]);
});

test('TeleStream accepts codec-2 records', () => {
  const record = Uint8Array.from([0x7E, (packet.length + 1), 2, ...packet]);
  const stream = new T.TeleStream();
  assert.equal(stream.feed(record).length, 3);
  assert.equal(stream.pending, 0);
  assert.equal(stream.stats.records, 1);
});

test('PMA2 rejects CRC-valid malformed metadata and preserves table state', () => {
  const table = new Map([[99, ['old', T.DT.U32]]]);
  // First event descriptor is at byte 15; kind 2 is not defined.
  assert.throws(() => T.decodePayload(2, validVariant(15, 2), table), /event descriptor/);
  assert.deepEqual(table, new Map([[99, ['old', T.DT.U32]]]));
});

test('PMA2 truncation never decodes a partial packet', () => {
  for (let n = 0; n < packet.length - 1; n++) assert.throws(() => T.pma2Decode(packet.subarray(0, n)));
});

test('PMA2 rejects CRC-valid oversized rows before block decode', () => {
  const at = packet.indexOf(Buffer.from('MTP2'));
  const shard = packet.subarray(at, packet.length - 4);
  const body = Buffer.concat([shard.subarray(0, 4), Buffer.from([0x81, 0x01]), shard.subarray(5, shard.length - 4)]);
  const repairedShard = Buffer.concat([body, Buffer.alloc(4)]);
  repairedShard.writeUInt32LE(crc32(body), body.length);
  const envelope = Buffer.concat([packet.subarray(0, at), repairedShard, Buffer.alloc(4)]);
  envelope.writeUInt32LE(crc32(envelope, envelope.length - 4), envelope.length - 4);
  assert.throws(() => T.pma2Decode(envelope), /row\/event/);
});
