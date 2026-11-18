// teledec.js `sc:` (sweep-curve console dump) line parser and assembler.
const test = require('node:test');
const assert = require('node:assert/strict');
const {parseScLine, SweepCurveParser, stripAnsi} = require('../teledec.js');

const HDR = 'sc: t=1763400000 n=3 mpp=152.5,41.25 sweeping=0';

function feedAll(lines) {
  const p = new SweepCurveParser();
  return lines.map((l) => p.feed(l)).filter((r) => r !== undefined);
}

test('stripAnsi removes colour sequences', () => {
  assert.equal(stripAnsi('\x1b[0;32mI (12) cli: sc: end\x1b[0m'), 'I (12) cli: sc: end');
});

test('parseScLine: header, data, end, none, non-sc', () => {
  assert.deepEqual(parseScLine(HDR),
    {kind: 'header', t: 1763400000, n: 3, mpp: {P: 152.5, V: 41.25}, sweeping: false});
  assert.equal(parseScLine('sc: t=1 n=0 mpp=0,0 sweeping=1').sweeping, true);
  assert.deepEqual(parseScLine('sc: 40.5 100 41 120.25'),
    {kind: 'data', points: [{V: 40.5, P: 100}, {V: 41, P: 120.25}], dropped: 0, truncated: false});
  assert.deepEqual(parseScLine('sc: end'), {kind: 'end'});
  assert.deepEqual(parseScLine('sc: none\r'), {kind: 'none'});
  assert.equal(parseScLine('V=41.2 P=120'), null);
  assert.equal(parseScLine('desc: 1 2'), null);
  assert.equal(parseScLine('disc:end'), null);
});

test('parseScLine matches sc: after a log prefix and ANSI codes', () => {
  assert.deepEqual(parseScLine('\x1b[0;32m[flat] I (5123) cli: sc: end\x1b[0m'), {kind: 'end'});
  assert.equal(parseScLine('12:00:01 fugu-flat sc: 1 2 3 4').points.length, 2);
});

test('parseScLine: odd value count keeps whole pairs and flags truncation', () => {
  assert.deepEqual(parseScLine('sc: 40 100 41'), {kind: 'data', points: [{V: 40, P: 100}], dropped: 0, truncated: true});
  assert.deepEqual(parseScLine('sc: 40 100 41 12x'), {kind: 'data', points: [{V: 40, P: 100}], dropped: 0, truncated: true});
  assert.equal(parseScLine('sc: t=abc n=3 mpp=1,2').kind, 'invalid');
  assert.equal(parseScLine('sc: t=1 n=3 mpp=1').kind, 'invalid');
  assert.equal(parseScLine('sc: hello').kind, 'data');
  assert.equal(parseScLine('sc: hello').truncated, true);
  assert.equal(parseScLine('sc:').kind, 'invalid');
});

test('SweepCurveParser assembles a complete curve over several data lines', () => {
  const r = feedAll(['boot', HDR, 'sc: 30 10 35 80', 'noise', 'sc: 41.25 152.5', 'sc: end']);
  assert.deepEqual(r, [{complete: true, curve: {t: 1763400000, n: 3, mpp: {P: 152.5, V: 41.25}, sweeping: false,
    points: [{V: 30, P: 10}, {V: 35, P: 80}, {V: 41.25, P: 152.5}]}}]);
});

test('SweepCurveParser: 12 pairs per line, prefixed lines', () => {
  const pts = Array.from({length: 30}, (_, i) => ({V: 20 + i, P: i * 3.5}));
  const lines = [`I (1) cli: sc: t=5 n=${pts.length} mpp=101.5,49 sweeping=1`];
  for (let i = 0; i < pts.length; i += 12) {
    lines.push('I (2) cli: sc: ' + pts.slice(i, i + 12).map((p) => `${p.V} ${p.P}`).join(' '));
  }
  lines.push('I (3) cli: sc: end');
  const [r] = feedAll(lines);
  assert.equal(r.complete, true);
  assert.equal(r.curve.sweeping, true);
  assert.deepEqual(r.curve.points, pts);
});

test('SweepCurveParser: n mismatch at end is incomplete (short)', () => {
  const [r] = feedAll([HDR, 'sc: 30 10', 'sc: end']);
  assert.equal(r.complete, false);
  assert.equal(r.reason, 'short');
  assert.deepEqual(r.curve.points, [{V: 30, P: 10}]);
});

test('SweepCurveParser: a truncated line with matching count is incomplete', () => {
  const [r] = feedAll(['sc: t=1 n=2 mpp=1,2 sweeping=0', 'sc: 1 2 3 4 5', 'sc: end']);
  assert.deepEqual(r, {complete: false, reason: 'truncated',
    curve: {t: 1, n: 2, mpp: {P: 1, V: 2}, sweeping: false, points: [{V: 1, P: 2}, {V: 3, P: 4}]}});
});

test('SweepCurveParser: new header mid-curve emits the incomplete one and restarts', () => {
  const p = new SweepCurveParser();
  assert.equal(p.feed(HDR), undefined);
  assert.equal(p.feed('sc: 30 10'), undefined);
  const r1 = p.feed('sc: t=1763400100 n=1 mpp=50,40 sweeping=0');
  assert.equal(r1.complete, false);
  assert.equal(r1.reason, 'restart');
  assert.equal(r1.curve.t, 1763400000);
  assert.equal(p.feed('sc: 40 50'), undefined);
  const r2 = p.feed('sc: end');
  assert.equal(r2.complete, true);
  assert.deepEqual(r2.curve.points, [{V: 40, P: 50}]);
});

test('SweepCurveParser: none, and data/end without a header', () => {
  assert.deepEqual(feedAll(['sc: none']), [{complete: true, curve: null}]);
  assert.deepEqual(feedAll(['sc: 1 2', 'sc: end']), []);
  assert.deepEqual(feedAll([HDR, 'sc: none', 'sc: end']), [{complete: true, curve: null}]);
  assert.deepEqual(feedAll([HDR, 'sc: t=x', 'sc: 30 10 35 80 41.25 152.5', 'sc: end']).map((r) => r.complete), [true]);
});

test('parseScLine: inf/-inf/nan tokens are non-finite; their pairs are dropped but counted', () => {
  const h = parseScLine('sc: t=1 n=3 mpp=inf,-inf sweeping=0');
  assert.equal(h.kind, 'header');
  assert.equal(h.mpp.P, Infinity);
  assert.equal(h.mpp.V, -Infinity);
  assert.deepEqual(parseScLine('sc: 30 inf -inf 5 40 100'),
    {kind: 'data', points: [{V: 40, P: 100}], dropped: 2, truncated: false});
  assert.deepEqual(parseScLine('sc: nan NaN'), {kind: 'data', points: [], dropped: 1, truncated: false});
  const p = new SweepCurveParser();
  for (const l of ['sc: t=1 n=3 mpp=100,40 sweeping=0', 'sc: 30 inf 35 nan', 'sc: 40 100']) p.feed(l);
  const r = p.feed('sc: end');
  assert.equal(r.complete, true);
  assert.deepEqual(r.curve.points, [{V: 40, P: 100}]);
});
