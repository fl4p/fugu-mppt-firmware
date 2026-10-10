// Spec §5 — line-oriented helpers (sendCommand, getConfig, hostname) and §5.3 — MQTT.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

// A multi-byte UTF-8 glyph (℃ = e2 84 83) can straddle two notifications/messages. A per-packet
// TextDecoder would flush the split halves as replacement chars (the half ending in e2 → one �,
// the next packet's 84 83 → two more) — the "garbage after svc" bug. The persistent {stream}
// decoder must stitch them.
function splitAtSecondGlyph(bytes) {
  const a = bytes.indexOf(0xe2);
  const b = bytes.indexOf(0xe2, a + 1);
  const cut = b + 1;                       // between the 2nd glyph's lead byte and its continuations
  return [bytes.slice(0, cut), bytes.slice(cut)];
}

test('onBleNotify reassembles a UTF-8 glyph split across two notifications', async () => {
  const { window } = await loadEditor();
  const seen = [];
  const fn = ln => seen.push(ln);
  window._lineListeners.add(fn);

  const [p1, p2] = splitAtSecondGlyph(new TextEncoder().encode('63℃65℃\n'));
  window.onBleNotify({ target: { value: p1 } });
  window.onBleNotify({ target: { value: p2 } });

  window._lineListeners.delete(fn);
  assert.deepEqual(seen, ['63℃65℃']);
});

test('mqttFeedPayload reassembles a UTF-8 glyph split across two messages', async () => {
  const { window } = await loadEditor();
  const client = window.mqtt.connect('ws://broker/', {});
  await new Promise(r => setTimeout(r, 0));
  await window.mqttBindDevice(client, 'fry');

  const seen = [];
  const fn = ln => seen.push(ln);
  window._lineListeners.add(fn);

  const [p1, p2] = splitAtSecondGlyph(new TextEncoder().encode('45℃57℃\n'));
  client.emit('message', 'pv/log/fry', p1);
  client.emit('message', 'pv/log/fry', p2);

  window._lineListeners.delete(fn);
  assert.deepEqual(seen, ['45℃57℃']);
});

test('sendCommand resolves true on OK echo and false on ERR echo', async () => {
  const { window } = await loadEditor();
  window._activeWrite = async () => {};

  const okP  = window.sendCommand('foo');
  window._dispatchLine('I (123) main: OK: foo');
  assert.equal(await okP, true);

  const errP = window.sendCommand('bar');
  window._dispatchLine('I (124) main: ERR: bar');
  assert.equal(await errP, false);
});

test('sendCommand streams every line to onRx until the marker', async () => {
  const { window } = await loadEditor();
  window._activeWrite = async () => {};
  const lines = [];

  const p = window.sendCommand('cmd', (ln) => lines.push(ln));
  window._dispatchLine('line one');
  window._dispatchLine('line two');
  window._dispatchLine('main: OK: cmd');
  await p;

  assert.deepEqual(lines, ['line one', 'line two', 'main: OK: cmd']);
});

test('getConfig parses "Conf \'<path>:<key>\' = \'<value>\'" lines until the OK marker', async () => {
  const { window } = await loadEditor();
  window._activeWrite = async () => {};
  const p = window.getConfig('board.conf');
  window._dispatchLine("I (267894) main: received serial command: 'get-config board.conf'");
  window._dispatchLine("Conf '/littlefs/conf/board.conf:mcu' = 'esp32s3'");
  window._dispatchLine("Conf '/littlefs/conf/board.conf:pwm_freq' = '39000'");
  window._dispatchLine("Conf '/littlefs/conf/other.conf:ignored' = 'x'");   // wrong file → skipped
  window._dispatchLine("I (267988) main: OK: get-config board.conf");

  const got = await p;
  // deepStrictEqual would trip on cross-realm Object.prototype; compare entries.
  assert.deepEqual(Object.entries(got).sort(), [['mcu','esp32s3'], ['pwm_freq','39000']].sort());
});

test('mqttBindDevice filters topics: only `pv/log/<host>`, never `/cmd` echoes', async () => {
  const { window } = await loadEditor();
  const client = window.mqtt.connect('ws://broker/', {});
  await new Promise(r => setTimeout(r, 0));   // wait for CONNACK
  await window.mqttBindDevice(client, 'fugu139C');

  // bound: subscribe was issued, activeWrite is mqttWrite
  assert.ok(client._subscribes.includes('pv/log/#'));
  assert.equal(window._lastTransport, 'mqtt');

  // capture forwarded lines via our own lineListener
  const seen = [];
  const fn = ln => seen.push(ln);
  window._lineListeners.add(fn);

  const enc = new TextEncoder();
  client.emit('message', 'pv/log/fugu139C',         enc.encode('hello\n'));   // 3-part → kept
  client.emit('message', 'pv/log/fugu139C/cmd',     enc.encode('echo\n'));    // 4-part → ignored
  client.emit('message', 'pv/log/other-device',     enc.encode('nope\n'));    // different host → ignored

  window._lineListeners.delete(fn);
  assert.deepEqual(seen, ['hello']);
});

test('mqttWrite publishes one trimmed line to pv/log/<host>/cmd', async () => {
  const { window } = await loadEditor();
  const client = window.mqtt.connect('ws://broker/', {});
  await new Promise(r => setTimeout(r, 0));
  await window.mqttBindDevice(client, 'host-x');

  await window.mqttWrite('  get-config board.conf  \r\n');
  assert.equal(client._publishes.length, 1);
  assert.equal(client._publishes[0].topic, 'pv/log/host-x/cmd');
  assert.equal(client._publishes[0].msg,   'get-config board.conf');
});

// §5 — importFromDevice hardening: retry on timeout (lost reply over MQTT QoS 0),
// but trust an OK-with-zero-keys as authoritatively absent (no wasted retries).
test('importFromDevice retries a lost reply but treats an empty OK as absent', async () => {
  const { window } = await loadEditor();
  window.fetchHostname = async () => 'fry';

  const attempts = {};
  window.getConfig = async (file) => {
    attempts[file] = (attempts[file] || 0) + 1;
    if (file === 'board.conf')  return { mcu: 'esp32s3' };          // present
    if (file === 'limits.conf') {                                    // dropped once, recovers
      if (attempts[file] < 2) throw new Error('timeout');
      return { hv_max: '85' };
    }
    if (file === 'coil.conf')   throw new Error('timeout');          // permanently lost
    return {};                                                       // genuinely absent (fast OK)
  };

  await window.importFromDevice('fry');

  const f = p => window._state.files['conf/' + p];
  // a dropped reply is recovered by the retry → real file, not a synthetic "new" tab
  assert.ok(f('limits.conf') && !f('limits.conf').isNew, 'limits.conf should recover via retry');
  // a genuinely-absent file replies fast with zero keys → exactly one attempt, no retries
  assert.equal(attempts['sensor.conf'], 1);
  // a permanently-lost file stays "not on device" and is surfaced in the status line
  assert.ok(f('coil.conf').isNew, 'coil.conf should remain synthetic after retries');
  assert.match(window.document.getElementById('serial-status').textContent,
               /timed out, skipped:.*coil\.conf/);
  // a successful read marks state for the overlay path (§1.1)
  assert.equal(window._state.fromDevice, true);
});

test('MQTT modal: Scan connects to the broker and lists devices seen on pv/log/<host>', async () => {
  const { window } = await loadEditor();
  // open the modal and fill the URL
  window.mqttModalOpen();
  window.document.getElementById('mqtt-url').value = 'ws://broker:9001';
  window.document.getElementById('mqtt-scan').click();

  // give the scan client a tick to connect
  await new Promise(r => setTimeout(r, 0));
  const clients = window._mqttClients || [];
  assert.equal(clients.length, 1, 'scan should open exactly one broker connection');
  const c = clients[0];
  assert.equal(c.url, 'ws://broker:9001');
  assert.ok(c._subscribes.includes('pv/log/#'), 'scan should subscribe to pv/log/#');

  // Fake two devices publishing and one /cmd echo we should ignore
  c.emit('message', 'pv/log/fry',         new TextEncoder().encode('hi\n'));
  c.emit('message', 'pv/log/flat',        new TextEncoder().encode('hi\n'));
  c.emit('message', 'pv/log/fry/cmd',     new TextEncoder().encode('echo\n'));

  // poll-tick is 0.8 s in the page; advance enough to trigger a render
  await new Promise(r => setTimeout(r, 900));
  const names = [...window.document.querySelectorAll('#mqtt-devlist .dev')]
    .map(d => d.textContent);
  assert.deepEqual(names.sort(), ['flat', 'fry']);
});
