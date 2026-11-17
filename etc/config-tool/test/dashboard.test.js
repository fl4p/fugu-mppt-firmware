// dashboard.html in jsdom: MQTT discovery/lease/console, BLE enable sequence, telemetry -> card.
const test = require('node:test');
const assert = require('node:assert/strict');
const {
  loadDash, closeAll, settle, makeFakeBleDevice, makeFakeBluetooth, mqttMsg, bleRecord, fakeScrollGeometry,
} = require('./_setup_dash');

test.after(closeAll);

const URL = 'ws://broker.test:1885';
const enc = (s) => Buffer.from(s);
const card = (window, host) => window.document.querySelector(`.card[data-host="${host}"]`);
const text = (window, host, sel) => card(window, host).querySelector(sel).textContent;
// page-realm arrays/objects fail deepStrictEqual on prototype; copy into this realm
const plain = (x) => JSON.parse(JSON.stringify(x));
const pubs = (client, topic) => [...client._publishes.filter(p => !topic || p.topic === topic).map(p => p.msg)];

async function withBroker(extra = {}) {
  const env = await loadDash({ storage: { 'mqtt-url': URL, ...extra } });
  const client = env.window._mqttClients[0];
  return { ...env, client };
}

// Mocked wall clock (Date.now inside the page)
function clock(window, t0 = 1_763_400_000_000) {
  let t = t0;
  window.Date.now = () => t;
  return { set: (x) => { t = x; }, add: (ms) => { t += ms; }, get: () => t };
}

function log(client, host, lines) {
  client.emit('message', `pv/log/${host}`, enc(lines.map(l => l + '\n').join('')));
}

test('no MQTT URL: no client is created, BLE-only enable sequence works without mqtt.js', async () => {
  const dev = makeFakeBleDevice({ hostname: 'fbuck', name: 'fugu-fbuck' });
  const bt = makeFakeBluetooth([dev]);
  const { window, Dash } = await loadDash({ bluetooth: bt, mqtt: false });
  assert.equal(window.mqtt, undefined);
  assert.equal(Dash.client, null);
  await Dash.addBle();
  await settle(5);
  assert.deepEqual(plain(bt.requests[0].filters), [{ namePrefix: 'fugu-' }, { services: [Dash.NUS.SVC] }]);
  assert.deepEqual(plain(bt.requests[0].optionalServices), [Dash.NUS.SVC]);
  assert.deepEqual(dev.writes.map(w => w.replace(/^set-time \d+$/, 'set-time N')),
    ['hostname', 'set-time N', 'tele-ble 1', 'sweep-curve']);
  assert.ok(dev.slices.every(n => n <= 20));
  // one telemetry record split over two notifications
  const rec = bleRecord([{ fields: { P: 123.46, Ui: 40.5, Uo: 27.25, I: 4.5, pwm_duty: 1234 }, ts: 1_763_400_000_123 }]);
  const cut = 9;
  dev.tele.notify(rec.slice(0, cut));
  assert.equal(Dash.devices.get('fbuck').lastTs, 0);
  dev.tele.notify(rec.slice(cut));
  Dash.flush();
  assert.equal(text(window, 'fbuck', '.pv'), '123.5');
  assert.equal(text(window, 'fbuck', '.vin'), '40.5 V');
  assert.equal(text(window, 'fbuck', '.duty'), '1234');
  assert.equal(card(window, 'fbuck').querySelector('.badge.ble').dataset.state, 'on');
  assert.equal(card(window, 'fbuck').querySelector('.badge.mqtt').dataset.state, 'off');
  assert.match(text(window, 'fbuck', '.rate'), /^BLE [\d.]+ pt\/s · dropped 0 B · resyncs 0$/);
});

test('BLE hostname falls back to the device name minus fugu-; missing TELE char is a reason', async () => {
  const dev = makeFakeBleDevice({ hostname: null, name: 'fugu-bench', tele: false });
  const { window, Dash } = await loadDash({ bluetooth: makeFakeBluetooth([dev]) });
  Dash.CFG.curveTimeoutMs = 10;
  const t0 = Date.now();
  await Dash.addBle();
  assert.ok(Date.now() - t0 >= 2400, 'waited for the hostname timeout');
  Dash.flush();
  assert.ok(card(window, 'bench'));
  const b = card(window, 'bench').querySelector('.badge.ble');
  assert.equal(b.dataset.state, 'refused');
  assert.match(b.title, /no telemetry characteristic/);
  assert.ok(!dev.writes.includes('tele-ble 1'));
});

test('tele-ble refusal reason is shown on the BLE badge', async () => {
  const dev = makeFakeBleDevice({
    hostname: 'flu',
    reply: (c) => c === 'tele-ble 1' ? ['W (9) main: tele-ble: clock not set', 'ERR: tele-ble 1'] : null,
  });
  const { window, Dash } = await loadDash({ bluetooth: makeFakeBluetooth([dev]) });
  await Dash.addBle();
  Dash.flush();
  const b = card(window, 'flu').querySelector('.badge.ble');
  assert.equal(b.dataset.state, 'refused');
  assert.equal(b.title, 'clock not set');
});

test('stored URL auto-connects, subscribes, pokes remembered hosts', async () => {
  const { window, client } = await withBroker({
    'mqtt-user': 'u', 'mqtt-pass': 'p',
    'mqtt-known-hosts': JSON.stringify({ [URL]: ['fry', 'flat'], 'ws://other:1': ['x'] }),
  });
  assert.equal(client.url, URL);
  assert.equal(client.opts.username, 'u');
  assert.equal(client.opts.password, 'p');
  assert.deepEqual(plain(client._subscribes), ['pv/log/#', 'pv/alert/#', 'pv/tele/#']);
  assert.deepEqual(plain(client._publishes.map(p => [p.topic, p.msg])),
    [['pv/log/fry/cmd', 'hostname'], ['pv/log/flat/cmd', 'hostname']]);
  assert.equal(window.document.getElementById('mqtt-url').value, URL);
  assert.equal(window.document.getElementById('mqtt-connect').textContent, 'Disconnect');
});

test('cards from pv/log and pv/alert; /cmd echoes ignored; hosts remembered', async () => {
  const { window, Dash, client } = await withBroker();
  client.emit('message', 'pv/log/fry/cmd', enc('status'));
  log(client, 'fry', ['I (1) main: hello', '\x1b[0;32mI (2) mppt: green\x1b[0m', 'OK: status']);
  client.emit('message', 'pv/alert/flat/bms', enc('cell over-voltage'));
  Dash.flush();
  assert.deepEqual(plain([...Dash.devices.keys()]), ['fry', 'flat']);
  const rows = [...card(window, 'fry').querySelectorAll('.log div')];
  assert.deepEqual(rows.map(r => r.textContent), ['I (1) main: hello', 'I (2) mppt: green', 'OK: status']);
  assert.ok(rows[1].querySelector('span[style*="color"]'));
  assert.equal(rows[2].className, 'ok');
  assert.equal(text(window, 'flat', '.log div'), '[alert bms] cell over-voltage');
  assert.deepEqual(JSON.parse(window.localStorage.getItem('mqtt-known-hosts'))[URL], ['flat', 'fry']);
});

test('pv/tele message renders KPIs; kWh above 1000 Wh; mode name; relay without a lease', async () => {
  const { window, Dash, client } = await withBroker();
  const c = clock(window);
  const st = { syms: new Map() };
  client.emit('message', 'pv/tele/fry', mqttMsg([
    { fields: { P: 312.44, Ui: 38.21, Uo: 27.1, I: 11.5, pwm_duty: 2047, mppt_state: 4, P_max_today: 401.5, E_today: 1534 },
      tags: { device: 'fry' }, ts: c.get() },
  ], st));
  client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P: 300.0 }, ts: c.get() + 20 }], st));
  Dash.flush();
  assert.equal(text(window, 'fry', '.pv'), '300.0');
  assert.equal(text(window, 'fry', '.peak'), 'peak today 401.5 W · 1.53 kWh');
  assert.equal(text(window, 'fry', '.mode'), 'MPPT');
  assert.equal(text(window, 'fry', '.vout'), '27.10 V');
  assert.match(text(window, 'fry', '.rate'), /^MQTT relay /);
  // telemetry within 5 s of discovery: never leased
  c.add(6000); Dash.tick(); await settle();
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), []);
});

test('lease: tele-mqtt 120 after 5 s without telemetry; on-reply; renew every 60 s; re-lease on reconnect', async () => {
  const { window, Dash, client } = await withBroker();
  const c = clock(window);
  log(client, 'flat', ['I (1) main: boot']);
  c.add(4000); Dash.tick();
  assert.deepEqual(pubs(client, 'pv/log/flat/cmd'), []);
  c.add(1500); Dash.tick();
  assert.deepEqual(pubs(client, 'pv/log/flat/cmd'), ['tele-mqtt 120']);
  log(client, 'flat', ['tele-mqtt: on, 120 s left, dropped 0 B', 'OK: tele-mqtt 120']);
  await settle();
  Dash.flush();
  const b = card(window, 'flat').querySelector('.badge.mqtt');
  assert.equal(b.dataset.state, 'on');
  assert.match(b.title, /120 s left/);
  c.add(30000); Dash.tick();
  assert.equal(pubs(client, 'pv/log/flat/cmd').length, 1);
  c.add(31000); Dash.tick();
  assert.equal(pubs(client, 'pv/log/flat/cmd').length, 2);
  log(client, 'flat', ['tele-mqtt: on, 120 s left, dropped 3 B', 'OK: tele-mqtt 120']);
  await settle();
  // renewal on an incoming record once 60 s have passed
  c.add(61000);
  client.emit('message', 'pv/tele/flat', mqttMsg([{ fields: { P: 1 }, ts: c.get() }]));
  assert.equal(pubs(client, 'pv/log/flat/cmd').length, 3);
  log(client, 'flat', ['OK: tele-mqtt 120']);
  await settle();
  Dash.flush();
  assert.match(text(window, 'flat', '.rate'), /^MQTT [\d.]+ pt\/s · dropped 3 B$/);
  // broker reconnect re-leases
  client.emit('connect');
  assert.equal(pubs(client, 'pv/log/flat/cmd').filter(m => m === 'tele-mqtt 120').length, 4);
  assert.ok(!pubs(client).includes('tele-mqtt 0'));
});

test('lease: clock refusal sends set-time once and retries', async () => {
  const { window, Dash, client } = await withBroker();
  const c = clock(window);
  log(client, 'fry', ['x']);
  c.add(5000); Dash.tick();
  log(client, 'fry', ['W (5) main: tele-mqtt: clock not set (send `set-time <epoch_ms>` or wait for SNTP)', 'ERR: tele-mqtt 120']);
  await settle();
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), ['tele-mqtt 120', `set-time ${c.get()}`, 'tele-mqtt 120']);
  log(client, 'fry', ['W (6) main: tele-mqtt: clock not set', 'ERR: tele-mqtt 120']);
  await settle();
  Dash.flush();
  const b = card(window, 'fry').querySelector('.badge.mqtt');
  assert.equal(b.dataset.state, 'refused');
  assert.match(b.title, /clock not set/);
  assert.equal(pubs(client, 'pv/log/fry/cmd').length, 3);
});

test('lease: firmware without tele-mqtt is treated as relay-fed; other reasons shown', async () => {
  const { window, Dash, client } = await withBroker();
  const c = clock(window);
  log(client, 'old', ['x']); log(client, 'busy', ['x']);
  c.add(5000); Dash.tick();
  log(client, 'old', ['E (5) main: Command not found: tele-mqtt', 'ERR: tele-mqtt 120']);
  log(client, 'busy', ['W (5) main: tele-mqtt: low heap', 'ERR: tele-mqtt 120']);
  await settle();
  Dash.flush();
  assert.equal(card(window, 'old').querySelector('.badge.mqtt').dataset.state, 'on');
  assert.match(card(window, 'old').querySelector('.badge.mqtt').title, /relay-fed/);
  assert.equal(card(window, 'busy').querySelector('.badge.mqtt').title, 'low heap');
  c.add(120000); Dash.tick();
  assert.equal(pubs(client, 'pv/log/old/cmd').length, 1, 'unsupported is not retried');
});

test('lease: no reply shows no-reply on the badge', async () => {
  const { window, Dash, client } = await withBroker();
  const c = clock(window);
  log(client, 'fry', ['x']);
  c.add(5000);
  window.setTimeout = ((st) => (fn, ms, ...a) => st(fn, ms === 5000 ? 0 : ms, ...a))(window.setTimeout);
  Dash.tick();
  await settle(5);
  Dash.flush();
  const b = card(window, 'fry').querySelector('.badge.mqtt');
  assert.equal(b.dataset.state, 'refused');
  assert.equal(b.title, 'no reply to tele-mqtt');
});

test('Stop sweep and leaving mppt_state 5 request sweep-curve (debounced, single in flight)', async () => {
  const { window, Dash, client } = await withBroker();
  Dash.CFG.curveDebounceMs = 5;
  log(client, 'fry', ['I (9) mppt: Stop sweep 2.10s mode=MPPT', 'I (9) mppt: Stop sweep again']);
  await settle(20);
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), ['sweep-curve']);
  log(client, 'fry', ['I (10) main: sc: t=1763400000 n=3 mpp=210.5,38.2 sweeping=0',
    '\x1b[0;32mI (10) main: sc: 30.0 100.0 35.0 180.0\x1b[0m', 'sc: 38.2 210.5', 'sc: end', 'OK: sweep-curve']);
  const dev = Dash.devices.get('fry');
  assert.equal(dev.curve.points.length, 3);
  assert.deepEqual(plain(dev.curve.mpp), { P: 210.5, V: 38.2 });
  assert.equal(dev.curveReq, null);
  // sweep witnessed over telemetry, then the state leaves 5
  const st = { syms: new Map() };
  let ts = 1_763_400_100_000;
  client.emit('message', 'pv/tele/fry', mqttMsg([
    { fields: { P: 5, Ui: 45, mppt_state: 5 }, ts: ts++ },
    { fields: { P: 50, Ui: 40 }, ts: ts++ },
    { fields: { P: 90, Ui: 36, mppt_state: 4 }, ts: ts++ },
  ], st));
  assert.deepEqual(plain(dev.sweepTrace), [{ V: 45, P: 5 }, { V: 40, P: 50 }]);
  await settle(20);
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), ['sweep-curve', 'sweep-curve']);
});

test('sweep-curve: 4 s timeout (scaled) with 2 retries; incomplete curve retries', async () => {
  const { Dash, client } = await withBroker();
  Dash.CFG.curveTimeoutMs = 5;
  log(client, 'fry', ['x']);
  const dev = Dash.devices.get('fry');
  assert.equal(Dash.requestCurve(dev), true);
  assert.equal(Dash.requestCurve(dev), false, 'single in flight');
  await settle(40);
  assert.equal(pubs(client, 'pv/log/fry/cmd').length, 3);
  assert.equal(dev.curveErr, 'no sweep-curve reply');
  Dash.CFG.curveTimeoutMs = 1000;
  Dash.requestCurve(dev);
  log(client, 'fry', ['sc: t=1 n=4 mpp=1,2 sweeping=0', 'sc: 1 2', 'sc: end']);
  assert.equal(pubs(client, 'pv/log/fry/cmd').length, 5, 'short curve re-requested');
  log(client, 'fry', ['sc: none']);
  assert.equal(dev.curveNone, true);
  assert.equal(dev.curveReq, null);
});

test('console: publish over MQTT, history with arrow keys, reply classes, sc lines hidden', async () => {
  const { window, Dash, client } = await withBroker();
  log(client, 'fry', ['x']);
  Dash.flush();
  const c = card(window, 'fry');
  const inp = c.querySelector('form.console input');
  const submit = (v) => { inp.value = v; c.querySelector('form.console').dispatchEvent(new window.Event('submit', { cancelable: true })); };
  submit('status'); submit('heap'); submit('heap');
  await settle();
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), ['status', 'heap', 'heap']);
  const key = (k) => inp.dispatchEvent(new window.KeyboardEvent('keydown', { key: k, cancelable: true }));
  key('ArrowUp'); assert.equal(inp.value, 'heap');
  key('ArrowUp'); assert.equal(inp.value, 'status');
  key('ArrowUp'); assert.equal(inp.value, 'status');
  key('ArrowDown'); assert.equal(inp.value, 'heap');
  key('ArrowDown'); assert.equal(inp.value, '');
  log(client, 'fry', ['OK: status', 'ERR: bogus', 'I (1) main: sc: none']);
  Dash.flush();
  const rows = [...c.querySelectorAll('.log div')];
  assert.deepEqual(plain(rows.slice(1, 4).map(r => [r.className, r.textContent])),
    [['sent', '» status'], ['sent', '» heap'], ['sent', '» heap']]);
  assert.deepEqual(rows.slice(-3).map(r => r.className), ['ok', 'err', 'sc']);
  assert.equal(c.classList.contains('show-sc'), false);
  const cb = c.querySelector('input.sc-toggle');
  cb.checked = true; cb.dispatchEvent(new window.Event('change'));
  assert.equal(c.classList.contains('show-sc'), true);
});

test('log keeps 100 rows and tail-follows only at the bottom', async () => {
  const { window, Dash, client } = await withBroker();
  log(client, 'fry', Array.from({ length: 150 }, (_, i) => 'line ' + i));
  Dash.flush();
  const box = card(window, 'fry').querySelector('.log');
  assert.equal(box.childElementCount, 100);
  assert.equal(box.firstElementChild.textContent, 'line 50');
  fakeScrollGeometry(box, { scrollHeight: 1000, clientHeight: 100, scrollTop: 200 });
  log(client, 'fry', ['more']);
  Dash.flush();
  assert.equal(box.scrollTop, 200);
  fakeScrollGeometry(box, { scrollHeight: 1000, clientHeight: 100, scrollTop: 900 });
  log(client, 'fry', ['more2']);
  Dash.flush();
  assert.equal(box.scrollTop, 1000);
});

test('BLE card merges into the MQTT card of the same hostname; console goes over BLE; points deduped', async () => {
  const dev = makeFakeBleDevice({ hostname: 'flat', name: 'fugu-flat' });
  const { window, Dash, client } = await loadDash({
    storage: { 'mqtt-url': URL }, bluetooth: makeFakeBluetooth([dev]) });
  const cl = window._mqttClients[0];
  log(cl, 'flat', ['I (1) main: from mqtt']);
  await Dash.addBle();
  await settle(5);
  Dash.flush();
  assert.deepEqual(plain([...Dash.devices.keys()]), ['flat']);
  assert.equal(window.document.querySelectorAll('.card').length, 1);
  const c = card(window, 'flat');
  assert.equal(c.querySelector('.badge.ble').dataset.state, 'on');
  // console prefers BLE
  const inp = c.querySelector('form.console input');
  inp.value = 'status';
  c.querySelector('form.console').dispatchEvent(new window.Event('submit', { cancelable: true }));
  await settle(5);
  assert.equal(dev.writes.at(-1), 'status');
  assert.ok(!pubs(cl, 'pv/log/flat/cmd').includes('status'));
  // same point over both transports is counted once
  const pt = [{ fields: { P: 77, Ui: 30 }, ts: 1_763_400_000_500 }];
  dev.tele.notify(bleRecord(pt));
  cl.emit('message', 'pv/tele/flat', mqttMsg(pt));
  assert.equal(Dash.devices.get('flat').points.length, 1);
  // MQTT log mirror is ignored while BLE carries the log
  log(cl, 'flat', ['I (2) main: dup']);
  Dash.flush();
  assert.ok(![...c.querySelectorAll('.log div')].some(r => r.textContent === 'I (2) main: dup'));
  // BLE telemetry streaming: no MQTT lease
  window.Date.now = () => Date.now() + 10000;
  Dash.tick();
  assert.ok(!pubs(cl, 'pv/log/flat/cmd').includes('tele-mqtt 120'));
  void client;
});

test('BLE probe: tele-ble every 60 s updates the dropped counter', async () => {
  const dev = makeFakeBleDevice({ hostname: 'fbuck' });
  const { window, Dash } = await loadDash({ bluetooth: makeFakeBluetooth([dev]) });
  await Dash.addBle();
  const c = clock(window, Date.now());
  c.add(61000);
  Dash.tick();
  await settle(5);
  assert.equal(dev.writes.at(-1), 'tele-ble');
  assert.equal(Dash.devices.get('fbuck').ble.dropped, 7);
});

test('BLE link loss: 3 backoff reconnects, then a Reconnect button that works', async () => {
  const dev = makeFakeBleDevice({ hostname: 'fbuck' });
  const { window, Dash } = await loadDash({ bluetooth: makeFakeBluetooth([dev]) });
  Dash.CFG.bleBackoffMs = [1, 1, 1];
  await Dash.addBle();
  dev.failConnect = 3;
  dev.dropLink();
  await settle(30);
  Dash.flush();
  const c = card(window, 'fbuck');
  assert.equal(c.querySelector('.badge.ble').dataset.state, 'off');
  const btn = c.querySelector('button.reconnect');
  assert.notEqual(btn.style.display, 'none');
  const n = dev.writes.length;
  btn.click();
  await settle(10);
  Dash.flush();
  assert.equal(c.querySelector('.badge.ble').dataset.state, 'on');
  assert.deepEqual(dev.writes.slice(n).map(w => w.replace(/\d+$/, 'N')), ['hostname', 'set-time N', 'tele-ble N', 'sweep-curve']);
  assert.equal(btn.style.display, 'none');
});

test('getDevices restores granted devices as placeholder cards', async () => {
  const bt = makeFakeBluetooth();
  bt.granted.push(makeFakeBleDevice({ name: 'fugu-fboost' }));
  const { window, Dash } = await loadDash({ bluetooth: bt });
  await settle(5);
  Dash.flush();
  const c = card(window, 'fboost');
  assert.ok(c);
  assert.equal(c.querySelector('.badge.ble').dataset.state, 'off');
  assert.notEqual(c.querySelector('button.reconnect').style.display, 'none');
});

test('stale threshold: clamp(3 x median interval, 5 s, 20 min), card greys out', async () => {
  const { window, Dash } = await loadDash();
  const c = clock(window);
  const dev = Dash.getDevice('x');
  assert.equal(Dash.staleAfter(dev), 1_200_000);
  for (const ms of [20, 20, 20]) dev.intervals.push(ms);
  assert.equal(Dash.staleAfter(dev), 5000);
  dev.intervals.a = [900_000, 900_000];
  assert.equal(Dash.staleAfter(dev), 1_200_000);
  dev.intervals.a = [100_000, 100_000, 100_000];
  assert.equal(Dash.staleAfter(dev), 300_000);
  Dash.ingestTele('x', Buffer.from([0, ...require('./_setup_dash').encodePayload([{ fields: { P: 1 }, ts: c.get() }])]), 'test');
  Dash.flush();
  c.add(299_000); Dash.tick();
  assert.equal(card(window, 'x').classList.contains('stale'), false);
  assert.equal(text(window, 'x', '.age'), '5 min ago');
  c.add(2_000); Dash.tick();
  assert.equal(card(window, 'x').classList.contains('stale'), true);
});

test('banners: missing navigator.bluetooth; https page with ws:// URL', async () => {
  const { window } = await loadDash({ bluetooth: null });
  const b = [...window.document.querySelectorAll('.banner')].map(e => e.textContent);
  assert.ok(b.some(t => /Web Bluetooth is not available/.test(t)));
  assert.equal(window.document.getElementById('ble-add').disabled, true);
  assert.equal(window.Dash.client, null);
  const h = await loadDash({ url: 'https://example.test/', storage: { 'mqtt-url': 'ws://b:1' } });
  assert.ok([...h.window.document.querySelectorAll('.banner')].some(e => /wss:\/\//.test(e.textContent)));
  const f = await loadDash({ url: 'file:///tmp/dashboard.html' });
  assert.ok([...f.window.document.querySelectorAll('.banner')].some(e => /file:\/\//.test(e.textContent)));
  assert.equal(f.window.Dash.client, null);
});

test('Connect button stores settings and toggles to Disconnect', async () => {
  const { window, Dash } = await loadDash();
  const d = window.document;
  d.getElementById('mqtt-url').value = 'ws://b:9';
  d.getElementById('mqtt-user').value = 'me';
  d.getElementById('mqtt-connect').click();
  await settle();
  assert.equal(window.localStorage.getItem('mqtt-url'), 'ws://b:9');
  assert.equal(window.localStorage.getItem('mqtt-user'), 'me');
  assert.equal(Dash.client.url, 'ws://b:9');
  d.getElementById('mqtt-connect').click();
  assert.equal(Dash.client, null);
  assert.equal(window._mqttClients[0].ended, true);
  assert.equal(d.getElementById('mqtt-connect').textContent, 'Connect');
});
