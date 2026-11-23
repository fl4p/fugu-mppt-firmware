// Handoff from the dashboard (spec §1.4): the #mqtt=…/#ble=… fragment, its removal from the address,
// the MQTT read with and without a stored password, the BLE read via getDevices (found / not found /
// busy / no getDevices), confirmDiscard, and that a handoff only reads, never pushes.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

const settle = (ms = 20) => new Promise(r => setTimeout(r, ms));
async function until(pred, ms = 4000) {
  const end = Date.now() + ms;
  while (!pred()) { if (Date.now() > end) throw new Error('timeout waiting'); await settle(10); }
}

const { firmware, mqttHook, bleDevice, fakeBluetooth } = require('./_fakes');

const CMDS = /^(set-config|del-config|svc (on|off|log))\b/;
const pushed = sent => sent.filter(c => CMDS.test(c));
const $ = (w, id) => w.document.getElementById(id);

test('parseHandoff: MQTT and BLE fragments; a password in the fragment is never read', async () => {
  const { window } = await loadEditor();
  const p = h => JSON.parse(JSON.stringify(window.parseHandoff(h)));
  assert.deepEqual(p('#mqtt=ws%3A%2F%2Fbroker%3A9001&user=pv&device=devA'),
    { transport: 'mqtt', url: 'ws://broker:9001', user: 'pv', device: 'devA' });
  assert.deepEqual(p('#mqtt=wss://b:8884&device=devA&pass=secret&password=x'),
    { transport: 'mqtt', url: 'wss://b:8884', user: '', device: 'devA' });
  assert.deepEqual(p('#ble=AbC%2B%3D%3D&name=fugu-devB&device=devB'),
    { transport: 'ble', id: 'AbC+==', name: 'fugu-devB', device: 'devB' });
  assert.deepEqual(p('#ble=&name=fugu-devB'), { transport: 'ble', id: '', name: 'fugu-devB', device: '' });
  assert.deepEqual(p('#mqtt=http://broker&device=devA'), { bad: 'mqtt' });
  assert.deepEqual(p('#mqtt=ws://broker&device=dev/A'), { bad: 'mqtt' });
  assert.deepEqual(p('#mqtt=ws://broker'), { bad: 'mqtt' });
  assert.deepEqual(p('#ble='), { bad: 'ble' });
  assert.equal(window.parseHandoff(''), null);
  assert.equal(window.parseHandoff('#section-2'), null);
});

test('MQTT handoff with a stored password: fragment cleared at once, config read, nothing pushed', async () => {
  const devA = firmware('devA');
  const dom = await loadEditor({
    url: 'http://localhost/tools/conf-editor.html?x=1#mqtt=ws%3A%2F%2Fbroker%3A9001&user=pv&device=devA',
    storage: { 'mqtt-pass': 'secret', 'mqtt-pass-for': JSON.stringify({ url: 'ws://broker:9001', user: 'pv' }) },
    beforeParse: w => mqttHook(w, { devA }),
  });
  const { window } = dom;
  assert.equal(window.location.hash, '');
  assert.equal(window.location.href, 'http://localhost/tools/conf-editor.html?x=1');
  await until(() => window._state && window._state.fromDevice);
  const c = window._mqttClients[0];
  assert.equal(c.url, 'ws://broker:9001');
  assert.equal(c.opts.username, 'pv');
  assert.equal(c.opts.password, 'secret');
  assert.equal(window._state.deviceName, 'devA');
  assert.ok(devA.sent.includes('get-config charger.conf'));
  assert.deepEqual(pushed(devA.sent), []);
  assert.equal($(window, 'conn').textContent, 'Connected to devA over MQTT');
  assert.ok(!$(window, 'mqtt-modal').classList.contains('open'));
  assert.ok(!window.location.href.includes('secret'));
});

test('MQTT handoff without a stored password: the dialog opens prefilled, the typed password reads', async () => {
  const devA = firmware('devA');
  const { window } = await loadEditor({
    url: 'http://localhost/#mqtt=ws%3A%2F%2Fbroker%3A9001&user=pv&device=devA',
    beforeParse: w => mqttHook(w, { devA }),
  });
  assert.equal(window.location.hash, '');
  assert.ok($(window, 'mqtt-modal').classList.contains('open'));
  assert.equal($(window, 'mqtt-url').value, 'ws://broker:9001');
  assert.equal($(window, 'mqtt-user').value, 'pv');
  assert.equal($(window, 'mqtt-pass').value, '');
  assert.ok(!$(window, 'mqtt-go').hidden);
  assert.equal($(window, 'mqtt-go').textContent, 'Read devA');
  assert.match($(window, 'mqtt-status').textContent, /password for pv to read devA/);
  assert.equal((window._mqttClients || []).length, 0, 'nothing connects before the password is typed');
  $(window, 'mqtt-pass').value = 'typed';
  $(window, 'mqtt-go').click();
  await until(() => window._state && window._state.fromDevice);
  assert.equal(window._mqttClients[0].opts.password, 'typed');
  assert.equal(window._state.deviceName, 'devA');
  assert.deepEqual(pushed(devA.sent), []);
  // remembered for exactly this broker and user
  assert.equal(window.mqttStoredPass('ws://broker:9001', 'pv'), 'typed');
  assert.equal(window.mqttStoredPass('ws://other:9001', 'pv'), null);
  assert.equal(window.mqttStoredPass('ws://broker:9001', 'admin'), null);
  assert.ok(!$(window, 'mqtt-modal').classList.contains('open'));
});

test('MQTT handoff: a password stored for another broker or user is not sent', async () => {
  for (const pf of [{ url: 'ws://other:9001', user: 'pv' }, { url: 'ws://broker:9001', user: 'admin' }, null]) {
    const storage = { 'mqtt-url': 'ws://broker:9001', 'mqtt-user': 'pv', 'mqtt-pass': 'secret' };
    if (pf) storage['mqtt-pass-for'] = JSON.stringify(pf);
    const { window } = await loadEditor({ url: 'http://localhost/#mqtt=ws://broker:9001&user=pv&device=devA', storage });
    assert.ok($(window, 'mqtt-modal').classList.contains('open'), JSON.stringify(pf));
    assert.equal($(window, 'mqtt-pass').value, '');
    assert.equal((window._mqttClients || []).length, 0);
  }
});

test('MQTT handoff to an anonymous broker reads at once; a failed read reopens the dialog', async () => {
  const devA = firmware('devA');
  let w = (await loadEditor({ url: 'http://localhost/#mqtt=ws://broker:9001&device=devA', beforeParse: x => mqttHook(x, { devA }) })).window;
  await until(() => w._state && w._state.fromDevice);
  assert.equal(w._mqttClients[0].opts.password, undefined);
  // the broker refuses: nothing is committed, the dialog offers a retry
  const refuse = x => { x._mqttHook = c => { const emit = c.emit.bind(c);
    c.emit = (ev, ...a) => ev === 'connect' ? emit('error', new Error('Connection refused: Not authorized')) : emit(ev, ...a); }; };
  w = (await loadEditor({ url: 'http://localhost/#mqtt=ws://broker:9001&device=ghost', beforeParse: refuse })).window;
  await until(() => w.document.getElementById('mqtt-modal').classList.contains('open'));
  assert.equal(w._state, null);
  assert.equal($(w, 'mqtt-status').textContent, 'Could not read ghost: Connection refused: Not authorized');
  assert.equal($(w, 'mqtt-go').textContent, 'Read ghost');
});

test('BLE handoff: a granted device is found by id and read without the chooser', async () => {
  const fw = firmware('devB');
  let dev, bt;
  const { window } = await loadEditor({
    url: 'http://localhost/#ble=id-2&name=fugu-devB&device=devB',
    beforeParse: w => {
      dev = bleDevice(w, { id: 'id-2', fw, advert: true });
      bt = fakeBluetooth([bleDevice(w, { id: 'id-1', name: 'fugu-other', fw: firmware('other') }), dev]);
      Object.defineProperty(w.navigator, 'bluetooth', { configurable: true, value: bt });
    },
  });
  assert.equal(window.location.hash, '');
  await until(() => window._state && window._state.fromDevice);
  assert.equal(bt.requests, 0, 'no chooser');
  assert.equal(window._state.deviceName, 'devB');
  assert.equal(window._lastTransport, 'ble');
  assert.equal($(window, 'conn').textContent, 'Connected to devB over Bluetooth');
  assert.deepEqual(pushed(fw.sent), []);
});

test('BLE handoff: device not granted, or no getDevices: a message and the Connect choice, no chooser', async () => {
  for (const [opts, re] of [[{}, /devB is not among the Bluetooth devices this page may use/],
                            [{ getDevices: false }, /cannot reopen devB without the Bluetooth chooser/]]) {
    let bt;
    const { window } = await loadEditor({
      url: 'http://localhost/#ble=id-9&name=fugu-devB&device=devB',
      beforeParse: w => {
        bt = fakeBluetooth([bleDevice(w, { id: 'id-1', name: 'fugu-other', fw: firmware('other') })], opts);
        Object.defineProperty(w.navigator, 'bluetooth', { configurable: true, value: bt });
      },
    });
    await until(() => re.test($(window, 'drop-msg').textContent));
    assert.equal(window._state, null);
    assert.equal(bt.requests, 0);
    assert.ok($(window, 'start-connect-pane').classList.contains('open'));
    assert.equal($(window, 'start-connect').getAttribute('aria-expanded'), 'true');
  }
});

test('BLE handoff: a device still held elsewhere gets the one-connection message; the old read stays', async () => {
  const fw = firmware('devB');
  let dev;
  const { window } = await loadEditor({
    url: 'http://localhost/#ble=id-2&device=devB',
    beforeParse: w => {
      dev = bleDevice(w, { id: 'id-2', fw, busy: true, advert: true });
      Object.defineProperty(w.navigator, 'bluetooth', { configurable: true, value: fakeBluetooth([dev]) });
    },
  });
  await until(() => /one Bluetooth connection at a time/.test($(window, 'drop-msg').textContent), 8000);
  assert.match($(window, 'drop-msg').textContent, /fugu-devB did not accept the Bluetooth link \(Connection attempt failed\)/);
  assert.equal(window._state, null);
  assert.equal(window._lastTransport, null);
  assert.ok(dev.connects >= 1);
  assert.deepEqual(fw.sent, []);
  assert.ok($(window, 'start-connect-pane').classList.contains('open'));
});

test('a handoff link on a loaded, edited config asks first; declining keeps the edits and the link', async () => {
  const devA = firmware('devA'), devB = firmware('devB');
  const { window } = await loadEditor({ beforeParse: w => mqttHook(w, { devA, devB }) });
  window.document.getElementById('mqtt-url').value = 'ws://broker:9001';
  await window.mqttConnectAndImport('devA');
  window._state.active = 'conf/charger.conf'; window.renderApp();
  const i = window.document.getElementById('f-vout_max');
  i.value = '29.2'; i.dispatchEvent(new window.Event('input'));
  assert.equal(window.pendingCount(), 1);
  const asked = [];
  window.confirm = (m) => { asked.push(m); return false; };
  window.location.hash = '#mqtt=ws://broker:9001&device=devB';
  await until(() => asked.length === 1);
  await settle(30);
  assert.match(asked[0], /1 change not pushed or saved will be lost/);
  assert.equal(window.location.hash, '');
  assert.equal(window._state.deviceName, 'devA');
  assert.equal(window.pendingCount(), 1);
  assert.equal(window._mqttDeviceHost, 'devA');
  assert.deepEqual(devB.sent, []);
  assert.deepEqual(pushed(devA.sent), []);
  // accepting reads devB; still nothing pushed anywhere
  window.confirm = (m) => { asked.push(m); return true; };
  window.location.hash = '#mqtt=ws://broker:9001&device=devB';
  await until(() => window._state.deviceName === 'devB');
  assert.equal(asked.length, 2);
  assert.deepEqual(pushed(devA.sent), []);
  assert.deepEqual(pushed(devB.sent), []);
});

test('bleWaitAdvert: true on an advertisement, false when none comes, null without the API', async () => {
  const { window } = await loadEditor();
  const fw = firmware('devB');
  assert.equal(await window.bleWaitAdvert(bleDevice(window, { fw, advert: true }), 500), true);
  assert.equal(await window.bleWaitAdvert(bleDevice(window, { fw, advert: false }), 50), false);
  assert.equal(await window.bleWaitAdvert(bleDevice(window, { fw }), 50), null);
});
