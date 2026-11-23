// dashboard.html: the Configure action (opens conf-editor.html on a device, MQTT or BLE, releasing the
// dashboard's own BLE link first) and the editor's "Open dashboard" fragment (#mqtt= / #ble=).
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadDash, closeAll, settle, until, makeFakeBleDevice, makeFakeBluetooth } = require('./_setup_dash');

test.after(closeAll);

const URL_ = 'ws://broker.test:1885';
const card = (window, host) => window.document.querySelector(`.card[data-host="${host}"]`);
const cfgBtn = (window, host) => card(window, host).querySelector('button.cfg');
function log(client, host, lines) { client.emit('message', `pv/log/${host}`, Buffer.from(lines.map(l => l + '\n').join(''))); }
function recordOpen(window) { const opened = []; window.open = (...a) => { opened.push(a); return null; }; return opened; }
const frag = (u) => Object.fromEntries(new URLSearchParams(new URL(u, 'http://localhost/tools/dashboard.html').hash.slice(1)));

test('Configure over MQTT: broker, user and device in the fragment, never the password; BLE link kept', async () => {
  const dev = makeFakeBleDevice({ hostname: 'flat', name: 'fugu-flat', id: 'id-7' });
  const bt = makeFakeBluetooth([dev]);
  const { window, Dash } = await loadDash({ bluetooth: bt,
    storage: { 'mqtt-url': URL_, 'mqtt-user': 'pv', 'dash-mqtt-pass': 'secret', 'mqtt-pass': 'secret2' } });
  const client = window._mqttClients[0];
  assert.equal(client.opts.password, 'secret');
  await Dash.addBle();
  await until(() => Dash.devices.get('flat') && Dash.devices.get('flat').ble.state === 'on', 3000, 'ble');
  log(client, 'flat', ['I (1) main: hello']);
  Dash.flush();
  const b = cfgBtn(window, 'flat');
  assert.equal(b.hidden, false);
  assert.equal(b.textContent, 'Configure');
  assert.match(b.title, /over MQTT$/);
  const opened = recordOpen(window);
  b.click();
  assert.equal(opened.length, 1);
  const [url, target, feat] = opened[0];
  assert.match(url, /^conf-editor\.html#/);
  assert.equal(target, '_blank');
  assert.equal(feat, 'noopener');
  assert.deepEqual(frag(url), { mqtt: URL_, user: 'pv', device: 'flat' });
  assert.ok(!url.includes('secret'));
  assert.equal(Dash.devices.get('flat').ble.state, 'on', 'MQTT has no connection limit: BLE stays');
});

test('Configure over BLE: the dashboard releases its link first and says so; Reconnect clears the note', async () => {
  const dev = makeFakeBleDevice({ hostname: 'fbuck', name: 'fugu-fbuck', id: 'id+/=9' });
  const bt = makeFakeBluetooth([dev]);
  const { window, Dash } = await loadDash({ bluetooth: bt, mqtt: false });
  await Dash.addBle();
  await until(() => Dash.devices.get('fbuck') && Dash.devices.get('fbuck').ble.state === 'on', 3000, 'ble');
  Dash.flush();
  const b = cfgBtn(window, 'fbuck');
  assert.equal(b.hidden, false);
  assert.match(b.title, /one Bluetooth connection at a time, so the dashboard releases its link/);
  const opened = recordOpen(window);
  b.click();
  assert.equal(dev.gatt.connected, false, 'released before the editor opens');
  assert.deepEqual(frag(opened[0][0]), { ble: 'id+/=9', name: 'fugu-fbuck', device: 'fbuck' });
  await settle(5);
  Dash.flush();
  const link = Dash.devices.get('fbuck').ble;
  assert.equal(link.state, 'off');
  assert.equal(link.userClosed, true, 'no automatic reconnect');
  const c = card(window, 'fbuck');
  assert.equal(c.querySelector('.handoff').hidden, false);
  assert.match(c.querySelector('.handoff').textContent, /released for the config editor/);
  assert.equal(c.querySelector('.badge.ble').title, 'released for the config editor');
  assert.notEqual(c.querySelector('button.reconnect').style.display, 'none');
  await settle(50);
  assert.equal(link.state, 'off', 'still released after the disconnect event');
  c.querySelector('button.reconnect').click();
  await until(() => link.state === 'on', 3000, 'reconnect');
  Dash.flush();
  assert.equal(c.querySelector('.handoff').hidden, true);
});

test('Configure is hidden for a card with no MQTT broker and no BLE device', async () => {
  const snap = { flat: { host: 'flat', seen: 1, via: { mqtt: true, ble: false }, tele: { P: 10 } } };
  const { window, Dash } = await loadDash({ storage: { 'dash-devices': JSON.stringify(snap) }, mqtt: false });
  Dash.flush();
  assert.equal(cfgBtn(window, 'flat').hidden, true);
  const opened = recordOpen(window);
  assert.equal(Dash.configure(Dash.devices.get('flat')), null);
  assert.equal(opened.length, 0);
});

test('Configure on a restored card with a saved BLE id hands over that id', async () => {
  const snap = { fboost: { host: 'fboost', seen: 1, via: { mqtt: false, ble: true }, tele: {}, bleId: 'id-3', bleName: 'fugu-fboost' } };
  const { window, Dash } = await loadDash({ storage: { 'dash-devices': JSON.stringify(snap) }, mqtt: false,
    bluetooth: makeFakeBluetooth() });
  Dash.flush();
  assert.equal(cfgBtn(window, 'fboost').hidden, false);
  const opened = recordOpen(window);
  cfgBtn(window, 'fboost').click();
  assert.deepEqual(frag(opened[0][0]), { ble: 'id-3', name: 'fugu-fboost', device: 'fboost' });
});

test('#mqtt= from the editor: prefilled, connects with the remembered password for that host and user, fragment cleared', async () => {
  const { window } = await loadDash({
    url: 'http://localhost/tools/dashboard.html#mqtt=' + encodeURIComponent(URL_) + '&user=pv&device=flat',
    storage: { 'mqtt-url': URL_, 'mqtt-user': 'pv', 'dash-mqtt-pass': 'secret' } });
  assert.equal(window.location.href, 'http://localhost/tools/dashboard.html');
  assert.equal(window._mqttClients.length, 1);
  assert.equal(window._mqttClients[0].url, URL_);
  assert.equal(window._mqttClients[0].opts.password, 'secret');
});

test('#mqtt= for another host or user: no remembered password, a hint instead of a connect', async () => {
  for (const [h, user] of [['ws://other.test:1885', 'pv'], [URL_, 'admin']]) {
    const { window, Dash } = await loadDash({
      url: 'http://localhost/dashboard.html#mqtt=' + encodeURIComponent(h) + '&user=' + user + '&device=flat',
      storage: { 'mqtt-url': URL_, 'mqtt-user': 'pv', 'dash-mqtt-pass': 'secret' } });
    assert.equal(window.location.hash, '');
    assert.equal(window.document.getElementById('mqtt-url').value, h);
    assert.equal(window.document.getElementById('mqtt-user').value, user);
    assert.equal(window.document.getElementById('mqtt-pass').value, '');
    assert.equal(Dash.client, null);
    assert.match(window.document.getElementById('banners').textContent, /password/);
  }
});

test('#ble= from the editor: the released device gets the full retry ladder; an unknown one a banner', async () => {
  const bt = makeFakeBluetooth();
  const d = makeFakeBleDevice({ name: 'fugu-flat', hostname: 'flat', id: 'id-7', failConnect: 2 });
  bt.granted.push(d);
  const { window, Dash } = await loadDash({ bluetooth: bt, mqtt: false, url: 'http://localhost/dashboard.html#ble=id-7&name=fugu-flat&device=flat' });
  assert.equal(window.location.hash, '');
  await until(() => Dash.devices.get('flat') && Dash.devices.get('flat').ble.state === 'on', 5000, 'handoff connect');
  const bt2 = makeFakeBluetooth();
  const { window: w2 } = await loadDash({ bluetooth: bt2, mqtt: false, url: 'http://localhost/dashboard.html#ble=id-9&device=fboost' });
  await settle(5);
  assert.match(w2.document.getElementById('banners').textContent, /fboost: this page may not use its Bluetooth link yet/);
});
