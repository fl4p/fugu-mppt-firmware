// dashboard.html: MQTT broker discovery from mqtt.conf over BLE, WebSocket port probe, query-parameter prefill.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadDash, closeAll, settle, until, makeFakeBleDevice, makeFakeBluetooth, makeFakeWebSocket } = require('./_setup_dash');

test.after(closeAll);

const HOST = '192.168.1.200';
const plain = (x) => JSON.parse(JSON.stringify(x));
const conf = (k, v) => `\x1b[0;32mI (1234) main: Conf '/littlefs/conf/mqtt.conf:${k}' = '${v}'\x1b[0m`;
const mqttConf = (c) => c === 'get-config mqtt.conf'
  ? [conf('broker_uri', `mqtt://${HOST}:1882`), conf('username', 'pv'), conf('password', 's3cr:et'), conf('cmd_input', '1'),
    'I (1240) main: OK: get-config mqtt.conf']
  : null;
const banner = (window) => [...window.document.querySelectorAll('#banners .banner')].map(b => b.textContent).join('\n');
const val = (window, id) => window.document.getElementById(id).value;

async function enable({ outcome, storage, url, before } = {}) {
  const dev = makeFakeBleDevice({ hostname: 'flat', reply: mqttConf });
  const ws = makeFakeWebSocket(outcome);
  const env = await loadDash({ bluetooth: makeFakeBluetooth([dev]), ws, storage, url });
  if (before) before(env);
  await env.Dash.addBle();
  return { ...env, dev, ws };
}

test('discovery: get-config mqtt.conf after the enable sequence, probe in order, stop at the first open', async () => {
  const { window, Dash, dev, ws } = await enable({ storage: { 'dash-mqtt-remember': '0' }, outcome: (u) => u.endsWith(':8083') ? 'open' : 'error' });
  await until(() => /broker found/.test(banner(window)), 3000, 'found banner');
  assert.deepEqual(plain(dev.writes), ['hostname', 'tele-ble 1', 'sweep-curve', 'status', 'get-config mqtt.conf']);
  assert.deepEqual(plain(ws.tried), [`ws://${HOST}:1885`, `ws://${HOST}:9001`, `ws://${HOST}:8083`].map(u => [u, 'mqtt']));
  assert.deepEqual(plain(ws.closed), [`ws://${HOST}:8083`]);
  assert.equal(val(window, 'mqtt-url'), `ws://${HOST}:8083`);
  assert.equal(val(window, 'mqtt-user'), 'pv');
  assert.equal(val(window, 'mqtt-pass'), 's3cr:et');
  assert.equal(banner(window), `MQTT broker found at ws://${HOST}:8083 (from flat) — Connect? Connect`);
  // no auto-connect, nothing stored yet
  await settle(20);
  assert.equal(window._mqttClients, undefined);
  assert.equal(Dash.client, null);
  assert.equal(window.localStorage.getItem('mqtt-url'), null);
  assert.equal(window.localStorage.getItem('dash-mqtt-pass'), null);
  window.document.querySelector('#banners .banner button').click();
  assert.equal(window._mqttClients.length, 1);
  const c = window._mqttClients[0];
  assert.equal(c.url, `ws://${HOST}:8083`);
  assert.equal(c.opts.username, 'pv');
  assert.equal(c.opts.password, 's3cr:et');
  assert.equal(window.localStorage.getItem('mqtt-url'), `ws://${HOST}:8083`);
  assert.equal(window.localStorage.getItem('dash-mqtt-pass'), null, 'password not stored without remember');
  assert.equal(banner(window), '');
});

test('discovery: a hanging port times out after CFG.wsProbeMs; no listener -> banner, MQTT port tried last', async () => {
  let env = await enable({
    outcome: (u) => u.endsWith(':1885') ? 'hang' : u.endsWith(':9001') ? 'open' : 'error',
    before: ({ Dash }) => { Dash.CFG.wsProbeMs = 20; },
  });
  await until(() => /broker found/.test(banner(env.window)), 3000, 'found banner');
  assert.equal(val(env.window, 'mqtt-url'), `ws://${HOST}:9001`);

  env = await enable({ before: ({ Dash }) => { Dash.CFG.wsProbeMs = 20; } });
  await until(() => /no WebSocket/.test(banner(env.window)), 3000, 'no-listener banner');
  assert.deepEqual(plain(env.ws.tried.map(t => t[0])), [1885, 9001, 8083, 8080, 1882].map(p => `ws://${HOST}:${p}`));
  assert.equal(banner(env.window), `broker ${HOST} has no WebSocket listener on the usual ports`);
  assert.equal(val(env.window, 'mqtt-url'), '');
  assert.equal(val(env.window, 'mqtt-pass'), '');
  assert.equal(env.window._mqttClients, undefined);
});

test('discovery: skipped with a stored mqtt-url or a connected client', async () => {
  let env = await enable({ storage: { 'mqtt-url': 'ws://broker.test:1885' } });
  await settle(30);
  assert.ok(!env.dev.writes.includes('get-config mqtt.conf'));
  assert.equal(env.ws.tried.length, 0);

  env = await enable({ before: ({ Dash }) => Dash.mqttConnect('ws://other.test:9001', '', '') });
  await settle(30);
  assert.ok(!env.dev.writes.includes('get-config mqtt.conf'));
  assert.equal(env.ws.tried.length, 0);
  assert.doesNotMatch(banner(env.window), /broker/);
});

test('discovery: an https page probes wss:// ports; without one it hints at TLS', async () => {
  let env = await enable({ url: 'https://localhost/', outcome: (u) => u.endsWith(':8884') ? 'open' : 'error' });
  await until(() => /broker found/.test(banner(env.window)), 3000, 'found banner');
  assert.ok(env.dev.writes.includes('get-config mqtt.conf'));
  assert.deepEqual(plain(env.ws.tried.map(t => t[0])), [`wss://${HOST}:8884`]);
  assert.equal(val(env.window, 'mqtt-url'), `wss://${HOST}:8884`);
  assert.equal(val(env.window, 'mqtt-user'), 'pv');

  env = await enable({ url: 'https://localhost/', outcome: () => 'error' });
  await until(() => /wss/.test(banner(env.window)), 3000, 'https hint');
  assert.deepEqual(plain(env.ws.tried.map(t => t[0])), [8884, 8084, 443].map(p => `wss://${HOST}:${p}`));
  assert.match(banner(env.window), /broker at 192\.168\.1\.200: no wss:\/\/ listener on the usual ports, or its certificate is not trusted/);
  assert.equal(val(env.window, 'mqtt-url'), '');
  assert.equal(val(env.window, 'mqtt-pass'), '');
});

test('probeWs/wsCandidates/parseBroker', async () => {
  const ws = makeFakeWebSocket((u) => u.endsWith(':2') ? 'open' : 'error');
  const { Dash } = await loadDash({ ws });
  assert.deepEqual(plain(Dash.wsCandidates(Dash.parseBroker('mqtt://h.test:9001'))),
    [1885, 9001, 8083, 8080].map(p => `ws://h.test:${p}`));
  assert.deepEqual(plain(Dash.parseBroker('h.test')), { host: 'h.test', port: 1883 });
  assert.equal(Dash.parseBroker(''), null);
  assert.equal(await Dash.probeWs(['ws://a:1', 'ws://a:2', 'ws://a:3']), 'ws://a:2');
  assert.deepEqual(plain(ws.tried.map(t => t[0])), ['ws://a:1', 'ws://a:2']);
  assert.equal(await Dash.probeWs(['ws://a:1']), null);
});

test('query parameters prefill URL and user, never a password, and do not connect', async () => {
  const { window, Dash, ws } = await loadDash({
    url: 'http://localhost/dashboard.html?mqtt=ws://q.test:9001&user=alice&pass=x&password=y',
    storage: { 'mqtt-url': 'ws://stored.test:1885', 'mqtt-user': 'bob', 'dash-mqtt-remember': '0' },
  });
  assert.equal(val(window, 'mqtt-url'), 'ws://q.test:9001');
  assert.equal(val(window, 'mqtt-user'), 'alice');
  assert.equal(val(window, 'mqtt-pass'), '');
  assert.equal(window._mqttClients, undefined);
  assert.equal(Dash.client, null);
  assert.equal(ws.tried.length, 0);
  window.document.getElementById('mqtt-connect').click();
  assert.equal(window._mqttClients[0].url, 'ws://q.test:9001');
  assert.equal(window._mqttClients[0].opts.username, 'alice');
});

test('a remembered password is not prefilled for a ?mqtt= link to another host, but is for the same host', async () => {
  const other = await loadDash({
    url: 'http://localhost/dashboard.html?mqtt=ws://evil.test:9001',
    storage: { 'mqtt-url': 'ws://stored.test:1885', 'mqtt-user': 'bob', 'dash-mqtt-remember': '1', 'dash-mqtt-pass': 's3cret' },
  });
  assert.equal(val(other.window, 'mqtt-url'), 'ws://evil.test:9001');
  assert.equal(val(other.window, 'mqtt-pass'), '');
  const same = await loadDash({
    url: 'http://localhost/dashboard.html?mqtt=ws://stored.test:9001',
    storage: { 'mqtt-url': 'ws://stored.test:1885', 'mqtt-user': 'bob', 'dash-mqtt-remember': '1', 'dash-mqtt-pass': 's3cret' },
  });
  assert.equal(val(same.window, 'mqtt-pass'), 's3cret');
});

test('broker discovery never keeps a remembered password for a device-supplied host', async () => {
  const noPass = (c) => c === 'get-config mqtt.conf'
    ? [conf('broker_uri', 'mqtt://dev.test:1882'), conf('username', 'u'), 'I (1240) main: OK: get-config mqtt.conf'] : null;
  const dev = makeFakeBleDevice({ hostname: 'flat', reply: noPass });
  const ws = makeFakeWebSocket((u) => u.endsWith(':1885') ? 'open' : 'error');
  const env = await loadDash({ bluetooth: makeFakeBluetooth([dev]), ws, storage: { 'dash-mqtt-remember': '1', 'dash-mqtt-pass': 's3cret' } });
  env.window.document.getElementById('mqtt-pass').value = 's3cret';   // a remembered password sitting in the field
  await env.Dash.addBle();
  await until(() => val(env.window, 'mqtt-url') === 'ws://dev.test:1885');
  assert.equal(val(env.window, 'mqtt-user'), 'u');
  assert.equal(val(env.window, 'mqtt-pass'), '');
});
