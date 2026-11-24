// Spec §2.14 (services sheet): `svc list` parsing, the commands the controls send, gating on
// the push target's open link, and the confirm before cutting the editor's own link.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

const COLOR = { Running: '\x1b[32m', Failed: '\x1b[31m', Stopped: '\x1b[90m' };
const pad = (s, n) => String(s).padEnd(n);
// Same layout as cmdService() in src/cli.cpp: "%-10s %s %-6s %-8s %s", state wrapped in SGR.
const svcLine = s => `${pad(s.name, 10)} ${COLOR[s.state]}${pad(s.state, 8)}\x1b[0m ${pad(s.log, 6)} ${pad(s.enabled ? 'yes' : 'no', 8)} ${s.detail || ''}`;

// A fake firmware: hostname, get-config and svc. `silent` lists commands that get no reply
// (the service the reply would travel over was just stopped).
function firmware(host, services, conf, opts = {}) {
  const sent = [];
  const handler = (cmd, say) => {
    sent.push(cmd);
    if ((opts.silent || []).includes(cmd)) return;
    let g;
    if (cmd === 'hostname') return say('Hostname: ' + host);
    if ((g = cmd.match(/^get-config (\S+)$/))) {
      for (const [k, v] of Object.entries(conf[g[1]] || {})) say(`I (100) main: Conf '/littlefs/conf/${g[1]}:${k}' = '${v}'`);
      return say('OK: get-config ' + g[1]);
    }
    const a = cmd.split(/\s+/);
    if (a[0] !== 'svc') return say('ERR: ' + cmd);
    const sub = a[1] || 'list';
    if (sub === 'list') {
      say(`${pad('NAME', 10)} ${pad('STATE', 8)} ${pad('LOG', 6)} ${pad('ENABLED', 8)} DETAIL`);
      for (const s of services) say(svcLine(s));
      say('V=24.10 I=1.20 P=29.0W');            // interim status line before the marker
      return say('OK: ' + cmd);
    }
    const s = services.find(x => x.name === a[2]);
    if (!s) { say(`\x1b[0;33mW (5000) main: svc: unknown '${a[2]}'\x1b[0m`); return say('ERR: ' + cmd); }
    if (sub === 'on' || sub === 'off') {
      s.enabled = sub === 'on';
      if (sub === 'off') s.state = 'Stopped';
      else if (s.failsToStart) { s.state = 'Failed'; say('start failed (state=Failed)'); }
      else s.state = 'Running';
    } else if (sub === 'restart' || sub === 'rs') {
      if (!s.enabled) { say(`\x1b[0;33mW (5000) main: svc: ${s.name} is disabled, use 'svc on ${s.name}'\x1b[0m`); return say('ERR: ' + cmd); }
      s.state = 'Running';
    }
    else if (sub === 'log') s.log = a[3];
    say('OK: ' + cmd);
  };
  return { handler, sent };
}

function mqttFirmware(window, devs) {
  const orig = window.mqtt.connect;
  window.mqtt.connect = (url, o) => {
    const c = orig(url, o), pub = c.publish.bind(c);
    c.publish = (topic, msg, po, cb) => {
      pub(topic, msg, po, cb);
      const m = topic.match(/^pv\/log\/([^/]+)\/cmd$/), d = m && devs[m[1]];
      if (d) setTimeout(() => d.handler(String(msg),
        s => c.emit('message', 'pv/log/' + m[1], new TextEncoder().encode(s + '\n'))), 1);
      return c;
    };
    return c;
  };
  window.document.getElementById('mqtt-url').value = 'ws://broker:9001';
}

function bleFirmware(window, name, fwm) {
  const enc = new TextEncoder();
  let buf = '';
  const chr = {
    async startNotifications() {}, async stopNotifications() {}, addEventListener() {}, removeEventListener() {},
    async writeValueWithResponse(d) {
      buf += new TextDecoder().decode(d);
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const cmd = buf.slice(0, i).replace(/\r$/, ''); buf = buf.slice(i + 1);
        setTimeout(() => fwm.handler(cmd, s => window.onBleNotify({ target: { value: enc.encode(s + '\n') } })), 1);
      }
    },
  };
  const dev = { name, gatt: { disconnect() {}, async connect() {
    return { connected: true, disconnect() {}, async getPrimaryService() { return { async getCharacteristic() { return chr; } }; } };
  } } };
  Object.defineProperty(window.navigator, 'bluetooth', { configurable: true, value: { requestDevice: async () => dev } });
}

const SERVICES = () => [
  { name: 'mqtt', state: 'Running', log: 'info', enabled: true, detail: 'connected' },
  { name: 'tele', state: 'Running', log: 'warn', enabled: true, detail: '→ 10.0.0.2 (udp, binary)' },
  { name: 'ftp', state: 'Stopped', log: 'info', enabled: false, detail: '' },
  { name: 'ble', state: 'Running', log: 'error', enabled: true, detail: 'connected, tele stream' },
  { name: 'lcd', state: 'Failed', log: 'info', enabled: true, detail: '' },
];
const CONF = { 'ftp.conf': { enabled: '0' }, 'charger.conf': { vout_max: '28.8' } };

const $ = (w, sel) => w.document.querySelector(sel);
const rowOf = (w, name) => $(w, `#svc-body .svc[data-svc="${name}"]`);
const settle = (ms = 40) => new Promise(r => setTimeout(r, ms));
async function until(pred, ms = 3000) {
  const end = Date.now() + ms;
  while (!pred()) { if (Date.now() > end) throw new Error('timeout waiting'); await settle(10); }
}
const btn = w => w.document.getElementById('open-services');

async function connectedMqtt(services = SERVICES(), opts) {
  const { window } = await loadEditor();
  const devA = firmware('devA', services, CONF, opts);
  mqttFirmware(window, { devA, devB: firmware('devB', [], {}) });
  await window.mqttConnectAndImport('devA');
  return { window, devA, services };
}
async function openSheet(window, fwm) {
  const before = fwm.sent.length;
  btn(window).click();
  await until(() => fwm.sent.slice(before).includes('svc list') && rowOf(window, 'mqtt'));
}

test('parseSvcList: ANSI state, empty and multi-word detail, prefixes, stray lines', async () => {
  const { window } = await loadEditor();
  const lines = [
    'I (900) main: some earlier log line Running info yes',   // before the header: ignored
    'NAME       STATE    LOG    ENABLED  DETAIL',
    svcLine({ name: 'mqtt', state: 'Running', log: 'info', enabled: true, detail: 'connected' }),
    svcLine({ name: 'ftp', state: 'Stopped', log: 'warn', enabled: false, detail: '' }),
    svcLine({ name: 'tele', state: 'Running', log: 'info', enabled: true, detail: '→ 10.0.0.2 (udp, binary)' }),
    'V=24.10 I=1.20 P=29.0W',
    'I (950) svc: ' + svcLine({ name: 'lcd', state: 'Failed', log: 'error', enabled: true, detail: 'no display' }),
    svcLine({ name: 'ftp', state: 'Running', log: 'warn', enabled: true, detail: '1 client' }),
    'OK: svc list',
  ];
  const rows = JSON.parse(JSON.stringify(window.parseSvcList(lines)));
  assert.deepEqual(rows, [
    { name: 'mqtt', state: 'Running', log: 'info', enabled: true, detail: 'connected' },
    { name: 'ftp', state: 'Running', log: 'warn', enabled: true, detail: '1 client' },
    { name: 'tele', state: 'Running', log: 'info', enabled: true, detail: '→ 10.0.0.2 (udp, binary)' },
    { name: 'lcd', state: 'Failed', log: 'error', enabled: true, detail: 'no display' },
  ]);
  assert.equal(window.parseSvcList(['mqtt Running info yes', 'OK: svc list']), null, 'no header → null');
  assert.deepEqual(JSON.parse(JSON.stringify(window.parseSvcList(['NAME STATE LOG ENABLED DETAIL', 'OK: svc list']))), []);
});

test('the sheet lists services with state, detail, switch, restart and log level', async () => {
  const { window, devA } = await connectedMqtt();
  assert.equal(btn(window).disabled, false);
  await openSheet(window, devA);
  assert.ok($(window, '#services').classList.contains('open'));
  assert.equal(window.document.activeElement.id, 'svc-close');
  assert.equal($(window, '#svc-sub').textContent, 'devA over MQTT');

  const tele = rowOf(window, 'tele');
  assert.equal(tele.querySelector('.svc-state').textContent, 'Running');
  assert.ok(tele.querySelector('.svc-state').classList.contains('running'));
  assert.equal(tele.querySelector('.svc-detail').textContent, '→ 10.0.0.2 (udp, binary)');
  assert.equal(tele.querySelector('.svc-log').value, 'warn');
  assert.equal(tele.querySelector('input[type=checkbox]').checked, true);

  const ftp = rowOf(window, 'ftp');
  assert.ok(ftp.querySelector('.svc-state').classList.contains('stopped'));
  assert.equal(ftp.querySelector('.svc-detail').textContent, '');
  assert.equal(ftp.querySelector('input[type=checkbox]').checked, false);
  assert.equal(ftp.querySelector('.rs').disabled, true, 'restart needs the service enabled');
  assert.ok(rowOf(window, 'lcd').querySelector('.svc-state').classList.contains('failed'));
  assert.equal(rowOf(window, 'lcd').querySelector('.rs').disabled, false, 'a failed, enabled service can be restarted');
});

test('controls send svc on/off/restart/log and re-read the list after each', async () => {
  const { window, devA, services } = await connectedMqtt();
  await openSheet(window, devA);
  let asked = 0;
  window.confirm = () => { asked++; return true; };

  const act = async (fn, cmd) => {
    const from = devA.sent.length;
    fn();
    await until(() => devA.sent.slice(from).join('|') === cmd + '|svc list' && !$(window, '#svc-sub').classList.contains('busy'));
  };
  const cb = name => rowOf(window, name).querySelector('input[type=checkbox]');

  await act(() => { const c = cb('ftp'); c.checked = true; c.dispatchEvent(new window.Event('change')); }, 'svc on ftp');
  assert.equal(rowOf(window, 'ftp').querySelector('.svc-state').textContent, 'Running');
  assert.equal(rowOf(window, 'ftp').querySelector('.rs').disabled, false);
  // the device saved enabled=1 in ftp.conf: the loaded copy follows, no pending edit
  assert.equal(window._state.files['conf/ftp.conf'].original.enabled, '1');
  assert.equal(window.pendingCount(), 0);

  await act(() => rowOf(window, 'tele').querySelector('.rs').click(), 'svc restart tele');
  await act(() => { const s = rowOf(window, 'tele').querySelector('.svc-log'); s.value = 'error'; s.dispatchEvent(new window.Event('change')); }, 'svc log tele error');
  assert.equal(rowOf(window, 'tele').querySelector('.svc-log').value, 'error');
  await act(() => { const c = cb('tele'); c.checked = false; c.dispatchEvent(new window.Event('change')); }, 'svc off tele');
  assert.equal(services.find(s => s.name === 'tele').enabled, false);
  assert.equal(asked, 0, 'stopping a service the link does not use asks nothing');

  const from = devA.sent.length;
  $(window, '#svc-refresh').click();
  await until(() => devA.sent.slice(from).join('|') === 'svc list' && !$(window, '#svc-sub').classList.contains('busy'));
});

test('a rejected command shows the firmware reason; a failed start is reported', async () => {
  const services = SERVICES();
  services.find(s => s.name === 'ftp').failsToStart = true;
  const { window, devA } = await connectedMqtt(services);
  await openSheet(window, devA);
  const c = rowOf(window, 'ftp').querySelector('input[type=checkbox]');
  c.checked = true; c.dispatchEvent(new window.Event('change'));
  await until(() => /did not start/.test($(window, '#svc-msg').textContent));
  assert.match($(window, '#svc-msg').textContent, /ftp did not start: state=Failed/);
  assert.ok($(window, '#svc-msg').classList.contains('err'));
  await until(() => !$(window, '#svc-sub').classList.contains('busy'));

  await window.svcAction('nope', 'restart');
  assert.match($(window, '#svc-msg').textContent, /The device rejected “svc restart nope”: unknown 'nope'/);

  // switched off behind the editor's back (console): the firmware refuses the restart
  services.find(s => s.name === 'tele').enabled = false;
  await window.svcAction('tele', 'restart');
  assert.match($(window, '#svc-msg').textContent, /The device rejected “svc restart tele”: tele is disabled, use 'svc on tele'/);
  assert.equal(services.find(s => s.name === 'tele').state, 'Running', 'state untouched');
});

test('gating: no services without a read, or after a failed switch to another device', async () => {
  const { window } = await loadEditor();
  assert.equal(btn(window).disabled, true, 'nothing loaded');
  window.load({ 'conf/ftp.conf': new TextEncoder().encode('enabled=1\n') }, 'preset');
  assert.equal(btn(window).disabled, true, 'a preset has no device');

  const devA = firmware('devA', SERVICES(), CONF), devB = firmware('devB', [], {});
  mqttFirmware(window, { devA, devB });
  window.confirm = () => true;
  await window.mqttConnectAndImport('devA');
  assert.equal(btn(window).disabled, false);

  await window.mqttConnectAndImport('devB');          // answers but has no config → import fails
  assert.equal(window._state.deviceName, 'devA');
  assert.equal(window.svcTarget(), null);
  assert.equal(btn(window).disabled, true, 'devA link is closed, devB was never read');

  const a = devA.sent.length, b = devB.sent.length;
  window.svcOpen();
  assert.equal($(window, '#services').classList.contains('open'), false);
  await window.svcAction('ftp', 'off');
  await window.svcRefresh();
  await settle();
  assert.equal(devA.sent.length, a);
  assert.equal(devB.sent.length, b);
});

test('the sheet goes read-only when the link closes while it is open', async () => {
  const { window, devA } = await connectedMqtt();
  await openSheet(window, devA);
  await window.disconnectMqtt(); window.refreshConn();
  assert.equal(rowOf(window, 'mqtt'), null);
  assert.match($(window, '#svc-body').textContent, /Not connected/);
  assert.equal($(window, '#svc-refresh').disabled, true);
  assert.equal(btn(window).disabled, true);
});

test('stopping mqtt over MQTT: confirm names the link and the BMS data, then the header shows disconnected', async () => {
  const { window, devA } = await connectedMqtt(SERVICES(), { silent: ['svc off mqtt'] });
  window.svcTimings().own = 60;
  await openSheet(window, devA);
  const cb = () => rowOf(window, 'mqtt').querySelector('input[type=checkbox]');

  let prompt = '';
  window.confirm = m => { prompt = m; return false; };
  cb().checked = false; cb().dispatchEvent(new window.Event('change'));
  assert.match(prompt, /^Stop mqtt on devA\?/);
  assert.match(prompt, /connected to devA over MQTT\. Stopping mqtt drops that link/);
  assert.match(prompt, /BMS data/);
  await settle();
  assert.ok(!devA.sent.includes('svc off mqtt'), 'declined → nothing sent');
  assert.equal(cb().checked, true, 'declined → switch back on');

  window.confirm = () => true;
  cb().checked = false; cb().dispatchEvent(new window.Event('change'));
  await until(() => /link to devA is closed/.test($(window, '#svc-msg').textContent));
  assert.match($(window, '#svc-msg').textContent, /sent, not confirmed/, 'no OK and no presence: the stop is not confirmed');
  assert.ok(devA.sent.includes('svc off mqtt'));
  assert.equal($(window, '#conn').textContent, 'Disconnected from devA over MQTT');
  assert.equal(window._activeWrite, null);
  assert.equal(btn(window).disabled, true);
});

test('restarting mqtt over MQTT waits for the device and keeps the link', async () => {
  const { window, devA } = await connectedMqtt(SERVICES(), { silent: ['svc restart mqtt'] });
  Object.assign(window.svcTimings(), { own: 60, retry: 20 });
  await openSheet(window, devA);
  let prompt = '';
  window.confirm = m => { prompt = m; return true; };
  rowOf(window, 'mqtt').querySelector('.rs').click();
  assert.match(prompt, /^Restart mqtt on devA\?/);
  assert.match(prompt, /link drops while mqtt restarts/);
  assert.doesNotMatch(prompt, /BMS/);
  await until(() => /link is back/.test($(window, '#svc-msg').textContent));
  assert.match($(window, '#svc-msg').textContent, /not confirmed/, 'no OK and no presence: running, but the restart is not confirmed');
  assert.equal($(window, '#conn').textContent, 'Connected to devA over MQTT');
  assert.ok(rowOf(window, 'mqtt'));
});

test('restart of the own link with no answer ends disconnected', async () => {
  const { window, devA } = await connectedMqtt(SERVICES(), { silent: ['svc restart mqtt', 'svc list'] });
  Object.assign(window.svcTimings(), { own: 30, retry: 10, tries: 2, cmd: 40 });
  window.confirm = () => true;
  await window.svcAction('mqtt', 'restart');
  assert.match($(window, '#svc-msg').textContent, /did not answer over MQTT again/);
  assert.equal($(window, '#conn').textContent, 'Disconnected from devA over MQTT');
});

test('over Bluetooth: stopping ble confirms the link; stopping mqtt only warns about BMS data', async () => {
  const { window } = await loadEditor();
  const fwm = firmware('devA', SERVICES(), CONF, { silent: ['svc off ble'] });
  bleFirmware(window, 'devA', fwm);
  await window.connectBluetooth();
  assert.equal($(window, '#conn').textContent, 'Connected to devA over Bluetooth');
  window.svcTimings().own = 60;

  const prompts = [];
  window.confirm = m => { prompts.push(m); return true; };
  await window.svcAction('mqtt', 'off');
  assert.equal(prompts.length, 1);
  assert.match(prompts[0], /BMS data/);
  assert.doesNotMatch(prompts[0], /drops that link/);
  assert.ok(fwm.sent.includes('svc off mqtt'));
  assert.equal($(window, '#conn').textContent, 'Connected to devA over Bluetooth');

  await window.svcAction('ble', 'off');
  assert.equal(prompts.length, 2);
  assert.match(prompts[1], /connected to devA over Bluetooth\. Stopping ble drops that link/);
  assert.match(prompts[1], /another link \(USB serial or MQTT\)/);
  assert.doesNotMatch(prompts[1], /BMS/);
  assert.equal($(window, '#conn').textContent, 'Disconnected from devA over Bluetooth');
});

test('Escape and the scrim close the sheet; focus returns to the Services button', async () => {
  const { window, devA } = await connectedMqtt();
  await openSheet(window, devA);
  rowOf(window, 'tele').querySelector('.rs').focus();
  window.document.dispatchEvent(new window.KeyboardEvent('keydown', { key: 'Escape', bubbles: true }));
  assert.equal($(window, '#services').classList.contains('open'), false);
  assert.equal(window.document.activeElement.id, 'open-services');

  await openSheet(window, devA);
  $(window, '#svc-scrim').click();
  assert.equal($(window, '#services').classList.contains('open'), false);
});
