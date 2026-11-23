// Spec §6.1 and §2.14: restarts only of enabled services, honest reporting of unconfirmed
// reboots/restarts/stops, failed starts, services missing from the build, the device lock,
// and the review warnings for switching a link service off.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

const enc = new TextEncoder();
const pad = (s, n) => String(s).padEnd(n);
const settle = (ms = 30) => new Promise(r => setTimeout(r, ms));
async function until(pred, ms = 3000) {
  const end = Date.now() + ms;
  while (!pred()) { if (Date.now() > end) throw new Error('timeout waiting'); await settle(5); }
}
function edit(w, key, value) { const i = w.document.getElementById('f-' + key); i.value = value; i.dispatchEvent(new w.Event('input')); }
const toggle = (w, key) => w.document.getElementById('f-' + key).click();
function showTab(w, path) { w._state.active = path; w.renderApp(); }

const CONF = () => ({
  'board.conf': { pwm_freq: '39000' },
  'mqtt.conf': { enabled: '1', broker_uri: 'mqtt://a' },
  'tele.conf': { enabled: '1', influxdb_host: 'a', log_level: 'info' },
  'ftp.conf': { enabled: '0' },
  'ble.conf': { enabled: '1' },
  'bsync.conf': { enabled: '0' },
});
const SERVICES = () => [
  { name: 'mqtt', state: 'Running', log: 'info', enabled: true },
  { name: 'tele', state: 'Running', log: 'info', enabled: true },
  { name: 'ftp', state: 'Stopped', log: 'info', enabled: false },
  { name: 'ble', state: 'Running', log: 'info', enabled: true },
  { name: 'telnet', state: 'Stopped', log: 'info', enabled: false },
];

// A fake firmware. Its own link (`link`: mqtt or ble) loses the reply of `svc off|restart <link>`
// (cmdService logs after stop()). opts: drop [regex] (command lost), presence (MQTT: announce
// offline/online like mqtt.cpp), fails [names] (onStart fails), delay ms (set-config replies).
function device(host, opts = {}) {
  const services = opts.services || SERVICES(), conf = opts.conf || CONF(), sent = [];
  const handler = (cmd, say, pres, link) => {
    sent.push(cmd);
    if ((opts.drop || []).some(r => r.test(cmd))) return;
    let g;
    if (cmd === 'hostname') return say('Hostname: ' + host);
    if ((g = cmd.match(/^get-config (\S+)$/))) {
      for (const [k, v] of Object.entries(conf[g[1]] || {})) say(`Conf '/littlefs/conf/${g[1]}:${k}' = '${v}'`);
      return say('OK: ' + cmd);
    }
    if (cmd === 'restart') return say('I (5) main: Rebooting');
    if ((g = cmd.match(/^(set|del)-config (\S+) (\S+)/))) {
      const reply = () => say('OK: ' + cmd);
      return opts.delay ? setTimeout(reply, opts.delay) : reply();
    }
    const a = cmd.split(/\s+/);
    if (a[0] !== 'svc') return say('ERR: ' + cmd);
    if (a[1] === 'list') {
      say(`${pad('NAME', 10)} ${pad('STATE', 8)} ${pad('LOG', 6)} ${pad('ENABLED', 8)} DETAIL`);
      for (const s of services) say(`${pad(s.name, 10)} \x1b[32m${pad(s.state, 8)}\x1b[0m ${pad(s.log, 6)} ${pad(s.enabled ? 'yes' : 'no', 8)} `);
      return say('OK: ' + cmd);
    }
    const s = services.find(x => x.name === a[2]);
    if (!s) { say(`\x1b[0;33mW (5000) main: svc: unknown '${a[2]}'\x1b[0m`); return say('ERR: ' + cmd); }
    const fails = (opts.fails || []).includes(s.name);
    const start = () => { if (fails) { s.state = 'Failed'; say(`\x1b[0;31mE (9) service: '${s.name}' failed: onStart returned false\x1b[0m`); return false; } s.state = 'Running'; return true; };
    if (a[1] === 'on') { s.enabled = true; if (!start()) say('start failed (state=Failed)'); }
    else if (a[1] === 'off') { s.enabled = false; s.state = 'Stopped'; }
    else if (a[1] === 'restart') { s.state = 'Stopped'; start(); }
    else if (a[1] === 'log') s.log = a[3];
    if (s.name === link && (a[1] === 'off' || a[1] === 'restart')) {
      if (opts.presence) { pres('offline'); if (a[1] === 'restart') pres('online'); }
      if (link === 'ble') opts.server && (opts.server.connected = false);
      return;                                   // the OK cannot travel over the stopped link
    }
    say('OK: ' + cmd);
  };
  return { handler, sent, services, conf };
}

function onMqtt(w, devs) {
  const orig = w.mqtt.connect;
  w.mqtt.connect = (url, o) => {
    const c = orig(url, o), pub = c.publish.bind(c);
    c.publish = (topic, msg, po, cb) => {
      pub(topic, msg, po, cb);
      const m = topic.match(/^pv\/log\/([^/]+)\/cmd$/), d = m && devs[m[1]];
      if (d) setTimeout(() => d.handler(String(msg),
        s => c.emit('message', 'pv/log/' + m[1], enc.encode(s + '\n')),
        v => c.emit('message', 'pv/dev/' + m[1], enc.encode(v), { retain: false }), 'mqtt'), 1);
      return c;
    };
    return c;
  };
  w.document.getElementById('mqtt-url').value = 'ws://broker:9001';
}
async function overMqtt(opts) {
  const { window: w } = await loadEditor();
  const dev = device('devA', opts);
  onMqtt(w, { devA: dev });
  await w.mqttConnectAndImport('devA');
  w.confirm = () => true;
  Object.assign(w.svcTimings(), { own: 40, retry: 10, tries: 3, cmd: 300 });
  return { w, dev };
}
async function overBle(opts = {}) {
  const { window: w } = await loadEditor();
  const server = { connected: true, disconnect() { this.connected = false; }, async getPrimaryService() { return { async getCharacteristic() { return chr; } }; } };
  const dev = device('devA', { ...opts, server });
  let buf = '';
  const chr = {
    async startNotifications() {}, async stopNotifications() {}, addEventListener() {}, removeEventListener() {},
    async writeValueWithResponse(d) {
      buf += new TextDecoder().decode(d);
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const cmd = buf.slice(0, i).replace(/\r$/, ''); buf = buf.slice(i + 1);
        setTimeout(() => dev.handler(cmd, s => w.onBleNotify({ target: { value: enc.encode(s + '\n') } }), () => {}, 'ble'), 1);
      }
    },
  };
  const bt = { name: 'devA', gatt: { disconnect() {}, async connect() { server.connected = true; return server; } } };
  Object.defineProperty(w.navigator, 'bluetooth', { configurable: true, value: { requestDevice: async () => bt } });
  await w.connectBluetooth();
  w.confirm = () => true;
  Object.assign(w.svcTimings(), { own: 40, retry: 10, tries: 3, cmd: 300 });
  return { w, dev, server };
}
const $ = (w, sel) => w.document.querySelector(sel);
const bar = w => w.document.getElementById('apply').hidden ? [] : [...w.document.querySelectorAll('#apply button')].map(b => b.textContent);
const barText = w => $(w, '#apply').firstElementChild.textContent;
const status = w => $(w, '#serial-status').textContent;
const svcMsg = w => $(w, '#svc-msg').textContent;
const after = (dev, from) => dev.sent.slice(from).filter(c => !/^(hostname|get-config )/.test(c));
async function openSheet(w) { w.svcOpen(); await until(() => $(w, '#svc-body .svc') && !$(w, '#svc-sub').classList.contains('busy')); }

// ---- B1: restart only enabled services ----

test('svc off + a host change in one push: no Restart, "applies when tele is turned on"', async () => {
  const { w, dev } = await overMqtt();
  showTab(w, 'conf/tele.conf'); toggle(w, 'enabled'); edit(w, 'influxdb_host', 'b');
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(bar(w), []);
  assert.equal($(w, '#apply').hidden, false);
  assert.equal(barText(w), 'tele: applies when tele is turned on');
  assert.match(status(w), /uploaded 2\/2 changed fields — tele: applies when tele is turned on/);
  const from = dev.sent.length;
  await w.applyStep('tele');
  assert.ok(!after(dev, from).includes('svc restart tele'), 'a restart would start the disabled service');
});

test('tele switched off in the Services sheet after the push: the Restart button goes', async () => {
  const { w, dev } = await overMqtt();
  showTab(w, 'conf/tele.conf'); edit(w, 'influxdb_host', 'b');
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(bar(w), ['Restart tele']);
  await openSheet(w);
  await w.svcAction('tele', 'off');
  assert.deepEqual(bar(w), []);
  assert.equal(barText(w), 'tele: applies when tele is turned on');
  const from = dev.sent.length;
  await w.applyStep('tele');
  assert.deepEqual(after(dev, from), []);
  assert.equal(dev.services.find(s => s.name === 'tele').state, 'Stopped');
  // turning it on applies the change: the note goes
  await w.svcAction('tele', 'on');
  assert.equal($(w, '#apply').hidden, true);
});

test('Restart re-checks enabled on the device right before sending', async () => {
  const { w, dev } = await overMqtt();
  showTab(w, 'conf/tele.conf'); edit(w, 'influxdb_host', 'b');
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(bar(w), ['Restart tele']);
  Object.assign(dev.services.find(s => s.name === 'tele'), { enabled: false, state: 'Stopped' });   // e.g. `svc off tele` typed elsewhere
  const from = dev.sent.length;
  await w.applyStep('tele');
  assert.deepEqual(after(dev, from), ['svc list']);
  assert.match(status(w), /tele is off on devA; the change applies when tele is turned on/);
  assert.deepEqual(bar(w), []);
});

test('Services sheet: no restart of a disabled service', async () => {
  const { w, dev } = await overMqtt();
  await openSheet(w);
  assert.equal($(w, '#svc-body .svc[data-svc="ftp"] .rs').disabled, true);
  const from = dev.sent.length;
  await w.svcAction('ftp', 'restart');
  assert.deepEqual(after(dev, from), []);
  assert.match(svcMsg(w), /ftp is off/);
});

// ---- B2: unconfirmed reboot / restart / stop ----

test('reboot: no "Rebooting" line → sent, not confirmed; the button and the link stay', async () => {
  const { w } = await overMqtt({ drop: [/^restart$/] });
  showTab(w, 'conf/board.conf'); edit(w, 'pwm_freq', '40000');
  await w.uploadChanged({ reviewed: true });
  await w.applyStep('');
  assert.match(status(w), /“restart” sent to devA, not confirmed: no “Rebooting” line/);
  assert.doesNotMatch(status(w), /is rebooting/);
  assert.deepEqual(bar(w), ['Reboot device']);
  assert.equal($(w, '#conn').textContent, 'Connected to devA over MQTT');
});

test('restart of a service on another link: no reply → not confirmed, button stays', async () => {
  const { w } = await overMqtt({ drop: [/^svc restart tele$/] });
  showTab(w, 'conf/tele.conf'); edit(w, 'influxdb_host', 'b');
  await w.uploadChanged({ reviewed: true });
  await w.applyStep('tele');
  assert.match(status(w), /“svc restart tele” sent to devA, not confirmed/);
  assert.deepEqual(bar(w), ['Restart tele']);
});

test('Restart mqtt over MQTT, command lost: not confirmed, the button stays', async () => {
  const { w } = await overMqtt({ drop: [/^svc restart mqtt$/] });
  showTab(w, 'conf/mqtt.conf'); edit(w, 'broker_uri', 'mqtt://b');
  await w.uploadChanged({ reviewed: true });
  await w.applyStep('mqtt');
  assert.match(status(w), /“svc restart mqtt” sent, not confirmed: mqtt is running and the MQTT link is back, but nothing showed that it restarted/);
  assert.deepEqual(bar(w), ['Restart mqtt']);
});

test('Restart mqtt over MQTT: presence offline/online confirms it without the OK', async () => {
  const { w } = await overMqtt({ presence: true });
  showTab(w, 'conf/mqtt.conf'); edit(w, 'broker_uri', 'mqtt://b');
  await w.uploadChanged({ reviewed: true });
  await w.applyStep('mqtt');
  assert.match(status(w), /mqtt restarted on devA; the MQTT link is back/);
  assert.equal($(w, '#apply').hidden, true);
});

test('Services sheet: svc off mqtt over MQTT, lost → sent, not confirmed; link closed; conf not changed', async () => {
  const { w, dev } = await overMqtt({ drop: [/^svc off mqtt$/] });
  await openSheet(w);
  await w.svcAction('mqtt', 'off');
  assert.match(svcMsg(w), /“svc off mqtt” sent, not confirmed: the reply cannot come back over MQTT once mqtt stops/);
  assert.ok($(w, '#svc-msg').classList.contains('err'));
  assert.equal($(w, '#conn').textContent, 'Disconnected from devA over MQTT');
  assert.equal(w._state.files['conf/mqtt.conf'].original.enabled, '1');
  assert.equal(dev.services.find(s => s.name === 'mqtt').enabled, true);
});

test('Services sheet: svc off mqtt over MQTT with the device announcing offline → stopped', async () => {
  const { w } = await overMqtt({ presence: true });
  await openSheet(w);
  await w.svcAction('mqtt', 'off');
  assert.match(svcMsg(w), /^mqtt stopped; the MQTT link to devA is closed/);
  assert.equal(w._state.files['conf/mqtt.conf'].original.enabled, '0');
});

test('Services sheet: restart mqtt over MQTT, lost → not reported as restarted', async () => {
  const { w } = await overMqtt({ drop: [/^svc restart mqtt$/] });
  await openSheet(w);
  await w.svcAction('mqtt', 'restart');
  assert.match(svcMsg(w), /not confirmed/);
  assert.doesNotMatch(svcMsg(w), /mqtt restarted/);
});

test('over Bluetooth: Restart ble is confirmed by the device dropping the GATT link', async () => {
  const { w } = await overBle();
  showTab(w, 'conf/tele.conf'); edit(w, 'adv_ms', '500');   // applies after `svc restart ble`
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(bar(w), ['Restart ble']);
  await w.applyStep('ble');
  assert.match(status(w), /ble restarted on devA; the Bluetooth link is back/);
  assert.equal($(w, '#apply').hidden, true);
});

// ---- should-fix 1: a failed start is a failure ----

test('push: svc on that fails to start is reported with the firmware text', async () => {
  const { w } = await overMqtt({ fails: ['ftp'] });
  showTab(w, 'conf/ftp.conf'); toggle(w, 'enabled');
  await w.uploadChanged({ reviewed: true });
  assert.match(status(w), /uploaded 0\/1 changed field — failed: ftp\.conf:enabled \(ftp did not start: onStart returned false\)/);
  assert.ok($(w, '#serial-status').classList.contains('err') || /failed/.test(status(w)));
});

test('apply: svc restart whose start fails is an error and the button stays', async () => {
  const { w } = await overMqtt({ fails: ['tele'] });
  showTab(w, 'conf/tele.conf'); edit(w, 'influxdb_host', 'b');
  await w.uploadChanged({ reviewed: true });
  await w.applyStep('tele');
  assert.match(status(w), /tele did not start after the restart on devA: onStart returned false/);
  assert.deepEqual(bar(w), ['Restart tele']);
});

test('Services sheet: a restart whose start fails says so', async () => {
  const { w } = await overMqtt({ fails: ['tele'] });
  await openSheet(w);
  await w.svcAction('tele', 'restart');
  assert.match(svcMsg(w), /tele did not start: onStart returned false/);
});

// ---- should-fix 2: services not in the build ----

test('push: svc unknown → falls back to set-config, after reboot', async () => {
  const { w, dev } = await overMqtt();
  showTab(w, 'conf/bsync.conf'); toggle(w, 'enabled');
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(after(dev, 0).slice(-2), ['svc on bsync', 'set-config bsync.conf enabled 1']);
  assert.match(status(w), /uploaded 1\/1 changed field — takes effect after: reboot/);
  assert.equal(w.pendingCount(), 0);
  // next time it goes as set-config straight away
  edit(w, 'log_level', 'warn');
  assert.deepEqual([...w.pendingCommands().map(c => c.str)], ['set-config bsync.conf log_level warn']);
});

test('with a svc list read, only listed services go as svc commands', async () => {
  const { w } = await overMqtt();
  await openSheet(w);
  showTab(w, 'conf/bsync.conf'); toggle(w, 'enabled');
  showTab(w, 'conf/tele.conf'); edit(w, 'log_level', 'warn');
  const cmds = w.pendingCommands();
  assert.deepEqual([...cmds.map(c => c.str)].sort(), ['set-config bsync.conf enabled 1', 'svc log tele warn']);
  w.openReview();
  const when = [...w.document.querySelectorAll('#review-body .diff .when')].map(e => e.textContent);
  assert.ok(when.includes('after reboot') && when.includes('now'), JSON.stringify(when));
});

// ---- should-fix 3: switching a link service off warns in the review ----

test('review warns when mqtt or ble is switched off', async () => {
  const { w } = await overMqtt();
  showTab(w, 'conf/mqtt.conf'); toggle(w, 'enabled');       // own link: set-config, after reboot
  showTab(w, 'conf/ble.conf'); toggle(w, 'enabled');        // svc off ble: now
  w.openReview();
  const warns = [...w.document.querySelectorAll('#review-body .rwarn li')].map(l => l.textContent);
  assert.deepEqual(warns, [
    'ble.conf enabled=0: ble stops now. Its Bluetooth console closes.',
    'mqtt.conf enabled=0: mqtt stops after reboot. Its console closes, and a charger that gets its BMS data over MQTT loses it and falls back to its stale-cell voltage cap.',
  ]);
});

// ---- should-fix 4: one device command sequence at a time ----

test('while a push runs, apply, Services, console and a second push send nothing', async () => {
  const { w, dev } = await overMqtt({ delay: 150 });
  showTab(w, 'conf/tele.conf'); edit(w, 'influxdb_host', 'b');
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(bar(w), ['Restart tele']);
  await openSheet(w);
  edit(w, 'influxdb_host', 'c');
  const push = w.uploadChanged({ reviewed: true });
  await until(() => dev.sent.includes('set-config tele.conf influxdb_host c'));
  const from = dev.sent.length;
  assert.ok($(w, '#uploadlog-cmd').disabled, 'console input disabled');
  assert.ok([...w.document.querySelectorAll('#apply button')].every(b => b.disabled), 'apply buttons disabled');
  assert.ok([...w.document.querySelectorAll('#svc-body .svc input, #svc-body .svc button, #svc-body .svc select')].every(e => e.disabled), 'sheet disabled');
  assert.ok($(w, '#svc-sub').classList.contains('busy'), 'spinner');
  assert.ok($(w, '#serial-status').classList.contains('busy'), 'status spinner');
  await w.applyStep('tele');
  await w.svcAction('tele', 'restart');
  await w.consoleSend('status');
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(dev.sent.slice(from), []);
  await push;
  assert.equal($(w, '#uploadlog-cmd').disabled, false);
  assert.deepEqual(bar(w), ['Restart tele']);
  assert.equal([...w.document.querySelectorAll('#apply button')].some(b => b.disabled), false);
});

// ---- should-fix 5: reboot when the own link was switched off in the push ----

test('reboot after pushing mqtt enabled=0 over MQTT: the confirm says the link does not come back', async () => {
  const { w } = await overMqtt();
  showTab(w, 'conf/mqtt.conf'); toggle(w, 'enabled');
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(bar(w), ['Reboot device']);
  let prompt = ''; w.confirm = m => { prompt = m; return true; };
  await w.applyStep('');
  assert.match(prompt, /this MQTT link does not come back after the reboot; connect over USB serial or Bluetooth instead/);
  assert.match(prompt, /stale-cell voltage cap/);
  assert.doesNotMatch(prompt, /reconnect from Open/);
  assert.match(status(w), /devA is rebooting; mqtt stays off there, so connect over USB serial or Bluetooth/);
});

// ---- nits ----

test('re-reading the same device keeps what is still to apply', async () => {
  const { w } = await overMqtt();
  showTab(w, 'conf/tele.conf'); edit(w, 'influxdb_host', 'b');
  await w.uploadChanged({ reviewed: true });
  assert.deepEqual(bar(w), ['Restart tele']);
  await w.mqttConnectAndImport('devA');
  assert.deepEqual(bar(w), ['Restart tele']);
});

test('a Services-sheet restart clears the matching apply button', async () => {
  const { w } = await overMqtt();
  showTab(w, 'conf/tele.conf'); edit(w, 'influxdb_host', 'b');
  await w.uploadChanged({ reviewed: true });
  await openSheet(w);
  await w.svcAction('tele', 'restart');
  assert.equal($(w, '#apply').hidden, true);
});

test('reply markers match the whole command: "OK: svc on telnet" does not answer "svc on tele"', async () => {
  const { window: w } = await loadEditor();
  w._activeWrite = async () => {};
  let res = 'pending';
  w.sendCommand('svc on tele', null, 200).then(r => { res = r; }, e => { res = e.message; });
  w._dispatchLine('OK: svc on telnet');
  w._dispatchLine('ERR: svc on teleX');
  await settle(5);
  assert.equal(res, 'pending');
  w._dispatchLine('\x1b[0mOK: svc on tele \r');
  await settle(5);
  assert.equal(res, true);
});
