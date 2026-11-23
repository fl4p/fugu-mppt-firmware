// Live strip (spec §2.1.1): parsing the `V=… st=…` status line, the stale state, stop states, when it
// shows, the dashboard link per transport, and that the strip itself sends nothing.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');
const { STATUS, firmware, mqttHook, bleDevice } = require('./_fakes');

const settle = (ms = 20) => new Promise(r => setTimeout(r, ms));
async function until(pred, ms = 4000) {
  const end = Date.now() + ms;
  while (!pred()) { if (Date.now() > end) throw new Error('timeout waiting'); await settle(10); }
}
const $ = (w, id) => w.document.getElementById(id);
const strip = w => {
  const [p, v, m, a] = $(w, 'live').querySelector('.lv-txt').children;
  return { p: p.textContent, v: v.textContent, m: m.textContent, a: a.textContent, stop: m.classList.contains('stop'),
           stale: $(w, 'live').classList.contains('stale'), hidden: $(w, 'live').hidden };
};
const line = (st, p = '118.4') => `I (5) main: V= 38.2/27.10 I=3.10/4.20A ${p}W 31℃40℃ 99sps  0㎅/s CCM(H|L|Lm)=1|1|1 st=${st},1 lag=1㎲ N=1 rssi=-6`;

async function mqttRead(user = 'pv') {
  const devA = firmware('devA');
  const { window } = await loadEditor({ beforeParse: w => mqttHook(w, { devA }) });
  $(window, 'mqtt-url').value = 'ws://broker:9001';
  $(window, 'mqtt-user').value = user;
  $(window, 'mqtt-pass').value = 'secret';
  assert.equal(await window.mqttConnectAndImport('devA'), true);
  return { window, devA };
}

test('liveParse: values and mode from the status line; other lines are ignored', async () => {
  const { window } = await loadEditor();
  const p = l => { const r = window.liveParse(l); return r && JSON.parse(JSON.stringify(r)); };
  assert.deepEqual(p(STATUS), { vin: 38.2, vout: 27.1, p: 118.4, st: 'MPPT', mode: 'MPPT', stop: false });
  assert.deepEqual(p('\x1b[0;32m' + line(' ⇣MPPT') + '\x1b[0m'), { vin: 38.2, vout: 27.1, p: 118.4, st: 'MPPT', mode: 'MPPT', stop: false });
  assert.equal(p(line('SWEEP')).mode, 'Sweep');
  assert.equal(p(line(' MANU')).mode, 'manual');
  assert.equal(p(line('  PVS')).mode, 'PV sim');
  assert.equal(p(line('   CV')).mode, 'CV');
  assert.deepEqual([p(line('START')).mode, p(line('START')).stop], ['stopped', true]);
  assert.deepEqual([p(line('   UV')).mode, p(line('   UV')).stop], ['undervoltage', true]);
  assert.equal(p('V=24.10 I=1.20 P=29.0W'), null);
  assert.equal(p("I (100) main: Conf '/littlefs/conf/charger.conf:vout_max' = '28.8'"), null);
  assert.equal(p('OK: get-config charger.conf'), null);
});

test('strip: hidden without a device read, shows the read device\'s status line, hidden on disconnect', async () => {
  const { window: w0 } = await loadEditor();
  w0._dispatchLine(STATUS);
  w0.renderLive();
  assert.equal(strip(w0).hidden, true, 'no push target: nothing shown');

  const { window } = await mqttRead();
  // the read itself carried interim status lines
  assert.deepEqual(strip(window), { p: '118.4 W', v: '38.2 → 27.10 V', m: 'MPPT', a: 'now', stop: false, stale: false, hidden: false });
  window._dispatchLine(line('START', '  0.0'));
  assert.deepEqual(strip(window), { p: '0.0 W', v: '38.2 → 27.10 V', m: 'stopped', a: 'now', stop: true, stale: false, hidden: false });
  await window.disconnectMqtt();
  window.refreshConn();
  assert.equal(strip(window).hidden, true);
  assert.match($(window, 'conn').textContent, /^Disconnected from devA/);
});

test('strip: grey after 10 s without a line (30 s while stopped or idle), live again on the next line', async () => {
  const { window } = await mqttRead();
  window._live.at -= 9000; window.renderLive();
  assert.equal(strip(window).stale, false);
  assert.equal(strip(window).a, '9 s ago');
  window._live.at -= 2000; window.renderLive();
  assert.equal(strip(window).stale, true);
  assert.match($(window, 'live').title, /\(stale\)/);
  window._dispatchLine(STATUS);
  assert.equal(strip(window).stale, false);
  for (const st of ['START', ' MPPT']) {
    window._dispatchLine(line(st, st === 'START' ? '  0.0' : '  0.2'));
    window._live.at -= 20000; window.renderLive();
    assert.equal(strip(window).stale, false, st + ' at 20 s');
    window._live.at -= 11000; window.renderLive();
    assert.equal(strip(window).stale, true, st + ' at 31 s');
  }
  window._live.at -= 120000; window.renderLive();
  assert.equal(strip(window).a, '3 min ago');
});

test('strip: waiting text until the first status line after a connect', async () => {
  const { window } = await mqttRead();
  window._live = null; window.renderLive();
  const s = strip(window);
  assert.equal(s.p, 'waiting for a status line');
  assert.equal(s.stale, true);
  assert.equal(s.m, '');
});

test('strip sends nothing: lines, renders and its 1 s timer publish no command', async () => {
  const { window, devA } = await mqttRead();
  const c = window._mqttClients[0], pubs = c._publishes.length, sent = devA.sent.length;
  for (let i = 0; i < 20; i++) window._dispatchLine(i % 2 ? STATUS : line('START', '0.0'));
  window.renderLive();
  await settle(1300);   // at least one timer tick
  assert.equal(c._publishes.length, pubs);
  assert.equal(devA.sent.length, sent);
});

test('dashboard link over MQTT: broker, user and device in the fragment, never the password', async () => {
  const { window } = await mqttRead();
  const a = $(window, 'live-dash');
  assert.equal(a.hidden, false);
  assert.equal(a.textContent, 'Open dashboard');
  assert.equal(a.target, '_blank');
  const u = new URL(a.href);
  assert.equal(u.pathname, '/dashboard.html');
  const h = new URLSearchParams(u.hash.slice(1));
  assert.deepEqual([...h.keys()].sort(), ['device', 'mqtt', 'user']);
  assert.equal(h.get('mqtt'), 'ws://broker:9001');
  assert.equal(h.get('user'), 'pv');
  assert.equal(h.get('device'), 'devA');
  assert.ok(!a.href.includes('secret'));
  // a click opens the tab (default action) and keeps the editor's own link
  a.dispatchEvent(new window.MouseEvent('click', { bubbles: true, cancelable: true }));
  assert.ok(window._activeWrite);
  assert.equal(strip(window).hidden, false);
});

test('dashboard link over Bluetooth releases the editor\'s link; disabled while a command runs', async () => {
  const fw = firmware('devB');
  let dev;
  const { window } = await loadEditor({
    beforeParse: w => {
      dev = bleDevice(w, { id: 'id-2', fw });
      Object.defineProperty(w.navigator, 'bluetooth', { configurable: true, value: { requestDevice: async () => dev } });
    },
  });
  await window.connectBluetooth();
  assert.equal(window._state.deviceName, 'devB');
  const a = $(window, 'live-dash');
  assert.equal(a.textContent, 'Release link & open dashboard');
  assert.match(a.title, /one Bluetooth connection at a time/);
  const h = new URLSearchParams(new URL(a.href).hash.slice(1));
  assert.deepEqual(Object.fromEntries(h), { ble: 'id-2', name: 'fugu-devB', device: 'devB' });
  // while a device command holds the lock the click does nothing
  let release;
  const busy = window.withDevice('test', () => new Promise(r => { release = r; }));
  window.renderLive();
  assert.ok(a.classList.contains('off'));
  let ev = new window.MouseEvent('click', { bubbles: true, cancelable: true });
  a.dispatchEvent(ev);
  assert.equal(ev.defaultPrevented, true);
  assert.equal(dev.gatt.connected, true);
  release(); await busy;
  ev = new window.MouseEvent('click', { bubbles: true, cancelable: true });
  a.dispatchEvent(ev);
  assert.equal(ev.defaultPrevented, false);
  await until(() => !window._activeWrite);
  assert.equal(dev.gatt.connected, false);
  assert.equal(strip(window).hidden, true);
  assert.equal($(window, 'conn').textContent, 'Disconnected from devB over Bluetooth');
  assert.match($(window, 'serial-status').textContent, /released for the dashboard/);
  // the push target is kept: a later push reconnects to the same device
  assert.equal(window._lastTransport, 'ble');
  assert.ok(window.pushTarget());
});
