// Spec §5.4 (push target), §2.2 (rows, groups), §6 (review sheet): push only goes to the
// device the loaded config was read from; invalid values, labels, folded groups.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

function mk(map){
  const enc = new TextEncoder(), out = {};
  for (const k in map) out[k] = enc.encode(map[k]);
  return out;
}
const rowFor = (window, key) => [...window.document.querySelectorAll('#panes .row')].find(r => r._key === key);
function edit(window, key, value){
  const i = window.document.getElementById('f-' + key);
  i.value = value; i.dispatchEvent(new window.Event('input'));
}
function showTab(window, path){ window._state.active = path; window.renderApp(); }

// MQTT devices on the stub broker: answer hostname / get-config / any other command.
function mqttDevices(window, devs){
  const orig = window.mqtt.connect;
  window.mqtt.connect = (url, opts) => {
    const c = orig(url, opts), pub = c.publish.bind(c);
    c.publish = (topic, msg, o, cb) => {
      pub(topic, msg, o, cb);
      const m = topic.match(/^pv\/log\/([^/]+)\/cmd$/), conf = m && devs[m[1]];
      if (conf) setTimeout(() => {
        const say = s => c.emit('message', 'pv/log/' + m[1], new TextEncoder().encode(s + '\n'));
        let g;
        if (msg === 'hostname') say('Hostname: ' + m[1]);
        else if ((g = String(msg).match(/^get-config (\S+)$/))){
          for (const [k, v] of Object.entries(conf[g[1]] || {})) say(`Conf '/littlefs/conf/${g[1]}:${k}' = '${v}'`);
          say('OK: get-config ' + g[1]);
        } else say('OK: ' + msg);
      }, 1);
      return c;
    };
    return c;
  };
  window.document.getElementById('mqtt-url').value = 'ws://broker:9001';
}
const allPublishes = window => (window._mqttClients || []).flatMap(c => c._publishes);

test('failed MQTT switch from devA to devB: state, header and push stay on devA', async () => {
  const { window } = await loadEditor();
  mqttDevices(window, { devA: { 'charger.conf': { vout_max: '28.8', cv_eoc: '3.55' } }, devB: {} });
  await window.mqttConnectAndImport('devA');
  assert.equal(window._state.deviceName, 'devA');
  showTab(window, 'conf/charger.conf');
  edit(window, 'vout_max', '29.2');

  let asked = 0;
  window.confirm = () => { asked++; return true; };
  await window.mqttConnectAndImport('devB');      // devB answers but has no config → import fails
  assert.equal(asked, 1, 'switching device with pending edits asks first');
  assert.match(window.document.getElementById('serial-status').textContent, /MQTT: no config received/);
  assert.equal(window._state.deviceName, 'devA');
  assert.equal(window.pendingCount(), 1, 'edits survive the failed switch');
  assert.equal(window.document.getElementById('conn').textContent, 'Disconnected from devA over MQTT');

  window.openReview();
  const up = window.document.getElementById('upload');
  assert.notEqual(up.style.display, 'none');
  assert.equal(up.textContent, 'Push 1 change to devA over MQTT');

  const before = allPublishes(window).length;
  await window.uploadChanged({ reviewed: true });
  const pubs = allPublishes(window).slice(before);
  assert.ok(pubs.every(p => !p.topic.includes('devB')), JSON.stringify(pubs));
  assert.ok(pubs.some(p => p.topic === 'pv/log/devA/cmd' && p.msg === 'set-config charger.conf vout_max 29.2'));
  assert.equal(window.pendingCount(), 0);
});

test('failed BLE connect, then a preset: push is not offered and nothing is sent', async () => {
  const { window } = await loadEditor();
  const writes = [];
  const chr = {
    async startNotifications(){ throw new Error('notify failed (mock)'); },
    async stopNotifications(){}, addEventListener(){}, removeEventListener(){},
    async writeValueWithResponse(d){ writes.push(new TextDecoder().decode(d)); },
  };
  const dev = { name: 'devB', gatt: { disconnect(){}, async connect(){
    return { connected: true, disconnect(){}, async getPrimaryService(){ return { async getCharacteristic(){ return chr; } }; } };
  } } };
  Object.defineProperty(window.navigator, 'bluetooth', { configurable: true, value: { requestDevice: async () => dev } });

  await window.connectBluetooth();
  assert.match(window.document.getElementById('drop-msg').textContent, /^Bluetooth: notify failed/);
  assert.equal(window._lastTransport, null, 'a failed connect commits no transport');

  window.applyUpload(mk({ 'conf/charger.conf': 'vout_max=28.8\n' }), 'preset demo');
  showTab(window, 'conf/charger.conf');
  edit(window, 'vout_max', '14.6');
  assert.equal(window.pushTarget(), null);
  assert.equal(window.document.getElementById('conn').textContent, 'Not connected');

  window.openReview();
  assert.equal(window.document.getElementById('upload').style.display, 'none');
  await window.uploadChanged({ reviewed: true });
  assert.deepEqual(writes, []);
});

test('clicking the key label of a bool row does not toggle it', async () => {
  const { window } = await loadEditor();
  window.load(mk({ 'conf/mqtt.conf': 'enabled=0\nbroker_uri=mqtt://x\n' }), 'demo');
  showTab(window, 'conf/mqtt.conf');
  const row = rowFor(window, 'enabled');
  const val = () => window._state.files['conf/mqtt.conf'].model.lines.find(l => l.key === 'enabled').value;

  row.querySelector('label.k').click();
  assert.equal(val(), '0');
  assert.equal(window.pendingCount(), 0);

  row.querySelector('.switch').click();              // the switch itself still toggles
  assert.equal(val(), '1');
});

test('review sheet flags an invalid value and the push asks once more', async () => {
  const { window } = await loadEditor();
  mqttDevices(window, { devA: { 'charger.conf': { vout_max: '28.8' } } });
  await window.mqttConnectAndImport('devA');
  showTab(window, 'conf/charger.conf');
  edit(window, 'vout_max', '28,8');

  window.openReview();
  const diff = window.document.querySelector('#review-body .diff');
  assert.ok(diff.classList.contains('invalid'));
  assert.match(diff.querySelector('.bad').textContent, /Not a number/);
  assert.match(window.document.getElementById('review-sub').textContent, /1 invalid/);
  assert.match(window.document.getElementById('review-note').textContent, /invalid/);

  const sent = () => allPublishes(window).filter(p => p.msg.startsWith('set-config'));
  let prompt = '';
  window.confirm = m => { prompt = m; return false; };
  await window.uploadChanged({ reviewed: true });
  assert.match(prompt, /charger\.conf:vout_max = 28,8/);
  assert.equal(sent().length, 0, 'declined → nothing sent');

  window.confirm = () => true;
  await window.uploadChanged({ reviewed: true });
  assert.equal(sent().length, 1);
});

test('valueProblem follows the firmware number parser (src/conf.h)', async () => {
  const { window } = await loadEditor();
  const p = (k, v) => window.valueProblem(k, v);
  // long/byte via strtol_2_8_10_16
  for (const ok of ['0', '39000', '-5', '+7', '-010', '0x1F', '0xff', '0b101', '007', ' 42 '])
    assert.equal(p('pwm_freq', ok), '', ok);
  for (const bad of ['08', '09', '0X10', '-0x10', '0B1', '0x', '0b', '1.5', '1e3', '12a', '2147483648'])
    assert.notEqual(p('pwm_freq', bad), '', bad);
  assert.match(p('pwm_freq', '010'), /octal.*8/);
  assert.equal(p('i2c_port', '255'), '');
  assert.match(p('i2c_port', '256'), /0 to 255/);
  assert.match(p('i2c_port', '-1'), /0 to 255/);
  // float via strtof
  for (const ok of ['0.5', '.5', '5.', '-1e-3', '1E5', 'nan', 'inf', '-Infinity', '08'])
    assert.equal(p('vout_max', ok), '', ok);
  for (const bad of ['28,8', '1e', '1.2.3', '', 'abc', '1e39', '1e-400'].filter(Boolean))
    assert.notEqual(p('vout_max', bad), '', bad);
  assert.equal(p('broker_uri', '08'), '', 'strings are not checked');
});

test('a folded group shows a marker for changed rows inside it', async () => {
  const { window } = await loadEditor();
  window.load(mk({ 'conf/charger.conf': 'vout_max=28.8\ncv_eoc=3.55\ncv_float=3.4\n' }), 'demo');
  showTab(window, 'conf/charger.conf');
  const group = () => [...window.document.querySelectorAll('#panes .rows details.kgroup')]
    .find(g => g.querySelector('summary').textContent.startsWith('cv'));
  const dot = () => group().querySelector(':scope > summary .dot');
  assert.equal(dot().style.display, 'none');

  edit(window, 'cv_eoc', '3.60');
  group().open = false;
  await new Promise(r => setTimeout(r, 10));        // toggle event records the fold
  showTab(window, 'conf/mqtt.conf'); showTab(window, 'conf/charger.conf');
  assert.equal(group().open, false, 'fold survives a re-render');
  assert.notEqual(dot().style.display, 'none');
  assert.equal(dot().textContent, '1');

  edit(window, 'cv_float', 'x');
  assert.equal(dot().textContent, '2, 1 invalid');
});

test('duplicate keys: one row and one id; the firmware\'s last value wins and edits hit every line', async () => {
  const { window } = await loadEditor();
  window.load(mk({ 'conf/board.conf': 'pwm_freq=39000\npwm_freq=40000\n' }), 'demo');
  assert.equal(window.document.querySelectorAll('[id="f-pwm_freq"]').length, 1);
  assert.equal(window.document.getElementById('f-pwm_freq').value, '40000');
  edit(window, 'pwm_freq', '41000');
  assert.equal(window.serializeFile(window._state.files['conf/board.conf']), 'pwm_freq=41000\npwm_freq=41000\n');
});

test('replacing an edited file/preset asks first; declining keeps the edits', async () => {
  const { window } = await loadEditor();
  window.load(mk({ 'conf/board.conf': 'pwm_freq=39000\n' }), 'first');
  edit(window, 'pwm_freq', '40000');
  let asked = 0;
  window.confirm = () => { asked++; return false; };
  window.applyUpload(mk({ 'conf/board.conf': 'pwm_freq=1\n' }), 'second');
  assert.equal(asked, 1);
  assert.equal(window._state.sourceName, 'first');
  assert.equal(window.pendingCount(), 1);

  const ev = new window.Event('beforeunload', { cancelable: true });
  window.dispatchEvent(ev);
  assert.equal(ev.defaultPrevented, true, 'leaving the page with edits is guarded');
});
