// Spec §4.1 — side (hv_/lv_) vs legacy role (vin_/vout_/iin_/iout_) keys, src/conv_side.h.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

function mkFiles(map){
  const enc = new TextEncoder(), out = {};
  for (const k in map) out[k] = enc.encode(map[k]);
  return out;
}
const rowKeys = window => [...window.document.querySelectorAll('#panes .row')].map(r => r._key);
const formErr = window => window.document.querySelector('#panes .keyform');

test('side keys resolve meta, type and default', async () => {
  const { window } = await loadEditor();
  for (const k of ['hv_max', 'lv_max', 'hv_i_max', 'lv_i_max']) {
    assert.equal(window.lookupType(k), 'float', k);
    assert.ok(window.lookupMeta('conf/limits.conf', k, ''), k);
    assert.equal(window.lookupDefault('conf/limits.conf', k), undefined, k);  // required
  }
  assert.equal(window.lookupMeta('conf/limits.conf', 'hv_i_max', '').unit, 'A');
  const f = window.lookupMeta('conf/sensor.conf', 'lv_i_factor', '');
  assert.equal(f.type, 'float');
  assert.match(f.desc, /LV-side current/);
  assert.equal(window.lookupType('hv_v_ch'), 'byte');
  assert.equal(window.lookupType('lv_v_adc'), 'string');
  assert.equal(window.lookupDefault('conf/sensor.conf', 'hv_i_filt_len'), '10');
});

// KEEP: legacy role keys must keep resolving; old files still use them.
test('legacy role keys still resolve meta, type and default', async () => {
  const { window } = await loadEditor();
  for (const k of ['vin_max', 'vout_max', 'iin_max', 'iout_max']) {
    assert.equal(window.lookupType(k), 'float', k);
    assert.match(window.lookupMeta('conf/limits.conf', k, '').desc, /Legacy role alias/, k);
  }
  assert.equal(window.lookupType('vin_factor'), 'float');
  assert.equal(window.lookupType('iout_ch'), 'byte');
  assert.match(window.lookupMeta('conf/sensor.conf', 'vin_rh', '').desc, /Vin/);
  assert.equal(window.lookupDefault('conf/sensor.conf', 'iout_midpoint'), '0.0');
});

test('keyFormConflict flags a mixed sensor.conf or limits.conf, naming one key of each form', async () => {
  const { window } = await loadEditor();
  assert.deepEqual({ ...window.keyFormConflict('conf/sensor.conf', ['adc', 'hv_v_rh', 'vin_rl', 'lv_i_ch']) },
                   { side: 'hv_v_rh', role: 'vin_rl' });
  assert.deepEqual({ ...window.keyFormConflict('conf/limits.conf', ['iout_max', 'p_max', 'lv_max']) },
                   { side: 'lv_max', role: 'iout_max' });
  assert.equal(window.keyFormError('conf/limits.conf', ['hv_max', 'vout_max']),
    'limits.conf: mixes side key hv_max with role key vout_max; use side keys (hv_/lv_) or role keys throughout, not both');
});

test('keyFormConflict passes pure side, pure role, and keys outside the rule', async () => {
  const { window } = await loadEditor();
  const ok = (file, keys) => assert.equal(window.keyFormConflict(file, keys), null, file + ' ' + keys);
  ok('conf/sensor.conf', ['hv_v_rh', 'hv_v_rl', 'lv_i_factor', 'ntc_ch']);
  ok('conf/sensor.conf', ['vin_rh', 'vout_rl', 'iout_factor', 'ntc_ch']);
  ok('conf/limits.conf', ['hv_max', 'lv_max', 'hv_i_max', 'lv_i_max', 'vin_min', 'iout_short', 'p_max']);
  ok('conf/limits.conf', ['vin_max', 'vout_max', 'iin_max', 'iout_max', 'vin_min']);
  ok('conf/sensor.conf', ['hv_v_rh', 'vin_min']);          // vin_min is no channel key
  ok('conf/charger.conf', ['vout_max', 'hv_max']);         // the rule is per file, charger has none
});

test('a mixed limits.conf shows the firmware error above its rows; a pure one shows none', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'conf/limits.conf': 'hv_max=80\nvout_max=60\nvin_min=10\n' }), 'mixed');
  window._state.active = 'conf/limits.conf'; window.renderPane();
  assert.match(formErr(window).textContent, /mixes side key hv_max with role key vout_max/);
  assert.notEqual(formErr(window).style.display, 'none');

  window.load(mkFiles({ 'conf/limits.conf': 'hv_max=80\nlv_max=60\nvin_min=10\n' }), 'pure');
  window._state.active = 'conf/limits.conf'; window.renderPane();
  assert.equal(formErr(window).textContent, '');
  assert.equal(formErr(window).style.display, 'none');
});

test('filling a role-key extra in a side-keyed file raises the error live', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'conf/sensor.conf': 'hv_v_rh=200000\n' }), 'demo');
  window._state.active = 'conf/sensor.conf';
  window._state.files['conf/sensor.conf'].order.push('vin_rl');   // e.g. added via "+ add key" earlier
  window.renderPane();
  assert.equal(formErr(window).textContent, '');
  const row = [...window.document.querySelectorAll('#panes .row')].find(r => r._key === 'vin_rl');
  const inp = row.querySelector('input');
  inp.value = '10000'; inp.dispatchEvent(new window.Event('input'));
  assert.match(formErr(window).textContent, /^sensor\.conf: mixes side key hv_v_rh with role key vin_rl/);
});

test('side keys are offered by default: empty or side-keyed limits.conf lists no role placeholders', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'conf/board.conf': 'mcu=esp32s3\n' }), 'demo');   // limits.conf synthetic
  window._state.active = 'conf/limits.conf'; window.renderPane();
  const keys = rowKeys(window);
  for (const k of ['hv_max', 'lv_max', 'hv_i_max', 'lv_i_max', 'vin_min', 'iout_short']) assert.ok(keys.includes(k), k);
  for (const k of ['vin_max', 'vout_max', 'iin_max', 'iout_max']) assert.ok(!keys.includes(k), k);

  // the add-key input suggests side channel keys on sensor.conf
  window._state.active = 'conf/sensor.conf'; window.renderPane();
  const opts = [...window.document.querySelectorAll('#addkey-list option')].map(o => o.value);
  assert.ok(opts.includes('hv_v_rh') && opts.includes('lv_i_factor'));
  assert.ok(!opts.some(k => /^(vin|vout|iin|iout)_/.test(k)));
});

// KEEP: dedicated legacy round-trip. A role-keyed file loads, is offered role keys only
// (so filling a placeholder cannot create a mix), and serializes byte-identical.
test('a legacy role-keyed file round-trips unchanged and is offered role placeholders', async () => {
  const { window } = await loadEditor();
  const limits = '# legacy\nvin_max=80  # panel\nvout_max=60\niin_max=20\niout_max=30\nvin_min=10\n';
  const sensor = 'adc=ads\nvin_rh=200000\nvin_rl=10000\niout_factor=-1\niout_midpoint=0.5\n';
  window.load(mkFiles({ 'conf/limits.conf': limits, 'conf/sensor.conf': sensor }), 'legacy');
  const st = window._state;
  assert.equal(window.serializeFile(st.files['conf/limits.conf']), limits);
  assert.equal(window.serializeFile(st.files['conf/sensor.conf']), sensor);

  st.active = 'conf/limits.conf'; window.renderPane();
  const keys = rowKeys(window);
  for (const k of ['vin_max', 'vout_max', 'iin_max', 'iout_max', 'vin_min']) assert.ok(keys.includes(k), k);
  for (const k of ['hv_max', 'lv_max', 'hv_i_max', 'lv_i_max']) assert.ok(!keys.includes(k), k);
  assert.equal(formErr(window).textContent, '');
  assert.equal(window.document.querySelector('#addkey-list'), null);   // all role keys already listed

  st.active = 'conf/sensor.conf'; window.renderPane();
  const opts = [...window.document.querySelectorAll('#addkey-list option')].map(o => o.value);
  assert.ok(opts.includes('vout_rh') && !opts.some(k => /^(hv|lv)_/.test(k)));
});

test('an overlay that adds a role-keyed limits.conf gets role placeholders, not side ones', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'conf/board.conf': 'mcu=esp32s3\n' }), 'fry');
  window._state.fromDevice = true; window._state.deviceName = 'fry';
  window.applyUpload(mkFiles({ 'conf/limits.conf': 'vin_max=85\n' }), 'backup');
  const order = window._state.files['conf/limits.conf'].order;
  assert.ok(order.includes('vin_max') && order.includes('iout_max'));
  assert.ok(!order.includes('hv_max'));
});
