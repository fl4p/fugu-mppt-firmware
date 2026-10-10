// Spec §4.1 — side (hv_/lv_) keys; the removed role keys (vin_/vout_/iin_/iout_) are flagged, src/conv_side.h.
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
const tab = (window, label) => [...window.document.querySelectorAll('#tabs .tab')].find(t => t.textContent.startsWith(label));

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

test('removed role keys are described as no longer read, naming both replacements', async () => {
  const { window } = await loadEditor();
  const vmax = window.lookupMeta('conf/limits.conf', 'vout_max', '');
  assert.ok(vmax.removed);
  assert.match(vmax.desc, /^No longer read: role key, migrate to lv_max \(buck\) \/ hv_max \(boost\)/);
  assert.match(window.lookupMeta('conf/sensor.conf', 'iin_factor', '').desc, /hv_i_factor \(buck\) \/ lv_i_factor \(boost\)/);
  assert.equal(window.lookupType('vin_max'), '');
  assert.equal(window.lookupType('iout_ch'), '');
  // charger.conf vout_max is its own key, not a removed one
  assert.ok(!window.lookupMeta('conf/charger.conf', 'vout_max', '').removed);
  assert.equal(window.sideKeyFor('conf/charger.conf', 'vout_max', false), null);
  assert.equal(window.sideKeyFor('conf/limits.conf', 'vin_min', false), null);
  assert.equal(window.sideKeyFor('conf/sensor.conf', 'ntc_ch', false), null);
});

test('sideKeyFor maps role to side by topo', async () => {
  const { window } = await loadEditor();
  const s = (f, k, b) => window.sideKeyFor('conf/' + f, k, b);
  assert.deepEqual(['vin_ch', 'vout_rh', 'iin_factor', 'iout_filt_len'].map(k => s('sensor.conf', k, false)),
                   ['hv_v_ch', 'lv_v_rh', 'hv_i_factor', 'lv_i_filt_len']);
  assert.deepEqual(['vin_ch', 'vout_rh', 'iin_factor', 'iout_filt_len'].map(k => s('sensor.conf', k, true)),
                   ['lv_v_ch', 'hv_v_rh', 'lv_i_factor', 'hv_i_filt_len']);
  assert.deepEqual(['vin_max', 'vout_max', 'iin_max', 'iout_max'].map(k => s('limits.conf', k, false)),
                   ['hv_max', 'lv_max', 'hv_i_max', 'lv_i_max']);
  assert.deepEqual(['vin_max', 'vout_max', 'iin_max', 'iout_max'].map(k => s('limits.conf', k, true)),
                   ['lv_max', 'hv_max', 'lv_i_max', 'hv_i_max']);
});

test('roleKeyError matches the firmware message', async () => {
  const { window } = await loadEditor();
  assert.equal(window.roleKeyError('conf/sensor.conf', ['adc', 'hv_v_rh', 'vout_ch'], 'boost'),
    'sensor.conf: vout_ch is no longer read; with topo=boost use hv_v_ch. Migrate the file with etc/migrate_side_keys.py.');
  assert.match(window.roleKeyError('conf/sensor.conf', ['iout_factor'], 'boost'),
    /use hv_i_factor with the sign flipped \(side factors are positive HV->LV\)/);
  assert.match(window.roleKeyError('conf/limits.conf', ['vin_min', 'iout_max'], 'buck'), /^limits\.conf: iout_max is no longer read; with topo=buck use lv_i_max\./);
  assert.equal(window.roleKeyError('conf/limits.conf', ['hv_max', 'vin_min', 'iout_short', 'p_max'], 'buck'), '');
  assert.equal(window.roleKeyError('conf/charger.conf', ['vout_max'], 'buck'), '');
});

test('a role-keyed file loads unchanged but is flagged for migration, using the topo of its converter.conf', async () => {
  const { window } = await loadEditor();
  const limits = '# legacy\nvin_max=60  # panel\nvout_max=85\nvin_min=10\n';
  window.load(mkFiles({ 'conf/converter.conf': 'topo=boost   # buck, boost\n', 'conf/limits.conf': limits }), 'legacy');
  const st = window._state;
  assert.equal(window.serializeFile(st.files['conf/limits.conf']), limits);   // not rewritten silently
  st.active = 'conf/limits.conf'; window.renderApp();
  assert.match(formErr(window).textContent, /^limits\.conf: vin_max is no longer read; with topo=boost use lv_max\./);
  assert.notEqual(formErr(window).style.display, 'none');
  assert.ok(tab(window, 'limits').classList.contains('rolekey'));
  assert.ok(!tab(window, 'converter').classList.contains('rolekey'));
  const row = [...window.document.querySelectorAll('#panes .row')].find(r => r._key === 'vout_max');
  assert.ok(row.querySelector('.desc').classList.contains('err'));

  window.load(mkFiles({ 'conf/limits.conf': 'hv_max=80\nlv_max=60\nvin_min=10\n' }), 'side');
  window._state.active = 'conf/limits.conf'; window.renderApp();
  assert.equal(formErr(window).textContent, '');
  assert.equal(formErr(window).style.display, 'none');
  assert.ok(!tab(window, 'limits').classList.contains('rolekey'));
});

test('filling a role-key extra raises the error live', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'conf/sensor.conf': 'hv_v_rh=200000\n' }), 'demo');
  window._state.active = 'conf/sensor.conf';
  window._state.files['conf/sensor.conf'].order.push('vin_rl');   // e.g. added via "+ add key" earlier
  window.renderPane();
  assert.equal(formErr(window).textContent, '');
  const row = [...window.document.querySelectorAll('#panes .row')].find(r => r._key === 'vin_rl');
  const inp = row.querySelector('input');
  inp.value = '10000'; inp.dispatchEvent(new window.Event('input'));
  assert.match(formErr(window).textContent, /^sensor\.conf: vin_rl is no longer read; with topo=buck use hv_v_rl/);
});

test('only side keys are offered: placeholders and add-key suggestions', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'conf/board.conf': 'mcu=esp32s3\n' }), 'demo');   // limits.conf synthetic
  window._state.active = 'conf/limits.conf'; window.renderPane();
  const keys = rowKeys(window);
  for (const k of ['hv_max', 'lv_max', 'hv_i_max', 'lv_i_max', 'vin_min', 'iout_short']) assert.ok(keys.includes(k), k);
  for (const k of ['vin_max', 'vout_max', 'iin_max', 'iout_max']) assert.ok(!keys.includes(k), k);

  window._state.active = 'conf/sensor.conf'; window.renderPane();
  const opts = [...window.document.querySelectorAll('#addkey-list option')].map(o => o.value);
  assert.ok(opts.includes('hv_v_rh') && opts.includes('lv_i_factor'));
  assert.ok(!opts.some(k => /^(vin|vout|iin|iout)_/.test(k)));
});
