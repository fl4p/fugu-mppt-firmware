// Spec §4 — type pill + meta lookup (META, FILE_META, channel prefixes, ssid pattern,
// fallback to inline file comment).
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

test('lookupType resolves the four ConfFile::get* buckets and channel suffixes', async () => {
  const { window } = await loadEditor();
  assert.equal(window.lookupType('skip_assert'),     'byte');     // bool/byte
  assert.equal(window.lookupType('pwm_freq'),        'long');
  assert.equal(window.lookupType('L0'),              'float');
  assert.equal(window.lookupType('mcu'),             'string');
  assert.equal(window.lookupType('vin_factor'),      'float');    // channel-suffix table
  assert.equal(window.lookupType('iout_ch'),         'byte');
  assert.equal(window.lookupType('hv_v_rh'),         'float');    // side-named channel
  assert.equal(window.lookupType('lv_i_ch'),         'byte');
  assert.equal(window.lookupType('hv_max'),          'float');
  assert.equal(window.lookupType('ssid_caravan'),    'string');   // wifi pattern
  assert.equal(window.lookupType('not_a_real_key'),  '');         // unknown
});

test('lookupMeta returns META entry with unit/desc and respects per-file override', async () => {
  const { window } = await loadEditor();
  const mcu = window.lookupMeta('conf/board.conf', 'mcu', '');
  assert.ok(mcu, 'expected META[mcu] to be present');
  assert.equal(mcu.type, 'string');

  // FILE_META override: limits.conf and charger.conf give different descriptions
  const vmaxLim = window.lookupMeta('conf/limits.conf',  'vout_max', '');
  const vmaxChg = window.lookupMeta('conf/charger.conf', 'vout_max', '');
  assert.equal(vmaxLim.unit, 'V');
  assert.equal(vmaxChg.unit, 'V');
  assert.notEqual(vmaxLim.desc, vmaxChg.desc);
});

test('channel-prefixed sensor key falls through to CHAN_PREFIX/SUFFIX synthesis', async () => {
  const { window } = await loadEditor();
  const m = window.lookupMeta('conf/sensor.conf', 'vin_rh', '');
  assert.ok(m);
  assert.equal(m.unit, 'Ω');
  assert.match(m.desc, /Vin/);                       // channel prefix injected
});

test('wifi ssid pattern returns string meta; *_psk is the password variant', async () => {
  const { window } = await loadEditor();
  assert.equal(window.lookupMeta('conf/wifi.conf', 'ssid_home',     '').type, 'string');
  assert.match(window.lookupMeta('conf/wifi.conf', 'ssid_home_psk', '').desc, /password/i);
});

test('unknown key with an inline comment falls back to the comment as description', async () => {
  const { window } = await loadEditor();
  const m = window.lookupMeta('conf/whatever.conf', 'totally_unknown', 'set this carefully');
  assert.ok(m);
  assert.equal(m.fromComment, true);
  assert.equal(m.desc, 'set this carefully');
});

test('unknown key without any source returns null (renderer shows the ? warning)', async () => {
  const { window } = await loadEditor();
  assert.equal(window.lookupMeta('conf/whatever.conf', 'unknown_unknown', ''), null);
});

test('lookupDefault returns the firmware-scraped default for known keys', async () => {
  const { window } = await loadEditor();
  // scalar defaults
  assert.equal(window.lookupDefault('conf/board.conf',   'i2c_freq'),         '100000');
  assert.equal(window.lookupDefault('conf/board.conf',   'i2c_sda'),          '255');
  assert.equal(window.lookupDefault('conf/charger.conf', 'cv_eoc'),           '3.6');
  assert.equal(window.lookupDefault('conf/charger.conf', 'recharge_dod'),     '0.2');
  assert.equal(window.lookupDefault('conf/charger.conf', 'recharge_vfloor_band'), '0.05');
  assert.equal(window.lookupDefault('conf/limits.conf',  'temp_max'),         '90.0');
  // string defaults stay quoted so the type is unambiguous
  assert.equal(window.lookupDefault('conf/converter.conf', 'topo'),           '"buck"');
  assert.equal(window.lookupDefault('conf/ble.conf',     'ble_security'),     '"justworks"');
  // channel-suffix table
  assert.equal(window.lookupDefault('conf/sensor.conf',  'vin_factor'),       '1.0');
  assert.equal(window.lookupDefault('conf/sensor.conf',  'iout_midpoint'),    '0.0');
  assert.equal(window.lookupDefault('conf/sensor.conf',  'ntc_filt_len'),     '10');
  // required keys (no default) → undefined
  assert.equal(window.lookupDefault('conf/board.conf',   'pwm_freq'),         undefined);
  assert.equal(window.lookupDefault('conf/coil.conf',    'L0'),               undefined);
  assert.equal(window.lookupDefault('conf/limits.conf',  'vin_max'),          undefined);
});

test('row description renders " · default: <value>" when a default is known', async () => {
  const { window } = await loadEditor();
  const enc = new TextEncoder();
  window.load({ 'config/conf/board.conf': enc.encode('mcu=esp32s3\n') }, 'demo');
  const rows = [...window.document.querySelectorAll('#panes .row')];
  const byKey = Object.fromEntries(rows.map(r => [r._key, r]));
  // has a default → suffix present
  assert.match(byKey.i2c_freq.querySelector('.desc').textContent,    /default:\s*100000/);
  assert.match(byKey.skip_assert.querySelector('.desc').textContent, /default:\s*0/);
  // no default scraped (required by firmware) → no suffix
  assert.doesNotMatch(byKey.pwm_freq.querySelector('.desc').textContent, /default:/);
});
