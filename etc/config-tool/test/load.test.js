// Spec §1 (loading clears prior state) and §2.1 (one tab per known file; synthetic
// tabs for files not present in the source; fixed TAB_ORDER for known files).
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

function mkFiles(map){
  const enc = new TextEncoder(), out = {};
  for (const k in map) out[k] = enc.encode(map[k]);
  return out;
}
function tabLabels(window){
  return [...window.document.querySelectorAll('#tabs .tab')].map(t => t.firstChild.textContent.trim());
}

test('load() reveals the app, sets the source label, and shows a tab per known file', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'config/conf/board.conf': 'mcu=esp32s3\n' }), 'demo');

  assert.equal(window.document.getElementById('app').style.display, 'block');
  assert.equal(window.document.getElementById('drop').style.display, 'none');
  assert.equal(window.document.getElementById('srcname').textContent, 'demo');

  // every key in FILE_KEYS should produce a tab
  const expected = new Set(Object.keys(window._tables.FILE_KEYS).map(b => b.replace(/\.conf$/, '')));
  const got      = new Set(tabLabels(window).map(s => s.replace(/\.conf$/, '')));
  for (const want of expected) assert.ok(got.has(want), 'missing tab for ' + want);
});

test('tab order honours TAB_ORDER (board, sensor, limits, …)', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({
    'config/conf/limits.conf': '',
    'config/conf/board.conf':  '',
    'config/conf/sensor.conf': '',
  }), 'demo');
  const labels = tabLabels(window);
  // first three in fixed order regardless of input order (labels strip ".conf", §2.1)
  assert.deepEqual(labels.slice(0, 3), ['board', 'sensor', 'limits']);
});

test('synthetic tabs are pushed to the right of all real (loaded) tabs', async () => {
  const { window } = await loadEditor();
  // load three files that are NOT first in TAB_ORDER, so a naive sort would
  // interleave coil/converter (synthetic) between them.
  window.load(mkFiles({
    'config/conf/board.conf':   '',
    'config/conf/charger.conf': '',
    'config/conf/wifi.conf':    '',
  }), 'demo');
  const tabs = [...window.document.querySelectorAll('#tabs .tab')];
  const realLabels = tabs.filter(t => !t.classList.contains('new'))
                         .map(t => t.firstChild.textContent.trim());
  const synthIndex = tabs.findIndex(t => t.classList.contains('new'));
  const lastRealIndex = tabs.map((t,i) => t.classList.contains('new') ? -1 : i)
                            .filter(i => i >= 0).pop();
  // every real tab comes before any synthetic tab
  assert.ok(synthIndex > lastRealIndex, 'synthetic tabs should follow all real tabs');
  // real tabs retained TAB_ORDER ranking (labels strip ".conf", §2.1)
  assert.deepEqual(realLabels, ['board', 'charger', 'wifi']);
});

test('synthetic tabs (files absent from the source) get the .new class', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'config/conf/board.conf': 'mcu=esp32s3\n' }), 'demo');
  const tabs = [...window.document.querySelectorAll('#tabs .tab')];
  const board = tabs.find(t => t.firstChild.textContent.trim() === 'board');
  const wifi  = tabs.find(t => t.firstChild.textContent.trim() === 'wifi');
  assert.ok(!board.classList.contains('new'),  'real tab should not be .new');
  assert.ok(wifi.classList.contains('new'),    'synthetic tab should be .new');
});

test('untouched synthetic files are dropped on download', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'config/conf/board.conf': 'mcu=esp32s3\n' }), 'demo');
  // pluck filtered file map the way download() does — drop synthetic & untouched
  const out = {};
  for (const p of Object.keys(window._state.files)) {
    const f = window._state.files[p];
    if (f.isNew && !window.isFileDirty(f)) continue;
    out[p] = p;
  }
  assert.deepEqual(Object.keys(out), ['config/conf/board.conf']);
});

test('loading a second source clears the first', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'a/conf/board.conf': 'mcu=esp32s3\n' }), 'first');
  window.load(mkFiles({ 'b/conf/board.conf': 'mcu=esp32\n'   }), 'second');
  assert.equal(window.document.getElementById('srcname').textContent, 'second');
  // confPaths from "first" must be gone
  for (const p of Object.keys(window._state.files)) assert.ok(!p.startsWith('a/'), p);
});

// §1.1 — after a live device read, an upload overlays its values onto the read
// (for review) instead of discarding it.
test('applyUpload after a device read overlays values, keeping the device baseline', async () => {
  const { window } = await loadEditor();
  // simulate a device read: board.conf present (incl. i2c_sda), limits.conf absent on device
  window.load(mkFiles({ 'conf/board.conf': 'mcu=esp32s3\npwm_freq=39000\ni2c_sda=8\n' }), 'fry');
  window._state.fromDevice = true; window._state.deviceName = 'fry';

  // overlay a backup that changes pwm_freq, omits i2c_sda, and adds limits.conf
  window.applyUpload(mkFiles({
    'conf/board.conf':  'mcu=esp32s3\npwm_freq=40000\n',
    'conf/limits.conf': 'hv_max=85\n',
  }), 'backup-fry');

  const board = window._state.files['conf/board.conf'];
  // device read is NOT discarded: original baseline preserved, overlay applied as a pending edit
  assert.equal(board.original.pwm_freq, '39000', 'device baseline preserved');
  // pwm_freq differs (flagged) and i2c_sda — present on device but omitted by the
  // uploaded board.conf — is cleared (flagged); mcu matches and stays clean.
  // (spread into a test-realm array so deepEqual isn't tripped by cross-realm Array.prototype)
  assert.deepEqual([...window.changedFields(board)].map(c => c.key), ['pwm_freq', 'i2c_sda']);
  const ch = window.changedFields(board).find(c => c.key === 'pwm_freq');
  assert.equal(ch.cur, '40000');
  assert.equal(ch.orig, '39000');
  // the file fully replaces board.conf: the omitted i2c_sda is removed from the
  // model and cleared (will be del-config'd on upload)
  assert.equal(board.model.lines.find(l => l.key === 'i2c_sda'), undefined);
  assert.equal(board.extras.i2c_sda, '');
  const ich = window.changedFields(board).find(c => c.key === 'i2c_sda');
  assert.equal(ich.cur, '');
  assert.equal(ich.orig, '8');

  // a file the device didn't have gets the overlay value as an addition ("was: (not set)")
  const limits = window._state.files['conf/limits.conf'];
  const lch = window.changedFields(limits).find(c => c.key === 'hv_max');
  assert.equal(lch.cur, '85');
  assert.equal(lch.orig, undefined);

  // source pill reflects the overlay
  assert.equal(window.document.getElementById('srcname').textContent, 'fry ← backup-fry');
});

test('applyUpload without a prior device read loads fresh (clears state)', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'a/conf/board.conf': 'mcu=esp32s3\n' }), 'first');
  // no fromDevice flag → applyUpload behaves like load()
  window.applyUpload(mkFiles({ 'b/conf/board.conf': 'mcu=esp32\n' }), 'second');
  assert.equal(window.document.getElementById('srcname').textContent, 'second');
  for (const p of Object.keys(window._state.files)) assert.ok(!p.startsWith('a/'), p);
});

// §2.3 — clicking the "was: …" hint reverts the field to its loaded value.
function rowFor(window, key){
  return [...window.document.querySelectorAll('#panes .row')].find(r => r._key === key);
}
test('clicking "was: …" restores the value the field loaded with', async () => {
  const { window } = await loadEditor();
  window.load(mkFiles({ 'conf/board.conf': 'pwm_freq=39000\n' }), 'demo');

  const row = rowFor(window, 'pwm_freq');
  const inp = row.querySelector('input');
  inp.value = '40000';
  inp.dispatchEvent(new window.Event('input'));

  const board = window._state.files['conf/board.conf'];
  assert.equal(window.changedFields(board).length, 1, 'edit flags the field');
  const orig = row.querySelector('.orig');
  assert.equal(orig.style.display, '', '"was: …" hint is visible');
  assert.ok(orig.textContent.includes('39000'));

  orig.click();  // restore
  assert.equal(window.changedFields(board).length, 0, 'restore clears the change');
  assert.equal(board.model.lines.find(l => l.key === 'pwm_freq').value, '39000');
});
