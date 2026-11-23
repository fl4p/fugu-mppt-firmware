// Spec §2.2 — rows: input event drives model, "was: …" hint, clear button visibility,
// dirty dot on the tab. §2.3 — clear-button removes the line from the model.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

const SRC = {
  'config/conf/board.conf': 'mcu=esp32s3\npwm_freq=39000\n',
};
function mk(map){
  const enc = new TextEncoder(), out = {};
  for (const k in map) out[k] = enc.encode(map[k]);
  return out;
}
function rowFor(window, key){
  return [...window.document.querySelectorAll('#panes .row')].find(r => r._key === key);
}
function activeTabDot(window){
  return window.document.querySelector('#tabs .tab.active .dot');
}

test('editing an input updates the model, shows "was: …" and the dirty dot', async () => {
  const { window } = await loadEditor();
  window.load(mk(SRC), 'demo');

  const row = rowFor(window, 'mcu');
  assert.ok(row, 'mcu row should render on the active board.conf tab');
  const input = row.querySelector('select');
  const orig  = row.querySelector('.orig');

  assert.equal(orig.style.display, 'none', '"was:" hidden before any edit');
  assert.equal(activeTabDot(window).style.display, 'none', 'dot hidden before any edit');

  input.value = 'esp32';
  input.dispatchEvent(new window.Event('input'));

  assert.notEqual(orig.style.display, 'none', '"was:" should appear after edit');
  assert.match(orig.textContent, /was:\s*esp32s3/);
  assert.notEqual(activeTabDot(window).style.display, 'none', 'tab dot should appear');

  // revert → "was:" and dot both clear
  input.value = 'esp32s3';
  input.dispatchEvent(new window.Event('input'));
  assert.equal(orig.style.display, 'none');
  assert.equal(activeTabDot(window).style.display, 'none');
});

test('clear button is visible iff the input has a value', async () => {
  const { window } = await loadEditor();
  window.load(mk(SRC), 'demo');
  const row = rowFor(window, 'mcu');
  const input = row.querySelector('select');
  const clr   = row.querySelector('.clear-btn');

  assert.notEqual(clr.style.display, 'none', 'clear visible for a populated input');

  input.value = '';
  input.dispatchEvent(new window.Event('input'));
  assert.equal(clr.style.display, 'none', 'clear hidden when input is empty');
});

test('changedFields lists del + set commands the upload will send', async () => {
  const { window } = await loadEditor();
  window.load(mk(SRC), 'demo');

  // edit one field and clear another
  const r1 = rowFor(window, 'mcu');
  r1.querySelector('select').value = 'esp32';
  r1.querySelector('select').dispatchEvent(new window.Event('input'));

  const r2 = rowFor(window, 'pwm_freq');
  r2.querySelector('input').value = '';
  r2.querySelector('input').dispatchEvent(new window.Event('input'));

  const f = window._state.files['config/conf/board.conf'];
  const changed = window.changedFields(f);
  const byKey = Object.fromEntries(changed.map(c => [c.key, c]));

  assert.equal(byKey.mcu.cur,      'esp32');
  assert.equal(byKey.mcu.orig,     'esp32s3');
  assert.equal(byKey.pwm_freq.cur, '');       // cleared → emitted as del-config
  assert.equal(byKey.pwm_freq.orig,'39000');
});

test('filling a key the source omitted shows "was: (not set)" and marks it changed', async () => {
  const { window } = await loadEditor();
  window.load(mk(SRC), 'demo');
  // i2c_sda is firmware-known (FILE_KEYS) but absent from the file → empty "extra" row
  const row = rowFor(window, 'i2c_sda');
  assert.ok(row && row.classList.contains('extra'), 'absent firmware key renders as an extra row');
  const input = row.querySelector('input');
  const orig  = row.querySelector('.orig');
  assert.equal(orig.style.display, 'none', '"was:" hidden while the row is unset');
  assert.ok(!row.classList.contains('changed'));

  input.value = '8';
  input.dispatchEvent(new window.Event('input'));

  assert.ok(row.classList.contains('changed'), 'filled absent key highlights as changed (matches the tab dot)');
  assert.match(orig.textContent, /was:\s*\(not set\)/);
  assert.notEqual(activeTabDot(window).style.display, 'none', 'tab dot should appear');
});

test('"<not set>" vs 0: empty input omits the key, explicit 0 keeps the row', async () => {
  const { window } = await loadEditor();
  // load a file where pwm_freq=0 (a real value)
  const src = mk({ 'config/conf/board.conf': 'pwm_freq=0\n' });
  window.load(src, 'demo');

  const f = window._state.files['config/conf/board.conf'];
  // baseline: no changes vs. loaded
  assert.equal(window.isFileDirty(f), false);
  assert.equal(window.serializeFile(f), 'pwm_freq=0\n');

  // clear it → row goes <not set>, file emits without the key
  const r = rowFor(window, 'pwm_freq');
  r.querySelector('.clear-btn').click();
  assert.equal(window.serializeFile(f).includes('pwm_freq'), false);
  // and changedFields shows orig was "0" (proves the "was: 0" guarantee in §2.3)
  const ch = window.changedFields(f).find(c => c.key === 'pwm_freq');
  assert.equal(ch.orig, '0');
});

test('bool keys render as a switch: unset → on → off, × clears it, odd values keep a dropdown', async () => {
  const { window } = await loadEditor();
  window.load(mk({ 'config/conf/board.conf': 'mcu=esp32s3\nhs_clamp=yes\n' }), 'demo');
  const f = window._state.files['config/conf/board.conf'];

  let row = rowFor(window, 'skip_assert');
  let cb = row.querySelector('.switch input');
  assert.ok(cb, 'skip_assert renders a switch');
  assert.ok(row.querySelector('.switch').classList.contains('unset'));
  assert.equal(row.querySelector('.sw-txt').textContent, '<not set>');

  cb.checked = true; cb.dispatchEvent(new window.Event('change'));
  assert.equal(f.extras.skip_assert, '1');
  assert.ok(!row.querySelector('.switch').classList.contains('unset'));
  cb.checked = false; cb.dispatchEvent(new window.Event('change'));
  assert.equal(f.extras.skip_assert, '0');
  assert.match(window.serializeFile(f), /^skip_assert=0$/m);

  row.querySelector('.clear-btn').click();
  row = rowFor(window, 'skip_assert');
  assert.doesNotMatch(window.serializeFile(f), /skip_assert/);
  assert.ok(row.querySelector('.switch').classList.contains('unset'));

  const odd = rowFor(window, 'hs_clamp');
  assert.equal(odd.querySelector('.switch'), null);
  assert.equal(odd.querySelector('select').value, 'yes');
});
