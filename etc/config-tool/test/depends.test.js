// Spec §2.6.1 (DEPENDS: hidden, faded and required keys, gate-driver block, review warnings).
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

function mk(map){
  const enc = new TextEncoder(), out = {};
  for (const k in map) out[k] = enc.encode(map[k]);
  return out;
}
function edit(window, key, value){
  const i = window.document.getElementById('f-' + key);
  i.value = value; i.dispatchEvent(new window.Event('input'));
}
function showTab(window, path){ window._state.active = path; window.renderApp(); }
const rows = window => [...window.document.querySelectorAll('#panes .row')];
const row = (window, key) => rows(window).find(r => r._key === key);
const shown = window => [...window.document.querySelectorAll('#panes .rows .row')].map(r => r._key);
const dep = (window, key) => { const d = row(window, key).querySelector('.dep'); return d.hidden ? '' : d.textContent; };
const gateKeys = window => [...window.document.querySelectorAll('#panes .gate .row')].map(r => r._key);
const gateNote = window => window.document.querySelector('#panes .gate .gate-note').textContent;
const moreBtn = window => window.document.querySelector('#panes .pane-foot button.more');
function showAll(window){
  const b = moreBtn(window);
  assert.match(b.textContent, /^Show all \d+ keys$/);
  b.click();
}
async function boardWith(conf, converter = ''){
  const { window } = await loadEditor();
  window.load(mk({ 'conf/board.conf': conf, 'conf/converter.conf': converter }), 'demo');
  showTab(window, 'conf/board.conf');
  return window;
}

test('HiLi: gate block with pwm_hi/pwm_li required and pwm_sd optional; InEn pins hidden', async () => {
  const window = await boardWith('pwm_freq=39000\npwm_driver_logic=HiLi\npwm_hi=1\n');
  assert.deepEqual(gateKeys(window), ['pwm_driver_logic', 'pwm_hi', 'pwm_li', 'pwm_sd']);
  assert.equal(gateNote(window), 'Buck (converter.conf topo): pwm_hi switches, pwm_li rectifies.');
  assert.equal(dep(window, 'pwm_li'), 'required for pwm_driver_logic=HiLi');
  assert.equal(dep(window, 'pwm_hi'), '');
  assert.equal(dep(window, 'pwm_sd'), '');
  assert.ok(row(window, 'pwm_li').closest('details.kgroup').open, 'a required key is in sight');
  assert.match(row(window, 'pwm_hi').querySelector('.klabel').textContent, /^HI, high-side input$/);
  assert.match(row(window, 'pwm_sd').querySelector('.klabel').textContent, /optional/);
  for (const k of ['pwm_in', 'pwm_en']) assert.ok(!shown(window).includes(k), k);
  showAll(window);
  for (const k of ['pwm_in', 'pwm_en']) assert.ok(!shown(window).includes(k), k + ' stays hidden under Show all');
  assert.ok(shown(window).includes('pwm_fault_pin'));
});

test('InEn: pwm_in/pwm_en required, a set pwm_hi is faded, unset HiLi pins hidden', async () => {
  const window = await boardWith('pwm_freq=39000\npwm_driver_logic=InEn\npwm_hi=1\n', 'topo=boost\n');
  assert.deepEqual(gateKeys(window), ['pwm_driver_logic', 'pwm_in', 'pwm_en', 'pwm_hi']);
  assert.equal(gateNote(window), 'Boost (converter.conf topo): pwm_en switches, pwm_in rectifies.');
  assert.equal(dep(window, 'pwm_in'), 'required for pwm_driver_logic=InEn');
  assert.equal(dep(window, 'pwm_en'), 'required for pwm_driver_logic=InEn');
  assert.ok(row(window, 'pwm_hi').classList.contains('unused'));
  assert.equal(dep(window, 'pwm_hi'), 'not used with pwm_driver_logic=InEn');
  for (const k of ['pwm_li', 'pwm_sd']) assert.ok(!shown(window).includes(k), k);
});

test('switching the driver logic re-renders at once and opens the group', async () => {
  const window = await boardWith('pwm_freq=39000\npwm_driver_logic=HiLi\npwm_hi=1\npwm_li=2\npwm_in=3\npwm_en=4\n');
  // HiLi complete: the group starts folded, the InEn pins (set) are faded
  assert.equal(row(window, 'pwm_hi').closest('details.kgroup').open, false);
  assert.ok(row(window, 'pwm_in').classList.contains('unused'));
  window.document.getElementById('f-pwm_driver_logic').focus();
  edit(window, 'pwm_driver_logic', 'InEn');
  assert.ok(row(window, 'pwm_in').closest('details.kgroup').open);
  assert.deepEqual(gateKeys(window), ['pwm_driver_logic', 'pwm_in', 'pwm_en', 'pwm_hi', 'pwm_li']);
  assert.ok(!row(window, 'pwm_in').classList.contains('unused'));
  assert.ok(row(window, 'pwm_li').classList.contains('unused'));
  assert.equal(window.document.activeElement.id, 'f-pwm_driver_logic', 'focus stays on the dropdown');
  // a pending deletion of an unused key still shows, faded
  row(window, 'pwm_li').querySelector('.clear-btn').click();
  assert.ok(shown(window).includes('pwm_li'));
  assert.ok(row(window, 'pwm_li').classList.contains('unused'));
});

test('pwm_driver_logic unset or unknown: required, pins left alone', async () => {
  let window = await boardWith('pwm_freq=39000\npwm_hi=1\n');
  assert.equal(dep(window, 'pwm_driver_logic'), 'required: HiLi or InEn');
  assert.ok(shown(window).includes('pwm_driver_logic'));
  assert.ok(!row(window, 'pwm_hi').classList.contains('unused'));
  assert.ok(!shown(window).includes('pwm_li'), 'short view as before');
  showAll(window);
  for (const k of ['pwm_li', 'pwm_in', 'pwm_en', 'pwm_sd']) assert.ok(shown(window).includes(k), k);
  assert.match(gateNote(window), /Set pwm_driver_logic/);

  window = await boardWith('pwm_freq=39000\npwm_driver_logic=Foo\n');
  assert.equal(dep(window, 'pwm_driver_logic'), 'unknown value, required: HiLi or InEn');
});

test('converter.conf mode: psu_* and pv_* keys follow the mode', async () => {
  const { window } = await loadEditor();
  window.load(mk({ 'conf/converter.conf': 'topo=buck\n' }), 'demo');
  showTab(window, 'conf/converter.conf');
  showAll(window);
  for (const k of ['psu_vout', 'pv_isc', 'pv_voc', 'pv_k', 'pv_slew', 'pv_iout_span']) assert.ok(!shown(window).includes(k), k);
  assert.ok(shown(window).includes('psu_slew_down'), 'read in every mode');

  edit(window, 'mode', 'psu');
  assert.ok(shown(window).includes('psu_vout'));
  assert.equal(dep(window, 'psu_vout'), 'required for mode=psu');
  edit(window, 'psu_vout', '24');
  assert.equal(dep(window, 'psu_vout'), '', 'filled in: the note goes');

  edit(window, 'mode', 'pv');
  assert.equal(dep(window, 'psu_vout'), 'not used with mode=pv');
  assert.equal(dep(window, 'pv_isc'), 'required for mode=pv');
  assert.equal(dep(window, 'pv_voc'), 'required for mode=pv');
  assert.equal(dep(window, 'pv_k'), '');
  for (const k of ['pv_k', 'pv_slew', 'pv_iout_span']) assert.ok(shown(window).includes(k), k);
});

test('needs: hs_clamp, bflow_bat and the buck topology', async () => {
  const window = await boardWith('pwm_freq=39000\npwm_driver_logic=HiLi\npwm_hi=1\npwm_li=2\nbflow_bat=0\nhs_clamp=1\nbflow_en=0\n');
  assert.equal(dep(window, 'hs_clamp'), 'needs bflow_bat on (non-zero)');
  window.document.getElementById('f-bflow_bat').click();
  assert.equal(dep(window, 'hs_clamp'), '');
  assert.equal(dep(window, 'bflow_bat'), 'needs bflow_en or bflow_sd set to a pin (non-zero)');
  const input = window.document.getElementById('f-bflow_en');
  edit(window, 'bflow_en', '5');                     // text input: notes refresh in place, no re-render
  assert.equal(dep(window, 'bflow_bat'), '');
  assert.equal(window.document.getElementById('f-bflow_en'), input);

  showTab(window, 'conf/converter.conf'); edit(window, 'topo', 'boost');
  showTab(window, 'conf/board.conf');
  assert.equal(dep(window, 'bflow_bat'), 'needs converter.conf topo=buck');
  assert.equal(dep(window, 'hs_clamp'), 'needs converter.conf topo=buck');
});

test('review sheet warns about combinations that involve a change', async () => {
  const window = await boardWith('pwm_freq=39000\npwm_driver_logic=HiLi\npwm_hi=1\npwm_li=2\nbflow_bat=1\nbflow_en=5\nhs_clamp=0\n');
  const warns = () => [...window.document.querySelectorAll('#review-body .rwarn li')].map(l => l.textContent);
  window.openReview();
  assert.deepEqual(warns(), []);
  window.closeReview();

  showTab(window, 'conf/converter.conf'); edit(window, 'topo', 'boost');   // cross-file
  edit(window, 'mode', 'psu');
  window.openReview();
  assert.deepEqual(warns(), ['board.conf bflow_bat: needs converter.conf topo=buck',
                             'converter.conf psu_vout: required for mode=psu']);
  window.closeReview();

  showTab(window, 'conf/board.conf'); edit(window, 'pwm_driver_logic', 'InEn');
  window.openReview();
  assert.ok(warns().includes('board.conf pwm_in: required for pwm_driver_logic=InEn'));
  assert.ok(warns().includes('board.conf pwm_hi: not used with pwm_driver_logic=InEn'));
});

test('a cross-file `if` rule hides and fades keys in another file', async () => {
  const { window } = await loadEditor();
  window._tables.DEPENDS.push({ file: 'tracker.conf', keys: ['sweep_speed', 'sweep_pace'], if: { file: 'converter.conf', key: 'mode', is: ['mppt'] } });
  window.load(mk({ 'conf/converter.conf': 'mode=mppt\n', 'conf/tracker.conf': 'sweep_speed=4\n' }), 'demo');
  showTab(window, 'conf/tracker.conf'); showAll(window);
  assert.ok(shown(window).includes('sweep_pace'));
  showTab(window, 'conf/converter.conf'); edit(window, 'mode', 'psu');
  showTab(window, 'conf/tracker.conf');
  assert.ok(!shown(window).includes('sweep_pace'));
  assert.equal(dep(window, 'sweep_speed'), 'not used with mode=psu');
});

test('hs_clamp: any non-zero bflow_bat counts as on (getByte)', async () => {
  const window = await boardWith('pwm_freq=39000\nbflow_bat=2\nhs_clamp=1\nbflow_en=5\n');
  assert.equal(dep(window, 'hs_clamp'), '');
  edit(window, 'bflow_bat', '0');
  assert.equal(dep(window, 'hs_clamp'), 'needs bflow_bat on (non-zero)');
});

test('tracker.conf target_duty_cycle in [0,1] overrides mode: psu_vout/pv_* not required', async () => {
  const { window } = await loadEditor();
  window.load(mk({ 'conf/converter.conf': 'topo=buck\nmode=psu\n', 'conf/tracker.conf': 'target_duty_cycle=0.3\n' }), 'demo');
  showTab(window, 'conf/converter.conf');
  assert.ok(!shown(window).includes('psu_vout'), 'unset and unused: hidden');
  window.openReview();
  assert.equal(window.document.querySelectorAll('#review-body .rwarn li').length, 0);
  window.closeReview();
  showTab(window, 'conf/tracker.conf'); edit(window, 'target_duty_cycle', '1.5');   // outside [0,1]: ignored by the firmware
  showTab(window, 'conf/converter.conf');
  assert.equal(dep(window, 'psu_vout'), 'required for mode=psu');
  showTab(window, 'conf/tracker.conf'); edit(window, 'target_duty_cycle', '0');      // explicit 0 = manual duty 0
  showTab(window, 'conf/converter.conf'); edit(window, 'psu_vout', '24');
  assert.equal(dep(window, 'psu_vout'), 'not used: tracker.conf target_duty_cycle=0 overrides mode=psu');
  showTab(window, 'conf/tracker.conf'); edit(window, 'target_duty_cycle', '');       // absent = MPPT: mode counts
  showTab(window, 'conf/converter.conf');
  assert.equal(dep(window, 'psu_vout'), '');
});

test('Show all sits at the bottom of the pane; its count matches the rows it reveals', async () => {
  const window = await boardWith('pwm_freq=39000\npwm_driver_logic=HiLi\npwm_hi=1\npwm_li=2\n');
  const head = window.document.querySelector('#panes .phead');
  assert.equal(head.querySelector('button'), null, 'no toggle in the header');
  assert.match(head.querySelector('.meta').textContent, /, \d+ of \d+ keys set$/);
  const foot = window.document.querySelector('#panes .pane-foot');
  assert.equal(foot.firstElementChild, moreBtn(window), 'first in the footer, before Add a key / Show as text');
  assert.equal(moreBtn(window).tagName, 'BUTTON');
  const n = +moreBtn(window).textContent.match(/\d+/)[0];
  const nAll = window._state.files['conf/board.conf'].order.length;
  assert.ok(n < nAll, 'pwm_in/pwm_en are not used with HiLi and not counted');
  assert.match(moreBtn(window).title, new RegExp(`^${nAll - n} more not used`));
  moreBtn(window).focus();
  moreBtn(window).click();
  assert.equal(shown(window).length, n);
  assert.equal(moreBtn(window).textContent, 'Show fewer keys');
  moreBtn(window).click();
  assert.match(moreBtn(window).textContent, /^Show all/);
});
