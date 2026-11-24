// Spec §2.15 (tests sheet): gating, the build probe, the Status rows read on open, collapsed action cards,
// confirms (plain and typed device name), refusals, result parsing per test, abort, link drop and timeout
// mid-test, the one-at-a-time guard and the sweep chart.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

const { W, I, vline, BLOCK, HELP, sweepReply, SWEEP, firmware, mqttFirmware } = require('./_tst_firmware');

const CONF = { 'coil.conf': { L0: '50e-6', rect_offset_ns: '500' }, 'charger.conf': { vout_max: '28.8' } };
const $ = (w, sel) => w.document.querySelector(sel);
const card = (w, id) => $(w, `#tst-body .tst[data-tst="${id}"]`);
const row = (w, id) => $(w, `#tst-body .tst-row[data-tst="${id}"]`);
const rowText = (w, id) => row(w, id).querySelector('.tst-val').textContent;
const settle = (ms = 30) => new Promise(r => setTimeout(r, ms));
async function until(pred, ms = 20000) {
  const end = Date.now() + ms;
  while (!pred()) { if (Date.now() > end) throw new Error('timeout waiting'); await settle(5); }
}
const btn = w => w.document.getElementById('open-tests');
const resText = (w, id) => card(w, id).querySelector('.tst-res').textContent;
// rendered: no [hidden] ancestor, and not inside a closed <details> other than in its summary
const seen = el => !!el && !el.closest('[hidden]') && !(el.parentElement.closest('details:not([open])') && !el.closest('summary'));
const def = (w, id) => w.tstDefs().find(x => x.id === id);
const fold = (w, id) => card(w, id).querySelector('.tst-x').click();
// everything under the head line: tag, buttons, result, chart, Check, output, details
const below = c => [...c.querySelectorAll('.tst-tag, .tst-ctl, .run, .read, .tst-state, .tst-res, .tst-chart, .tst-log, .tst-out, .tst-det, .recheck')];

async function connected(opts = {}) {
  const { window } = await loadEditor({ teledec: opts.teledec !== false });
  const dev = firmware('fry', { conf: CONF, ...opts });
  // every line the fake emits passes the taps after the page has handled it (the page handles a line synchronously)
  const taps = new Set(), h0 = dev.handler;
  dev.handler = (cmd, s) => h0(cmd, ln => { s(ln); for (const f of [...taps]) f(ln); });
  // resolves right after the line that makes pred() true, before the fake's next timer can move the page on
  dev.when = (pred, ms = 20000) => pred() ? Promise.resolve() : new Promise((res, rej) => {
    const f = () => { if (pred()) { taps.delete(f); clearTimeout(t); res(); } };
    const t = setTimeout(() => { taps.delete(f); rej(new Error('timeout waiting for a line')); }, ms);
    taps.add(f);
  });
  mqttFirmware(window, { fry: dev, other: firmware('other', {}) });
  await window.mqttConnectAndImport('fry');
  // reply deadlines generous for a loaded host; tests of a silent command set their own
  Object.assign(window.tstTimings(), { cmd: 3000, probe: 3000, settle: 20, poll: 5 });
  return { window, dev };
}
// The page's run timeouts count Date.now(): hold it so a short d.timeout cannot end a run while the test looks at it.
function holdClock(window) {
  const now = window.Date.now, t = now.call(window.Date);
  window.Date.now = () => t;
  return () => { window.Date.now = now; };
}
const quick = w => Object.assign(w.tstTimings(), { cmd: 300, probe: 300 });   // "within 0.3 s" notes, fast silent replies
async function openSheet(window) {
  btn(window).click();
  await until(() => $(window, '#tst-body .tst-status'));
}
const idle = w => until(() => !w.tstBusyNow() && !$(w, '#tst-sub').classList.contains('busy'));

test('parsers: help, selftest, backflow verdicts, coil, rect-cal status, sweep-curve, live status line', async () => {
  const { window } = await loadEditor();
  const names = window.parseHelp(['help,?', 'restart,reset,reboot', 'measure-coil <value> <value> ...', 'fan <...>', 'V=1/2 I=1/1A 1W', 'OK: help']);
  assert.ok(names.has('help') && names.has('reboot') && names.has('measure-coil') && names.has('fan'));
  assert.ok(!names.has('OK:') && !names.has('V=1/2'));
  assert.equal(window.parseHelp(['restart', 'OK: help']), null, 'without its first line the list is not trusted');

  const s = window.parseSelftest(['hs-short: LATCHED, gates off, idle check on, zero 0.010 A, start blocked',
    'bflow: FAIL LATCHED; last run F/- 0.1 h ago; blocks pass never (unverified)', 'selftest bflow: running']);
  assert.equal(s.hsLatched, true); assert.equal(s.bfLatched, true); assert.equal(s.running, true); assert.equal(s.bfChecked, true);
  assert.equal(window.parseSelftest(['bflow: not checked (board.conf bflow_bat=0)']).bfChecked, false);

  const pass = window.parseBflow(['selftest bflow blocks: PASS I=0.40 A (min 0.05) Vin=36.10 Vout=26.80 D=0.7100 dV=1.17',
    'selftest bflow conducts: PASS I=-0.62 A (min -0.62) Vin=36.10 Vout=26.80 D=0.7200 dV=0.80 (reverse current)']);
  assert.equal(pass.cls, 'pass'); assert.match(pass.text, /PASS \(blocks, conducts\)/);
  const tog = window.parseBflow(['selftest bflow toggle (battery): PASS I on 1.20, off 0.10 A, min step 0.85 V Vin=36.10 Vout=26.80 D=0.7400']);
  assert.equal(tog.cls, 'pass'); assert.match(tog.text, /toggle \(battery\)/);
  const fail = window.parseBflow(['selftest bflow blocks: FAIL I=-0.45 A (min -0.52) Vin=36.10 Vout=26.80 D=0.7100 dV=1.17 (reverse current)']);
  assert.equal(fail.cls, 'fail'); assert.match(fail.text, /FAIL in blocks/);
  const ab = window.parseBflow(['selftest bflow toggle: ABORTED', 'selftest bflow: Vin dropped']);
  assert.equal(ab.cls, 'warn'); assert.equal(ab.text, 'No verdict: toggle check stopped');
  assert.deepEqual([...ab.details], ['Toggle: stopped', 'The input voltage dropped during the check (less light).']);
  assert.equal(window.parseBflow(['selftest bflow: started, usually 10-30 s']), null, 'the start line is no result');

  const c = window.parseCoil(['measure-coil: L = 50.34 uH  (median of 9 DCM pts, Iout>=0.15A; ring not resolved: too few points, expect up to +-5%)',
    'measure-coil: Iout zero +0.003/+0.030 A (off, before/after), subtracted; DISTURBED: moved > 0.02 A, L not for coil.conf']);
  assert.equal(c.L, '50.34'); assert.equal(c.disturbed, true); assert.match(c.kind, /median/);
  assert.equal(window.parseCoil(['  converter disabled (protection?); aborting']).bad, 'converter disabled (protection?); aborting');

  const r = window.parseRectStatus([I('rectcal', 'state=failed: not in DCM auto=1 needed=1 drv=ledc offset=-3 ct (-75 ns, uncalibrated) backstop=2 unsaved=0')]);
  assert.equal(r.state, 'failed'); assert.equal(r.why, 'not in DCM'); assert.equal(r.ns, '-75'); assert.equal(r.cal, 'uncalibrated');
  assert.equal(window.parseCurve(['sc: none']).none, true);
  assert.equal(window.parseCurve(['sc: t=1700000000 n=87 mpp=312.4,35.2 sweeping=0 flag=light land=250.0']).flag, 'light');
  const live = window.tstLive(['x', vline(115.5), 'OK: selftest']);
  assert.equal(live.p, 115.5); assert.equal(live.vin, 36.1); assert.equal(live.mode, 'MPPT↑');
});

test('gating: no tests without a read, or after a failed switch to another device', async () => {
  const { window } = await loadEditor();
  assert.equal(btn(window).disabled, true, 'nothing loaded');
  const fry = firmware('fry', { conf: CONF }), other = firmware('other', {});
  mqttFirmware(window, { fry, other });
  await window.mqttConnectAndImport('fry');
  assert.equal(btn(window).disabled, false);
  await window.mqttConnectAndImport('other');          // answers, no config → import fails
  assert.equal(window._state.deviceName, 'fry');
  assert.equal(btn(window).disabled, true, 'fry link closed, other was never read');
  const a = fry.sent.length, b = other.sent.length;
  window.tstOpen();
  assert.equal($(window, '#tests').classList.contains('open'), false);
  await window.tstStart('bflow'); await window.tstRead('selftest'); await settle();
  assert.equal(fry.sent.length, a); assert.equal(other.sent.length, b);
});

test('opening reads the build, then the Status rows, and nothing else', async () => {
  const { window, dev } = await connected({ coil: true });
  window.confirm = () => { throw new Error('read-only commands must not confirm'); };
  window.prompt = window.confirm;
  const from = dev.sent.length;
  await openSheet(window);
  await idle(window);
  assert.deepEqual(dev.sent.slice(from), ['help', 'selftest', 'rect-cal status', 'wire', 'adc-cal']);
  assert.match($(window, '.tst-ago').textContent, /^read just now$/);
});

test('opening probes measure-coil when help lacks it; absent tests are hidden', async () => {
  const { window, dev } = await connected();
  const from = dev.sent.length;
  await openSheet(window);
  await idle(window);
  assert.deepEqual(dev.sent.slice(from), ['help', 'measure-coil', 'selftest', 'rect-cal status', 'wire', 'adc-cal']);
  assert.equal(card(window, 'coil'), null); assert.equal(card(window, 'coills'), null);
  assert.ok(row(window, 'adccal'), 'adc-cal is in help');
  assert.match($(window, '#tst-body .tst-absent').textContent, /measure-coil \(CONFIG_FUGU_WITH_MEASURE_COIL\)/);
  for (const g of ['Status', 'Measurements', 'Self-tests', 'Calibration'])
    assert.ok([...window.document.querySelectorAll('#tst-body .tst-group')].some(h => h.textContent === g), g);
  assert.equal($(window, '#tst-sub').textContent, 'fry over MQTT');
  assert.equal(window.document.activeElement.id, 'tst-close');
});

test('measure-coil found by the bare probe when help omits it; adc-cal hidden when help lacks it', async () => {
  const { window, dev } = await connected({ coil: true, coilInHelp: false, absent: ['adc-cal'] });
  await openSheet(window); await idle(window);
  assert.ok(dev.sent.includes('measure-coil'));
  assert.ok(card(window, 'coil')); assert.ok(card(window, 'coills'));
  assert.equal(row(window, 'adccal'), null);
  assert.ok(!dev.sent.includes('adc-cal'), 'a row the build lacks is not read');
  assert.match($(window, '#tst-body .tst-absent').textContent, /adc-cal \(ESP32-S3 builds\)/);
});

test('Status rows: one parsed line each, no buttons, the raw output on click, a quiet refresh', async () => {
  const { window, dev } = await connected();
  await openSheet(window); await idle(window);
  for (const [id, re] of [['selftest', /^OK, nothing latched$/], ['rcstatus', /^offset 500 ns, idle$/],
    ['wire', /^on, baseline 15 mΩ$/], ['adccal', /^eFuse version 1$/]]) {
    assert.match(rowText(window, id), re, id);
    assert.equal(row(window, id).classList.contains('bad'), false, id + ': no dot when all is well');
    assert.equal(row(window, id).querySelector('button'), null, id + ': no per-row button');
  }
  assert.equal(row(window, 'selftest').querySelector('.tst-lbl').textContent, 'Protection');
  assert.doesNotMatch($(window, '#tst-body .tst-status').textContent, /Takes|Sends|Read\b/);
  assert.doesNotMatch($(window, '#tst-body').textContent, /read-only/);
  const r = row(window, 'wire');
  assert.equal(r.open, false);
  r.querySelector('summary').click();
  assert.equal(r.open, true);
  assert.match(r.querySelector('.tst-out').textContent, /wire: armed, level ok.*baseline: 12 days/);
  window.confirm = () => { throw new Error('read-only commands must not confirm'); };
  window.prompt = window.confirm;
  const from = dev.sent.length;
  $(window, '#tst-refresh').click();
  await idle(window);
  assert.deepEqual(dev.sent.slice(from), ['help', 'measure-coil', 'selftest', 'rect-cal status', 'wire', 'adc-cal']);
});

test('Status rows: amber dot on a latched fault, a failed calibration and a refusal', async () => {
  const { window } = await connected({ state: { hsLatched: true, rect: { state: 'failed', why: 'not in DCM', ct: -3, ns: '-75', unsaved: 0 } } });
  await openSheet(window); await idle(window);
  assert.ok(row(window, 'selftest').classList.contains('bad'));
  assert.equal(rowText(window, 'selftest'), 'High-side short latched: starts blocked');
  assert.ok(row(window, 'rcstatus').classList.contains('bad'));
  assert.equal(rowText(window, 'rcstatus'), 'offset -75 ns, failed: not in DCM');
  assert.equal(row(window, 'wire').classList.contains('bad'), false);
  assert.equal(card(window, 'clear').querySelector('.run').disabled, false, 'clear offered for the latch');
});

test('Status rows: a dot before each value, green fine, amber attention, red a latch, grey unknown, none for ADC calibration', async () => {
  const dot = (w, id) => row(w, id).querySelector('.tst-dot');
  const lv = (w, id) => [dot(w, id).className.replace('tst-dot', '').trim(), dot(w, id).getAttribute('aria-label')];
  let { window } = await connected({});
  await openSheet(window); await idle(window);
  assert.deepEqual(lv(window, 'selftest'), ['ok', 'OK']);
  assert.deepEqual(lv(window, 'rcstatus'), ['ok', 'OK']);
  assert.deepEqual(lv(window, 'wire'), ['ok', 'OK']);
  assert.deepEqual(lv(window, 'adccal'), ['', null], 'information only');
  assert.equal(dot(window, 'adccal').getAttribute('role'), null);
  assert.equal(dot(window, 'selftest').getAttribute('role'), 'img');
  assert.equal(dot(window, 'selftest').nextElementSibling, row(window, 'selftest').querySelector('.tst-val'), 'before the value');
  ({ window } = await connected({ silent: ['wire'], state: { hsLatched: true, rect: { state: 'idle', ct: 20, ns: '500', unsaved: 0, cal: 'uncalibrated' } } }));
  quick(window);
  await openSheet(window); await idle(window);
  assert.deepEqual(lv(window, 'selftest'), ['fail', 'Fault']);
  assert.ok(row(window, 'selftest').classList.contains('fail'));
  assert.deepEqual(lv(window, 'rcstatus'), ['warn', 'Needs attention']);
  assert.match(rowText(window, 'rcstatus'), /uncalibrated/);
  assert.deepEqual(lv(window, 'wire'), ['warn', 'Needs attention'], 'no reply, with its note');
});

test('Status rows: a silent command shows "no reply" with its note', async () => {
  const { window } = await connected({ silent: ['wire'] });
  quick(window);
  await openSheet(window); await idle(window);
  assert.equal(rowText(window, 'wire'), 'no reply');
  assert.ok(row(window, 'wire').classList.contains('bad'));
  const note = row(window, 'wire').querySelector('.tst-note');
  assert.match(note.textContent, /No reply to “wire” within 0\.3 s/);
  assert.equal(row(window, 'wire').open, true, 'the note unfolds its row once');
  row(window, 'wire').querySelector('summary').click();
  assert.equal(seen(note), false, 'a folded row hides its note and output');
  window.tstRender();
  assert.equal(row(window, 'wire').open, false, 'the fold survives a repaint');
  assert.equal(rowText(window, 'adccal'), 'eFuse version 1', 'the next row is still read');
});

test('action cards: collapsed to title and description; tag, buttons, a run, its result and its output fold with the card', async () => {
  const { window, dev } = await connected({ coil: true });
  await openSheet(window); await idle(window);
  for (const id of ['sweep', 'coil', 'bflow', 'clear', 'rectcal', 'coills']) {
    const c = card(window, id);
    assert.equal(c.classList.contains('open'), false, id);
    assert.equal(c.classList.contains('shown'), false, id + ': nothing to show yet');
    assert.equal(c.querySelector('.tst-more').hidden, true, id + ': folded');
    assert.equal(c.querySelector('.tst-res').hidden, true, id);
    assert.equal(c.querySelector('.tst-x').getAttribute('aria-expanded'), 'false');
    assert.equal(c.querySelector('.tst-x').getAttribute('aria-controls'), c.querySelector('.tst-more').id);
    for (const e of below(c)) assert.ok(c.querySelector('.tst-more').contains(e), id + ': ' + e.className + ' is in the fold');
    assert.ok(c.querySelector('.tst-what').textContent.length <= 70, id + ': short description');
    assert.ok(seen(c.querySelector('.tst-what')) && seen(c.querySelector('.tst-x')), id + ': title and description on the line');
    assert.equal(c.querySelector('.tst-head button:not(.tst-x):not([hidden])'), null, id + ': no buttons on the line');
    assert.equal(c.querySelector('.tst-ti').textContent, def(window, id).title + def(window, id).what, id + ': nothing else on the line');
  }
  const cl = card(window, 'clear');
  assert.equal(cl.querySelector('.tst-state').textContent, 'Nothing latched');
  cl.querySelector('.tst-what').click();
  assert.ok(seen(cl.querySelector('.tst-state')) && seen(cl.querySelector('.run')), 'the reason sits by its button in the fold');
  assert.equal(cl.querySelector('.tst-state').previousElementSibling, cl.querySelector('.run'));
  assert.equal(cl.querySelector('.run').disabled, true);
  fold(window, 'clear');
  const rc = card(window, 'rectcal');
  fold(window, 'rectcal');
  const more = rc.querySelector('.tst-more');
  assert.equal(more.firstElementChild, rc.querySelector('.tst-tag'), 'the tag leads the fold');
  assert.equal(rc.querySelector('.tst-tag').textContent, 'Drives the converter · writes coil.conf');
  assert.deepEqual([...more.children].map(e => e.className.split(' ').pop()), ['tst-tag', 'tst-det', 'tst-ctl', 'tst-prog', 'tst-res', 'tst-log'],
    'tag, characteristics, buttons, progress, result, output');
  for (const id of ['bflow', 'clear']) assert.equal(card(window, id).querySelector('.tst-tag'), null, id + ': no tag');
  assert.deepEqual(['sweep', 'coil', 'coills'].map(id => card(window, id).querySelector('.tst-tag').textContent),
    ['Drives the converter', 'Drives the half-bridge', 'Drives the half-bridge']);
  fold(window, 'rectcal');
  const c = card(window, 'coil');
  c.querySelector('.tst-x').click();
  assert.ok(c.classList.contains('open'));
  assert.equal(c.querySelector('.tst-x').getAttribute('aria-expanded'), 'true');
  assert.equal(c.querySelector('.tst-more').hidden, false);
  assert.ok(seen(c.querySelector('.run')), 'Run in view once unfolded');
  assert.match(c.querySelector('.tst-det').textContent, /Risk.*no abort.*Needs.*Takes.*Sends.*measure-coil l0$/);
  c.querySelector('.tst-what').click();
  assert.equal(c.classList.contains('open'), false, 'a click on the line folds it again');
  c.querySelector('.tst-what').click();
  await idle(window);                                      // unfolding reads coil.conf L0
  window.prompt = () => null;
  c.querySelector('.run').click();                         // a button does not toggle the fold
  await idle(window);
  assert.ok(c.classList.contains('open'), 'a cancelled start leaves the card as it was');
  assert.match(resText(window, 'coil'), /Not started/);
  assert.ok(seen(c.querySelector('.tst-res')));
  fold(window, 'coil');
  assert.equal(seen(c.querySelector('.tst-res')), false);

  window.prompt = () => 'fry';
  const b = card(window, 'bflow');
  const run = window.tstStart('bflow');
  await dev.when(() => window.tstBusyNow() && b.querySelector('.tst-out').textContent.includes('forced PWM'));
  assert.ok(b.classList.contains('open'), 'a run unfolds its card');
  assert.ok(seen(b.querySelector('.tst-res')) && seen(b.querySelector('.tst-out')), 'live output in view');
  assert.equal(b.querySelector('.tst-log').open, true);
  fold(window, 'bflow');
  assert.equal(b.classList.contains('open'), false, 'a running card folds');
  for (const e of below(b)) assert.equal(seen(e), false, 'folded while running: ' + e.className);
  assert.ok(seen(b.querySelector('.abort')), 'Abort stays on the head line');
  assert.ok(b.querySelector('.tst-head').contains(b.querySelector('.abort')));
  assert.equal(b.querySelector('.tst-runs').textContent, 'running …', 'a folded run says so on its line');
  fold(window, 'bflow');
  assert.equal(b.querySelector('.tst-runs').textContent, '', 'no hint while unfolded');
  fold(window, 'bflow');
  await run;
  assert.ok(b.classList.contains('open'), 'the result unfolds the card once');
  assert.match(resText(window, 'bflow'), /PASS/);
  assert.ok(seen(b.querySelector('.tst-res')) && seen(b.querySelector('.tst-det')));
  assert.equal(b.querySelector('.tst-runs').textContent, '');
  assert.equal(seen(b.querySelector('.abort')), false, 'Abort only while running');
  const shown = resText(window, 'bflow');
  fold(window, 'bflow');
  for (const e of below(b)) assert.equal(seen(e), false, 'folded with a result: ' + e.className);
  window.tstRender();
  $(window, '#tst-body')._key = null; window.tstRender();  // rebuild
  const b2 = card(window, 'bflow');
  assert.notEqual(b2, b, 'rebuilt');
  assert.equal(b2.classList.contains('open'), false, 'the fold survives a rebuild');
  assert.equal(b2.querySelector('.tst-x').getAttribute('aria-expanded'), 'false');
  assert.equal(seen(b2.querySelector('.tst-res')), false);
  fold(window, 'bflow');
  assert.ok(seen(b2.querySelector('.tst-res')));
  assert.equal(resText(window, 'bflow'), shown, 'unfolding shows the same result');
  assert.equal($(window, '#tests .sheet-foot').textContent.trim(), '', 'no footer note');
});

test('action cards: Abort on a folded running card; an unknown state unfolds once', async () => {
  const { window, dev } = await connected({ bfNeverEnds: true });
  window.tstDefs().find(d => d.id === 'bflow').timeout = 400;
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  const release = holdClock(window);
  const run = window.tstStart('bflow');
  const b = card(window, 'bflow');
  await until(() => window.tstBusyNow() && b.querySelector('.tst-out').textContent.includes('started'));
  fold(window, 'bflow');
  const from = dev.sent.length;
  b.querySelector('.tst-head .abort').click();
  await until(() => dev.sent.length > from);
  assert.equal(dev.sent[from], 'selftest abort');
  assert.equal(b.classList.contains('open'), false, 'Abort does not unfold');
  release();
  await run;
  assert.ok(b.classList.contains('open'), 'the unknown-state note unfolds the card');
  assert.ok(seen(b.querySelector('.recheck')), 'Check in view');
  fold(window, 'bflow'); window.tstRender();
  assert.equal(seen(b.querySelector('.tst-res .note')), false, 'and folds with it');
});

test('backflow switch test: typed device name, live power in the prompt, live output, PASS from selftest', async () => {
  const { window, dev } = await connected();
  await openSheet(window); await idle(window);
  let prompt = '';
  window.prompt = m => { prompt = m; return 'fr'; };
  const from = dev.sent.length;
  await window.tstStart('bflow');
  assert.match(prompt, /run “selftest bflow” on fry\?/);
  assert.match(prompt, /fry is a live power converter: it is converting 115\.5 W \(Vin 36\.1 V, Vout 26\.8 V, MPPT↑\) now/);
  assert.match(prompt, /conversion stops/);
  assert.match(prompt, /Type the device name \(fry\) to confirm/);
  assert.deepEqual(dev.sent.slice(from), ['selftest'], 'wrong name → only the read-only pre-check was sent');
  assert.match(resText(window, 'bflow'), /Not started/);

  window.prompt = () => ' fry ';
  const run = window.tstStart('bflow');
  await dev.when(() => card(window, 'bflow').querySelector('.tst-out').textContent.includes('forced PWM from'));
  assert.ok(window.tstBusyNow());
  assert.equal(card(window, 'bflow').querySelector('.abort').hidden, false, 'Abort shown while running');
  assert.equal(card(window, 'sweep').querySelector('.run').disabled, true, 'one test at a time');
  await run;
  assert.deepEqual(dev.sent.slice(from + 1), ['selftest', 'selftest bflow', 'selftest']);
  const res = card(window, 'bflow').querySelector('.tst-res');
  assert.ok(res.classList.contains('pass'));
  assert.match(res.textContent, /Backflow switch PASS \(blocks, conducts\)/);
  assert.match(res.textContent, /Conducts: passed \(−0\.62 A reverse current\)/);
  assert.doesNotMatch(res.textContent, /PASS I=/, 'no raw firmware line in the summary');
  assert.equal(card(window, 'bflow').querySelector('.abort').hidden, true);
  assert.equal(window.tstBusyNow(), false);
});

test('backflow switch results in plain words: no verdict with an unreached phase, PASS/PASS, FAIL', async () => {
  const { window } = await loadEditor();
  const nv = window.parseBflow([
    'selftest bflow blocks: PASS I=0.00 A (min -0.05) Vin=68.71 Vout=26.41 D=0.3697 dV=1.00',
    'selftest bflow conducts: ABORTED I=0.00 A (min 0.00) Vin=0.00 Vout=0.00 D=0.0000 dV=0.00',
    'selftest bflow: no CCM forward current at the duty cap']);
  assert.equal(nv.cls, 'warn');
  assert.equal(nv.text, 'No verdict: switch-on check not reached');
  assert.deepEqual([...nv.details], ['Blocks: passed (0.00 A at D 0.370, Vin 68.7 V)', 'Conducts: not reached',
    'The converter stayed in DCM up to the highest duty the check may use, so the switch-on check could not start. Nothing is wrong; it usually passes at lower power or Vin.']);
  assert.doesNotMatch(nv.details.join(' '), /0\.00 V|D 0\.000|Vin=/, 'no zeros of a phase that never measured, no raw lines');

  const pp = window.parseBflow(['selftest bflow blocks: PASS I=0.40 A (min 0.05) Vin=36.10 Vout=26.80 D=0.7100 dV=1.17',
    'selftest bflow conducts: PASS I=-0.51 A (min -0.51) Vin=36.10 Vout=26.80 D=0.7200 dV=0.80 (reverse current)']);
  assert.equal(pp.cls, 'pass'); assert.equal(pp.text, 'Backflow switch PASS (blocks, conducts)');
  assert.deepEqual([...pp.details], ['Blocks: passed (0.40 A at D 0.710, Vin 36.1 V)', 'Conducts: passed (−0.51 A reverse current)']);

  const f = window.parseBflow(['selftest bflow blocks: FAIL I=-0.45 A (min -0.52) Vin=36.10 Vout=26.80 D=0.7100 dV=1.17 (reverse current)']);
  assert.equal(f.cls, 'fail'); assert.match(f.text, /^Backflow switch FAIL in blocks: every start is blocked/);
  assert.deepEqual([...f.details], ['Blocks: FAILED: reverse current −0.45 A through the open switch at D 0.710, Vin 36.1 V, Vout 26.8 V']);
  const fc = window.parseBflow(['selftest bflow blocks: PASS I=0.40 A (min 0.05) Vin=36.10 Vout=26.80 D=0.7100 dV=1.17',
    'selftest bflow conducts: FAIL I=0.30 A (min 0.02) Vin=36.10 Vout=26.80 D=0.6000 dV=1.10']);
  assert.equal(fc.cls, 'fail'); assert.match(fc.text, /FAIL in conducts/);
  assert.equal(fc.details[1], 'Conducts: FAILED: no reverse current (min 0.02 A) through the closed switch at D 0.600, Vin 36.1 V, Vout 26.8 V');

  const unk = window.parseBflow(['selftest bflow blocks: ABORTED I=0.80 A (min 0.10) Vin=36.10 Vout=26.80 D=0.7300 dV=0.45', 'selftest bflow: something new']);
  assert.equal(unk.text, 'No verdict: switch-off check stopped');
  assert.deepEqual([...unk.details], ['Blocks: stopped (0.80 A at D 0.730, Vin 36.1 V)', 'Reason: something new'], 'unknown reasons pass through');
  assert.equal(window.parseBflow(['selftest bflow: take-over timed out']).text, 'No verdict: no check ran');
  const mr = window.parseBflow([...pp.details.length ? ['selftest bflow: mode restore timed out'] : [],
    'selftest bflow blocks: PASS I=0.40 A (min 0.05) Vin=36.10 Vout=26.80 D=0.7100 dV=1.17',
    'selftest bflow conducts: PASS I=-0.51 A (min -0.51) Vin=36.10 Vout=26.80 D=0.7200 dV=0.80 (reverse current)']);
  assert.equal(mr.cls, 'pass'); assert.match(mr.details[2], /not confirmed back/);

  const st = window.tstSummary('selftest', ['hs-short: ok', 'bflow: ok; last run P/A 0.1 h ago; blocks pass 0.1 h ago (verified)',
    'selftest bflow blocks: PASS I=0.00 A (min -0.05) Vin=68.71 Vout=26.41 D=0.3697 dV=1.00',
    'selftest bflow conducts: ABORTED I=0.00 A (min 0.00) Vin=0.00 Vout=0.00 D=0.0000 dV=0.00',
    'selftest bflow: no CCM forward current at the duty cap']);
  assert.ok(st.details.includes('last run blocks: passed (0.00 A at D 0.370, Vin 68.7 V)'));
  assert.ok(st.details.includes('last run conducts: not reached'));
});

test('backflow switch card: a no-verdict run shows plain phase lines, the raw lines stay in the device output', async () => {
  const bfLines = ['selftest bflow blocks: PASS I=0.00 A (min -0.05) Vin=68.71 Vout=26.41 D=0.3697 dV=1.00',
    'selftest bflow conducts: ABORTED I=0.00 A (min 0.00) Vin=0.00 Vout=0.00 D=0.0000 dV=0.00',
    'selftest bflow: no CCM forward current at the duty cap'];
  const { window } = await connected({ bfLines });
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  await window.tstStart('bflow');
  const c = card(window, 'bflow'), res = c.querySelector('.tst-res');
  assert.ok(res.classList.contains('warn'));
  assert.equal(res.querySelector('strong').textContent, 'No verdict: switch-on check not reached');
  assert.match(res.textContent, /Blocks: passed \(0\.00 A at D 0\.370, Vin 68\.7 V\)/);
  assert.match(res.textContent, /Conducts: not reached/);
  assert.match(res.textContent, /stayed in DCM up to the highest duty the check may use/);
  assert.doesNotMatch(res.textContent, /ABORTED|Vin=0\.00/);
  assert.match(c.querySelector('.tst-out').textContent, /conducts: ABORTED I=0\.00 A \(min 0\.00\) Vin=0\.00/);
});

test('a refusal is shown verbatim and nothing else is sent', async () => {
  const { window, dev } = await connected({ bfRefuse: 'Vin too low (needs >= 1.2 Vout and Vout + 3 V)' });
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  await window.tstStart('bflow');
  assert.match(resText(window, 'bflow'), /fry refused “selftest bflow”: selftest bflow: Vin too low \(needs >= 1\.2 Vout and Vout \+ 3 V\)/);
  assert.equal(dev.sent.filter(c => c === 'selftest bflow').length, 1, 'never retried');
  assert.equal(window.tstBusyNow(), false);
});

test('backflow FAIL: amber result, latch shown, clear needs the typed name and clears', async () => {
  const { window, dev } = await connected({ bfFail: true });
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  await window.tstStart('bflow');
  const res = card(window, 'bflow').querySelector('.tst-res');
  assert.ok(res.classList.contains('fail'));
  assert.match(res.textContent, /FAIL in blocks: every start is blocked/);
  assert.match(rowText(window, 'selftest'), /Backflow switch failure latched/);
  assert.ok(row(window, 'selftest').classList.contains('bad'));
  assert.equal(card(window, 'clear').querySelector('.run').disabled, false);

  let prompt = '';
  window.prompt = m => { prompt = m; return 'fry'; };
  await window.tstStart('clear');
  assert.match(prompt, /Latched: backflow switch failure\./);
  assert.match(prompt, /may start again/);
  assert.ok(dev.sent.includes('selftest clear'));
  assert.match(resText(window, 'clear'), /Cleared bf-fail/);
  assert.equal(rowText(window, 'selftest'), 'OK, nothing latched');
  assert.equal(row(window, 'selftest').classList.contains('bad'), false);
  assert.equal(card(window, 'clear').querySelector('.run').disabled, true, 'nothing latched any more');
});

test('clear with nothing latched sends nothing after the status read', async () => {
  const { window, dev } = await connected();
  await openSheet(window); await idle(window);
  window.prompt = () => { throw new Error('no prompt'); };
  await window.tstStart('clear');
  assert.ok(!dev.sent.includes('selftest clear'));
  assert.match(resText(window, 'clear'), /Nothing is latched on fry; nothing was sent/);
});

test('abort: Abort sends selftest abort and the run ends without a verdict', async () => {
  const { window, dev } = await connected({ bfTicks: 1000 });
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  const run = window.tstStart('bflow');
  await until(() => !card(window, 'bflow').querySelector('.abort').hidden);
  card(window, 'bflow').querySelector('.abort').click();
  await run;
  assert.ok(dev.sent.includes('selftest abort'));
  const res = card(window, 'bflow').querySelector('.tst-res');
  assert.ok(res.classList.contains('warn'));
  assert.match(res.textContent, /No verdict: aborted/);
});

test('measure-coil L0: Save L0 to coil.conf only after a value, with the current and measured value', async () => {
  const { window, dev } = await connected({ coil: true, conf: { ...CONF, 'coil.conf': { L0: '5.090000e-05', rect_offset_ns: '500' } } });
  await openSheet(window); await idle(window);
  const c = card(window, 'coil'), sv = c.querySelector('.save'), txt = () => c.querySelector('.tst-l0').textContent;
  assert.equal(c.querySelector('.tst-opt'), null, 'no apply option');
  assert.equal(c.querySelector('.tst-cmd').textContent, 'measure-coil l0');
  assert.equal(c.querySelector('.abort'), null, 'no abort: the firmware has none');
  assert.equal(sv.textContent, 'Save L0 to coil.conf');
  assert.equal(sv.disabled, true); assert.equal(sv.title, 'Run the measurement first');
  let from = dev.sent.length;
  fold(window, 'coil'); await idle(window);
  assert.deepEqual(dev.sent.slice(from), ['get-config coil.conf'], 'unfolding reads the current value');
  assert.equal(txt(), 'Current L0: 50.9 µH');
  fold(window, 'coil'); fold(window, 'coil'); await idle(window);
  assert.equal(dev.sent.length, from + 1, 'read once');
  let prompt = '';
  window.confirm = m => { prompt = m; return true; };
  await window.tstStart('coil');
  assert.match(prompt, /There is no abort command/);
  assert.match(prompt, /Nothing is written/);
  assert.ok(!dev.sent.some(x => /apply|set-config/.test(x)), 'the run writes nothing');
  const res = c.querySelector('.tst-res');
  assert.ok(res.classList.contains('pass'));
  assert.match(res.textContent, /L = 50\.34 µH/);
  assert.match(res.textContent, /ring fit, ±0\.4 % noise, ±1\.1 % syst/);
  assert.equal(sv.disabled, false);
  assert.equal(txt(), 'Current L0: 50.9 µH · Measured 50.34 µH (−1.1 %)');
  window.tstLock({ id: 'sweep', cmd: 'x', link: window.tstTarget() }); window.tstRender();
  assert.equal(sv.disabled, true, 'not while the device is busy');
  window.tstLock(null); window.tstRender();
  from = dev.sent.length;
  sv.click(); await idle(window);
  assert.deepEqual(dev.sent.slice(from), ['set-config coil.conf L0 5.034000e-05', 'get-config coil.conf']);
  assert.equal(txt(), 'Current L0: 50.34 µH · Measured 50.34 µH (+0.0 %)', 'refreshed from the device');
  assert.match(res.textContent, /Saved L0 to coil\.conf; it takes effect after a restart\./);
  assert.equal(sv.disabled, true); assert.equal(sv.title, 'Saved');
  assert.equal(window._state.files['conf/coil.conf'].original.L0, '5.034000e-05', 'the loaded coil.conf follows');
  assert.equal(window.pendingCount(), 0);
});

test('measure-coil L0: no value (failure or disturbed zero) keeps Save disabled; a refused save is amber', async () => {
  let { window, dev } = await connected({ coil: true, coilFail: true });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('coil');
  let sv = card(window, 'coil').querySelector('.save');
  assert.equal(sv.disabled, true); assert.equal(sv.title, 'This run gave no value for coil.conf');
  const l0 = window.parseCoil(['measure-coil: L = 50.34 uH  (ring fit, 31 DCM pts)', '  -> coil.conf L0=5.034000e-05   note: …']).l0;
  assert.equal(l0, '5.034000e-05');
  assert.equal(window.parseCoil(['measure-coil: L = 50.34 uH  (ring fit, 31 DCM pts)',
    'measure-coil: Iout zero +0.003/+0.030 A (off, before/after), subtracted; DISTURBED: moved > 0.02 A, L not for coil.conf']).l0, null);
  ({ window, dev } = await connected({ coil: true, setRefuse: 'cannot write /littlefs/conf/coil.conf' }));
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('coil');
  sv = card(window, 'coil').querySelector('.save');
  sv.click(); await idle(window);
  const note = card(window, 'coil').querySelector('.tst-res .note');
  assert.match(note.textContent, /“set-config coil\.conf L0 5\.034000e-05” refused: set-config: cannot write/);
  assert.ok(note.classList.contains('err'));
  assert.equal(sv.disabled, false, 'can be retried');
});

test('measure-coil failure and the LS sweep', async () => {
  let { window } = await connected({ coil: true, coilFail: true });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('coil');
  assert.ok(card(window, 'coil').querySelector('.tst-res').classList.contains('fail'));
  assert.match(resText(window, 'coil'), /No result: need Vin>Vout\+1 \(Vin=12\.00 Vout=26\.80\)/);

  ({ window } = await connected({ coil: true }));
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  const cb = card(window, 'coills').querySelector('.tst-opt input');
  cb.checked = true; cb.dispatchEvent(new window.Event('change'));
  await window.tstStart('coills');
  assert.match(resText(window, 'coills'), /Low-side peak at 151 ct, \+12 ct from ideal/);
  assert.match(resText(window, 'coills'), /L ≈ 49\.8 µH/);
  assert.match(resText(window, 'coills'), /applied rect_offset 300 ns \(0 ct, mcpwm\).*Read the device again/);
});

test('rect-cal: plain confirm names the device; result from the done line and rect-cal status; force needs the typed name', async () => {
  const { window, dev } = await connected();
  await openSheet(window); await idle(window);
  let msg = '';
  window.confirm = m => { msg = m; return true; };
  window.prompt = () => { throw new Error('plain rect-cal must not ask for the name'); };
  await window.tstStart('rectcal');
  assert.match(msg, /^Low-side timing \(rect-cal\): run “rect-cal” on fry\?/);
  assert.match(msg, /fry is a live power converter: it is converting 115\.5 W/);
  assert.match(msg, /reduced operating point/);
  assert.deepEqual(dev.sent.slice(-3), ['selftest', 'rect-cal', 'rect-cal status']);
  assert.match(resText(window, 'rectcal'), /rect_offset 875 ns \(35 ct, mcpwm\).*saved to coil\.conf once the converter has been off.*Read the device again/);

  const cb = card(window, 'rectcal').querySelector('.tst-opt input');
  cb.checked = true; cb.dispatchEvent(new window.Event('change'));
  let asked = '';
  window.prompt = m => { asked = m; return null; };
  await window.tstStart('rectcal');
  assert.match(asked, /run “rect-cal force” on fry/);
  assert.match(asked, /including full power/);
  assert.ok(!dev.sent.includes('rect-cal force'), 'cancelled → not sent');
});

test('rect-cal refusal and abort', async () => {
  let { window, dev } = await connected({ rcRefuse: 'not in DCM' });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('rectcal');
  assert.match(resText(window, 'rectcal'), /fry refused “rect-cal”: rect-cal: refused: not in DCM/);

  ({ window, dev } = await connected({ rcHold: true }));      // rect-cal ends only on its abort
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  const run = window.tstStart('rectcal');
  await until(() => !card(window, 'rectcal').querySelector('.abort').hidden);
  card(window, 'rectcal').querySelector('.abort').click();
  await run;
  assert.ok(dev.sent.includes('rect-cal abort'));
  assert.ok(card(window, 'rectcal').querySelector('.tst-res').classList.contains('fail'));
  assert.match(resText(window, 'rectcal'), /Failed: aborted/);
});

test('global sweep: confirm, MPP from sweep-curve; no MPP reported in amber', async () => {
  let { window, dev } = await connected({ power: 0.4 });
  await openSheet(window); await idle(window);
  let msg = '';
  window.confirm = m => { msg = m; return true; };
  await window.tstStart('sweep');
  assert.match(msg, /fry is not converting now: 0\.4 W/);
  assert.deepEqual(dev.sent.slice(-3), ['selftest', 'sweep', 'sweep-curve']);
  assert.match(resText(window, 'sweep'), new RegExp(`MPP ${SWEEP.P} W at ${SWEEP.V} V`));
  assert.equal(card(window, 'sweep').querySelector('.tst-chart svg').getAttribute('data-points'), '87');

  ({ window } = await connected({ noMpp: true }));
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('sweep');
  assert.ok(card(window, 'sweep').querySelector('.tst-res').classList.contains('warn'));
  assert.match(resText(window, 'sweep'), /No MPP found.*best 0\.30W < 1\.0W/);
});

// Night: the firmware queues the sweep until the converter can start (st=START), so the run ends there.
const queuedRe = /Sweep queued: the converter can't start \(the input voltage is too close to the output \(Vin-Vout\), Vin 6\.8 V, Vout 26\.6 V\)/;
const free = (w, id) => !w.tstBusyNow() && !card(w, id).querySelector('.run').disabled;

test('sweep at night: the pre-check says why the converter cannot start and queues on OK', async () => {
  const { window, dev } = await connected({ power: 0, mode: 'START', night: 'new' });
  await openSheet(window); await idle(window);
  window._dispatchLine(I('mppt', 'START blocked: Vin-Vout (Vin=6.8 Vout=26.6 ntc=18 mcu=28)'));
  let msg = '';
  window.confirm = m => { msg = m; return false; };
  await window.tstStart('sweep');
  assert.match(msg, /The converter can't start now: the input voltage is too close to the output \(Vin-Vout\), Vin 6\.8 V, Vout 26\.6 V\. A sweep waits until it can start, then runs\. OK queues it anyway\./);
  assert.equal(dev.sent.at(-1), 'selftest', 'nothing sent on Cancel');
  assert.match(card(window, 'sweep').querySelector('.tst-res').textContent, /Not queued: the converter can't start now \(.*Vin-Vout/);
  assert.ok(free(window, 'sweep'), 'lock released');

  window.confirm = () => true;
  const t0 = Date.now();
  await window.tstStart('sweep');
  assert.ok(Date.now() - t0 < 2000, 'ends at the queued reply, not the timeout');
  assert.equal(dev.sent.at(-1), 'sweep', 'no sweep-curve for a queued sweep');
  assert.match(resText(window, 'sweep'), queuedRe);
  assert.match(resText(window, 'sweep'), /It runs when it can; the chart updates after that/);
  assert.ok(!card(window, 'sweep').querySelector('.tst-res').classList.contains('fail'));
  assert.equal(card(window, 'sweep').querySelector('.tst-chart svg'), null, 'no empty chart');
  assert.ok(free(window, 'sweep'), 'lock released');
});

test('sweep pre-check: reason from the status line alone, UV, and no warning for the idle profile', async () => {
  let { window } = await connected({ power: 0, mode: 'START', night: 'new' });
  await openSheet(window); await idle(window);
  let msg = '';
  window.confirm = m => { msg = m; return false; };
  await window.tstStart('sweep');
  assert.match(msg, /can't start now: no reason reported, Vin 36\.1 V, Vout 26\.8 V/);

  ({ window } = await connected({ power: 0, mode: '   UV', night: 'new' }));
  await openSheet(window); await idle(window);
  window.confirm = m => { msg = m; return false; };
  await window.tstStart('sweep');
  assert.match(msg, /can't start now: board supply under-voltage \(supply-UV\)/);

  ({ window } = await connected({ power: 0, mode: 'START' }));
  await openSheet(window); await idle(window);
  window._dispatchLine(I('mppt', 'START blocked: idle profile (Vin=36.1 Vout=26.8 ntc=18 mcu=28)'));
  window.confirm = m => { msg = m; return true; };
  await window.tstStart('sweep');
  assert.doesNotMatch(msg, /can't start/);
  assert.match(resText(window, 'sweep'), new RegExp(`MPP ${SWEEP.P} W at ${SWEEP.V} V`));
});

test('sweep at night on older firmware: a START blocked line or st=START ends the run as queued', async () => {
  let { window, dev } = await connected({ power: 0, mode: 'START', night: 'old' });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('sweep');
  assert.equal(dev.sent.at(-1), 'sweep');
  assert.match(resText(window, 'sweep'), queuedRe);
  assert.ok(free(window, 'sweep'));

  // no START blocked line: the status line still in START after the zero calibration
  ({ window, dev } = await connected({ power: 0, night: 'status' }));
  window.tstTimings().cal = 25;
  // the START status line comes cal ms after the calibration on the page's clock, however late the fake's timers fire
  const h = dev.handler;
  let calDone = false;
  dev.handler = (cmd, s) => h(cmd, ln => {
    if (/\bCalibration done\b/.test(ln)) calDone = true;
    else if (calDone && /\bst=START\b/.test(ln)) {
      calDone = false;
      const now = window.Date.now, cal = window.tstTimings().cal;
      window.Date.now = () => now.call(window.Date) + cal;
    }
    s(ln);
  });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('sweep');
  assert.equal(dev.sent.at(-1), 'sweep');
  assert.match(resText(window, 'sweep'), /Sweep queued: the converter can't start \(no reason reported, Vin 36\.1 V, Vout 26\.8 V\)/);
  assert.ok(free(window, 'sweep'));
});

test('a normal sweep ignores st=START during the zero calibration and ends on Stop sweep', async () => {
  const stop = 'Stop sweep 3.52s mode=CC (lim=3 tgt=2.01 act=2.01) PWM=1164 MPP=(54.8W,1164,66.8V)';
  const { window, dev } = await connected({ sweepStop: stop });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('sweep');
  assert.deepEqual(dev.sent.slice(-2), ['sweep', 'sweep-curve']);
  assert.match(resText(window, 'sweep'), /Stop point .* W at .* V/);
  assert.doesNotMatch(resText(window, 'sweep'), /queued/);
});

test('link drop mid-test: state unknown, no result claimed, a check is offered', async () => {
  const { window } = await connected({ bfNeverEnds: true });
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  const run = window.tstStart('bflow');
  await until(() => window.tstBusyNow() && card(window, 'bflow').querySelector('.tst-out').textContent.includes('started'));
  await window.disconnectMqtt(); window.refreshConn();
  await run;
  const res = card(window, 'bflow').querySelector('.tst-res');
  assert.match(res.textContent, /link to fry closed during the test: its state is unknown/);
  assert.doesNotMatch(res.textContent, /PASS|FAIL/);
  assert.ok(!res.classList.contains('pass') && !res.classList.contains('fail'));
  const chk = res.querySelector('.recheck');
  assert.ok(chk); assert.equal(chk.textContent, 'Check (selftest)'); assert.equal(chk.disabled, true, 'no link to check over');
  assert.equal(window.tstBusyNow(), false);
  assert.equal(btn(window).disabled, true);
});

test('timeout: state unknown, Check reads selftest and reports it without claiming a verdict', async () => {
  const { window, dev } = await connected({ bfNeverEnds: true });
  window.tstDefs().find(d => d.id === 'bflow').timeout = 60;
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  await window.tstStart('bflow');
  assert.match(resText(window, 'bflow'), /No end of the test within 0 s: its state is unknown/);
  const from = dev.sent.length;
  card(window, 'bflow').querySelector('.recheck').click();
  await idle(window);
  assert.deepEqual(dev.sent.slice(from), ['selftest']);
  assert.match(resText(window, 'bflow'), /still running on the device/);
});

// measure-coil runs only from `dc 0`: Run stops MPPT itself, retries through the ramp-down and resumes MPPT in every outcome
// stop: the retry deadline after `dc 0`; generous unless the test wants the converter to never stop
async function coilRun({ stop = 10000, ...opts }, id = 'coil') {
  const { window, dev } = await connected({ coil: true, ...opts });
  Object.assign(window.tstTimings(), { stop });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  const busy = [], h = dev.handler;
  const from = dev.sent.length;
  dev.handler = (cmd, s) => { busy.push(window.tstBusyNow()); return h(cmd, s); };
  await window.tstStart(id);
  const c = card(window, id);
  return { window, dev, sent: dev.sent.slice(from), busy, note: c.querySelector('.tst-res .note') };
}

test('measure-coil from MPPT: dc 0, measure, mppt under one lock; Resumed MPPT in the result', async () => {
  const { window, sent, busy, note } = await coilRun({});
  assert.deepEqual(sent, ['selftest', 'measure-coil l0', 'dc 0', 'measure-coil l0', 'mppt']);
  assert.ok(busy.every(Boolean), 'the device lock is held for every command');
  assert.equal(window.tstBusyNow(), false);
  assert.match(resText(window, 'coil'), /L = 50\.34 µH/);
  assert.equal(note.textContent, 'Resumed MPPT.');
  assert.ok(!note.classList.contains('err'));
  const ls = await coilRun({}, 'coills');
  assert.deepEqual(ls.sent, ['selftest', 'measure-coil ls', 'dc 0', 'measure-coil ls', 'mppt']);
});

test('output lines during a run keep the same spinner node, so its animation does not restart', async () => {
  const { window, dev } = await connected({ coil: true });
  Object.assign(window.tstTimings(), { stop: 100 });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  const h = dev.handler, seen = [];
  dev.handler = (cmd, s) => { seen.push([cmd, card(window, 'coil').querySelector('.tst-res .busy')]); return h(cmd, s); };
  await window.tstStart('coil');
  const at = c => seen.filter(x => x[0] === c).map(x => x[1]);
  const measured = at('measure-coil l0').at(-1), resumed = at('mppt')[0];
  assert.ok(measured && resumed, 'a spinner shows while measuring and resuming');
  assert.strictEqual(resumed, measured);
});

test('measure-coil waits out a protection backoff that dc 0 tripped, and ends a persistent one with a sentence', async () => {
  let { window, sent } = await coilRun({ backoff: 2 });
  assert.deepEqual(sent, ['selftest', 'measure-coil l0', 'dc 0', 'measure-coil l0', 'measure-coil l0', 'measure-coil l0', 'mppt']);
  assert.match(resText(window, 'coil'), /L = 50\.34 µH/);
  let note;
  ({ window, sent, note } = await coilRun({ backoff: 1e9, stop: 100 }));
  assert.equal(sent.at(-1), 'mppt');
  assert.match(note.textContent, /stayed in a protection backoff for 0 s after “dc 0”; nothing was measured\. Resumed MPPT\./);
});

test('measure-coil already at dc 0: no dc 0, no mppt afterwards', async () => {
  const { window, sent, note } = await coilRun({ op: 'manu' });
  assert.deepEqual(sent, ['selftest', 'measure-coil l0']);
  assert.match(resText(window, 'coil'), /L = 50\.34 µH/);
  assert.equal(note, null);
});

test('measure-coil retries through the ramp-down; a converter that does not stop ends the run and resumes MPPT', async () => {
  let { window, sent, note } = await coilRun({ rampDown: 2 });
  assert.deepEqual(sent, ['selftest', 'measure-coil l0', 'dc 0', 'measure-coil l0', 'measure-coil l0', 'measure-coil l0', 'mppt']);
  assert.match(resText(window, 'coil'), /L = 50\.34 µH/);
  ({ window, sent, note } = await coilRun({ rampDown: 1e9, stop: 100 }));
  assert.ok(!sent.slice(3, -1).some(c => c !== 'measure-coil l0'));
  assert.equal(sent.at(-1), 'mppt');
  assert.match(note.textContent, /did not stop within 0 s of “dc 0”; nothing was measured\. Resumed MPPT\./);
  assert.ok(note.classList.contains('err'));
});

test('measure-coil: a failed measurement, a refused dc 0 and a timeout still send mppt; a refused resume is loud', async () => {
  let { window, sent, note } = await coilRun({ coilFail: true });
  assert.equal(sent.at(-1), 'mppt');
  assert.match(resText(window, 'coil'), /No result: need Vin>Vout\+1/);
  assert.equal(note.textContent, 'Resumed MPPT.');
  ({ window, sent, note } = await coilRun({ dcRefuse: 'selftest running (mppt aborts it)' }));
  assert.deepEqual(sent, ['selftest', 'measure-coil l0', 'dc 0', 'mppt']);
  assert.match(note.textContent, /“dc 0” refused: dc: selftest running.*nothing was measured\. Resumed MPPT\./);
  const t = await connected({ coil: true, coilNeverEnds: true, measuring: true });
  window = t.window;
  window.tstDefs().find(d => d.id === 'coil').timeout = 60;
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('coil');
  assert.equal(t.dev.sent.at(-1), 'mppt');
  note = card(window, 'coil').querySelector('.tst-res .note');
  assert.match(note.textContent, /No end of the test.*MPPT not resumed \(mppt: refused, measure-coil running\): send “mppt”\./);
  assert.ok(note.classList.contains('err'));
});

test('measure-coil in PSU/PV mode or at a manual duty: the firmware refusal, nothing stopped', async () => {
  for (const op of ['psu', 'duty']) {
    const { window, sent, note } = await coilRun({ op });
    assert.deepEqual(sent, ['selftest', 'measure-coil l0'], op);
    assert.match(note.textContent, op === 'psu' ? /refused “measure-coil l0”: measure-coil: PSU\/PV mode on.*The editor stops only MPPT for this test\.$/
      : /manual duty 812 set.*The editor stops only MPPT/);
    assert.ok(note.classList.contains('err'));
  }
});

test('measure-coil progress: L0 and LS rows against the sweep band, early stops complete it, the dc 0 phase is indeterminate', async () => {
  const { window } = await loadEditor();
  const P = ls => { const p = window.tstProgress(ls); return [p.f == null ? null : +p.f.toFixed(3), p.text]; };
  const L0 = ['measure-coil L0: fsw=39000 pwmMax=4103 band 1200..2600 step 100  Vin=36.10 Vout=26.80 M=0.742',
    '      H      D    Vin   Vout   Iout mode    L_uH'];
  const row = H => `  ${String(H).padStart(5)}  0.292  36.10  26.80  0.210  DCM   50.10`;
  assert.deepEqual(P([]), [null, 'starting …']);
  assert.deepEqual(P(['W (1) main: measure-coil: MPPT on, stop conversion first (dc 0)', 'ERR: measure-coil l0', 'OK: dc 0']), [null, 'stopping the converter …']);
  assert.deepEqual(P(['W (1) main: measure-coil: converter still on, wait for the ramp-down']), [null, 'stopping the converter …']);
  assert.deepEqual(P(L0), [0, 'step 0 of 15']);
  assert.deepEqual(P([...L0, row(1200)]), [0, 'step 1 of 15']);
  assert.deepEqual(P([...L0, row(1200), row(1300), row(1900)]), [0.5, 'step 8 of 15']);
  assert.deepEqual(P([...L0, row(2600)]), [1, 'step 15 of 15']);
  for (const stop of ['  entered CCM; stopping', '  Iout 2.10 > 2.00; stopping', '  converter disabled (protection?); aborting',
    '  Vin no longer above Vout+1; stopping', 'measure-coil: L = 50.34 uH  (ring fit, 31 DCM pts)'])
    assert.deepEqual(P([...L0, row(1500), stop]), [1, 'finishing …'], stop);
  const LS = ['measure-coil LS: HS=400 D=0.097 Vin=36.10 Vout=26.80 ideal_LS=139 auto_LS=130 sweep 70..195', '     LS    Iout    Vin   Vout mode'];
  const lrow = n => `  ${String(n).padStart(5)}   0.412  36.10  26.80  DCM`;
  assert.deepEqual(P(LS), [0, '0 %']);
  assert.deepEqual(P([...LS, lrow(70), lrow(120)]), [0.4, '40 %']);
  assert.deepEqual(P([...LS, lrow(151), '  passed peak; stopping']), [1, 'finishing …']);
  assert.deepEqual(P([...LS, lrow(151), 'measure-coil: peak LS 151 (Iout_peak 0.412 A at raw argmax 150, two-sided fit)']), [1, 'finishing …']);
});

test('progress bar: shown while a test runs, a value for measure-coil, indeterminate otherwise, hidden afterwards', async () => {
  const { window, dev } = await connected({ coil: true, coilNeverEnds: true, bfNeverEnds: true, rowMs: 40 });
  window.tstDefs().find(d => d.id === 'coil').timeout = 4000;
  await openSheet(window); await idle(window);
  window.confirm = () => true; window.prompt = () => 'fry';
  const c = card(window, 'coil'), pg = c.querySelector('.tst-prog');
  assert.equal(pg.getAttribute('role'), 'progressbar');
  assert.equal(pg.getAttribute('aria-label'), 'Coil inductance (L0) progress');
  assert.equal(pg.hidden, true);
  assert.equal(c.querySelector('.tst-ctl').nextElementSibling, pg, 'below the buttons');
  let release = holdClock(window);
  const run = window.tstStart('coil');
  await dev.when(() => c.querySelector('.tst-out').textContent.includes('1200  0.292'));
  assert.equal(seen(pg), true);
  const step = +c.querySelector('.tst-pct').textContent.match(/^step (\d+) of 47$/)[1];
  assert.ok(step >= 1 && step <= 2, 'the first rows');
  assert.equal(pg.getAttribute('aria-valuenow'), String(Math.round((step - 1) * 30 / 1400 * 100)));
  await dev.when(() => c.querySelector('.tst-out').textContent.includes('L = 50.34'));
  assert.equal(pg.getAttribute('aria-valuenow'), '100');
  assert.match(pg.querySelector('i').style.width, /^100(\.0)?%$/);
  release();
  await run;
  assert.equal(pg.hidden, true);
  const b = card(window, 'bflow'), bp = b.querySelector('.tst-prog');
  window.tstDefs().find(d => d.id === 'bflow').timeout = 200;
  release = holdClock(window);
  const r2 = window.tstStart('bflow');
  await until(() => window.tstBusyNow() && b.querySelector('.tst-out').textContent.includes('started'));
  assert.equal(seen(bp), true);
  assert.ok(bp.classList.contains('ind'));
  assert.equal(bp.getAttribute('aria-valuenow'), null);
  release();
  await r2;
});

test('coil chart parsing: L0 rows to L over Ipk, the fit bins and slope, the result; LS rows, ideal, auto and peak', async () => {
  const { window } = await loadEditor();
  const { l0Rows, lsRows } = require('./_tst_firmware');
  const L0 = ['measure-coil L0: fsw=39000 pwmMax=4103 band 1200..2600 step 30  Vin=36.10 Vout=26.80 M=0.742',
    '      H      D    Vin   Vout   Iout mode    L_uH', ...l0Rows(),
    'measure-coil: L = 50.34 uH  (ring fit, 31 DCM pts, Iout>=0.30A, +-0.4% noise +-1.1% syst; ring 310 kHz = 5.2 nF node C)',
    'measure-coil: L(Ipk), +-noise/offsets: 1.4-1.6A 50.62uH+-0.3/0.9% | 1.6-1.9A 50.40uH+-0.3/0.9% | 1.9-2.2A 50.15uH+-0.4/1.0%',
    'measure-coil: L(Ipk) slope -1.10 +-0.30 %/A over 1.4-2.2 A, Iout/on-time offsets add up to +-0.50: coil -2.20..+0.00 %/A, drop <= 2.2 %/A'];
  const p = window.parseCoilPlot(L0);
  assert.equal(p.ls, false); assert.equal(p.fsw, 39000);
  assert.equal(p.pts.length, 23, 'DCM rows only');
  const r0 = l0Rows()[0].trim().split(/\s+/).map(Number);   // H D Vin Vout Iout mode L
  assert.ok(Math.abs(p.pts[0].x - (r0[2] - r0[3]) * r0[1] / (39000 * r0[6] * 1e-6)) < 1e-9, 'Ipk = (Vin-Vout)·D/(fsw·L)');
  assert.ok(Math.abs(p.pts[0].x - 1.376) < 0.005);
  assert.equal(p.pts[0].y, r0[6]);
  assert.deepEqual([...p.pts].map(q => q.g), [...p.pts].map(q => q.io < 0.30), 'grey below the fit floor');
  assert.ok(p.pts.some(q => q.g) && p.pts.some(q => !q.g));
  assert.equal(p.L, 50.34); assert.equal(p.floor, 0.30);
  assert.deepEqual([...p.bins].map(b => [b.lo, b.hi, b.L, +b.se.toFixed(4), +b.sys.toFixed(4)]), [[1.4, 1.6, 50.62, 0.003, 0.009], [1.6, 1.9, 50.4, 0.003, 0.009], [1.9, 2.2, 50.15, 0.004, 0.01]]);
  assert.deepEqual({ ...p.slope, k: +p.slope.k.toFixed(4) }, { k: -0.011, lo: 1.4, hi: 2.2, drop: 2.2 });
  const s2 = window.parseCoilPlot(['measure-coil: L(Ipk) slope +0.40 +-0.30 %/A over 0.5-1.8 A, Iout/on-time offsets add up to +-0.20: coil -0.40..+1.20 %/A, no drop resolved']).slope;
  assert.deepEqual({ ...s2, k: +s2.k.toFixed(4) }, { k: 0.004, lo: 0.5, hi: 1.8, drop: null });
  assert.equal(window.parseCoilPlot(L0.slice(0, 5)).L, null, 'no result while rows arrive');
  const LS = ['measure-coil LS: HS=400 D=0.097 Vin=36.10 Vout=26.80 ideal_LS=139 auto_LS=130 sweep 70..195',
    '     LS    Iout    Vin   Vout mode', ...lsRows(), 'measure-coil: peak LS 151 (Iout_peak 0.412 A at raw argmax 150, two-sided fit)'];
  const q = window.parseCoilPlot(LS);
  assert.equal(q.ls, true); assert.equal(q.ideal, 139); assert.equal(q.auto, 130);
  assert.equal(q.pts.length, 22);
  assert.deepEqual({ ...q.pts[0] }, { x: 70, y: +(0.412 - 0.00012 * 81 ** 2).toFixed(3), g: true }, 'CCM grey');
  assert.deepEqual({ ...q.peak }, { x: 151, y: 0.412 });
});

test('coil L model: the measured line from the fit slope, incremental (DC bias) L = L0·(1 + 3k·I), none without a resolved drop', async () => {
  const { window } = await loadEditor();
  const bins = 'measure-coil: L(Ipk), +-noise/offsets: 1.4-1.6A 50.62uH+-0.3/0.9% | 1.6-1.9A 50.40uH+-0.3/0.9% | 1.9-2.2A 50.15uH+-0.4/1.0%';
  const slope = 'measure-coil: L(Ipk) slope -1.10 +-0.30 %/A over 1.4-2.2 A, Iout/on-time offsets add up to +-0.50: coil -2.20..+0.00 %/A, drop <= 2.2 %/A';
  const m = window.coilLModel(window.parseCoilPlot([bins, slope]));
  const ic = (1.5 + 1.75 + 2.05) / 3, L0 = (50.62 + 50.40 + 50.15) / 3 / (1 - 0.011 * ic);
  assert.ok(Math.abs(m.L0 - L0) < 1e-9 && Math.abs(m.L0 - 51.389) < 0.001, 'L(Ipk -> 0) from the bins\' mean and k');
  assert.ok(Math.abs(m.meas(ic) - 50.39) < 1e-9, 'the line passes the bins\' mean');
  // a DCM cycle measures λ²/(2E): with λ = ∫L_inc dI and E = ∫I·L_inc dI that is the measured line, to first order in k·I
  for (const I of [0.5, 1.4, 2.2]) {
    let lam = 0, E = 0;
    for (let j = 0, N = 2000; j < N; j++) { const i = (j + 0.5) * I / N, l = m.inc(i) * I / N; lam += l; E += i * l; }
    assert.ok(Math.abs(lam * lam / (2 * E) / m.meas(I) - 1) < 2e-4, `λ²/(2E) at ${I} A`);
  }
  assert.ok(Math.abs(m.inc(2.2) - 47.66) < 0.01);
  assert.ok(Math.abs((m.inc(2.2) / m.L0 - 1) * 100 + 7.26) < 0.01);
  const nd = window.coilLModel(window.parseCoilPlot([bins, 'measure-coil: L(Ipk) slope +0.40 +-0.30 %/A over 1.4-2.2 A, Iout/on-time offsets add up to +-0.20: coil -0.40..+1.20 %/A, no drop resolved']));
  assert.ok(nd.meas && nd.inc === null, 'no drop resolved: the measured line only');
  assert.equal(window.coilLModel(window.parseCoilPlot([bins])), null, 'no slope line');
  const fig = window.document.createElement('figure');
  window.tstCoilChart(fig, ['measure-coil L0: fsw=39000 pwmMax=4103', '      H      D    Vin   Vout   Iout mode    L_uH',
    '   1200  0.292  36.10  26.80  0.600  DCM  50.50', bins, 'measure-coil: L(Ipk) slope +0.40 +-0.30 %/A over 1.4-2.2 A, Iout/on-time offsets add up to +-0.20: coil -0.40..+1.20 %/A, no drop resolved'], 390, 0);
  assert.equal(fig.querySelectorAll('.inc').length, 0);
  assert.equal(fig.querySelectorAll('.fit').length, 1);
  assert.equal(fig.querySelector('.tst-verdict').textContent, 'No drop resolved up to 2.2 A, so no incremental L');
});

test('coil chart: drawn live in the card, the result, coil.conf L0 and the 0.95·L0 model, bins, slope and its verdict', async () => {
  const { window, dev } = await connected({ coil: true, rowMs: 30, conf: { ...CONF, 'coil.conf': { L0: '5.090000e-05' } } });
  window.tstDefs().find(d => d.id === 'coil').timeout = 60000;
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  const c = card(window, 'coil'), fig = c.querySelector('.tst-chart');
  assert.equal(fig.hidden, true, 'no chart before a run');
  const run = window.tstStart('coil');
  await dev.when(() => +((fig.querySelector('svg') || { getAttribute: () => 0 }).getAttribute('data-points')) >= 3);
  assert.equal(seen(fig), true, 'drawn while rows arrive');
  assert.equal(c.querySelector('.tst-log').open, false, 'the device output stays folded under the chart');
  assert.equal(fig.querySelector('.res'), null, 'no result line yet');
  await run;
  const svg = fig.querySelector('svg');
  assert.equal(svg.getAttribute('data-points'), '23');
  assert.equal(fig.querySelectorAll('circle.pt').length + fig.querySelectorAll('circle.pt.gr').length >= 23, true);
  const labs = [...svg.querySelectorAll('text')].map(t => t.textContent);
  assert.ok(labs.includes('L = 50.34 µH (ring fit)'));
  assert.ok(labs.includes('coil.conf L0 50.9 µH'));
  assert.ok(labs.includes('model (0.95·L0)'));
  assert.ok(labs.includes('A') || labs.some(t => / A$/.test(t)), 'Ipk axis in A');
  assert.ok(labs.some(t => / µH$/.test(t)), 'L axis in µH');
  assert.equal(svg.querySelectorAll('.ref.dash').length, 1);
  assert.equal(svg.querySelectorAll('.bpt').length, 3);
  assert.equal(svg.querySelectorAll('.ese').length, 3); assert.equal(svg.querySelectorAll('.esys').length, 3);
  assert.equal(svg.querySelectorAll('.fit').length, 1);
  assert.equal(svg.querySelectorAll('.inc').length, 1);
  assert.ok(labs.includes('incremental (DC bias)'));
  assert.equal(fig.querySelector('.tst-verdict').textContent, 'L drops ≤ 2.2 %/A up to 2.2 A; incremental L at 2.2 A: 47.7\u00a0µH (−7.3\u00a0% vs 0 A)');
  assert.match(svg.getAttribute('aria-label'), /^Inductance over peak coil current, 23 points, L = 50\.34 µH \(ring fit\), coil\.conf L0 50\.9 µH; L drops/);
  assert.equal(fig.querySelector('.tst-cap').textContent, 'Dots: raw per-step values, they swing with the DCM ring (310 kHz); the line is the ring-corrected result.');
  assert.ok(!svg.outerHTML.includes('NaN'));
  fold(window, 'coil');
  assert.equal(seen(fig), false, 'folds with the card');

  window.confirm = () => true;
  await window.tstStart('coills');
  const ls = card(window, 'coills').querySelector('.tst-chart svg');
  const ll = [...ls.querySelectorAll('text')].map(t => t.textContent);
  assert.ok(ll.includes('ideal 139') && ll.includes('auto 130') && ll.includes('peak 151'));
  assert.equal(ls.getAttribute('data-points'), '22');
  assert.ok(ll.some(t => / ct$/.test(t)));
  assert.equal(card(window, 'coills').querySelector('.tst-verdict'), null);
  assert.ok(!ls.outerHTML.includes('NaN'));
});

test('coil chart with DCM ringing: faded raw dots joined in step order, the ring-fit or median result, caption, labels apart', async () => {
  const { window } = await loadEditor();
  const { RING_ROWS, ringRun } = require('./_tst_firmware');
  const head = ['measure-coil L0: fsw=39000 pwmMax=4103 band 374..1634 step 20  Vin=73.07 Vout=26.78 M=0.366', '      H      D    Vin   Vout   Iout mode    L_uH'];
  const draw = (lines, l0 = 0) => { const fig = window.document.createElement('figure'); window.tstCoilChart(fig, lines, 520, l0); return fig; };
  const xy = e => [+e.getAttribute('cx'), +e.getAttribute('cy')];
  const verts = fig => fig.querySelector('path.step').getAttribute('d').match(/[ML][\d.-]+ [\d.-]+/g).map(v => v.slice(1).split(' ').map(Number));
  const texts = fig => [...fig.querySelectorAll('svg text')].map(t => t.textContent);

  let fig = draw([...head, ...RING_ROWS]);                         // live, the board's rows so far
  const dots = [...fig.querySelectorAll('circle.pt')];
  assert.equal(dots.length, 13);
  assert.ok(dots.every(c => c.classList.contains('raw')), 'raw rows are faded dots');
  assert.deepEqual(verts(fig), dots.map(xy), 'the step line joins the dots in H order');
  const xs = dots.map(c => +c.getAttribute('cx'));
  assert.ok(xs.some((x, i) => i && x < xs[i - 1]), 'Ipk goes back within a ring cycle: H order is not x order');
  fig = draw([...head, ...[...RING_ROWS].reverse()]);
  assert.deepEqual(verts(fig), [...fig.querySelectorAll('circle.pt')].map(xy).reverse(), 'sorted by H, not by arrival');
  assert.equal(fig.querySelector('.res'), null, 'no result line while the run goes on');
  assert.ok(texts(fig).includes('ring-corrected L at the end'));
  assert.equal(fig.querySelector('.tst-cap'), null);
  fig = draw([...head, ...RING_ROWS, 'measure-coil: only 2 DCM pts above 0.40 A; widen band or wait for sun']);
  assert.ok(!texts(fig).includes('ring-corrected L at the end'), 'no hint once the run ended without a value');

  const ring = ringRun();
  fig = draw(ring, 7.4e-5);
  assert.equal(fig.querySelector('svg').getAttribute('data-points'), '45');
  assert.equal(fig.querySelectorAll('circle.pt.raw').length, 42, 'grey rows outside the fitted rows\' range are off the axis');
  assert.equal(fig.querySelectorAll('circle.pt.raw.gr').length, 7, 'grey below the fit floor');
  assert.deepEqual(verts(fig), [...fig.querySelectorAll('circle.pt')].map(xy), 'the step line starts at the first point on the axis');
  assert.equal(fig.querySelectorAll('.res').length, 1);
  const rl = fig.querySelector('text.res-lab');
  assert.equal(rl.textContent, 'L = 73.88 µH (ring fit)');
  assert.ok(Math.abs(+rl.getAttribute('y') + 4 - +fig.querySelector('line.res').getAttribute('y1')) < 0.2, 'label on its line');
  assert.ok(!texts(fig).includes('ring-corrected L at the end'));
  assert.equal(fig.querySelector('.tst-cap').textContent, 'Dots: raw per-step values, they swing with the DCM ring (310 kHz); the line is the ring-corrected result.');
  assert.ok(fig.querySelector('.tst-cap').compareDocumentPosition(fig.querySelector('.tst-verdict')) & window.Node.DOCUMENT_POSITION_FOLLOWING);
  assert.match(fig.querySelector('svg').getAttribute('aria-label'), /, L = 73\.88 µH \(ring fit\), coil\.conf L0 74 µH;/);
  const box = t => [t.getAttribute('text-anchor'), +t.getAttribute('y')];
  const cl = [...fig.querySelectorAll('text.ref-lab')].find(t => t.textContent === 'coil.conf L0 74 µH');
  const [a1, y1] = box(rl), [a2, y2] = box(cl);
  assert.ok(a1 !== a2 || Math.abs(y1 - y2) >= 13, 'L and coil.conf L0 0.2 % apart: labels do not overlap');
  const right = [...fig.querySelectorAll('text.res-lab, text.ref-lab')].filter(t => t.getAttribute('text-anchor') === 'end').map(t => +t.getAttribute('y'));
  assert.ok(right.every((y, i) => right.every((z, j) => i === j || Math.abs(y - z) >= 13)));
  assert.equal(fig.querySelectorAll('.bpt').length, 3); assert.equal(fig.querySelectorAll('.fit').length, 1); assert.equal(fig.querySelectorAll('.inc').length, 1);
  assert.ok(texts(fig).includes('model (0.95·L0)') && texts(fig).includes('incremental (DC bias)'));
  assert.ok(!fig.outerHTML.includes('NaN'));

  fig = draw(ringRun({ fit: 'median' }));
  assert.equal(fig.querySelector('text.res-lab').textContent, 'L = 73.93 µH (median, ring not resolved)');
  assert.equal(fig.querySelector('.tst-cap'), null, 'no ring caption without a ring fit');
  assert.match(fig.querySelector('svg').getAttribute('aria-label'), /L = 73\.93 µH \(median, ring not resolved\)/);
  const p = window.parseCoilPlot(ring);
  assert.deepEqual([p.fit, p.ring, p.floor, p.end], ['ring', 310, 0.37, true]);
  assert.deepEqual([...p.pts.slice(0, 3).map(q => q.h)], [374, 394, 414]);
});

// Coil DC bias: measure-coil l0, then l0 f=<Hz> (the lowest the firmware takes for coil.conf L0), mppt; both passes in one chart
const DONE = "measure-coil: done, manual PWM at duty 0 ('mppt' resumes tracking)";
const biasFig = (w, lines, l0 = 50e-6, cw = 520) => { const fig = w.document.createElement('figure'); w.tstCoilChart(fig, lines, cw, l0, true); return fig; };

test('coil DC bias: dc 0, both passes and mppt under one lock; the second at the lowest frequency coil.conf L0 allows', async () => {
  const { window, sent, busy, note } = await coilRun({ coilBias: true }, 'coilbias');
  assert.deepEqual(sent, ['selftest', 'measure-coil l0', 'dc 0', 'measure-coil l0', 'measure-coil l0 f=21500', 'mppt']);
  assert.ok(busy.every(Boolean), 'the device lock is held for every command');
  assert.equal(window.tstBusyNow(), false);
  const res = card(window, 'coilbias').querySelector('.tst-res');
  assert.ok(res.classList.contains('pass'));
  assert.match(res.querySelector('.sum').textContent, /^50\.6 µH at low current; 10 % lower at 4\.0 A, 20 % lower at 6\.2 A$/);
  assert.match(res.textContent, /Incremental inductance, the value a datasheet's DC-bias curve shows, from a fit through 8 ring-corrected values of both passes\./);
  assert.match(res.textContent, /Averaged from zero current \(secant inductance\): 10 % lower at 7\.3 A, no 20 % drop up to 8\.3 A\./);
  assert.match(res.textContent, /First pass at 39 kHz: L = 49\.6 µH, peak current up to 5\.9 A\..*Second pass at 21\.5 kHz: L = 48\.78 µH, peak current up to 8\.3 A\./);
  assert.equal(note.textContent, 'Resumed MPPT.');
  assert.equal(card(window, 'coilbias').querySelector('.tst-cmd').textContent, 'measure-coil l0  ·  measure-coil l0 f=21500');
  assert.match(def(window, 'coilbias').does(), /at 21\.5 kHz, which drives a higher peak current/);
  let asked = '';
  window.confirm = m => { asked = m; return false; };
  await window.tstStart('coilbias');
  assert.match(asked, /^Coil DC bias: run “measure-coil l0”, then “measure-coil l0 f=21500” on fry\?/);
  const ok = await coilRun({ coilBias: true, op: 'manu' }, 'coilbias');
  assert.deepEqual(ok.sent, ['selftest', 'measure-coil l0', 'measure-coil l0 f=21500'], 'already at dc 0: no dc 0, no mppt');
});

test('coil DC bias frequency: 20 kHz, or the fsw·L0·0.95 > 1 Ω floor rounded up to 500 Hz with a margin', async () => {
  for (const [L0, hz] of [['80e-6', 20000], ['50e-6', 21500], ['5.034000e-05', 21500], ['5.2e-05', 20500], ['', 20000]]) {
    const { window } = await connected({ coil: true, conf: { ...CONF, 'coil.conf': { L0 } } });
    assert.equal(window.tstBiasHz(), hz, L0);
  }
  const { window } = await loadEditor();
  const p1 = ['measure-coil L0: fsw=22000 pwmMax=7258 band 1..2 step 1', 'measure-coil: L = 50.00 uH  (median of 9 DCM pts, Iout>=0.15A; ring not resolved: too few points, expect up to +-5%)'];
  assert.match(window.tstBiasNext(p1).skip, /no switching frequency below 20 kHz, close to the normal 22 kHz/);
  assert.match(window.tstBiasNext(['measure-coil: need Vin>Vout+1 (Vin=12.00 Vout=26.80)']).skip, /first pass gave no inductance/);
});

test('coil DC bias model: 1/L even in the flux, the incremental and secant drop of a saturating coil, monotonic only', async () => {
  const { window } = await loadEditor();
  const { biasRun } = require('./_tst_firmware');
  const ps = window.coilBiasPasses([...biasRun(39000), DONE, ...biasRun(21500), DONE]);
  assert.deepEqual(Array.from(ps, p => [p.fsw, p.bins.length]), [[39000, 5], [21500, 3]]);
  const m = window.coilBiasModel(ps.flatMap(window.coilFitPts));
  // the fake's coil: L_inc = 50.6 µH/(1 + (i/12 A)²): 10 % lower at 4 A, 20 % at 6 A
  assert.ok(Math.abs(m.L0 - 50.6) < 0.1, m.L0);
  assert.ok(Math.abs(m.drop(m.inc, 0.1) - 4.0) < 0.15 && Math.abs(m.drop(m.inc, 0.2) - 6.0) < 0.3);
  for (const l of [50, 150, 300]) {
    const h = 1e-3, di = (m.cur(l + h) - m.cur(l - h)) / (2 * h);
    assert.ok(Math.abs(m.inc(l) - 1 / di) < 1e-6 * m.inc(l), 'L_inc = dλ/di');
    assert.ok(Math.abs(m.sec(l) - l / m.cur(l)) < 1e-9, 'secant λ/i');
    let E = 0;
    for (let j = 0, N = 2000; j < N; j++) { const a = l * j / N, b = l * (j + 1) / N; E += (m.cur(a) + m.cur(b)) / 2 * (b - a); }
    assert.ok(Math.abs(l * l / (2 * E) / m.meas(l) - 1) < 1e-5, 'the measured L is λ²/(2E)');
  }
  // too few points, a rising L the even model cannot bend into a monotonic curve: no model
  assert.equal(window.coilBiasModel([{ x: 2, y: 50, s: 0.01 }]), null);
  const flat = window.coilBiasModel([1, 2, 3, 4].map(x => ({ x, y: 50, s: 0.01 })));
  assert.ok(Math.abs(flat.L0 - 50) < 1e-9 && flat.drop(flat.inc, 0.1) === null);
  assert.equal(window.coilDropTxt(flat, flat.inc), 'no 10 % drop up to 4.0 A');
  const one = window.coilBiasSum(['measure-coil L0: fsw=39000 pwmMax=4089 band 457..1644 step 25', '    600  0.147  60.00  26.80  0.500  DCM   50.10',
    'measure-coil: L = 50.10 uH  (median of 9 DCM pts, Iout>=0.15A; ring not resolved: too few points, expect up to +-5%)', DONE], 'The higher-current pass was refused: x.');
  assert.deepEqual([one.cls, one.text], ['warn', 'L = 50.10 µH; too few ring-corrected values for a DC-bias curve']);
  const wavy = window.coilBiasModel([[1, 50], [2, 48], [3, 52], [4, 47], [5, 53]].map(([x, y]) => ({ x, y, s: 0.001 })));
  assert.ok(!wavy || !wavy.quart, 'a wavy fit is not monotonic: no quartic term');
});

test('coil DC bias chart: both passes (round, square), bins, the fit and incremental curves from 0 A, coil.conf L0, a legend', async () => {
  const { window, dev } = await connected({ coil: true, coilBias: true, rowMs: 10 });
  window.tstDefs().find(d => d.id === 'coilbias').timeout = 60000;
  Object.assign(window.tstTimings(), { stop: 100 });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  const c = card(window, 'coilbias'), fig = c.querySelector('.tst-chart');
  const run = window.tstStart('coilbias');
  await dev.when(() => fig.querySelectorAll('circle.pt.raw').length >= 3);
  assert.ok([...fig.querySelectorAll('svg text')].some(t => t.textContent === 'curves after the first pass'), 'a hint while the first pass runs');
  await dev.when(() => fig.querySelectorAll('rect.pt.raw').length >= 3);
  assert.equal(fig.querySelectorAll('path.inc.cv').length, 1, 'the curves from the first pass while the second runs');
  await run;
  const svg = fig.querySelector('svg');
  assert.equal(svg.getAttribute('data-points'), '58');
  assert.equal(fig.querySelectorAll('circle.pt.raw').length, 36); assert.equal(fig.querySelectorAll('rect.pt.raw').length, 22);
  assert.equal(fig.querySelectorAll('circle.bpt').length, 5 + 1); assert.equal(fig.querySelectorAll('rect.bpt').length, 3 + 1, 'bins, and one each in the legend');
  assert.equal(fig.querySelectorAll('path.step').length, 2);
  assert.equal(fig.querySelectorAll('path.fit.cv').length, 1); assert.equal(fig.querySelectorAll('path.inc.cv').length, 1);
  assert.ok(fig.querySelector('path.inc.cv').getAttribute('d').startsWith('M' + fig.querySelector('path.fit.cv').getAttribute('d').slice(1).split('L')[0]), 'both from 0 A');
  const labs = [...svg.querySelectorAll('text')].map(t => t.textContent);
  assert.ok(labs.includes('coil.conf L0 50 µH') && !labs.includes('model (0.95·L0)'));
  assert.ok(labs.includes('0') && labs.some(t => / A$/.test(t)), 'the current axis starts at 0 A');
  assert.equal(svg.querySelectorAll('.res').length, 0);
  assert.deepEqual([...fig.querySelectorAll('.tst-legend > span')].map(s => s.textContent), ['first pass, 39 kHz', 'second pass, 21.5 kHz', 'fit', 'incremental (DC bias)']);
  assert.match(svg.getAttribute('aria-label'), /^Inductance over peak coil current, 58 points in 2 passes \(39 kHz, 21\.5 kHz\), coil\.conf L0 50 µH; 50\.6 µH at low current, incremental inductance 10 % lower at 4\.0 A, 20 % lower at 6\.2 A\.$/);
  assert.equal(fig.querySelector('.tst-cap').textContent, 'Small marks: raw per-step values, they swing with the DCM ring; large marks: ring-corrected values with their error bars.');
  assert.equal(fig.querySelector('.tst-verdict'), null);
  assert.ok(!svg.outerHTML.includes('NaN'));
  assert.equal(c.querySelector('.tst-log').open, false, 'the device output stays folded under the chart');
});

test('coil DC bias progress: the first pass fills half the bar, the second the rest', async () => {
  const { window } = await loadEditor();
  const { biasRun } = require('./_tst_firmware');
  const P = ls => { const p = window.tstBiasProgress(ls); return [p.f == null ? null : +p.f.toFixed(3), p.text]; };
  const a = biasRun(39000), b = biasRun(21500), rows = n => a.slice(0, 2 + n);
  assert.deepEqual(P([]), [null, 'starting …']);
  assert.deepEqual(P(rows(0)), [0, 'pass 1 of 2: step 0 of 48']);
  assert.deepEqual(P(rows(12)), [0.116, 'pass 1 of 2: step 12 of 48']);
  assert.deepEqual(P([...a, DONE]), [0.5, 'pass 2 of 2: starting …']);
  assert.deepEqual(P([...a, DONE, ...b.slice(0, 12)]), [0.594, 'pass 2 of 2: step 10 of 48']);
  assert.deepEqual(P([...a, DONE, ...b]), [1, 'pass 2 of 2: finishing …']);
  const { window: w2, dev } = await connected({ coil: true, coilBias: true, coilNeverEnds: true, rowMs: 40 });
  w2.tstDefs().find(d => d.id === 'coilbias').timeout = 300;
  await openSheet(w2); await idle(w2);
  w2.confirm = () => true;
  const release = holdClock(w2);
  const c = card(w2, 'coilbias'), run = w2.tstStart('coilbias');
  await dev.when(() => c.querySelector('.tst-out').textContent.includes('  457 '));
  assert.equal(c.querySelector('.tst-prog').getAttribute('aria-label'), 'Coil DC bias progress');
  assert.match(c.querySelector('.tst-pct').textContent, /^pass 1 of 2: step [12] of 48$/);
  release();
  await run;
});

test('coil DC bias: a refused second pass, firmware that ignores f= and a failed frequency change keep the first pass, in amber', async () => {
  let { window, sent, note } = await coilRun({ coilBias: true, coilFswRefuse: 'f= needs the MCPWM gate driver build' }, 'coilbias');
  assert.deepEqual(sent, ['selftest', 'measure-coil l0', 'dc 0', 'measure-coil l0', 'measure-coil l0 f=21500', 'mppt']);
  let res = card(window, 'coilbias').querySelector('.tst-res');
  assert.ok(res.classList.contains('warn'));
  assert.match(res.querySelector('.sum').textContent, /^50\.6 µH at low current; 10 % lower at 4\.0 A, no 20 % drop up to 5\.9 A$/);
  assert.match(res.textContent, /The higher-current pass was refused: measure-coil: f= needs the MCPWM gate driver build\./);
  assert.match(res.textContent, /fit through 5 ring-corrected values\./);
  assert.ok(!/Second pass/.test(res.textContent));
  assert.equal(note.textContent, 'Resumed MPPT.');

  ({ window, sent } = await coilRun({ coilBias: true, coilOld: true }, 'coilbias'));
  assert.deepEqual(sent.slice(-2), ['measure-coil l0 f=21500', 'mppt']);
  res = card(window, 'coilbias').querySelector('.tst-res');
  assert.ok(res.classList.contains('warn'));
  assert.match(res.textContent, /This firmware ignored “f=” and ran the second pass at 39 kHz again: the higher-current pass needs newer firmware \(measure-coil l0 f=\)\. Only the first pass is used\./);
  const fig = card(window, 'coilbias').querySelector('.tst-chart');
  assert.equal(fig.querySelectorAll('rect.pt').length, 0, 'the repeated pass is not drawn');
  assert.deepEqual([...fig.querySelectorAll('.tst-legend > span')].map(s => s.textContent), ['first pass, 39 kHz', 'fit', 'incremental (DC bias)']);

  ({ window } = await coilRun({ coilBias: true, coilFswFail: 'fsw*L0 out of range for this coil (coil.conf::L0)' }, 'coilbias'));
  assert.match(resText(window, 'coilbias'), /Second pass: fsw 21500 Hz: fsw\*L0 out of range for this coil \(coil\.conf::L0\)/);
  ({ window } = await coilRun({ coilBias: true, coilRestoreFail: 'measure-coil: fsw 39000 Hz restore failed, now 21500 Hz: run `pwm-freq 39000`' }, 'coilbias'));
  res = card(window, 'coilbias').querySelector('.tst-res');
  assert.ok(res.classList.contains('warn'));
  assert.match(res.textContent, /The switching frequency was not restored: fsw 39000 Hz restore failed, now 21500 Hz: run `pwm-freq 39000`\./);
});

test('coil DC bias failures still send mppt: a failed first pass skips the second, a second pass that never ends', async () => {
  let { window, sent, note } = await coilRun({ coilBias: true, coilFail: true }, 'coilbias');
  assert.deepEqual(sent, ['selftest', 'measure-coil l0', 'dc 0', 'measure-coil l0', 'mppt']);
  const res = card(window, 'coilbias').querySelector('.tst-res');
  assert.ok(res.classList.contains('fail'));
  assert.match(res.textContent, /No result: need Vin>Vout\+1.*The first pass gave no inductance, so the higher-current pass was not started\./);
  assert.equal(note.textContent, 'Resumed MPPT.');
  const t = await connected({ coil: true, coilBias: true, coilFNeverEnds: true });
  window = t.window;
  window.tstDefs().find(d => d.id === 'coilbias').timeout = 1500;
  Object.assign(window.tstTimings(), { stop: 100 });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  // the first pass must end within the timeout however slow the host: the clock runs from the second pass on
  const release = holdClock(window), h = t.dev.handler;
  t.dev.handler = (cmd, s) => { if (/ f=/.test(cmd)) release(); return h(cmd, s); };
  await window.tstStart('coilbias');
  assert.deepEqual(t.dev.sent.slice(-2), ['measure-coil l0 f=21500', 'mppt']);
  note = card(window, 'coilbias').querySelector('.tst-res .note');
  assert.match(note.textContent, /^No end of the second pass within 2 s: its state is unknown\. MPPT not resumed \(mppt: refused, measure-coil running\): send “mppt”\.$/);
  assert.match(resText(window, 'coilbias'), /50\.6 µH at low current; 10 % lower at 4\.0 A/, 'the first pass\'s result stays');
  assert.ok(card(window, 'coilbias').querySelector('.recheck'));
});

test('measure-coil link check after a timeout uses the status line', async () => {
  const { window } = await connected({ coil: true, coilNeverEnds: true, measuring: true });
  window.tstDefs().find(d => d.id === 'coil').timeout = 60;
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('coil');
  card(window, 'coil').querySelector('.recheck').click();
  await idle(window);
  assert.match(resText(window, 'coil'), /printed no status line: a coil measurement may still be running/);
});

test('one at a time: services actions and pushes wait for a running test, and the reverse', async () => {
  const { window, dev } = await connected({ bfTicks: 200 });
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  window.confirm = () => true;
  const run = window.tstStart('bflow');
  await until(() => window.tstBusyNow() && dev.sent.includes('selftest bflow'));
  await window.svcAction('ftp', 'off');
  assert.ok(!dev.sent.includes('svc off ftp'));
  assert.match($(window, '#uploadlog-cmd').placeholder, /waiting for “selftest bflow”/);   // shared devBusy lock
  await window.uploadChanged({ reviewed: true });   // nothing pending anyway; must not send
  const nWire = dev.sent.filter(c => c === 'wire').length;
  await window.tstRead('wire');
  assert.equal(dev.sent.filter(c => c === 'wire').length, nWire, 'a second test does not start');
  card(window, 'bflow').querySelector('.abort').click();
  await run;
  assert.equal(window.tstBusyNow(), false);
});

test('a status command the build lacks drops its row (help unavailable)', async () => {
  const { window, dev } = await connected({ helpSilent: true, absent: ['wire'] });
  quick(window);
  await openSheet(window); await idle(window);
  assert.match($(window, '#tst-msg').textContent, /No reply to “help”/);
  assert.equal(row(window, 'adccal'), null, 'unknown build: adc-cal stays hidden');
  assert.ok(dev.sent.includes('wire'));
  assert.equal(row(window, 'wire'), null);
  assert.ok(row(window, 'selftest') && row(window, 'rcstatus'));
  assert.match($(window, '#tst-body .tst-absent').textContent, /\bwire\b/);
  const from = dev.sent.length;
  await window.tstRead('selftest');
  assert.deepEqual(dev.sent.slice(from), ['selftest']);
  assert.equal(rowText(window, 'selftest'), 'OK, nothing latched');
});

test('backflow card disabled on a board without bflow_bat; Escape and the scrim close the sheet', async () => {
  const { window } = await connected({ bfChecked: false });
  await openSheet(window); await idle(window);
  assert.equal(card(window, 'bflow').querySelector('.run').disabled, true);
  assert.match(card(window, 'bflow').querySelector('.run').title, /bflow_bat=0/);

  row(window, 'wire').querySelector('summary').focus();
  window.document.dispatchEvent(new window.KeyboardEvent('keydown', { key: 'Escape', bubbles: true }));
  assert.equal($(window, '#tests').classList.contains('open'), false);
  assert.equal(window.document.activeElement.id, 'open-tests');
  btn(window).click();
  assert.equal($(window, '#tests').classList.contains('open'), true);
  $(window, '#tst-scrim').click();
  assert.equal($(window, '#tests').classList.contains('open'), false);
});

const chart = w => card(w, 'sweep').querySelector('.tst-chart');
function drawn(window, lines) {
  const sum = window.tstSummary('sweep', lines), fig = window.document.createElement('figure');
  window.tstChart(fig, sum.curve, 520);
  const svg = fig.querySelector('svg');
  if (svg) assert.doesNotMatch(svg.outerHTML, /NaN|Infinity/);
  return { sum, fig, svg, cap: fig.querySelector('figcaption') };
}

// The page's "N min ago" reads its own Date.now(); anchor it s seconds after the fixture sweep so the
// label does not depend on how long the suite ran since SWEEP was built.
function pinSweepAge(window, s = 120) {
  const t = +SWEEP.lines[0].match(/\bt=(\d+)/)[1], real = window.Date.now.bind(window.Date);
  const off = (t + s) * 1000 - real();
  window.Date.now = () => real() + off;
}

test('MPP sweep card: Read last sweep sends sweep-curve without a confirm and draws the P-V curve', async () => {
  const { window, dev } = await connected();
  pinSweepAge(window);
  await openSheet(window); await idle(window);
  assert.equal(card(window, 'curve'), null, 'one sweep card');
  const c = card(window, 'sweep');
  assert.equal(c.querySelector('.tst-x').textContent, 'MPP sweep');
  assert.equal(c.querySelector('.tst-tag').textContent, 'Drives the converter');
  assert.equal(c.querySelector('.run').textContent, 'Run sweep');
  assert.equal(c.querySelector('.read').textContent, 'Read last sweep');
  assert.match(c.querySelector('.tst-cmd').textContent, /^sweep {2}· {2}sweep-curve$/);
  assert.equal(chart(window).hidden, true, 'no chart before a read');
  assert.equal(c.classList.contains('shown'), false);
  window.confirm = () => { throw new Error('reading the last sweep must not confirm'); };
  const from = dev.sent.length;
  c.querySelector('.read').click();
  await idle(window);
  assert.deepEqual(dev.sent.slice(from), ['sweep-curve']);
  assert.match(resText(window, 'sweep'), new RegExp(`MPP ${SWEEP.P} W at ${SWEEP.V} V.*87 points, 2 min ago`));
  assert.ok(c.classList.contains('shown'), 'the result shows in the card');
  assert.equal(c.querySelector('.tst-res .sum').childElementCount, 3, 'summary, points and landing on one line');
  const fig = chart(window), svg = fig.querySelector('svg');
  assert.equal(fig.hidden, false);
  assert.equal(svg.getAttribute('role'), 'img');
  assert.equal(svg.getAttribute('data-points'), '87');
  assert.equal((svg.querySelector('path.ln').getAttribute('d').match(/[ML]/g) || []).length, 87);
  assert.equal(svg.querySelectorAll('circle.pt').length, 0, 'no point markers on a full curve');
  assert.equal(svg.querySelectorAll('circle.mpp').length, 1);
  assert.equal(svg.querySelector('text.mlab').textContent, `${SWEEP.P} W · ${SWEEP.V} V`);
  assert.match(svg.getAttribute('aria-label'), new RegExp(`MPP ${SWEEP.P} W at ${SWEEP.V} V, 87 points`));
  const ticks = [...svg.querySelectorAll('text:not(.mlab)')].map(t => t.textContent);
  assert.ok(ticks.some(t => / W$/.test(t)) && ticks.some(t => / V$/.test(t)), 'axis units');
  assert.match(fig.querySelector('figcaption').textContent, /^Sweep at \S/);
  assert.doesNotMatch(svg.outerHTML, /NaN|Infinity/);
});

test('without teledec.js the sweep card keeps the text summary and draws nothing', async () => {
  const { window } = await connected({ teledec: false });
  pinSweepAge(window);
  assert.equal(window.TeleDec, undefined);
  await openSheet(window); await idle(window);
  card(window, 'sweep').querySelector('.read').click();
  await idle(window);
  assert.match(resText(window, 'sweep'), new RegExp(`MPP ${SWEEP.P} W at ${SWEEP.V} V.*87 points, 2 min ago`));
  assert.equal(chart(window).hidden, true);
  assert.equal(chart(window).querySelector('svg'), null);
  assert.equal(window.parseCurve(SWEEP.lines).points, undefined, 'header only');
});

// A real run cut short by the output-current limit after 3.5 s; its `sweep-curve` as the device returned it.
const STOP = 'Stop sweep 3.50s mode=CC (lim=3 tgt=2.00 act=2.04) PWM=812 MPP=(54.3W,812,67.4V)';
const CUT = { lines: ['sc: t=1795441928 n=1 mpp=54.3,67.4 sweeping=0', 'sc: 68.11 0.0', 'sc: end'] };

test('a sweep stopped by a limiter: "stop point", why it ended, and no chart for one point', async () => {
  let { window, dev } = await connected({ sweepStop: STOP, curve: CUT });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('sweep');
  assert.deepEqual(dev.sent.slice(-3), ['selftest', 'sweep', 'sweep-curve']);
  const res = card(window, 'sweep').querySelector('.tst-res');
  assert.ok(res.classList.contains('warn'));
  assert.match(res.querySelector('strong').textContent, /^Stop point 54\.3 W at 67\.4 V$/);
  assert.match(res.textContent, /Stopped after 3\.5 s by the output-current limit \(2\.0 A\): the MPP shown is where the sweep stopped, not the panel's maximum\./);
  assert.match(res.textContent, /The device recorded only 1 point, so there is no curve\./);
  assert.match(res.textContent, /1 point, /); assert.doesNotMatch(res.textContent, /1 points/);
  assert.equal(chart(window).hidden, true);
  assert.equal(chart(window).querySelector('svg'), null);

  ({ window } = await connected({ curve: CUT }));
  await openSheet(window); await idle(window);
  card(window, 'sweep').querySelector('.read').click();
  await idle(window);
  assert.match(resText(window, 'sweep'), /^MPP 54\.3 W at 67\.4 V/, 'no stop line: an MPP');
  assert.doesNotMatch(resText(window, 'sweep'), /Stopped after/);
  assert.match(resText(window, 'sweep'), /only 1 point, so there is no curve/);
  assert.equal(chart(window).hidden, true);
});

// A full-range sweep that ended at the PWM bound: the firmware still names the power controller (smallest response).
const FULL = 'Stop sweep 8.43s mode=CV (lim=4 tgt=800.00 act=142.80) PWM=3782 MPP=(142.8W,3100,59.7V)';

test('a sweep ending at the duty bound with an idle power limiter is a normal MPP', async () => {
  const { window } = await connected({ sweepStop: FULL });
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('sweep');
  const res = card(window, 'sweep').querySelector('.tst-res');
  assert.ok(res.classList.contains('pass'));
  assert.match(res.querySelector('strong').textContent, /^MPP /);
  assert.doesNotMatch(res.textContent, /Stopped after|Stop point/);
});

test('sweep stop lines: limiter names, normal ends, an unverified landing', async () => {
  const { window } = await loadEditor({ teledec: true });
  const stop = (lim, tgt, act = tgt) => window.tstSummary('sweep', [I('mppt', `Stop sweep 4.20s mode=CV (lim=${lim} tgt=${tgt} act=${act}) PWM=1 MPP=(9.0W,1,30.0V)`), ...SWEEP.lines]);
  assert.match(stop(1, '28.80', '28.79').details[0], /by the output-voltage limit \(28\.8 V\)/);
  assert.match(stop(2, '9.00').details[0], /by the input-current limit \(9\.0 A\)/);
  assert.match(stop(4, '400.00', '396.10').details[0], /by the power limit \(400 W\)/);
  assert.match(stop(3, '2.00').text, /^Stop point /);
  assert.ok(stop(3, '2.00').curve.stop);
  const normal = [[0, '20.00'], [-1, '20.00', '1.00'], [-1, 'nan', 'nan'], [4, '800.00', '142.80'], [1, '28.80', '27.50'],
    [3, '2.00', '1.95']];
  for (const [lim, tgt, act] of normal) {
    const s = stop(lim, tgt, act);
    assert.match(s.text, /^MPP /, `lim=${lim} tgt=${tgt} act=${act} ends a sweep normally`);
    assert.equal(s.cls, 'pass');
    assert.equal(s.details.length, 0);
    assert.ok(!s.curve.stop);
  }
  const fig = window.document.createElement('figure');
  window.tstChart(fig, stop(3, '2.00').curve, 520);
  assert.match(fig.querySelector('svg').getAttribute('aria-label'), /stop point \d/);
  const nv = window.tstSummary('sweep', [...SWEEP.lines, I('mppt', 'sweep not verified (limiter)')]);
  assert.match(nv.details.join(' '), /The landing was not verified: a limiter was active\./);
});

test('sweep chart: none since boot, in progress, flagged, few points, degenerate ranges, incomplete', async () => {
  let { window } = await connected({ noCurve: true });
  await openSheet(window); await idle(window);
  card(window, 'sweep').querySelector('.read').click();
  await idle(window);
  assert.match(resText(window, 'sweep'), /No sweep recorded since boot/);
  assert.equal(chart(window).hidden, true);

  const run = drawn(window, sweepReply({ sweeping: true, n: 10 }).lines);
  assert.equal(run.cap.textContent, 'Sweep in progress, 10 points so far');
  assert.equal(run.svg.querySelectorAll('circle.pt').length, 10, 'few points are marked');
  assert.match(run.svg.getAttribute('aria-label'), /best so far/);

  const fl = drawn(window, sweepReply({ flag: 'light' }).lines);
  assert.equal(fl.sum.cls, 'warn');
  assert.ok(fl.svg.querySelector('path.ln').classList.contains('flag'));
  assert.match(fl.cap.textContent, /sky changed during sweep/);

  const one = drawn(window, sweepReply({ pts: [[30, 0]] }).lines);
  assert.equal(one.svg, null, 'one point is no curve');
  assert.match(one.sum.details.join(' '), /The device recorded only 1 point, so there is no curve\./);
  assert.doesNotMatch(one.sum.meta.join(' '), /1 points/);
  assert.equal(drawn(window, sweepReply({ pts: [[30, 0], [31, 0]] }).lines).svg, null, 'nor are two');
  const zero = drawn(window, sweepReply({ pts: [[30, 0], [31, 0], [32, 0]] }).lines);
  assert.equal(zero.svg.getAttribute('data-points'), '3');
  assert.equal(zero.svg.querySelectorAll('circle.pt').length, 3);
  assert.equal(zero.svg.querySelector('circle.mpp'), null, 'no MPP at 0 W');
  assert.match(zero.svg.getAttribute('aria-label'), /no maximum power point/);

  const flat = drawn(window, sweepReply({ pts: [[35, 100], [35, 100], [35, 100]] }).lines);
  assert.equal(flat.svg.querySelectorAll('circle.mpp').length, 1);
  const neg = drawn(window, sweepReply({ pts: [[40, -0.4], [38, -0.2], [36, 0]] }).lines);
  assert.ok(neg.svg.querySelector('path.ln'));

  const cut = SWEEP.lines.filter((_, i) => i !== 2);
  const inc = drawn(window, cut);
  assert.equal(inc.fig.hidden, true, 'an incomplete curve is not drawn');
  assert.match(inc.sum.details.join(' '), /curve incomplete \(75 of 87 points, short\): not drawn/);
  assert.match(inc.sum.text, new RegExp(`MPP ${SWEEP.P} W`));
});
