// Spec §2.15 (tests sheet): gating, the build probe, confirms (plain and typed device name), refusals,
// result parsing per test, abort, link drop and timeout mid-test, and the one-at-a-time guard.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

const W = s => `\x1b[0;33mW (5000) main: ${s}\x1b[0m`;
const I = (tag, s) => `\x1b[0;32mI (5001) ${tag}: ${s}\x1b[0m`;
const vline = p => `V=36.1/26.80 I=${(p / 36.1).toFixed(2)}/4.31A ${p.toFixed(1)}W 31℃42℃ 450sps 12㎅/s CCM(H|L|Lm)=1234|2345|2400 st= MPPT↑,1 lag=2100㎲ N=123 rssi=-60`;
const HELP = ['help,?', 'restart,reset,reboot', 'mppt', 'sweep <value> <value> ...', 'sweep-curve', 'rect-cal <value> <value> ...',
  'bootinfo', 'adc-cal', 'uptime', 'status', 'wire', 'wire-reset', 'hostname,hn <value> <value> ...', 'service,svc <value> <value> ...',
  'selftest <value> <value> ...'];

// A scripted fake firmware. Async test output is emitted with setTimeout after the start marker.
function firmware(host, opts = {}) {
  const sent = [];
  const st = { hsLatched: false, bfLatched: false, bflowRunning: false, bfAbort: null, power: opts.power ?? 115.5,
    lastBf: [], rect: { state: 'idle', unsaved: 0, ct: 20, ns: '500' }, ...opts.state };
  let say = () => {};
  const later = (ms, fn) => setTimeout(fn, ms);
  const statusLines = () => {
    const out = [`hs-short: ${st.hsLatched ? 'LATCHED, gates off' : 'ok'}, idle check on, zero 0.012 A, start ${st.hsLatched ? 'blocked' : 'free'}`,
      opts.bfChecked === false ? 'bflow: not checked (board.conf bflow_bat=0)'
        : `bflow: ${st.bfLatched ? 'FAIL LATCHED' : 'ok'}; last run P/P 3.2 h ago; blocks pass 3.2 h ago (verified)${st.bflowRunning ? '; running' : ''}`];
    if (st.bflowRunning) out.push('selftest bflow: running'); else out.push(...st.lastBf);
    return out;
  };
  const handler = (cmd, s) => {
    say = s;
    sent.push(cmd);
    if ((opts.silent || []).includes(cmd)) return;
    const ok = () => { s(vline(st.power)); s('OK: ' + cmd); };
    const err = msg => { if (msg) s(W(msg)); s('ERR: ' + cmd); };
    let g;
    if (cmd === 'hostname') return s('Hostname: ' + host);
    if ((g = cmd.match(/^get-config (\S+)$/))) {
      for (const [k, v] of Object.entries((opts.conf || {})[g[1]] || {})) s(`I (100) main: Conf '/littlefs/conf/${g[1]}:${k}' = '${v}'`);
      return s('OK: get-config ' + g[1]);
    }
    const root = cmd.split(' ')[0];
    if ((opts.absent || []).includes(root) || ![...HELP.map(h => h.split(/[ ,]/)[0]), 'measure-coil', 'svc'].includes(root)
        || (root === 'measure-coil' && !opts.coil)) {
      s(`\x1b[0;31mE (5002) main: Command not found at '${cmd}'\x1b[0m`); return s('ERR: ' + cmd);
    }
    if (cmd === 'help') {
      if (opts.helpSilent) return;
      for (const h of HELP) if (!(opts.absent || []).includes(h.split(/[ ,]/)[0])) s(h);
      if (opts.coil && opts.coilInHelp !== false) s('measure-coil <value> <value> ...');
      return s('OK: help');
    }
    if (cmd === 'uptime') { s('uptime 1234 s'); if (!opts.measuring) s(vline(st.power)); return s('OK: uptime'); }
    if (cmd === 'selftest') { statusLines().forEach(x => s(x)); return ok(); }
    if (cmd === 'selftest bflow') {
      if (opts.bfRefuse) return err('selftest bflow: ' + opts.bfRefuse);
      st.bflowRunning = true;
      s('selftest bflow: started, usually 10-30 s, up to ~3 min with slow sensor filters'); ok();
      if (opts.bfNeverEnds) return;
      later(20, () => s('selftest bflow blocks: forced PWM from D=0.7412, I=1.20 A'));
      const finish = () => {
        st.bflowRunning = false;
        if (st.bfAbort) st.lastBf = ['selftest bflow blocks: ABORTED I=0.80 A (min 0.10) Vin=36.10 Vout=26.80 D=0.7300 dV=0.45', 'selftest bflow: aborted'];
        else if (opts.bfFail) { st.bfLatched = true; s('\x1b[0;31mE (6000) selftest: backflow switch FAILED (blocks F, conducts -): start blocked until `selftest clear`\x1b[0m');
          st.lastBf = ['selftest bflow blocks: FAIL I=-0.45 A (min -0.52) Vin=36.10 Vout=26.80 D=0.7100 dV=1.17 (reverse current)']; }
        else st.lastBf = ['selftest bflow blocks: PASS I=0.40 A (min 0.05) Vin=36.10 Vout=26.80 D=0.7100 dV=1.17',
          'selftest bflow conducts: PASS I=-0.62 A (min -0.62) Vin=36.10 Vout=26.80 D=0.7200 dV=0.80 (reverse current)'];
        st.lastBf.forEach(x => s(x));
      };
      const tick = n => { if (st.bfAbort || n === 0) finish(); else later(15, () => tick(n - 1)); };
      later(40, () => tick(opts.bfTicks ?? 3));
      return;
    }
    if (cmd === 'selftest abort') {
      if (!st.bflowRunning) return err('selftest abort: not running');
      st.bfAbort = true; s('selftest bflow: aborting'); return ok();
    }
    if (cmd === 'selftest clear') {
      if (!st.hsLatched && !st.bfLatched) return err('selftest: nothing latched');
      s(`selftest: cleared${st.hsLatched ? ' hs-short' : ''}${st.bfLatched ? ' bf-fail' : ''}`);
      st.hsLatched = st.bfLatched = false; return ok();
    }
    if (cmd === 'wire') {
      s('wire: armed, level ok (effective ok), wf=0 Pw=0.12W cap=30.0A');
      s('  baseline: 12 days, Rc 15.20 a 3.1mV Rb 0.00 Ro 0.00 mOhm, unverified days 0'); return ok();
    }
    if (cmd === 'sweep-curve') {
      if (opts.noCurve) { s('sc: none'); return ok(); }
      s(`sc: t=${Math.floor(Date.now() / 1000) - 120} n=87 mpp=312.4,35.2 sweeping=0 flag=0 land=309.0`);
      s('sc: 20.00 10.0 25.00 120.0'); s('sc: end'); return ok();
    }
    if (cmd === 'sweep') {
      ok();
      later(30, () => opts.noMpp ? s(I('mppt', 'Stop sweep: no MPP (best 0.30W < 1.0W), backing off 60s'))
        : s(I('mppt', 'sweep landed 35.0 V 309.0 W, MPP 35.2 V 312.4 W')));
      return;
    }
    if (cmd === 'adc-cal') {
      s('ADCCAL efuse_ver=1');
      for (let a = 0; a < 4; a++) s(`ADCCAL adc1 atten=${a} init_code=2100 cal_mv=850 cal_code=3000 gain_uv_per_code=${(283.33 + a).toFixed(2)}`);
      return ok();
    }
    if (root === 'rect-cal') {
      const sub = cmd.split(' ')[1] || '';
      if (sub === 'status') {
        const r = st.rect;
        s(I('rectcal', `state=${r.state}${r.why ? ': ' + r.why : ''} auto=0 needed=0 drv=mcpwm offset=${r.ct} ct (${r.ns} ns) backstop=0 unsaved=${r.unsaved}`));
        return ok();
      }
      if (sub === 'abort') { if (st.rect.state !== 'running') return err('rect-cal: not running'); st.rect.abort = true; return ok(); }
      if (opts.rcRefuse) return err('rect-cal: refused: ' + opts.rcRefuse);
      st.rect.state = 'running'; s(I('main', 'rect-cal: running at hs=812 (up to 60 s), see rect-cal status')); ok();
      later(40, () => {
        if (st.rect.abort) { st.rect = { state: 'failed', why: 'aborted', ct: 20, ns: '500', unsaved: 0 };
          return s(W('rectcal: failed: aborted (7 pts), offset stays 20 ct').replace('main: ', '')); }
        st.rect = { state: 'done', ct: 35, ns: '875', unsaved: 1 };
        s(I('rectcal', 'done: hs=812 ideal=900 zc=920 peak=935 learned=+35 ct, applied +35 ct (875 ns, mcpwm, 18 pts)'));
      });
      return;
    }
    if (root === 'measure-coil') {
      const a = cmd.split(' ').slice(1);
      if (!a.length) return err('measure-coil: expected l0|ls [args]');
      s('OK: ' + cmd);
      const apply = a[a.length - 1] === 'apply';
      const lines = opts.coilFail ? ['measure-coil: need Vin>Vout+1 (Vin=12.00 Vout=26.80)']
        : a[0] === 'l0' ? ['measure-coil L0: fsw=39000 pwmMax=4103 band 1200..2600 step 30  Vin=36.10 Vout=26.80 M=0.742',
          '      H      D    Vin   Vout   Iout mode    L_uH', '   1200  0.292  36.10  26.80  0.210  DCM   50.10',
          'measure-coil: L = 50.34 uH  (ring fit, 31 DCM pts, Iout>=0.15A, +-0.4% noise +-1.1% syst; ring 310 kHz = 5.2 nF node C, ripple +-3%, 2.1 periods; median of points 50.40 uH)',
          'measure-coil: Iout zero +0.003/+0.004 A (off, before/after), subtracted', '  -> coil.conf L0=5.034000e-05   note: scales with Iout cal & pwmMax; dead-time/DCR ~few % low',
          ...(apply ? ['  applied coil.conf L0=5.034000e-05 (effective next boot)'] : [])]
        : ['measure-coil LS: HS=400 D=0.097 Vin=36.10 Vout=26.80 ideal_LS=139 auto_LS=130 sweep 70..195', '     LS    Iout    Vin   Vout mode',
          'measure-coil: peak LS 151 (Iout_peak 0.412 A at raw argmax 150, two-sided fit)',
          '  ideal LS 139  auto LS 130  offset peak-ideal +12 ct (+3.0% HS)  peak-auto +21 ct', '  L (peak curvature) = 49.8 uH (cross-check)',
          ...(apply ? ['  applied rect_offset 300 ns (0 ct, mcpwm); saved to coil.conf once the converter is off'] : [])];
      lines.forEach((x, i) => later(10 + 5 * i, () => s(x)));
      if (!opts.coilNeverEnds) later(20 + 5 * lines.length, () => s('measure-coil: done, mode restored'));
      return;
    }
    if (root === 'svc') { s('NAME       STATE    LOG    ENABLED  DETAIL'); return ok(); }
    return err('unhandled in fake: ' + cmd);
  };
  return { handler, sent, st, say: x => say(x) };
}

function mqttFirmware(window, devs) {
  const orig = window.mqtt.connect;
  window.mqtt.connect = (url, o) => {
    const c = orig(url, o), pub = c.publish.bind(c);
    c.publish = (topic, msg, po, cb) => {
      pub(topic, msg, po, cb);
      const m = topic.match(/^pv\/log\/([^/]+)\/cmd$/), d = m && devs[m[1]];
      if (d) setTimeout(() => d.handler(String(msg),
        s => c.emit('message', 'pv/log/' + m[1], new TextEncoder().encode(s + '\n'))), 1);
      return c;
    };
    return c;
  };
  window.document.getElementById('mqtt-url').value = 'ws://broker:9001';
}

const CONF = { 'coil.conf': { L0: '50e-6', rect_offset_ns: '500' }, 'charger.conf': { vout_max: '28.8' } };
const $ = (w, sel) => w.document.querySelector(sel);
const card = (w, id) => $(w, `#tst-body .tst[data-tst="${id}"]`);
const settle = (ms = 30) => new Promise(r => setTimeout(r, ms));
async function until(pred, ms = 3000) {
  const end = Date.now() + ms;
  while (!pred()) { if (Date.now() > end) throw new Error('timeout waiting'); await settle(5); }
}
const btn = w => w.document.getElementById('open-tests');
const resText = (w, id) => card(w, id).querySelector('.tst-res').textContent;

async function connected(opts = {}) {
  const { window } = await loadEditor();
  const dev = firmware('fry', { conf: CONF, ...opts });
  mqttFirmware(window, { fry: dev, other: firmware('other', {}) });
  await window.mqttConnectAndImport('fry');
  Object.assign(window.tstTimings(), { cmd: 300, probe: 300, settle: 20, poll: 5 });
  return { window, dev };
}
async function openSheet(window) {
  btn(window).click();
  await until(() => card(window, 'selftest'));
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
  assert.equal(ab.cls, 'warn'); assert.match(ab.text, /No verdict: Vin dropped/);
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

test('opening probes the build only: help, then the bare measure-coil probe; absent tests are hidden', async () => {
  const { window, dev } = await connected();
  const from = dev.sent.length;
  await openSheet(window);
  await idle(window);
  assert.deepEqual(dev.sent.slice(from), ['help', 'measure-coil'], 'nothing but the probe is sent on open');
  assert.equal(card(window, 'coil'), null); assert.equal(card(window, 'coills'), null);
  assert.ok(card(window, 'adccal'), 'adc-cal is in help');
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
  assert.equal(card(window, 'adccal'), null);
  assert.match($(window, '#tst-body .tst-absent').textContent, /adc-cal \(ESP32-S3 builds\)/);
});

test('status cards read and summarise: selftest, rect-cal status, wire, sweep-curve, adc-cal', async () => {
  const { window, dev } = await connected();
  await openSheet(window); await idle(window);
  window.confirm = () => { throw new Error('read-only commands must not confirm'); };
  window.prompt = window.confirm;
  for (const [id, cmd, re] of [['selftest', 'selftest', /Nothing latched.*high-side short: ok.*backflow switch: ok; last run P\/P/],
    ['rcstatus', 'rect-cal status', /rect_offset 500 ns \(20 ct, mcpwm\), calibrated.*state idle/],
    ['wire', 'wire', /Wire guard armed, level ok.*baseline: 12 days/],
    ['curve', 'sweep-curve', /MPP 312.4 W at 35.2 V.*87 points, 2 min ago/],
    ['adccal', 'adc-cal', /eFuse calibration version 1.*atten 3: 286.33 µV\/code/]]) {
    const from = dev.sent.length;
    card(window, id).querySelector('.run').click();
    await idle(window);
    assert.deepEqual(dev.sent.slice(from), [cmd]);
    assert.match(resText(window, id), re, id);
    assert.ok(card(window, id).querySelector('.tst-out').textContent.length > 0, 'live output shown');
  }
  assert.ok(card(window, 'selftest').querySelector('.tst-res').classList.contains('pass'));
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
  await until(() => card(window, 'bflow').querySelector('.tst-out').textContent.includes('forced PWM from'));
  assert.ok(window.tstBusyNow());
  assert.equal(card(window, 'bflow').querySelector('.abort').hidden, false, 'Abort shown while running');
  assert.equal(card(window, 'sweep').querySelector('.run').disabled, true, 'one test at a time');
  await run;
  assert.deepEqual(dev.sent.slice(from + 1), ['selftest', 'selftest bflow', 'selftest']);
  const res = card(window, 'bflow').querySelector('.tst-res');
  assert.ok(res.classList.contains('pass'));
  assert.match(res.textContent, /Backflow switch PASS \(blocks, conducts\)/);
  assert.match(res.textContent, /conducts: PASS I=-0\.62 A/);
  assert.equal(card(window, 'bflow').querySelector('.abort').hidden, true);
  assert.equal(window.tstBusyNow(), false);
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
  assert.match(resText(window, 'selftest'), /Backflow switch failure latched/);
  assert.equal(card(window, 'clear').querySelector('.run').disabled, false);

  let prompt = '';
  window.prompt = m => { prompt = m; return 'fry'; };
  await window.tstStart('clear');
  assert.match(prompt, /Latched: backflow switch failure\./);
  assert.match(prompt, /may start again/);
  assert.ok(dev.sent.includes('selftest clear'));
  assert.match(resText(window, 'clear'), /Cleared bf-fail/);
  assert.match(resText(window, 'selftest'), /Nothing latched/);
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

test('measure-coil L0 with apply: result parsed, coil.conf L0 follows the device', async () => {
  const { window, dev } = await connected({ coil: true });
  await openSheet(window); await idle(window);
  const cb = card(window, 'coil').querySelector('.tst-opt input');
  cb.checked = true; cb.dispatchEvent(new window.Event('change'));
  assert.equal(card(window, 'coil').querySelector('.tst-cmd').textContent, 'measure-coil l0 apply');
  assert.equal(card(window, 'coil').querySelector('.abort'), null, 'no abort: the firmware has none');
  let prompt = '';
  window.prompt = m => { prompt = m; return 'fry'; };
  await window.tstStart('coil');
  assert.match(prompt, /There is no abort command/);
  assert.match(prompt, /written to coil\.conf L0/);
  assert.ok(dev.sent.includes('measure-coil l0 apply'));
  const res = card(window, 'coil').querySelector('.tst-res');
  assert.ok(res.classList.contains('pass'));
  assert.match(res.textContent, /L = 50\.34 µH/);
  assert.match(res.textContent, /ring fit, ±0\.4 % noise, ±1\.1 % syst/);
  assert.match(res.textContent, /saved to coil\.conf: L0=5\.034000e-05/);
  assert.equal(window._state.files['conf/coil.conf'].original.L0, '5.034000e-05');
  assert.equal(window.pendingCount(), 0);
});

test('measure-coil failure and the LS sweep', async () => {
  let { window } = await connected({ coil: true, coilFail: true });
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
  await window.tstStart('coil');
  assert.ok(card(window, 'coil').querySelector('.tst-res').classList.contains('fail'));
  assert.match(resText(window, 'coil'), /No result: need Vin>Vout\+1 \(Vin=12\.00 Vout=26\.80\)/);

  ({ window } = await connected({ coil: true }));
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
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

  ({ window, dev } = await connected());
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
  assert.match(resText(window, 'sweep'), /MPP 312\.4 W at 35\.2 V/);

  ({ window } = await connected({ noMpp: true }));
  await openSheet(window); await idle(window);
  window.confirm = () => true;
  await window.tstStart('sweep');
  assert.ok(card(window, 'sweep').querySelector('.tst-res').classList.contains('warn'));
  assert.match(resText(window, 'sweep'), /No MPP found.*best 0\.30W < 1\.0W/);
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

test('measure-coil link check after a timeout uses the status line', async () => {
  const { window } = await connected({ coil: true, coilNeverEnds: true, measuring: true });
  window.tstDefs().find(d => d.id === 'coil').timeout = 60;
  await openSheet(window); await idle(window);
  window.prompt = () => 'fry';
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
  await window.tstRead('wire');
  assert.ok(!dev.sent.includes('wire'), 'a second test does not start');
  card(window, 'bflow').querySelector('.abort').click();
  await run;
  assert.equal(window.tstBusyNow(), false);
});

test('a command missing at run time hides its card (help unavailable)', async () => {
  const { window, dev } = await connected({ helpSilent: true, absent: ['wire'] });
  await openSheet(window); await idle(window);
  assert.match($(window, '#tst-msg').textContent, /No reply to “help”/);
  assert.equal(card(window, 'adccal'), null, 'unknown build: adc-cal stays hidden');
  card(window, 'wire').querySelector('.run').click();
  await idle(window);
  assert.ok(dev.sent.includes('wire'));
  assert.equal(card(window, 'wire'), null);
  assert.match($(window, '#tst-msg').textContent, /“wire” is not in this firmware build/);
});

test('backflow card disabled on a board without bflow_bat; Escape and the scrim close the sheet', async () => {
  const { window } = await connected({ bfChecked: false });
  await openSheet(window); await idle(window);
  card(window, 'selftest').querySelector('.run').click();
  await idle(window);
  assert.equal(card(window, 'bflow').querySelector('.run').disabled, true);
  assert.match(card(window, 'bflow').querySelector('.run').title, /bflow_bat=0/);

  card(window, 'wire').querySelector('.run').focus();
  window.document.dispatchEvent(new window.KeyboardEvent('keydown', { key: 'Escape', bubbles: true }));
  assert.equal($(window, '#tests').classList.contains('open'), false);
  assert.equal(window.document.activeElement.id, 'open-tests');
  btn(window).click();
  assert.equal($(window, '#tests').classList.contains('open'), true);
  $(window, '#tst-scrim').click();
  assert.equal($(window, '#tests').classList.contains('open'), false);
});
