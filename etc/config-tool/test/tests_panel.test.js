// Spec §2.15 (tests sheet): gating, the build probe, the Status rows read on open, collapsed action cards,
// confirms (plain and typed device name), refusals, result parsing per test, abort, link drop and timeout
// mid-test, the one-at-a-time guard and the sweep chart.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

const W = s => `\x1b[0;33mW (5000) main: ${s}\x1b[0m`;
const I = (tag, s) => `\x1b[0;32mI (5001) ${tag}: ${s}\x1b[0m`;
const vline = (p, mode = ' MPPT↑') => `V=36.1/26.80 I=${(p / 36.1).toFixed(2)}/4.31A ${p.toFixed(1)}W 31℃42℃ 450sps 12㎅/s CCM(H|L|Lm)=1234|2345|2400 st=${mode},1 lag=2100㎲ N=123 rssi=-60`;
const BLOCK = pre => `${pre}START blocked: Vin-Vout (Vin=6.8 Vout=26.6)`;
const HELP = ['help,?', 'restart,reset,reboot', 'mppt', 'sweep <value> <value> ...', 'sweep-curve', 'rect-cal <value> <value> ...',
  'bootinfo', 'adc-cal', 'uptime', 'status', 'wire', 'wire-reset', 'hostname,hn <value> <value> ...', 'service,svc <value> <value> ...',
  'selftest <value> <value> ...'];

// `sweep-curve` reply as src/cli.cpp cmdSweepCurve prints it: header (mpp=P,V), " %.2f %.1f" pairs, 12 per line.
// A one-diode panel model swept from near Voc down to 18 V.
function sweepReply({ n = 87, t = Math.floor(Date.now() / 1000) - 120, sweeping = false, flag = '0', pts } = {}) {
  pts = pts || Array.from({ length: n }, (_, i) => {
    const V = 43.2 - i * (25.2 / Math.max(1, n - 1));
    return [V, V * 9.4 * (1 - Math.exp((V - 43.6) / 2.4))];
  });
  const mp = pts.reduce((a, b) => (b[1] > a[1] ? b : a), [0, -Infinity]);
  const lines = [`sc: t=${sweeping ? 0 : t} n=${pts.length} mpp=${mp[1].toFixed(1)},${mp[0].toFixed(1)} sweeping=${+sweeping}`
    + (flag && !sweeping ? ` flag=${flag} land=${(mp[1] * (flag === 'light' ? 0.8 : 0.99)).toFixed(1)}` : '')];
  for (let i = 0; i < pts.length; i += 12)
    lines.push('sc:' + pts.slice(i, i + 12).map(([V, P]) => ` ${V.toFixed(2)} ${P.toFixed(1)}`).join(''));
  lines.push('sc: end');
  return { lines, n: pts.length, P: mp[1].toFixed(1), V: mp[0].toFixed(1) };
}
const SWEEP = sweepReply();

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
    const ok = () => { s(vline(st.power, opts.mode)); s('OK: ' + cmd); };
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
      (opts.curve || SWEEP).lines.forEach(x => s(x)); return ok();
    }
    if (cmd === 'sweep') {
      // src/cli.cpp cmdSweep: the queued line comes before OK; older firmware only logs START blocked
      if (opts.night === 'new') { s(BLOCK('sweep: queued, ')); return ok(); }
      ok();
      later(5, () => s(I('mppt', 'Start calibration')));
      later(10, () => s(vline(0, 'START')));                 // converter off during the zero calibration
      later(20, () => s(I('sampler', 'Calibration done!')));
      if (opts.night === 'old') return later(40, () => s(I('mppt', 'START blocked: Vin-Vout (Vin=6.8 Vout=26.6 ntc=18 mcu=28)')));
      if (opts.night === 'status') return later(60, () => s(vline(0, 'START')));
      if (opts.night) return;
      later(25, () => s(vline(0, 'START')));                 // just after it, before the converter starts
      later(30, () => opts.noMpp ? s(I('mppt', 'Stop sweep: no MPP (best 0.30W < 1.0W), backing off 60s'))
        : opts.sweepStop ? s(I('mppt', opts.sweepStop))
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
const row = (w, id) => $(w, `#tst-body .tst-row[data-tst="${id}"]`);
const rowText = (w, id) => row(w, id).querySelector('.tst-val').textContent;
const settle = (ms = 30) => new Promise(r => setTimeout(r, ms));
async function until(pred, ms = 3000) {
  const end = Date.now() + ms;
  while (!pred()) { if (Date.now() > end) throw new Error('timeout waiting'); await settle(5); }
}
const btn = w => w.document.getElementById('open-tests');
const resText = (w, id) => card(w, id).querySelector('.tst-res').textContent;
// rendered: no [hidden] ancestor, and not inside a closed <details> other than in its summary
const seen = el => !!el && !el.closest('[hidden]') && !(el.parentElement.closest('details:not([open])') && !el.closest('summary'));
const fold = (w, id) => card(w, id).querySelector('.tst-x').click();
// everything under the head line: result, chart, Check, output, details
const below = c => [...c.querySelectorAll('.tst-res, .tst-chart, .tst-log, .tst-out, .tst-det, .recheck')];

async function connected(opts = {}) {
  const { window } = await loadEditor({ teledec: opts.teledec !== false });
  const dev = firmware('fry', { conf: CONF, ...opts });
  mqttFirmware(window, { fry: dev, other: firmware('other', {}) });
  await window.mqttConnectAndImport('fry');
  Object.assign(window.tstTimings(), { cmd: 300, probe: 300, settle: 20, poll: 5 });
  return { window, dev };
}
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

test('Status rows: a silent command shows "no reply" with its note', async () => {
  const { window } = await connected({ silent: ['wire'] });
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

test('action cards: collapsed to the head line; a run, its result and its output fold with the card', async () => {
  const { window } = await connected({ coil: true });
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
    assert.ok(seen(c.querySelector('.tst-ctl .run')), id + ': primary button on the line');
  }
  assert.ok(seen(card(window, 'clear').querySelector('.tst-state')), 'the reason stays on the head line');
  const c = card(window, 'coil');
  c.querySelector('.tst-x').click();
  assert.ok(c.classList.contains('open'));
  assert.equal(c.querySelector('.tst-x').getAttribute('aria-expanded'), 'true');
  assert.equal(c.querySelector('.tst-more').hidden, false);
  assert.match(c.querySelector('.tst-det').textContent, /Save L0 to coil\.conf.*Risk.*no abort.*Needs.*Takes.*Sends.*measure-coil l0/);
  c.querySelector('.tst-what').click();
  assert.equal(c.classList.contains('open'), false, 'a click on the line folds it again');
  window.prompt = () => null;
  c.querySelector('.run').click();                         // a button does not toggle the fold
  await idle(window);
  assert.equal(c.classList.contains('open'), false, 'a cancelled start leaves the card folded');
  assert.match(resText(window, 'coil'), /Not started/);
  assert.equal(seen(c.querySelector('.tst-res')), false);

  window.prompt = () => 'fry';
  const b = card(window, 'bflow');
  const run = window.tstStart('bflow');
  await until(() => window.tstBusyNow() && b.querySelector('.tst-out').textContent.includes('forced PWM'));
  assert.ok(b.classList.contains('open'), 'a run unfolds its card');
  assert.ok(seen(b.querySelector('.tst-res')) && seen(b.querySelector('.tst-out')), 'live output in view');
  assert.equal(b.querySelector('.tst-log').open, true);
  fold(window, 'bflow');
  assert.equal(b.classList.contains('open'), false, 'a running card folds');
  for (const e of below(b)) assert.equal(seen(e), false, 'folded while running: ' + e.className);
  assert.ok(seen(b.querySelector('.abort')), 'Abort stays on the head line');
  assert.equal(b.querySelector('.tst-state').textContent, 'running …');
  await run;
  assert.ok(b.classList.contains('open'), 'the result unfolds the card once');
  assert.match(resText(window, 'bflow'), /PASS/);
  assert.ok(seen(b.querySelector('.tst-res')) && seen(b.querySelector('.tst-det')));
  assert.equal(b.querySelector('.tst-state').textContent, '');
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
  const run = window.tstStart('bflow');
  const b = card(window, 'bflow');
  await until(() => window.tstBusyNow() && b.querySelector('.tst-out').textContent.includes('started'));
  fold(window, 'bflow');
  const from = dev.sent.length;
  b.querySelector('.tst-head .abort').click();
  await until(() => dev.sent.length > from);
  assert.equal(dev.sent[from], 'selftest abort');
  assert.equal(b.classList.contains('open'), false, 'Abort does not unfold');
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
  const nWire = dev.sent.filter(c => c === 'wire').length;
  await window.tstRead('wire');
  assert.equal(dev.sent.filter(c => c === 'wire').length, nWire, 'a second test does not start');
  card(window, 'bflow').querySelector('.abort').click();
  await run;
  assert.equal(window.tstBusyNow(), false);
});

test('a status command the build lacks drops its row (help unavailable)', async () => {
  const { window, dev } = await connected({ helpSilent: true, absent: ['wire'] });
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

test('MPP sweep card: Read last sweep sends sweep-curve without a confirm and draws the P-V curve', async () => {
  const { window, dev } = await connected();
  await openSheet(window); await idle(window);
  assert.equal(card(window, 'curve'), null, 'one sweep card');
  const c = card(window, 'sweep');
  assert.equal(c.querySelector('.tst-x').textContent, 'MPP sweep');
  assert.equal(c.querySelector('.tst-tag').textContent, 'drives the converter');
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

test('sweep stop lines: limiter names, normal ends, an unverified landing', async () => {
  const { window } = await loadEditor({ teledec: true });
  const stop = (lim, tgt) => window.tstSummary('sweep', [I('mppt', `Stop sweep 4.20s mode=CV (lim=${lim} tgt=${tgt} act=1.00) PWM=1 MPP=(9.0W,1,30.0V)`), ...SWEEP.lines]);
  assert.match(stop(1, '28.80').details[0], /by the output-voltage limit \(28\.8 V\)/);
  assert.match(stop(2, '9.00').details[0], /by the input-current limit \(9\.0 A\)/);
  assert.match(stop(4, '400.00').details[0], /by the power limit \(400 W\)/);
  assert.match(stop(3, '2.00').text, /^Stop point /);
  assert.ok(stop(3, '2.00').curve.stop);
  for (const lim of [0, -1]) {
    const s = stop(lim, '20.00');
    assert.match(s.text, /^MPP /, `lim=${lim} ends a sweep normally`);
    assert.equal(s.details.length, 0);
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
