// Scripted fake firmware for the Tests sheet, shared by the jsdom suite (require) and the browser spec,
// which injects this file's text into the page. No require/import: it must run as a plain script.
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
// L0 sweep rows as src/selftest/measure_coil.cpp prints them: L falls 1.1 %/A with Ipk, a little noise, CCM at the end
function l0Rows() {
  const rows = [], vi = 36.1, vo = 26.8, fsw = 39000;
  for (let i = 0; i < 24; i++) {
    const H = 1200 + 30 * i, D = H / 4103, L = 50.6e-6 * (1 - 0.011 * ((vi - vo) * D / (fsw * 50.6e-6) - 1.4)) * (1 + 0.003 * Math.sin(i * 2.1));
    const io = (vi - vo) * vi * D * D / (2 * vo * fsw * L);
    rows.push(`  ${String(H).padStart(5)} ${D.toFixed(3).padStart(6)} ${vi.toFixed(2).padStart(6)} ${vo.toFixed(2).padStart(6)} ${io.toFixed(3).padStart(6)}  ${i < 23 ? 'DCM' : 'CCM'} ${(L * 1e6).toFixed(2).padStart(7)}`);
  }
  return [...rows, '  entered CCM; stopping'];
}

// An L0 run on a board with DCM switch-node ringing: RING_ROWS as a board printed them (each duty step catches the
// ring at another phase, so L swings ±8-15 % in a sawtooth), continued to Ipk 4.5 A, then the fit's lines.
const RING_ROWS = [
  '    374  0.091  73.07  26.78  0.221  DCM   60.95', '    394  0.096  73.06  26.79  0.235  DCM   63.43',
  '    414  0.101  73.05  26.78  0.242  DCM   68.10', '    434  0.106  73.02  26.78  0.249  DCM   72.56',
  '    454  0.111  73.02  26.79  0.258  DCM   76.66', '    474  0.116  73.00  26.79  0.263  DCM   81.86',
  '    494  0.120  72.98  26.79  0.274  DCM   85.22', '    514  0.125  72.95  26.78  0.285  DCM   88.91',
  '    534  0.130  72.86  26.78  0.310  DCM   87.83', '    554  0.135  72.69  26.78  0.352  DCM   82.84',
  '    574  0.140  72.35  26.77  0.444  DCM   69.52', '    594  0.145  72.09  26.79  0.503  DCM   65.12',
  '    614  0.150  71.99  26.78  0.530  DCM   65.83'];
function ringRun({ fit = 'ring' } = {}) {
  const fsw = 39000, Lt = i => 76 * (1 - 0.008 * i), rows = [...RING_ROWS], pts = [];
  for (const r of RING_ROWS) { const [, D, vi, vo, io, , L] = r.trim().split(/\s+/).map(Number); pts.push({ x: (vi - vo) * D / (fsw * L * 1e-6), io, L }); }
  let last = '';
  for (let i = 13; i < 64; i++) {
    const H = 374 + 20 * i, D = H / 4103, ph = ((i - 11) % 12) / 12, saw = ph < 8 / 12 ? -1 + 2 * ph * 12 / 8 : 1 - 2 * (ph - 8 / 12) * 3;
    const vo = 26.78 + 0.01 * Math.sin(i * 1.7);
    let vi = 72, L = 0, io = 0.53;
    for (let k = 0; k < 3; k++) {
      L = Lt((vi - vo) * D / (fsw * Lt(1) * 1e-6)) * (1 + 0.15 * Math.exp(-(i - 13) / 45) * saw);
      io = (vi - vo) * vi * D * D / (2 * vo * fsw * L * 1e-6); vi = 72 - 0.6 * (io - 0.53);
    }
    const x = (vi - vo) * D / (fsw * L * 1e-6);
    rows.push(`  ${String(H).padStart(5)} ${D.toFixed(3).padStart(6)} ${vi.toFixed(2).padStart(6)} ${vo.toFixed(2).padStart(6)} ${io.toFixed(3).padStart(6)}  DCM ${L.toFixed(2).padStart(7)}`);
    pts.push({ x, io, L });
    if (x > 4.5) { last = `  Ipk ${x.toFixed(2)} > 4.50; stopping`; break; }
  }
  const floor = Math.max(0.15, 0.2 * Math.max(...pts.map(q => q.io))), use = pts.filter(q => q.io >= floor);
  const lo = Math.min(...use.map(q => q.x)), hi = Math.max(...use.map(q => q.x)), e = [0, 1, 2, 3].map(j => lo + (hi - lo) * j / 3);
  const bins = [0, 1, 2].map(j => `${e[j].toFixed(1)}-${e[j + 1].toFixed(1)}A ${Lt((e[j] + e[j + 1]) / 2).toFixed(2)}uH+-0.6/1.2%`).join(' | ');
  const L = Lt(use.reduce((a, q) => a + q.x, 0) / use.length).toFixed(2), med = [...use].sort((a, b) => a.L - b.L)[use.length >> 1].L.toFixed(2);
  const l0 = ((fit === 'ring' ? L : med) * 1e-6).toExponential(6).replace(/e-(\d)$/, 'e-0$1');
  return ['measure-coil L0: fsw=39000 pwmMax=4103 band 374..1634 step 20  Vin=73.07 Vout=26.78 M=0.366',
    '      H      D    Vin   Vout   Iout mode    L_uH', ...rows, last,
    `measure-coil: L(Ipk), +-noise/offsets: ${bins}`,
    `measure-coil: L(Ipk) slope -0.80 +-0.40 %/A over ${lo.toFixed(1)}-${hi.toFixed(1)} A, Iout/on-time offsets add up to +-0.60: coil -1.80..+0.00 %/A, drop <= 1.8 %/A`,
    fit === 'ring' ? `measure-coil: L = ${L} uH  (ring fit, ${use.length} DCM pts, Iout>=${floor.toFixed(2)}A, +-0.6% noise +-1.2% syst; ring 310 kHz = ${(1e9 / (39.478418 * 310e3 ** 2 * L * 1e-6)).toFixed(1)} nF node C, ripple +-14%, 3.4 periods; median of points ${med} uH)`
      : `measure-coil: L = ${med} uH  (median of ${use.length} DCM pts, Iout>=${floor.toFixed(2)}A; ring not resolved: no periodic term, expect up to +-5%)`,
    'measure-coil: Iout zero +0.003/+0.004 A (off, before/after), subtracted', `  -> coil.conf L0=${l0}`];
}

// LS sweep rows: Iout peaks near 151 ct, the sweep stops past the peak
function lsRows() {
  const rows = [];
  for (let ls = 70; ls <= 175; ls += 5) {
    const io = 0.412 - 0.00012 * (ls - 151) ** 2;
    rows.push(`  ${String(ls).padStart(5)} ${io.toFixed(3).padStart(7)}  36.10  26.80  ${ls < 80 ? 'CCM' : 'DCM'}`);
  }
  return [...rows, '  passed peak; stopping'];
}

function firmware(host, opts = {}) {
  opts = { ...opts, conf: structuredClone(opts.conf || {}) };      // set-config writes to its own copy
  const sent = [];
  const st = { hsLatched: false, bfLatched: false, bflowRunning: false, bfAbort: null, power: opts.power ?? 115.5,
    lastBf: [], rect: { state: 'idle', unsaved: 0, ct: 20, ns: '500' },
    op: opts.op ?? 'mppt', ramp: opts.rampDown ?? 0, backoff: opts.backoff ?? 0, measuring: false, ...opts.state };   // op: mppt | manu | psu | duty (manual duty set)
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
    if ((g = cmd.match(/^set-config (\S+) (\S+) (.+)$/))) {
      if (opts.setRefuse) return err('set-config: ' + opts.setRefuse);
      (opts.conf[g[1]] = opts.conf[g[1]] || {})[g[2]] = g[3];
      s(I('main', `Set conf '/littlefs/conf/${g[1]}:${g[2]}' = '${g[3]}'`)); return s('OK: ' + cmd);
    }
    const root = cmd.split(' ')[0];
    if ((opts.absent || []).includes(root) || ![...HELP.map(h => h.split(/[ ,]/)[0]), 'measure-coil', 'svc', 'dc'].includes(root)
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
        else if (opts.bfLines) st.lastBf = opts.bfLines;
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
        s(I('rectcal', `state=${r.state}${r.why ? ': ' + r.why : ''} auto=0 needed=0 drv=mcpwm offset=${r.ct} ct (${r.ns} ns${r.cal ? ', ' + r.cal : ''}) backstop=0 unsaved=${r.unsaved}`));
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
      if (st.op === 'psu') return err('measure-coil: PSU/PV mode on, stop conversion first (dc 0)');
      if (st.op === 'mppt') return err('measure-coil: MPPT on, stop conversion first (dc 0)');
      if (st.op === 'duty') return err('measure-coil: manual duty 812 set, stop conversion first (dc 0)');
      if (st.ramp-- > 0) return err('measure-coil: converter still on, wait for the ramp-down');
      if (st.backoff-- > 0) return err('measure-coil: protection backoff');
      st.measuring = true;
      s('OK: ' + cmd);
      const apply = a[a.length - 1] === 'apply';
      const lines = opts.coilFail ? ['measure-coil: need Vin>Vout+1 (Vin=12.00 Vout=26.80)']
        : a[0] === 'l0' && opts.coilRing ? ringRun({ fit: opts.coilRing })
        : a[0] === 'l0' ? ['measure-coil L0: fsw=39000 pwmMax=4103 band 1200..2600 step 30  Vin=36.10 Vout=26.80 M=0.742',
          '      H      D    Vin   Vout   Iout mode    L_uH', ...l0Rows(),
          'measure-coil: L(Ipk), +-noise/offsets: 1.4-1.6A 50.62uH+-0.3/0.9% | 1.6-1.9A 50.40uH+-0.3/0.9% | 1.9-2.2A 50.15uH+-0.4/1.0%',
          'measure-coil: L(Ipk) slope -1.10 +-0.30 %/A over 1.4-2.2 A, Iout/on-time offsets add up to +-0.50: coil -2.20..+0.00 %/A, drop <= 2.2 %/A',
          'measure-coil: L = 50.34 uH  (ring fit, 31 DCM pts, Iout>=0.15A, +-0.4% noise +-1.1% syst; ring 310 kHz = 5.2 nF node C, ripple +-3%, 2.1 periods; median of points 50.40 uH)',
          'measure-coil: Iout zero +0.003/+0.004 A (off, before/after), subtracted', '  -> coil.conf L0=5.034000e-05   note: scales with Iout cal & pwmMax; dead-time/DCR ~few % low',
          ...(apply ? ['  applied coil.conf L0=5.034000e-05 (effective next boot)'] : [])]
        : ['measure-coil LS: HS=400 D=0.097 Vin=36.10 Vout=26.80 ideal_LS=139 auto_LS=130 sweep 70..195', '     LS    Iout    Vin   Vout mode', ...lsRows(),
          'measure-coil: peak LS 151 (Iout_peak 0.412 A at raw argmax 150, two-sided fit)',
          '  ideal LS 139  auto LS 130  offset peak-ideal +12 ct (+3.0% HS)  peak-auto +21 ct', '  L (peak curvature) = 49.8 uH (cross-check)',
          ...(apply ? ['  applied rect_offset 300 ns (0 ct, mcpwm); saved to coil.conf once the converter is off'] : [])];
      lines.forEach((x, i) => later(10 + (opts.rowMs ?? 5) * i, () => s(x)));
      if (!opts.coilNeverEnds) later(20 + (opts.rowMs ?? 5) * lines.length, () => { st.measuring = false; s('measure-coil: done, manual PWM at duty 0 (\'mppt\' resumes tracking)'); });
      return;
    }
    if (cmd === 'dc 0') {
      if (opts.dcRefuse) return err('dc: ' + opts.dcRefuse);
      st.op = 'manu'; s(I('main', 'Switched to manual PWM')); return ok();
    }
    if (cmd === 'mppt') {
      if (st.measuring) return err('mppt: refused, measure-coil running');
      if (st.op === 'mppt') return err('MPPT already enabled');
      st.op = 'mppt'; s(I('main', 'MPPT re-enabled')); return ok();
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

if (typeof module !== 'undefined') module.exports = { l0Rows, lsRows, RING_ROWS, ringRun, W, I, vline, BLOCK, HELP, sweepReply, SWEEP, firmware, mqttFirmware };
