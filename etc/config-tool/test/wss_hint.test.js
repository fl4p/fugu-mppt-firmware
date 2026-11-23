// The certificate advice after a wss:// connect fails before CONNACK (spec §5.3.1), in both tools:
// shown only for wss:// before the first CONNACK and not for a refused login, window.open only from the
// button's click, one repeat when the page regains focus, no loop. Also: the two WssHint copies match.
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { loadEditor } = require('./_setup');
const { loadDash, closeAll, settle, until } = require('./_setup_dash');
const { firmware, mqttHook } = require('./_fakes');

test.after(closeAll);

const WSS = 'wss://broker.example:8884';
const HREF = 'https://broker.example:8884/';

// Each new client follows the next outcome of `plan` instead of connecting: 'close' (what mqtt.js
// reports for an untrusted certificate), 'error' (a transport error, then close), 'rc5' / 'rc4'
// (CONNACK refusal: error "Connection refused: …" with the return code, then close), 'connect'.
function scripted(plan, inner) {
  return (window) => {
    if (inner) inner(window);
    const prev = window._mqttHook;
    window._mqttHook = (c) => {
      if (prev) prev(c);
      const o = plan.length ? plan.shift() : 'connect';
      const emit = c.emit.bind(c);
      c.emit = (ev, ...a) => {
        if (ev !== 'connect' || o === 'connect') return emit(ev, ...a);
        if (o === 'error') emit('error', new Error('WebSocket error'));
        if (o === 'rc4' || o === 'rc5') {
          const e = new Error('Connection refused: ' + (o === 'rc4' ? 'Bad username or password' : 'Not authorized'));
          e.code = o === 'rc4' ? 4 : 5;
          emit('error', e);
        }
        return emit('close');
      };
    };
  };
}
// fetch for the probe: 'reject' (untrusted certificate, unreachable, blocked), 'resolve' (TLS accepted)
const withFetch = (how, calls = []) => (window) => {
  window.fetch = (url, o) => { calls.push([url, o]); return how === 'resolve' ? Promise.resolve({ type: 'opaque' }) : Promise.reject(new TypeError('Failed to fetch')); };
};
const both = (...fns) => (w) => fns.forEach(f => f(w));
function recordOpen(window) { const opened = []; window.open = (...a) => { opened.push(a); return null; }; return opened; }
const focus = (window) => window.dispatchEvent(new window.Event('focus'));
const clients = (window) => (window._mqttClients || []).length;

/* ---------------------------------------------------------------- shared helper -------------- */

function wssBlock(file) {
  const s = fs.readFileSync(path.resolve(__dirname, '..', file), 'utf8');
  const a = s.indexOf('// WssHint: kept identical'), b = s.indexOf('// end WssHint');
  assert.ok(a >= 0 && b > a, file + ': WssHint block');
  return s.slice(a, b);
}

test('WssHint is identical in conf-editor.html and dashboard.html', () => {
  assert.equal(wssBlock('conf-editor.html'), wssBlock('dashboard.html'));
});

test('WssHint.refusal / target: CONNACK codes and messages, wss:// only', async () => {
  const { window } = await loadEditor();
  const H = window.eval('WssHint');
  const err = (m, code) => Object.assign(new Error(m), code === undefined ? {} : { code });
  assert.equal(H.refusal(err('Connection refused: Not authorized', 5)), 'login');
  assert.equal(H.refusal(err('Connection refused: Bad username or password', 4)), 'login');
  assert.equal(H.refusal(err('x', 134)), 'login');
  assert.equal(H.refusal(err('x', 135)), 'login');
  assert.equal(H.refusal(err('Connection refused: Not authorized')), 'login');
  assert.equal(H.refusal(err('Connection refused: Server unavailable', 3)), 'other');
  assert.equal(H.refusal(err('connection closed')), null);
  assert.equal(H.refusal(err('connect ECONNREFUSED', 'ECONNREFUSED')), null);
  assert.equal(H.refusal(undefined), null);
  assert.equal(H.refusedText(err('Connection refused: Not authorized', 5)), 'the broker refused the login (Not authorized)');
  assert.deepEqual({ ...H.target('wss://b.example:8884/mqtt') }, { name: 'b.example:8884', href: 'https://b.example:8884/' });
  assert.deepEqual({ ...H.target('wss://b.example') }, { name: 'b.example:443', href: 'https://b.example:443/' });
  assert.deepEqual({ ...H.target('wss://[fd00::1]:8884') }, { name: '[fd00::1]:8884', href: 'https://[fd00::1]:8884/' });
  assert.equal(H.target('ws://b.example:1885'), null);
  assert.equal(H.target('nonsense'), null);
});

/* ---------------------------------------------------------------- editor --------------------- */

const $ = (w, id) => w.document.getElementById(id);
const hint = (w) => $(w, 'mqtt-hint');
const hintBtn = (w) => hint(w).querySelector('button');

async function scanEditor(plan, { url = WSS, fetchHow = 'reject', calls } = {}) {
  const { window } = await loadEditor({ beforeParse: both(scripted(plan), withFetch(fetchHow, calls)) });
  window.mqttModalOpen();
  $(window, 'mqtt-url').value = url;
  $(window, 'mqtt-user').value = 'pv';
  $(window, 'mqtt-pass').value = 'secret';
  $(window, 'mqtt-scan').click();
  return window;
}

test('editor: wss:// closed before CONNACK shows the advice; the button opens the page, one retry on focus', async () => {
  const calls = [];
  const w = await scanEditor(['close', 'connect'], { calls });
  try {
    await until(() => !hint(w).hidden, 2000, 'hint');
    assert.match(hint(w).textContent, /^Could not open a secure connection to broker\.example:8884\. If the broker uses a private certificate, open it once and accept the warning, then come back\./);
    assert.match(hint(w).textContent, /local network, allow it/);
    assert.equal(hintBtn(w).textContent, 'Open https://broker.example:8884');
    assert.equal(calls.length, 1, 'one probe');
    assert.equal(calls[0][0], HREF);
    assert.equal(calls[0][1].mode, 'no-cors');
    assert.equal($(w, 'mqtt-status').textContent, '');
    const opened = recordOpen(w);
    // nothing retries or opens without the click
    focus(w); await settle(20);
    assert.equal(clients(w), 1);
    assert.equal(opened.length, 0);
    hintBtn(w).click();
    assert.deepEqual(opened, [[HREF, '_blank', 'noopener']]);
    assert.ok(!opened[0][0].includes('secret'));
    assert.equal(clients(w), 1, 'no retry before the page regains focus');
    focus(w);
    assert.equal(clients(w), 2, 'one retry on focus');
    assert.equal($(w, 'mqtt-status').textContent, 'Retrying broker.example:8884 …');
    assert.equal(w._mqttClients[1].opts.password, 'secret');
    await until(() => hint(w).hidden, 2000, 'hint cleared after connect');
    focus(w); await settle(20);
    assert.equal(clients(w), 2, 'no second retry');
  } finally { w.mqttModalClose(); }
});

test('editor: a retry that fails again keeps the advice and the button, and does not loop', async () => {
  const w = await scanEditor(['close', 'close', 'connect']);
  try {
    await until(() => !hint(w).hidden, 2000, 'hint');
    const opened = recordOpen(w);
    hintBtn(w).click();
    focus(w);
    assert.equal(clients(w), 2);
    await until(() => !hint(w).hidden && hintBtn(w) && $(w, 'mqtt-status').textContent === '', 2000, 'hint again');
    await settle(30);
    assert.ok(hintBtn(w), 'button still there');
    focus(w); w.document.dispatchEvent(new w.Event('visibilitychange')); await settle(30);
    assert.equal(clients(w), 2, 'no automatic retry without another click');
    hintBtn(w).click();
    assert.equal(opened.length, 2);
    focus(w);
    assert.equal(clients(w), 3);
    await until(() => hint(w).hidden, 2000, 'connected');
  } finally { w.mqttModalClose(); }
});

test('editor: no advice for ws://, a refused login (rc 4 / 5) or a transport error after connecting', async () => {
  let w = await scanEditor(['close'], { url: 'ws://broker.example:1885' });
  await until(() => /broker: connection closed/.test($(w, 'mqtt-status').textContent), 2000, 'ws status');
  await settle(30);
  assert.equal(hint(w).hidden, true);
  w.mqttModalClose();

  for (const [rc, why] of [['rc5', 'Not authorized'], ['rc4', 'Bad username or password']]) {
    w = await scanEditor([rc]);
    await until(() => /refused/.test($(w, 'mqtt-status').textContent), 2000, 'refused status');
    await settle(30);
    assert.equal($(w, 'mqtt-status').textContent, `The broker refused the login (${why})`);
    assert.equal(hint(w).hidden, true);
    w.mqttModalClose();
  }

  // connected scan, then the broker drops: reconnecting, no advice
  w = await scanEditor(['connect']);
  try {
    await until(() => w._mqttClients[0] && w._mqttClients[0]._subscribes.length, 2000, 'scan up');
    w._mqttClients[0].emit('close');
    await settle(30);
    assert.match($(w, 'mqtt-status').textContent, /reconnecting/);
    assert.equal(hint(w).hidden, true);
  } finally { w.mqttModalClose(); }
});

test('editor: the probe reaches https, so the WebSocket was refused: no certificate advice, no button', async () => {
  const w = await scanEditor(['close'], { fetchHow: 'resolve' });
  try {
    await until(() => !hint(w).hidden, 2000, 'hint');
    assert.match(hint(w).textContent, /broker\.example:8884 accepts a secure connection, but the broker did not accept the MQTT WebSocket/);
    assert.equal(hintBtn(w), null);
  } finally { w.mqttModalClose(); }
});

test('editor: closing the dialog after the click cancels the retry', async () => {
  const w = await scanEditor(['close', 'connect']);
  await until(() => !hint(w).hidden, 2000, 'hint');
  recordOpen(w);
  hintBtn(w).click();
  w.mqttModalClose();
  focus(w); await settle(20);
  assert.equal(clients(w), 1);
});

test('editor: wss:// handoff that fails before CONNACK reopens the dialog with the advice; the retry reads the device', async () => {
  const devA = firmware('devA');
  const { window: w } = await loadEditor({
    url: 'http://localhost/#mqtt=' + encodeURIComponent(WSS) + '&device=devA',
    beforeParse: both(scripted(['close', 'connect'], x => mqttHook(x, { devA })), withFetch('reject')),
  });
  await until(() => !hint(w).hidden, 2000, 'hint');
  assert.equal($(w, 'mqtt-go').textContent, 'Read devA');
  assert.match(hint(w).textContent, /Could not open a secure connection to broker\.example:8884/);
  const opened = recordOpen(w);
  hintBtn(w).click();
  assert.equal(opened.length, 1);
  w.document.dispatchEvent(new w.Event('visibilitychange'));
  await until(() => clients(w) === 2, 2000, 'retry');
  focus(w);
  await until(() => w._state && w._state.fromDevice, 4000, 'read');
  assert.equal($(w, 'mqtt-modal').classList.contains('open'), false);
  assert.equal(hint(w).hidden, true);
  assert.equal(clients(w), 2, 'one retry');
});

/* ---------------------------------------------------------------- dashboard ------------------ */

const STORE = { 'mqtt-url': WSS, 'mqtt-user': 'pv', 'dash-mqtt-pass': 'secret' };
const banner = (w) => w.document.querySelector('#banners .wss-hint');
const dstatus = (w) => w.document.getElementById('mqtt-status').textContent;

async function dash(plan, { storage = STORE, fetchHow = 'reject' } = {}) {
  return loadDash({ storage, beforeParse: both(scripted(plan), withFetch(fetchHow)) });
}

test('dashboard: wss:// closed before CONNACK shows the banner; the button opens the page, one retry on focus', async () => {
  const { window: w } = await dash(['close', 'connect']);
  await until(() => banner(w), 2000, 'banner');
  assert.match(banner(w).textContent, /^Could not open a secure connection to broker\.example:8884\. If the broker uses a private certificate/);
  const b = banner(w).querySelector('button');
  assert.equal(b.textContent, 'Open https://broker.example:8884');
  const opened = recordOpen(w);
  focus(w); await settle(20);
  assert.equal(clients(w), 1, 'no retry without the click');
  b.click();
  assert.deepEqual(opened, [[HREF, '_blank', 'noopener']]);
  assert.equal(clients(w), 1);
  focus(w);
  assert.equal(clients(w), 2, 'one retry on focus');
  assert.equal(w._mqttClients[1].url, WSS);
  assert.equal(w._mqttClients[1].opts.password, 'secret');
  assert.ok(banner(w), 'the advice stays while retrying');
  assert.equal(dstatus(w), 'retrying broker.example:8884…');
  await until(() => dstatus(w) === 'connected', 2000, 'connected');
  assert.equal(banner(w), null);
  focus(w); await settle(20);
  assert.equal(clients(w), 2, 'no second retry');
  // a drop after a successful connect: no advice
  w._mqttClients[1].emit('close');
  await settle(30);
  assert.equal(dstatus(w), 'disconnected, retrying…');
  assert.equal(banner(w), null);
});

test('dashboard: a retry that fails again keeps the banner and the button, no loop', async () => {
  const { window: w } = await dash(['close', 'close']);
  await until(() => banner(w), 2000, 'banner');
  recordOpen(w);
  banner(w).querySelector('button').click();
  focus(w);
  assert.equal(clients(w), 2);
  await settle(50);
  assert.ok(banner(w) && banner(w).querySelector('button'), 'banner and button stay');
  focus(w); w.document.dispatchEvent(new w.Event('visibilitychange')); await settle(30);
  assert.equal(clients(w), 2, 'no automatic retry without another click');
});

test('dashboard: a refused login stops and says so; ws:// failures get no banner', async () => {
  let { window: w } = await dash(['rc5']);
  await until(() => /refused/.test(dstatus(w)), 2000, 'refused');
  await settle(30);
  assert.equal(dstatus(w), 'the broker refused the login (Not authorized): check the user and password');
  assert.equal(banner(w), null);
  assert.equal(w.document.getElementById('mqtt-connect').textContent, 'Connect');
  assert.equal(w._mqttClients[0].ended, true);

  ({ window: w } = await dash(['rc4']));
  await until(() => /refused/.test(dstatus(w)), 2000, 'refused');
  assert.equal(dstatus(w), 'the broker refused the login (Bad username or password): check the user and password');

  ({ window: w } = await dash(['close'], { storage: { ...STORE, 'mqtt-url': 'ws://broker.example:1885' } }));
  await until(() => /retrying/.test(dstatus(w)), 2000, 'closed');
  await settle(30);
  assert.equal(banner(w), null);
});

test('dashboard: probe reaches https: the banner says the WebSocket was refused, no button', async () => {
  const { window: w } = await dash(['close'], { fetchHow: 'resolve' });
  await until(() => banner(w), 2000, 'banner');
  assert.match(banner(w).textContent, /accepts a secure connection, but the broker did not accept the MQTT WebSocket/);
  assert.equal(banner(w).querySelector('button'), null);
});

test('dashboard: Disconnect or another URL drops the banner and the pending retry', async () => {
  const { window: w } = await dash(['close', 'connect']);
  await until(() => banner(w), 2000, 'banner');
  recordOpen(w);
  banner(w).querySelector('button').click();
  const url = w.document.getElementById('mqtt-url');
  url.value = 'wss://other.example:8884'; url.dispatchEvent(new w.Event('input'));
  assert.equal(banner(w), null, 'hidden for another URL');
  w.document.getElementById('mqtt-connect').click();   // Disconnect
  focus(w); await settle(20);
  assert.equal(clients(w), 1, 'no retry after Disconnect');
});

test('dashboard: ?mqtt= prefill does not connect; the #mqtt= handoff connects and gets the advice', async () => {
  let { window: w } = await loadDash({ url: 'http://localhost/?mqtt=' + encodeURIComponent(WSS) + '&user=pv',
    beforeParse: both(scripted(['close']), withFetch('reject')) });
  await settle(30);
  assert.equal(clients(w), 0);
  assert.equal(banner(w), null);
  ({ window: w } = await loadDash({ url: 'http://localhost/#mqtt=' + encodeURIComponent(WSS) + '&device=flat',
    beforeParse: both(scripted(['close']), withFetch('reject')) }));
  await until(() => banner(w), 2000, 'banner');
  assert.equal(clients(w), 1);
});
