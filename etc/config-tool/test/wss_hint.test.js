// MQTT failure messages (spec §5.3.1), in both tools: each names the broker, the stage and the next step,
// never a bare "connection closed". The certificate advice after a wss:// connect fails before CONNACK:
// not for a refused login, window.open only from the button's click, one repeat when the page regains
// focus, no loop. A drop while the tab is hidden is calm. Also: the two WssHint copies match.
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
// reports for an untrusted certificate), 'error' (a transport error, then close), 'timeout' (mqtt.js
// "connack timeout", then close), 'rc5' / 'rc4' (CONNACK refusal: error "Connection refused: …" with the
// return code, then close), 'connect'.
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
        if (o === 'timeout') emit('error', new Error('connack timeout'));
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
// WebSocket for the probe, how(url) or how: 'fail' (error, then close 1006 without opening: untrusted
// certificate, unreachable, blocked), 'open' (TLS and the upgrade work), 'hang'. calls: [url, protocol];
// closed: the URLs the page closed.
const withWs = (how, calls = [], closed = []) => (window) => {
  window.WebSocket = class {
    constructor(url, protocol) {
      calls.push([url, protocol]);
      this.url = url; this.readyState = 0;
      const o = typeof how === 'function' ? how(url) : how;
      if (o === 'hang') return;
      setTimeout(() => {
        if (this.readyState) return;
        if (o === 'open') { this.readyState = 1; if (this.onopen) this.onopen({}); return; }
        this.readyState = 3;
        if (this.onerror) this.onerror({});
        if (this.onclose) this.onclose({ code: 1006, wasClean: false });
      }, 1);
    }
    close() { closed.push(this.url); this.readyState = 3; }
  };
};
const both = (...fns) => (w) => fns.forEach(f => f(w));
function recordOpen(window) { const opened = []; window.open = (...a) => { opened.push(a); return null; }; return opened; }
const focus = (window) => window.dispatchEvent(new window.Event('focus'));
const clients = (window) => (window._mqttClients || []).length;
const BARE = /connection closed|^\s*(closed|error|disconnected)\b/i;
const hide = (w, hidden) => Object.defineProperty(w.document, 'visibilityState', { value: hidden ? 'hidden' : 'visible', configurable: true });

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
  assert.equal(H.refusedText(err('Connection refused: Not authorized', 5), WSS), 'broker.example:8884 refused the login (Not authorized). Check the user and password.');
  assert.equal(H.refusedText(err('Connection refused: Server unavailable', 3), 'ws://b.example:1885'),
    "b.example:1885 refused the MQTT connection (Server unavailable, code 3). Check the broker's log.");
  assert.equal(H.refusedText(err('connection closed')), null);
  assert.equal(H.addr('wss://b.example/mqtt'), 'b.example:443');
  assert.equal(H.addr('ws://b.example'), 'b.example:80');
  assert.equal(H.addr('ws://b.example:1885'), 'b.example:1885');
  assert.deepEqual([H.dur(4000), H.dur(89000), H.dur(180000), H.dur(7200000)], ['4 s', '89 s', '3 min', '2 h']);
  const now = Date.now();
  assert.equal(H.lostText(WSS, now - 180000, now - 65000, err('Keepalive timeout')),
    'lost broker.example:8884 after 3 min (last data 65 s ago): Keepalive timeout');
  assert.equal(H.lostText(WSS, now - 5000, 0), 'lost broker.example:8884 after 5 s (last data 5 s ago)');
  assert.deepEqual({ ...H.target('wss://b.example:8884/mqtt') }, { name: 'b.example:8884', href: 'https://b.example:8884/' });
  assert.deepEqual({ ...H.target('wss://b.example') }, { name: 'b.example:443', href: 'https://b.example:443/' });
  assert.deepEqual({ ...H.target('wss://[fd00::1]:8884') }, { name: '[fd00::1]:8884', href: 'https://[fd00::1]:8884/' });
  assert.equal(H.target('ws://b.example:1885'), null);
  assert.equal(H.target('nonsense'), null);
});

test('WssHint.probe: a bare WebSocket with subprotocol mqtt; open, closed (1006) or no answer, closed again', async () => {
  const calls = [], closed = [];
  const how = (u) => u.includes('open') ? 'open' : u.includes('hang') ? 'hang' : 'fail';
  const { window } = await loadEditor({ beforeParse: withWs(how, calls, closed) });
  const H = window.eval('WssHint');
  assert.equal(await H.probe('wss://open.example:8884/mqtt'), true);
  assert.deepEqual(calls[0], ['wss://open.example:8884/mqtt', 'mqtt']);
  assert.deepEqual(closed, ['wss://open.example:8884/mqtt'], 'closed right after it opened');
  assert.equal(await H.probe('wss://fail.example:8884'), false);
  assert.equal(await H.probe('wss://hang.example:8884', 30), null);
  assert.ok(closed.includes('wss://hang.example:8884'), 'a hanging probe is closed at the timeout');
  window.WebSocket = undefined;
  assert.equal(await H.probe('wss://open.example:8884'), null);
});

test('WssHint.explain: the stage, the broker and the next step for each failure before CONNACK', async () => {
  const how = (u) => u.includes('open') ? 'open' : u.includes('hang') ? 'hang' : 'fail';
  const { window } = await loadEditor({ beforeParse: withWs(how) });
  const H = window.eval('WssHint');
  const x = async (url, e, ms) => { const r = await H.explain(url, e, ms); return { ...r }; };
  assert.deepEqual(await x('wss://fail.example:8884'), { stage: 'socket', addr: 'fail.example:8884', href: 'https://fail.example:8884/',
    short: 'no secure connection', text: 'Could not open a secure connection to fail.example:8884. If the broker uses a private certificate, open it once and accept the warning, then come back.' });
  assert.deepEqual(await x('ws://fail.example:1885'), { stage: 'socket', addr: 'fail.example:1885', href: null, short: 'no WebSocket',
    text: 'Could not open a WebSocket to fail.example:1885. Check the address, the port and that the broker has a WebSocket listener there.' });
  assert.equal((await x('wss://open.example:8884')).text,
    "open.example:8884 accepts the secure WebSocket, but the MQTT connection failed: the broker closed it without answering. Check the broker's log.");
  assert.equal((await x('ws://open.example:1885', new Error('connack timeout'))).text,
    "open.example:1885 accepts the WebSocket, but the MQTT connection failed: no answer to the MQTT login in time. Check the broker's log.");
  const t = await x('wss://hang.example:8884', undefined, 1000);
  assert.equal(t.stage, 'timeout');
  assert.equal(t.text, 'No answer from hang.example:8884 within 1 s. Check the address and that the broker is running.');
  assert.equal((await x(WSS, Object.assign(new Error('Connection refused: Not authorized'), { code: 5 }))).stage, 'login');
  // the box: a button only for a wss:// certificate failure; the local-network note for socket and timeout
  const box = (r) => H.box(r, () => {});
  assert.ok(box(await H.explain('wss://fail.example:8884')).querySelector('button'));
  const tb = box(await H.explain('wss://hang.example:8884', undefined, 30));
  assert.equal(tb.querySelector('button'), null);
  assert.match(tb.textContent, /local network, allow it/);
  assert.doesNotMatch(tb.textContent, /after accepting/);
  const ob = box(await H.explain('wss://open.example:8884'));
  assert.equal(ob.querySelector('button'), null);
  assert.equal(ob.querySelectorAll('p').length, 1);
});

/* ---------------------------------------------------------------- editor --------------------- */

const $ = (w, id) => w.document.getElementById(id);
const hint = (w) => $(w, 'mqtt-hint');
const hintBtn = (w) => hint(w).querySelector('button');

async function scanEditor(plan, { url = WSS, wsHow = 'fail', calls } = {}) {
  const { window } = await loadEditor({ beforeParse: both(scripted(plan), withWs(wsHow, calls)) });
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
    assert.match(hint(w).textContent, /An empty or error page after accepting is normal/);
    assert.match(hint(w).textContent, /local network, allow it/);
    assert.equal(hintBtn(w).textContent, 'Open https://broker.example:8884');
    assert.deepEqual(calls, [[WSS, 'mqtt']], 'one probe: a WebSocket to the broker URL');
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

test('editor: ws:// gets the WebSocket text without a button; a refused login (rc 4 / 5) only its status', async () => {
  let w = await scanEditor(['close'], { url: 'ws://broker.example:1885' });
  await until(() => !hint(w).hidden, 2000, 'ws hint');
  assert.match(hint(w).textContent, /^Could not open a WebSocket to broker\.example:1885\. Check the address/);
  assert.equal(hintBtn(w), null);
  assert.equal($(w, 'mqtt-status').textContent, '');
  w.mqttModalClose();

  for (const [rc, why] of [['rc5', 'Not authorized'], ['rc4', 'Bad username or password']]) {
    w = await scanEditor([rc]);
    await until(() => /refused/.test($(w, 'mqtt-status').textContent), 2000, 'refused status');
    await settle(30);
    assert.equal($(w, 'mqtt-status').textContent, `broker.example:8884 refused the login (${why}). Check the user and password.`);
    assert.equal(hint(w).hidden, true);
    w.mqttModalClose();
  }
});

test('editor: a scan that worked, dropped: how long it ran, its last data, the reconnects; no advice', async () => {
  const w = await scanEditor(['connect', 'close', 'connect'], { wsHow: 'open' });
  try {
    await until(() => w._mqttClients[0] && w._mqttClients[0]._subscribes.length, 2000, 'scan up');
    w._mqttClients[0].emit('error', new Error('Keepalive timeout'));
    const st = () => $(w, 'mqtt-status').textContent;
    assert.match(st(), /^Lost broker\.example:8884 after 0 s \(last data 0 s ago\): Keepalive timeout, reconnecting every 3 s …$/);
    assert.equal($(w, 'mqtt-status').classList.contains('err'), true);
    await until(() => clients(w) === 2 && /last try/.test(st()), 5000, 'failed reconnect');
    assert.match(st(), /^Lost broker\.example:8884 after .*; reconnecting every 3 s \(last try: MQTT failed: the broker closed it without answering\) …$/);
    assert.doesNotMatch(st(), BARE);
    assert.equal(hint(w).hidden, true);
    await until(() => clients(w) === 3 && /listening/.test(st()), 5000, 'reconnected');
  } finally { w.mqttModalClose(); }
});

test('editor: a scan dropped while the tab is hidden: calm, not an error', async () => {
  const w = await scanEditor(['connect']);
  try {
    await until(() => w._mqttClients[0] && w._mqttClients[0]._subscribes.length, 2000, 'scan up');
    hide(w, true);
    w._mqttClients[0].emit('close');
    assert.equal($(w, 'mqtt-status').textContent, 'Paused in the background, reconnecting every 3 s …');
    assert.equal($(w, 'mqtt-status').classList.contains('err'), false);
  } finally { w.mqttModalClose(); }
});

test('editor: the probe WebSocket opens, so TLS works: the MQTT failure, no certificate advice, no button', async () => {
  let w = await scanEditor(['close'], { wsHow: 'open' });
  try {
    await until(() => !hint(w).hidden, 2000, 'hint');
    assert.equal(hint(w).textContent, "broker.example:8884 accepts the secure WebSocket, but the MQTT connection failed: the broker closed it without answering. Check the broker's log.");
    assert.equal(hintBtn(w), null);
  } finally { w.mqttModalClose(); }
  w = await scanEditor(['timeout'], { wsHow: 'open' });
  try {
    await until(() => !hint(w).hidden, 2000, 'hint');
    assert.equal(hint(w).textContent, "broker.example:8884 accepts the secure WebSocket, but the MQTT connection failed: no answer to the MQTT login in time. Check the broker's log.");
  } finally { w.mqttModalClose(); }
});

test('editor: a device from the list that fails to connect, and a device session that drops: no bare "connection closed"', async () => {
  const devA = firmware('devA');
  // the scan connects and lists devA; the scan client drops, so the pick needs a fresh connect, which fails
  const { window: w } = await loadEditor({ beforeParse: both(scripted(['connect', 'close']), withWs('fail')) });
  w.mqttModalOpen();
  $(w, 'mqtt-url').value = WSS;
  $(w, 'mqtt-scan').click();
  await until(() => w._mqttClients[0] && w._mqttClients[0]._subscribes.length, 2000, 'scan up');
  w._mqttClients[0].emit('message', 'pv/log/devA', 'x');
  await until(() => w.document.querySelector('#mqtt-devlist .dev'), 2000, 'listed');
  w._mqttClients[0].connected = false;
  w.document.querySelector('#mqtt-devlist .dev').click();
  await until(() => !hint(w).hidden, 4000, 'hint');
  assert.equal($(w, 'mqtt-status').textContent, 'Could not read devA.');
  assert.match(hint(w).textContent, /^Could not open a secure connection to broker\.example:8884\./);
  assert.match($(w, 'drop-msg').textContent, /^Could not open a secure connection to broker\.example:8884\./);
  for (const t of [$(w, 'mqtt-status').textContent, hint(w).textContent, $(w, 'drop-msg').textContent]) assert.doesNotMatch(t, BARE);
  w.mqttModalClose();

  // a read over a handoff, then the broker drops the session
  const { window: v } = await loadEditor({ url: 'http://localhost/#mqtt=' + encodeURIComponent(WSS) + '&device=devA',
    beforeParse: both(x => mqttHook(x, { devA }), withWs('fail')) });
  await until(() => v._state && v._state.fromDevice, 4000, 'read');
  v._mqttClients[0].emit('close');
  assert.match($(v, 'drop-msg').textContent, /^MQTT: lost broker\.example:8884 after \d+ s \(last data \d+ s ago\); the next push reconnects\.$/);
  assert.doesNotMatch($(v, 'drop-msg').textContent, BARE);
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
    beforeParse: both(scripted(['close', 'connect'], x => mqttHook(x, { devA })), withWs('fail')),
  });
  await until(() => !hint(w).hidden, 2000, 'hint');
  assert.equal($(w, 'mqtt-go').textContent, 'Read devA');
  assert.equal($(w, 'mqtt-status').textContent, 'Could not read devA.');
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

async function dash(plan, { storage = STORE, wsHow = 'fail', calls } = {}) {
  return loadDash({ storage, beforeParse: both(scripted(plan), withWs(wsHow, calls)) });
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
  assert.equal(dstatus(w), 'lost broker.example:8884 after 0 s (last data 0 s ago), retrying every 5 s…');
  assert.equal(banner(w), null);
});

test('dashboard: before CONNACK the status names the broker; after a drop, how long it ran and its last data', async () => {
  const { window: w } = await dash(['close', 'connect']);
  await until(() => banner(w), 2000, 'banner');
  assert.equal(dstatus(w), 'not connected to broker.example:8884, retrying every 5 s…');
  w._mqttClients[0].emit('close');   // a failed mqtt.js reconnect: the same status
  assert.equal(dstatus(w), 'not connected to broker.example:8884, retrying every 5 s…');
  w.document.getElementById('mqtt-connect').click(); w.document.getElementById('mqtt-connect').click();   // Disconnect, Connect
  assert.equal(clients(w), 2);
  await until(() => dstatus(w) === 'connected', 2000, 'connected');
  const c = w._mqttClients[1];
  c.emit('packetreceive');
  c.emit('error', new Error('Keepalive timeout'));
  c.emit('close');
  assert.equal(dstatus(w), 'lost broker.example:8884 after 0 s (last data 0 s ago): Keepalive timeout, retrying every 5 s…');
  c.emit('close');   // mqtt.js reconnects that fail keep the message
  assert.equal(dstatus(w), 'lost broker.example:8884 after 0 s (last data 0 s ago): Keepalive timeout, retrying every 5 s…');
  assert.equal(w.document.getElementById('mqtt-status').classList.contains('err'), true);
  assert.equal(banner(w), null);
  w.document.getElementById('mqtt-connect').click();
  assert.equal(dstatus(w), 'disconnected from broker.example:8884');
});

test('dashboard: a drop while the tab is hidden is calm and reconnects as soon as the tab is visible', async () => {
  const { window: w } = await dash(['connect']);
  await until(() => dstatus(w) === 'connected', 2000, 'connected');
  const c = w._mqttClients[0];
  hide(w, true);
  c.emit('error', new Error('Keepalive timeout'));
  c.emit('close');
  assert.equal(dstatus(w), 'paused in the background, reconnecting…');
  assert.equal(w.document.getElementById('mqtt-status').classList.contains('err'), false);
  assert.equal(banner(w), null);
  c.emit('close');
  assert.equal(dstatus(w), 'paused in the background, reconnecting…');
  assert.equal(c.reconnects, undefined);
  hide(w, false);
  w.document.dispatchEvent(new w.Event('visibilitychange'));
  assert.equal(c.reconnects, 1, 'reconnects at once');
  await until(() => dstatus(w) === 'connected', 2000, 'reconnected');
  assert.equal(clients(w), 1);
});

test('dashboard: a drop just after the tab came back from the background is calm too', async () => {
  const { window: w } = await dash(['connect']);
  await until(() => dstatus(w) === 'connected', 2000, 'connected');
  const c = w._mqttClients[0];
  await settle(5);
  hide(w, true); w.document.dispatchEvent(new w.Event('visibilitychange'));
  await settle(5);
  hide(w, false); w.document.dispatchEvent(new w.Event('visibilitychange'));
  c.emit('close');
  assert.equal(dstatus(w), 'paused in the background, reconnecting…');
  await until(() => c.reconnects === 1 && dstatus(w) === 'connected', 2000, 'reconnected');
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

test('dashboard: a refused login stops and says so; a ws:// failure gets the WebSocket banner without a button', async () => {
  let { window: w } = await dash(['rc5']);
  await until(() => /refused/.test(dstatus(w)), 2000, 'refused');
  await settle(30);
  assert.equal(dstatus(w), 'broker.example:8884 refused the login (Not authorized). Check the user and password.');
  assert.equal(banner(w), null);
  assert.equal(w.document.getElementById('mqtt-connect').textContent, 'Connect');
  assert.equal(w._mqttClients[0].ended, true);

  ({ window: w } = await dash(['rc4']));
  await until(() => /refused/.test(dstatus(w)), 2000, 'refused');
  assert.equal(dstatus(w), 'broker.example:8884 refused the login (Bad username or password). Check the user and password.');

  ({ window: w } = await dash(['close'], { storage: { ...STORE, 'mqtt-url': 'ws://broker.example:1885' } }));
  await until(() => banner(w), 2000, 'banner');
  assert.equal(dstatus(w), 'not connected to broker.example:1885, retrying every 5 s…');
  assert.match(banner(w).textContent, /^Could not open a WebSocket to broker\.example:1885\./);
  assert.equal(banner(w).querySelector('button'), null);
});

test('dashboard: the probe WebSocket opens: the banner names the MQTT failure, no button', async () => {
  const calls = [];
  let { window: w } = await dash(['close'], { wsHow: 'open', calls });
  await until(() => banner(w), 2000, 'banner');
  assert.equal(banner(w).textContent, "broker.example:8884 accepts the secure WebSocket, but the MQTT connection failed: the broker closed it without answering. Check the broker's log.");
  assert.equal(banner(w).querySelector('button'), null);
  assert.deepEqual(calls, [[WSS, 'mqtt']]);
  assert.doesNotMatch(dstatus(w), BARE);
  ({ window: w } = await dash(['timeout'], { wsHow: 'open' }));
  await until(() => banner(w), 2000, 'banner');
  assert.equal(banner(w).textContent, "broker.example:8884 accepts the secure WebSocket, but the MQTT connection failed: no answer to the MQTT login in time. Check the broker's log.");
  assert.equal(dstatus(w), 'not connected to broker.example:8884, retrying every 5 s…');
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
    beforeParse: both(scripted(['close']), withWs('fail')) });
  await settle(30);
  assert.equal(clients(w), 0);
  assert.equal(banner(w), null);
  ({ window: w } = await loadDash({ url: 'http://localhost/#mqtt=' + encodeURIComponent(WSS) + '&device=flat',
    beforeParse: both(scripted(['close']), withWs('fail')) }));
  await until(() => banner(w), 2000, 'banner');
  assert.equal(clients(w), 1);
});
