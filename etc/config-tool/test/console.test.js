// Spec §7 — device log: toggle attaches/detaches listener, ANSI rendering,
// tail-follow only when at the bottom, no-wrap CSS, command input through activeWrite.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor, fakeScrollGeometry } = require('./_setup');

const PANEL  = '#uploadlog';
const BODY   = '#uploadlog-body';
const INPUT  = '#uploadlog-cmd';

test('ulShow/ulHide attach and detach the console listener', async () => {
  const { window } = await loadEditor();
  const base = window._lineListeners.size;   // the live strip's listener stays attached
  assert.ok(!window._lineListeners.has(window.ulConsoleListener));
  window.ulShow();
  assert.equal(window._lineListeners.size, base + 1);
  assert.ok(window._lineListeners.has(window.ulConsoleListener));
  window.ulHide();
  assert.equal(window._lineListeners.size, base);
  assert.ok(!window._lineListeners.has(window.ulConsoleListener));
});

test('opening the panel renders ESP_LOG lines via the persistent listener', async () => {
  const { window } = await loadEditor();
  window.ulShow();
  window._dispatchLine('plain hello');
  window._dispatchLine('main: OK: hello');
  window._dispatchLine('main: ERR: nope');

  const rows = [...window.document.querySelectorAll(BODY + ' div')];
  assert.equal(rows.length, 3);
  assert.equal(rows[0].className, 'resp');
  assert.equal(rows[1].className, 'resp ok');
  assert.equal(rows[2].className, 'resp err');
});

test('ANSI-coloured lines go through ansiConvert.toHtml; plain lines do not', async () => {
  const { window } = await loadEditor();
  window.ulShow();
  window._dispatchLine('\x1b[0;32mI (123) main: hello\x1b[0m');
  window._dispatchLine('plain line');

  const rows = [...window.document.querySelectorAll(BODY + ' div')];
  assert.ok(rows[0].querySelector('span[style*="color"]'), 'ANSI line should produce a colour span');
  assert.equal(rows[1].querySelector('span[style*="color"]'), null, 'plain line should not');
});

test('tail-follow: auto-scroll only when the user is within 4 px of the bottom', async () => {
  const { window } = await loadEditor();
  const log = window.document.querySelector(PANEL);
  window.ulShow();

  // case 1: at bottom → scrollTop snaps to scrollHeight
  fakeScrollGeometry(log, { scrollHeight: 100, clientHeight: 40, scrollTop: 60 });
  window._dispatchLine('one');
  assert.equal(log.scrollTop, 100, 'should follow when at bottom');

  // case 2: scrolled up → scrollTop stays put
  fakeScrollGeometry(log, { scrollHeight: 100, clientHeight: 40, scrollTop: 0 });
  window._dispatchLine('two');
  assert.equal(log.scrollTop, 0, 'should not follow when scrolled away');
});

test('command input is disabled when no transport, enabled once activeWrite is set', async () => {
  const { window } = await loadEditor();
  window.ulRefreshInputState();
  assert.equal(window.document.querySelector(INPUT).disabled, true);

  window._activeWrite = async () => {};
  window.ulRefreshInputState();
  assert.equal(window.document.querySelector(INPUT).disabled, false);
});

test('submitting the input writes "<cmd>\\r\\n" through activeWrite and echoes "» cmd"', async () => {
  const { window } = await loadEditor();
  const sent = [];
  window._activeWrite = async (s) => { sent.push(s); };
  window.ulShow();

  const input = window.document.querySelector(INPUT);
  input.value = 'help';
  window.document.getElementById('uploadlog-input').dispatchEvent(new window.Event('submit', { cancelable: true }));
  await new Promise(r => setTimeout(r, 0));   // consoleSend is async

  assert.deepEqual(sent, ['help\r\n']);
  assert.equal(input.value, '', 'input should clear after submit');
  const lastSent = [...window.document.querySelectorAll(BODY + ' .sent')].pop();
  assert.match(lastSent.textContent, /^» help/);
});

test('log body uses white-space:pre so long lines do not wrap', async () => {
  const { window } = await loadEditor();
  // jsdom doesn't apply CSS, but it does parse it — read the stylesheet rules.
  const rules = [...window.document.styleSheets].flatMap(s => [...s.cssRules]);
  const want = rules.find(r => r.selectorText === '#uploadlog-body div');
  assert.ok(want, 'expected a rule for "#uploadlog-body div"');
  assert.equal(want.style.getPropertyValue('white-space'), 'pre');
});
