// Spec §6.1 (when a pushed change takes effect, svc commands, apply step after a push).
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadEditor } = require('./_setup');

function edit(window, key, value){
  const i = window.document.getElementById('f-' + key);
  i.value = value; i.dispatchEvent(new window.Event('input'));
}
const toggle = (window, key) => window.document.getElementById('f-' + key).click();
function showTab(window, path){ window._state.active = path; window.renderApp(); }

// MQTT devices on the stub broker. `restart` prints "Rebooting" and never answers;
// a command matching `err` is answered with ERR.
function mqttDevices(window, devs, err){
  const orig = window.mqtt.connect;
  window.mqtt.connect = (url, opts) => {
    const c = orig(url, opts), pub = c.publish.bind(c);
    c.publish = (topic, msg, o, cb) => {
      pub(topic, msg, o, cb);
      const m = topic.match(/^pv\/log\/([^/]+)\/cmd$/), conf = m && devs[m[1]];
      if (conf) setTimeout(() => {
        const say = s => c.emit('message', 'pv/log/' + m[1], new TextEncoder().encode(s + '\n'));
        let g;
        if (msg === 'hostname') say('Hostname: ' + m[1]);
        else if ((g = String(msg).match(/^get-config (\S+)$/))){
          for (const [k, v] of Object.entries(conf[g[1]] || {})) say(`Conf '/littlefs/conf/${g[1]}:${k}' = '${v}'`);
          say('OK: get-config ' + g[1]);
        } else if (msg === 'restart') say('I (5) main: Rebooting');
        else say((err && err.test(msg) ? 'ERR: ' : 'OK: ') + msg);
      }, 1);
      return c;
    };
    return c;
  };
  window.document.getElementById('mqtt-url').value = 'ws://broker:9001';
}
const sent = window => [...(window._mqttClients || [])].flatMap(c => [...c._publishes])
  .filter(p => !/^(hostname|get-config )/.test(p.msg)).map(p => p.topic.split('/')[2] + ': ' + p.msg);

const DEV = {
  'board.conf': { pwm_freq: '39000' },
  'mqtt.conf': { enabled: '1', broker_uri: 'mqtt://a', log_level: 'info' },
  'tele.conf': { enabled: '0', log_level: 'info', ble: '0' },
  'ftp.conf': { ftp_user: 'u' },
  'lcd.conf': { enabled: '1' },
};
async function connected(err){
  const { window } = await loadEditor();
  mqttDevices(window, { devA: DEV, devB: { 'board.conf': { pwm_freq: '40000' } } }, err);
  await window.mqttConnectAndImport('devA');
  return window;
}
const applyBar = window => window.document.getElementById('apply');
const applyButtons = window => [...applyBar(window).querySelectorAll('button')].map(b => b.textContent);

test('enabled/log_level go as svc commands; the review lists exactly what is sent', async () => {
  const window = await connected();
  showTab(window, 'conf/tele.conf');
  toggle(window, 'enabled');                  // 0 -> 1
  edit(window, 'log_level', 'warn');
  showTab(window, 'conf/mqtt.conf');
  toggle(window, 'enabled');                  // 1 -> 0 over MQTT: would cut the link, stays set-config
  edit(window, 'log_level', '');              // cleared: del-config
  showTab(window, 'conf/lcd.conf');
  edit(window, 'log_level', 'warn');          // lcd has no log_level yet: added

  const expect = ['set-config mqtt.conf enabled 0', 'del-config mqtt.conf log_level',
                  'svc on tele', 'svc log tele warn', 'svc log lcd warn'];
  window.openReview();
  const pre = window.document.querySelector('#review-body details.cmds pre').textContent.split('\n');
  assert.deepEqual([...pre].sort(), [...expect].sort());
  await window.uploadChanged({ reviewed: true });
  assert.deepEqual(sent(window).map(s => s.slice(6)).sort(), [...expect].sort());
  assert.equal(window.pendingCount(), 0, 'OK replies to svc commands clear the marks');
});

test('svc off for the link the editor is not on goes as svc off', async () => {
  const window = await connected();
  showTab(window, 'conf/lcd.conf');
  toggle(window, 'enabled');
  assert.deepEqual([...window.pendingCommands()].map(c => c.str), ['svc off lcd']);
});

test('the review shows when each change takes effect, and a summary', async () => {
  const window = await connected();
  showTab(window, 'conf/board.conf'); edit(window, 'pwm_freq', '40000');
  showTab(window, 'conf/mqtt.conf'); edit(window, 'broker_uri', 'mqtt://b');
  showTab(window, 'conf/tele.conf'); toggle(window, 'enabled'); toggle(window, 'ble');
  showTab(window, 'conf/ftp.conf'); edit(window, 'ftp_user', 'v');
  window.openReview();
  const hints = Object.fromEntries([...window.document.querySelectorAll('#review-body .diff')]
    .map(d => [d.closest('section').querySelector('h3').firstChild.textContent + ':' + d.querySelector('.dk').textContent,
               d.querySelector('.when').textContent]));
  assert.deepEqual(hints, {
    'board.conf:pwm_freq': 'after reboot',
    'mqtt.conf:broker_uri': 'after restarting mqtt',
    'tele.conf:enabled': 'now',
    'tele.conf:ble': 'when next used',
    'ftp.conf:ftp_user': 'after restarting ftp and http (ignored while credentials are stored in NVS)',
  });
  assert.equal(window.document.getElementById('review-when').textContent,
    'Takes effect after: restarting mqtt, restarting ftp, restarting http, reboot');
});

test('no hints without a push target', async () => {
  const { window } = await loadEditor();
  window.load({ 'conf/mqtt.conf': new TextEncoder().encode('broker_uri=mqtt://a\n') }, 'demo');
  edit(window, 'broker_uri', 'mqtt://b');
  window.openReview();
  assert.equal(window.document.querySelector('#review-body .when'), null);
  assert.equal(window.document.getElementById('review-when').textContent, '');
});

test('after a clean push: restart buttons for exactly the services needed', async () => {
  const window = await connected();
  assert.equal(applyBar(window).hidden, true);
  showTab(window, 'conf/mqtt.conf'); edit(window, 'broker_uri', 'mqtt://b');
  showTab(window, 'conf/ftp.conf'); edit(window, 'ftp_user', 'v');
  showTab(window, 'conf/tele.conf'); toggle(window, 'enabled');     // svc on: nothing to apply
  window.openReview();
  assert.equal(applyBar(window).hidden, true, 'nothing offered before the push');
  await window.uploadChanged({ reviewed: true });
  assert.equal(applyBar(window).hidden, false);
  assert.deepEqual(applyButtons(window), ['Restart mqtt', 'Restart ftp', 'Restart http']);
  assert.match(window.document.getElementById('serial-status').textContent, /takes effect after: restarting mqtt, restarting ftp, restarting http/);

  // a restart that does not touch the link: no confirm, OK removes the button
  let asked = 0; window.confirm = () => { asked++; return true; };
  await window.applyStep('ftp');
  assert.equal(asked, 0);
  assert.ok(sent(window).includes('devA: svc restart ftp'));
  assert.deepEqual(applyButtons(window), ['Restart mqtt', 'Restart http']);

  // the editor's own link (mqtt over MQTT): confirm; OK confirms the restart, the editor then waits
  // for `svc list` (this fake has none) and closes the link
  Object.assign(window.svcTimings(), { retry: 10, tries: 2 });
  let prompt = ''; window.confirm = m => { prompt = m; return true; };
  await window.applyStep('mqtt');
  assert.match(prompt, /connected through mqtt, so its connection drops/);
  assert.equal(window.document.getElementById('conn').textContent, 'Disconnected from devA over MQTT');
  assert.deepEqual(applyButtons(window), ['Restart http']);
});

test('a reboot covers the restarts, and is sent only after confirm', async () => {
  const window = await connected();
  showTab(window, 'conf/mqtt.conf'); edit(window, 'broker_uri', 'mqtt://b');
  showTab(window, 'conf/board.conf'); edit(window, 'pwm_freq', '40000');
  await window.uploadChanged({ reviewed: true });
  assert.deepEqual(applyButtons(window), ['Reboot device']);
  assert.equal(applyBar(window).firstElementChild.textContent, 'Takes effect after: reboot');

  let prompt = ''; window.confirm = m => { prompt = m; return false; };
  applyBar(window).querySelector('button').click();
  await new Promise(r => setTimeout(r, 20));
  assert.match(prompt, /^Reboot devA over MQTT\?/);
  assert.match(prompt, /stops converting/);
  assert.match(prompt, /connection drops/);
  assert.ok(!sent(window).includes('devA: restart'), 'declined: nothing sent');

  window.confirm = () => true;
  await window.applyStep('');
  assert.ok(sent(window).includes('devA: restart'));
  assert.ok(!sent(window).some(s => /svc restart/.test(s)));
  assert.equal(applyBar(window).hidden, true);
  assert.equal(window.document.getElementById('conn').textContent, 'Disconnected from devA over MQTT');
  assert.match(window.document.getElementById('serial-status').textContent, /devA is rebooting — reconnect from Open when it is back/);
});

test('no apply step when the push failed', async () => {
  const window = await connected(/^set-config mqtt\.conf/);
  showTab(window, 'conf/mqtt.conf'); edit(window, 'broker_uri', 'mqtt://b');
  showTab(window, 'conf/board.conf'); edit(window, 'pwm_freq', '40000');
  await window.uploadChanged({ reviewed: true });
  assert.equal(applyBar(window).hidden, true);
  assert.equal(applyButtons(window).length, 0);
});

test('apply is offered only for the device that was pushed to', async () => {
  const window = await connected();
  showTab(window, 'conf/board.conf'); edit(window, 'pwm_freq', '40000');
  await window.uploadChanged({ reviewed: true });
  assert.equal(applyBar(window).hidden, false);

  await window.mqttConnectAndImport('devB');
  assert.equal(window._state.deviceName, 'devB');
  assert.equal(applyBar(window).hidden, true);
  window.confirm = () => true;
  await window.applyStep('');
  assert.ok(!sent(window).some(s => s.startsWith('devB:')), JSON.stringify(sent(window)));
});
