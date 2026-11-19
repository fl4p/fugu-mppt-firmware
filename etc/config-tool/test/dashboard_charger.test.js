// dashboard.html: charger state from the `status` reply (parser, triggers, card line, snapshot).
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadDash, closeAll, settle, until, makeFakeBleDevice, makeFakeBluetooth, mqttMsg } = require('./_setup_dash');

test.after(closeAll);

const URL = 'ws://broker.test:1885';
const card = (window, host) => window.document.querySelector(`.card[data-host="${host}"]`);
const plain = (x) => JSON.parse(JSON.stringify(x));
const cmds = (client, host) => client._publishes.filter(p => p.topic === `pv/log/${host}/cmd`).map(p => p.msg);
const nStatus = (client, host) => cmds(client, host).filter(m => m === 'status').length;
const log = (client, host, lines) => client.emit('message', `pv/log/${host}`, Buffer.from(lines.map(l => l + '\n').join('')));

const TERMINATED = [
  'Charger: TERMINATED (float)',
  '  Vbat_max=29.00V Vout_max=26.80V  Ibat_lim=39.0A Iout_max=39.0A',
  '  OV limit=derived from Vbat_max',
  '  v_term=3.366V cv_min=3.350 cv_eoc=3.500  Cbat=280.0Ah recharge_dod=0.15',
  '  BMS vcell_high=3.341V (ok, 1s ago)  ibat=1.48A (ok, 0s ago)  ahSinceFull=0.49Ah (bms)  vout_avg=26.78V',
  '  DoD since full: 0% / 15% to recharge',
  '  ceiling=80%  last full 3.2h ago (full charge every 7.0 d)',
  '  holiday off (auto)  quiet days 0/3 (gaps 0)  today: discharging 0.5h, mean 1.76/2.30A',
  '  pack temp 12.0..13.5°C (5s ago)  charge min 0 derate 5 max 45°C',
  'OK: status',
];
const CHARGING = [
  'Charger: charging',
  '  Vbat_max=29.00V Vout_max=28.40V  Ibat_lim=39.0A Iout_max=20.0A',
  'I (123) mppt: unrelated log line',
  '  OV limit=2.90V (explicit ovset)',
  '  BMS vcell_high=3.402V (stale/na, 300s ago)  ibat=0.00A (stale/na, 400s ago)  ahSinceFull=0.00Ah (ibat)  vout_avg=27.10V',
  'V=40.1/27.1 I=3.0/4.4A 120W st=MPPT,1',
  'OK: status',
];

async function withBroker(extra = {}) {
  const env = await loadDash({ storage: { 'mqtt-url': URL, ...extra } });
  return { ...env, client: env.window._mqttClients[0] };
}

test('parser: terminated sample with every optional line', async () => {
  const { Dash } = await loadDash({ mqtt: false });
  const c = plain(Dash.parseChargerStatus(TERMINATED));
  assert.ok(Number.isFinite(c.at));
  delete c.at;
  assert.deepEqual(c, {
    state: 'terminated', detail: 'float', vbatMax: 29, voutMax: 26.8, ibatLim: 39, ioutMax: 39,
    vcell: 3.341, vcellOk: true, vcellAgeS: 1, ibat: 1.48, ibatOk: true, ibatAgeS: 0,
    dod: 0, dodRecharge: 15, ceiling: 80, lastFullH: 3.2, fullEveryD: 7,
    holiday: 'off', holidayMode: 'auto', quietDays: 0, quietNeed: 3, packTempMin: 12, packTempMax: 13.5,
  });
});

test('parser: charging without DoD/ceiling/holiday/temp, stale BMS, interleaved lines; odd input does not throw', async () => {
  const { Dash } = await loadDash({ mqtt: false });
  const c = Dash.parseChargerStatus(CHARGING);
  assert.equal(c.state, 'charging');
  assert.equal(c.detail, undefined);
  assert.equal(c.voutMax, 28.4);
  assert.equal(c.vcell, 3.402);
  assert.equal(c.vcellOk, false);
  assert.equal(c.ibatOk, false);
  for (const k of ['dod', 'dodRecharge', 'ceiling', 'holiday', 'holidayMode', 'packTempMin', 'packTempMax'])
    assert.equal(c[k], undefined, k);
  assert.equal(Dash.parseChargerStatus(['\x1b[0;33mCharger: PARTIAL HOLD (load-following)\x1b[0m']).state, 'hold');
  assert.equal(Dash.parseChargerStatus(['Charger: BLOCKED (pack cold)']).detail, 'pack cold');
  assert.equal(Dash.parseChargerStatus(['  Vbat_max=nanV Vout_max=26.80V  Ibat_lim=infA Iout_max=1.0A']).vbatMax, undefined);
  for (const odd of [[], [''], [null], [undefined, 42], ['Charger: weird'], ['BMS vcell_high=']])
    assert.equal(Dash.parseChargerStatus(odd).state, undefined);
});

test('status after the first lease and after a broker reconnect; reply renders the .chg line', async () => {
  const { window, Dash, client } = await withBroker();
  let t = 1_763_400_000_000;
  window.Date.now = () => t;
  client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P: 1 }, ts: t }]));
  Dash.flush();
  assert.equal(card(window, 'fry').querySelector('.chg').hidden, true, 'hidden until the first reply');
  t += 5000; Dash.tick();
  assert.equal(nStatus(client, 'fry'), 1);
  t += 1000; Dash.tick();
  assert.equal(nStatus(client, 'fry'), 1);
  log(client, 'fry', TERMINATED);
  await settle();
  Dash.flush();
  const e = card(window, 'fry').querySelector('.chg');
  assert.equal(e.hidden, false);
  assert.equal(e.textContent, 'charger terminated (float) · cap 26.80 V · cell 3.341 V · DoD 0 % / 15 %');
  assert.equal(e.querySelector('span').className, 's-terminated');
  assert.equal(e.classList.contains('stale'), false);
  // reconnect re-requests
  client.emit('connect');
  assert.equal(nStatus(client, 'fry'), 2);
  log(client, 'fry', ['Charger: PARTIAL HOLD (load-following)', CHARGING[4],
    '  holiday ON (on)  quiet days 3/3 (gaps 0)  today: discharging 0.0h, mean 0.10/2.30A', 'OK: status']);
  await settle();
  Dash.flush();
  assert.equal(e.textContent, 'charger hold (load-following) · cell 3.402 V stale · holiday');
  assert.equal(e.querySelector('span').className, 's-hold');
  assert.equal(e.querySelector('.dim').textContent, 'cell 3.402 V stale');
});

test('status: every CFG.chargerMs from the tick, single in flight, one retry on a timeout, none without a link', async () => {
  const { window, Dash, client } = await withBroker();
  assert.equal(Dash.CFG.chargerMs, 300000);
  let t = 1_763_400_000_000;
  window.Date.now = () => t;
  Dash.CFG.chargerTimeoutMs = 5;
  log(client, 'fry', ['x']);
  const dev = Dash.devices.get('fry');
  assert.equal(nStatus(client, 'fry'), 0, 'not before the lease');
  t += 5000; Dash.tick();
  assert.equal(nStatus(client, 'fry'), 1);
  assert.equal(await Dash.requestChargerStatus(dev), false, 'single in flight');
  await settle(40);
  assert.equal(nStatus(client, 'fry'), 2, 'one retry');
  assert.equal(dev.chgReq, false);
  t += 100000; Dash.tick(); await settle(40);
  assert.equal(nStatus(client, 'fry'), 2);
  t += 200000; Dash.tick(); await settle(40);
  assert.equal(nStatus(client, 'fry'), 4, 'periodic request + its retry');
  // ERR is not retried
  Dash.CFG.chargerTimeoutMs = 1000;
  t += 300000; Dash.tick();
  log(client, 'fry', ['ERR: status']);
  await settle(5);
  assert.equal(nStatus(client, 'fry'), 5);
  assert.equal(dev.chg, null);
  // broker down: canSend false, nothing published
  client.emit('close');
  t += 300000; Dash.tick(); await settle(5);
  assert.equal(nStatus(client, 'fry'), 5);
});

test('BLE enable sequence asks for status; charging colour class; more info carries the rest', async () => {
  const dev = makeFakeBleDevice({ hostname: 'fbuck', name: 'fugu-fbuck',
    reply: (cmd) => cmd === 'status' ? TERMINATED.slice(0, -1).concat('OK: status') : null });
  const { window, Dash } = await loadDash({ bluetooth: makeFakeBluetooth([dev]), mqtt: false });
  await Dash.addBle();
  await until(() => Dash.devices.get('fbuck') && Dash.devices.get('fbuck').chg, 3000, 'status reply');
  assert.ok(dev.writes.includes('status'));
  Dash.flush();
  const more = card(window, 'fbuck').querySelector('details.more');
  more.open = true; await settle(5); Dash.flush();
  const info = more.querySelector('.info').textContent;
  assert.match(info, /Vbat_max 29\.00 V · Ibat_lim 39\.0 A/);
  assert.match(info, /ibat 1\.48 A/);
  assert.match(info, /ceiling 80 % · last full 3\.2 h ago/);
  assert.match(info, /holiday off \(auto\) · quiet days 0\/3/);
  assert.match(info, /pack 12\.0\.\.13\.5 °C/);
  Dash.devices.get('fbuck').chg = Dash.parseChargerStatus(CHARGING);
  Dash.flush(); Dash.ingest(Dash.devices.get('fbuck'), { fields: { P: 1 }, ts: 0 }, 'test'); Dash.flush();
  assert.equal(card(window, 'fbuck').querySelector('.chg span').className, 's-charging');
  Dash.devices.get('fbuck').chg = Dash.parseChargerStatus(['Charger: BLOCKED (pack cold)']);
  Dash.ingest(Dash.devices.get('fbuck'), { fields: { P: 2 }, ts: 0 }, 'test'); Dash.flush();
  assert.equal(card(window, 'fbuck').querySelector('.chg').textContent, 'charger blocked (pack cold)');
  assert.equal(card(window, 'fbuck').querySelector('.chg span').className, 's-blocked');
});

test('snapshot: dev.chg round trip, restored as stale until a fresh reply', async () => {
  const a = await withBroker();
  a.Dash.CFG.snapMs = 0;
  log(a.client, 'fry', ['x']);
  const dev = a.Dash.devices.get('fry');
  const p = a.Dash.requestChargerStatus(dev);
  log(a.client, 'fry', TERMINATED);
  assert.equal(await p, true);
  a.Dash.saveSnaps();
  const json = a.window.localStorage.getItem('dash-devices');
  assert.equal(JSON.parse(json).fry.chg.state, 'terminated');
  // a bogus stored state is dropped
  const bad = JSON.parse(json); bad.bad = { host: 'bad', seen: 1, chg: { state: '<b>x' } };
  const b = await loadDash({ storage: { 'dash-devices': JSON.stringify(bad) }, url: 'http://localhost/?b' });
  b.Dash.flush();
  const e = card(b.window, 'fry').querySelector('.chg');
  assert.equal(e.hidden, false);
  assert.equal(e.textContent, 'charger terminated (float) · cap 26.80 V · cell 3.341 V · DoD 0 % / 15 %');
  assert.equal(e.classList.contains('stale'), true);
  assert.equal(b.Dash.devices.get('bad').chg, null);
  assert.equal(card(b.window, 'bad').querySelector('.chg').hidden, true);
  // fresh reply over a new broker connection clears the stale mark
  b.Dash.mqttConnect(URL);
  await settle();
  const c2 = b.window._mqttClients[0];
  const fry = b.Dash.devices.get('fry');
  const q2 = b.Dash.requestChargerStatus(fry);
  log(c2, 'fry', CHARGING);
  assert.equal(await q2, true);
  b.Dash.flush();
  assert.equal(e.classList.contains('stale'), false);
  assert.match(e.textContent, /^charger charging · cap 28\.40 V/);
});
