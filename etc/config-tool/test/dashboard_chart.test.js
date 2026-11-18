// dashboard.html P-vs-Vin chart and sweep tracking in jsdom.
const test = require('node:test');
const assert = require('node:assert/strict');
const { loadDash, closeAll, mqttMsg } = require('./_setup_dash');

test.after(closeAll);

const URL = 'ws://broker.test:1885';
const plain = (x) => JSON.parse(JSON.stringify(x));
const card = (window, host) => window.document.querySelector(`.card[data-host="${host}"]`);
const pubs = (client, topic) => [...client._publishes.filter(p => p.topic === topic).map(p => p.msg)];
const log = (client, host, lines) => client.emit('message', `pv/log/${host}`, Buffer.from(lines.map(l => l + '\n').join('')));

async function withBroker() {
  const env = await loadDash({ storage: { 'mqtt-url': URL } });
  return { ...env, client: env.window._mqttClients[0] };
}

test('relay-fed card fetches the sweep curve once 5 s after discovery', async () => {
  const { window, Dash, client } = await withBroker();
  let t = 1_763_400_000_000;
  window.Date.now = () => t;
  client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P: 1 }, ts: t }]));
  t += 5000; Dash.tick();
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), ['sweep-curve']);
  t += 5000; Dash.tick();
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), ['sweep-curve']);
});

test('chart: empty state, curve path, MPP ring, live dot, trace, caption, refresh', async () => {
  const { window, Dash, client } = await withBroker();
  log(client, 'fry', ['x']);
  Dash.flush();
  const svg = card(window, 'fry').querySelector('svg.chart');
  const caption = () => card(window, 'fry').querySelector('.chartbar .caption').textContent;
  assert.equal(svg.getAttribute('viewBox'), '0 0 320 170');
  assert.equal(svg.querySelector('text.empty').textContent, 'no sweep curve yet');
  assert.equal(svg.querySelector('text.empty').getAttribute('visibility'), 'visible');
  log(client, 'fry', ['sc: none']);
  Dash.flush();
  assert.equal(svg.querySelector('text.empty').textContent, 'no sweep recorded since boot');

  const t = Math.floor(Date.now() / 1000);
  log(client, 'fry', [`I (1) main: sc: t=${t} n=4 mpp=200.0,36.0 sweeping=0`,
    'sc: 44.0 10.0 40.0 150.0 36.0 200.0', 'sc: 30.0 120.0', 'sc: end']);
  Dash.flush();
  assert.equal(svg.querySelector('text.empty').getAttribute('visibility'), 'hidden');
  const d = svg.querySelector('path.curve').getAttribute('d');
  assert.equal((d.match(/[ML]/g) || []).length, 4);
  // y 0..200 -> ticks of 50; x 30..44 -> step 2
  assert.deepEqual(plain([...svg.querySelectorAll('g.ylab text')].map(e => e.textContent)),
    ['0', '50', '100', '150', '200 W']);
  const xl = [...svg.querySelectorAll('g.xlab text')].map(e => e.textContent);
  assert.equal(xl[0], '30');
  assert.equal(xl.at(-1), '44 V');
  assert.equal(svg.querySelectorAll('g.grid line').length, 4);
  const mpp = svg.querySelector('path.mpp');
  assert.equal(mpp.getAttribute('visibility'), 'visible');
  assert.equal(svg.querySelector('circle.live').getAttribute('visibility'), 'hidden');
  assert.match(caption(), /^sweep \d\d?:\d\d( [AP]M)? · MPP 200\.0 W @ 36\.0 V$/);

  // a live point inside the domain moves only the dot
  client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P: 180, Ui: 37 }, ts: Date.now() }]));
  Dash.flush();
  const live = svg.querySelector('circle.live');
  assert.equal(live.getAttribute('visibility'), 'visible');
  assert.ok(+live.getAttribute('cx') > +mpp.getAttribute('transform').match(/translate\(([\d.]+)/)[1]);
  assert.equal(svg.querySelector('path.curve').getAttribute('d'), d);

  // witnessed sweep trace (new symbol table per stream)
  const st = { syms: new Map() };
  client.emit('message', 'pv/tele/flat', mqttMsg([
    { fields: { P: 0, Ui: 45, mppt_state: 5 }, ts: Date.now() + 1 },
    { fields: { P: 100, Ui: 41 }, ts: Date.now() + 2 },
  ], st));
  Dash.flush();
  const fsvg = card(window, 'flat').querySelector('svg.chart');
  assert.equal((fsvg.querySelector('path.trace').getAttribute('d').match(/[ML]/g) || []).length, 2);
  assert.equal(fsvg.querySelector('path.curve').getAttribute('d'), '');
  assert.match(card(window, 'flat').querySelector('.chartbar .caption').textContent, /sweep in progress/);

  // refresh button sends sweep-curve and disables itself while in flight
  const n = pubs(client, 'pv/log/fry/cmd').filter(m => m === 'sweep-curve').length;
  card(window, 'fry').querySelector('button.refresh').click();
  assert.equal(pubs(client, 'pv/log/fry/cmd').filter(m => m === 'sweep-curve').length, n + 1);
  Dash.flush();
  assert.equal(card(window, 'fry').querySelector('button.refresh').disabled, true);
  assert.match(caption(), /loading/);
});

test('chart: y-axis top sticks at the session maximum and only grows', async () => {
  const { window, Dash, client } = await withBroker();
  const svg = () => card(window, 'fry').querySelector('svg.chart');
  const top = () => [...svg().querySelectorAll('g.ylab text')].at(-1).textContent;
  const st = { syms: new Map() };
  const tele = (fields, dt) => client.emit('message', 'pv/tele/fry', mqttMsg([{ fields, ts: Date.now() + dt }], st));
  tele({ P: 380, Ui: 40 }, 1);
  Dash.flush();
  assert.equal(top(), '400 W');

  const t = Math.floor(Date.now() / 1000);
  log(client, 'fry', [`sc: t=${t} n=3 mpp=60.0,36.0 sweeping=0`, 'sc: 44.0 10.0 40.0 50.0 36.0 60.0', 'sc: end']);
  tele({ P: 55, Ui: 37 }, 2);
  Dash.flush();
  assert.equal(top(), '400 W');

  tele({ P: 50, Ui: 37, P_max_today: 520 }, 3);
  Dash.flush();
  assert.equal(top(), '600 W');
  const y520 = [...svg().querySelectorAll('g.ylab text')].map(e => e.textContent);
  assert.deepEqual(plain(y520), ['0', '100', '200', '300', '400', '500', '600 W']);
});
