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

test('relay-fed card fetches the sweep curve and leases once 5 s after discovery', async () => {
  const { window, Dash, client } = await withBroker();
  let t = 1_763_400_000_000;
  window.Date.now = () => t;
  client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P: 1 }, ts: t }]));
  t += 5000; Dash.tick();
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), ['tele-mqtt 120', 'sweep-curve']);
  t += 5000; Dash.tick();
  assert.deepEqual(pubs(client, 'pv/log/fry/cmd'), ['tele-mqtt 120', 'sweep-curve']);
});

test('chart: empty state, curve path, MPP ring, live dot, caption, refresh', async () => {
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
  // live dashes above the sweep curve, MPP star and live dot on top
  const order = [...svg.children].map(e => e.getAttribute('class'));
  const at = (c) => order.indexOf(c);
  assert.ok(at('curve') >= 0 && at('curve') < at('recon') && at('curve') < at('recon stale'), order.join());
  assert.ok(Math.max(at('recon'), at('recon stale')) < at('mpp-halo') && at('mpp-halo') < at('mpp') && at('mpp') < at('live'), order.join());
  assert.ok(+live.getAttribute('cx') > +mpp.getAttribute('transform').match(/translate\(([\d.]+)/)[1]);
  assert.equal(svg.querySelector('path.curve').getAttribute('d'), d);

  // sweep in progress (new symbol table per stream); every update is a live-curve bin
  const st = { syms: new Map() };
  client.emit('message', 'pv/tele/flat', mqttMsg([
    { fields: { P: 0, Ui: 45, mppt_state: 5 }, ts: Date.now() + 1 },
    { fields: { P: 100, Ui: 41 }, ts: Date.now() + 2 },
  ], st));
  Dash.flush();
  const fsvg = card(window, 'flat').querySelector('svg.chart');
  assert.equal((fsvg.querySelector('path.recon').getAttribute('d').match(/[ML]/g) || []).length, 2);
  assert.equal(fsvg.querySelector('path.curve').getAttribute('d'), '');
  assert.match(card(window, 'flat').querySelector('.chartbar .caption').textContent, /sweep in progress/);

  // refresh button sends sweep-curve and disables itself while in flight
  const n = pubs(client, 'pv/log/fry/cmd').filter(m => m === 'sweep-curve').length;
  card(window, 'fry').querySelector('button.refresh').click();
  assert.equal(pubs(client, 'pv/log/fry/cmd').filter(m => m === 'sweep-curve').length, n + 1);
  Dash.flush();
  assert.equal(card(window, 'fry').querySelector('button.refresh').disabled, true);
  assert.match(caption(), /loading/);

  // sweep button sends `sweep`; disabled while the device reports a sweep in progress
  card(window, 'fry').querySelector('button.sweep').click();
  assert.equal(pubs(client, 'pv/log/fry/cmd').filter(m => m === 'sweep').length, 1);
  assert.equal(card(window, 'flat').querySelector('button.sweep').disabled, true);
  assert.equal(card(window, 'fry').querySelector('button.sweep').disabled, false);
});

test('chart: y-axis top sticks at the plotted session maximum, ignores P_max_today', async () => {
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
  assert.equal(top(), '400 W');

  tele({ P: 510, Ui: 38 }, 4);
  Dash.flush();
  assert.equal(top(), '600 W');
});

test('chart: live P-V reconstruction bins Vin, decays toward new P, keeps and fades stale bins', async () => {
  const { window, Dash, client } = await withBroker();
  let t = 1_763_400_000_000;
  window.Date.now = () => t;
  const st = { syms: new Map() };
  const tele = (Ui, P) => client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P, Ui }, ts: t }], st));
  const svg = () => card(window, 'fry').querySelector('svg.chart');
  const verts = () => [...(svg().querySelector('path.recon').getAttribute('d') || '').matchAll(/[ML]([\d.]+) ([\d.]+)/g)]
    .map(m => [+m[1], +m[2]]);
  for (const [v, p] of [[40, 100], [41, 150], [42, 120]]) { t += 100; tele(v, p); }
  Dash.flush();
  const d = Dash.devices.get('fry');
  assert.equal(d.recon.size, 3);
  const { X, Y } = d.chartScale;
  const v = verts();
  assert.equal(v.length, 3);
  assert.deepEqual(plain(v), [[40, 100], [41, 150], [42, 120]].map(([V, P]) => [+X(V).toFixed(1), +Y(P).toFixed(1)]));
  assert.ok(card(window, 'fry').querySelector('.chartkeys .k-recon'));
  assert.equal(card(window, 'fry').querySelector('.chartkeys .k-trace'), null);

  // 3 min later a lower P at 41 V replaces the bin's value
  const b41 = d.recon.get(Math.round(41 / 0.25));
  t += 180000; tele(41, 60);
  assert.equal(b41.P, 60);
  Dash.flush();
  assert.equal(verts()[1][1], +d.chartScale.Y(b41.P).toFixed(1));

  // 11 min without updates: all bins persist but fade
  const stale = () => svg().querySelector('path.recon.stale').getAttribute('d') || '';
  assert.equal(stale(), '');
  t += 660000; Dash.tick(); Dash.flush();
  assert.equal(d.recon.size, 3);
  assert.equal(verts().length, 0);
  assert.equal((stale().match(/M/g) || []).length, 1);
  assert.equal((stale().match(/[ML]/g) || []).length, 3);

  // a segment is fresh only when both ends are: 42-43 V after updating 42 and adding 43
  t += 100; tele(42, 125); t += 100; tele(43, 90);
  Dash.flush();
  assert.equal(verts().length, 2);
  assert.equal(d.recon.size, 4);

  // cap: oldest-updated bin goes first, after hours nothing else expires
  t += 3_600_000; Dash.tick(); Dash.flush();
  assert.equal(d.recon.size, 4);
  for (let i = 0; i < 400; i++) { t += 10; tele(50 + i * 0.25, 10); }
  assert.equal(d.recon.size, 400);
  assert.ok(!d.recon.has(Math.round(40 / 0.25)) && d.recon.has(Math.round(50 / 0.25)));
});

test('chart: a live sample overwrites existing bins within ±0.5 V, creates only its own', async () => {
  const { window, Dash, client } = await withBroker();
  let t = 1_763_400_000_000;
  window.Date.now = () => t;
  const st = { syms: new Map() };
  const tele = (Ui, P) => client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P, Ui }, ts: t }], st));
  for (const v of [30.25, 30.5, 30.75, 31.25, 31.5]) { t += 100; tele(v, 250); }
  const d = Dash.devices.get('fry');
  const bin = (v) => d.recon.get(Math.round(v / 0.25));
  assert.equal(d.recon.size, 5);
  t += 60000; tele(31.0, 0);
  assert.equal(d.recon.size, 6);
  for (const v of [30.5, 30.75, 31.0, 31.25, 31.5]) {
    assert.equal(bin(v).P, 0, `${v} V`);
    assert.equal(bin(v).ts, t);
    assert.equal(bin(v).at, t);
  }
  assert.equal(bin(30.25).P, 250);
  assert.ok(bin(30.25).ts < t);
});

test('chart: live path breaks where neighbouring bins are more than 2 V apart', async () => {
  const { window, Dash, client } = await withBroker();
  let t = 1_763_400_000_000;
  window.Date.now = () => t;
  const st = { syms: new Map() };
  const tele = (Ui, P) => client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P, Ui }, ts: t }], st));
  for (const [v, p] of [[5, 10], [6, 20], [30, 200], [31, 210]]) { t += 100; tele(v, p); }
  Dash.flush();
  const path = (sel) => card(window, 'fry').querySelector(sel).getAttribute('d') || '';
  let dd = path('path.recon:not(.stale)');
  assert.equal((dd.match(/M/g) || []).length, 2);
  assert.equal((dd.match(/L/g) || []).length, 2);
  // stale path breaks the same way
  t += 660000; Dash.tick(); Dash.flush();
  assert.equal(path('path.recon:not(.stale)'), '');
  dd = path('path.recon.stale');
  assert.equal((dd.match(/M/g) || []).length, 2);
  assert.equal((dd.match(/L/g) || []).length, 2);
  // 2 V exactly still joins (6-8 V is stale: one end is)
  t += 100; tele(8, 30); Dash.flush();
  dd = path('path.recon.stale');
  assert.equal((dd.match(/M/g) || []).length, 2);
  assert.equal((dd.match(/L/g) || []).length, 3);
});

test('niceTicks is bounded and falls back on non-finite or runaway input', async () => {
  const { Dash } = await loadDash();
  const nt = (lo, hi, n) => plain(Dash.niceTicks(lo, hi, n));
  assert.deepEqual(nt(0, 200, 4).ticks, [0, 50, 100, 150, 200]);
  for (const [lo, hi] of [[1e18, 1e18 + 256], [0, 1.7e308], [0, Infinity], [NaN, 5], [-Infinity, Infinity], [5, 5]]) {
    const r = nt(lo, hi, 5);
    assert.ok(r.ticks.length >= 2 && r.ticks.length <= 12, `${lo}..${hi}: ${r.ticks.length} ticks`);
    assert.ok(r.ticks.every(Number.isFinite) || (!Number.isFinite(lo) || !Number.isFinite(hi)), `${lo}..${hi}`);
  }
  const f = nt(NaN, 5, 5);
  assert.deepEqual([f.lo, f.hi], [0, 1]);
});

test('chart: a new sweep resets the sticky y top; refetching the same sweep keeps it', async () => {
  const { Dash, client } = await withBroker();
  log(client, 'fry', ['sc: t=100 n=2 mpp=800,60 sweeping=0', 'sc: 40 500 60 800', 'sc: end']);
  Dash.flush();
  const dev = Dash.devices.get('fry');
  assert.ok(dev.pTop >= 800, String(dev.pTop));
  log(client, 'fry', ['sc: t=100 n=2 mpp=800,60 sweeping=0', 'sc: 40 500 60 800', 'sc: end']);
  Dash.flush();
  assert.ok(dev.pTop >= 800, String(dev.pTop));
  log(client, 'fry', ['sc: t=200 n=2 mpp=400,60 sweeping=0', 'sc: 40 250 60 400', 'sc: end']);
  Dash.flush();
  assert.ok(dev.pTop >= 400 && dev.pTop < 800, String(dev.pTop));
});

test('chart: hostile sc: values and telemetry are ignored; pTop never exceeds the clamp', async () => {
  const { window, Dash, client } = await withBroker();
  log(client, 'fry', ['sc: t=1 n=4 mpp=1.7e308,1e18 sweeping=0', 'sc: 30 1.7e308 1e18 20 -5 10 40 100',
    'sc: end']);
  log(client, 'fry', ['sc: t=1 n=2 mpp=1e999,30 sweeping=0', 'sc: 30 1e999 41 60000', 'sc: end']);
  const dev = Dash.devices.get('fry');
  client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P: 3e38, Ui: 40 }, ts: Date.now() }]));
  client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P: 100, Ui: 1e30 }, ts: Date.now() + 1 }]));
  Dash.flush();
  assert.ok(dev.pTop <= Dash.LIM.P, String(dev.pTop));
  const svg = card(window, 'fry').querySelector('svg.chart');
  assert.ok(svg.querySelectorAll('g.xlab text').length <= 13);
  assert.ok(svg.querySelectorAll('g.ylab text').length <= 13);
  assert.equal(svg.querySelector('path.curve').getAttribute('d'), '');
  assert.equal(svg.querySelector('circle.live').getAttribute('visibility'), 'hidden');
  assert.equal(dev.recon.size, 0);
  // a sane point still plots
  log(client, 'fry', ['sc: t=1 n=2 mpp=100,40 sweeping=0', 'sc: 30 50 40 100', 'sc: end']);
  Dash.flush();
  assert.equal((svg.querySelector('path.curve').getAttribute('d').match(/[ML]/g) || []).length, 2);
  assert.equal(svg.querySelector('g.ylab text:last-child').textContent, '100 W');
});

test('vin chart: last 30 s of Vin, <= 301 vertices, hidden without telemetry', async () => {
  const { window, Dash, client } = await withBroker();
  let t = 1_763_400_000_000;
  window.Date.now = () => t;
  log(client, 'nopts', ['x']);
  const st = { syms: new Map() };
  const t0 = t;
  for (let i = 0; i < 400; i++){
    const ts = t0 + i * 100;
    client.emit('message', 'pv/tele/fry', mqttMsg([{ fields: { P: 100, Ui: 30 + i * 0.025 }, ts }], st));
  }
  t = t0 + 39_900;
  Dash.flush();
  const c = card(window, 'fry');
  assert.equal(Dash.devices.get('fry').points.length, 400);
  const path = c.querySelector('svg.vinchart path.vin');
  assert.ok(path, 'path.vin exists');
  assert.equal(c.querySelector('.vinwrap').style.display, '');
  const v = [...path.getAttribute('d').matchAll(/[ML]([\d.]+) ([\d.]+)/g)].map(m => [+m[1], +m[2]]);
  assert.ok(v.length <= 301 && v.length >= 290, String(v.length));
  // x: left edge 34 = -30 s, right edge 310 = newest; the first vertex sits at ~-30 s
  assert.ok(Math.abs(v[0][0] - 34) < 2, String(v[0][0]));
  assert.ok(Math.abs(v.at(-1)[0] - 310) < 0.1, String(v.at(-1)[0]));
  // Vin over the window: 30 + 100*0.025 = 32.5 .. 39.975
  const yl = [...c.querySelectorAll('svg.vinchart g.ylab text')].map(e => parseFloat(e.textContent));
  assert.ok(yl[0] <= 32.5 && yl.at(-1) >= 39.975, yl.join());
  assert.match([...c.querySelectorAll('svg.vinchart g.ylab text')].at(-1).textContent, / V$/);
  assert.deepEqual(plain([...c.querySelectorAll('svg.vinchart g.xlab text')].map(e => e.textContent)), ['-30 s', '-20 s', '-10 s', '0']);
  assert.equal(c.querySelector('.vinbar .vincap').textContent, 'Vin · last 30 s');
  assert.equal(c.querySelector('.vinbar .vinv').textContent, '40.0 V');
  assert.equal(card(window, 'nopts').querySelector('path.vin'), null);
  assert.equal(card(window, 'nopts').querySelector('.vinwrap').style.display, 'none');

  // a single sample: +-1 V range
  client.emit('message', 'pv/tele/one', mqttMsg([{ fields: { P: 1, Ui: 20 }, ts: t }], { syms: new Map() }));
  Dash.flush();
  const yo = [...card(window, 'one').querySelectorAll('svg.vinchart g.ylab text')].map(e => parseFloat(e.textContent));
  assert.ok(yo[0] <= 19 && yo.at(-1) >= 21 && yo.at(-1) > yo[0], yo.join());
});

test('chart: a flagged sweep draws the curve dimmed and says so; the verdict line refetches', async () => {
  const { window, Dash, client } = await withBroker();
  log(client, 'fry', ['x']);
  Dash.flush();
  const svg = card(window, 'fry').querySelector('svg.chart');
  const caption = () => card(window, 'fry').querySelector('.chartbar .caption').textContent;
  const t = Math.floor(Date.now() / 1000);
  const dev = Dash.devices.get('fry');
  const deb = dev.curveDebounce;
  log(client, 'fry', ['I (9) mppt: sweep flagged: landed 452 W vs MPP 780 W (-42 %), scan now']);
  assert.notEqual(dev.curveDebounce, deb);
  log(client, 'fry', [`sc: t=${t} n=3 mpp=780.0,64.5 sweeping=0 flag=light land=452.0`,
    'sc: 70.0 100.0 64.5 780.0 50.0 400.0', 'sc: end']);
  Dash.flush();
  assert.equal(svg.querySelector('path.curve').classList.contains('flagged'), true);
  assert.equal(svg.querySelector('path.mpp').getAttribute('visibility'), 'visible');
  assert.match(caption(), /MPP 780\.0 W @ 64\.5 V · sky changed during sweep$/);

  log(client, 'fry', [`sc: t=${t + 60} n=2 mpp=470.0,58.0 sweeping=0 flag=0 land=468.0`, 'sc: 70.0 100.0 58.0 470.0', 'sc: end']);
  Dash.flush();
  assert.equal(svg.querySelector('path.curve').classList.contains('flagged'), false);
  assert.doesNotMatch(caption(), /sky changed/);
});
