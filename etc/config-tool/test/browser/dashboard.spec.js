// Real-browser smoke for dashboard.html: CDN + teledec load, a telemetry message renders a card
// with the live dot, a sweep curve draws and the crosshair readout follows the pointer.
const { test, expect } = require('@playwright/test');
const { mqttMsg } = require('../_setup_dash');

async function gotoDash(page) {
  const errs = [];
  if (process.env.DASH_VIEWPORT_W) await page.setViewportSize({ width: +process.env.DASH_VIEWPORT_W, height: 900 });
  page.on('pageerror', e => errs.push('pageerror: ' + e.message));
  page.on('console', m => { if (m.type() === 'error') errs.push('console: ' + m.text()); });
  await page.goto('/dashboard.html');
  await expect.poll(() => page.evaluate(() => typeof window.mqtt?.connect === 'function'
                                        && typeof window.TeleDec?.decodePayload === 'function'
                                        && typeof window.Dash === 'object')).toBe(true);
  return errs;
}

test('dashboard loads with no console errors; no MQTT client without a URL', async ({ page }) => {
  const errs = await gotoDash(page);
  expect(await page.evaluate(() => window.Dash.client)).toBe(null);
  await expect(page.locator('#empty')).toBeVisible();
  expect(errs).toEqual([]);
});

test('ingestTele renders a card with KPIs, the live dot and a sweep curve', async ({ page }) => {
  const errs = await gotoDash(page);
  const ts = Date.now();
  const bytes = [...mqttMsg([
    { fields: { P: 187.5, Ui: 37.2, Uo: 27.3, I: 6.9, pwm_duty: 1800, mppt_state: 4, P_max_today: 240.25, E_today: 812 }, ts },
  ])];
  await page.evaluate((b) => window.Dash.ingestTele('demo', new Uint8Array(b)), bytes);
  const card = page.locator('.card[data-host="demo"]');
  await expect(card.locator('.pv')).toHaveText('187.5');
  await expect(card.locator('.peak')).toHaveText('peak today 240.3 W · 812 Wh');
  await expect(card.locator('.mode')).toHaveText('MPPT');
  await expect(card.locator('circle.live')).toHaveAttribute('visibility', 'visible');
  await expect(card.locator('svg.vinchart path.vin')).toHaveAttribute('d', /^M[\d. ]+$/);
  await expect(card.locator('.vinbar .vinv')).toHaveText('37.2 V');

  await page.evaluate((t) => {
    const dev = window.Dash.getDevice('demo');
    const pts = [];
    for (let v = 46; v >= 28; v -= 1) pts.push(v.toFixed(1), (Math.max(0, 260 - 1.6 * (v - 36) ** 2) * (v < 36 ? (v - 27) / 9 : 1)).toFixed(1));
    const lines = [`I (1) main: sc: t=${t} n=${pts.length / 2} mpp=245.0,36.0 sweeping=0`];
    for (let i = 0; i < pts.length; i += 12) lines.push('sc: ' + pts.slice(i, i + 12).join(' '));
    lines.push('sc: end');
    for (const l of lines) window.Dash.onLine(dev, l, 'test');
    window.Dash.flush();
  }, Math.floor(ts / 1000));
  await expect(card.locator('svg.chart path.curve')).toHaveAttribute('d', /^M[\d. ]+(L[\d. ]+){18}$/);
  await expect(card.locator('svg.chart path.mpp')).toHaveAttribute('visibility', 'visible');
  await expect(card.locator('.chartbar .caption')).toContainText('MPP 245.0 W @ 36.0 V');
  // sc: lines are hidden in the log by default
  await expect(card.locator('.log .sc').first()).toBeHidden();

  // crosshair snaps to the nearest curve point
  const box = await card.locator('svg.chart').boundingBox();
  await page.mouse.move(box.x + box.width * 0.5, box.y + box.height * 0.5);
  await expect(card.locator('svg.chart g.hover')).toHaveAttribute('visibility', 'visible');
  await expect(card.locator('svg.chart g.hover text')).toHaveText(/^\d+\.\d W · \d+\.\d V$/);
  await page.mouse.move(0, 0);
  await expect(card.locator('svg.chart g.hover')).toHaveAttribute('visibility', 'hidden');

  // caption + refresh on one row, legend on one row, nothing clipped
  const rows = await card.evaluate((c) => {
    const mid = (e) => { const r = e.getBoundingClientRect(); return Math.round((r.top + r.bottom) / 2); };
    const bar = c.querySelector('.chartbar'), keys = [...c.querySelectorAll('.chartkeys .key')];
    return { bar: Math.abs(mid(bar.querySelector('.caption')) - mid(bar.querySelector('button.refresh'))) <= 2,
             keys: new Set(keys.map(mid)).size, fits: keys.every(k => k.getBoundingClientRect().right <= c.getBoundingClientRect().right) };
  });
  expect(rows).toEqual({ bar: true, keys: 1, fits: true });
  await expect(card.locator('form.console')).toBeHidden();

  // 30 s of Vin samples draw one line with many vertices
  await page.evaluate((t0) => {
    const dev = window.Dash.getDevice('demo');
    for (let i = 1; i <= 300; i++)
      window.Dash.ingest(dev, { ts: t0 + i * 100, fields: { P: 187 + 3 * Math.sin(i / 9), Ui: 37.2 + 0.4 * Math.sin(i / 25) + 0.05 * Math.sin(i * 1.7) } }, 'test');
    window.Dash.flush();
  }, ts);
  await expect(card.locator('svg.vinchart path.vin')).toHaveAttribute('d', /^M[\d. ]+(L[\d. ]+){250,}$/);
  // peak text shares the line with the big power number on a wide card
  const same = await card.evaluate((c) => {
    const p = c.querySelector('.kpis .p').getBoundingClientRect(), k = c.querySelector('.kpis .peak').getBoundingClientRect();
    return k.top < p.bottom && k.left > p.right;
  });
  if ((await card.boundingBox()).width > 400) expect(same).toBe(true);
  if (process.env.DASH_SCREENSHOT) await card.screenshot({ path: process.env.DASH_SCREENSHOT });
  expect(errs).toEqual([]);
});

test('charger state in the header with its colour, extras under the KPIs', async ({ page }) => {
  const errs = await gotoDash(page);
  await page.evaluate((b) => {
    window.Dash.ingestTele('demo', new Uint8Array(b));
    const dev = window.Dash.getDevice('demo');
    dev.chg = window.Dash.parseChargerStatus(['Charger: TERMINATED (float)',
      '  Vbat_max=29.00V Vout_max=26.80V  Ibat_lim=39.0A Iout_max=39.0A',
      '  BMS vcell_high=3.341V (stale/na, 400s ago)  ibat=1.48A (ok, 0s ago)  ahSinceFull=0.49Ah (bms)  vout_avg=26.78V',
      '  DoD since full: 0% / 15% to recharge']);
    window.Dash.ingest(dev, { fields: { P: 1 }, ts: 0 }, 'test');
    window.Dash.flush();
  }, [...mqttMsg([{ fields: { P: 10, Ui: 30 }, ts: Date.now() }])]);
  const chg = page.locator('.card[data-host="demo"] .kpis .chg');
  await expect(chg).toHaveText('cap 26.80 V · cell 3.341 V stale · DoD 0 % / 15 %');
  await expect(page.locator('.card[data-host="demo"] .top .cstate')).toHaveText('terminated (float)');
  const st = await chg.evaluate((e) => ({
    color: getComputedStyle(e.closest('.card').querySelector('.top .cstate.s-terminated')).color,
    accent2: getComputedStyle(document.documentElement).getPropertyValue('--accent2').trim(),
    dim: getComputedStyle(e.querySelector('.dim')).opacity,
    below: e.getBoundingClientRect().top >= e.parentElement.querySelector('.small').getBoundingClientRect().bottom - 1,
  }));
  expect(st.color).toBe('rgb(46, 160, 67)');
  expect(st.accent2).toBe('#2ea043');
  expect(st.dim).toBe('0.5');
  expect(st.below).toBe(true);
  expect(errs).toEqual([]);
});

test('log opens scrolled to the bottom when shown after lines arrived hidden', async ({ page }) => {
  await gotoDash(page);
  await page.evaluate(() => {
    window.Dash.ingestTele('demo', new Uint8Array(0));
    const dev = window.Dash.getDevice('demo');
    for (let i = 0; i < 150; i++) window.Dash.onLine(dev, 'line ' + i, 'test');
    window.Dash.flush();
  });
  const log = page.locator('.card[data-host="demo"] .log');
  await expect(log).toBeHidden();
  await page.locator('#console-toggle').check();
  await expect(log).toBeVisible();
  await expect.poll(() => log.evaluate(b => b.scrollHeight - b.clientHeight - b.scrollTop)).toBeLessThan(4);
  expect(await log.evaluate(b => b.scrollHeight > b.clientHeight)).toBe(true);
});

test('log follows again after a line filter makes it fit while scrolled to the top', async ({ page }) => {
  await gotoDash(page);
  await page.locator('#console-toggle').check();
  await page.evaluate(() => {
    window.Dash.ingestTele('demo', new Uint8Array(0));
    const dev = window.Dash.getDevice('demo');
    for (let i = 0; i < 60; i++) window.Dash.onLine(dev, 'sc: ' + i, 'test');
    for (let i = 0; i < 3; i++) window.Dash.onLine(dev, 'line ' + i, 'test');
    window.Dash.flush();
  });
  const card = page.locator('.card[data-host="demo"]');
  const log = card.locator('.log');
  await card.locator('.sc-toggle').check();
  await log.evaluate(b => { b.scrollTop = 0; });
  await expect.poll(() => log.evaluate(b => b.stick)).toBe(false);
  await card.locator('.sc-toggle').uncheck();
  await page.evaluate(() => {
    const dev = window.Dash.getDevice('demo');
    for (let i = 0; i < 40; i++) window.Dash.onLine(dev, 'more ' + i, 'test');
    window.Dash.flush();
  });
  await expect.poll(() => log.evaluate(b => b.scrollHeight - b.clientHeight - b.scrollTop)).toBeLessThan(4);
});
