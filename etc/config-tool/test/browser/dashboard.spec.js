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
