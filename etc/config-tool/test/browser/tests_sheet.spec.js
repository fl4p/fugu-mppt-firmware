// Real-browser checks for the Tests sheet's action cards (spec §2.15): a folded card shows title and
// description only and its buttons are out of the Tab order; unfolded: tag, characteristics, then the buttons;
// a folded running card keeps Abort and a "running …" hint on its line. Runs against the jsdom suite's
// fake firmware (test/_tst_firmware.js). TST_SHOTS=<dir> saves screenshots (dark theme).
const fs = require('fs');
const path = require('path');
const { test, expect } = require('@playwright/test');
const { buildHtml } = require('../_setup');

const FW = fs.readFileSync(path.join(__dirname, '..', '_tst_firmware.js'), 'utf8');
const SHOTS = process.env.TST_SHOTS;
test.use({ colorScheme: 'dark', viewport: { width: 1280, height: 1100 } });

async function openTests(page, opts = {}) {
  const html = buildHtml({ teledec: true });
  await page.route('**/tst-sheet.html', r => r.fulfill({ contentType: 'text/html', body: html }));
  await page.goto('/tst-sheet.html');
  await page.addScriptTag({ content: FW });
  await page.evaluate(async o => {
    const conf = { 'coil.conf': { L0: '50e-6', rect_offset_ns: '500' }, 'charger.conf': { vout_max: '28.8' } };
    mqttFirmware(window, { fry: firmware('fry', { conf, coil: true, ...o }) });
    document.getElementById('mqtt-url').value = 'ws://broker:9001';
    await window.mqttConnectAndImport('fry');
    Object.assign(window.tstTimings(), { cmd: 300, probe: 300, settle: 20, poll: 5 });
    window.confirm = () => true; window.prompt = () => 'fry';
  }, opts);
  await page.locator('#open-tests').click();
  await expect(page.locator('#tst-body .tst-status')).toBeVisible();
  await expect.poll(() => page.evaluate(() => !window.tstBusyNow())).toBe(true);
  await expect(page.locator('#tst-sub')).not.toHaveClass(/busy/);
}
const card = (page, id) => page.locator(`#tst-body .tst[data-tst="${id}"]`);
const shot = async (page, name) => { if (SHOTS) await page.locator('#tests').screenshot({ path: path.join(SHOTS, name) }); };

test('folded cards: title and description only, buttons out of the Tab order; unfolded: tag, characteristics, buttons', async ({ page }) => {
  await openTests(page);
  for (const id of ['sweep', 'coil', 'bflow', 'clear', 'rectcal', 'coills']) {
    const c = card(page, id);
    await expect(c.locator('.tst-x')).toHaveAttribute('aria-expanded', 'false');
    await expect(c.locator('.tst-what')).toBeVisible();
    await expect(c.locator('.tst-tag')).toBeHidden();
    expect(await c.locator('button:visible').count(), id).toBe(1);   // the title
  }
  await shot(page, 'tests-collapsed.png');

  const s = card(page, 'sweep');
  await s.locator('.tst-x').focus();
  await page.keyboard.press('Tab');
  await expect(card(page, 'coil').locator('.tst-x')).toBeFocused();   // nothing focusable in a folded card
  await page.keyboard.press('Shift+Tab');
  await page.keyboard.press('Enter');
  await expect(s.locator('.tst-x')).toHaveAttribute('aria-expanded', 'true');
  await expect(s.locator('.tst-tag')).toHaveText('Drives the converter');
  const y = async sel => (await s.locator(sel).boundingBox()).y;
  expect(await y('.tst-tag')).toBeLessThan(await y('.tst-det'));
  expect(await y('.tst-det')).toBeLessThan(await y('.tst-ctl'));
  await page.keyboard.press('Tab');
  await expect(s.locator('.read')).toBeFocused();
  await page.keyboard.press('Tab');
  await expect(s.locator('.run')).toBeFocused();
  await s.locator('.read').click();
  await expect(s.locator('.tst-chart svg')).toBeVisible();

  await card(page, 'clear').locator('.tst-what').click();
  await expect(card(page, 'clear').locator('.tst-state')).toHaveText('Nothing latched');
  await expect(card(page, 'clear').locator('.run')).toBeDisabled();
  await card(page, 'clear').locator('.tst-what').click();
  await shot(page, 'tests-expanded.png');
});

test('a folded running card keeps Abort and says it is running', async ({ page }) => {
  await openTests(page, { bfNeverEnds: true });
  const b = card(page, 'bflow');
  await b.locator('.tst-x').click();
  await b.locator('.run').click();
  await expect(b.locator('.tst-out')).toContainText('started');
  await expect(b.locator('.tst-head .abort')).toBeVisible();
  await expect(b.locator('.tst-runs')).toBeHidden();
  await b.locator('.tst-x').click();
  await expect(b.locator('.tst-x')).toHaveAttribute('aria-expanded', 'false');
  await expect(b.locator('.tst-runs')).toHaveText('running …');
  await expect(b.locator('.tst-runs')).toBeVisible();
  await expect(b.locator('.tst-head .abort')).toBeVisible();
  await expect(b.locator('.tst-out')).toBeHidden();
  await shot(page, 'tests-running-collapsed.png');
});

test('a coil run shows its progress below the buttons', async ({ page }) => {
  await openTests(page, { rowMs: 300, coilNeverEnds: true });
  const c = card(page, 'coil');
  await c.locator('.tst-x').click();
  await c.locator('.run').click();
  const pg = c.locator('.tst-prog');
  await expect(pg).toHaveAttribute('aria-valuenow', '0');
  await expect(c.locator('.tst-pct')).toHaveText('step 1 of 47');
  const y = async sel => (await c.locator(sel).boundingBox()).y;
  expect(await y('.tst-ctl')).toBeLessThan(await y('.tst-prog'));
  await shot(page, 'tests-coil-progress.png');
});

test('a coil run on a ringing board: faded raw dots in step order, then the ring-fit line and its caption', async ({ page }) => {
  test.setTimeout(90_000);
  const conf = { 'coil.conf': { L0: '7.4e-05', rect_offset_ns: '500' }, 'charger.conf': { vout_max: '28.8' } };
  const apart = async c => {                                   // the horizontal lines' labels do not overlap
    const b = await c.locator('.tst-chart text.res-lab, .tst-chart text.ref-lab, .tst-chart text.hint-lab').evaluateAll(ts => ts.map(t => t.getBoundingClientRect().toJSON()));
    expect(b.length).toBeGreaterThan(1);
    for (const [i, r] of b.entries()) for (const l of b.slice(i + 1))
      expect(r.left >= l.right || l.left >= r.right || r.top >= l.bottom || l.top >= r.bottom, JSON.stringify([r, l])).toBe(true);
  };
  for (const [w, tag] of [[1280, 'desktop'], [390, '390']]) {
    await page.setViewportSize({ width: w, height: 1100 });
    await openTests(page, { rowMs: 80, coilRing: 'ring', conf });
    const c = card(page, 'coil');
    await c.locator('.tst-x').click();
    await c.locator('.run').click();
    await expect.poll(() => c.locator('circle.pt.raw').count()).toBeGreaterThan(24);
    await expect(c.locator('text.hint-lab')).toHaveText('ring-corrected L at the end');
    await expect(c.locator('path.step')).toHaveCount(1);
    await expect(c.locator('line.res')).toHaveCount(0);
    await apart(c);
    if (SHOTS) await c.screenshot({ path: path.join(SHOTS, `coil-ring-running-${tag}.png`) });
    await expect(c.locator('text.res-lab')).toHaveText('L = 73.88 µH (ring fit)', { timeout: 20_000 });
    await expect(c.locator('.tst-cap')).toHaveText('Dots: raw per-step values, they swing with the DCM ring (310 kHz); the line is the ring-corrected result.');
    await expect(c.locator('text.hint-lab')).toHaveCount(0);
    await apart(c);
    if (SHOTS) await c.screenshot({ path: path.join(SHOTS, `coil-ring-result-${tag}.png`) });
  }
});

test('an open sheet is the only scroller: the page behind it does not scroll', async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 700 });
  await openTests(page);
  await page.locator('#tst-close').click();
  await page.evaluate(() => { if (document.scrollingElement.scrollHeight <= innerHeight) document.body.append(Object.assign(document.createElement('div'), { style: 'height:3000px' })); });
  const pageScrolls = () => page.evaluate(() => getComputedStyle(document.documentElement).overflowY !== 'hidden'
    && document.scrollingElement.scrollHeight > document.scrollingElement.clientHeight);
  expect(await pageScrolls()).toBe(true);
  for (const id of ['sweep', 'coil', 'rectcal']) {        // enough content for the sheet to scroll
    await page.locator('#open-tests').click();
    await card(page, id).locator('.tst-x').click();
    await page.locator('#tst-close').click();
  }
  await page.locator('#open-tests').click();
  await expect(page.locator('#tests')).toHaveClass(/open/);
  expect(await pageScrolls()).toBe(false);
  await page.mouse.move(20, 300);                          // over the scrim
  await page.mouse.wheel(0, 600);
  await page.waitForTimeout(200);
  expect(await page.evaluate(() => document.scrollingElement.scrollTop)).toBe(0);
  const scrollers = await page.evaluate(() => [...document.querySelectorAll('#tests, #tests *')].filter(e => {
    const o = getComputedStyle(e).overflowY;
    return (o === 'auto' || o === 'scroll') && e.scrollHeight > e.clientHeight;
  }).map(e => e.id || e.className));
  expect(scrollers).toEqual(['tst-body']);
  await page.mouse.move(200, 400);                         // over the sheet: it scrolls, the page does not
  await page.mouse.wheel(0, 400);
  await page.waitForTimeout(200);
  expect(await page.evaluate(() => document.getElementById('tst-body').scrollTop)).toBeGreaterThan(0);
  expect(await page.evaluate(() => document.scrollingElement.scrollTop)).toBe(0);
  if (SHOTS) await page.screenshot({ path: path.join(SHOTS, 'tests-one-scroller.png') });
  await page.keyboard.press('Escape');
  await expect(page.locator('#tests')).not.toHaveClass(/open/);
  expect(await pageScrolls()).toBe(true);
  // the other modal sheets lock the same way
  await page.evaluate(() => document.getElementById('open-services').click());
  await expect(page.locator('#services')).toHaveClass(/open/);
  expect(await pageScrolls()).toBe(false);
  await page.locator('#svc-scrim').click({ position: { x: 5, y: 5 } });
  expect(await pageScrolls()).toBe(true);
});
