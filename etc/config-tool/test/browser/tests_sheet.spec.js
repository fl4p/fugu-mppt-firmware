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
