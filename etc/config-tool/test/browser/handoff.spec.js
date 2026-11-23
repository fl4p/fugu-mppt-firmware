// Real-browser checks for the dashboard ↔ editor handoff and the editor's live strip: the fragment
// is gone from the address, Configure opens a reading editor tab, and the strip does not change the
// header's height or overflow it (desktop and narrow).
const { test, expect } = require('@playwright/test');
const { installFakeMqtt } = require('./_fake_mqtt');

test('editor: #mqtt= handoff reads the device, clears the fragment, shows the live strip', async ({ page, context }) => {
  await installFakeMqtt(context);
  await page.goto('/conf-editor.html#mqtt=ws%3A%2F%2Ffake%3A9001&device=devA');
  await expect(page.locator('#conn')).toHaveText('Connected to devA over MQTT');
  expect(await page.evaluate(() => location.href)).toMatch(/\/conf-editor\.html$/);
  await expect(page.locator('#live')).toBeVisible();
  await expect(page.locator('#live .lv-p')).toHaveText('118.4 W');
  await expect(page.locator('#live .lv-m')).toHaveText('MPPT');
  const href = await page.locator('#live-dash').getAttribute('href');
  expect(href).toBe('dashboard.html#mqtt=ws%3A%2F%2Ffake%3A9001&user=&device=devA');
  const sent = await page.evaluate(() => window.__mqttLog.map(x => x[1]));
  expect(sent.filter(c => /^(set-config|del-config|svc )/.test(c))).toEqual([]);
  // reload: the fragment is gone, so nothing is read again
  await page.reload();
  await page.waitForTimeout(300);
  await expect(page.locator('#conn')).toBeHidden();
  expect(await page.evaluate(() => window.__mqttLog.length)).toBe(0);
});

for (const [w, h] of [[1280, 800], [1024, 768], [375, 760]]) {
  test(`editor: the strip keeps the header height and fits at ${w}px`, async ({ page, context }) => {
    await installFakeMqtt(context);
    await page.setViewportSize({ width: w, height: h });
    await page.goto('/conf-editor.html#mqtt=ws%3A%2F%2Ffake%3A9001&device=devA');
    await expect(page.locator('#live')).toBeVisible();
    const m = () => page.evaluate(() => {
      const hd = document.querySelector('header'), src = document.querySelector('header .src');
      return { h: hd.getBoundingClientRect().height, over: hd.scrollWidth - hd.clientWidth, srcOver: src.scrollWidth - src.clientWidth,
               panes: document.querySelector('#panes, .shell') && document.querySelector('.shell').getBoundingClientRect().top };
    });
    const shown = await m();
    await page.evaluate(() => { document.getElementById('live').hidden = true; });
    const hidden = await m();
    expect(shown.h).toBe(hidden.h);
    expect(shown.panes).toBe(hidden.panes);
    expect(shown.over).toBeLessThanOrEqual(hidden.over);   // the narrow header's actions row overflows on its own
    expect(shown.srcOver).toBeLessThanOrEqual(0);
    await page.evaluate(() => { document.getElementById('live').hidden = false; });
    await expect(page.locator('#live .lv-p')).toBeVisible();
    if (w <= 520) {
      await expect(page.locator('#live .lv-dash')).toBeHidden();
      await expect(page.locator('#live .lv-age')).toBeHidden();
    } else if (w >= 1280) await expect(page.locator('#live .lv-dash')).toBeVisible();
  });
}

test('dashboard: Configure opens the editor on that device in a new tab, which reads it', async ({ page, context }) => {
  await installFakeMqtt(context, { every: 500 });
  await page.addInitScript(() => { try { localStorage.setItem('mqtt-url', 'ws://fake:9001'); } catch (_) {} });
  await page.goto('/dashboard.html');
  const btn = page.locator('.card[data-host="devA"] button.cfg');
  await expect(btn).toBeVisible();
  const [tab] = await Promise.all([context.waitForEvent('page'), btn.click()]);
  await expect(tab.locator('#conn')).toHaveText('Connected to devA over MQTT');
  expect(tab.url()).toMatch(/\/conf-editor\.html$/);
  await expect(tab.locator('#live .lv-p')).toHaveText('118.4 W');
});
