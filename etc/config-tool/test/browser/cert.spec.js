// The certificate advice against a real wss:// listener with a self-signed certificate (_wss_broker.js) and
// the real mqtt@5 from the CDN, in Chromium: what the browser reports for an untrusted certificate, a
// refused login and a listener without WebSocket, and the whole click-through when the browser can show
// the certificate interstitial. CERT_SHOTS=<dir> saves screenshots of the advice, light and dark.
const path = require('node:path');
const { test, expect, chromium } = require('@playwright/test');
const { startWssBroker, haveOpenssl } = require('./_wss_broker');

test.skip(!haveOpenssl(), 'needs openssl for the self-signed certificate');

const SHOTS = process.env.CERT_SHOTS;
async function shot(page, name, el) {
  if (!SHOTS) return;
  for (const scheme of ['light', 'dark']) {
    await page.emulateMedia({ colorScheme: scheme });
    await (el || page).screenshot({ path: path.join(SHOTS, `${name}-${scheme}.png`) });
  }
}

async function scan(page, url) {
  await page.goto('/conf-editor.html');
  await expect.poll(() => page.evaluate(() => typeof window.mqtt?.connect === 'function')).toBe(true);
  await page.evaluate(() => window.mqttModalOpen());
  await page.fill('#mqtt-url', url);
  await page.fill('#mqtt-user', 'pv');
  await page.fill('#mqtt-pass', 'secret');
  await page.click('#mqtt-scan');
}

async function dashboard(page, url) {
  await page.addInitScript(u => {
    if (location.port !== '8765') return;
    localStorage.setItem('mqtt-url', u);
    localStorage.setItem('mqtt-user', 'pv');
    localStorage.setItem('dash-mqtt-pass', 'secret');
  }, url);
  await page.goto('/dashboard.html');
}

test.describe('untrusted certificate', () => {
  let broker;
  test.beforeAll(async () => { broker = await startWssBroker(); });
  test.afterAll(async () => { await broker.close(); });

  test('editor: the scan shows the advice and the button', async ({ page }) => {
    await scan(page, broker.url);
    const hint = page.locator('#mqtt-hint');
    await expect(hint).toBeVisible();
    await expect(hint).toContainText(`Could not open a secure connection to localhost:${broker.port}.`);
    await expect(hint.locator('button')).toHaveText(`Open https://localhost:${broker.port}`);
    expect(broker.st.connects).toBe(0);
    await shot(page, 'editor-cert', page.locator('#mqtt-modal .modal'));
  });

  test('dashboard: the stored broker shows the banner and the button', async ({ page }) => {
    await page.setViewportSize({ width: 1100, height: 420 });
    await dashboard(page, broker.url);
    const banner = page.locator('#banners .wss-hint');
    await expect(banner).toBeVisible();
    await expect(banner).toContainText(`Could not open a secure connection to localhost:${broker.port}.`);
    await expect(banner.locator('button')).toHaveText(`Open https://localhost:${broker.port}`);
    await shot(page, 'dashboard-cert');
  });
});

test.describe('trusted certificate', () => {
  test.use({ ignoreHTTPSErrors: true });

  test('editor: a refused login says so, no advice', async ({ page }) => {
    const broker = await startWssBroker({ rc: 5 });
    try {
      await scan(page, broker.url);
      await expect(page.locator('#mqtt-status')).toHaveText('The broker refused the login (Not authorized)');
      await expect(page.locator('#mqtt-hint')).toBeHidden();
      await shot(page, 'editor-login', page.locator('#mqtt-modal .modal'));
    } finally { await broker.close(); }
  });

  test('dashboard: a refused login stops and says so, no banner', async ({ page }) => {
    const broker = await startWssBroker({ rc: 5 });
    try {
      await dashboard(page, broker.url);
      await expect(page.locator('#mqtt-status')).toHaveText('the broker refused the login (Not authorized): check the user and password');
      await expect(page.locator('#banners .wss-hint')).toHaveCount(0);
      await expect(page.locator('#mqtt-connect')).toHaveText('Connect');
    } finally { await broker.close(); }
  });

  test('editor and dashboard: https answers but no WebSocket: the WebSocket was refused, no button', async ({ page }) => {
    const broker = await startWssBroker({ ws: false });
    try {
      await scan(page, broker.url);
      const hint = page.locator('#mqtt-hint');
      await expect(hint).toContainText('accepts a secure connection, but the broker did not accept the MQTT WebSocket');
      await expect(hint.locator('button')).toHaveCount(0);
      await shot(page, 'editor-nows', page.locator('#mqtt-modal .modal'));
      await page.evaluate(() => window.mqttModalClose());
      await dashboard(page, broker.url);
      const banner = page.locator('#banners .wss-hint');
      await expect(banner).toContainText('accepts a secure connection, but the broker did not accept the MQTT WebSocket');
      await expect(banner.locator('button')).toHaveCount(0);
      expect(broker.st.gets).toBeGreaterThanOrEqual(2);
    } finally { await broker.close(); }
  });
});

test('unreachable wss:// port: the same advice (the browser cannot tell it from a certificate)', async ({ page }) => {
  const broker = await startWssBroker();
  const port = broker.port;
  await broker.close();
  await scan(page, `wss://localhost:${port}`);
  await expect(page.locator('#mqtt-hint')).toContainText(`Could not open a secure connection to localhost:${port}.`);
});

// Playwright's headless shell shows no certificate interstitial; the full Chromium build does.
test('click-through: open the page, proceed past the warning, come back, the retry connects', async ({ baseURL }) => {
  let browser;
  try { browser = await chromium.launch({ channel: 'chromium' }); }
  catch (e) { test.skip(true, 'full Chromium build not installed: ' + e.message.split('\n')[0]); }
  const broker = await startWssBroker();
  try {
    for (const tool of ['editor', 'dashboard']) {
      const ctx = await browser.newContext({ baseURL });   // a fresh context remembers no certificate decision
      const page = await ctx.newPage();
      if (tool === 'editor') await scan(page, broker.url); else await dashboard(page, broker.url);
      const box = page.locator(tool === 'editor' ? '#mqtt-hint' : '#banners .wss-hint');
      const btn = box.locator('button');
      await expect(btn).toBeVisible();
      const gets = broker.st.gets;
      const [tab] = await Promise.all([ctx.waitForEvent('page'), btn.click()]);
      await tab.waitForLoadState().catch(() => {});
      await tab.click('#details-button');
      await tab.click('#proceed-link');
      await expect.poll(() => broker.st.gets).toBeGreaterThan(gets);
      await tab.close();
      await page.bringToFront();
      // headless Chromium fires neither focus nor visibilitychange on bringToFront: send the focus event
      await page.evaluate(() => window.dispatchEvent(new Event('focus')));
      if (tool === 'editor') {
        await expect(page.locator('#mqtt-hint')).toBeHidden();
        await expect(page.locator('#mqtt-status')).toContainText('listening');
      } else {
        await expect(page.locator('#mqtt-status')).toHaveText('connected');
        await expect(box).toHaveCount(0);
      }
      expect(broker.st.connects).toBeGreaterThan(0);
      await ctx.close();
    }
  } finally {
    await broker.close();
    await browser.close();
  }
});
