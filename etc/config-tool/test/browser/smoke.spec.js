// Real-browser smoke tests that cover what jsdom can't:
//   §0 CDN sanity   — the real mqtt@5 / ansi-to-html bundles load and set their globals
//   §7 tail-follow  — driven by real layout, not patched scroll geometry
//   §1 drag-drop    — folder upload via webkitGetAsEntry (a non-standard, browser-only API)
const { test, expect } = require('@playwright/test');

async function gotoEditor(page) {
  const errs = [];
  page.on('pageerror', e => errs.push('pageerror: ' + e.message));
  page.on('console', m => { if (m.type() === 'error') errs.push('console: ' + m.text()); });
  await page.goto('/conf-editor.html');
  // wait for the ESM module import (esm.sh fetch) to settle
  await expect.poll(() => page.evaluate(() => typeof window.ansiConvert === 'object'
                                        && typeof window.mqtt?.connect === 'function')).toBe(true);
  return errs;
}

test('CDN sanity: page loads, both globals appear, no console errors', async ({ page }) => {
  const errs = await gotoEditor(page);
  expect(errs).toEqual([]);
});

test('real ansi-to-html: ESC[…m renders as a coloured span', async ({ page }) => {
  await gotoEditor(page);
  await page.evaluate(() => {
    window.ulShow();
    window.ulConsoleListener('\x1b[0;31mERROR\x1b[0m hello');
    window.ulConsoleListener('\x1b[0;32mI (123) main:\x1b[0m OK: ping');
    window.ulConsoleListener('no escape here');
  });
  // two ANSI lines → at least two coloured spans; plain line stays uncoloured
  const colored = await page.locator('#uploadlog-body span[style*="color"]').count();
  expect(colored).toBeGreaterThanOrEqual(2);
  // sanity: the green span should be a recognizable green colour
  const styles = await page.locator('#uploadlog-body span[style*="color"]').evaluateAll(
    els => els.map(e => e.getAttribute('style')));
  expect(styles.some(s => /#[0-9a-fA-F]{3,6}/.test(s))).toBe(true);
});

test('tail-follow under real layout: scroll snaps to bottom only when at bottom', async ({ page }) => {
  await gotoEditor(page);
  // force the panel small so a few lines overflow it
  await page.addStyleTag({ content: '#uploadlog { max-height: 120px !important; }' });
  await page.evaluate(() => window.ulShow());

  // push enough lines to overflow
  await page.evaluate(() => { for (let i = 0; i < 60; i++) window.ulConsoleListener('line ' + i); });
  // at-bottom assertion: within a few px of the bottom of the scroll area
  const distAfterAutoTail = await page.evaluate(() => {
    const el = document.getElementById('uploadlog');
    return el.scrollHeight - el.clientHeight - el.scrollTop;
  });
  expect(distAfterAutoTail).toBeLessThan(5);

  // scroll the user up; new lines must NOT yank the view down
  await page.evaluate(() => { document.getElementById('uploadlog').scrollTop = 0; });
  await page.evaluate(() => window.ulConsoleListener('arrived while user scrolled up'));
  const st = await page.evaluate(() => document.getElementById('uploadlog').scrollTop);
  expect(st).toBe(0);
});

test('drag-drop folder upload via webkitGetAsEntry loads the dropped tree', async ({ page }) => {
  await gotoEditor(page);
  // Synthesise a fake FileSystemEntry tree the way the drop handler reads it.
  // dataTransfer is readonly on DragEvent, so dispatch a plain Event with a
  // defined `dataTransfer` property — the handler only reads e.dataTransfer.items.
  await page.evaluate(() => {
    const fakeFile = (name, content) => ({
      isFile: true, isDirectory: false, name,
      file(cb) { cb(new File([content], name)); },
    });
    const fakeDir = (name, kids) => ({
      isFile: false, isDirectory: true, name,
      createReader() {
        let done = false;
        // mirror the real DirectoryReader.readEntries — async; cb queued on a
        // microtask so the handler's recursive readBatch() doesn't blow the stack
        return { readEntries(cb) {
          const batch = done ? [] : kids; done = true;
          queueMicrotask(() => cb(batch));
        } };
      },
    });
    const root = fakeDir('myconf', [
      fakeDir('conf', [
        fakeFile('board.conf',  'mcu=esp32s3\npwm_freq=39000\n'),
        fakeFile('limits.conf', 'hv_max=80\nlv_max=60\n'),
      ]),
    ]);
    const items = [{ webkitGetAsEntry: () => root }];
    const evt = new Event('drop', { bubbles: true, cancelable: true });
    Object.defineProperty(evt, 'dataTransfer', { value: { items, files: [] } });
    document.getElementById('drop').dispatchEvent(evt);
  });

  // app should switch on with the dropped folder's root as the source label
  await expect(page.locator('#srcname')).toHaveText('myconf');
  await expect(page.locator('#tabs .tab').filter({ hasText: 'board' })).toBeVisible();  // label strips ".conf" (§2.1)
  // and the real .conf bytes should have populated the mcu input
  const mcu = await page.locator('#panes .row input').first();
  await expect(mcu).toHaveValue('esp32s3');
});
