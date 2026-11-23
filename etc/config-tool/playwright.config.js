// Playwright wraps the smoke suite under test/browser/. The jsdom suite under
// test/*.test.js is run separately by node:test (different glob, no overlap).
// Static-file server: Python's built-in http.server, no extra npm dep.
const { defineConfig, devices } = require('@playwright/test');

module.exports = defineConfig({
  testDir: 'test/browser',
  testMatch: '*.spec.js',
  timeout: 30_000,
  expect: { timeout: 10_000 },
  fullyParallel: false,                 // tests share one server; serial is fine
  retries: 0,
  reporter: 'list',
  use: {
    baseURL: 'http://localhost:8765',
    headless: true,
  },
  projects: [
    { name: 'chromium', use: { ...devices['Desktop Chrome'] } },
    { name: 'firefox', testMatch: 'dashboard.spec.js', use: { ...devices['Desktop Firefox'] } },
    { name: 'webkit', testMatch: 'dashboard.spec.js', use: { ...devices['Desktop Safari'] } },
  ],
  webServer: {
    command: 'python3 -m http.server 8765',
    url: 'http://localhost:8765/conf-editor.html',
    reuseExistingServer: !process.env.CI,
    timeout: 15_000,
  },
});
