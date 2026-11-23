*this document is an LLM generated placeholder*

# Tests for `conf-editor.html`

These tests load the real `conf-editor.html` into [jsdom](https://github.com/jsdom/jsdom) and exercise it through the same DOM the browser sees. The two CDN dependencies (`mqtt@5`, `ansi-to-html`) are stubbed in `_setup.js` so the tests need no network at run time.

## Run

```bash
cd etc/config-tool
npm install                                # one-time: jsdom + @playwright/test
npx playwright install chromium firefox webkit  # one-time browser binaries
npm test            # node:test + jsdom    (~5 s; default)
npm run test:browser  # playwright: conf editor in chromium, dashboard in chromium, firefox, webkit  (~10 s)
npm run test:all      # both
```

Uses Node's built-in test runner — works on Node 18+ (verified on 24).
The browser suite uses Python's built-in `http.server` as the static server (no extra npm dep).

## What's tested

Files map roughly to spec sections (`../spec.md`):

| file | spec section |
| ---- | ------------ |
| `parse.test.js`      | §2.5 (conf parser), §2.3 (`<not set>` / `0` / `""` semantics via serialize) |
| `meta.test.js`       | §4 (`META`, `FILE_META`, channel prefixes, ssid patterns) |
| `sidekeys.test.js`   | §4.1 (side keys, removed role keys flagged for migration) |
| `zip.test.js`        | §1 (ZIP reader/writer), §3 (download path: `-edited` vs dated `-backup`) |
| `load.test.js`       | §1 (loading clears state), §1.1 (upload overlays a device read), §2.1 (synthetic tabs, tab order) |
| `edit.test.js`       | §2.2 (rows, "was: …" incl. "(not set)", clear button), §2.1 (dirty dot) |
| `console.test.js`    | §7 (device-log panel, ANSI rendering, tail-follow, command input) |
| `services.test.js`  | §2.14 (services sheet: `svc list` parsing, commands, gating, own-link confirm) |
| `tests_panel.test.js` | §2.15 (tests sheet: build probe, Status rows read on open, collapsed action cards, confirms, refusals, result parsing, abort, link drop, one-at-a-time guard, MPP sweep card and chart, limiter-stopped sweeps, the fallback without `teledec.js`) |
| `transports.test.js` | §5 (`sendCommand`, `getConfig`, `importFromDevice` retry-on-timeout), §5.3 (MQTT scan modal + topics) |
| `teledec_{tamp,payload,framing}.test.js` | `../teledec.js` (binary telemetry decoder, no jsdom) against `fixtures/tele_fixtures.json`; regenerate with `/path/to/repo/.venv/bin/python3 test/gen_tele_fixtures.py` (tamp 2.3.0, expected lines from `etc/fugu/teledec.py`) |
| `sweepcurve.test.js` | `../teledec.js` `sc:` sweep-curve lines (`parseScLine`, `SweepCurveParser`) |
| `handoff.test.js` | §1.4 (handoff fragment parsing and clearing, MQTT with / without a stored password, BLE via `getDevices`: found, not granted, no `getDevices`, busy; confirmDiscard; nothing pushed) |
| `live_strip.test.js` | §2.1.1 (status-line parsing, stale and stop states, when the strip shows, the dashboard link per transport, no commands sent) |
| `dashboard_handoff.test.js` | `../dashboard.html` Configure (MQTT / BLE fragment, BLE link released first, no password in the URL) and the `#mqtt=` / `#ble=` fragment from the editor |
| `_fakes.js` | fake firmware, MQTT hook and granted BLE device shared by the two editor files above |
| `browser/handoff.spec.js` | the handoff end to end in Chromium (dashboard Configure → editor tab reads), fragment gone after load, the strip keeps the header height at 1280/1024/375 px; `browser/_fake_mqtt.js` is the in-page fake broker |
| `browser/smoke.spec.js` | CDN sanity (real `mqtt@5` + `ansi-to-html` load); real-layout tail-follow; folder drag-drop via `webkitGetAsEntry` |

## Adding tests

Use `loadEditor()` from `_setup.js` — it returns a jsdom `dom`, and `dom.window` exposes:

* every top-level `function ...()` from the script (e.g. `parseConf`, `serializeFile`, `sendCommand`, `mqttBindDevice`, `ulShow`, `consoleSend`, `lookupType`, `lookupMeta`, `writeZip`, `readZip`, …).
* test hooks injected at the bottom of the script: `_lineListeners`, `_dispatchLine(line)`, `_state` (getter), `_activeWrite` (get/set), `_lastTransport` (get/set), `_mqttClient`, `_mqttDeviceHost`, `_mqttClients` (list of stub clients).
* `loadEditor({ teledec: true })` inlines `../teledec.js` where the page loads it; by default the page runs without it (jsdom fetches no subresources), as when the HTML is opened on its own.
* stubs: `window.mqtt.connect(url,opts)` returns a fake client that records `subscribe`/`publish` calls; `window.ansiConvert.toHtml(text)` wraps any ESC-containing text in a single `<span style="color:#0a0">…</span>` so tests can assert on the conversion path.
