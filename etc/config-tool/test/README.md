*this document is an LLM generated placeholder*

# Tests for `conf-editor.html`

These tests load the real `conf-editor.html` into [jsdom](https://github.com/jsdom/jsdom) and exercise it through the same DOM the browser sees. The two CDN dependencies (`mqtt@5`, `ansi-to-html`) are stubbed in `_setup.js` so the tests need no network at run time.

## Run

```bash
cd etc/config-tool
npm install                                # one-time: jsdom + @playwright/test
npx playwright install chromium            # one-time: ~150 MB chromium binary
npm test            # node:test + jsdom    (~5 s; default)
npm run test:browser  # playwright + real chromium  (~5 s)
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
| `transports.test.js` | §5 (`sendCommand`, `getConfig`, `importFromDevice` retry-on-timeout), §5.3 (MQTT scan modal + topics) |
| `browser/smoke.spec.js` | CDN sanity (real `mqtt@5` + `ansi-to-html` load); real-layout tail-follow; folder drag-drop via `webkitGetAsEntry` |

## Adding tests

Use `loadEditor()` from `_setup.js` — it returns a jsdom `dom`, and `dom.window` exposes:

* every top-level `function ...()` from the script (e.g. `parseConf`, `serializeFile`, `sendCommand`, `mqttBindDevice`, `ulShow`, `consoleSend`, `lookupType`, `lookupMeta`, `writeZip`, `readZip`, …).
* test hooks injected at the bottom of the script: `_lineListeners`, `_dispatchLine(line)`, `_state` (getter), `_activeWrite` (get/set), `_lastTransport` (get/set), `_mqttClient`, `_mqttDeviceHost`, `_mqttClients` (list of stub clients).
* stubs: `window.mqtt.connect(url,opts)` returns a fake client that records `subscribe`/`publish` calls; `window.ansiConvert.toHtml(text)` wraps any ESC-containing text in a single `<span style="color:#0a0">…</span>` so tests can assert on the conversion path.
