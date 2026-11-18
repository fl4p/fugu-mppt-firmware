*this document is an LLM generated placeholder*

# Review: dashboard.html, teledec.js and their tests (main @ ab2eac2a, 2026-11-18)

Scope: `etc/config-tool/teledec.js`, `etc/config-tool/dashboard.html`, `test/teledec_*.test.js`,
`test/sweepcurve.test.js`, `test/dashboard*.test.js`, `test/browser/dashboard.spec.js`, `test/_setup_dash.js`,
`test/gen_tele_fixtures.py`, `test/fixtures/`, `website/scripts/copy-tools.mjs` and
`website/docs/reference/dashboard.md`. No tracked file was modified and no device was contacted. The scratch
scripts were in `/tmp/dashrev/`.

The review started at 1f40d4e7. While it ran, four dashboard commits landed on main (0cddf26e, c15d42bb,
ea3ee018, ab2eac2a). All findings were rechecked against ab2eac2a, and the line numbers below refer to it.
H1 was found at 1f40d4e7 and is fixed by ea3ee018. It stays in the report as a resolved finding.

## Test runs

- `npm test` at 1f40d4e7: 162 tests, 159 pass, 3 fail. At ab2eac2a: 166 tests, 164 pass, 2 fail.
  - "BLE link loss: 3 backoff reconnects…" (`dashboard.test.js:465` at HEAD) failed in the first full parallel run
    (`'pending' !== 'off'`). It passed 3 of 3 times alone and in the second full run, so it is **flaky**: it waits a
    fixed wall-clock `settle(30)` (`:472`) for three backoff reconnects.
  - Two conf-editor failures are outside this scope: `meta.test.js:62` (default `3.5` vs `3.6`, scraped-default
    drift) and "importFromDevice retries a lost reply".
- `npx playwright test`: 6 of 6 pass (Chromium).
- `gen_tele_fixtures.py` was run in a scratch copy of the tree with the venv's tamp 2.3.0. It reproduces
  `test/fixtures/tele_fixtures.json` byte for byte.

## Findings, ranked

### HIGH (resolved on main by ea3ee018 during the review)

**H1. A stream that is already flowing at page load counts as "relay", so the page never leases it and telemetry stops within 120 s.** Verified at 1f40d4e7.
At ab2eac2a the page always leases after 5 s, `relay` is set only on `Unknown command`/`Command not found`
(`:583`, `:595`), and a lease is requested again after 15 s of stream silence (`:1208`). The B1 re-run now
publishes `tele-mqtt 120`. New tests cover it (`dashboard_chart.test.js:19`, `dashboard.test.js`).
The original finding follows.
At 1f40d4e7, `tick`: if any `pv/tele/<host>` message arrives in the first 5 s after a card
appears, `L.state = "relay"`. No code path leaves `relay`: `tick` renews only `on`/`no-reply`, the MQTT
`connect` handler (1f40d4e7 `:501`) only `on`/`pending`, and `maybeRenew` only `on`.
Failure scenario: reload the dashboard (or open a second tab or browser). The previous page's
`tele-mqtt 120` lease is still running, so telemetry arrives within 5 s and the card is set to `relay`. When the
old lease expires (at most 120 s later), no one renews it. The card goes stale but keeps a green MQTT badge
saying "telemetry (relay)". The same happens to a second viewer when the first viewer closes its tab.
Reproduced in jsdom (`/tmp/dashrev/b.js`, B1): telemetry for 100 s, then none. After 300 s the state is still `relay`
and no `tele-mqtt` was ever published.
At 1f40d4e7 no test caught it, and the relay test asserted the `relay` classification.

### MEDIUM

**M1. A hostile `sc:` line or telemetry value can hang the tab: `niceTicks` loops forever.** Verified.
`dashboard.html:1019`, `for (let v = a; v <= b + step / 2; v += step) ticks.push(...)`. There are two runaway cases:
(a) all X values are near 1e18 and differ by a few ULP. `step` = 20 is below half an ULP, so `v += step` never
advances. (b) `pTop` is about 1.7e308 (`:1062`, sticky). `b = ceil(hi/step)*step` overflows to `Infinity`, and so does
`v`. A loop counter showed both cases past 1e6 iterations, and the array grows until V8 throws "Invalid array length"
after allocating about 1 GB. This repeats on every render of that card (4 Hz). The exception escapes `flush()`, so the
other cards dirty in the same flush are skipped.
Path: any publisher on `pv/log/<host>` can send an unsolicited `sc: t=1 n=2 mpp=0,0 sweeping=0`,
`sc: 30 1.7e308 31 1`, `sc: end`. `onCurve` (`:390`) accepts a complete curve whether or not it was requested
(checked: D1 stored the curve with `P = 1.7e308`),
and `parseScLine` accepts `1.7e308` and `1e999` (giving `Infinity` for `mpp.P`). The telemetry path works too
with an F64-typed `P`/`Ui` from a hostile symbol table. Firmware output cannot reach this, because the curve
holds f32 values printed with `%.1f`/`%.2f`.
Tests: none. There are no hostile-value chart tests.
Fix direction: clamp the plotted values to a sane range (|V|, P < 1e6), and cap the tick loop at about 50
iterations.

**M2. Every BLE connect and reconnect sends `set-time <browser clock>`, which steps an SNTP-synced live converter's clock.** Verified by reading.
`dashboard.html:692` sends it unconditionally, and `src/cli.cpp:896-906` (`cmdSetTime`) calls `settimeofday` with
no check of the current sync state. A laptop clock that is off by seconds or minutes shifts the InfluxDB timestamps
and the day boundary, and moves the device's wall-clock lease deadline (`tele_mqtt.cpp:65`; a forward step of more
than 120 s expires an MQTT lease at once). On the page, a backward step stops `lastTs`/`intervals` updates
(`:301`) until device time catches up, which skews the stale threshold.
The MQTT path already does this correctly: it sends `set-time` only after a "clock" refusal (`:584-588`).
Tests codify the current behaviour (`dashboard.test.js` expects `set-time N` in the enable sequence).
The docs page (`dashboard.md:71`) describes it as intended.
Fix direction: send `tele-ble 1` first, and send `set-time` only on a clock refusal.

**M3. Overlapping BLE reconnect chains: the `gatt.disconnect()` inside `connect()`'s retry loop fires `gattserverdisconnected`.** Verified with a spec-conformant fake.
`dashboard.html:659` calls `this.device.gatt.disconnect()` after a partly successful attempt, for example when
`connect()` succeeds but `getPrimaryService` fails, which is common while pairing. Per the Web Bluetooth spec this
fires `gattserverdisconnected`. `onDisconnected` (`:720`) sees `state === "connecting"`, not `off`, and starts its
own backoff and `connect(1)`+`enable()` loop alongside the original `connect()` loop.
Each drop during an `enable()` adds another loop, so a disconnect storm multiplies them.
Reproduced at both commits (`/tmp/dashrev/b.js`, B3, with the fake's `gatt.disconnect()` firing the event): two full
`hostname | set-time | tele-ble 1 | sweep-curve` sequences ran for one Add click. Combined with M2, the device
clock is set twice.
Tests: `_setup_dash.js:123`'s fake `gatt.disconnect()` does not fire the event, so this cannot be caught.
Fix direction: use a per-link generation counter or an `inFlight` promise. `onDisconnected` should return if a
connect is in progress, and the retry loop should remove its own handler while it runs.

**M4. Password storage: `conf-editor.html` defeats the opt-in, and the hosted copy shares an origin with every `fl4p.github.io` page.** Verified by reading.
- `conf-editor.html:1602` writes `mqtt-url`/`mqtt-user`/`mqtt-pass` to `localStorage` unconditionally on every
  MQTT scan. It uses the same keys as the dashboard (`dashboard.html:177`) and the same origin when served by
  `python3 -m http.server` from `etc/config-tool` (the documented quick start). Using the conf editor therefore
  leaves the password in plain text even with **remember** unticked. Loading the dashboard then deletes it
  (`:1250`), which in turn breaks the conf editor's recall.
  The caution in `dashboard.md:103-106` ("Unticked … the password stays in memory for the session only") is not
  true in that setup.
- The hosted copy is at `https://fl4p.github.io/fugu-mppt-firmware/tools/dashboard.html`
  (`docusaurus.config: url/baseUrl`). `localStorage` is per origin, so a remembered password is readable by script
  on every other `fl4p.github.io/<repo>/` project page. The docs say "the page's origin" but do not say that
  this origin is shared.
- Tests: `dashboard.test.js:450-480` check only the dashboard in isolation.
Fix direction: give the dashboard keys a prefix and never store the password from the conf editor without the
same opt-in. State in the docs that the hosted origin is shared.

### LOW

**L1. Deduplication marks a timestamp as seen before it checks that the point is relevant, so a raw-sensor point drops the main point.** Verified (B2).
`dashboard.html:287-290`: `seenTs.add(p.ts)` (`:289`) runs before the `P`/`Ui`/`mppt_state` filter (`:293`). With
`rawTelemetry` sensors enabled, `dcdcDataChanged` (`src/tele/telemetry.cpp:376-383`) emits extra `mppt` points
with a single field at millisecond timestamps through `telemetryAddPoint`, which feeds BLE and MQTT. When such a
point shares the ms timestamp of the main point and arrives first, the main point is discarded (B2: `ingest`
returned `false`, `P` undefined). The fix is to key on (ts, field set) or to filter before deduplicating.

**L2. `sendCommand` matches reply markers by substring.** Verified (B4).
`:422-423` `ln.includes("OK: " + cmd)`. A pending `tele-mqtt 120` resolves on `ERR: tele-mqtt 1200`, for example
one from another tab or from fugu_console, and `hostname` resolves on `OK: hostname newname`. Match the whole line
(`=== "OK: " + cmd` after `stripAnsi`/trim) instead.

**L3. Re-adding a restored (placeholder) BLE device does nothing.** Verified (C1).
`linkBle` (`:786`) returns the existing link without connecting even when `connect=true`. The card stays `off`
until the user finds the Reconnect button.

**L4. The silence-triggered re-lease ignores the refusal kind and repeats every 15 s with no end.** Verified by reading at ab2eac2a.
At 1f40d4e7 a transient refusal (`low heap`) was never retried. ea3ee018 fixes that with `L.retry`/`leaseRetries`.
The new silence branch (`:1208`), however, runs for every state except `relay`, including a permanent
`refused` and `no-reply`. Once telemetry has been seen (from a relay, another client's lease, or this page), a
device that goes silent gets `tele-mqtt 120` every 15 s for as long as the tab is open. Examples are a converter
that is switched off, has left Wi-Fi, or has `cmd_input=0`. This contradicts the docs, which say there is no
retry after a permanent error such as `disabled` (`dashboard.md`, badge table). The load is small (QoS 0), but
the retry has no limit. Gating the silence branch on `leaseRetries(L) || L.state === "off"`, plus backoff, would
match the docs.

**L5. Listener leak when a provisional BLE card merges.** Verified by reading (rare trigger).
`attachBle` copies `old.lineListeners` into `dev` (`:759`). Each listener's `done()` deletes itself from the
original device's set (`:426`), so a command still in flight during the merge leaves a listener on `dev` for
good. That listener then appends every later line to its `lines` array without limit. It needs a pending
`sendCommand` on the provisional card at merge time, which today only an overlapping enable can produce (see M3).

**L6. Decoder divergence from Python on unknown or string-typed field symbols.** Verified.
`teledec.js:165` throws `unknown field datatype` for DT 0 or DT > 10, which aborts the rest of the payload, while
`teledec.py:89-93` skips only that frame (KeyError → `continue`). In the test payload table+[frame with DT 11]+[good frame],
Python decodes `mppt P=1 6` and JS decodes 0 points. A firmware that adds a WireDT would lose every frame after the
first use in a batch on the page, including later table frames. `frameDecodable` (`:195`) could return `false` for
`!DT_WIDTH[dt]` to match Python.
The fixtures do not cover this because they are generated only from well-formed streams.

**L7. Device names from BLE are used in MQTT topics unchecked.** Suspected.
A BLE `Hostname:` reply (`:688`, regex `\S+`) can contain `/`, `+` or `#`. `cmdHostname` (`src/cli.cpp:1626-1629`)
does not validate. `send` then publishes to `pv/log/<host>/cmd` (`:411`): `#`/`+` is a protocol error at the
broker (disconnect and reconnect), and `/` targets a different topic. Topic names that come from MQTT are safe,
because brokers reject wildcards in PUBLISH and `onMqttMessage` ignores extra levels.

**L8. Unbounded growth with a hostile or busy broker.** Verified by reading.
The `devices` map gets one card per `pv/log|alert|tele/<host>`, with no limit. The MQTT `lineSplitter` buffer
(`:221`) grows without limit if no `\n` arrives, and `indexOf` per feed makes it quadratic. The per-device
symbol `table` grows with arbitrary SIDs. `tampDecompress` output for a large hostile MQTT payload is about 12x
the input. The page also re-sends `tele-mqtt 120` every 60 s (every 15 s through the silence branch, see L4), without end, to any `pv/log` publisher that does
not answer (for example non-fugu devices, or `cmd_input=0`).

**L9. Smaller robustness and observability gaps.**
- `pTop` sticks for the whole session (`:1062`), so one glitch value flattens the chart until reload.
- `parseScLine` accepts `inf`, but `Number("inf")` is NaN.
- Decode errors are counted (`dev.mqtt.errors` `:546`, BLE `decodeErr` `:620/:648`) but never shown.
- The MQTT symbol table persists across a device reboot (SIDs are reassigned). A data frame that arrives before
  the new table would decode with stale names or widths. This is suspected, and `teledec.py` has the same
  behaviour.

**L10. A `hostname` reply slower than 2.5 s gives a card name that does not match the MQTT name.** Suspected.
`:688-689`: on the first pairing the reply can take more than 2.5 s. The fallback strips `fugu-`, but a hostname
that already starts with `fugu-` is advertised unchanged (`console_ble_service.h:41`). The BLE card then gets
the stripped name and duplicates the MQTT card until the next reconnect renames it.

**L11. `pointToLine` field order and duplicates differ from Python.** Verified. This affects only the test and
debug path.
Numeric-looking field names come first (JS object key order), and a duplicated field SID keeps only its last
value. Python gives `m b=7i,10=8i,b=9i 1`, JS gives `m 10=8i,b=9i 1`.

**L12. `readVarint` has no shift cap.** Verified, no impact.
More than 146 continuation bytes make the value NaN (`teledec.js:30`). In practice this surfaces as an EOF error
and only skips the record.

**L13. Flaky test.** `dashboard.test.js:465` depends on wall-clock time (see "Test runs"). Use fake timers or poll
until `state === "off"`.

## Areas with no finding

- **tamp port**: the Huffman table matches the C `HUFFMAN_TABLE` symbol for symbol, and the code is complete.
  Overlap copy (read the whole match, then write) gives the same memmove result as `tamp_window_copy`. FLUSH byte
  alignment is equivalent to the C `bit_buffer_pos & ~7`. The out-of-window check is the same, and EOF mid-token
  stops output as in C.
  Fuzzed against Python tamp (non-extended, `extended=False`) with 400 streams covering window 8-15, literal 5-8 and
  random `flush(write_token=True)`: 0 mismatches. The firmware uses window 10, literal 8 and non-extended
  (`tamp_compress.cpp:8-12`). The fuzz ran under a system tamp 2.2.2; the fixtures use the venv's 2.3.0.
- **`%.6g`**: 19,834 random f32/f16/f64/decimal values plus all 65,536 f16 bit patterns are identical to Python
  `'%.6g'`, including subnormals, `-0`, inf, nan and rounding ties.
- **varint**: exact above 2^32 up to 2^53. Framing and resync follow `teledec.py` exactly (same bound, cid check,
  and skipping a failed record while keeping the framing). Unknown-SID frames are skipped as in Python.
- **XSS**: hostnames, log lines, alert payloads, reasons and SVG labels all go through `textContent` or attribute
  setters. The ANSI path escapes `& < >` before `createContextualFragment` and only emits fixed-palette `<span>`s.
  No injection path was found.
- **Topic injection from MQTT-derived hosts**: not possible (see L7).
- **Bounded rings**: log 200 rows and 100 DOM rows, points 600, `seenTs` 2000, live bins 400, intervals 50,
  history 50, and arrivals trimmed to 10 s. While the tab is hidden, `requestAnimationFrame` pauses rendering, so
  `logNew` and `dirty` stay bounded. There is a single `setInterval`, and timers are not re-armed per message.
- **Lease under tab throttling**: renewal is driven by message arrival (`maybeRenew`), which is not throttled,
  with the 1 s tick and `visibilitychange` as backstops. A broker reconnect re-leases every non-`off`,
  non-`relay` state. In a hidden tab with intensive timer throttling (about 1 tick per minute), the 15 s silence
  check and the 5 s reply timeout run late, but a lease confirmed by a reply is never lost.
- **Live-curve bins (ab2eac2a)**: overwriting within ±0.5 V and breaking the path at gaps over 2 V are bounded
  (400 bins, O(n) eviction per new bin), and NaN inputs are filtered before `reconAdd`.
- **`copy-tools.mjs`**: correct, fails loudly when a source is missing, and `static/tools/` is gitignored. One
  note: the hosted HTML carries the `<!-- …LLM generated placeholder -->` comment. It is invisible and not caught
  by docs-lint, because it is not a docs page.
- **`sc:` parser**: matches the firmware format (`src/cli.cpp:496-519`: header `%lu/%u/%.1f,%.1f/%d`, up to 12
  pairs per line, `sc: end`/`sc: none`), including prefixed and ANSI-coloured lines, and reports short or
  truncated curves.
- **Fixture generator**: deterministic and reproduces the checked-in fixtures byte for byte.

## Would the tests catch these?

At ab2eac2a none of M1-M4 or L1-L8 is covered (H1 now is). One test codifies the current behaviour:
`set-time` on every BLE connect (M2). The BLE fake cannot express M3 because its `gatt.disconnect()` does not
fire `gattserverdisconnected`. The fixtures come only from well-formed Python-encoded streams, so decoder
divergence on malformed or forward-compat input (L6) is invisible. Suggested additions:
1. A device that stays silent after a permanent refusal (L4).
2. Chart with extreme values.
3. A fake that fires the event on `disconnect()`.
4. Same-origin conf-editor + dashboard storage.
5. Same-ts non-mppt point.
6. Exact reply-marker matching.
7. Decoder parity with Python on DT 0 and DT 11 fields.
