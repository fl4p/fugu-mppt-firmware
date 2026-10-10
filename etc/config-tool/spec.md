*this document is an LLM generated placeholder*

# `conf-editor.html` — specification

A single-page HTML editor for Fugu MPPT board-config files (the `.conf` files
under `config/<board>/conf/`). Loads a set of conf files from a `.zip`, a
folder, or live from a device (Serial, Bluetooth or MQTT), lets the user edit
fields with type/unit/description hints, then writes the result back as a
downloadable `.zip` or as a stream of `set-config` / `del-config` commands.

Self-contained: one HTML file, no build step. Two external CDN dependencies
(both browser-side only): `mqtt@5` (WebSocket MQTT client) and
`ansi-to-html@0.7.2` (ESP\_LOG colour rendering).

---

## 1. Loading config

The user opens config either as files or live off a device.

| Source                | UI                                                                 |
| --------------------- | ------------------------------------------------------------------ |
| `.zip` archive        | "Choose .zip" picker, drag-drop, or "Open .zip" reload             |
| folder                | "Choose folder" picker (`webkitdirectory`), drag-drop, or "Open folder" reload |
| USB serial            | "Connect serial (115200)" → `navigator.serial.requestPort()`       |
| Bluetooth (NUS)       | "Connect Bluetooth" → `navigator.bluetooth.requestDevice()`        |
| MQTT                  | "Connect MQTT" → opens scan modal (see §6.3)                       |

Loading any source clears any previously loaded state. The last key inside a
zip/folder is treated as the source label (equal keys: last one overwrites the previous key's value); with serial/BLE the device's
`hostname` reply is used (falls back to "serial-device" / "ble-device").

Only `*.conf` files are parsed for editing; any other file in the zip/folder
is carried opaquely and re-emitted on download (same byte payload).

### 1.1 Overlay after a device read

Exception to "clears previously loaded state": once the config has been read
**live from a device** (`state.fromDevice`), opening a `.zip` or folder does
**not** reset the editor. Instead the uploaded values are *overlaid* onto the
live read as pending edits (`mergeUpload`), keeping each file's `original` (the
device values) as the baseline. Every uploaded value that differs from the
device lights up as a change (orange border + "was: …" / dirty dot) so the user
can review what the file would alter before pushing it back with "Upload
changes to device". Within a file the upload includes, a key the device
reported but the uploaded file omits is **cleared** (removed from the model,
shown as `<not set>`, and flagged as changed) — the uploaded file fully
replaces that file. Files the upload doesn't include at all are left untouched
(omitting a whole file never clears its device keys). Files matched by basename;
a file the device didn't report is added as a new tab. The source pill reads
`<device> ← <upload>`, and the editor jumps to the first changed file. A
non-device load (zip/folder with no prior device read) still clears state as
before.

ZIP support: built-in reader (stored + deflate via `DecompressionStream`) and
writer (stored only). No external zip dependency.

---

## 2. Editor UI

### 2.1 Tabs

* One tab per known conf file. The tab order is fixed within a group:
  `board, sensor, limits, coil, converter, charger, tracker, mqtt, tele,
  pprof, wifi`, then any unknown file alphabetically.
* **Label**: the trailing `.conf` is stripped from the visible label
  (`mqtt.conf` → `mqtt`); the tooltip still carries the full filename.
* **Sensitive tabs** (`board`, `sensor`, `limits`, `coil`, `converter`)
  render in a muted red (`.tab.sensitive`), with a tooltip suffix
  "— sensitive (hardware/calibration)". Their active and hover states use a
  lighter shade so the selected red tab is still readable.
* **Synthetic tabs**: every file the firmware reads (the `FILE_KEYS` table)
  gets a tab even if it's missing from the source. Synthetic tabs render
  italic/faded with title "(not on device — fill a key to create)" and are
  pushed to the **right** of all real (loaded) tabs; the same TAB_ORDER
  ranking applies within each group. An untouched synthetic file is *not*
  exported.
* A small orange dot next to the tab name marks the file as dirty (any
  field's current value differs from its loaded value). Reverting every
  field clears the dot.

### 2.2 Rows

Each `.conf` field is one row with:

| Element        | Source                                                             |
| -------------- | ------------------------------------------------------------------ |
| key name       | the literal config key                                             |
| type pill      | byte / long / float / string — which `ConfFile::get*()` the firmware uses; hover shows the getter name |
| `?` warning    | shown when no metadata is hand-curated for this key                |
| description    | from `META[key]` / `FILE_META[file][key]` / channel pattern; if absent, falls back to the inline `# comment` from the file (suffixed "(from file comment)"). If `DEFAULTS[key]` is known, the description is suffixed with " · default: \<value\>" |
| text input     | the current value                                                  |
| unit suffix    | rendered right-aligned inside the input (V, A, Ω, Hz, GPIO, …)     |
| `×` clear-btn  | only visible when the input is non-empty                           |
| "was: …" line  | shown when the current value differs from the value the source loaded with — "was: (empty)" for a cleared string, "was: (not set)" for a key the device/source didn't have but now holds a value (e.g. an overlay addition). **Clickable**: clicking it reverts the field to the value it loaded with (restoring the line, clearing it, or re-filling it as needed) and clears the row's changed state |

Stable row order: keys present in the file first, then firmware-known keys
from `FILE_KEYS` that the file omits. This keeps rows from jumping when the
user fills / clears a key.

### 2.3 Field semantics

* **`<not set>` vs `""` vs `0`**: distinct. A row whose input is empty is
  `<not set>` and will be omitted from the file on serialize / sent as
  `del-config` on upload. A `0` is a real value; clearing it still shows
  "was: 0" until reverted.
* **String emptiness**: a string set to `""` (e.g. `password=`) is preserved
  as such in the file — it is not the same as `<not set>`.
* **Clear button** (`×`): removes the key from the file model. Visible only
  when the input has a value; clicking it both empties the input and removes
  the key's line from the in-memory model.
* **Restore** ("was: …" line): clicking the "was: …" hint reverts the field to
  the value it loaded with. If the key was absent in the source it is cleared
  again; if it was present its loaded value is restored. Reverting clears the
  row highlight and the file's dirty dot.
* **Adding a key**: an "+ add key" row at the bottom of each pane accepts an
  arbitrary `key` + `value` and appends it; thereafter the row is treated
  like a present field.

### 2.4 Raw preview

A `<details class="raw">` collapsible at the bottom of each pane shows the
serialized file contents (kept in sync as the user edits). The serializer is
round-trippable: it preserves comment lines, inline `# comments` after a
value, and original whitespace.

### 2.5 Conf-file parser

A flat `key=value` model that retains every original line as `{raw, key,
value, comment}`. Edits rewrite only the value portion of `raw`; comments
and blank lines pass through unchanged. New keys are appended to the file
just before any trailing blank line.

---

## 3. Saving (file output)

"Download .zip" repacks all carried files plus the serialized `.conf`s into
a new zip. The name reflects whether anything was changed: if any field is
dirty it is `<source>-edited.zip`; if nothing was touched (e.g. a freshly read,
unedited device config) it is `<source>-backup-<YYYY-MM-DD>.zip` (today's local
date). `<source>` is the device hostname / zip / folder name with any
`.zip` and any " ← overlay" suffix stripped. Synthetic, untouched tabs are
dropped (only synthetic files the user actually filled get written).

---

## 4. Type and metadata lookup

Two compile-time tables, both hand-mined from the firmware source:

* `FILE_KEYS[file]` — keys the firmware reads from that file.
* `TYPE_KEYS[type]` — keys read with `ConfFile::getByte / getLong / getFloat
  / getString`. Determines the type pill.
* `META[key]` — `{unit, desc, type}` shared across files.
* `FILE_META[file][key]` — per-file overrides (e.g. `vout_max` differs between
  `limits.conf`, where it is the legacy role alias of `lv_max`/`hv_max`,
  `charger.conf` and `converter.conf`).
* `DEFAULTS[key]` + `CHAN_SUFFIX_DEFAULT[suffix]` — default values scraped
  from `ConfFile::get*(key, default)` calls in `src/`. When present, the row's
  description is suffixed with " · default: \<value\>". Strings are kept
  quoted (`""`, `"buck"`) so the value's type is unambiguous. Keys without
  a hard-coded default (`L0`, `pwm_freq`, `hv_max`, …) are treated as
  required by the firmware and left unannotated.
* Channel-prefixed sensor keys (side `hv_v_`, `lv_v_`, `hv_i_`, `lv_i_`; legacy
  role `vin_`, `vout_`, `iin_`, `iout_`; and `ntc_`) plus a suffix table for
  `adc / ch / rh / rl / factor / midpoint / filt_len`.
* `wifi.conf` is pattern-based — keys matching `ssid_<name>` and
  `ssid_<name>_psk` are treated as string SSID / passkey pairs.

When a key has no entry in any of these tables, the row still renders, but
shows the `?` warning pill.

### 4.1 Side and role keys

`sensor.conf` channels and the four mapped `limits.conf` limits can be named by
side (`hv_v_*`, `lv_v_*`, `hv_i_*`, `lv_i_*`; `hv_max`, `lv_max`, `hv_i_max`,
`lv_i_max`) or by legacy role (`vin_*`, `vout_*`, `iin_*`, `iout_*`; `vin_max`,
`vout_max`, `iin_max`, `iout_max`). `converter.conf` `topo` maps one to the
other (`src/conv_side.h`). Each file uses one form throughout; the firmware
refuses a mix. `vin_min`, `iout_short`, `p_max` and `ntc_*` are outside the rule.

* Placeholders and add-key suggestions offer the side form, unless the file
  already has role keys and no side keys; then they offer the role form, so a
  legacy file is never nudged into a mix. Legacy files load and serialize
  unchanged.
* A file holding both forms shows the firmware's error above its rows, naming
  one key of each form:
  `limits.conf: mixes side key hv_max with role key vout_max; use side keys (hv_/lv_) or role keys throughout, not both`.
  It updates as extras are filled; it does not block download or upload.

**When adding, renaming, or removing a key in firmware** the editor's tables
(`META`, `FILE_META`, `FILE_KEYS`, `TYPE_KEYS`, `DEFAULTS`) and
`website/docs/reference/config/<file>.md` must be updated together so the three sources don't
drift.

**Automation:** `etc/config-tool/scrape_conf_keys.py` walks `src/` and `main/`
for every `ConfFile::get*(key, default)` call and rewrites the three
auto-managed blocks in `conf-editor.html`. They are delimited by
`// SCRAPER:BEGIN <NAME>` / `// SCRAPER:END <NAME>` comments around
`FILE_KEYS`, `TYPE_KEYS` and `DEFAULTS`. The hand-curated `META`,
`FILE_META`, `CHAN_*` tables and prose descriptions are *not* touched. The
script also reports keys read by firmware that have no `META[]`
description (so you know what to write) and `META[]` entries that point at
keys no longer seen in firmware (likely stale). Usage:

```
python3 etc/config-tool/scrape_conf_keys.py            # drift report
python3 etc/config-tool/scrape_conf_keys.py --write    # apply
python3 etc/config-tool/scrape_conf_keys.py --check    # CI gate
```

Keys the literal-string scraper cannot reach (runtime key vars like
`boardConf.getByte(pnCtrl)`, or inline `ConfFile{path}.getByte(...)`
temporaries) are hand-listed inside the script's `EXTRA_KEYS` table.

---

## 5. Device connection (transports)

All three transports share the same line-oriented protocol and the same
upload code path. A single `lineListeners: Set<(line)=>void>` dispatches
every received line to whoever is interested (`getConfig`, `sendCommand`,
`fetchHostname`, and the device-log panel).

Helpers:

| Name                                 | Purpose                                                              |
| ------------------------------------ | -------------------------------------------------------------------- |
| `getConfig(file, timeoutMs=3000)`    | Sends `get-config <file>`, returns `{key:value, …}` when device echoes `OK: get-config <file>` |
| `sendCommand(cmd, onRx, timeoutMs)`  | Sends one command; resolves `true`/`false` on `OK:`/`ERR:` echo; streams lines to `onRx` |
| `fetchHostname()`                    | Sends `hostname`, picks the device's reply for labelling             |
| `importFromDevice(label)`            | Loops `FILE_KEYS`, calls `getConfig` (4000 ms) for each, then `load()` the result. Retries a file up to 3× **on timeout only**: a missing file still replies fast with the `OK: get-config` marker and zero keys (authoritatively absent — no retry), so a timeout means the command/reply was dropped (common over MQTT QoS 0) and the file would otherwise falsely look "not on device". Files still lost after retries are listed in the status line. |

### 5.1 Serial

Web Serial (Chrome / Edge over localhost or https). Opens at 115200, runs
a background read loop that re-acquires the reader on transient errors.
Writes go out as plain UTF-8 with `\r\n` terminator. The port stays open
between read and upload so that DTR/RTS isn't toggled and the device isn't
reset between flows.

### 5.2 Bluetooth (NUS)

Chromium-only (`navigator.bluetooth`). UUIDs:
`6e400001-…` service, `…0002-…` RX (write), `…0003-…` TX (notify).
Outgoing data is chunked to 20-byte ATT payloads. Writes use
`writeValueWithResponse` so that the OS triggers pairing on an encrypted
characteristic (justworks / passkey). Connect+discover retries up to 4×
because Chrome/macOS often fails the first GATT attempt. Notifications
feed the same `lineListeners` after `\n` framing.

### 5.3 MQTT

Browser ↔ broker via WebSocket only (`ws://` or `wss://`), using
`mqtt.js`. Connection details (URL, username, password) persist in
`localStorage` between sessions.

**Modal flow:**

1. Click "Connect MQTT" → open modal pre-filled from `localStorage` (default
   URL `ws://192.168.1.200:9001`).
2. Click **Scan for devices** → broker `connect()` then `subscribe pv/log/#`.
   Immediately after the subscribe succeeds, every host previously seen on
   *this broker URL* (cached in `localStorage["mqtt-known-hosts"]` as
   `{<url>: [host, …]}`, up to 32 per URL) is probed by publishing the
   benign command `ip` to `pv/log/<host>/cmd`. For 5 s, every 3-segment
   topic `pv/log/<host>` contributes its host to a set, capturing both
   passive chatter and probe replies. The list re-renders every 0.8 s as
   devices appear.
3. Click a device → the scan client is *handed off* (its message listener is
   rebound, no second CONNACK) and the regular `importFromDevice(hostname)`
   flow runs. On success the hostname is added to the per-URL cache.
4. The full `{url, username, password, device}` is cached as
   `grantedMqttConfig` so the upload can silently reconnect later.

Brand-new devices still need to chirp once on `pv/log/<host>` before they
can be cached; once cached they show up on subsequent scans even when
quiet.

**Wire protocol** (mirrors `etc/fugu/transport.py::MqttTransport`):

* Subscribe `pv/log/#`. Only 3-part topics `pv/log/<hostname>` are treated as
  device log; deeper topics (the `/cmd` echo) are ignored.
* Each `write(str)` publishes one trimmed line to `pv/log/<hostname>/cmd`
  with QoS 0 (MQTT is message-framed, not a byte stream).

---

## 6. Upload (changed-only writeback)

"Upload changes to device" walks every conf file and emits one command per
changed field:

* `del-config <file> <key>` for fields the user cleared
* `set-config <file> <key> <value>` for fields the user added or edited

Before sending anything, a `window.confirm()` lists every command verbatim
along with the transport name (Serial / Bluetooth / MQTT). On approval:

1. `ensureConnection()` reuses the live transport, or silently re-opens it
   from the cached handle (`grantedSerialPort` / `grantedBleDevice` /
   `grantedMqttConfig`) when the read flow had closed it.
2. For each command, `sendCommand()` waits for the device's
   `OK: <cmd>` / `ERR: <cmd>` echo.
3. On success the field's `original` value is updated to the new value
   (clearing the dirty mark and the "was: …" hint).
4. The device-log panel (§7) is open during upload, so every reply line is
   visible in real time.

If a transport was newly opened just for the upload, it is closed in the
`finally` block; if it was the live read connection, it stays open.

---

## 7. Device log panel

A fixed-position panel at the bottom-right of the page.

* **Toggle**: floating **Device log** button pinned to the bottom-right
  corner (shown once any transport connect succeeds). Opens the panel; the
  button hides itself while the panel occupies the same corner, and
  reappears on close. An upload also opens the panel implicitly.
* **Persistent listener**: while the panel is visible, a `lineListeners`
  entry feeds every line into the panel body, classified by content:
  `OK: …` → green, `ERR: …` → red, everything else → muted.
* **ANSI colours**: the firmware's `ESP_LOG*` output carries CSI SGR escapes
  (`\x1b[0;32m` etc.). Lines that contain an escape are passed through
  `ansi-to-html@0.7.2` with `escapeXML:true`, and the resulting safe HTML is
  inserted as DOM nodes via `Range.createContextualFragment` (no
  `innerHTML` on the live tree). Lines without escapes use plain
  `textContent`.
* **No line wrap**: `white-space:pre`. Long lines produce a horizontal
  scrollbar on the panel instead of wrapping.
* **Tail-follow**: the panel auto-scrolls to the bottom on each new line
  *only* if the user was already within 4 px of the bottom. Scrolling up
  pauses following; scrolling back resumes it.
* **Command input**: a text field + Send button at the bottom of the panel.
  Submitting (Enter or Send) echoes `» <cmd>` and writes through
  `activeWrite` to whichever transport is current. Input is disabled with a
  "connect to a device first" placeholder when no transport is active.

---

## 8. Edge-case rules (collected from the design notes)

* `<not set>` is omitted from the serialised file; `0` is not the same as
  unset; an empty string `""` is not the same as unset either.
* Deleting a key with value `0` still shows "was: 0" until reverted. Same for emtpy strings: del → "was: '''"
* Comments and blank lines in the file are preserved across round-trip.
* New keys added in the editor are appended just before any trailing blank
  line.
* Synthetic ("not on device") tabs are skipped on download unless the user
  filled at least one key.
* The console marker matchers use `includes(...)` rather than
  `endsWith(...)`: the device's reply line can carry a trailing whitespace
  or terminator char that defeats `endsWith`.
* `lineListeners` is the single fan-out point — adding a new transport only
  means feeding it newline-framed text and providing an `activeWrite(str)`.

---

## 9. Not implemented (gaps vs. `vibe.md`)

Items mentioned in `vibe.md` that this spec does **not** cover (because the
HTML editor does not actually do them):

* **Tests**. `vibe.md` calls for a test for every feature, a headless run,
  tests against single `.conf` / folder / `.zip`, and regression coverage.
  An initial suite exists under `etc/config-tool/test/` (node:test + jsdom,
  no network at run time; see `test/README.md`) covering parsing,
  type/meta lookup, ZIP round-trip, load + synthetic tabs, edit/dirty
  semantics, the device-log panel (incl. ANSI, tail-follow, command input)
  and the line-oriented helpers + MQTT modal. Still **missing**: a real
  Web Serial / Web Bluetooth path (those need a chooser gesture; only
  protocol layer is covered) and an end-to-end upload exerciser.

* ~~**Automated re-scrape of `ConfFile` keys**~~ — covered by
  `etc/config-tool/scrape_conf_keys.py` (see §4). `FILE_KEYS`, `TYPE_KEYS`
  and `DEFAULTS` are regenerated from `src/`; `META`/`FILE_META` prose still
  needs a human eye.

* **CLI companion tool** (`conf-tool.py` flow described under "web app
  editor with upload"). `etc/config-tool/conf-tool.py` exists but is out of
  scope for this HTML editor and is not exercised by anything here.
