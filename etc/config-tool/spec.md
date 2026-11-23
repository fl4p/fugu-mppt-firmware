*this document is an LLM generated placeholder*

# `conf-editor.html` — specification

A single-page HTML editor for Fugu MPPT board-config files (the `.conf` files
under `config/<board>/conf/`). Loads a set of conf files from a `.zip`, a
folder, a shipped board preset, or live from a device (USB serial, Bluetooth or
MQTT), lets the user edit fields with unit/description/default hints, then
writes the result back as a downloadable `.zip` or as a stream of
`set-config` / `del-config` commands.

Self-contained: one HTML file, no build step. One external CDN dependency
(browser-side only): `mqtt@5` (WebSocket MQTT client, pinned with SRI). The
ANSI colour converter for the console is inline, so log colouring works
offline.

---

## 1. Loading config

### 1.0 Start screen

Before anything is loaded the header shows only the wordmark ("Fugu MPPT
config"). The page body is a centred column: the heading "Board
configuration", a one-paragraph intro, and three choices separated by rules.
Each choice is a row with an icon, a title and a one-line explanation; clicking
it expands its actions in place (chevron rotates, `aria-expanded` toggles).
The choices are mutually exclusive: opening one collapses the others.

| Choice                  | Actions in the expanded pane                                         |
| ----------------------- | -------------------------------------------------------------------- |
| **Connect to a device** | **MQTT broker…**, **Bluetooth**, **USB serial**, plus a fine-print line on browser requirements |
| **Open a file**         | **Choose .zip**, **Choose folder**, plus "Files other than `*.conf` are kept and written back unchanged." |
| **Start from a preset** | a preset `<select>` and a **Load** button; expanding it fetches `presets/index.json` |

Below the choices: "You can also drop a .zip or folder anywhere on this page."
The `#drop` element fills the viewport below the header and is the drop target;
a dashed outline (`#drop.hover::after`) appears only while dragging. Drop is
handled only while the start screen is visible.

`#drop-msg` under the hint carries the status line on the start screen (see §2.9).

Returning to the start screen from the editor (Open menu → **Start screen**)
shows a "Back to <source>" link above the hint that re-renders the editor
without reloading.

### 1.1 Sources

| Source          | Start screen                             | Open menu (editor)        | Implementation                                      |
| --------------- | ---------------------------------------- | ------------------------- | --------------------------------------------------- |
| `.zip` archive  | Open a file → **Choose .zip**, or drop   | **Choose .zip…**          | built-in zip reader                                 |
| folder          | Open a file → **Choose folder**, or drop | **Choose folder…**        | `webkitdirectory`; dropped folders walk `FileSystemEntry` |
| USB serial      | Connect → **USB serial**                 | **USB serial**            | `navigator.serial.requestPort()`, 115200 baud       |
| Bluetooth (NUS) | Connect → **Bluetooth**                  | **Bluetooth**             | `navigator.bluetooth.requestDevice()`               |
| MQTT            | Connect → **MQTT broker…**               | **MQTT broker…**          | opens the scan modal (§5.3)                         |
| board preset    | Start from a preset → **Load**           | (via Start screen)        | `presets/index.json` + one fetch per file           |

Source label (shown in the header, used for download names): the zip file name;
the top-level folder name; the first dropped entry's name; "preset <label>"; for
a device the `hostname` reply (fallback "serial-device" / "ble-device"; MQTT
uses the picked hostname).

Loading any source clears any previously loaded state, except for the overlay
case in §1.3. If the loaded state has changes that were neither pushed nor
saved, replacing it (a device read, or a file or preset loaded without a prior
device read) first asks "N change(s) not pushed or saved will be lost.
Continue?"; declining keeps everything. For a device read the question comes
after the device chooser and before the current link is dropped. While anything
is changed, leaving or reloading the page triggers the browser's leave-page
prompt (`beforeunload`). Only `*.conf` files are parsed for editing; any other file in the
zip/folder is carried opaquely and re-emitted on download (same byte payload).

ZIP support: built-in reader (stored + deflate via `DecompressionStream`) and
writer (stored only). No external zip dependency. A browser without
`DecompressionStream` gets a status note at startup.

### 1.2 Board presets

`presets/` is a **generated** mirror of the `config/<board>/conf/*.conf` dirs
(`node etc/config-tool/gen_presets.mjs`, gitignored; `config/dl/` captures are
skipped). The editor fetches `presets/index.json`, then one
`presets/<id>/conf/<file>` per file, and feeds them to the same path as a folder
upload, so the source is `config/`, never a hand-maintained second copy. The
website build regenerates the same tree into `static/tools/presets/`
(`website/scripts/copy-tools.mjs`). Fetch needs the page on http(s); over
`file://` the preset pane reports "Presets unavailable: … run: node
etc/config-tool/gen_presets.mjs". Each option reads "<label> — N files".

### 1.3 Overlay after a device read

Exception to "clears previously loaded state": once the config has been read
**live from a device** (`state.fromDevice`), opening a `.zip`, folder or preset
does **not** reset the editor. Instead the values are *overlaid* onto the live
read as pending edits (`mergeUpload`), keeping each file's `original` (the
device values) as the baseline. Every value that differs from the device shows
up as a change (amber bar, "was: …", per-file count in the file list, Review
count), so the user can check what the file would alter before pushing it.

* Within a file the upload includes, a key the device reported but the uploaded
  file omits is **cleared** (removed from the model, shown as `<not set>`, and
  flagged as changed): the uploaded file fully replaces that file.
* Files the upload doesn't include at all are left untouched (omitting a whole
  file never clears its device keys).
* Files are matched by basename; a file the device didn't report is added under
  "Not on device".
* The source label becomes `<device> ← <upload>`, and the editor jumps to the
  first changed file unless the active file already has changes. The status
  line reports how many fields differ.

To overlay from the editor, use the Open menu's **Choose .zip…** /
**Choose folder…**, or **Start screen** and then drop or pick a preset.

---

## 2. Editor UI

Visual language: ink on a neutral grey scale. Amber (`--accent`, `#b35c00`
light / `#f0a24a` dark) is used only for state: a changed row's bar, input
border and "was: …"; the per-file change count; errors and invalid values;
role-key files. The live-connection dot and the start-screen icons use a
separate green (`--icon`). Focus rings are ink. System sans for
text, mono for keys, values and file names. No web fonts.

### 2.1 Header (top bar)

Sticky, 56 px. After a load it shows, left to right:

* the wordmark;
* the **source label** (`#srcname`, ellipsised, full text in the tooltip);
* the **connection state** (`#conn`), always about the push target (§2.7):
  "Connected to <host> over USB serial|Bluetooth|MQTT" (green dot) while the
  link to the device the config was read from is open, "Disconnected from
  <host> over …" when that link is closed (a push reopens it), "Connecting …"
  during a connect and read, and "Not connected" (grey dot) when there is no
  push target. Every connect, success or failure, updates it;
* the **status line** (`#serial-status`, §2.9);
* right-aligned actions: the **Open** menu, the **Console** toggle
  (`aria-pressed`), the **Services** button (§2.14; icon only at ≤520 px), and
  the primary **Review changes (N)** button.

**Open menu** (`<details>` dropdown, closes on an outside click, after a
choice, or with Esc, which returns focus to the **Open** button):

* "Read from a device": **MQTT broker…**, **Bluetooth**, **USB serial**
* "Open a file": **Choose .zip…**, **Choose folder…**
* **Start screen**

**Primary button** (`#review-btn`): solid ink "Review changes" with a count
badge when anything changed; outlined "Save a copy" (no badge) when nothing
changed. Both open the review sheet (§2.7).

### 2.2 File list

A grey left column (216 px, sticky, scrolls on its own) with one entry per conf
file, labelled without `.conf` in mono (tooltip: full path). Grouped under small
headings:

* **Hardware**: `board`, `sensor`, `limits`, `coil`, `converter` (`SENSITIVE`);
  tooltip suffix "— sensitive (hardware/calibration)".
* **Settings**: every other loaded file.
* **Not on device**: synthetic files (below).

Order within a group follows `TAB_ORDER` (`board, sensor, limits, coil,
converter, charger, tracker, mqtt, tele, pprof, wifi`), then unknown files
alphabetically.

* **Synthetic files**: every file the firmware reads (`FILE_KEYS`) gets an
  entry even if the source lacks it. These render faint, sit in "Not on
  device", and carry the tooltip "(not on device — fill a key to create)". An
  untouched synthetic file is *not* exported or pushed.
* **Change count**: an amber dot plus the number of changed fields
  (`.dot`, shown via `style.display`; tooltip "N changed"). Reverting every
  field hides it.
* **Role-key files** (§4.1) are drawn in amber, with the error appended to the
  tooltip.
* Entries are keyboard-reachable (`tabIndex=0`, Enter/Space), the active one
  has `aria-current="page"` and a white raised background.

### 2.3 File pane

A centred column (max 780 px):

* **Heading**: the file name in large mono, with `.conf` faint.
* **Meta line**: `<path>, N of M keys set` and a link-style toggle
  (**Show all M keys** / **Show set keys only**, §2.4). M is the file's row
  count (keys in the file plus firmware-known keys it omits).
* **Note line**: the `FILE_DESC` sentence for the file; Hardware files append
  "A wrong value can damage the converter." Synthetic files instead read "Not
  on the device. Fill in a key to create this file; left empty, it is not
  written."
* **Role-key error** (`.err.keyform`, amber left rule), only when present.
* The key rows (§2.5), grouped as a prefix tree.
* **Footer** (above a rule): two folded disclosures, **Add a key** and
  **Show <file> as text** (§2.8).

Stable row order: keys present in the file first, then firmware-known keys from
`FILE_KEYS` that the file omits. This keeps rows from jumping when the user
fills or clears a key.

**Prefix groups** (`keyTree`): keys sharing a `_`-separated prefix with 2+
members form a collapsible group (`<details class="kgroup">`, chevron, prefix
name in mono). Single-branch chains are merged; single keys come before groups
at each level. Inside a group the shared prefix of each key is drawn faint.
Groups start collapsed; an opened group stays open per file + prefix for the
session, and jumping to a key from the review sheet opens its groups. A collapsed
group that contains changed or invalid rows (also in nested groups) shows an
amber dot after its name with the number of changed rows and ", K invalid"
(tooltip "N changed, K invalid"); the marker is hidden while the group is open.

### 2.4 Short view and Show all

Each file opens in the **short view**: rows for keys that are set in the file,
were loaded from the source, are changed/filled, or are listed in `COMMON` for
that file (`mcu`, `pwm_freq`; the four side limits; `L0`; `topo`, `mode`;
`vout_max`, `cv_eoc`, `cv_float`, `ibat_max`, `bat_c`; `enabled` +
`broker_uri` / `influxdb_host`). The other rows are still rendered, hidden
(`.rows-hidden`, `.is-hidden`), so the full model is always in the DOM.

**Show all M keys** switches that file to the full tree with every
firmware-known key; group headings then show a count ("N keys" when all are
set, else "n of N set"). **Show set keys only** switches back. The choice is
per file and kept for the session.

When the short view has no rows: "No keys are set in <file>. **Show all M keys**
to fill one in." A file with no known keys at all: "<file> has no known keys.
Add one below."

### 2.5 Rows

Two columns (label | value, the value column 200–300 px), whitespace-separated,
no borders.

| Element        | Behaviour                                                          |
| -------------- | ------------------------------------------------------------------ |
| key            | mono `<label>`, linked to the text input or select (a click focuses it); on a switch row it only names the switch (`aria-labelledby`), so a click on the key does not toggle the value. Tooltip carries the type: "<type>, read with ConfFile::get<Type>()" or "type unknown", plus "; no metadata in the firmware tables" when no description exists |
| description    | muted, one line with ellipsis (two on mobile); expands while the row has focus or when clicked; full text in the tooltip. Source: `FILE_META` / `META` / channel pattern / ssid pattern; else the file's inline `# comment` suffixed "(from file comment)"; else faint italic "No description". A known default follows in a fainter tone as "default: <value>" (bool defaults as on/off) |
| value          | text input on a grey fill (mono); `<select>` for keys with `opts`; a switch for bool keys (§2.6). Unset firmware-known keys show the placeholder `<not set>` on a transparent, outlined field |
| unit           | own column right of the input (V, A, Hz, GPIO, …), muted          |
| `×`            | clears the key (§2.6); shown only when the field has a value, and only on row hover/focus (always faintly visible on touch devices) |
| number warning | under the input, amber (§2.6)                                     |
| "was: X revert" | amber mono line under the input when the value differs from the loaded one (§2.6) |

A changed row gets a 2 px amber bar at its left edge and an amber input border
(a ring around a switch).

### 2.6 Field semantics

* **`<not set>` vs `""` vs `0`**: distinct. An unset key is omitted from the
  file on serialize and sent as `del-config` on push. `0` is a real value. A
  string set to `""` (e.g. `password=`) in the source is kept as `key=` while
  untouched.
* **Emptying an existing key by hand** keeps `key=` in the serialized file
  (download), but counts as removed for the push (`del-config`), since an empty
  value can't be sent. Use `×` to drop the line from the file too.
* **Clear** (`×`): removes the key's line from the model and re-renders the row
  as `<not set>`.
* **Changed**: a key present in the source is changed when its value differs
  from the loaded one; a key absent from the source is changed once it holds a
  non-empty value. Row bar, file-list count and Review count use the same rule
  (`changedFields`).
* **Revert** ("was: …" line): text "was: <value>", "was: (empty)" for an empty
  string, "was: (not set)" for a key the source didn't have, followed by a
  muted "revert". It is a `role=button` (click, Enter or Space) that restores
  the loaded value (re-filling, restoring or clearing the line as needed) and
  clears the change mark.
* **Bool keys** (`type:"bool"`) whose value is `""`, `0` or `1` render as an
  on/off switch with the text "on"/"off"; unset shows a hollow track and
  `<not set>` (or "(empty)" for a present empty key). Other values fall back to
  a 0/1 select so nothing is lost.
* **Enum keys** (`opts`): a select whose first option is `<not set>` /
  "(empty)"; an unknown current value is kept as an extra option.
* **Number check** (`valueProblem`, non-blocking): mirrors the firmware's
  `ConfFile` getters (`src/conf.h`) on the trimmed value, so it flags what the
  firmware throws on at read time or reads differently:
  * `float` (`strtof`): a decimal number with optional sign, dot and exponent,
    or `inf`/`infinity`/`nan`; else "Not a number: …". Values that overflow a
    float or underflow to 0 get "Out of range for a float".
  * `byte` and `long` (`strtol_2_8_10_16`): decimal (a sign forces decimal, so
    `-010` is -10), lowercase `0x…` hex, lowercase `0b…` binary, or a leading
    `0` = octal. Anything else ("08", "0X10", "-0x10", "1.5", "1e3") is "Not a
    whole number: …". An octal value that differs from its decimal reading
    ("010" = 8) gets "A leading 0 means octal: the firmware reads 8". `byte`
    must be 0–255 (the firmware keeps only the low byte), `long` must fit 32
    bits.

  An invalid input gets a dashed amber border and the message; the value is
  still kept and saved. The review sheet repeats the message and the push asks
  once more (§6).

#### 2.6.1 Key dependencies (`DEPENDS`)

A declarative table; one mechanism (`depState`) applies every rule to the rows
and the review sheet. A rule names a file and keys plus one of:

* `if: {file?, key, is:[…]}`: the keys are read only while that key (same file
  unless `file`) has a value in `is`. The value used is the current one, else
  the firmware default (`DEFAULTS`). An unset value without a default, or a
  value outside the controller's `opts`, changes nothing.
* `required`: the key needs a value (while its `if` holds). Without `if`, an
  enum value outside `opts` is flagged too.
* `needs: [cond, …]`: the key, when on (not unset or `0`), works only if one of
  the conditions holds (`{file?, key, is:[…]}` or `{key, not:["0"]}` = set and
  not 0).

Rows:

* An **unused** key (its `if` fails) that is unset and unchanged is hidden,
  also under **Show all**. With a value in the file or a pending change it
  shows faded with "not used with <key>=<value>" (muted), so it can be removed.
* A **used** key (its `if` holds) shows in the short view even when unset.
* A **required** key that is empty always shows, with an amber "required for
  <key>=<value>" ("required: HiLi or InEn" without `if`). A folded group that
  holds one opens.
* A failed `needs` shows "needs <cond> or <cond>" in amber under the field.

Notes are non-blocking. Editing a controlling key of an `if` re-renders the
pane at once (focus stays on the field) and opens the groups of the keys it
controls; other edits refresh the notes in place. A controller in another file
takes effect when that file's tab is shown.

Rules (with the firmware source they come from):

| Keys | Rule | Source |
| ---- | ---- | ------ |
| board `pwm_driver_logic` | required, `HiLi` or `InEn` (no default; other values throw) | `src/buck.h:1205-1243` |
| board `pwm_in`, `pwm_en` | used with `pwm_driver_logic=InEn`, required | `src/buck.h:1208-1222` |
| board `pwm_hi`, `pwm_li` | used with `HiLi`, required | `src/buck.h:1223-1236` |
| board `pwm_sd` | used with `HiLi` (optional, default 255) | `src/buck.h:1231` |
| converter `psu_vout` | used with `mode=psu`, required | `src/mppt.cpp:575-576` |
| converter `pv_isc`, `pv_voc` | used with `mode=pv`, required | `src/mppt.cpp:584-590` |
| converter `pv_k`, `pv_slew`, `pv_iout_span` | used with `mode=pv` | `src/mppt.cpp:584-590` |
| board `hs_clamp` | needs `bflow_bat=1` | `src/mppt.h:1632,1650` |
| board `bflow_bat` | needs `bflow_en` or `bflow_sd` set (pin 0 = none) | `src/pwm/backflow.h:38-39,86` |
| board `bflow_bat`, `hs_clamp` | need converter.conf `topo=buck` | `src/mppt.h:1632`, `src/mppt.cpp:638` |

`psu_slew_down` is read in every mode (`src/mppt.cpp:609`), so it has no rule.

**Gate driver block** (`GATE`): in board.conf the `pwm` group starts with a
block "Gate driver": a quiet line, then `pwm_driver_logic` and the pins, used
ones first. The line names the switching and rectifying pin for converter.conf
`topo` (absent = buck): buck InEn switches `pwm_in`, rectifies `pwm_en`; buck
HiLi switches `pwm_hi`, rectifies `pwm_li`; boost swaps them. With the logic
unset or unknown it says "Set pwm_driver_logic to HiLi or InEn to show the
pins it uses." The pins carry plain labels next to the key (`KEY_LABEL`): HI,
high-side input; LI, low-side input; IN, PWM input; EN, enable; DIS, shutdown
(optional).

### 2.7 Review sheet

Opened by the primary button. A right-hand sheet (520 px) over a scrim on
desktop; a bottom sheet (86 vh, rounded top) on mobile. Closes with **Close**,
Esc, or a click on the scrim. It is a modal dialog: opening it moves focus to
**Close**, Tab and Shift+Tab cycle inside the sheet, and closing returns focus
to the **Review changes** button (jumping to a field focuses that field
instead).

* **Head**: "Review changes" and "N changes in M files", plus ", K invalid"
  when any new value fails the number check.
* **Body**: one section per file (`board.conf  2 changes`), one entry per
  change: the key (click → jump to the field: closes the sheet, switches file,
  expands collapsed groups, scrolls the row to the centre and focuses its input),
  an **Undo** link (reverts that field; the sheet re-renders), and old → new:
  old struck through ("not set" in italics when absent, `""` when empty), new
  value, or italic amber "removed" for a cleared key. An invalid new value gets
  an amber wavy underline and the row's number warning below it.
* With a push target, each change ends with a quiet hint of when it takes
  effect (§6.1): "now", "after restarting mqtt", "after restarting ftp and
  http", "after reboot" or "when next used".
* Above the files, an amber list warns about dependency problems (§2.6.1) that
  involve a changed key, e.g. "board.conf hs_clamp: needs bflow_bat=1" or
  "converter.conf psu_vout: required for mode=psu". It does not block the push.
* A folded disclosure **Commands sent to the device (N)** lists the exact
  commands the push sends (§6.1).
* With no changes: "No changes yet — Edited fields collect here, so you can
  check them before pushing or saving. You can still download this
  configuration as a backup."
* **Footer**: with a push target and changes, a summary of what applying them
  needs, "Takes effect after: restarting mqtt, reboot" (services first, then
  reboot), or "No restart needed."; then a note and the buttons.

| State                          | Note                                               | Buttons |
| ------------------------------ | -------------------------------------------------- | ------- |
| no changes                     | (none)                                             | **Download backup .zip** (primary) |
| changes, no push target        | "Not read from a device. Download the .zip, or read the device first (Open) and then open this file over it to push the differences." | **Download .zip** (primary) |
| changes, push target           | "Each change is sent to <host> over <transport> as the command listed above and confirmed by the device." | **Download .zip**, **Push N changes to <host> over <USB serial\|Bluetooth\|MQTT>** (primary) |
| any invalid value              | "K values are invalid (marked above); the firmware rejects or misreads them. Fix them first, or confirm when pushing." (amber) | as above |

**Push target** (`pushTarget()`): the loaded config came from a device read
(`state.fromDevice`), and the transport and authorized handle stored for the
push (`lastTransport` and `grantedSerialPort` / `grantedBleDevice` /
`grantedMqttConfig`) are the ones that read went over (`state.link`). There is
none while a connect is in progress, after loading a file or preset without a
device read, or after a failed connect with nothing read before. Overlaying a
file onto a device read (§1.3) keeps the target. The link may be closed; the
push reopens it (§6).

### 2.8 Add a key, text view

* **Add a key**: a folded disclosure with a key input, a value input and
  **Add**. The key is appended to the file and then treated like a present
  field. For `sensor.conf` the key input offers the side channel keys
  (`hv_v_*`, `lv_v_*`, `hv_i_*`, `lv_i_*`) the file doesn't list yet as a
  datalist, and its placeholder shows the first one.
* **Show <file> as text**: a folded `<pre>` with the serialized file, refreshed
  while open. The serializer is round-trippable: it preserves comment lines,
  inline `# comments` after a value, and original whitespace.

### 2.9 Status line and busy spinner

`showStatus(msg, isErr, busy)` writes the same text to two places:

* **Start screen**: `#drop-msg` under the hint. The header status is hidden
  while nothing is loaded, so the text shows once.
* **Editor**: `#serial-status` inline in the header after the connection state
  (ellipsised, full text in the tooltip); on mobile it takes its own row and
  wraps.

Errors are amber. With `busy`, a small rotating ring (`.busy::before`) precedes
the text. Busy messages: "connecting to <device> …" (Bluetooth), "BLE connect
attempt i/4 failed (…) — retrying…", "connected — reading config from device …"
(serial, Bluetooth) or "… from <host> …" (MQTT), "reading config from <label>
…", "retrying N file(s) …". A successful read clears the line; a read with lost
files reports "read N file(s) from <host> — K timed out, skipped: …". Push
progress ("uploading i/N: <file>:<key> …") has the spinner; the final
"uploaded k/N changed fields[ — failed: …]" does not. On mobile the header
status line wraps instead of being cut off.

### 2.10 Console

The **Console** button toggles a sheet (640 px wide, right-hand, below the
header so the header buttons stay usable, no scrim) on desktop; at ≤760 px it
is a full-width bottom sheet with a scrim (the same breakpoint as the rest of
the mobile layout). Head: "Console", a status text ("device log", "not
connected", "sending N command(s) to <host> over <transport>", "done: k/N
ok[, f failed]"), **Close**.

* **Persistent listener**: while the console is open, a `lineListeners` entry
  feeds every received line into `#uploadlog-body`, classified: lines with
  `OK: ` in ink, `ERR: ` in amber, everything else muted. Sent commands are
  echoed bold as `» <cmd>`. Empty log: "Device output appears here."
* **ANSI colours**: the firmware's `ESP_LOG*` output carries CSI SGR escapes.
  Lines containing an escape are converted by the inline `ansiConvert` (escapes
  the text, wraps coloured/bold runs in one `<span>` at a time, colours are CSS
  variables `--a30…--a97` per theme) and inserted via
  `Range.createContextualFragment`; other lines use `textContent`.
* **No line wrap**: `white-space:pre`; long lines scroll horizontally.
* **Tail-follow**: `#uploadlog` auto-scrolls on a new line only if the user was
  within 4 px of the bottom.
* **Command input**: a field + **Send** at the bottom; Enter or Send echoes
  `» <cmd>` and writes through `activeWrite`. Disabled with the placeholder
  "connect to a device first" when no transport is active.
* A push opens the console implicitly and clears its log first.
* Esc closes the console when neither the review sheet nor the Open menu is
  open; if focus was inside the console it returns to the **Console** button.

### 2.11 Mobile layout

At ≤760 px: the header wraps (status on its own row); the file list becomes a
horizontal scrolling strip under the header, with the group headings
(Hardware, Settings, Not on device) as small inline labels in the strip; rows stack
(label above value); descriptions clamp to two lines; both sheets become bottom
sheets and footer buttons fill the width. At ≤520 px (once loaded) the wordmark
is hidden, the source/connection takes the first row and the actions a
right-aligned row of their own; the Open menu opens to the left edge.

### 2.12 Dark mode

Follows `prefers-color-scheme` (no in-page toggle; `color-scheme: light dark`).
All colours are CSS variables redefined in the dark block, including the
accent, the icon tint, the scrim, the shadow, the chevron SVG and the ANSI
palette.

### 2.13 Conf-file parser

A flat `key=value` model that retains every original line as `{raw, key,
value, comment}`. Edits rewrite only the value portion of `raw`; comments
and blank lines pass through unchanged. New keys are appended to the file
just before any trailing blank line; filled-in firmware-known keys that were
absent are appended at the end on serialize. A key that appears more than once
gets one row; as in the firmware, the last occurrence gives its value, and an
edit rewrites every occurrence so the file stays unambiguous.

### 2.14 Services sheet

The **Services** button (server icon, `aria-expanded`) is enabled only while
`svcTarget()` is non-null: there is a push target (§2.7) **and** its link is
open on that same handle (`activeWrite` is the target's transport and
`serialPort` / `bleDevice` is the granted handle, or the MQTT client is bound
to the target's hostname). It never reconnects or talks to another device; a
closed link means "read the device again". `refreshConn()` re-evaluates it.

A modal right-hand sheet (520 px) over its own scrim (`#svc-scrim`); a bottom
sheet on mobile like the others. Opening moves focus to **Close**, Tab cycles
inside, Esc (handled in the capture phase, before the other sheets' handler),
the scrim, **Close** and **Start screen** close it, focus returns to the
button. Head: "Services", the target ("<host> over <transport>", or the
running command with a spinner), **Refresh** (↻), **Close**.

* **Read**: `svc list` on open, on Refresh and after every action, via
  `sendCommand` (completion = the `OK: <cmd>` / `ERR: <cmd>` marker, 4 s).
  `parseSvcList()` strips SGR escapes, starts at the header line
  `NAME STATE LOG ENABLED DETAIL` (no header → incomplete reply), and takes
  each line matching `<name> Running|Stopped|Failed error|warn|info yes|no
  [detail]`, tolerating an ESP log prefix; the detail is the rest of the line
  (may be empty or contain spaces). Other lines (the interim status line
  before the marker, interleaved log output) are ignored; a repeated name
  keeps its last row.
* **Row**: name (mono), state dot + text (green Running, grey Stopped, amber
  Failed), the detail below; controls: log-level select (`svc log <name>
  <level>`), **Restart** (`svc restart <name>`, disabled while the service is
  off), and an on/off switch (`svc on|off <name>`). Tooltips and the footer
  note say that on/off and the level are saved on the device and apply at
  once. All controls are disabled while a command runs.
* **Result**: `ERR` shows "The device rejected “<cmd>”: <reason>" with the
  reason from the firmware's `svc: …` warning; a `start failed (state=…)`
  line after `svc on` shows "<name> did not start: …"; no marker shows
  "No reply to “<cmd>” within 4 s". Errors are amber in the footer.
* **Loaded copy**: after a confirmed `svc on|off|log`, the loaded
  `<name>.conf` gets the saved `enabled` / `log_level` as its original value
  (and as current value unless that field has a pending edit), so the review
  sheet and downloads match the device.
* **Confirms** (`window.confirm`): stopping or restarting the service the
  editor's own link runs over (`mqtt` over MQTT, `ble` over Bluetooth) says
  the link drops; stopping also says another link is then needed to turn it
  back on. Stopping `mqtt` over any link says a charger that gets BMS data
  over MQTT falls back to its stale-cell voltage cap. Stopping adds that the
  setting is saved and survives a reboot. Declining sends nothing and resets
  the control.
* **Own link**: such a command waits 2.5 s for its marker (it usually never
  arrives). After a stop the editor closes that link, so the header reads
  "Disconnected from <host> over …" and the sheet shows "Not connected".
  After a restart it closes a Bluetooth link and reconnects to the same
  granted device, or keeps the MQTT session; it then retries `svc list` up to
  4 times, 1.5 s apart: on an answer "<name> restarted; the <transport> link
  is back", otherwise it closes the link and says to read the device again.
* When the link closes or changes while the sheet is open, the rows are
  replaced by "Not connected" and Refresh is disabled.

---

## 3. Saving (file output)

**Download .zip** / **Download backup .zip** in the review sheet repacks all
carried files plus the serialized `.conf`s into a new zip. The name reflects
whether anything was changed: if any field is changed it is
`<source>-edited.zip`; if nothing was touched (e.g. a freshly read, unedited
device config) it is `<source>-backup-<YYYY-MM-DD>.zip` (today's local date).
`<source>` is the device hostname, else the source label with any `.zip` and
any " ← overlay" suffix stripped. Synthetic, untouched files are dropped (only
synthetic files the user actually filled get written).

---

## 4. Type and metadata lookup

Compile-time tables, hand-mined or scraped from the firmware source:

* `FILE_KEYS[file]` — keys the firmware reads from that file.
* `TYPE_KEYS[type]` — keys read with `ConfFile::getByte / getLong / getFloat
  / getString`. Drives the key tooltip and the number check (§2.6).
* `META[key]` — `{unit, desc, type, opts?}` shared across files. `type:"bool"`
  gives a switch, `opts` a select.
* `FILE_META[file][key]` — per-file overrides (e.g. `vout_max` differs between
  `charger.conf` and `converter.conf`).
* `DEFAULTS[key]` + `CHAN_SUFFIX_DEFAULT[suffix]` — default values scraped
  from `ConfFile::get*(key, default)` calls in `src/`, shown as "default:
  <value>" after the description. Strings are kept quoted (`""`, `"buck"`) so
  the value's type is unambiguous. Keys without a hard-coded default (`L0`,
  `pwm_freq`, `hv_max`, …) are treated as required by the firmware and left
  unannotated.
* Channel-prefixed sensor keys (side `hv_v_`, `lv_v_`, `hv_i_`, `lv_i_`, and
  `ntc_`) plus a suffix table for `adc / ch / rh / rl / factor / midpoint /
  filt_len / gain / offset` (`CHAN_SUFFIX`, `CHAN_SUFFIX_TYPE`).
* `wifi.conf` is pattern-based — keys matching `ssid_<name>` and
  `ssid_<name>_psk` are treated as string SSID / passkey pairs.
* `FILE_DESC[file]` — the one-sentence note under each file heading;
  `COMMON[file]` — unset keys kept in the short view (§2.4).

When a key has no entry in any of these tables, the row still renders, with
"No description" and a tooltip saying there is no metadata.

### 4.1 Side keys and removed role keys

`sensor.conf` channels and the four mapped `limits.conf` limits are named by
side (`hv_v_*`, `lv_v_*`, `hv_i_*`, `lv_i_*`; `hv_max`, `lv_max`, `hv_i_max`,
`lv_i_max`); `converter.conf` `topo` maps them to roles (`src/conv_side.h`).
The firmware no longer reads the role keys (`vin_*`, `vout_*`, `iin_*`,
`iout_*`; `vin_max`, `vout_max`, `iin_max`, `iout_max`) and fails setup on one.
`vin_min`, `iout_short`, `p_max`, `ntc_*` and `charger.conf` `vout_max` are not
role keys.

* Placeholders and add-key suggestions offer side keys only.
* A role-keyed file loads and serializes unchanged, but is flagged: its
  file-list entry turns amber, its role-key rows' descriptions read in amber
  "No longer read: role key, migrate to <buck key> (buck) / <boost key> (boost)
  with etc/migrate_side_keys.py", and the firmware's error for the first role
  key shows above the rows, with the topo taken from the `converter.conf` next
  to it (buck when absent), e.g.
  `limits.conf: vin_max is no longer read; with topo=boost use lv_max. Migrate the file with etc/migrate_side_keys.py.`
  It updates as extras are filled; it does not block download or push.

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
push code path. A single `lineListeners: Set<(line)=>void>` dispatches
every received line to whoever is interested (`getConfig`, `sendCommand`,
`fetchHostname`, `importFromDevice`, and the console).

Connect flow (serial, Bluetooth, MQTT alike): after the device chooser (or the
MQTT device pick) and the discard prompt (§1.1), any open link is closed and the
new one opened. Its transport and handle become the push target only when
`importFromDevice` succeeds. A failed connect or read closes the new link and
keeps the previous read's state and target (now disconnected), so a later push
still goes to the device that state was read from; the header is updated either
way. A second connect is ignored while one is in progress.

Helpers:

| Name                                 | Purpose                                                              |
| ------------------------------------ | -------------------------------------------------------------------- |
| `getConfig(file, timeoutMs=3000)`    | Sends `get-config <file>`, returns `{key:value, …}` when the device echoes `OK: get-config <file>` |
| `sendCommand(cmd, onRx, timeoutMs=4000)` | Sends one command; resolves `true`/`false` on the `OK:`/`ERR:` echo; streams lines to `onRx`, and `onRx` returning `true` resolves `true` early (`restart` never echoes) |
| `fetchHostname()`                    | Sends `hostname`, picks the "Hostname: <name>" reply for labelling (2.5 s) |
| `importFromDevice(label, link)`      | One listener collects `Conf '/littlefs/conf/<file>:<key>' = '<value>'` lines for all `FILE_KEYS` files. Sends every `get-config <file>` back-to-back (pipelined), waits up to 6 s for all `OK: get-config <file>` markers, then re-sends only the files without a marker, up to 2 more passes (2.5 s each). A marker with zero lines means the file is authoritatively absent (no retry); a missing marker means the command or reply was dropped (common over MQTT QoS 0). Files still without a marker are listed in the status line and appear as "Not on device". Then `load()`s the result, sets `state.fromDevice`, `state.deviceName` and `state.link`, and commits `link` (`{transport, handle}`) as `lastTransport` and its granted handle (clearing the other transports' handles). |

### 5.1 Serial

Web Serial (Chrome / Edge over localhost or https). Opens at 115200, runs
a background read loop that re-acquires the reader on transient errors.
Writes go out as plain UTF-8 with `\r\n` terminator. The port stays open
between read and push so that DTR/RTS isn't toggled and the device isn't
reset between flows.

### 5.2 Bluetooth (NUS)

Chromium-only (`navigator.bluetooth`). UUIDs:
`6e400001-…` service, `…0002-…` RX (write), `…0003-…` TX (notify).
Outgoing data is chunked to 20-byte ATT payloads. Writes use
`writeValueWithResponse` so that the OS triggers pairing on an encrypted
characteristic (justworks / passkey). Connect+discover retries up to 4×
because Chrome/macOS often fails the first GATT attempt. Notifications
are stream-decoded (a UTF-8 glyph may straddle two notifications) and feed the
same `lineListeners` after `\n` framing. A failed connect suggests removing a
stale OS pairing and freeing the link from other clients.

### 5.3 MQTT

Browser ↔ broker via WebSocket only (`ws://` or `wss://`), using
`mqtt.js`. Connection details (URL, username, password) persist in
`localStorage` between sessions.

**Modal flow** ("Connect over MQTT"):

1. **MQTT broker…** opens the modal pre-filled from `localStorage` (default
   URL `ws://192.168.1.200:1885`).
2. **Scan for devices** saves the fields, connects to the broker and subscribes
   to `pv/log/#` and the retained presence topics `pv/dev/+`. Once subscribed,
   every host previously seen on *this broker URL* (cached in
   `localStorage["mqtt-known-hosts"]` as `{<url>: [host, …]}`, up to 32 per
   URL, most recent first) is probed by publishing `ip` to
   `pv/log/<host>/cmd`. Every 3-segment `pv/log/<host>` topic adds the host;
   `pv/dev/<host>` = `online`/`offline` adds it and marks it offline (faint,
   "offline" label). The list re-renders every 0.8 s ("N device(s) found —
   listening …") and the scan keeps listening until the modal closes or a
   device is picked. If the broker drops, it reconnects every 3 s
   ("reconnecting …"); a failure on the first connect is reported instead.
3. Clicking a device *hands off* the scan client (message listeners rebound,
   `pv/dev/+` unsubscribed, no second CONNACK), closes the modal and runs
   `importFromDevice(hostname, …)`. Once bound, the hostname is added to the
   per-URL cache.
4. Only when that read succeeds is `{url, user, pass, device}` kept as
   `grantedMqttConfig`, so a push can silently reconnect to that device later.

**Cancel**, a click on the backdrop, or picking a device closes the modal. Esc
does not.

**Wire protocol** (mirrors `etc/fugu/transport.py::MqttTransport`):

* Subscribe `pv/log/#`. Only 3-part topics `pv/log/<hostname>` of the bound
  device are treated as device log; deeper topics (the `/cmd` echo) are
  ignored.
* Each `write(str)` publishes one trimmed line to `pv/log/<hostname>/cmd`
  with QoS 0 (MQTT is message-framed, not a byte stream).

---

## 6. Push (changed-only writeback)

**Push N changes to <host> over <transport>** in the review sheet sends one command per
changed field (`pendingCommands`), in file-list order:

* `del-config <file> <key>` for fields that are now empty
* `set-config <file> <key> <value>` for fields the user added or edited
* `svc on|off <name>` for `enabled` = 1/0, and `svc log <name> <error|warn|info>`
  for `log_level`, in the conf file of a service (`<name>.conf`, `SERVICES`:
  mqtt, tele, ftp, http, telnet, tlscon, lcd, scope, ble, bsync). These write
  the conf like `set-config` and apply it at once (`src/cli.cpp:2193-2205`),
  and reply `OK:` / `ERR:` like every command (a service missing from the
  build answers `ERR: …`). Exception: `svc off` for the service the editor is
  connected through (mqtt over MQTT, ble over Bluetooth) stays `set-config`,
  so the push does not cut its own link. Other values, and a cleared
  `enabled`/`log_level` (`del-config`), take effect after a reboot.

`uploadChanged()` refuses ("Nothing to push to: read the config from a device
first") without a push target (§2.7). The review sheet already lists every
change, command and the target device, so pushing from it does **not** ask again
(`uploadChanged({reviewed:true})`), unless a new value is invalid (§2.6): then a
`window.confirm()` lists each invalid `<file>:<key> = <value>: <problem>` and asks
"Push to <host> over <transport> anyway?". A direct `uploadChanged()` call
without the flag also shows a `window.confirm()` with every command and the
target.

1. The review sheet closes and the console opens with a cleared log and
   "sending N command(s) to <host> over <transport>"; the Push button is
   disabled.
2. `ensureConnection(target)` reuses the live link, or silently re-opens it
   from the push target's handle when it was closed.
3. For each command, the console shows `» <cmd>`, the status line "uploading
   i/N: <file>:<key> …", and `sendCommand()` waits for the device's
   `OK: <cmd>` / `ERR: <cmd>` echo (4 s).
4. On `OK` the field's `original` is updated to the new value (or deleted for
   `del-config`), which clears its change mark; `ERR` or a timeout records a
   failure (timeouts also log how many lines arrived).
5. The console status ends with "done: k/N ok[, f failed]" (amber on
   failures), the status line with "uploaded k/N changed fields[ — failed:
   …]". A connection failure shows "connection failed" instead.

If a transport was newly opened just for the push, it is closed in the
`finally` block (the header then reads "Disconnected from <host> over
<transport>"); if it was
the live read connection, it stays open.

### 6.1 When a change takes effect, apply step

The firmware applies nothing on `set-config`/`del-config`. `APPLY` maps a file
(and per-key exceptions) to when its keys take effect; any other file is
"reboot":

| File | Keys | Takes effect |
| ---- | ---- | ------------ |
| mqtt.conf | all | after `svc restart mqtt` |
| tele.conf | all | after `svc restart tele` |
| tele.conf | `ble` | when next used (read at every `tele-ble on`) |
| tele.conf | `adv_ms` | after `svc restart ble` |
| tele.conf | `idle_interval_ms` | after reboot |
| ftp.conf | all | after `svc restart ftp` |
| ftp.conf | `ftp_user`, `ftp_pass` | after `svc restart ftp` and `svc restart http` (HTTP login) |
| http.conf | all | after `svc restart http` |
| tlscon.conf | all | after `svc restart tlscon` |
| bsync.conf | all | after `svc restart bsync` |
| any service conf | `enabled`, `log_level` | now when pushed as `svc …`, else after reboot |
| everything else (board, sensor, limits, coil, converter, charger, tracker, pprof, wifi, ble, lcd, …) | all | after reboot |

A push collects, per push target, the services to restart and whether a reboot
is due (`state.apply`), from the commands the device confirmed. After a push
with no failed command and no connection failure, the console shows an
**apply bar** under its head: "Takes effect after: restarting mqtt" with one
**Restart <name>** button per service, or "Takes effect after: reboot" with
only **Reboot device** when a reboot is due (it covers the restarts). A failed
push hides it; a later clean push to the same target shows what has collected.
The bar only shows while `pushTarget()` is the link that was pushed to, so a
read of another device hides it and its buttons never send elsewhere.

* **Restart <name>** sends `svc restart <name>` and waits for `OK:`; the
  button goes. Restarting the service the editor is connected through (mqtt
  over MQTT, ble over Bluetooth) first asks "Restart <name> on <host>? This
  editor is connected through <name>, so its connection drops."; a missing
  reply then counts as done, and the editor closes the link.
* **Reboot device** asks "Reboot <host> over <transport>? The device stops
  converting and reboots. This editor's connection drops; reconnect from Open
  when the device is back." Only after OK it sends `restart` (the firmware
  stops conversion itself). The device prints `Rebooting` and never answers
  `OK:`; that line, or no reply in 5 s, counts as rebooting: the apply bar
  clears, the editor closes the link (header "Disconnected from <host> over
  <transport>") and the status reads "<host> is rebooting — reconnect from
  Open when it is back". A refused restart (`restart refused`, then `OK:`) is
  reported as an error and the button stays.

A link opened just for an apply command is closed afterwards, as for a push.

---

## 7. Edge-case rules

* `<not set>` is omitted from the serialised file; `0` is not the same as
  unset; an empty string `""` is not the same as unset either.
* Deleting a key with value `0` still shows "was: 0" until reverted. Same for
  empty strings: clearing gives "was: (empty)".
* Comments and blank lines in the file are preserved across round-trip.
* New keys added in the editor are appended just before any trailing blank
  line.
* Synthetic ("Not on device") files are skipped on download and push unless the
  user filled at least one key.
* The console marker matchers use `includes(...)` rather than
  `endsWith(...)`: the device's reply line can carry a trailing whitespace
  or terminator char that defeats `endsWith`.
* `lineListeners` is the single fan-out point — adding a new transport only
  means feeding it newline-framed text and providing an `activeWrite(str)`.
* The pre-redesign element IDs the tests use are kept (`srcname`,
  `serial-btn`, `reload-zip`, `reload-dir`, `download`, `upload`,
  `uploadlog*`, `mqtt-*`), as are the `.dot`, `.orig`, `.clear-btn`,
  `.switch` and `.keyform` classes.

---

## 8. Not implemented (gaps vs. `vibe.md`)

Items mentioned in `vibe.md` that this spec does **not** cover (because the
HTML editor does not actually do them):

* **Tests**. `vibe.md` calls for a test for every feature, a headless run,
  tests against single `.conf` / folder / `.zip`, and regression coverage.
  A suite exists under `etc/config-tool/test/` (node:test + jsdom, no network
  at run time; see `test/README.md`) covering parsing, type/meta lookup, ZIP
  round-trip, load + synthetic files, edit/dirty semantics, the console (incl.
  ANSI, tail-follow, command input) and the line-oriented helpers + MQTT modal,
  plus a Playwright browser smoke test. Still **missing**: a real Web Serial /
  Web Bluetooth path (those need a chooser gesture; only the protocol layer is
  covered) and an end-to-end push exerciser.

* ~~**Automated re-scrape of `ConfFile` keys**~~ — covered by
  `etc/config-tool/scrape_conf_keys.py` (see §4). `FILE_KEYS`, `TYPE_KEYS`
  and `DEFAULTS` are regenerated from `src/`; `META`/`FILE_META` prose still
  needs a human eye.

* **CLI companion tool** (`conf-tool.py` flow described under "web app
  editor with upload"). `etc/config-tool/conf-tool.py` exists but is out of
  scope for this HTML editor and is not exercised by anything here.
