---
title: Config editor
sidebar_position: 2
---

# Config editor

[`etc/config-tool/conf-editor.html`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/etc/config-tool/conf-editor.html)
is a single-page browser editor for the board `.conf` files. It reads them from a device, an archive or a shipped
board preset, shows each key with its unit, description and default, and then downloads a `.zip` or writes only the
changes back to the device. A hosted copy is at [/tools/conf-editor.html](pathname:///tools/conf-editor.html).

The editor is one self-contained HTML file with no build step. The MQTT transport loads `mqtt.js` from a CDN. The
other paths work offline.

## Quick start

Web Serial and Web Bluetooth need a Chromium browser (Chrome, Edge) and a secure context (`https://` or
`localhost`), so serve the file locally:

```bash
cd etc/config-tool
node gen_presets.mjs        # board presets for "Start from a preset" (generated, not committed)
python3 -m http.server 8000
# open http://localhost:8000/conf-editor.html
```

The start screen offers three choices: **Connect to a device**, **Open a file** and **Start from a preset**. Each
one expands in place to show its buttons.

To read, edit and write back a device config over USB, follow these steps:

1. Open **Connect to a device**, click **USB serial** and pick the port. The editor sends `hostname` and one
   `get-config <file>` per known file, then opens the editor.
2. Edit values. A changed field shows "was: … revert" under it, and the file list shows a change count.
3. Click **Review changes** in the top bar. The sheet lists every change as old → new.
4. Click **Push N changes to `<hostname>` over USB serial**. The editor sends each command and waits for its
   `OK:`/`ERR:` reply.
5. Optionally, click **Download .zip** in the same sheet to keep a copy.

`set-config` persists each value to the file on flash. Most files need a reboot before a key is applied. See
[When changes take effect](config/index.md#when-changes-take-effect) for details.

## Sources

The editor can load a config from any of these sources:

| Source          | Button                                           | Notes                                                                 |
|-----------------|--------------------------------------------------|-----------------------------------------------------------------------|
| `.zip` archive  | **Choose .zip**, or drop it on the start screen  | Stored and deflate entries                                            |
| Folder          | **Choose folder**, or drop it on the start screen | e.g. a `config/<board>` folder                                       |
| Board preset    | **Start from a preset**, then **Load**           | A `config/<board>` shipped with the firmware; see below               |
| USB serial      | **USB serial**                                   | Web Serial at 115200 baud; the port stays open between read and push, so the device is not reset |
| Bluetooth (NUS) | **Bluetooth**                                    | Web Bluetooth, Nordic UART service; the OS may prompt for pairing     |
| MQTT            | **MQTT broker…**                                 | WebSocket broker URL (`ws://` / `wss://`); needs `mqtt.conf` `cmd_input=1` |

Once a config is loaded, the **Open** menu in the top bar offers the same device and file sources, plus
**Start screen**. From there a "Back to …" link returns to the loaded config.

Files other than `*.conf` in an archive or folder are kept and written back unchanged.

### Board presets

**Start from a preset** loads one of the board configs under `config/`, so you can edit a known board without a
device or an archive. The list comes from `presets/index.json`, which `etc/config-tool/gen_presets.mjs` generates by
mirroring every `config/<board>/conf/*.conf`. The `presets/` tree is a build artifact and is not committed, so
re-run the script after a config change. The website build regenerates its own copy into `static/tools/presets/`.

### Overlay on a device read

Loading a source replaces the previous config, with one exception: after a live device read, opening a `.zip`,
folder or preset overlays it onto the device values instead. Every value that differs shows up as a pending change,
so you can review what a stored config would alter before pushing it.

- A file included in the overlay fully replaces that file. Keys it omits are cleared.
- Files it doesn't include are left untouched.
- The editor jumps to the first file with a change, and the top bar shows `<device> ← <source>`.

To drop an archive or folder for an overlay, go to **Open** → **Start screen** first.

If the loaded config has changes that were neither pushed nor saved, the editor asks before a load replaces them.
For a device read it asks after you pick the device. Leaving or reloading the page with changes also asks first.

### MQTT scan

**MQTT broker…** opens a dialog for the broker URL, username and password, which are kept in browser
`localStorage`. **Scan for devices** subscribes to `pv/log/#` and the retained presence topics `pv/dev/+`. It also
probes the hosts remembered for that broker with the `ip` command.

The list shows every `<hostname>` seen publishing or present and marks those whose presence is `offline`. It keeps
listening while the dialog is open and reconnects every 3 s if the broker drops. Pick a device to read its config.
See [MQTT topics](mqtt.md).

## Layout

The top bar shows the source name and the connection state. The state always refers to the device a push would go
to: "Connected to `<hostname>` over MQTT" with a green dot while that link is open, "Disconnected from
`<hostname>` over MQTT" once it is closed, "Connecting …" during a read, and "Not connected" when there is nothing
to push to. Its right side holds the **Open** menu, the **Console** toggle, the **Services** button and the
primary button: **Review changes** with a count, or **Save a copy** when nothing has changed.

The file list on the left has one entry per conf file, in three groups:

| Group             | Files                                                                                   |
|-------------------|-----------------------------------------------------------------------------------------|
| **Hardware**      | `board`, `sensor`, `limits`, `coil`, `converter`; a wrong pin, divider or limit can damage the converter |
| **Settings**      | All other files the source contains                                                     |
| **Not on device** | Files the firmware reads but the source lacks. Fill a key to create the file; untouched, it is not written |

A changed file shows an amber dot with its number of changed fields. A file that still uses the removed
input/output role keys is drawn in amber (see [Removed role keys](#removed-role-keys)).

Each file page starts with the file name, its path, "N of M keys set" and a one-line note on what the file
controls. The keys follow, grouped by shared prefix (`i2c_`, `ina22x_`, …) into groups that start collapsed. A collapsed
group that holds changed or invalid keys shows an amber dot with their count. At the bottom sit **Show all N keys**
(see [Short view](#short-view)) and the folded sections **Add a key** and **Show `<file>` as text**.

On a narrow screen the file list becomes a horizontal strip with small group labels, rows stack, and the review,
console and services sheets open from the bottom. The page follows the system light or dark setting.

## Short view

Each file opens in a short view. It shows the keys that are set, changed, or commonly edited (such as `mcu`,
`pwm_freq`, the side limits, `L0`, `topo` and the charge targets). **Show all N keys** at the bottom of the page
lists every key the firmware reads from that file, except unset keys that the current settings make unused (see
[Dependent keys](#dependent-keys)); N counts exactly the rows it shows. Group headings then show how many of their
keys are set. **Show fewer keys** switches back. The choice is kept per file.

## Keys

Each row shows these elements:

- The key name. Its tooltip names the type (`byte`, `long`, `float`, `string`) and the `ConfFile` getter the firmware
  uses. Clicking it focuses a text field; it never toggles an on/off switch.
- A one-line description, followed by the default value where the firmware has one. Click the description or focus
  the field to read the full text. A key without curated metadata says "No description".
- The value: a text field, a dropdown for keys with a fixed set of values, or an on/off switch for bool keys. The
  unit follows the field.
- A **×** button on hover, which removes the key from the file.

An unset key shows `<not set>`. A changed row gets an amber bar and border and a "was: … revert" line under the
field. Click it, or press Enter on it, to restore the loaded value.

A `float`, `byte` or `long` field is checked the way the firmware's `ConfFile` getters parse it. A value that fails
gets a dashed border and a message under the field:

- `float`: a decimal number with an optional sign, dot and exponent (`0.5`, `-1e-3`), or `inf`/`nan`. A comma, a
  trailing letter or a value beyond the float range fails.
- `byte`, `long`: decimal, lowercase `0x` hex or lowercase `0b` binary. A leading `0` means octal, so `08` fails and
  `010` reads as 8. A sign forces decimal, so `-0x10` fails. A `byte` must be 0–255, a `long` must fit in 32 bits.

The check warns but does not block. The value is still saved, and a push asks once more before it sends an invalid
value.

If a file sets a key more than once, the editor shows one row with the last value, the one the firmware uses. An edit
rewrites every occurrence.

### Unset and empty values

- Clearing a field, or ×, deletes the key on push. `0` is a real value.
- An existing empty value (`key=`) is kept if untouched, but an empty value can't be written.
- In a downloaded `.zip`, a hand-emptied existing key stays as `key=`. Use × to drop the line.

### Dependent keys

Some keys only matter for a certain value of another key. The editor follows these rules from the firmware:

| Keys | Rule |
|------|------|
| `pwm_driver_logic` (board) | Required, `HiLi` or `InEn`. The firmware has no default and stops on another value |
| `pwm_in`, `pwm_en` (board) | Read with `pwm_driver_logic=InEn`, both required |
| `pwm_hi`, `pwm_li` (board) | Read with `HiLi`, both required |
| `pwm_sd` (board) | Read with `HiLi`, optional |
| `psu_vout` (converter) | Read with `mode=psu`, required |
| `pv_isc`, `pv_voc` (converter) | Read with `mode=pv`, required |
| `pv_k`, `pv_slew`, `pv_iout_span` (converter) | Read with `mode=pv` |
| `psu_vout`, `pv_*` (converter) | Not read while tracker.conf `target_duty_cycle` is between 0 and 1, which overrides `mode` |
| `hs_clamp` (board) | Needs `bflow_bat` on (any non-zero value) |
| `bflow_bat` (board) | Needs `bflow_en` or `bflow_sd` set to a pin (non-zero) |
| `bflow_bat`, `hs_clamp` (board) | Need converter.conf `topo=buck` |

A key that is not read under the current value is hidden while it is unset, also under **Show all**. If it has a value
or a pending change, it shows faded with "not used with …", so you can remove it. A required key that is empty
always shows, with an amber "required for …" note, and its group opens. A missing condition shows as "needs …". The
notes warn but do not block. An unset `mode` counts as `mppt`, its firmware default; an unset or unknown
`pwm_driver_logic` leaves the pins as they are.

Changing a controlling value updates the rows at once and opens the group with the keys it brings in. A rule that
depends on another file applies when you open that file's tab.

In board.conf the `pwm` group starts with a **Gate driver** block: `pwm_driver_logic`, then the pins that logic uses,
each with a plain label (HI, high-side input; LI, low-side input; IN, PWM input; EN, enable; DIS, shutdown). A line
above them names the switching and the rectifying pin for the converter.conf `topo`:

| Logic | Buck: switches / rectifies | Boost: switches / rectifies |
|-------|----------------------------|-----------------------------|
| InEn  | `pwm_in` / `pwm_en`        | `pwm_en` / `pwm_in`         |
| HiLi  | `pwm_hi` / `pwm_li`        | `pwm_li` / `pwm_hi`         |

### Removed role keys

The firmware names sensor channels and the four mapped limits by converter side (`hv_v_*`, `lv_i_*`, `hv_max`, …)
and fails setup on the old `vin_*`/`vout_*`/`iin_*`/`iout_*` keys. A file with such keys still loads and saves
unchanged. The editor shows the firmware's error above the rows, with the replacement key for the configured
`topo`, and marks each old key in amber. Migrate the file with `etc/migrate_side_keys.py`.

## Review and push

**Review changes** opens a sheet with every change, grouped by file:

- Each change reads old → new, or → *removed* for a cleared key. An invalid new value is underlined and its warning
  repeated.
- **Undo** reverts that change. Clicking a key name jumps to its field.
- With a device to push to, each change says when it takes effect: "now", "after restarting mqtt", "after reboot",
  or "when next used". `ftp_user` and `ftp_pass` add that they are ignored while credentials are stored in NVS. The
  footer sums it up, for example "Takes effect after: restarting mqtt, reboot".
- An amber list above the files warns about dependent keys a change breaks (see [Dependent keys](#dependent-keys)),
  and about switching off a service that carries a console: `mqtt` (a charger that gets its BMS data over MQTT then
  falls back to its stale-cell voltage cap), `ble`, `telnet` or `tlscon`.
- **Commands sent to the device** unfolds the exact command list.

The sheet footer holds the save actions:

| Action                         | Result                                                                                   |
|--------------------------------|------------------------------------------------------------------------------------------|
| **Download .zip**              | `<source>-edited.zip` if anything changed, else `<source>-backup-<YYYY-MM-DD>.zip`. Non-`.conf` files from the source are carried over unchanged |
| **Push N changes to `<hostname>` over …** | `set-config <file> <key> <value>` per added or edited key, `del-config <file> <key>` per cleared key. A service's `enabled` and `log_level` go as `svc on`/`svc off <name>` and `svc log <name> <level>` instead, unless the firmware does not have that service: then they go as `set-config` and take effect after a reboot. Only changed fields are sent |

The push button appears only when the loaded config was read from a device, and names that device. It stays
available after a file or preset is overlaid onto that read. A config loaded from a file or preset alone has no
push target; read the device first and then open the file over it. If a connect to another device fails, the
loaded config and its push target stay with the device it was read from. The push reopens a closed connection
without asking for the port or device again. With no changes, the sheet offers **Download backup .zip** instead.

A push starts from this button without a second confirmation, unless a value is invalid: then a dialog lists the
invalid values and asks again. The console opens and shows each command and its reply. A field the device
confirmed with `OK:` loses its change mark, and fields that failed stay marked. A service switched on by the push
that fails to start counts as failed, with the firmware's reason.

The editor sends one command sequence at a time: while a push, an apply step or a Services action runs, the push
and apply buttons, the Services controls and the console input are disabled.

### Applying a push

The firmware writes a pushed key to its file but does not apply it. When it takes effect depends on the file:

| Change | Takes effect |
|--------|--------------|
| `enabled`, `log_level` of a service (mqtt, tele, ftp, http, telnet, tlscon, lcd, scope, ble, bsync) | Now: sent as `svc on`/`svc off`/`svc log`, which also save the file |
| mqtt.conf | After `svc restart mqtt` |
| tele.conf | After `svc restart tele`; `adv_ms` after `svc restart ble`; `ble` the next time the stream is started; `idle_interval_ms` after a reboot |
| ftp.conf | After `svc restart ftp`; `ftp_user` and `ftp_pass` also after `svc restart http`, which uses the same login |
| http.conf, tlscon.conf, bsync.conf | After restarting that service |
| Everything else, and a cleared `enabled` or `log_level` | After a reboot |

Switching off the service the editor is connected through (mqtt over MQTT, ble over Bluetooth) is sent as
`set-config` and takes effect after a reboot, so the push keeps its connection.

After a push in which every command succeeded, the console shows what is still needed, with a button for each step:
**Restart mqtt** and so on, or only **Reboot device** when a reboot is due, since it covers the restarts. A failed
push shows no buttons. The buttons only act on the device that was pushed to; reading another device hides them,
while reading the same device again keeps them.

A restart starts a service even when it is switched off. So a service that is off, after the push or later, gets
no **Restart** button; the console says the change applies when that service is turned on. Before it restarts a
service, the editor reads `svc list` again and stops if the service has been switched off meanwhile.

**Reboot device** asks first: the converter stops converting, reboots, and the editor's connection drops. If the
push switched off the service the editor is connected through, the dialog says instead that this connection does
not come back after the reboot and that you need another connection. Restarting the service the editor is
connected through also asks first, and drops the connection.

A step counts as done only when the device confirms it, and only then does its button go:

| Step | Confirmed by |
|------|--------------|
| Reboot device | The device's `Rebooting` line; the editor then closes the connection |
| Restart of another service | `OK:` without a failed-start message |
| Restart of the service the editor is connected through | `OK:`, or the link going down (presence message over MQTT, dropped Bluetooth link), and the service listed as Running afterwards |

Without that confirmation the status reads "sent, not confirmed", the button stays, and you can check in
**Services** or read the device again. A failed start shows the firmware's reason.

The sheet is a modal dialog: Tab stays inside it, Esc closes it, and focus returns to **Review changes**. Esc also
closes the **Open** menu and the console.

:::warning
Pushing edits `board.conf`, `sensor.conf` and `limits.conf` on a running converter. Wrong pin, divider or limit
values can damage hardware. Check the change list in the review sheet before you push.
:::

## Status and console

Progress and errors appear in the top bar next to the connection state, or under the choices on the start screen. A
spinner runs while the editor connects and reads the config. If some files did not answer in time, the status line
lists them, and they appear under "Not on device".

**Console** opens a side panel below the top bar (a bottom sheet on a narrow screen) with the live device output and a command field that writes to the active connection.
It renders the firmware's ANSI log colours, shows `ERR:` replies in amber, and follows new output unless you have
scrolled up. See [Serial Console](console.md).

## Services

**Services** opens a panel with the firmware's services (see `svc` in [Serial Console](console.md)), read with
`svc list`. It is available only while the editor is connected to the device the config was read from. Each row
shows the service name, its state (green dot for Running, grey for Stopped, amber for Failed) and its status
detail, with three controls:

| Control   | Command                        | Effect                                                         |
|-----------|--------------------------------|----------------------------------------------------------------|
| Log level | `svc log <name> <level>`       | Saved in the service's `.conf` file and applied at once        |
| Restart   | `svc restart <name>`           | Stops and starts the service; off while the service is switched off, because a restart would start it |
| On/off    | `svc on <name>` / `svc off <name>` | Saved in the service's `.conf` file and started or stopped at once |

The list is read again after each action and with the refresh button. A rejected command shows the firmware's
reason, and a service that fails to start after **On** or **Restart** shows why. A command without a reply reads
"sent, not confirmed". The loaded `<name>.conf` follows a saved on/off or log level, unless that field has an unsent
edit. A restart here also removes the matching **Restart** button from the console.

Stopping or restarting the service the editor is connected through (`mqtt` over MQTT, `ble` over Bluetooth)
drops that link, so the editor asks first. The device cannot send its `OK:` over a link it just stopped. A stop
therefore counts as confirmed only when the link visibly goes down: an `offline` presence message over MQTT, or the
device dropping the Bluetooth link. Otherwise the panel says the command was sent but not confirmed. After a stop
the top bar shows the device as disconnected either way; turning the service back on needs another connection.
After a restart the editor waits for the device to answer again and checks that the service is Running.
Stopping `mqtt` also asks first, because a charger that gets its BMS data over MQTT loses it and falls back to
its stale-cell voltage cap.

## conf-tool.py (FTP)

[`etc/config-tool/conf-tool.py`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/etc/config-tool/conf-tool.py)
is a command-line companion to the editor. It needs the `ftp` service on the device. The tool performs these steps:

1. Discovers devices on the LAN.
2. Downloads `/littlefs/conf` over FTP.
3. Diffs it key-by-key against a local config folder and lets you pick which values to take.
4. Uploads the merged files after confirmation.

A typical invocation looks like this:

```bash
etc/config-tool/conf-tool.py --hosts '<hostname-regex>' --local-conf config/fmetal \
    --user <user> --password <password>
```

The tool accepts these flags:

| Flag             | Default       | Meaning                                                   |
|------------------|---------------|-----------------------------------------------------------|
| `--hosts`        | `.+`          | Regex matched against discovered host name or IP          |
| `--local-conf`   |               | Local config folder to diff against; omit to only download and view |
| `--dl-dir`       | `etc/config-tool/dl` | Where downloaded configs are stored                       |
| `--remote-conf`  | `conf`        | Remote subdirectory                                       |
| `--user`, `--password` | `user`, `password` | FTP login. The defaults do not match a device without FTP credentials, which uses the chip ID as password; see [`ftp.conf`](config/ftp.md) |
| `--timeout`      | `4.0`         | FTP timeout, s                                            |
| `--view`         |               | Also print the downloaded files                           |
| `--no-upload`    |               | Diff and merge, never upload                              |

## Maintaining the metadata

Key lists, types, defaults and descriptions live in tables inside the HTML (`FILE_KEYS`, `TYPE_KEYS`, `DEFAULTS`,
`META`, `FILE_META`). When a firmware conf key is added, renamed, or removed, update these tables and the matching page
under [Configuration files](config/index.md) together.

`etc/config-tool/scrape_conf_keys.py` reports drift between `ConfFile::get*()` calls in `src/` and the editor
tables:

```bash
python3 etc/config-tool/scrape_conf_keys.py            # drift report
python3 etc/config-tool/scrape_conf_keys.py --write    # rewrite FILE_KEYS, TYPE_KEYS, DEFAULTS
python3 etc/config-tool/scrape_conf_keys.py --check    # exit 1 on drift, 2 on a getter it cannot resolve (CI)
```

It resolves every getter to its `.conf` file from the code and fails on any it cannot, so a key is never silently
missing from the tables. `--check` also fails on a `META` description for a key no getter reads.

[`etc/config-tool/spec.md`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/etc/config-tool/spec.md) specifies
the full behaviour.
