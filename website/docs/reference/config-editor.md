---
title: Config editor
sidebar_position: 2
---

# Config editor

[`etc/config-tool/conf-editor.html`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/etc/config-tool/conf-editor.html)
is a single-page browser editor for the board `.conf` files. It reads them from a device or an archive, lets you edit
them with type, unit, and default hints, and then downloads a `.zip` or writes only the changes back to the device.

The editor is one self-contained HTML file with no build step. The MQTT transport loads `mqtt.js` from a CDN. The
other paths work offline.

## Quick start

Web Serial and Web Bluetooth need a Chromium browser (Chrome, Edge) and a secure context (`https://` or
`localhost`), so serve the file locally:

```bash
cd etc/config-tool
python3 -m http.server 8000
# open http://localhost:8000/conf-editor.html
```

To read, edit, and write back a device config over serial, follow these steps:

1. Click **Connect serial (115200)** and pick the device port. The editor sends `hostname` and one
   `get-config <file>` per known file and opens a tab per file.
2. Edit values. Changed fields get a "was: …" hint (click it to revert), and the tab gets a dirty dot.
3. Click **Upload changes to device**. A confirmation lists every command. The editor then sends each one and
   awaits its `OK:`/`ERR:` reply.
4. Optionally, click **Download .zip** to keep a backup.

`set-config` persists each value to the file on flash. Most files need a reboot before a key is applied. See
[When changes take effect](config/index.md#when-changes-take-effect) for details.

## Sources

The editor can load a config from any of these sources:

| Source          | Button                                 | Notes                                                                 |
|-----------------|----------------------------------------|-----------------------------------------------------------------------|
| `.zip` archive  | **Choose .zip**, drag-and-drop         | Stored and deflate entries                                            |
| Folder          | **Choose folder**, drag-and-drop       | e.g. a `config/<board>/conf` folder                                   |
| USB serial      | **Connect serial (115200)**            | Web Serial; the port stays open between read and upload so the device is not reset |
| Bluetooth (NUS) | **Connect Bluetooth**                  | Web Bluetooth, Nordic UART service; the OS may prompt for pairing     |
| MQTT            | **Connect MQTT**                       | WebSocket broker URL (`ws://` / `wss://`); needs `mqtt.conf` `cmd_input=1` |

Loading a source replaces the previous state, with one exception. After a live device read, opening a `.zip` or
folder overlays it onto the device values. Every value that differs shows up as a pending change, so you can
review what a stored config would alter before uploading it.

A file included in the upload fully replaces that file, and keys it omits are cleared. Files it doesn't include are
left untouched.

### MQTT scan

**Connect MQTT** opens a dialog for the broker URL, username, and password, which are kept in browser
`localStorage`. **Scan for devices** subscribes to `pv/log/#` for 5 s and probes the hosts remembered for that
broker with the `ip` command. It lists every `<hostname>` seen publishing. Pick a device to read its config. See
[MQTT topics](mqtt.md).

## Editing

The editor shows each file as a form with these features:

- One tab per conf file. Hardware and calibration files (`board`, `sensor`, `limits`, `coil`, `converter`) are
  tinted red.
- Files the firmware reads but the source lacks appear as faded tabs. Fill a key to create the file.
- Each row shows the key, a type pill (`byte`/`long`/`float`/`string`, the getter the firmware uses), unit,
  description, and default. A `?` pill marks a key without curated metadata.
- Clearing a field (or ×) deletes the key on upload. An existing empty value (`key=`) is kept if untouched,
  but an empty value can't be written. In a downloaded `.zip`, a hand-emptied existing key stays as `key=`.
  Use × to drop it. `0` is a real value.
- **+ add key** appends an arbitrary key.
- The **raw** section shows the serialized file. Comments, inline `# comments`, and whitespace survive the
  round trip.

## Saving

The editor saves edits in two ways:

| Action                        | Result                                                                                   |
|-------------------------------|------------------------------------------------------------------------------------------|
| **Download .zip**             | `<source>-edited.zip` if anything changed, else `<source>-backup-<YYYY-MM-DD>.zip`. Non-`.conf` files from the source are carried over unchanged |
| **Upload changes to device**  | `set-config <file> <key> <value>` per added or edited key, `del-config <file> <key>` per cleared key. Only changed fields are sent |

:::warning
Uploading edits `board.conf`, `sensor.conf`, and `limits.conf` on a running converter. Wrong pin, divider, or limit
values can damage hardware. Review the command list in the confirmation dialog.
:::

## Device log

Once you're connected, the **Device log** button opens a panel with the live console output and a command input
that writes to the active transport. The panel renders ANSI colours and shows `OK:` in green and `ERR:` in red. See
[Serial Console](console.md).

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

Key lists, types, and defaults live in tables inside the HTML (`FILE_KEYS`, `TYPE_KEYS`, `DEFAULTS`, `META`,
`FILE_META`). When a firmware conf key is added, renamed, or removed, update these tables and the matching page
under [Configuration files](config/index.md) together.

`etc/config-tool/scrape_conf_keys.py` reports drift between `ConfFile::get*()` calls in `src/` and the editor
tables:

```bash
python3 etc/config-tool/scrape_conf_keys.py          # drift report
python3 etc/config-tool/scrape_conf_keys.py --check  # non-zero exit on drift (CI)
```

[`etc/config-tool/spec.md`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/etc/config-tool/spec.md) specifies
the full behaviour.
