---
title: Bench operations
sidebar_position: 3
---

*this document is an LLM generated placeholder*

# Bench Operations

Operational detail for working Fugu boards on a bench from a macOS/Linux host: toolchain quirks,
board identification, flashing, port recovery, console mechanics, live config edits, BLE failure
modes and OTA. Related: [Console commands](../reference/console.md),
[Agentic programming](../development/agentic-programming.md),
[OTA over BLE](../guide/updating/ota-ble.md), [OTA over Wi-Fi](../guide/updating/ota-wifi.md).

Everything here was observed in bench sessions unless marked *inferred*.

## Toolchain & environment

`. ./idf-export.sh` (repo root only — it sources the ESP-IDF `export.sh` by a relative path; adjust
it to where your IDF lives) sets `IDF_TARGET=esp32s3` and enters IDF 5.5.1. Two side effects:

- It runs `deactivate`, dropping the repo venv. All repo Python tools must be invoked as
  `.venv/bin/python3 etc/…` afterwards. `etc/fugu_console.py` has no exec bit, so
  `python3 etc/fugu_console.py` fails with `Permission denied` regardless.
- It exports `ESPPORT` from the **first** `/dev/cu.usbmodem*` glob match — a coin flip with more
  than one board attached (one session flashed one board's build onto another board's port this
  way). Always pass `-p` explicitly.

`esptool` on PATH can resolve to a broken PlatformIO shim (`ModuleNotFoundError: esptool`);
`python -m esptool …` after sourcing IDF always works. On S3, `chip_id` prints the efuse MAC
("ESP32-S3 has no Chip ID. Reading MAC instead"), not an id.

The ESP-IDF tree used for the BLE builds carries a local NimBLE patch
([`etc/patches/nimble-att-tx-no-panic.patch`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/etc/patches/nimble-att-tx-no-panic.patch)). The NimBLE tree is
a git submodule *inside* IDF, so **an IDF update silently reverts the patch** and devices resume
panicking on mbuf exhaustion — reapply after every IDF bump. When re-applying, verify the `bt`
component's object actually rebuilt; the build banner timestamp does not move.

Build variants: multiple defaults fragments MUST be quoted —
`-DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.vconv_s3"`; unquoted, the shell eats the
second fragment silently. `idf.py set-config` does not exist in IDF 5.5, and `SDKCONFIG` as an
environment variable has no effect — it must be `-DSDKCONFIG=` on the command line. Side IDF
projects under `etc/*` (e.g. `etc/bsync-beacon`) keep their own gitignored `sdkconfig` which goes
stale — `rm` it and let it regenerate.

If a flash/build dies with a truncated tail, the real error is in
`build-<tag>/log/idf_py_std{err,out}_output_<pid>` (pid is in the filename).

After changing a firmware global's type, both `idf.py -B build build` **and**
`RUN_TESTS=1 idf.py -B build-tests build` must be green — the test TUs are only compiled by the
second, and an `extern` type mismatch links silently (observed: `time_us` vs `unsigned long`).

## Identifying the board on a port

The port↔board mapping is not stable with several boards attached. Observed failures: two
`usbmodem*` ports swapped board assignments mid-session (role configs landed on the wrong boards);
a port that used to be a Fugu board became a different, non-Fugu S3 after a replug (wrong-device
flash, see below); consecutive `flash.sh <name> -p …` runs went to a different board than intended
while a test hit the unflashed one. **Re-verify identity after every replug and before any
board-specific write (role configs, `set-config`), not only before flashing.**

Cheap checks, in escalation order:

1. `system_profiler SPUSBDataType` — each S3's "USB JTAG/serial debug unit" reports its **efuse MAC
   as the USB Serial Number**, without touching the board. Keep a list of your boards' MACs to map
   serial number → board.
2. Console `hostname` / the boot banner / BLE `advertising as 'fugu-<name>'` in the log. A
   MAC-derived `fugu-esp32s3-<digits>` answer means *unnamed board*, not *wrong board*.
3. `python -m esptool -p PORT chip_id` (resets the board into the bootloader — last resort on a
   running converter).

Wrong-device signature: esptool errors that read as link problems — `Invalid head of packet`,
`Corrupt data, expected 0x1000 bytes but received …`, an apparent ROM boot-loop — appeared when the
device on the port was not the expected board at all. Check identity before blaming cables; also,
dropping to `-b 115200` fixed mid-transfer corrupt-data on a genuine link.

esptool does **not** validate the image against the target's partition table: a 1.7 MB fugu app was
written onto a foreign S3 with a 1448 KB app partition and reported `Hash of data verified. Done`,
silently overrunning ~216 KB into that device's spiffs. Offline partition check for an unknown
device: `python -m esptool -p PORT read_flash 0x8000 0xc00 pt.bin` then
`python $IDF_PATH/components/partition_table/gen_esp32part.py pt.bin`.

## Flashing

Flash through the wrapper so the ELF archive is keyed by board name:
`./flash.sh <name> -B build-<tag> -p PORT app-flash` (equivalently
`FUGU_DEVICE=<name> idf.py … app-flash`). Without `FUGU_DEVICE` the archive falls back to the
serial-port basename — unstable, so a later `elf_archive.py decode --device <name>` misses.
**Hazard: `./flash.sh <name>` with *no* idf.py args runs `idf.py build flash`** — the full flash
including the littlefs image that the app-flash rule exists to prevent. Always append `app-flash`.

## Reset & port recovery

- **Never hand-roll a pyserial DTR/RTS reset.** One held IO0 low and strapped the board into ROM
  download mode (`rst:0x15 … boot:0x1 (DOWNLOAD)`) — a mute console indistinguishable from a
  firmware hang; several `set-config` writes were silently lost to it. The repo tool is
  `etc/idf-devtools/rts.py`, but per its own docstring it does not work on the S3's native
  USB-JTAG port.
- **Stuck IN download mode** (inverse of the can't-enter case): a board replugged with BOOT held
  latches the strap and esptool's soft resets re-sample it — esptool works perfectly while the app
  never runs. Recover with a plain replug **without** touching BOOT. Probe such a device
  non-disruptively with `python -m esptool -p PORT --before no_reset --after no_reset chip_id`;
  kick it into the app with `--before no_reset run`.
- USB-CDC occasionally fails to re-enumerate after a console `restart` (observed once); an RTS
  reset or replug brings it back.
- **A dead USB serial port does not mean a dead board.** One board gave nothing on serial through
  every reset trick, and answered instantly over BLE — switch transport and keep diagnosing;
  only a power cycle restores serial in that state.
- zsh aborts the **whole** command on any unmatched glob (`no matches found`), so
  `ls /dev/cu.usbmodem* /dev/cu.usbserial*` loses both listings and `--include=*.cpp` dies too;
  `2>/dev/null` does not help (the shell fails before exec). Quote patterns or split per-glob.

## Console access mechanics

Canonical invocation (see [Agentic programming](../development/agentic-programming.md) for the
tool's design):

```bash
timeout 30 .venv/bin/python3 etc/fugu_console.py -p PORT -c "status" -c "svc" | grep -a -v '^V='
```

- Wrap in `timeout N` — the client does not exit on its own. Pipe through `grep -a` (the stream
  carries non-UTF-8 bytes; plain grep prints "binary file matches" and nothing else) and
  `tr -d '\0'` where NULs break downstream tools.
- Transport flags: `--ip host:port` is **one** argument (behind a NAT/port forward, confirm the
  device with `hostname` — forwarded mappings can move); `--mqtt host --mqtt-port N --name <dev>`
  (`--mqtt host:port` does not parse); `--ble --name <hostname>` takes the bare hostname.
- **Batched replies interleave with log output** and can fabricate errors: a `-c "ip"` in a batch
  produced `Command not found at 'ip'` *and* the correct reply. Character-interleaved garbage was
  also observed. Re-send a command **alone** before trusting an `ERR:`; under heavy log flood send
  it twice, or quiet the board first (`log <tag> error`, see [Console](../reference/console.md)).
- **Reads right after a state-changing command return the pre-change state** — the `-c` commands go
  back-to-back with no settle. Sleep 3–10 s between the change and the read; ~12 s after
  `restart`; 10–15 s after `wifi on` before `ip` means anything.
- **Async events only reach a connection that is open when they fire.** A protection-trip reason,
  backoff or ADC-init error printed between two client invocations is lost. To capture one: put
  log-level raise + trigger + device-side `sleep 3` + read in a *single* `--stdin` script.
- The client's per-command timeout is 4 s by default (overrides for `ota`, `scan-i2c`, `curl`,
  `ping`); a slow reply looks like a dead board.
- Commands can be silently mode-dependent: `bf`/`dc`/`sync` are no-ops outside manual mode, with no
  `ERR`. Verify the effect in the status line; absence of an error proves nothing.
- The log line `received serial command: '<cmd>'` is printed by the shared dispatcher for **all**
  transports (serial, telnet, MQTT, BLE) — it never identifies the transport.
- If basic verbs (`help`, `status`, `svc`, `hostname`) answer `unknown or unexpected command`, the
  board runs firmware older than the current console — flash before diagnosing further.
- One reader per serial port: contention does not error cleanly — the losing process gets
  "device reports readiness to read but returned no data" and background loggers die silently.
  `lsof /dev/cu.usbmodemXXX` before opening (also catches your own stale monitor).
- Smoke-test after a firmware change:
  `.venv/bin/python3 etc/e2e-test/test_console_plan.py --serial PORT --mock` (PASS/FAIL per verb).

## Editing config on a live board

- **Read back every `set-config` with `get-config` in the same invocation** — a stray `~` framing
  corruption twice mangled a command into `~set-config …` and the write was silently lost.
- `set-config k ""` stores the two quote characters literally; clear a key with `del-config <file> <key>` instead, and `get-config` cannot distinguish a missing key from an empty one (probe a
  known-value key to test config presence).
- The deployed `/littlefs/conf/` **drifts from `config/` in the repo** — provisioning is the only
  thing that syncs them; `get-config` is ground truth for what a board is actually using.
- WiFi keys (`ssid_<tag>` / `ssid_<tag>_psk`) are only loaded at boot (`wifi_load_conf`), so after
  `wifi-add`/`set-config wifi.conf …` a `restart` is required before the board will associate.
- A checksum boot-loop (`Checksum failed. Calculated 0x.. read 0x..`) straight after a
  `provision.py` run was recovered by one full `idf.py flash`; the board was not dead.
- Read a live config partition back with `.venv/bin/python3 etc/dump_littlefs.py <outdir>`
  (writes the raw image + extracted `conf/*.conf`; round-trips into `provision.py`). It goes
  through `parttool.py`, i.e. **it reboots the board** — never on a live converter; for one value
  use `get-config`.

## BLE failure modes

Symptom → cause table:

| Symptom | Cause / action |
|---|---|
| Absent from scans entirely | Someone holds the single connection (`NIMBLE_MAX_CONNECTIONS=1`): another console session, or a BLE telemetry bridge in connect mode. The link lives in bluetoothd, so killing the client does not free it — `bluetoothctl devices Connected` / `disconnect <mac>` on the holder; device-side `svc rs ble` restores the connectable advertisement. Also: a WiFi scan-loop against a missing AP starved BLE until `wifi off`. Also: a board powered from Vin drops off BLE when the bench supply drops — check the boot log's vin calibration value. |
| Visible to a raw scan but not to the tooling | With BLE_ADV telemetry the adv payload has no room for the NUS UUID — UUID-filtered scanners show nothing. Match by **name**. |
| Scans fine, every connect times out | Not the GATT-cache case: someone else holds the link (see row 1), and with `WITH_BLE_ADV` the board stays visible-but-non-connectable while connected. |
| ATT 3 "Writing is not permitted" on subscribe (bonded Mac) | Stale macOS GATT cache after a layout change — cache is keyed by BLE address; toggle Bluetooth (`blueutil -p 0 && blueutil -p 1`, see [OTA over BLE](../guide/updating/ota-ble.md)). `blueutil` is not installed by default on macOS; unpairing otherwise needs the GUI. |
| ATT 5/15 "Insufficient Authentication/Encryption" | Unbonded host vs `ble_security=justworks` — pair/bond the host first (see [`ble.conf`](../reference/config/ble.md)). |
| ATT 17 "Insufficient Resources", intermittent | NimBLE mbuf starvation from rapid connect/disconnect cycles; clears on device `restart`. Space out reconnects. |

`ble.conf` changes (`ble_security`, name) do **not** apply via `svc rs ble` *or* `svc off/on ble`:
the props are set once at stack init (`console_ble.cpp` — `bleInited` early-return; `bleConsoleEnd`
deliberately never deinits because the Arduino BLE wrapper is not reinit-safe). Only a full
`restart` applies them. To see boot-time (`setup()`) log lines over BLE you must reconnect within
~3 s of `restart` — later connects get no backlog of those lines, which has been misread as "the
code isn't running".

## OTA

`etc/ota.py` (Wi-Fi; flags in [OTA over Wi-Fi](../guide/updating/ota-wifi.md)) refuses
non-interactively: dirty images, `WITH_NETW=n` builds, `WITH_VCONV=y` plant-sim builds. Scripted
shells are not a tty, so commit — or `git stash push -- <unrelated paths>` — before a Wi-Fi OTA.

Over BLE ([OTA over BLE](../guide/updating/ota-ble.md)): `.venv/bin/python3 etc/ota_ble.py -n <name> -y`.

- The version string is `git describe`-derived: rebuilding an uncommitted tree does **not** change
  it, so the tool takes its skip path (`☑️ skip: already at <ver>`) and exits looking successful.
  Pass `-f` whenever the tree changed without a commit, and never grep-filter OTA output down to a
  success/fail whitelist that omits `skip`. A real push emits hundreds of progress lines.
- Run it detached / in the background: a full 1.7 MB image takes 2–3+ min and a 300 s foreground
  timeout killed one mid-transfer.
- One BLE OTA at a time — two concurrent pushes contend for the host's single radio (both stalled
  at ~11 %).
- A failed or killed push leaves the device armed; clear with console `ota-ble abort` (the
  registered verb is `ota-ble`; the doc and the firmware's error text say `otab`), then retry.
- Right after an OTA or `restart` the board is not advertising for ~10–15 s; a scan miss or a
  READY timeout there is retryable, not a failure.
- Last-resort remote path with no working client: publish a console line (e.g. `ota <url>`) to
  `pv/log/<name>/cmd` on your MQTT broker while serving the image with
  `python3 -m http.server 9000` from the repo root; prove the path with `uptime` first.

## Verifying what a board is running

Every bench build is `-dirty`, so two sessions' builds carry near-identical `git describe` strings —
`bootinfo`'s **`built <timestamp>`** is the only reliable discriminator. The version string
identifies a commit, not a binary: a fix that exists uncommitted in the tree is *not* on the
device, and someone else may have reflashed the board from a different tree. Before diagnosing a
crash on a shared board:
read `bootinfo`, map version → commit, and `git diff HEAD` / `git show <sha>:<file>` against what
you are reading.

Before moving a USB cable off a board, confirm it keeps a remote path: `svc` shows `ble` running, or `ip` is
non-zero. A board left with Wi-Fi off and BLE stopped is unreachable until it is plugged back in.
