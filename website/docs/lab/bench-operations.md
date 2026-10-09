---
title: Bench operations
sidebar_position: 3
---

# Bench operations

This page collects operational detail for working Fugu boards on a bench from a macOS or Linux
host: toolchain quirks, board identification, flashing, port recovery, console mechanics, live
config edits, BLE failure modes, and OTA. Related pages are [Console commands](../reference/console.md),
[Agentic programming](../development/agentic-programming.md),
[OTA over BLE](../guide/updating/ota-ble.md), and [OTA over Wi-Fi](../guide/updating/ota-wifi.md).

Everything here was observed in bench sessions unless marked *inferred*.

## Toolchain and environment

Source ESP-IDF (`. $IDF_PATH/export.sh`, 5.5+) and run `idf.py set-target esp32s3` once per build
dir. Afterwards, invoke repo tools with a Python that has their dependencies ([Host tools](../reference/host-tools.md)).
`etc/fugu_console.py` has no exec bit, so call it through an interpreter:
`python3 etc/fugu_console.py`.

Pass the port explicitly (`-p`). With several boards attached, a `$ESPPORT` guessed from a
`/dev/cu.usbmodem*` glob picks the wrong one. One session flashed one board's build onto another
board's port this way.

`esptool` on PATH can resolve to a broken PlatformIO shim (`ModuleNotFoundError: esptool`).
`python -m esptool …` after sourcing IDF always works. On S3, `chip_id` prints the efuse MAC
("ESP32-S3 has no Chip ID. Reading MAC instead"), not an id.

The ESP-IDF tree used for the BLE builds carries a local NimBLE patch
([`etc/patches/nimble-att-tx-no-panic.patch`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/etc/patches/nimble-att-tx-no-panic.patch)). The NimBLE tree is
a git submodule *inside* IDF, so an IDF update silently reverts the patch and devices resume
panicking on mbuf exhaustion. Reapply it after every IDF bump. When you reapply it, verify that the
`bt` component's object actually rebuilt, because the build banner timestamp does not move.

For build variants, quote multiple defaults fragments:
`-DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.vconv_s3"`. Unquoted, the shell silently eats
the second fragment. `idf.py set-config` does not exist in IDF 5.5, and `SDKCONFIG` as an
environment variable has no effect, so pass `-DSDKCONFIG=` on the command line. Side IDF
projects under `etc/*` (e.g. `etc/bsync-beacon`) keep their own gitignored `sdkconfig`, which goes
stale. Delete it with `rm` and let it regenerate.

If a flash or build dies with a truncated tail, the real error is in
`build-<tag>/log/idf_py_std{err,out}_output_<pid>` (the pid is in the filename).

After you change a firmware global's type, both `idf.py -B build build` and
`RUN_TESTS=1 idf.py -B build-tests build` must be green. Only the second compiles the test TUs,
and an `extern` type mismatch links silently (observed: `time_us` vs `unsigned long`).

## Identifying the board on a port

Re-verify a board's identity after every replug and before any board-specific write (role configs,
`set-config`), not only before flashing. The port↔board mapping is not stable with several boards
attached. The following failures were observed:

- Two `usbmodem*` ports swapped board assignments mid-session, and role configs landed on the
  wrong boards.
- A port that used to be a Fugu board became a different, non-Fugu S3 after a replug (wrong-device
  flash, see below).
- Consecutive `flash.sh <name> -p …` runs went to a different board than intended while a test hit
  the unflashed one.

The following checks are cheap. Try them in this order:

1. `system_profiler SPUSBDataType`: each S3's "USB JTAG/serial debug unit" reports its efuse MAC
   as the USB Serial Number, without touching the board. Keep a list of your boards' MACs to map
   serial number → board.
2. Console `hostname`, the boot banner, or BLE `advertising as 'fugu-<name>'` in the log. A
   MAC-derived `fugu-esp32s3-<digits>` answer means *unnamed board*, not *wrong board*.
3. `python -m esptool -p PORT chip_id`. It resets the board into the bootloader, so it is the last
   resort on a running converter.

Check identity before you blame cables. Esptool errors that read as link problems (`Invalid head of packet`,
`Corrupt data, expected 0x1000 bytes but received …`, an apparent ROM boot-loop) appeared when the
device on the port was not the expected board at all. On a genuine link, dropping to `-b 115200`
fixed mid-transfer corrupt data.

esptool writes an image without validating it against the target's partition table. A 1.7 MB fugu
app was written onto a foreign S3 with a 1448 KB app partition, and esptool reported
`Hash of data verified. Done` while silently overrunning ~216 KB into that device's spiffs. To check
an unknown device's partition table offline, run
`python -m esptool -p PORT read_flash 0x8000 0xc00 pt.bin` and then
`python $IDF_PATH/components/partition_table/gen_esp32part.py pt.bin`.

## Flashing

Flash through the wrapper so the ELF archive is keyed by board name:
`./flash.sh <name> -B build-<tag> -p PORT app-flash` (equivalently
`FUGU_DEVICE=<name> idf.py … app-flash`). Without `FUGU_DEVICE`, the archive falls back to the
serial-port basename, which is unstable, so a later `elf_archive.py decode --device <name>` misses.

Always append `app-flash`. With *no* idf.py args, `./flash.sh <name>` runs `idf.py build flash`,
a full flash including the littlefs image that the app-flash rule exists to prevent.

## Reset and port recovery

The following items cover resets, stuck boot modes, and unresponsive ports:

- Never hand-roll a pyserial DTR/RTS reset. One held IO0 low and strapped the board into ROM
  download mode (`rst:0x15 … boot:0x1 (DOWNLOAD)`), leaving a mute console indistinguishable from a
  firmware hang. Several `set-config` writes were silently lost to it. The repo tool is
  `etc/idf-devtools/rts.py`, but its own docstring says it does not work on the S3's native
  USB-JTAG port.
- A board can also get stuck *in* download mode, the inverse of the can't-enter case. A board
  replugged with BOOT held latches the strap, and esptool's soft resets re-sample it, so esptool
  works normally while the app never runs. Recover with a plain replug without touching BOOT.
  Probe such a device non-disruptively with
  `python -m esptool -p PORT --before no_reset --after no_reset chip_id`, and kick it into the app
  with `--before no_reset run`.
- USB-CDC occasionally fails to re-enumerate after a console `restart` (observed once). An RTS
  reset or replug brings it back.
- A board with a dead USB serial port can still be alive. One board gave nothing on serial through
  every reset trick and answered instantly over BLE. Switch transport and keep diagnosing. Only a
  power cycle restores serial in that state.
- zsh aborts the whole command on any unmatched glob (`no matches found`), so
  `ls /dev/cu.usbmodem* /dev/cu.usbserial*` loses both listings and `--include=*.cpp` dies too.
  `2>/dev/null` does not help, because the shell fails before exec. Quote patterns or split them
  per glob.

## Console access mechanics

The canonical invocation follows. See [Agentic programming](../development/agentic-programming.md)
for the tool's design.

```bash
timeout 30 python3 etc/fugu_console.py -p PORT -c "status" -c "svc" | grep -a -v '^V='
```

The client behaves as follows:

- With `-c`/`--stdin`, the client exits after the last reply. Use `timeout N` only as an overall
  deadline against a stalled connect or scan. Pipe through `grep -a`, because the stream carries
  non-UTF-8 bytes and plain grep prints "binary file matches" and nothing else. Use
  `tr -d '\0'` where NULs break downstream tools.
- `--ip host:port` is one argument. Behind a NAT or port forward, confirm the device with
  `hostname`, since forwarded mappings can move. MQTT takes `--mqtt host --mqtt-port N --name <dev>`
  (`--mqtt host:port` does not parse). `--ble --name <hostname>` takes the bare hostname.
- Batched replies interleave with log output and can fabricate errors. A `-c "ip"` in a batch
  produced `Command not found at 'ip'` *and* the correct reply. Character-interleaved garbage was
  also observed. Re-send a command alone before you trust an `ERR:`. Under heavy log flood, send
  it twice or quiet the board first (`log <tag> error`, see [Console](../reference/console.md)).
- Reads right after a state-changing command return the pre-change state, because the `-c`
  commands go back-to-back with no settle. Sleep 3–10 s between the change and the read, ~12 s
  after `restart`, and 10–15 s after `wifi on` before `ip` means anything.
- Async events only reach a connection that is open when they fire. A protection-trip reason,
  backoff, or ADC-init error printed between two client invocations is lost. To capture one, put
  the log-level raise, the trigger, a device-side `sleep 3`, and the read in a *single* `--stdin`
  script.
- The client's per-command timeout is 4 s by default, with overrides for `ota`, `scan-i2c`, `curl`,
  and `ping`. A slow reply looks like a dead board.
- The shared dispatcher prints the log line `received serial command: '<cmd>'` for all
  transports (serial, telnet, MQTT, BLE), so the line never identifies the transport.
- If basic verbs (`help`, `status`, `svc`, `hostname`) answer `unknown or unexpected command`, the
  board runs firmware older than the current console. Flash before diagnosing further.
- Only one reader can use a serial port, and contention does not error cleanly. The losing process
  gets "device reports readiness to read but returned no data", and background loggers die
  silently. Run `lsof /dev/cu.usbmodemXXX` before opening the port. It also catches your own stale
  monitor.
- To smoke-test after a firmware change, run
  `python3 etc/e2e-test/test_console_plan.py --serial PORT --mock` (PASS/FAIL per verb).

:::danger `dc` takes over from any mode
`dc N` switches to manual PWM from any mode and drives duty N (`dc 0` stops conversion).
`sync on|off|forced` and `bf 0|1` are rejected with `ERR` outside manual mode. A bare `sync` reports
the rectifier state in any mode. Verify effects in the status line anyway, since batched replies can
interleave.
:::

## Editing config on a live board

Config edits on a live board have these pitfalls:

- Read back every `set-config` with `get-config` in the same invocation. A stray `~` framing
  corruption twice mangled a command into `~set-config …`, and the write was silently lost.
- `set-config k ""` stores the two quote characters literally. Clear a key with
  `del-config <file> <key>` instead. `get-config` cannot distinguish a missing key from an empty
  one, so probe a known-value key to test config presence.
- The deployed `/littlefs/conf/` drifts from `config/` in the repo. Provisioning is the only
  thing that syncs them, and `get-config` is ground truth for what a board actually uses.
- WiFi keys (`ssid_<tag>` / `ssid_<tag>_psk`) are only loaded at boot (`wifi_load_conf`). After
  `wifi-add` or `set-config wifi.conf …`, run `restart` before the board will associate.
- A checksum boot-loop (`Checksum failed. Calculated 0x.. read 0x..`) straight after a
  `provision.py` run was recovered by one full `idf.py flash`. The board was not dead.
- `python3 etc/dump_littlefs.py <outdir>` reads a live config partition back. It writes the raw
  image and the extracted `conf/*.conf`, which round-trip into `provision.py`. It goes through
  `parttool.py`, so it reboots the board. Never run it on a live converter. To read one value, use
  `get-config`.

## BLE failure modes

The following table maps each symptom to its cause and action:

| Symptom | Cause / action |
|---|---|
| Absent from scans entirely | Someone holds the single connection (`NIMBLE_MAX_CONNECTIONS=1`): another console session, or a BLE telemetry bridge in connect mode. The link lives in bluetoothd, so killing the client does not free it; run `bluetoothctl devices Connected` / `disconnect <mac>` on the holder; device-side `svc rs ble` restores the connectable advertisement. Also: a WiFi scan-loop against a missing AP starved BLE until `wifi off`. Also: a board powered from Vin drops off BLE when the bench supply drops; check the boot log's vin calibration value. |
| Visible to a raw scan but not to the tooling | With BLE_ADV telemetry the adv payload has no room for the NUS UUID, so UUID-filtered scanners show nothing. Match by **name**. |
| Scans fine, every connect times out | Not the GATT-cache case: someone else holds the link (see row 1), and with `WITH_BLE_ADV` the board stays visible-but-non-connectable while connected. |
| ATT 3 "Writing is not permitted" on subscribe (bonded Mac) | Stale macOS GATT cache after a layout change (the cache is keyed by BLE address); toggle Bluetooth (`blueutil -p 0 && blueutil -p 1`, see [OTA over BLE](../guide/updating/ota-ble.md)). `blueutil` is not installed by default on macOS; unpairing otherwise needs the GUI. |
| ATT 5/15 "Insufficient Authentication/Encryption" | Unbonded host vs `ble_security=justworks`; pair/bond the host first (see [`ble.conf`](../reference/config/ble.md)). |
| ATT 17 "Insufficient Resources", intermittent | NimBLE mbuf starvation from rapid connect/disconnect cycles; clears on device `restart`. Space out reconnects. |

Only a full `restart` applies `ble.conf` changes (`ble_security`, name). Neither `svc rs ble` nor
`svc off/on ble` applies them, because the props are set once at stack init (`console_ble.cpp`:
`bleInited` early-return). `bleConsoleEnd` deliberately never deinits, because the Arduino BLE
wrapper is not reinit-safe.

To see boot-time (`setup()`) log lines over BLE, reconnect within ~3 s of `restart`. Later
connects get no backlog of those lines, which has been misread as "the code isn't running".

## OTA

`etc/ota.py` (Wi-Fi; flags in [OTA over Wi-Fi](../guide/updating/ota-wifi.md)) refuses dirty
images, `WITH_NETW=n` builds, and `WITH_VCONV=y` plant-sim builds when it runs non-interactively.
Scripted shells are not a tty, so commit (or `git stash push -- <unrelated paths>`) before a Wi-Fi
OTA.

Over BLE ([OTA over BLE](../guide/updating/ota-ble.md)), run `python3 etc/ota_ble.py -n <name> -y`.
These points apply to BLE pushes:

- The version string is `git describe`-derived. Rebuilding an uncommitted tree doesn't change it,
  so the tool takes its skip path (`☑️ skip: already at <ver>`) and exits looking successful.
  Pass `-f` whenever the tree changed without a commit, and never grep-filter OTA output down to a
  success/fail whitelist that omits `skip`. A real push emits hundreds of progress lines.
- Run it detached or in the background. A full 1.7 MB image takes 2–3+ min, and a 300 s foreground
  timeout killed one mid-transfer.
- Run one BLE OTA at a time. Two concurrent pushes contend for the host's single radio (both
  stalled at ~11 %).
- A failed or killed push leaves the device armed. Clear it with console `ota-ble abort`, then
  retry. The registered verb is `ota-ble`, although the doc and the firmware's error text say
  `otab`.
- Right after an OTA or `restart`, the board is not advertising for ~10–15 s. A scan miss or a
  READY timeout in that window is retryable, not a failure.
- As a last-resort remote path with no working client, publish a console line (e.g. `ota <url>`)
  to `pv/log/<name>/cmd` on your MQTT broker while serving the image with
  `python3 -m http.server 9000` from the repo root. Prove the path with `uptime` first.

## Verifying what a board is running

`bootinfo`'s `built <timestamp>` is the only reliable way to tell two bench builds apart. Every
bench build is `-dirty`, so two sessions' builds carry near-identical `git describe` strings.

The version string identifies a commit, not a binary. A fix that exists uncommitted in the tree is
*not* on the device, and someone else may have reflashed the board from a different tree. Before
you diagnose a crash on a shared board, follow these steps:

1. Read `bootinfo`.
2. Map version → commit.
3. Compare against what you are reading with `git diff HEAD` / `git show <sha>:<file>`.

Before moving a USB cable off a board, confirm it keeps a remote path: `svc` shows `ble` running,
or `ip` is non-zero. A board left with Wi-Fi off and BLE stopped is unreachable until it is
plugged back in.
