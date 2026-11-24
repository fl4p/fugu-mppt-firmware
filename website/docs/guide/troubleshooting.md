---
title: Troubleshooting
sidebar_position: 9
---

# Troubleshooting

This page lists common problems and their fixes, starting from what the device logs. The device prints every error
and panic on the console, so read the log first.

## Quick start

To collect the boot, status, sensor, service, and coredump information in one call, run:

```bash
python3 etc/fugu_console.py -p $ESPPORT -c bootinfo -c status -c sensor -c "svc list" -c "coredump info"
```

`bootinfo` shows the last reset reason (`PANIC`, `TASK_WDT`, `BROWNOUT`, …), whether setup failed and why
(`setup: FAILED: <reason>`, see [Setup failure](../reference/console.md#setup-failure)) and the OTA slot state.
`coredump info` shows whether a crash dump is waiting.

## ADC and control loop

:::danger
Treat any ADC error in the log as critical. Without fresh samples the control loop and the protections are blind.
:::

The following table lists the setup, ADC and control-loop log lines, what they mean, and what to check.

| Log line | Meaning | Check |
|---|---|---|
| `setup: FAILED: <reason>` in `bootinfo`/`status`, ` SETUP-ERR` at the end of the status line | setup failed; the converter stays off and the control loop is not started, so sensors read `nan` | the reason names the file or key, see [Setup failure](../reference/console.md#setup-failure) |
| `error during sensor/converter/tracker setup: …` | a sensor, converter, charger or tracker setting is invalid or a sensor does not answer | the message names the key |
| `conf <file>: …` | a config file does not parse (for example a duplicate key or a malformed value) | the named file |
| `board.conf expects MCU …` | config for the other chip | [ESP32 variants](hardware/esp32-variants.md) |
| `Error mounting LittleFS partition!` | the config partition is missing or unreadable | reprovision with `./provision.py <board>` |
| `Never got a sample! Please check ADC` | no sample 20 s after boot; converter disabled, the device restarts after 15 min | ADC backend in `sensor.conf`, I²C pins, `ina22x_alert` / `ads_alert` pin, `scan-i2c` |
| `ADC stall (…, last sample <n> ms ago), resetting` | no fresh sample for 200 ms; the ADC is reset | wiring, I²C noise, the tasks starving the sampler (`rt-stats`) |
| `ADC stall <n> ms, shutdown` | the stall lasted over 0.8 s during conversion; the converter backs off 16 s, the device restarts after 60 s of stall | as above |
| `ina22x: <n> timeout!` | the INA226 alert edge did not arrive in time and the firmware polled the chip instead; logged every 20000 timeouts | `ina22x_alert` pin, `ina22x_conv_time_us` |
| `Loop latency high (…), shutdown!` | Vout sampled at less than half its nominal rate for three 3 s windows in a row (the first two log `Loop latency high (…), window <n>/3`); the converter backs off 4 s | a blocked RT path, ADC alert pin or I²C errors (`rt-stats`, `sensor`) |
| `backoff <n>s [<reason>]` | a protection or watchdog stopped the converter; no restart for at least `<n>` s | the reason in brackets and the log line before it |
| `Calibration failed, <sensor> …` | a sensor was not at rest during zero-current calibration; the converter backs off 4 s | load or input connected at boot, wrong `_midpoint` |
| `Sensor <sensor> zero not settled #<n>: <checks> …` | after a stop, the current-sensor zero did not settle within 1.2 s; the previous zero is kept. `<checks>` names the failed tests (`mean`: the reading still moves, `noise`: noise above 5 % of the zero limit, `ratio`: noise changed between blocks) with their numbers. From the fourth calibration in a row, a zero whose mean is stable is taken without the noise tests (`zero taken w/o noise check`) | the unsettled count in `sensor`; a slow current decay after high-power stops, or a noisy current sensor |

After a setup error, the console and network stay up. Fix the file with `set-config`, then `restart`. A freshly
installed OTA image is not confirmed while setup fails, so with a rollback-capable bootloader the next reset returns
to the previous image.

## Crashes and coredumps

After a panic, the device stores a coredump in flash. To fetch and decode it, run:

```bash
python3 etc/fugu_console.py -p $ESPPORT --coredump get      # writes coredump.bin and decodes it
python3 etc/idf-devtools/elf_archive.py decode coredump.bin  # decode again later
```

`--coredump get` decodes the dump through the ELF archive when it finds the matching build, else it falls back to
`build/fugu-firmware.elf` (`--elf` picks another one).

The dump decodes only against the ELF of the exact build that crashed. Builds flashed with `idf.py flash`,
`app-flash`, `etc/ota.py`, `etc/ota_ble.py` or `etc/ota_mqtt.py` are archived automatically.

Fetch the dump over serial, MQTT, or a telnet console with `trusted=1` (a read-only console allows only
`coredump info`). Over BLE, `coredump get` truncates at about 8 KB. For details, see
[Debugging](../development/debugging/index.md#coredumps).

## Device does not boot (brick recovery)

You can recover a device that hangs before Wi-Fi comes up only over serial. To recover it, follow these steps:

1. Put the chip in download mode: hold **BOOT** (GPIO0), press **RESET**, release BOOT.
2. If you can still reach the device, back up `/littlefs/conf/` first over FTP or with the config editor. Then
   flash the bootloader and application:

   ```bash
   idf.py -p $ESPPORT flash      # also rewrites littlefs from CMakeLists.txt!
   ```

   `idf.py flash` also overwrites the configuration with the littlefs image selected in the top-level
   `CMakeLists.txt`. Point it at your board, or reprovision afterwards with `./provision.py <board>`. To keep the
   configuration, flash only the bootloader and the application: `idf.py -p $ESPPORT bootloader-flash app-flash`.
3. Read the boot log with `python3 etc/fugu_console.py -p $ESPPORT`.

:::warning Rollback needs the right bootloader
If the device has a rollback-enabled bootloader, an OTA image that resets before confirming itself healthy rolls
back to the previous slot. An OTA writes only the app. A device that was never serial-flashed with the current
bootloader bricks instead of reverting. See [OTA updates](updating/ota-wifi.md#rollback-and-boot-watchdog).
:::

On ESP32-S3, the firmware can be built with the optional esp-bootguard. If that bootloader was flashed over serial,
it parks the chip in download mode after repeated crash resets. See
[ESP32 variants](hardware/esp32-variants.md#bootloader-s3-only).

## BLE

The following table lists BLE symptoms and their fixes.

| Symptom | Fix |
|---|---|
| Device not found | The `ble` service is off by default: `svc on ble` over serial or a trusted telnet console. Needs a `CONFIG_FUGU_WITH_BLE` build. |
| Only one client connects | The console accepts one BLE connection; disconnect the other client. |
| macOS: characteristic not found, or `Writing is not permitted` on subscribe after a firmware update | macOS caches the GATT table of a known or paired device. Unpair the device (Bluetooth settings, or `blueutil --unpair <address>`), or toggle Bluetooth off and on, then reconnect. |
| Weak link from the computer | Use an [ESPHome bluetooth_proxy](connecting.md#ble-through-an-esphome-proxy) near the device. |

## Wi-Fi and telnet

The following table lists Wi-Fi and telnet symptoms and their fixes.

| Symptom | Fix |
|---|---|
| Never joins Wi-Fi | `wifi-add <ssid>:<password>`, then `restart`. `wifi off` without minutes persists across reboots; `wifi on` undoes it. |
| No IP shown | `ip` on the serial console. Several networks can be stored in `wifi.conf`. |
| Wi-Fi drops when hot | Above 95 °C chip temperature the device shuts Wi-Fi down; it does not reconnect while above 80 °C. |
| Telnet refuses the connection | The `telnet` service is off by default: `svc on telnet`. |
| Telnet commands refused with `ERR` | Without `telnet.conf` `trusted=1` the telnet console accepts read-only commands only (see [Trusted consoles](../reference/console.md#trusted-consoles)); apply a change with `svc rs telnet`. |
| Telnet connects but commands are ignored | Terminate lines with `\n`. Only one telnet client at a time; close the other session. |
| Telnet/FTP/MQTT not running after Wi-Fi came up late | Services start on the Wi-Fi-up edge; check `svc list` and `svc rs <name>`. |

## FTP

When Wi-Fi is up, the FTP server exposes the littlefs partition. It serves two sessions at a time and refuses a
third connection with `421`. FileZilla opens extra connections for transfers, so limit it to 2 simultaneous
connections (Site Manager, Transfer Settings).

The control port is 21. Passive mode (`PASV`, `EPSV`) uses data port 50009 for the first session and 50010 for the
second (`ftp.conf` `pasv_port` moves the pair); allow ports 21 and 50009-50010 through any firewall or NAT. Active mode (`PORT`, `EPRT`) also works, but
only towards the client's own address. Credentials come from NVS, or from `ftp_user`/`ftp_pass` in
[`ftp.conf`](../reference/config/ftp.md).

An upload goes to a temporary file that replaces the target only when the transfer completes, so an interrupted
upload leaves the old file intact. Sessions close after 10 s without login or 5 min without activity.

To change a single value, prefer `set-config`, which needs no FTP.

## HTTP

Builds with `CONFIG_FUGU_WITH_HTTP_CONF` serve the config files over HTTP as well, for example
`curl -u user:pass http://<device>/conf/`. It uses the FTP credentials and port 80; see
[`http.conf`](../reference/config/http.md) for the endpoints and limits.

## Charging

The following table lists charging symptoms and what to check.

| Symptom | Check |
|---|---|
| Charges only to the float voltage | BMS feed missing or stale; `status` shows `vcell_high` and its age. See [BMS integration](charging/bms-integration.md). |
| Converter idles in daylight | The log line `START blocked: <reason>` names the condition, see [First power-up](getting-started/first-power-up.md#common-scenarios). |
| Unknown or misspelled keys | `conf-check` lists the `charger.conf` and `limits.conf` keys no loader reads. At boot, `charger.conf`, `coil.conf`, `converter.conf`, `tracker.conf` and `limits.conf` also log their unknown keys. |

See also: [Console reference](../reference/console.md), [Connecting](connecting.md).
