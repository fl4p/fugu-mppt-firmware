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

`bootinfo` shows the last reset reason (`PANIC`, `TASK_WDT`, `BROWNOUT`, …) and the OTA slot state. `coredump info`
shows whether a crash dump is waiting.

## ADC and control loop

:::danger
Treat any ADC error in the log as critical. Without fresh samples the control loop and the protections are blind.
:::

The following table lists the ADC and control-loop log lines, what they mean, and what to check.

| Log line | Meaning | Check |
|---|---|---|
| `Never got a sample! Please check ADC` | no sample 20 s after boot; converter disabled | ADC backend in `sensor.conf`, I²C pins, `ina22x_alert` / `ads_alert` pin, `scan-i2c` |
| `ADC stall <n> ms, shutdown` | samples stopped for > 0.8 s during conversion; the ADC is reset, the device restarts after 60 s of stall | wiring, I²C noise, the tasks starving the sampler (`rt-stats`) |
| `ina22x: <n> timeout!` | the INA226 alert did not arrive in time | `ina22x_alert` pin, `ina22x_conv_time_us` |
| `Loop latency high (…), shutdown!` | control loop slower than `sensor.conf::expected_hz` | `expected_hz` too high for the ADC settings, or a blocked RT path |
| `Calibration failed, <sensor> …` | a sensor was not at rest during zero-current calibration | load or input connected at boot, wrong `_midpoint` |
| `error during sensor/converter/tracker setup: …` | a config file is invalid; the control loop is not started | the message names the key |
| `board.conf expects MCU …` | config for the other chip | [ESP32 variants](hardware/esp32-variants.md) |

After a setup error, the console and network stay up. Fix the file with `set-config`, then `restart`.

## Crashes and coredumps

After a panic, the device stores a coredump in flash. To fetch and decode it, run:

```bash
python3 etc/fugu_console.py -p $ESPPORT --coredump get      # writes coredump.bin
python3 etc/idf-devtools/elf_archive.py decode coredump.bin
```

The dump decodes only against the ELF of the exact build that crashed. Builds flashed with `idf.py flash`,
`app-flash`, or `etc/ota.py` are archived automatically.

Fetch the dump over serial, telnet, or MQTT. Over BLE, `coredump get` truncates at about 8 KB. For details, see
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
   `CMakeLists.txt`. Point it at your board, or reprovision afterwards with `./provision.py <board>`.
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
| Device not found | The `ble` service is off by default: `svc on ble` over serial or telnet. Needs a `CONFIG_FUGU_WITH_BLE` build. |
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
| Telnet connects but commands are ignored | Terminate lines with `\n`. Only one telnet client at a time; close the other session. |
| Telnet/FTP/MQTT not running after Wi-Fi came up late | Services start on the Wi-Fi-up edge; check `svc list` and `svc rs <name>`. |

## FTP

When Wi-Fi is up, the FTP server exposes the littlefs partition. It accepts one connection, so set your client, such
as FileZilla, to 1 simultaneous connection.

Passive mode uses data port 50009, and the control port is 21. Allow both through any firewall or NAT. Credentials
come from NVS, or from `ftp_user`/`ftp_pass` in [`ftp.conf`](../reference/config/ftp.md).

To change a single value, prefer `set-config`, which needs no FTP.

## Charging

The following table lists charging symptoms and what to check.

| Symptom | Check |
|---|---|
| Charges only to the float voltage | BMS feed missing or stale; `status` shows `vcell_high` and its age. See [BMS integration](charging/bms-integration.md). |
| Converter idles in daylight | The log line `START blocked: <reason>` names the condition, see [First power-up](getting-started/first-power-up.md#common-scenarios). |
| Unknown or misspelled keys | `conf-check` lists keys no loader reads. |

See also: [Console reference](../reference/console.md), [Connecting](connecting.md).
