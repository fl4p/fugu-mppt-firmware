---
title: "Configuration Files"
sidebar_position: 0
---

# Configuration files

Hardware behavior and runtime options are **not compiled in**. They live as flat `key=value`
`.conf` files on the device's `littlefs` partition under `/littlefs/conf/`, parsed by `ConfFile`
(`src/conf.h`). Source-of-truth images per board live under `config/` (e.g. `config/fmetal`,
`config/lab/wokwi_mock`).

Editing options:

- `set-config <file>.conf <key> <value>` from the serial / telnet / MQTT / BLE console (in place, no
  re-flash)
- `get-config <file>.conf <key>` reads one back. `get-config <file>.conf` reads the whole file
- `del-config <file>.conf <key>` to remove a value
- FTP when Wi-Fi is up (1 connection, no passive mode), or `etc/config-tool/conf-tool.py`.
- The single-page editor `etc/config-tool/conf-editor.html` which can connect via serial, BLE or read from uploads.
  Export as zip file.
- `./provision.py <board>` writes a whole `config/<board>` image to the littlefs partition.

Conventions used in the tables below:

- **default** — the value the firmware uses when the key is absent. `—` means the key is
  **required / board-specific** (no built-in default; usually a divider ratio, pin, or limit that
  must come from the board image).
- A missing **current** sensor (`iin`/`iout`) is replaced by a `VirtualSensor` derived from the
  other side and `power_conversion_eff`; channel `255` means absent.

## Files

| File | Contents |
|---|---|
| [`board.conf`](board.md) | pins, buses, ADC/driver wiring |
| [`sensor.conf`](sensor.md) | channel map, divider ratios, calibration |
| [`limits.conf`](limits.md) | protection cutouts |
| [`coil.conf`](coil.md) | inductor |
| [`converter.conf`](converter.md) | topology |
| [`charger.conf`](charger.md) | battery termination |
| [`tracker.conf`](tracker.md) | MPPT |
| [`mqtt.conf`](mqtt.md) | broker + BMS coupling |
| [`tele.conf`](tele.md) | InfluxDB telemetry |
| [`wifi.conf`](wifi.md) | Wi-Fi credentials + roaming |
| [`ftp.conf`](ftp.md) | FTP service |
| [`telnet.conf`](telnet.md) | telnet console service |
| [`scope.conf`](scope.md) | raw ADC streaming service |
| [`lcd.conf`](lcd.md) | status display service |
| [`ble.conf`](ble.md) | BLE/NUS console service |
| [`bsync.conf`](bsync.md) | beacon PWM-clock sync service |
| [`pprof.conf`](pprof.md) | sampling profiler |
| [`vconv.conf`](vconv.md) | virtual converter plant (simulation builds) |
| [Examples](examples.md) | Complete configurations |
