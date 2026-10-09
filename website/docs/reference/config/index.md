---
title: "Configuration Files"
sidebar_position: 0
---

# Configuration files

Hardware behavior and runtime options live in flat `key=value` `.conf` files on the device's
`littlefs` partition under `/littlefs/conf/`, rather than being compiled in. `ConfFile`
(`src/conf.h`) parses them. The source-of-truth image for each board lives under `config/` (e.g.
`config/fmetal`, `config/lab/wokwi_mock`).

## Editing configuration

You can read and change the configuration in these ways:

- `set-config <file>.conf <key> <value>` from the serial, telnet, MQTT, or BLE console edits a value
  in place, without a re-flash.
- `get-config <file>.conf <key>` reads one value back, and `get-config <file>.conf` reads the whole
  file.
- `del-config <file>.conf <key>` removes a value.
- FTP works when Wi-Fi is up (1 connection; passive mode, data port 50009, works from the same
  subnet only), or use `etc/config-tool/conf-tool.py`.
- The single-page editor `etc/config-tool/conf-editor.html` connects via serial or BLE or reads
  uploaded files, and exports a zip file.
- `./provision.py <board>` writes a whole `config/<board>` image to the littlefs partition.

## When changes take effect

`set-config` only rewrites the file. What applies the new value depends on the file:

- `board`, `sensor`, `limits`, `coil`, `converter`, `charger`, and `tracker` are read at boot, so
  reboot after `set-config`. Console verbs (`vset`, `iset`, `dt`, `pwm-freq`, …) change RAM only.
- For service confs, `svc rs <name>` re-reads the service-specific keys. To change `enabled`, use
  `svc on|off`, and to change `log_level`, use `svc log`. A hand-edited value of either takes effect
  at the next boot.
- A change to `ble.conf` security, passkey, or the device name needs a reboot.

## Table conventions

The tables on the file pages use these conventions:

- **default** is the value the firmware uses when the key is absent. `—` means the key is
  required and board-specific: there is no built-in default, and the value (usually a divider
  ratio, pin, or limit) must come from the board image.
- Channel `255` means absent. The firmware replaces a missing current sensor (`iin`/`iout`) with a
  `VirtualSensor` derived from the other side and `power_conversion_eff`.

## Files

Each file has its own reference page:

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
