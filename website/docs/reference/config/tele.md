---
title: tele.conf
sidebar_position: 9
---

# tele.conf

`tele.conf` configures the InfluxDB telemetry service `tele`, which is off by default, and the BLE telemetry
paths. `influxdb_host` and `binary` are read when the service starts (`svc rs tele`), `ble` on each `tele-ble 1`,
`adv_ms` when BLE starts (`svc rs ble`), and `idle_interval_ms` at boot. See
[Telemetry fields](../telemetry-fields.md) for the point contents.

| key                | unit | type   | default | description |
|--------------------|------|--------|---------|-------------|
| `influxdb_host`    |      | string | `""`    | InfluxDB IP address for UDP line protocol (port 8086); empty = no target |
| `binary`           |      | bool   | 0       | 1 = binary symbol-table wire, tamp-compressed; 0 = text line protocol. Needs `CONFIG_FUGU_WITH_TELE_BINARY` (selected by `BLE_TELE` and `TELE_MQTT`); other builds log a warning and send text |
| `ble`              |      | bool   | 1       | Allow the BLE telemetry stream (`tele-ble` command, `CONFIG_FUGU_WITH_BLE_TELE`). Gates BLE only; `enabled` gates UDP only |
| `adv_ms`           | ms   | int    | 500     | Refresh interval of the BLE advertising broadcast (`CONFIG_FUGU_WITH_BLE_ADV`); 0 = off, values below 100 = 100. Keeps broadcasting while a client holds the console link |
| `idle_interval_ms` | ms   | int    | 900000  | Minimum interval between points (UDP and BLE stream) while the idle power profile is on (see [`board.conf::idle_cpu_mhz`](board.md#idle-power-profile)), 0..3600000; otherwise 20 ms. At 1 s or more every point carries all fields and is sent at once. 0 = unchanged |
| `enabled`          |      | bool   | 0       | Start the UDP flush service at boot |
| `log_level`        |      | enum   | info    | Verbosity: `error`, `warn` or `info` |
