---
title: tele.conf
sidebar_position: 9
---

# tele.conf

`tele.conf` configures the InfluxDB telemetry service `tele`, which is off by default. The file holds the following keys.

| key             | unit | type   | default | description                                                          |
|-----------------|------|--------|---------|----------------------------------------------------------------------|
| `influxdb_host` |      | string | —       | InfluxDB host IP for UDP telemetry (port 8086)                       |
| `binary`        |      | bool   | 0       | 1 = binary symbol-table wire (sym_line_protocol.h, tamp-compressed); 0 = text influx |
| `ble`           |      | bool   | 1       | Allow the BLE telemetry stream (`tele-ble` command; needs CONFIG_FUGU_WITH_BLE_TELE). Gates BLE only; `enabled` gates the UDP service only |
| `adv_ms`        | ms   | int    | 500     | BLE advertising telemetry broadcast refresh interval (needs CONFIG_FUGU_WITH_BLE_ADV), 0 = off, min 100. Connectionless: observers decode the advertisement (`influx_binary_proxy.py --adv`). Keeps broadcasting while a client holds the NUS link |
| `enabled`       |      | bool   | 0       | Start the UDP flush service at boot                                  |
| `log_level`     |      | enum   | info    | Verbosity: `error`, `warn` or `info`                                 |
