---
title: tele.conf
sidebar_position: 9
---

# tele.conf

`tele.conf` configures the InfluxDB telemetry service `tele`, which is off by default. The file holds the following keys.

| key             | unit | type   | default | description                                                          |
|-----------------|------|--------|---------|----------------------------------------------------------------------|
| `influxdb_host` |      | string | —       | InfluxDB host IP for UDP telemetry (port 8086)                       |
| `binary`        |      | bool   | 0       | 1 = binary symbol-table wire (sym_line_protocol.h, tamp-compressed); 0 = text influx. Needs CONFIG_FUGU_WITH_TELE_BINARY (or BLE_TELE); other builds log a warning and send text |
| `ble`           |      | bool   | 1       | Allow the BLE telemetry stream (`tele-ble` command; needs CONFIG_FUGU_WITH_BLE_TELE). Gates BLE only; `enabled` gates the UDP service only |
| `adv_ms`        | ms   | int    | 500     | BLE advertising telemetry broadcast refresh interval (needs CONFIG_FUGU_WITH_BLE_ADV), 0 = off, min 100. Connectionless: observers decode the advertisement (`influx_binary_proxy.py --adv`). Keeps broadcasting while a client holds the NUS link |
| `idle_interval_ms` | ms | int  | 900000  | Minimum interval between telemetry points (UDP and BLE stream) while the idle power profile is on (see [`board.conf::idle_cpu_mhz`](board.md)), up to 3600000 (1 h); otherwise 20 ms. While the interval is 1 s or longer, every point carries all fields (otherwise some are sent only with every 10th to 40th point) and is sent at once instead of waiting for a full UDP batch. The first point after the profile ends is not delayed. 0 = unchanged |
| `enabled`       |      | bool   | 0       | Start the UDP flush service at boot                                  |
| `log_level`     |      | enum   | info    | Verbosity: `error`, `warn` or `info`                                 |
