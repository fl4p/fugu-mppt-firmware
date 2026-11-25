---
title: ftp.conf
sidebar_position: 11
---

# ftp.conf

The `ftp` service gives access to the littlefs partition, including the config files, over the LAN (port 21). `enabled` (0/1) and `log_level` (`error`, `warn` or `info`) are the common service keys (see [Service architecture](../services.md#per-service-conf-file)).

NVS credentials take precedence, and the conf values are the fallback. If either credential is still
unset after that, both fall back to user `user` and the chip ID as password. `ftp_user`, `ftp_pass` and `pasv_port` are read when the service
starts, so `svc rs ftp` applies them (`svc rs http` for the [HTTP endpoint](http.md), which uses the same
credentials). The console masks `ftp_pass` as `***` (see [Secrets](../console.md#secrets)).

:::warning
FTP gives read-write access to the whole littlefs partition, including `/littlefs/conf` (calibration, limits, Wi-Fi and MQTT secrets). Set both
`ftp_user` and `ftp_pass`, because the fallback password is the chip ID, which is not secret. The `ftp` warning log
says the fallback is in use (`using user=user, password = chip ID`) but does not print the password.
:::

The file accepts the following keys:

| key         | unit | type   | default | description                          |
|-------------|------|--------|---------|--------------------------------------|
| `ftp_user`  |      | string | `user` (if either credential is unset) | FTP username |
| `ftp_pass`  |      | string | chip ID (if either is unset) | FTP password |
| `pasv_port` |      | int    | 50009   | Passive data port of the first session; the second uses `pasv_port`+1. Outside 1024..65534 falls back to 50009 |
| `enabled`   |      | bool   | 1       | Start this service at boot           |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info` |
