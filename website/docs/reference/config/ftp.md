---
title: ftp.conf
sidebar_position: 11
---

# ftp.conf

Config file access over the LAN (service `ftp`). `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys, see [Service Architecture](../services.md#per-service-conf-file).

NVS credentials take precedence; the conf values are the fallback. If either credential is still
unset after that, **both** fall back to user `user` and the chip ID as password.

:::warning
FTP gives read-write access to `/littlefs/conf` (calibration, limits, Wi-Fi and MQTT secrets). Set both
`ftp_user` and `ftp_pass`: the fallback password is printed in the `ftp` warning log, and the log is mirrored
to the connected console transports (telnet, and MQTT `pv/log/<host>` while connected).
:::

| key         | unit | type   | default | description                          |
|-------------|------|--------|---------|--------------------------------------|
| `ftp_user`  |      | string | `user` (if either credential is unset) | FTP username |
| `ftp_pass`  |      | string | chip ID (if either is unset) | FTP password |
| `enabled`   |      | bool   | 1       | Start this service at boot           |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info` |
