---
title: ftp.conf
sidebar_position: 11
---

# ftp.conf

Config file access over the LAN (service `ftp`). `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys, see [Service Architecture](../services.md#per-service-conf-file).

NVS credentials take precedence; the conf values are the fallback.

| key         | unit | type   | default | description                          |
|-------------|------|--------|---------|--------------------------------------|
| `ftp_user`  |      | string | —       | FTP username                         |
| `ftp_pass`  |      | string | —       | FTP password                         |
| `enabled`   |      | bool   | 1       | Start this service at boot           |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info` |
