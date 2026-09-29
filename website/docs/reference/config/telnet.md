---
title: telnet.conf
sidebar_position: 12
---

# telnet.conf

Remote console over TCP (service `telnet`). `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys, see [Service Architecture](../services.md#per-service-conf-file).

| key         | unit | type | default | description                          |
|-------------|------|------|---------|--------------------------------------|
| `enabled`   |      | bool | 1       | Start this service at boot           |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn` or `info` |
