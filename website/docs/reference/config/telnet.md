---
title: telnet.conf
sidebar_position: 12
---

# telnet.conf

`telnet.conf` configures the remote console over TCP, which runs as the service `telnet`. Its keys `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys described in [Service Architecture](../services.md#per-service-conf-file).

The following table lists the keys in `telnet.conf`.

| key         | unit | type | default | description                          |
|-------------|------|------|---------|--------------------------------------|
| `enabled`   |      | bool | 1       | Start this service at boot           |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn` or `info` |
