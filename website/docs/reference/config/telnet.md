---
title: telnet.conf
sidebar_position: 12
---

# telnet.conf

`telnet.conf` configures the remote console over TCP port 23, which runs as the service `telnet`. Its keys `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys described in [Service Architecture](../services.md#per-service-conf-file).

The telnet console has no authentication and no encryption. It is off unless `enabled=1`, and it runs only
read-only commands unless `trusted=1` (see [Trusted consoles](../console.md#trusted-consoles)). With `trusted=1`
anyone on the network can run any command, including `ota`. The service is compiled in only with `CONFIG_FUGU_WITH_TELNET` (on by default, see [build options](../../guide/getting-started/build-options.md)).

The following table lists the keys in `telnet.conf`.

| key         | unit | type | default | description                          |
|-------------|------|------|---------|--------------------------------------|
| `enabled`   |      | bool | 0       | Start this service at boot           |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn` or `info` |
| `trusted`   |      | bool | 0       | Allow every command; 0 = read-only commands only. Read when the service starts, so `svc rs telnet` applies it |
