---
title: ble.conf
sidebar_position: 15
---

# ble.conf

`ble.conf` configures the BLE/NUS console, which is the service `ble` in `CONFIG_FUGU_WITH_BLE` builds. The service is off by default because it exposes the console.

`enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys. For how services use them, see [Service Architecture](../services.md#per-service-conf-file).

The file has the following keys:

| key            | unit | type   | default   | description                                                                 |
|----------------|------|--------|-----------|-----------------------------------------------------------------------------|
| `ble_security` |      | string | justworks | `justworks` (encrypted, no passkey), `passkey` (bond + MITM); any other value = open console |
| `ble_passkey`  |      | int    | 0         | Static passkey for `passkey` mode                                           |
| `enabled`      |      | bool   | 0         | Start this service at boot                                                  |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info` |
