---
title: ble.conf
sidebar_position: 15
---

# ble.conf

BLE/NUS console (service `ble`, `CONFIG_FUGU_WITH_BLE` builds). Off by default because it exposes the console. `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys, see [Service Architecture](../services.md#per-service-conf-file).

| key            | unit | type   | default   | description                                                                 |
|----------------|------|--------|-----------|-----------------------------------------------------------------------------|
| `ble_security` |      | string | justworks | `justworks` (encrypted, no passkey), `passkey` (bond + MITM); any other value = open console |
| `ble_passkey`  |      | int    | 0         | Static passkey for `passkey` mode                                           |
| `enabled`      |      | bool   | 0         | Start this service at boot                                                  |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info` |
