---
title: scope.conf
sidebar_position: 13
---

# scope.conf

Raw ADC streaming over TCP (service `scope`), for noise debugging. `enabled` (0/1) and `log_level` (`error`/`warn`/`info`) are the common service keys, see [Service Architecture](../services.md#per-service-conf-file).

| key         | unit | type | default | description                          |
|-------------|------|------|---------|--------------------------------------|
| `enabled`   |      | bool | 1       | Start this service at boot           |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn` or `info` |
