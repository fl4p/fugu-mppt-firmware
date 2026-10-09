---
title: scope.conf
sidebar_position: 13
---

# scope.conf

The `scope` service streams raw ADC data over TCP for noise debugging.

This file holds the common service keys `enabled` (0/1) and `log_level` (`error`/`warn`/`info`). [Service Architecture](../services.md#per-service-conf-file) describes them.

The following table lists the keys:

| key         | unit | type | default | description                          |
|-------------|------|------|---------|--------------------------------------|
| `enabled`   |      | bool | 1       | Start this service at boot           |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn` or `info` |
