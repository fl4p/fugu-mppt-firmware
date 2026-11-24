---
title: scope.conf
sidebar_position: 13
---

# scope.conf

The `scope` service streams raw ADC data over TCP port 24 for noise debugging; it also announces `_scope._tcp` over mDNS. It exists in builds with `CONFIG_FUGU_WITH_NETW` and `CONFIG_FUGU_WITH_SCOPE` (both on by default) and runs while Wi-Fi is connected. The host client is `etc/scope.py`.

This file holds the common service keys `enabled` (0/1) and `log_level` (`error`/`warn`/`info`). [Service Architecture](../services.md#per-service-conf-file) describes them.

The following table lists the keys:

| key         | unit | type | default | description                          |
|-------------|------|------|---------|--------------------------------------|
| `enabled`   |      | bool | 1       | Start this service at boot           |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn` or `info` |
