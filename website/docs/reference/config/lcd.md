---
title: lcd.conf
sidebar_position: 14
---

# lcd.conf

`lcd.conf` configures the I2C status display (service `lcd`). `enabled` (0/1) and `log_level` (`error`, `warn` or `info`) are the common service keys. For details, see [Service architecture](../services.md#per-service-conf-file).

The file has the following keys:

| key         | unit | type | default | description                                |
|-------------|------|------|---------|--------------------------------------------|
| `addr`      |      | int  | 0       | LCD I2C address; 0, or an address that does not answer, probes 0x27 and 0x3F. Read when the service starts (`svc rs lcd`) until a display is found |
| `enabled`   |      | bool | 0       | Start this service at boot                 |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn`, or `info` |
