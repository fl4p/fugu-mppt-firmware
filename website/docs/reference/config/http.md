---
title: http.conf
sidebar_position: 11.5
---

# http.conf

`http.conf` configures the HTTP file endpoint for the config partition, which runs as the service `http`. It exists
only in builds with `CONFIG_FUGU_WITH_HTTP_CONF` (off by default, see
[Build options](../../guide/getting-started/build-options.md#feature-flags)). Its keys `enabled` (0/1) and
`log_level` (`error`/`warn`/`info`) are the common service keys described in
[Service Architecture](../services.md#per-service-conf-file).

The following table lists the keys in `http.conf`.

| key         | unit | type | default | description                          |
|-------------|------|------|---------|--------------------------------------|
| `enabled`   |      | bool | 1       | Start this service at boot           |
| `log_level` |      | enum | info    | Verbosity: `error`, `warn` or `info` |
| `port`      |      | int  | 80      | TCP port, read when the service starts (`svc rs http`) |

The server uses the same credentials as the FTP service (NVS `ftp_user`/`ftp_pass`, then [`ftp.conf`](ftp.md), then
user `user` with the chip ID as password), sent as HTTP Basic auth. They are read when the service starts, so
`svc rs http` applies a change.

:::warning
The endpoint gives read-write access to `/littlefs/conf` (calibration, limits, Wi-Fi and MQTT secrets) over plain
HTTP. Basic auth sends the password unencrypted; use it only on a trusted network and set `ftp_user` and `ftp_pass`.
:::

## Endpoints

All paths are relative to `/littlefs/conf`. File names may contain only `A-Z`, `a-z`, `0-9`, `.`, `_` and `-`, must
not start with a dot, and are at most 32 characters long; anything else returns 400.

| Request | Response |
|---|---|
| `GET /conf/` | `200`, one `name<TAB>size` line per file (`text/plain`) |
| `GET /conf/<name>` | `200` with the file, or `404` |
| `PUT /conf/<name>` | `201` (created) or `204` (replaced); needs `Content-Length` (`411`), at most 16 KB (`413`) |
| `DELETE /conf/<name>` | `204`, or `404` |
| `OPTIONS` any path | `204` CORS preflight, no auth |

A `PUT` writes to a temporary file and renames it over the target only after the full body has arrived, so an
aborted upload leaves the old file in place. Every response carries `Access-Control-Allow-Origin: *` and
`Connection: close`.

## Limits

The server runs inside the network loop without its own task:

- at most 2 connections; further clients wait in the listen backlog until one closes
- request line and headers up to 1 KB (`414`/`431` beyond)
- 10 s to send the headers, then 10 s for the body and the response; a stalled connection is dropped

## Example

```bash
curl -u user:pass http://<device>/conf/
curl -u user:pass http://<device>/conf/charger.conf -o charger.conf
curl -u user:pass -T charger.conf http://<device>/conf/charger.conf
curl -u user:pass -X DELETE http://<device>/conf/old.conf
```

Settings read at boot take effect after `restart`; to change a single value, `set-config` needs no file transfer.
