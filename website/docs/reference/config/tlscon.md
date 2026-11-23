---
title: tlscon.conf
sidebar_position: 12.5
---

# tlscon.conf

`tlscon.conf` configures the TLS-PSK console, which runs as the service `tlscon`. It is the console over TCP,
encrypted and authenticated with TLS 1.2 and a pre-shared key (cipher suite `TLS-PSK-WITH-AES-128-GCM-SHA256`). It
exists only in builds with `CONFIG_FUGU_WITH_TLS_CONSOLE` (off by default, see
[Build options](../../guide/getting-started/build-options.md#feature-flags)). Its keys `enabled` (0/1) and
`log_level` (`error`/`warn`/`info`) are the common service keys described in
[Service Architecture](../services.md#per-service-conf-file).

The following table lists the keys in `tlscon.conf`.

| key         | unit | type   | default | description                                                    |
|-------------|------|--------|---------|----------------------------------------------------------------|
| `enabled`   |      | bool   | 0       | Start this service at boot                                     |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info`                           |
| `port`      |      | int    | 992     | TCP port                                                       |
| `psk_id`    |      | string | `""`    | PSK identity the client must present                           |
| `psk`       |      | string | `""`    | Key as 32 to 64 hex digits (16 to 32 bytes)                    |

The service does not start without `psk_id` and a valid `psk`; `svc list` then shows it as failed. There is no
built-in key. Generate one per device and set it over a console that is already trusted, such as serial or BLE:

```bash
python3 -c "import secrets; print(secrets.token_hex(32))"
```

```
set-config tlscon.conf psk_id <identity>
set-config tlscon.conf psk <hex key>
svc on tlscon
```

Setting the key over the MQTT console sends it through the broker, and in plaintext unless the broker connection
uses TLS. The key is stored in `/littlefs/conf/tlscon.conf` like the other credentials, so it is readable by anyone
with console, FTP or HTTP config access. `set-config` logs each command with its value to every log output (serial,
telnet, the MQTT log topic, a `tlscon` session), so the key also appears there.

## Sessions

The server accepts one client at a time. Once the handshake completes, the client receives the banner
(`Welcome to <hostname> (<ip>)`, the firmware version) and from then on the muxed log, like a telnet client. Each
line it sends runs as a console command; `exit` closes the session. A second client is refused while a session is
open, and so is a client when the free heap is below 24 KB. A handshake that has not completed after 10 s is
dropped.

The TLS context is allocated only while a client is connected. The listening socket and the PSK configuration stay
allocated while the service runs.

## Client

`etc/fugu_console.py` connects with `--tls HOST[:PORT]`; the key comes from `--psk` and `--psk-id`, or from
`$FUGU_PSK` and `$FUGU_PSK_ID`. It needs Python 3.13 or later. Any TLS client with PSK support works as well:

```bash
python3 etc/fugu_console.py --tls <device-ip> --psk-id <identity> --psk <hex key> -c status
openssl s_client -connect <device-ip>:992 -tls1_2 -psk_identity <identity> -psk <hex key> \
  -cipher PSK-AES128-GCM-SHA256
```
