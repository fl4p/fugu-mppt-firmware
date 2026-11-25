---
title: tlscon.conf
sidebar_position: 12.5
---

# tlscon.conf

`tlscon.conf` configures the TLS-PSK console, which runs as the service `tlscon`. It is the console over TCP,
encrypted and authenticated with TLS 1.2 and a pre-shared key (cipher suite `TLS-PSK-WITH-AES-128-GCM-SHA256`). It
exists only in builds with `CONFIG_FUGU_WITH_TLS_CONSOLE` (off by default, see
[Build options](../../guide/getting-started/build-options.md#feature-flags)). Its keys `enabled` (0/1) and
`log_level` (`error`, `warn` or `info`) are the common service keys described in
[Service architecture](../services.md#per-service-conf-file).

The following table lists the keys in `tlscon.conf`.

| key         | unit | type   | default | description                                                    |
|-------------|------|--------|---------|----------------------------------------------------------------|
| `enabled`   |      | bool   | 0       | Start this service at boot                                     |
| `log_level` |      | enum   | info    | Verbosity: `error`, `warn` or `info`                           |
| `port`      |      | int    | 992     | TCP port                                                       |
| `psk_id`    |      | string | `""`    | PSK identity the client must present                           |
| `psk`       |      | string | `""`    | Key as an even number of 32 to 64 hex digits (16 to 32 bytes)  |

`port`, `psk_id` and `psk` are read when the service starts, so `svc rs tlscon` applies them. The service does not
start without `psk_id` and a valid `psk`; `svc list` then shows it as failed. There is no
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
with FTP or HTTP config access. The console masks it as `***` in the command echo, the logs, `get-config` and `cat`
(see [Secrets](../console.md#secrets)).

## Sessions

The server accepts one client at a time:

- Once the handshake completes, the client receives the banner (`Welcome to <hostname> (<ip>)`, the firmware
  version) and from then on the muxed log, like a telnet client.
- Each line it sends runs as a console command; `exit` closes the session.
- A line longer than 128 bytes is discarded and answered with `ERR: line too long`.
- A second client is refused while a session is open, and so is any client while the free heap is below
  24 000 bytes, or below the session cost plus 2 KB with static buffers (about 27 KB at the defaults). With static
  buffers the largest free block must also hold the input buffer.
- A handshake that has not completed after 10 s is dropped.

The TLS context is allocated only while a client is connected. The listening socket and the PSK configuration stay
allocated while the service runs. An open session costs about 2 KB of heap with `CONFIG_MBEDTLS_DYNAMIC_BUFFER=y` and
about 24.6 KB with the default static buffers (see [TLS heap](../../guide/getting-started/build-options.md#tls-heap)).

## Client

`etc/fugu_console.py` connects with `--tls HOST[:PORT]`; the key comes from `--psk` and `--psk-id`, or from
`$FUGU_PSK` and `$FUGU_PSK_ID`; the identity defaults to `fugu`. It needs Python 3.13 or later. Any TLS client with PSK support works as well:

```bash
python3 etc/fugu_console.py --tls <device-ip> --psk-id <identity> --psk <hex key> -c status
openssl s_client -connect <device-ip>:992 -tls1_2 -psk_identity <identity> -psk <hex key> \
  -cipher PSK-AES128-GCM-SHA256
```
