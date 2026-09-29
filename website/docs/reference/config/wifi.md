---
title: wifi.conf
sidebar_position: 10
---

# wifi.conf

Wi-Fi credentials + roaming.

SSIDs are pattern-based; add as many `ssid_<name>` / `ssid_<name>_psk` pairs as needed (also via the
`wifi-add ssid:psk` console command). The primary SSID may instead live in NVS (`wifi_ssid` /
`wifi_psk`), which takes precedence. Wi-Fi is a precondition for the network services, not a service itself.

| key               | unit | type   | default | description                                                                                                  |
|-------------------|------|--------|---------|--------------------------------------------------------------------------------------------------------------|
| `ssid_<name>`     |      | string | —       | SSID to join                                                                                                 |
| `ssid_<name>_psk` |      | string | —       | Passphrase for that SSID                                                                                     |
| `switch_delay`    | s    | int    | 30      | Keep retrying the lost AP (router reboot) before roaming to another configured network; 0 = roam immediately |
