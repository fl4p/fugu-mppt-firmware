---
title: wifi.conf
sidebar_position: 10
---

# wifi.conf

This file holds the Wi-Fi credentials and the roaming setting.

SSID keys are pattern-based. Add as many `ssid_<name>` / `ssid_<name>_psk` pairs as needed, either by editing the file
or with the `wifi-add ssid:psk` console command. The primary SSID may instead live in NVS (`wifi_ssid` /
`wifi_psk`), which takes precedence.

Wi-Fi is a precondition for the network services. It isn't a service itself.

The file accepts the following keys:

| key               | unit | type   | default | description                                                                                                  |
|-------------------|------|--------|---------|--------------------------------------------------------------------------------------------------------------|
| `ssid_<name>`     |      | string | —       | SSID to join                                                                                                 |
| `ssid_<name>_psk` |      | string | —       | Passphrase for that SSID                                                                                     |
| `switch_delay`    | s    | int    | 30      | Keep retrying the lost AP (router reboot) before roaming to another configured network; 0 = roam immediately |
