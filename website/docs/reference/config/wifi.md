---
title: wifi.conf
sidebar_position: 10
---

# wifi.conf

This file holds the Wi-Fi credentials and the roaming setting. Wi-Fi is a precondition for the network services, not
a service itself.

Any key that starts with `ssid` (and doesn't end in `_psk`) names a network; its passphrase is the same key plus
`_psk`, and a missing passphrase means an open network. The `wifi-add ssid:psk` console command writes such a pair
as `ssid_<name>` and `ssid_<name>_psk`. A network stored in NVS (`wifi_ssid`, `wifi_psk`) is joined as well. The
device scans and joins the strongest configured network it sees.

The file is read when Wi-Fi first starts after boot, so edits take effect after `restart`; `wifi-add` applies at
once. The console masks `_psk` values as `***` (see [Secrets](../console.md#secrets)).

| key                | unit | type   | default | description |
|--------------------|------|--------|---------|-------------|
| `ssid_<name>`      |      | string | —       | SSID to join |
| `ssid_<name>_psk`  |      | string | `""`    | Passphrase for that SSID; empty = open network |
| `switch_delay`     | s    | int    | 30      | Keep retrying the lost AP (router reboot) before roaming to another configured network; 0 = roam immediately |
| `improv_offline_s` | s    | int    | 600     | Wi-Fi down this long before [Improv over BLE](../../guide/connecting.md#improv-over-ble) is offered; while the BLE pairing window is open, at most 20 s. 0 = only while no network is configured; negative = Improv off (no GATT service). Read when BLE first starts after boot |
