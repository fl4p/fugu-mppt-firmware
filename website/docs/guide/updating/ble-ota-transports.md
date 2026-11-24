---
title: "Direct OTA Transport Selection"
sidebar_position: 3
---

# Direct OTA transport selection

`etc/ota_ble.py` accepts the shared esp-ota-ble transport controls. It loads the
host module from the esp-ota-ble component: a local `../esp-ota-ble` checkout beside
the repository, else `managed_components/esp-ota-ble/host` after the first build.
`ESP_OTA_BLE_HOST` overrides both.

## Backend selection

`--ble-backend auto` picks the native CoreBluetooth sender on macOS when the Xcode
command line tools are installed, and Bleak everywhere else. It is the default in host
modules that offer it; the component revision pinned in `main/idf_component.yml`
predates it and defaults to Bleak. To force Bleak, pass `--ble-backend bleak`.

The native sender waits for CoreBluetooth's readiness callback. Bleak can only poll
`canSendWriteWithoutResponse`; when that flag stays false on a live link, the push
fails with `CoreBluetooth remained unwritable`.

On Linux, `--ble-backend bumble` takes `--adapter`, `--chunk`, `--ble-interval-ms`,
and `--ble-phy`. `--experimental-hci-packet-size 251` requires Bumble and `--adapter`,
and uses a nonstandard HCI packet size that only some controllers accept.

For prerequisites, restrictions, measurement limitations, and examples, see
[host transports](https://github.com/fl4p/esp-ota-ble/blob/main/doc/host-transports.md)
in the esp-ota-ble repository.

## Verification and scope

Direct OTA verifies the target digest on the new slot after reboot. The ESPHome
proxy path rejects direct-only flags.

## Tests

`etc/test_ota_transport.py` tests the transport selection against fake peers; set
`ESP_OTA_BLE_HOST` to the component's `host` directory.
