---
title: "Direct OTA Transport Selection"
sidebar_position: 3
---

# Direct OTA transport selection

`etc/ota_ble.py` accepts the shared esp-ota-ble transport controls. It loads the
host module from the esp-ota-ble component (`managed_components/esp-ota-ble/host`
after the first build, or a local `../esp-ota-ble`); `ESP_OTA_BLE_HOST` overrides it.

The default `--ble-backend auto` picks the native CoreBluetooth sender on macOS when the Xcode
command line tools are installed, and Bleak everywhere else. The native sender waits for
CoreBluetooth's readiness callback; Bleak can only poll `canSendWriteWithoutResponse`, which has
been seen to stay false on a live link until the push fails with `CoreBluetooth remained
unwritable`. Pass `--ble-backend bleak` to force Bleak. Linux can select `--ble-backend bumble`
with `--adapter`, `--chunk`, `--ble-interval-ms` and `--ble-phy`.
The nonstandard HCI experiment requires the separate
`--experimental-hci-packet-size 251` flag and the specific tested controller.

See `esp-ota-ble/doc/host-transports.md` for prerequisites, exact restrictions,
measurement limitations and examples. Direct OTA verifies the target digest
on the new slot after reboot. ESPHome proxy behavior is unchanged and rejects
direct-only flags. These host changes do not modify converter firmware or
authorize flashing a live converter.

Validation: `ESP_OTA_BLE_HOST=/path/to/esp-ota-ble/host python3 etc/test_ota_transport.py`.
Tests use fake peers; no live converter was flashed for this integration.
