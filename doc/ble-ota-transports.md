*this document is an LLM generated placeholder*

# Direct OTA transport selection

`etc/ota_ble.py` accepts the shared esp-ota-ble transport controls. Keep the
host checkout current, or set `ESP_OTA_BLE_HOST` to its `host/` directory.

Append `--ble-backend native` to the normal Mac OTA invocation to use the
native CoreBluetooth readiness queue. Linux can select `--ble-backend bumble`
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
