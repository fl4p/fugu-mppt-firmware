---
title: "BTHome Advertising (proposal)"
sidebar_position: 4
---

# BTHome v2 Advertising Spec — Charger Telemetry

:::note Proposal — not implemented
BTHome advertising is a design proposal. The firmware contains no BTHome code.

The firmware implements a custom connectionless broadcast instead (`CONFIG_FUGU_WITH_BLE_ADV`,
default off, needs `CONFIG_FUGU_WITH_BLE`; `src/tele/tele_adv.cpp`). It sends a 17-byte
little-endian record in the manufacturer-data AD (company id `0xFFFF`), laid out as
`u8 magic=0xF7, u8 seq, f16 Ui Uo I P, i8 mcu_temp ntc_temp, u16 pwm_duty, u16 lag_us, u8 state (mppt_state | cv_lim_idx<<4)`.
`tele.conf::adv_ms` sets the refresh interval (default 500 ms, min 100, `0` = off). The broadcast
continues (non-connectable) while a console client is connected. On the host,
`etc/influx_binary_proxy.py --adv` receives it. Home Assistant's BTHome integration does not decode it.
:::

## Context

This spec lets Home Assistant (HA) discover a Fugu charger through its BTHome integration, which currently has no way to do so. Fugu chargers publish telemetry over MQTT/InfluxDB (Wi-Fi) and offer a BLE NUS console for service work (`src/tele/console_ble.cpp`). Many users keep a Bluetooth proxy in range of the converter but have no Wi-Fi on the converter itself, for example in a caravan or an off-grid shed, or when they monitor multiple chargers from one Home Assistant.

The spec adds an always-on, plaintext BTHome v2 service-data field to the existing NimBLE advertising. The single-frame layout publishes one of each BTHome property: PV voltage, charge power, NTC temperature, daily energy, and a bit-packed status byte. Charge power is signed, so backflow shows as negative. The frame omits currents and Vout, for the reasons in *Why these ranges / type choices*. AES-CCM, total-energy, and Vout/Iout rotation frames are deferred (see *Open items*).

The change is limited to the payload. It adds no Kconfig option of its own (it builds whenever `CONFIG_FUGU_WITH_BLE` is set) and no GATT service, and its flash growth does not put the tight ~1.87 MB OTA slot at risk (current image ~1.86 MB).

## On-air format

The frame uses a legacy BLE advertising PDU, 31 B max. The existing NUS console keeps its 128-bit service UUID in the scan response (already enabled by `setScanResponse(true)` in `src/tele/console_ble.cpp`), and the advertising data carries BTHome instead.

The advertising data has the following structure (max 31 B, plaintext BTHome v2):

```
02 01 06                                    ; Flags AD (LE-only general-discoverable)
HH 16 D2 FC 40 <objects…>                   ; Service Data AD, UUID 0xFCD2 LE, info 0x40
```

The fields of the Service Data AD are:

- `HH` = length byte = `1 (type) + 2 (UUID) + 1 (info) + Nobjects`
- `0x16` = AD type "Service Data — 16-bit UUID"
- `0xD2 0xFC` = BTHome UUID (LE on-air)
- `0x40` = BTHome v2 device-info: bit 7..5 = `010` (v2), bit 2 = 0 (periodic), bit 0 = 0 (plaintext)

This leaves a maximum object payload of `31 - 3 (flags) - 1 (len) - 4 (type+UUID+info) = 23 B`.

### Object layout (primary frame, 19 B)

The primary frame carries six objects in this order:

| Order | BTHome ID | Field    | Type            | Factor      | Source                                                  |
|------:|----------:|----------|-----------------|-------------|---------------------------------------------------------|
| 1     | `0x00`    | pkt-id   | u8              | —           | wraps 0..255 per frame, used by HA to dedupe            |
| 2     | `0x4A`    | Vin      | u16 LE, 0.1 V   | 0.1 V       | `sensors.Vin->ewm.avg.get()`                            |
| 3     | `0x5C`    | Pout     | i32 LE, 0.01 W  | 0.01 W      | `sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get()` (signed → backflow shows negative) |
| 4     | `0x45`    | T_ntc    | i16 LE, 0.1 °C  | 0.1 °C      | `mppt.ntc.last()`                                       |
| 5     | `0x0A`    | E_daily  | u24 LE, kWh     | 0.001 kWh   | `mppt.meter.dailyEnergyMeter.today.energyYield / 1000`  |
| 6     | `0x09`    | state    | u8              | —           | bit-packed: see *State byte* below                      |

Bytes per object include the 1-byte ID. Total = `2+3+5+3+4+2 = 19 B`, with 4 B headroom inside the 23 B budget.

Every object ID appears exactly once, so HA names entities directly (`voltage`, `power`, `temperature`, `energy`, `count`) without `_2` suffixes. HA users don't need to remember which side a value refers to.

### State byte (`0x09` count)

The state object is a single u8 that holds two nibbles:

- Low nibble (bits 0..3): MPPT phase, mapped from `MpptControlMode` (`src/mppt.h`): `0 = off`, `1 = sweep`, `2 = fast P&O`, `3 = slow P&O`, `4 = CV`, `5 = CC`, `6 = CP`, `7 = manualPwm`. The encoder fixes the final enum mapping. Document the exact byte values on this page once the encoder lands.
- High nibble (bits 4..7): charger phase: `0 = idle`, `1 = bulk`, `2 = absorption/CV`, `3 = float`, `4 = terminated`, `5 = fault/backoff`. Derived from `mppt.charger.termCond`, `Vout_max()`, and the protection state.

Users can split the two nibbles with an HA template helper such as this one:

```yaml
sensor:
  - platform: template
    sensors:
      fugu_mppt_phase:
        value_template: "{{ states('sensor.fugu_count') | int(0) % 16 }}"
      fugu_charger_phase:
        value_template: "{{ states('sensor.fugu_count') | int(0) // 16 }}"
```

### Why these ranges / type choices

Each field uses the following type for the following reason:

- Vin range `0x4A` (0.1 V, 0..6553 V): some Fugu boards exceed the 65 V cap of `0x0C` (0.001 V).
- Pout uses signed `0x5C` i32 (±21.4 MW at 0.01 W). It costs 1 B more than unsigned `0x0B` u24, but it exposes reverse-current / sink behaviour as negative power, which makes it the most informative single field.
- Temp uses `0x45` (0.1 °C, signed). It is coarser than `0x02` and saves 0 bytes; it was chosen for HA readability.
- E_daily uses `0x0A` u24 kWh (max 16.7 MWh), enough for a single charger-day, and fits in 4 B.
- No Iin / no Iout: Omitting Iin follows the Victron-MPPT precedent (they publish solar *power*, never solar current). Dropping Iout removes the `voltage` (PV) vs `current` (battery) HA-side ambiguity, so users don't have to remember which side `current` refers to. Signed Pout covers backflow diagnostics, and per-side currents stay available via MQTT / the console.
- Vout omitted: BMS-equipped users already see pack & cell voltages via the BMS in HA. Calibration-delta debugging loses one data source, which is accepted in v1.

## Firmware changes

### New: `src/tele/bthome.h` / `src/tele/bthome.cpp`

The encoder is a pure function with no state, globals, or BLE dependencies, so it builds on the host for unit tests. Its interface is:

```cpp
// src/tele/bthome.h
#pragma once
#include <cstdint>
#include <cstddef>

namespace bthome {
// Builds the BTHome v2 service-data AD value (info byte + objects), excluding the AD length/type/UUID
// header — BLEAdvertising::setServiceData() handles that. Returns the number of bytes written.
// `buf` must be ≥ 24 bytes.
struct ChargerFrame {
    uint8_t  pktId;     // monotonic
    float    vinV;
    float    poutW;     // signed; negative = power into the converter (backflow / sink)
    float    tempC;
    float    dailyKWh;
    uint8_t  state;     // low nibble = MPPT phase, high nibble = charger phase
};
size_t encodeChargerFrame(uint8_t *buf, size_t cap, const ChargerFrame &f);
} // namespace bthome
```

The encoder follows these rules:
- Clamp to representable range before scaling; saturate on overflow.
- All multi-byte values little-endian.
- Skip a field only by dropping its bytes entirely (no zero-pad).
- The function is `constexpr`-friendly enough to fuzz on the host.

### New: `src/tele/ble_bthome_service.h`

The service is a small `Service` subclass that needs no `.cpp` file:

```cpp
class BleBthomeService : public Service {
public:
    BleBthomeService() : Service("bthome", "/littlefs/conf/bthome.conf",
                                 /*requiresNetwork*/ false, /*enabledDefault*/ false) {}
protected:
    bool onStart() override;   // captures BLEAdvertising* and seeds the first frame
    void onStop() override;    // setServiceData("") to clear the field; advertising continues
    void onTick() override;    // every ~5 s, rebuild ChargerFrame and call setServiceData()
};
inline BleBthomeService bleBthomeService;
```

The service behaves as follows:
- `#ifdef WITH_BLE` gates it. `main/CMakeLists.txt` defines `WITH_BLE` when `CONFIG_FUGU_WITH_BLE` is set. With BLE compiled out, the file collapses to an empty stub, as `BleConsoleService` does.
- It depends on `BleConsoleService` starting first. It doesn't call `BLEDevice::init()` or `startAdvertising()`. It only calls `getAdvertising()->setServiceData(BLEUUID((uint16_t)0xFCD2), payload)` to populate the AD. `BleConsoleService` registers before `BleBthomeService` in `setup()`. Document this dependency in the service description so the manager start order is obvious.
- If `BleConsoleService` is not running (NUS disabled / Wi-Fi-only deployment), `onStart()` reports `Failed` with detail `"requires ble console"`. A future v2 could own the advertising lifecycle when NUS is off, in a separate spec.
- `onTick()` limits itself to ≥5 s between rebuilds. `loopNetwork_task` already calls the service tick often enough.
- It pulls data only from `mppt` / `sensors` on core 0, with no extra synchronization. This is the same access pattern as `MpptController::telemetry()` in `src/mppt.cpp`.

### Modified: `src/tele/console_ble.cpp`

To make room for the BTHome service data in the primary PDU, the NUS service UUID moves from the advertising data to the scan response. The change sits in the advertising setup of `console_ble.cpp`:

```cpp
BLEAdvertising *adv = BLEDevice::getAdvertising();
adv->setScanResponse(true);
auto *scanResp = BLEDevice::getAdvertising();   // NimBLE uses the same handle; pick the
scanResp->setScanResponseData(/* NUS UUID + name */);  // exact API per the wrapper version
// Primary adv: leave UUID list empty so BleBthomeService can own ~23 B of service-data room
BLEDevice::startAdvertising();
```

The exact NimBLE-Arduino call that moves the NUS UUID into the scan response depends on the wrapper version (`setScanResponseData(BLEAdvertisementData&)` vs the deprecated boolean knob). During implementation, verify it against the pinned arduino-esp32 version. That version is capped below 3.3.8, whose BLE code needs an IDF NimBLE API absent from IDF 5.5.1.

### New: `/littlefs/conf/bthome.conf`

The file is a standard service conf (`enabled`, `log_level`) for the manager. v1 has no BTHome-specific keys. The encryption key and rotation interval would live here when AES-CCM is added.

### Build / no flash growth claim

The change adds `bthome.cpp` (≈ 100 lines including range clamps) and a one-file service wrapper, with no new dependencies. The NimBLE stack and `BLEAdvertising::setServiceData` are already linked (`src/tele/console_ble.cpp`). Image growth target: < 2 KB, well within the OTA slot margin.

## Documentation changes

The feature needs these documentation updates:

- [Configuration reference](../../reference/config/index.md): add a `bthome.conf` page with the standard `enabled` / `log_level` keys.
- `etc/config-tool/conf-editor.html`: add `bthome.conf` to `FILE_KEYS` with the same two keys.
- The BLE console docs: add a short "BTHome telemetry" subsection with the object table, the byte budget, and an HA discovery screenshot URL placeholder.

## Verification

Verify the feature in these steps:

1. Host unit tests (`test/host-stub/` already exists): hand-craft a `ChargerFrame` with known values, encode, compare bytes against a hex-string fixture computed by hand. Cover range saturation (Vin > 6553.5 V), negative Pout (backflow), zero values, all-zero frame (must still produce a parseable empty-objects payload with only the info byte), and state-byte bit-pack roundtrip.
2. On-target smoke: flash a build with `CONFIG_FUGU_WITH_BLE=y` (the default), enable both `ble` and `bthome` services (`svc on ble; svc on bthome`). Use a host BLE scanner (`bluetoothctl` on Linux or the `bleak` library) to dump the raw advertising bytes. Expect a `16 D2 FC 40 …` substring that matches the encoded frame.
3. HA end-to-end: with an ESPHome Bluetooth Proxy in range, the BTHome integration should auto-discover the device. Verify the sensor list contains exactly `voltage`, `power`, `temperature`, `energy`, `count` (rename per `Voltage PV`, `Charge Power`, `Temp`, `Daily Yield`, `Status`). Compare values against the converter's MQTT topics over a 60 s window; they should match within rounding (`0.1 V`, `0.01 W`, `0.1 °C`, `0.001 kWh`). The daily-energy reset at midnight should show up within the next 5 s advertising tick. Force a state transition (e.g. `dc 0` to idle, `mppt` to re-enable) and confirm the `count` value flips nibbles accordingly.
4. Sniff a packet (optional): `etc/fugu/discover.py` doesn't sniff adv, but `etc/pico_capture.py`'s sibling tooling lives in the same `etc/` tree. Capture with `bluetoothctl --monitor` or `nrfutil` and confirm the byte sequence matches the table.
5. Check for regressions: confirm that the NUS console is still connectable after the AD reshuffle. Run `etc/fugu_console.py --ble fugu-<hostname>` and execute a few commands.

## Open items (deferred, not v1)

These items are deferred beyond v1:

- AES-CCM encryption: add `bthome_key` (32 hex chars) and a monotonic counter persisted in NVS; the info byte becomes `0x41`, with +8 B overhead (counter LE + MIC). Counter rollover is fatal, so it needs a clear "burn the key" failure mode. Reference implementation: ESPHome `bthome.cpp` or the spec page https://bthome.io/encryption.
- Rotating Frame B for the leftovers: total kWh (`mppt.meter.totalEnergy.get()` / 1000, `0x0A` u24 or `0x4D` u32) + Vout + per-side currents (Iin/Iout) for non-BMS users / power-flow debug. Alternates with Frame A by pkt-id parity. HA picks up both within one re-scan cycle (~10–30 s).
- Extended advertising (BLE 5 `ADV_EXT_IND`): if more fields ever become essential and the rotating-frame UX is poor, a non-connectable extended-adv set on a separate handle gives up to 254 B at the cost of sdkconfig changes (`CONFIG_BT_NIMBLE_EXT_ADV`) and a flash bump that must be re-checked against the OTA slot ceiling.
- Per-board calibration awareness: v1 drops Vout, so a board whose Vout sensor reads low (one unit was observed ~0.35 V low) is affected only via Pout (Vout × Iout). HA will see a slightly low Pout on that unit until calibration is fixed in `sensor.conf`. The BLE doc should mention this.
