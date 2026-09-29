---
title: "BTHome Advertising (proposal)"
sidebar_position: 4
---

*this document is an LLM generated placeholder*

# BTHome v2 Advertising Spec — Charger Telemetry

:::note Proposal — not implemented
BTHome advertising is a design proposal; no BTHome code exists in the firmware.

What is implemented is a custom connectionless broadcast (`CONFIG_FUGU_WITH_BLE_ADV`, default off,
needs `CONFIG_FUGU_WITH_BLE`; `src/tele/tele_adv.cpp`): a 17-byte little-endian record in the
manufacturer-data AD (company id `0xFFFF`) — `u8 magic=0xF7, u8 seq, f16 Ui Uo I P,
i8 mcu_temp ntc_temp, u16 pwm_duty, u16 lag_us, u8 state (mppt_state | cv_lim_idx<<4)`.
`tele.conf::adv_ms` sets the refresh interval (default 500 ms, min 100, `0` = off). It keeps
broadcasting (non-connectable) while a console client is connected. Host side:
`etc/influx_binary_proxy.py --adv`. Home Assistant's BTHome integration does not decode it.
:::

## Context

Fugu charges publish telemetry over MQTT/InfluxDB (Wi-Fi) and offer a BLE NUS console for service work (`src/tele/console_ble.cpp`). Many users keep a Bluetooth proxy in range of the converter but have no Wi-Fi on the converter itself (caravan, off-grid sheds, monitoring multiple chargers from one Home Assistant). Today there is no way for HA's BTHome integration to discover the charger.

This spec defines an **always-on, plaintext BTHome v2 service-data field** piggybacked on the existing NimBLE advertising. The single-frame layout publishes one of each BTHome property — PV voltage, charge power (signed, so backflow shows as negative), NTC temperature, daily energy, and a bit-packed status byte — so HA names every entity unambiguously (`voltage`, `power`, `temperature`, `energy`, `count`) with no `_2` disambiguation guesswork. Currents are intentionally omitted (Iin is a nerd stat per Victron's MPPT precedent; Iout would otherwise force the user to remember which side `current` refers to); Vout is omitted because BMS-equipped users already see it. AES-CCM, total-energy, and Vout/Iout rotation frames are deferred — see *Open items*.

The work is payload-only: no new sdkconfig flags, no new GATT service, no flash growth that risks the tight ~1.87 MB OTA slot (current image ~1.86 MB).

## On-air format

Legacy BLE advertising PDU, 31 B max. The existing NUS console keeps its 128-bit service UUID in the **scan response** (already enabled at `src/tele/console_ble.cpp:249`); the **advertising data** is repurposed to carry BTHome.

Advertising data structure (max 31 B, plaintext BTHome v2):

```
02 01 06                                    ; Flags AD (LE-only general-discoverable)
HH 16 D2 FC 40 <objects…>                   ; Service Data AD, UUID 0xFCD2 LE, info 0x40
```

- `HH` = length byte = `1 (type) + 2 (UUID) + 1 (info) + Nobjects`
- `0x16` = AD type "Service Data — 16-bit UUID"
- `0xD2 0xFC` = BTHome UUID (LE on-air)
- `0x40` = BTHome v2 device-info: bit 7..5 = `010` (v2), bit 2 = 0 (periodic), bit 0 = 0 (plaintext)

Maximum object payload = `31 - 3 (flags) - 1 (len) - 4 (type+UUID+info) = 23 B`.

### Object layout (primary frame, 19 B)

| Order | BTHome ID | Field    | Type            | Factor      | Source                                                  |
|------:|----------:|----------|-----------------|-------------|---------------------------------------------------------|
| 1     | `0x00`    | pkt-id   | u8              | —           | wraps 0..255 per frame, used by HA to dedupe            |
| 2     | `0x4A`    | Vin      | u16 LE, 0.1 V   | 0.1 V       | `sensors.Vin->ewm.avg.get()`                            |
| 3     | `0x5C`    | Pout     | i32 LE, 0.01 W  | 0.01 W      | `sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get()` (signed → backflow shows negative) |
| 4     | `0x45`    | T_ntc    | i16 LE, 0.1 °C  | 0.1 °C      | `mppt.ntc.last()`                                       |
| 5     | `0x0A`    | E_daily  | u24 LE, kWh     | 0.001 kWh   | `mppt.meter.dailyEnergyMeter.today.energyYield / 1000`  |
| 6     | `0x09`    | state    | u8              | —           | bit-packed: see *State byte* below                      |

Bytes per object include the 1-byte ID. Total = `2+3+5+3+4+2 = 19 B`, with 4 B headroom inside the 23 B budget.

Every object ID appears exactly once, so HA names entities directly (`voltage`, `power`, `temperature`, `energy`, `count`) — no `_2` ambiguity, no need for HA users to remember which side a value refers to.

### State byte (`0x09` count)

Single u8, two nibbles:

- **Low nibble (bits 0..3)** — MPPT phase, mapped from `MpptControlMode` (`src/mppt.h`): `0 = off`, `1 = sweep`, `2 = fast P&O`, `3 = slow P&O`, `4 = CV`, `5 = CC`, `6 = CP`, `7 = manualPwm`. Final enum mapping is finalised in the encoder; document the exact byte values on this page once the encoder lands.
- **High nibble (bits 4..7)** — charger phase: `0 = idle`, `1 = bulk`, `2 = absorption/CV`, `3 = float`, `4 = terminated`, `5 = fault/backoff`. Derived from `mppt.charger.termCond`, `Vout_max()`, and the protection state.

HA template helper example for users:

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

- Vin range `0x4A` (0.1 V, 0..6553 V): some Fugu boards exceed the 65 V cap of `0x0C` (0.001 V).
- Pout uses **signed** `0x5C` i32 (±21.4 MW at 0.01 W) — costs 1 B vs unsigned `0x0B` u24 but exposes reverse-current / sink behaviour as negative power, giving the most informative single field.
- Temp uses `0x45` (0.1 °C, signed) — coarser than `0x02` but saves 0 bytes; chosen for HA readability.
- E_daily uses `0x0A` u24 kWh (max 16.7 MWh) — plenty for a single charger-day, fits in 4 B.
- **No Iin / no Iout**: Iin matches the Victron-MPPT precedent (they publish solar *power*, never solar current). Dropping Iout removes the `voltage` (PV) vs `current` (battery) HA-side ambiguity. Signed Pout covers the backflow-diagnostics niche; per-side currents stay available via MQTT / the console.
- **Vout omitted**: BMS-equipped users see pack & cell voltages via the BMS in HA already. Calibration-delta debugging loses one data source — accept this in v1.

## Firmware changes

### New: `src/tele/bthome.h` / `src/tele/bthome.cpp`

Pure encoder. No state, no globals, no BLE deps. Buildable on host for unit tests.

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

Encoder rules:
- Clamp to representable range before scaling; saturate on overflow.
- All multi-byte values little-endian.
- Skip a field only by dropping its bytes entirely (no zero-pad).
- The function is `constexpr`-friendly enough to fuzz on the host.

### New: `src/tele/ble_bthome_service.h`

Tiny `Service` subclass (no `.cpp` needed):

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

Implementation notes:
- Gated by `#ifdef WITH_BLE`. With BLE compiled out the file collapses to an empty stub like `BleConsoleService` does.
- Takes a hard dependency on `BleConsoleService` being started first — it does **not** call `BLEDevice::init()` or `startAdvertising()`; only `getAdvertising()->setServiceData(BLEUUID((uint16_t)0xFCD2), payload)` to populate the AD. Document this in the service description so the manager start order is obvious; `BleConsoleService` registers before `BleBthomeService` in `setup()`.
- If `BleConsoleService` is not running (NUS disabled / Wi-Fi-only deployment), `onStart()` reports `Failed` with detail `"requires ble console"`. (Future v2: own the advertising lifecycle when NUS is off — a separate spec.)
- `onTick()` is rate-limited internally to ≥5 s between rebuilds; `loopNetwork_task` already calls the service tick often enough.
- Pulls data exclusively from `mppt` / `sensors` on core 0 — same access pattern as `mppt.telemetry()` in `src/mppt.cpp:424`, no extra synchronization.

### Modified: `src/tele/console_ble.cpp`

One change: move the NUS service UUID from advertising data to scan response so BTHome service data fits in the primary PDU.

Around `console_ble.cpp:247`:

```cpp
BLEAdvertising *adv = BLEDevice::getAdvertising();
adv->setScanResponse(true);
auto *scanResp = BLEDevice::getAdvertising();   // NimBLE uses the same handle; pick the
scanResp->setScanResponseData(/* NUS UUID + name */);  // exact API per the wrapper version
// Primary adv: leave UUID list empty so BleBthomeService can own ~23 B of service-data room
BLEDevice::startAdvertising();
```

The exact NimBLE-Arduino call to move NUS UUID into the scan response is wrapper-version-specific (`setScanResponseData(BLEAdvertisementData&)` vs the deprecated boolean knob); verify against the pinned arduino-esp32 version (capped below 3.3.8, whose BLE code needs an IDF NimBLE API absent from IDF 5.5.1) during implementation.

### New: `/littlefs/conf/bthome.conf`

Standard service conf (`enabled`, `log_level`) for the manager. No BTHome-specific keys in v1 — encryption key + rotation interval would live here when AES-CCM is added.

### Build / no flash growth claim

The added code is `bthome.cpp` (≈ 100 lines including range clamps) and a one-file service wrapper. No new dependencies. The NimBLE stack and `BLEAdvertising::setServiceData` are already linked (`src/tele/console_ble.cpp`). Image growth target: < 2 KB, well within the OTA slot margin.

## Documentation changes

- [Configuration reference](../../reference/config/index.md) — add a `bthome.conf` page with the standard `enabled` / `log_level` keys.
- `etc/config-tool/conf-editor.html` — add `bthome.conf` to `FILE_KEYS` with the same two keys.
- The BLE console docs — add a short "BTHome telemetry" subsection: object table, byte budget, HA discovery screenshot URL placeholder.

## Verification

1. **Host unit tests** (`test/host-stub/` already exists): hand-craft a `ChargerFrame` with known values, encode, compare bytes against a hex-string fixture computed by hand. Cover: range saturation (Vin > 6553.5 V), negative Pout (backflow), zero values, all-zero frame (must still produce a parseable empty-objects payload — info byte only), and state-byte bit-pack roundtrip.
2. **On-target smoke**: flash with `WITH_BLE=1`, enable both `ble` and `bthome` services (`svc on ble; svc on bthome`). Use a host BLE scanner — `bluetoothctl` on Linux or the `bleak` library — to dump the raw advertising bytes; expect to see `16 D2 FC 40 …` substring matching the encoded frame.
3. **HA end-to-end**: with an ESPHome Bluetooth Proxy in range, the BTHome integration should auto-discover the device. Verify the sensor list contains exactly `voltage`, `power`, `temperature`, `energy`, `count` (rename per `Voltage PV`, `Charge Power`, `Temp`, `Daily Yield`, `Status`). Compare values against the converter's MQTT topics over a 60 s window — should match within rounding (`0.1 V`, `0.01 W`, `0.1 °C`, `0.001 kWh`). Daily-energy reset at midnight should show up within the next 5 s advertising tick. Force a state transition (e.g. `dc 0` to idle, `mppt` to re-enable) and confirm the `count` value flips nibbles accordingly.
4. **Sniff a packet** (optional): `etc/fugu/discover.py` doesn't sniff adv, but `etc/pico_capture.py`'s sibling tooling lives in the same `etc/` tree — capture with `bluetoothctl --monitor` or `nrfutil` and confirm the byte sequence matches the table.
5. **Reject regressions**: NUS console still connectable after the AD reshuffle. Run `etc/fugu_console.py --ble fugu-<hostname>` and execute a few commands.

## Open items (deferred, not v1)

- **AES-CCM encryption**: add `bthome_key` (32 hex chars) and a monotonic counter persisted in NVS; info byte becomes `0x41`; +8 B overhead (counter LE + MIC). Counter rollover is fatal — needs a clear "burn the key" failure mode. Reference implementation: ESPHome `bthome.cpp` or the spec page https://bthome.io/encryption.
- **Rotating Frame B for the leftovers**: total kWh (`mppt.meter.totalEnergy.get()` / 1000, `0x0A` u24 or `0x4D` u32) + Vout + per-side currents (Iin/Iout) for non-BMS users / power-flow debug. Alternates with Frame A by pkt-id parity. HA picks up both within one re-scan cycle (~10–30 s).
- **Extended advertising** (BLE 5 `ADV_EXT_IND`): if more fields ever become essential and the rotating-frame UX is poor, a non-connectable extended-adv set on a separate handle gives up to 254 B at the cost of sdkconfig changes (`CONFIG_BT_NIMBLE_EXT_ADV`) and a flash bump that must be re-checked against the OTA slot ceiling.
- **Per-board calibration awareness**: a board whose Vout sensor reads low (one unit was observed ~0.35 V low) is, with Vout dropped from v1, only affected via Pout (Vout × Iout), so HA will see a slightly low Pout on that unit until calibration is fixed in `sensor.conf`. Worth a sentence in the BLE doc.
