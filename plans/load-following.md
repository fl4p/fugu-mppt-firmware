*this document is an LLM generated placeholder*

# Load-Following for Terminated Chargers — Phased Plan

## Background

When charge termination latches, converters pin `vpack_pin` to a fixed voltage and
stop tracking. Two outcomes observed on 2026-06-30:

- **With Vout authority** (converter had duty when termination latched):
  `vpack_pin` glides to `Vbat_fallback` (26.8V). If bus < 26.8V (cable drop),
  converter produces power — but capped at float voltage, not `Vbat_max` (29V).
  Also trickle-charges the battery.

- **Without Vout authority** (converter was at 0A when termination latched):
  `vpack_pin` pinned to `Vbat_fallback - vout_offset_max` (26.2V). Bus sits at
  26.75V → converter stays at 0W. All solar wasted.

The asymmetry (flat at 400W, fry at 0W) is a race on who lost authority first.

---

## Phase 1 — Fix voutAuthority asymmetry

**Goal**: Make both converters behave the same when terminated + battery
discharging. Get fry producing power again.

**Root cause**: The `!voutAuthority && nowTerm` branch pins to the floor
(26.2V) "to yield to the sibling converter." But when the battery is
discharging (`ibat < -0.1A`), the pack is not full — there is no sibling
regulating termination, and the converter should participate.

### Changes

#### `src/charger.h` — `_updatePackVoltagePinning()`, `!voutAuthority` branch

Current (line 296-317):
```cpp
if (!voutAuthority && batDataOk) {
    if (nowTerm) {
        float floor = params.Vbat_fallback - params.vout_offset_max;
        ...
        vpack_pin = floor;
        ...
    } else {
        releaseVoutPinning("no Vout authority");
    }
    return;
}
```

Change: when `nowTerm` and battery is discharging, use `Vbat_fallback` instead
of the floor. Keep the floor only when `ibat ≈ 0` (pack genuinely full).

```cpp
if (!voutAuthority && batDataOk) {
    if (nowTerm) {
        float ibat = batSt.ibatSmoothed();
        bool discharging = std::isfinite(ibat) && ibat < -0.1f;
        float target = discharging ? params.Vbat_fallback
                                    : params.Vbat_fallback - params.vout_offset_max;
        if (std::isnan(vpack_pin) || vpack_pin > target + 0.01f)
            ESP_LOGW("charger", "no Vout authority (terminated%s): yield %.3f -> %.3f",
                     discharging ? ", discharging" : "", vpack_pin, target);
        vpack_pin = target;
        ioutLim = NAN;
        _vPinFilt.reset();
        _fallbackGlide.reset();
        _floatGlide.reset();
        _wasTerminated = nowTerm;
    } else {
        releaseVoutPinning("no Vout authority");
    }
    return;
}
```

#### `src/mppt.cpp` line 70 — allow sweeps when discharging

Current:
```cpp
bool batteryFull = bool(charger.termCond) || ctrlState.mode == MpptControlMode::CV;
```

Change:
```cpp
float ibat = charger.batSt.ibatSmoothed();
bool batteryDischarging = std::isfinite(ibat) && ibat < -0.1f;
bool batteryFull = (bool(charger.termCond) || ctrlState.mode == MpptControlMode::CV)
                   && !batteryDischarging;
```

#### `src/main.cpp` line 934 — allow startup sweep when discharging

Current:
```cpp
bool full = bool(mppt.charger.termCond) && bmsFresh;
```

Change:
```cpp
bool discharging = std::isfinite(mppt.charger.batSt.ibatSmoothed())
                   && mppt.charger.batSt.ibatSmoothed() < -0.1f;
bool full = bool(mppt.charger.termCond) && bmsFresh && !discharging;
```

### Result

Both converters target `Vbat_fallback` (26.8V) when terminated + discharging.
Both can produce power if bus < 26.8V. Symmetric. The startup sweep fires
from duty=0 because `batteryFull` is false.

**Known limitation**: The converter targets 26.8V (3.35V/cell) regardless of
load — it trickle-charges the battery if solar > load. Phase 2 fixes this.

### Risk

Low. The floor bypass only triggers when `ibat < -0.1A` (pack discharging,
not full). The floor is still used when `ibat ≈ 0` (genuinely full pack,
EOC limit-cycling protection intact).

### Testing

1. Build and OTA both fry and flat
2. With battery at 3.34V/cell and -2A discharge: both start sweeps, produce
   power, targeting 26.8V
3. Verify fry is no longer stuck at 0W
4. Verify both converters share the load (symmetric `vpack_pin = 26.8V`)
5. Verify no errors in logs
6. When battery recovers to full (ibat → 0): floor pin resumes, converters
   yield correctly

---

## Phase 2 — Load-following via current observation

**Goal**: Replace the fixed `Vbat_fallback` target with an OCV estimate so the
converter supplies the load without trickle-charging the battery.

**Principle**:
```
r_cell = (cv_eoc - cv_min) / (tail_c_rate * Cbat)   # from termination line config
R_pack = r_cell * n_cells
vpack_pin = Vbat_fallback - ibat * R_pack            # ibat<0 → raise, ibat>0 → lower
```

The converter targets the battery's OCV. It produces just enough power for
ibat → 0 (load covered, no charge current).

### BMS sampling rate

BMS publishes ibat and cell voltages on separate topics at different rates
(observed: ibat ~4s, cell voltage ~12s; `CoulombCounter` has `maxDt = 30s`).
Key decisions:

1. **Gate on ibat frames**: Add `ibat_t` timestamp to `BatteryState`, set in
   `updateBatCurrent()`. Step load-following on `ibat_t` changes, not
   `vcell_high_t`.
2. **No additional EWMA**: `ibatSmoothed()` is already EWMA-filtered by the
   producer (8 samples = 32s at 4s rate, 4 min at 30s rate). Apply the
   proportional calculation directly.
3. **Response time**: At 30s BMS rate, `vpack_pin` adjusts every 30s. The
   VoutController inner loop (443-511 Hz) clamps Vout to `vpack_pin` every
   tick. Battery buffers transients. Acceptable for house loads.

### Changes

#### `src/charger.h`

1. **`BatteryState`**: Add `volatile uint32_t ibat_t = 0;` set in
   `updateBatCurrent()`.

2. **`BatteryCharger`**:
   - Add `bool _loadFollowing = false;`, `uint32_t _lastIbatFrameUs = 0;`
   - Add `_packResistance()`:
     ```cpp
     float _packResistance() const {
         if (!std::isfinite(params.Cbat) || params.Cbat <= 0 || params.tail_c_rate <= 0)
             return NAN;
         return ((params.cv_eoc - params.cv_min) / (params.tail_c_rate * params.Cbat))
                * params.n_cells;
     }
     ```
   - Add `_loadFollowUpdate()`:
     - Gate on `batSt.ibat_t != _lastIbatFrameUs`
     - `R = _packResistance(); if (!std::isfinite(R)) return;`
     - `vPin = params.Vbat_fallback - ibat * R;`
     - Clamp to `[params.Vbat_fallback - params.vout_offset_max, params.Vbat_max]`
     - No EWMA, reset glides, log at info level
   - Add `[[nodiscard]] bool isLoadFollowing() const`

3. **`_updatePackVoltagePinning()`** — restructure:

   Compute `loadFollow` early:
   ```cpp
   float ibat = batSt.ibatSmoothed();
   constexpr float LOAD_FOLLOW_THRESHOLD = 0.1f;
   bool loadFollow = nowTerm && batDataOk
                     && std::isfinite(ibat) && ibat < -LOAD_FOLLOW_THRESHOLD;
   ```

   State transitions:
   - `!_loadFollowing && loadFollow` → enter: log, reset glides
   - `_loadFollowing && !loadFollow` → exit: log, start float glide from
     current `vpack_pin` to `Vbat_fallback`

   **`!voutAuthority` branch**: Replace the Phase 1 `Vbat_fallback` target with
   `_loadFollowUpdate()` when `loadFollow`. Keep floor for `ibat ≈ 0`.

   **Main if-else chain**: Add `else if (loadFollow)` before the
   `nowTerm && batDataOk` float branch. This unifies authority/non-authority
   paths — both converters call `_loadFollowUpdate()` with the same `ibat`.

#### `src/cli.cpp`

`cmdStatus()`:
```cpp
UART_LOG("Charger: %s%s",
         (bool) chg.termCond ? "TERMINATED" : "charging",
         chg.isLoadFollowing() ? " (load-following)" : "");
```

#### `website/docs/guide/charging/termination.md`

Add section on load-following mode.

### Result

Both converters target OCV, supply the load without charging, symmetric
sharing, no trickle. If solar < load, `vpack_pin` rises to `Vbat_max` → full
MPPT. If solar > load, `vpack_pin` drops below float → curtailed.

### Edge cases

- **BMS ibat not configured**: `ibatSmoothed()` → NAN → `loadFollow` false →
  Phase 1 behavior (float at `Vbat_fallback`).
- **BMS cell voltage stale**: `batDataOk` false → `loadFollow` false →
  BMS-stale glide.
- **Solar < load**: ibat stays negative → `vpack_pin` → `Vbat_max` →
  full MPPT → battery covers deficit.
- **Solar > load**: ibat positive → `vpack_pin` drops → VoutController
  clamps → curtailed.
- **Termination release**: voltage floor (3.32V) or DoD (56Ah) →
  load-following exits → bulk/MPPT.
- **EOC feedback priority**: if `vcell_high >= v_eoc`, EOC feedback runs
  first. Load-following only when cells below `v_eoc`.

### No new config parameters

Threshold (0.1A) and gain (`R_pack` from existing config) need no new keys.

### Testing

1. Build and OTA both fry and flat
2. With battery at 3.34V/cell and -2A discharge: both enter load-following,
   produce power matching the load
3. `status` shows "TERMINATED (load-following)"
4. ibat moves toward 0 on BMS topic
5. No charge current (ibat stays ≤ 0)
6. Both converters share the load symmetrically
7. Termination still releases when battery drops below 3.32V/cell
