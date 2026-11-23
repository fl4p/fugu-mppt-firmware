---
name: vconv builds: S3, classic, OTA hazard
description: S3/classic vconv build + flash recipes, the 0W-OTA hazard, and the fixed MPPT fragility mechanism
created: 2026-11-23T09:45:45.316Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

`CONFIG_FUGU_WITH_VCONV` is one of the three gate drivers in the build-time `FUGU_GATE_DRIVER`
choice (LEDC / MCPWM / VCONV, commit 1e5c0225). It no longer `depends on !FUGU_WITH_MCPWM`, so the
old "VCONV is invisible while MCPWM is on" gotcha is gone; VCONV swaps in the software plant
(`src/sim/vconv.*`, `sensor.conf adc=vconv`) and drives no gates. See
[[project_runtime_selectable_pwm_driver]].

**S3** — separate build dir + its own sdkconfig, or the root `sdkconfig` (MCPWM=y) wins:

```bash
idf.py -B build-vconv -DSDKCONFIG=build-vconv/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.vconv_s3" build
```

`config/lab/vconv_mock` is the S3 board config (mcu=esp32s3), `mode=psu`/`psu_vout=28` for PSU-mode
testing; the plant is a buck with PV input (Isc=13, Voc=76) and a 27 V/0.05 Ω battery, driven with
`vconv bat <v>|short|open`.

**Classic ESP32** — own sdkconfig too (`sdkconfig.esp32vconv` = `sdkconfig.esp32app` + VCONV=y),
`IDF_TARGET=esp32 idf.py -B build-esp32-vconv -DSDKCONFIG=sdkconfig.esp32vconv build` (fits the
classic OTA slot). `config/lab/vconv_mock` is **S3-only** (mcu + GPIO 47/40/42 crash `pinMode` on
classic); use `config/lab/vconv_mock_esp32`. Flash the firmware alone (`app-flash` — the build
bundles the `fugu1_esp32` littlefs), then `./provision.py config/lab/vconv_mock_esp32`.

**Never OTA a vconv build to fry/flat.** 2026-05-30: an 18:15 OTA left both at 0 W, endless
SWEEP→`Vr-sensor-fail`→backoff while the pack discharged ~42 A. `buck.h` picks `PwmDriver` at
compile time, so the real gates were never toggled — but sensor.conf still had the real ADC
backends, so Vin read a real ~74 V Voc and the device looked alive (sampler running, sweeps) while
transferring nothing at any duty. The `Vr-sensor-fail` guard was correct, not overzealous. Before
OTAing fry/flat: `grep -iE 'WITH_VCONV|MEASURE_COIL' sdkconfig` must be off. Signature on real HW:
0 W, Vin glued at Voc with no sag across all duties, repeated `Vr-sensor-fail`.

**Convergence is healthy since 285cb181** (re-verified 2026-05-30 on the S3 bench: 814-815 W at
Vin≈64.6 V, advancing to slow P&O, stable over 35 s; classic same day: ~815 W, Iout≈27.8 A,
Vout≈28.4 V, 3000 sps, no trips). The old timing-fragility was **notification coalescing under RT
overrun**, not a wall-clock dt: `dt = 1/adcFreq` is fixed (`src/adc/vconv.h`), but
`ulTaskNotifyTake(pdTRUE)` collapsed several ticks into one wakeup while the plant stepped once, so
plant-time fell behind controller-time and bistable P&O on the steep PV knee landed in either basin.
Fix: `hasData()` steps the plant by the coalesced tick count (`stepSeconds(dt*ticks)`, capped at
kMaxCatchupTicks=8). Lesson kept: rtcount means are inflated by ADC-ISR preemption — the `min`
column is the truer compute figure. Related: [[project_fry_flat_may29_crash_rootcause]].
