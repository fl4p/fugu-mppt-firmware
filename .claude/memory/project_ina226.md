---
name: INA226 alert path, latency shutdowns, OV knob
description: INA226 alert path (CVRF re-arm, why the MASK_EN read is load-bearing), the loop-latency shutdowns, and the bus-OV alert knob
created: 2026-11-23T09:47:04.968Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

**The alert path (`src/adc/ina226.h`).** In conv-ready mode the ALERT pin tracks CVRF, and CVRF
clears **only** on a Mask/Enable read (`readAndClearFlags`) or a Config write — not on the next
conversion and not by reading BUS/CURRENT. So `hasData()`'s per-sample `MASK_EN` read is
load-bearing and cannot be dropped for speed: without it you get exactly one falling edge and then
silence. Proven by `testContinuousAlert()` (calls `readAndClearFlags()` between edges) and by
production sustaining ~450 SPS only because `hasData()` reads it every cycle. Disambiguation is
*not* a reason: alert sources are mutually exclusive, only CNVR is enabled, and OVF (bit 2) does not
assert the pin. The floor is 3 I2C reads/sample (MASK_EN re-arm + BUS + CURRENT); the only real
latency lever is conversion time (`CONV_TIME` 1100→588 µs). No auto-re-arming conv-ready mode
exists on INA226 (INA228/237 differ = hardware swap). See [[project_fry_ina226_undereads_157]].

**"Loop latency high, shutdown!" (2026-05-22, 21 trips at 230-380 W).** The INA226 chronically
misses conversion-ready alerts: `hasData()` waits 3 ms per miss (logging only every 20000th) and fry
logged 20000→120000 in ~31 min ≈ 54 timeouts/s, each injecting a 3 ms wait into the RT loop. On
fmetal both `vout` (ch0/ChVBus) and `iout` (ch1/ChI) are INA226 channels, so a missed alert freezes
`Vout->numSamples` — which *is* the watchdog metric; sustained loss drops sps under
`expected_hz` (200, normal ~451-511) for >2.7 s → `stopAndBackoff`. The path is edge-fragile:
continuous mode, non-latched conv-ready, FALLING-edge ISR (GPIO 41), flag cleared by the MASK_EN
read; a late read-clear vs the next ~1.1 ms conversion loses the edge. `i2c_freq=800000` is
aggressive and fry's INA226 is suspected counterfeit (much higher shunt noise), both marginalising
the path under switching EMI. **The watchdog is a correct symptom reporter, not the bug** — tell-tale
`lag` constant (~2705 µs on fry) with `0 kB/s` scope means the RT loop is blocked waiting for data,
not stalled computing.

**Refinement 2026-05-28 (idle, ~0 W in CV).** ~7 shutdowns/hour, median 511 sps, ~10% of 1 s windows
<200, max lag ~3 ms — and **7/7 fired ~1 s after a console command** (`hostname`, `ip`, `uptime`,
`getc`), including pure in-memory ones that read no flash. So at idle the trigger is console-command
handling on core 0 stalling the RT loop on core 1 (the same heap-lock/log-alloc contention as the
"deferred logging still mallocs on the RT core" note in Real-Time Latency.md). At high power INA226
starvation dominates; at idle the loop sits near the edge and any console command pushes a window
under 200 — a discovery poller thus shuts the converter down every poll. Fixes are written up in
Real-Time Latency.md ("Console commands trip the loop-latency watchdog": log-alloc off the RT path,
require N consecutive low windows, stop polling the console) but **not implemented**.

**Bus-OV alert knob (`board.conf ina22x_ov`, main bd1f472, 2026-11-14).** Value is volts after the
divider; 0 = the old conv-ready alert. When set, ALERT = BOL|LEN, the ISR (`g_hwOvIsrHook`) forces
the MCPWM gates low and sets `g_hwOvTrip` → `protect()` → `shutdownDcdc("hw-OV")`; samples are found
by polling CVRF (esp_timer wakes 300 µs early, busy-wait ≤400 µs). Fry-tested: trips mid-sweep under
switching, 20/20 forced OTAs clean with the knob armed, 511-513 sps, +~0.6 ms max loop gap, 1.38
MASK_EN reads/sample; fry was left with the knob at 0. 9053667 + 3815a3c add a GPIO ISR service with
`ESP_INTR_FLAG_IRAM` behind Kconfig `FUGU_GPIO_ISR_IRAM` (default n, S3, needs `rt_iram_extra.lf`);
`AdsAlertISR` became `IRAM_ATTR` (ICACHE_RAM_ATTR expands to nothing in arduino-esp32 3.x).
**Not device-tested on the caches-off path and not reviewed.** d53f9cb (11-14, not device-tested)
carries review fixes: lost arm trips and retries once per period (ISR/hasData keyed on `ovRaw_`, not
`ovArmed_`), a mismatch no longer re-arms in the same poll, `armOv` restores CAL/conv and never
clears `g_hwOvTrip`, `forceShutdown` always_inline. Before enabling `FUGU_GPIO_ISR_IRAM`: review plus
a forced caches-off test (patch `fugu_flash_sus_set(false)` in flash_probe.cpp, constant OV alert, 20
set-config writes, no panic). Open issue: the enable path can clear the ISR force for one sample.
