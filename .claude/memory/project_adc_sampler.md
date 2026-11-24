---
name: project_adc_sampler
description: ADC sampler/continuous-DMA bugs and checks — no-sample watchdog deadlock, TaskNotify burst at boot, fry ADC HW healthy in isolation, IDF uniform-atten rule
metadata:
  type: project
---

## No-sample watchdog deadlock
FIXED (2026-05, after `81c9c10` regressed fry to 0 sps). `ADC_Sampler::_updateAdc` returned `AdcError` on
`!isGood()` before calling `read()`, but for the continuous-DMA backend `read()` is what drains DMA and refreshes
`lastDataUs_` → one >1 s gap = dead forever. Fix: drain `read()` first, then judge `isGood()`.
**Lesson:** a watchdog must never gate the operation that clears it.

## Boot "ADC error" = TaskNotify burst
FIXED (fry-brk1 regression). `TaskNotification::wait()` (`src/etc/rt.h`) used `ulTaskNotifyTake(pdFALSE)==1`; during the
`delay(1000)` in loopRT the conv-done ISR piled up notifications → wait false → `read()` never ran → watchdog trip.
Fix: `ulTaskNotifyTake(pdTRUE, …) != 0` (binary-semaphore use; old code also dropped samples when ≥2 frames queued).
Debug gotcha: `%lld` misparses under newlib-nano ([[project_build_toolchain]]).

## fry ADC HW healthy in isolation
2026-05-29: `test/test_adc_hw.cpp` (`RUN_TESTS=1 TEST_ADC_HW=1`, forces gate pins 21/14 low, safe on a live converter,
flash app-only at 0x10000; hardcodes fry pins) passed: internal ADC 458 sps, INA226 512 conv/s, alert ISR 1025×/2 s,
0 I2C errors. So production stalls (`40000 timeout!`, `no ADC samples`) are integration/timing (single-shot INA226
in the RT loop, alert ISR core), not dead silicon. See [[project_ina226]], [[project_crash_rootcauses]].

## IDF requires uniform attenuation per ADC unit
2026-08-12, FIXED in `adc_esp32_cont.h/.cpp` (`maxAtten`, all channels normalized in `start()`).
`adc_continuous_config()` returns `ESP_ERR_INVALID_ARG` silently when ADC1 channels differ in atten (e.g. Vin divider
→ DB_6, NTC → DB_12) → 0 sps / NaN sensors, loopRT blocked. **How to apply:** on a new board with 0 sps, check
`adc-reset` output for this signature.
