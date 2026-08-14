---
name: IDF adc_continuous_config requires uniform attenuation per ADC unit
description: IDF adc_continuous_config rejects mixed attenuation across ADC1 channels with ESP_ERR_INVALID_ARG; fixed by normalizing all channels to maxAtten in adc_esp32_cont.h
created: 2026-08-12T21:22:45.225Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_00826dadfffeNbdVew49f9Z8v4
---

ESP-IDF `adc_continuous_config()` (adc_continuous.c:510-513) requires all ADC1 channels in the DMA pattern table to share the **same attenuation**. If two channels have different atten (e.g. ch3 at DB_6, ch7 at DB_12), it silently returns `ESP_ERR_INVALID_ARG` — no log, just the error code. This manifested as `0sps` / all-NaN sensors on a bench device: the ADC init failed at boot, `loopRt` blocked forever in `adcSampler.update()`.

**Root cause:** `setMaxExpectedVoltage()` in `adc_esp32_cont.h` picks attenuation per-channel based on the expected voltage at the ADC pin. A Vin divider (200k/7.5k, max 1.45V → DB_6) and an NTC divider (max 3.3V → DB_12) produce mixed atten. The pattern duplication code (`[vin, ntc, vin]`) makes it worse but the real issue is the per-channel atten difference.

**Fix** (adc_esp32_cont.h, adc_esp32_cont.cpp): track `maxAtten` across all `setMaxExpectedVoltage()` calls. In `start()`, normalize all `attenByCh[ch]` to `maxAtten` and create the calibration handle for `maxAtten` if not already present. Lower-voltage channels lose a tiny bit of resolution but the IDF constraint is satisfied. The `read()` path already uses `calByAtten[attenByCh[chan_num]]` so it picks up the normalized atten automatically.

**Why:** Discovered 2026-08-12 testing PSU mode on a real Fugu2 bench board (MAC 34:85:18:82:40:28). The fmetal buck config happened to work because both internal ADC channels (ch3 Vin, ch7 NTC) got the same atten by coincidence. The solar-boost config also works because Vout (ch3, high-V divider → DB_12) and NTC (ch7 → DB_12) both land on DB_12. The bug only surfaces when channels have genuinely different voltage ranges.

**How to apply:** The fix is in adc_esp32_cont.h (`maxAtten` member, normalization in `start()`). Any new ADC channel config will now work regardless of per-channel voltage differences. If debugging "ADC error" / 0sps on a new board, check `adc-reset` output for the `ESP_ERR_INVALID_ARG` / `adc_continuous_config` signature.
