*this document is an LLM generated placeholder*

# fry: recurring `adc_esp32: resetPeripherals: restarting continuous ADC`

Investigated 2026-11-01 on fry running `fry-brk1-395-gaebc8e42` (clean HEAD aebc8e4, OTA'd 2026-10-31 13:25,
archived ELF sha 11f99dc0). Only read-only console commands were used (`hostname`, `peek`, `rt-stats`). No
firmware or config was changed.

## Summary

- **Trigger:** the ADC stall watchdog in `loopRT`, `src/main.cpp:689-701`, through its **"stale" arm**. It
  fires when no new **Vout** sample has arrived for more than 200 ms. Vout and Iout come from the INA226.
  The watchdog then calls `adcSampler.resetPeripherals()`, which tears down and restarts **every** ADC: the
  healthy ESP32 DMA (hence the `adc_esp32` warning) and the INA226.
- **Why no `main: ADC stall ...` line was logged:** that `ESP_LOGW` exists only as an **uncommitted
  working-tree edit** (mtime 2026-10-23). It is not in HEAD and not in the running build. In aebc8e4 the
  watchdog resets silently. Nothing is being dropped by the logging path.
- **Why the INA226 starves:** sometimes its ALERT falling edge is missed, so `new_data` never gets set
  again. HEAD has a fallback that polls MASK_EN when `taskNotification.wait(alertTimeoutMs_)` times out.
  That fallback is **dead on any board that also runs the continuous ESP32 ADC**. The FreeRTOS task
  notification is per task, and the ESP32 DMA ISR notifies the RT task about every 1.45 ms (128 B frame =
  32 results at 22 kHz). `alertTimeoutMs_` is 6 ms (`ina226AlertTimeoutMs(1100)`), so the wait never
  times out, the poll never runs, and only the 200 ms watchdog with a full re-init recovers the chip.
- **Second, independent trigger (17 % of resets):** at the end of every sensor calibration. During
  calibration `loopRTNewData()` is not called, so `timeLastSampler` ages. On the first `NoNewData`
  iteration after "Calibration done!" the stale arm fires. The converter is still disabled at that point,
  so this one is harmless but noisy.
- **Harm:** each event is about 270 ms with Vout and Iout frozen. During that time the controller does not
  update the duty, and OV/OC protection compares stale values. The last ~68 ms of it block the RT core
  entirely, so Vin protection is also not running then. At fry's MPPT rate (~185/h) that is roughly
  50 s/h, about 1.4 % of converting time, blind on the output side. No shutdown has resulted: the 800 ms
  sustained-stall guard was never reached.

## Evidence

### 1. Only one automatic caller, and its log line is not in the build

`grep` over `src/`: `ADC_Sampler::resetPeripherals()` is called from `main.cpp:701` (stall watchdog) and
`ADC_Sampler::applyAdcResetRt()`. The latter is reached only through the `adc-reset`/`adc-restart` console
commands, and it logs `sampling: ADC reset/restart` (never seen). `ADC_ESP32_Cont::resetPeripherals()` has no
other caller. Calibration does not reset peripherals.

`git diff HEAD -- src/main.cpp` shows the `ESP_LOGW("main", "ADC stall (...), resetting")` line as an unstaged
addition. `git show aebc8e4:src/main.cpp` has `adcSampler.resetPeripherals()` with no log before it.
`elf_archive.py list` records the fry build as `fry-brk1-395-gaebc8e42` with no `-dirty` suffix.

The AdcError arm is ruled out. It needs the ESP32 DMA silent for more than 250 ms, and that logs
`adc_esp32: no ADC samples for N ms` (first occurrence unthrottled), or it needs a read error, which logs
`adc_continuous_read error`. Neither string appears in the fry log since 2026-10-28.

### 2. Telemetry: Vout and Iout freeze, Vin keeps moving

Influx `mppt` for fry is per-sample (~47 rows/s), and log timestamps match Influx UTC to within ~30 ms. In
2026-10-31 14:00-14:20Z (~345 W, 11.7 A) there are 56 runs of identical `Uo` and `I` lasting 120 ms or
more. In 15:00-15:20Z (low power, CV/MPPT) there are 61. Throughout these runs `Ui` (ESP32 ADC) keeps
changing. Each run ends at a logged reset:

| frozen Uo/I from (UTC) | duration | logged reset |
|---|---|---|
| 15:00:11.652 | 265 ms | 15:00:11,945 |
| 15:00:26.410 | 260 ms | 15:00:26,717 |
| 15:02:12.871 | 582 ms | 15:02:13,187 and 15:02:13,494 (back-to-back) |

265 ms ≈ 200 ms watchdog threshold + ~68 ms reset.

### 3. The fallback poll has never run

`peek 0x3fca75f4 12` on live fry, after 19.2 h uptime and several hundred resets. The symbols come from the
archived ELF: `ADC_INA226::hasData()::numRecovered` @ 0x3fca75f4, `::numTimeouts` @ 0x3fca75f8.

```
0x3fca75f4: 00 00 00 00 00 00 00 00 d0 22 cc 3f
```

Both counters are 0, so `taskNotification.wait(alertTimeoutMs_)` has never timed out since boot.

### 4. Reset statistics (fry log, 2026-10-28 .. 2026-10-31)

- 925 resets in total: 156 directly after `sampler: Calibration done!` (within 1 ms), 769 others.
- By converter state at the time: ↑/⇡/⇣/↓ MPPT 667, CV 96, START 6, after calibration 156. Nights and
  disabled (START) periods are almost reset-free, although the sampler runs at the same rate then. Resets
  also happen at about 0 W while the converter is switching, so they track **control-loop activity, not
  power**.
- Per hour: up to 193/h on 2026-10-31 afternoon. 20-100/h already on 2026-10-30 on the June build. The
  apparent increase after the OTA follows more MPPT time, not the new code.
- Intervals: median 11.8 s. 58 intervals are under 1 s, mostly exactly 301-303 ms. That is the
  `kAdcResetThrottleMs` cadence: the stall persisted past the first reset, partly because `timeLastSampler`
  is not refreshed after a watchdog reset (`applyAdcResetRt` does refresh it).
- RT-core block per reset, from device timestamps of the reset line to the `ina22x: measured N SPS` line:
  min 64, median 68, p95 71, max 72 ms (n=884). This is mostly the INA226 re-init busy-waits
  (`testConvReadyAlert`, `testContinuousAlert`, `measurePairUs(16)` ≈ 16 × 1.96 ms).
- flat, same build since 2026-10-31 13:34: the same pattern at a lower rate (~29/h in MPPT, none while it
  sits in CV). flat's INA226 is the genuine 444 SPS part; fry's is the 510 SPS one suspected to be a clone.

## Effect on control

| aspect | impact per event |
|---|---|
| Sampling gap | Vout and Iout: ~200 ms with no samples, plus ~68 ms reset. Vin: ~68 ms (DMA torn down and restarted). Status line sps dips 452 → 410. |
| Controller | `loopRTNewData` runs on each Vin sample, but `haveNewSample` (keyed on Vout count) stays false, so `mppt.update()` is not called. Duty is frozen for ~270 ms. |
| Protection | `mppt.protect()`/`protectLf()` keep running during the 200 ms detection window, but on frozen Vout/Iout, so OV/OC are blind. During the 68 ms reset nothing runs on the RT core. With a battery on the bus Vout is clamped. The exposure is a BMS disconnect or load dump inside the window, about 1.4 % of MPPT time on fry. |
| Filters | `ADC_Sampler::resetPeripherals()` does not reset sensor filters or calibration (only `adc-restart` does). Notch, median and EWM state simply resumes after a 270 ms time gap, giving a small transient. The ESP32 `avgBuf` is not cleared on restart, so one partial average mixes pre- and post-restart codes (negligible). If `rippleSrc` is an INA226 channel, the Goertzel block spanning the gap is corrupted. Plausible, but not checked, as a contributor to the frequent `ripple notch ->` retunes. |
| Watchdogs | The 800 ms sustained-stall shutdown and the loop-rate watchdog never tripped. The status line `lag=131005 µs` max loop lag is consistent with a reset (or two back-to-back) inside a single loop window. |

## Why the ALERT edge is missed (not proven)

The firmware cannot currently tell how the edge is lost; with the fallback fixed, the poll counters will
show it. Candidates, most likely first:

1. **MASK_EN read races the next conversion-ready.** Reading MASK_EN clears CVRF and releases ALERT. If the
   read is delayed by about one pair period (1.96 ms, a heavier RT iteration in MPPT mode), the next CVRF
   can re-assert ALERT before the open-drain pull-up has risen past V_IH. There is then no falling edge,
   ALERT stays low, and the chip waits for a MASK_EN read that only the dead poll would do. This fits the
   correlation with control activity and the higher rate on fry's faster part.
2. **Switching noise on ALERT or I2C**, for example a pointer write corrupted so that the "MASK_EN" read
   returns another register and CVRF stays set. This fits "only while switching". No I2C or
   `failure reading register` errors are logged, though.
3. **Non-IRAM GPIO ISR masked during flash-cache-off.** Only 31 stats writes happened in the period, and
   GPIO edges latch while masked. Unlikely to be the main cause.

## Proposed fix

1. **Make the missed-edge poll work** (main fix). Pace it by time since the last INA226 sample instead of by
   a wait timeout on the shared task notification. The working tree already has exactly this as an
   uncommitted `src/adc/ina226.h` edit from 2026-10-23 (`lastAlertUs_`, poll when no sample for
   `alertTimeoutMs_`). It looks correct: the poll reads MASK_EN, which clears CVRF and re-arms ALERT, so a
   missed edge costs at most ~6 ms and one sample, with no re-init. Expected after deploy:
   `numRecovered`/`numTimeouts` climb at about the former reset rate, and stall resets drop to about 0.
   Keep the counters readable (`peek`) or log them every N, so the edge-loss rate stays observable.
2. **Calibration false positive:** treat a `Calibrating` sampler return as fresh, for example in `loopRT`:
   `if (samplerRet == UpdateRet::Calibrating && timeLastSampler) timeLastSampler = nowUs;`. Alternatively,
   refresh `timeLastSampler` when calibration completes.
3. **Commit the `ADC stall (...)` log line**, so the trigger and its arm (AdcError or stale) are visible.
4. **Refresh `timeLastSampler` after a watchdog reset**, as `applyAdcResetRt` already does, so a slow first
   sample after re-init does not cause a second reset 300 ms later.
5. **Optional, smaller blast radius:** reset only the stalled ADC (an INA226 `isGood()` that reports "no
   sample for N ms", instead of the current hard-coded `true`), leaving a healthy ESP32 DMA running. Skip
   `measurePairUs()` on a re-init (keep the boot-time `measuredPairUs_`) to cut the RT block from ~68 ms to
   a few ms.

### Guard checklist (for fix 1)

- **Unevaluable input:** a failed MASK_EN read in the poll returns `false` (no sample), and the 200 ms
  watchdog stays as the backstop. Nothing is reported as OK on failure.
- **Monotonicity:** a longer ALERT outage means more polls. The watchdog still trips at 200 ms if polls
  find nothing.
- **Preconditions:** the poll depends on `lastAlertUs_` being updated on every real sample. The edit does
  this in the `new_data` path.
- **Known-bad calibration:** on a bench board with internal ADC + INA226, mask the ALERT GPIO interrupt
  (or hold off one MASK_EN read past the next conversion) and check that `numRecovered` increments and no
  `resetPeripherals` follows.

## Validation plan (needs Fab's go-ahead; state-changing)

Build HEAD + fixes 1-4 and run on a bench board with internal ADC + INA226 (Fugu2 bench S3) under the
known-bad test above. Then confirm on fry, `-n` first, with `dc 0` before the OTA at high power. Success
criteria: `restarting continuous ADC` count about 0 per hour in MPPT, frozen Uo/I runs of 120 ms or more in
Influx about 0, and `numRecovered` growing.
