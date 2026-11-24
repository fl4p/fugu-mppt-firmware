---
name: project_ripple_filter
description: ADC ripple filtering: notch before median (rule), inverter-fed bus needs Vout notch, adaptive Goertzel notch (live fry+flat), ripple comb #37 trial on fry, despike=8 on fry+flat
metadata:
  type: project
---

## Notch before median (rule)
Chain in `Sensor::add_sample`: raw -> notch -> RunningMedian5 -> EWM -> AdaptiveNoiseFilter. Don't suggest "median first".
- **Why:** fs ~400 Hz vs 100 Hz interferer (fs/f0 = 4); a 5-tap median spans 1.25 periods, scatters the tone into harmonics the notch then can't cancel. "Median first" assumes fs/f0 ≫ 10.
- **How to apply:** keep the notch ahead of the median; check fs and the interferer before proposing any reordering.

## Inverter-fed topology
Vout can sit on an inverter DC bus: at kW, ~1 mF bulk is ZC ≈ 1.6 Ω at 100 Hz, so volts of ripple; the battery is the only low-Z sink. Without the notch the CV loop chases 100 Hz. **How to apply:** never suggest removing the Vout/Vin notch; off-grid inverters drift 1-2 %, so a fixed Q=20 notch may miss.

## Adaptive notch
2026-06-06/07, a99aec9 + 6164333 + 195fdc9, live on fry and flat. A 2 kW inverter collapsed fry from ~500 W to ~130 W (tracker parked near Voc): the fixed 100 Hz notch missed the tone and P&O drowned. Fix: `src/math/ripple_freq.h` streaming Goertzel (80-140 Hz, SNR ≥15, slewed) on the RT core retunes all notches. `sensor.conf`: `notch_adaptive` (1), `notch_freq` (100), `notch_q` (20). Tests `test/host-stub/ripple-freq-test.cpp`, `test/test_ripple_freq.cpp`, fixture `test/data/fry_vout_inverter_646.h`.
- Live: fry notch settled at ~89 Hz labelled, flat ~103-107 Hz; both are the real ~100 Hz. fry's label was off because the INA226 nominal rate (454.5 sps) was wrong (real ~512); notch is unaffected since fs cancels. `CONFIG_FUGU_INA226_MEASURED_RATE` (default y since 195fdc9) measures the real rate. Scope absolute Hz is unreliable (estimated rate).
- vconv A/B (495d317 fixed a gptimer prescaler overflow below ~2.5 kHz adc_freq): mistuned notch 570 W @ 72 V vs tuned 809 W @ Vmp (baseline 815 W). Tracker power uses notched physical sensors (virtual-Iin worry was wrong).
- Inverter ripple is a near-pure sine at 2× line (~0.6 % RMS, h2 0.7 %), not |sin|; vconv `vbat_ac_shape` 0/1, `vbat_ac_amp=0.26` fry profile (4ad1c85). See [[project_vconv_builds]].

## Ripple comb (#37)
2026-11-24, on main (b492e5e7..93946687), default `ripple_filter=notch`. `sensor.conf ripple_filter=notch|comb|both`, `notch_depth` (-15 dB default; a deeper notch hurts at the Vout cap). Sim at cap: comb/both 94-97 % vs notch 33-80 %; costs 1.5-7 % with spiky current and no cap. Review doc/reviews/2026-11-24-ripple-comb-codex-review.md.
- **11-24 14:13: fry on fry-brk1-964-g195073d3** (a52c7d7c + comb commits, branch `fugu37-on-a52c`, not main) with `ripple_filter=both` since 11-24 14:13 UTC; flat is the control. Superseded ~15:30 by the fleet OTA to 1009-g44e16128 which does NOT contain the comb commits (b492e5e7..93946687 not ancestors of 44e16128) — the fry trial is suspended until a comb image is flashed again. **How to apply:** compare fry vs flat harvest/trips over the next days.
- notch+despike=8 (flat's config) failed 10 hs-* sim scenarios: start gates read Vin EWM, limiter reads med3; despike on a dead-quiet Vin passes a step only to the EWM -> start + instant "no MPP" 30 s backoff. Fix 62d3e49c on main (`vinStart()=fmin(ewm,med3)`), not flashed. Open for Fab: despike clips nothing while `despikeScale==0` (contradicts sensor.md).

## despike=8 on fry+flat
2026-11-18 14:38 UTC, Fab ok'd; needs calibration noise-gate fix 670a3ef9 (else every zero calibration fails). Sim +0.15 pp mean, +0.63 pp low light (doc/research/2026-11-18-filter-chain-noise-study.md). WARN-spam fix 7c80224c (4-step floor, WARN only >0.5 %/window) on both since 11-19 as `fleet-20261119-despike`. Revert: `set-config sensor.conf despike 0` + restart.
- fry still WARNs ~0.5-0.7 % clipped, mostly ntc (esp32adc1, averaged, so the floor doesn't engage). Resolved: 7f049d96 excludes temperature channels from despike (deployed 11-18, WARN gone; `src/adc/sampling.h`). Related [[project_tracker_sweep]].
