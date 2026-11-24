---
name: project_rectification
description: Sync-rect timing: rect_offset_ns per board (fry -325 ns via rect-cal, flat 0), flat MCPWM sweep loop, intrinsic L-vs-H oscillation, diode-emulation tests and boot_refresh_ns 2000 ns
metadata:
  type: project
---

## rect-cal field results
2026-11-24. fry: `rect-cal` succeeded 11-21 (69 W DCM, charging): zc 32 ct before ideal, applied -52 ct = **-325 ns** (replaced a wrong -1312 ns); hand-written to coil.conf 11-22, live since the 11-23 OTA. Earlier runs failed on float (`limiting`), forced CCM (`Iout follows LS`), light ramps (`V step`). flat: 11-24 failed `sense follows LS` (Vout reading -0.51 % vs ±0.20 %), same root as flat's low Vout reading ([[project_charger_bms_feed]]). Restart/OTA now saves a pending result (87c82e29, 181c04b4).
- 11-15 history: fry's -1312 ns (-210 ct) was stored by hand from a 113 W run (learned -162; a repeat on fixed code learned -142). flat's 11-15 rect-cal already failed: Vout +1.3 % and Iout +60 mA rose steadily with LS on-time (switching-dependent INA226 error, not real current); needs a HW look at the sense filter.
- **Why:** rect-cal needs a steady DCM point (0.3-4 A, charging, P spread ≤2.5 %/60 s). Records: doc/research/2026-11-21-fry-rectcal-runs.md, 2026-11-24-flat-rectcal.md.
- **How to apply:** Fab: flat stays at rect_offset 0; don't rerun rect-cal on flat until its Vout sensing is fixed (RC filter or sense fix). Check fry light-load efficiency vs pre-11-23 from ~11-26 (Influx).

## flat MCPWM sweep loop
2026-11-09, mitigated (offset 875 -> 0 ns at 21:39, Fab approved); firmware fix deferred. After the MCPWM move every 30-min midday re-sweep on flat looped 4-12 min (~54 min at 0 W, ~25 % of midday yield): in DCM near D=Vout/Vin current collapsed to negative, bflow opened, `highD-bflowOpen` tripped ([[project_backflow_wire_guard]]). Hypothesis: `rect_offset_ns=875` was LEDC-calibrated incl. dead time; on MCPWM the DCM LS window hit the period-H-1 clamp below D=M (forced CCM with D·Vin < Vout). Host repro (/tmp, not committed): ideal plant never trips; trips once LS extension reaches ~220 ct with flat's 0.4 V low Vout reading — i.e. needs ~80 ct (~0.5 µs) of unexplained real LS extension beyond the 140 ct offset; offset 0 leaves ~140 ct margin.
- **How to apply:** Fab wants a later firmware fix: keep the DCM LS window from reaching forced CCM below D=M, and/or recalibrate rect_offset_ns per driver.

## Legacy count offsets and intrinsic oscillation
2026-06-07, superseded values. Old count keys (fry 100, flat 78) were from `measure-coil ls <hs>` on LEDC; now ns in `coil.conf::rect_offset_ns`, count key ignored. Measuring lesson: use a steep-edge HS (~500 on flat); auto-HS (~244) buries the Iout peak in noise. The robust quantity is the offset (peak−ideal), not absolute LS. After `measure-coil ... apply` a live converter can sit at 0 W until a manual `sweep`.
- The L-vs-H oscillation (~100-115 ct period, ±10-25 %) is **intrinsic**, not WiFi (wifi on/off A/B identical). The N↔N+1 LS dither can't fix it: Iout-vs-LS is asymmetric (body-diode plateau vs reverse-current cliff), so averaging gives a biased Iout (Jensen); a positive rect_offset moves toward the cliff and may worsen it. A fix likely needs asymmetric/stochastic LS dithering; test on `config/lab/dry_mock`.

## Diode-emulation tests
Written in `test/test_buck.cpp` (ratio clamp near/at unity, never-negative sweep, DCM rectMax not full CCM near unity, sync-rect thresholds, boot_refresh_ns->pwmRectMin incl. boost=0); compile-verified, **not run on target** (needs a safe bench board, the suite actuates gates).
- **boot_refresh_ns** regression: 434efa14 cut it 1500 -> 500 ns; fbuck with open output showed a starved HS gate (slow resonant turn-on; with no load pwmRectMin is the only bootstrap recharge). Default now **2000 ns** (320 ct, above Fab's 300-ct floor), 452761c6. The old test was tautological (same constant); it now asserts against `0.06 * pwmMaxDriver` ±5 %. Find with `git log -S "boot_refresh_ns\", 500" -- src/buck.h`.
- Still open: full single-writer (`setManualRect`/`enableSyncRect` actuate PWM from core 0 in manual mode); throw in `pwmPerturbFractional` on the RT path. Related [[project_mcpwm]], [[project_build_toolchain]] (perf.h single-TU).
