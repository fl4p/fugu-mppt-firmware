---
name: project_psu_pv_modes
description: PSU constant-voltage mode (vconv-validated, real-HW pending) and PV-sim output mode (2440421, bench pending); entry keys, HW fixes, risks
metadata:
  type: project
---

## PSU mode
2026-08-12, vconv-validated; real-HW validation pending. `OpMode::Psu` alongside MPPT/Manual; limiter chain regulates Vout to `psuVsetpoint` with CV/CC foldback, no tracker/sweep/charger. Entry `converter.conf mode=psu` + `psu_vout`, or `psu <V>`; exit `psu off`, `mppt`, `sweep`, `dc N`. Trips: 100 ms auto-retry only for ≤5 s backoffs from transient Vout-OV/supply-UV (sensor-fail, revI, highI keep normal backoff; `src/mppt.h` psuFastRetry), escalation after 5, latch after 8 in 60 s. Plan plans/psu-mode.md, tests test/test_psu_mode.cpp; 11 codex findings fixed (±N rejected, OTA/measure-coil restore opMode, opMode written before setpoint on exit, ...).
- Real-HW fixes: bflow enabled in PSU (else `highD-bflowOpen` trips), boost skips the `Vin < Vout+1` start check, Vr-sensor-fail skipped while Vout < setpoint, boost `pwmCtrlMax` capped 90 %. ADC atten issue -> [[project_adc_sampler]].
- Bench S3 (fmetal) had a broken gate-driver supply (no power transfer), so no real-HW result. Profile `config/psu/boost80V/`.
- **Why:** boost supply for an audio PA; MPPT on a stiff battery gives sub-30 Hz rail wander and 30-min re-sweeps.
- **How to apply:** don't OTA to live converters before on-target checks (plan items 5-11) with a resistive dummy load on a working board: rail reaches setpoint, no sweep after 45+ min, CC foldback, OV escalation.

## PV-sim mode
2026-08-15, 2440421; bench validation pending. `mode=pv`, `pv <isc> <voc> [k]` / `pv scale` / `pv off` on PSU machinery: per-tick Vset = PvModel(Iout_ewma), clamped [Vin+0.5, min(Voc, vout_max)], slewed (`pv_slew`); OV/ovset pinned at Voc, Iout cap 1.1·Isc. Profile `config/lab/boost_pv`, plan plans/pv-sim-mode.md, test/test_pv_sim.cpp (not run on HW).
- Pending: plan step 7 (ovset 70, `pv 2 60 0.8`, fbuck converges at Vmp 48 V).
- Risk: virtual Iout on fboost can ring near the knee (knobs `pv_slew` down, `pv_iout_span` up). Never repurpose the boost_bench profile; it conflicts with the rig's fixed dc 2499 ([[reference_bench_rigs]]).
