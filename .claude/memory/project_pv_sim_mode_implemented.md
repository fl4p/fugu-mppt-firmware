---
name: pv-sim-mode-implemented
description: "PV-sim (solar-array-simulator) output mode implemented 2026-08-15, commit 2440421; bench validation on fboost->fbuck still pending"
metadata: 
  node_type: memory
  type: project
  originSessionId: 46c7e808-20eb-4e70-8a46-f280e1c16d5a
  modified: 2026-08-15T06:27:10.133Z
---

PV-sim mode (converter.conf `mode=pv`, console `pv <isc> <voc> [k]` / `pv scale` / `pv off`)
implemented and committed 2026-08-15 (2440421), plan in `plans/pv-sim-mode.md`. Runs on the
PSU machinery: per-tick Vset = PvModel::voltage(Iout_ewma) in `pvAdvanceSetpoint`
(mppt.cpp), clamped [Vin+0.5, min(Voc, vout_max)], slew-limited (pv_slew, dt clamped 10 ms).
OV threshold / feasibility / ovset pinned at Voc; Iout limiter capped 1.1·Isc; profile
`config/lab/fboost_pv`. Codex review found 4 blockers pre-implementation (Isc not enforced,
setpoint jump on curve re-issue, ovset vs moving setpoint latch, vconv.cpp voc_ refs) — all
fixed; boot mode=psu/pv now calibrates explicitly instead of relying on the fallback sweep.

**Not yet done:** bench validation (plan step 7: ovset 70, `pv 2 60 0.8`, curve walk, fbuck
MPPT converge at Vmp=48 V) — on 2026-08-15 the only attached board (usbmodem1101, likely flu)
was lock-held by another session's Coss sweep. On-target `RUN_TESTS` suite compiled
(test/test_pv_sim.cpp) but has not run on hardware.

**Main open risk:** virtual-Iout coupling (Iout = Iin·Vin/Vout·eff on fboost) can ring near
the curve knee — bench knobs are `pv_slew` down / `pv_iout_span` up. See
[[fboost-rig-fixed-duty-2499]]: PV-sim conflicts with the pwr-metering rig's locked dc 2499;
use runtime `pv` entry or the separate fboost_pv profile, never repurpose the fboost profile.
