---
name: project_backflow_wire_guard
description: Backflow-open high-duty trip (accepted join-at-float nuisance, low-light fix) and the #47 wire-fault guard (warn-only, Rc/Ro fits, RTC-pinned learning state)
metadata:
  type: project
---

## highD with open backflow: accepted nuisance
Fab 2026-11-04: "just leave it". Since aebc8e4 (terminated sibling joins at float, 10-31) fry/flat log `E MPPT: High duty
cycle with open backflow switch!` + 5 s backoff a few times/day: joining at 26.8 V gives high-duty/~0 A states, bflow opens
below 0.05 A and the guard (mppt.h) trips.
**Why:** guard is protective, cost is only the backoff. **How to apply:** don't report as a regression or propose a fix
unprompted (a duty-ramp-before-bflow-open idea needs a bench test).

## highD at low light
FIXED and live 11-11 (3e0539c). fry tripped 1361×/day on a dim day: at 0.05–0.1 A, Vout/Vin ~0.8, DCM duty >0.33 with bflow
open. 944f541/19c9409: trip only if `dcmCurrent()` = D²·Vin·(Vin−Vout)/(2·fL·Vout) is not <1 A (NAN when D ≥ 0.9·M).
0b5c23b: a sweep reaching the ratio limit with I<0.05 A ends via `_stopSweep` (commit MPP or 30 s sweep-no-mpp backoff).
3e0539c fixed 0b5c23b's protect()→stopAndBackoff(4) dropping the MPP. Host tests dcm-current-test.cpp, sim test/host-stub/sim/.

## Wire-fault guard (#47)
Warn only (`limits.conf wire_protect=1`) since 11-12 (1b96f1f). `src/wire_guard.h`, plan `plans/wire-fault-protection.md`;
telemetry `wf`/`Pw`, retained `pv/alert/<host>/wire`, console `wire`/`wire-reset`, baseline in NVS blob "wire". Replay
tools: etc/wire_replay_fetch.py + test/host-stub/wire-guard-replay.cpp.
- Fits: Rc/Rb (BMS wire fit) work on fry (Rc ~14–18 mΩ, Rb 1.3–1.9). Tracker-step Ro is NOT viable (1-count step ≈ 8 mA,
  SE 226 mΩ fry / 1160 flat) — don't revive; replaced by per-sweep Vout-vs-Iout fit (5edefd4: span ≥3 A, res ≤40 mV, SE ≤3 mΩ,
  fry ~21–24 mΩ), graded as median of last 5 sweeps (e4cb0616). flat publishes nothing: missing Vout RC filter
  ([[project_charger_bms_feed]]).
- Fixes live: converter-vs-BMS offset subtraction, ΔR≥8 mΩ day warning, `unverified` without data (d75e236b); open day in
  RTC, learning accepts all good days, day quality gate ≥4 slow-fit samples at Iout≥5 A, ≥300 BMS frames, Iout var ≥4 A²
  (53906f0b, fixed a fry false warn from a 1-day 3.13 mΩ baseline after 16 OTAs); grade only vs ≥3 stored days (9c2ddeaa);
  offset regression in RTC (6f35af23).
- RTC layout: any new RTC_NOINIT var shifted the wire state (804 lost the open day). Now wire state and charger RTC pinned
  (main/rtc_pin.ld: charger 0x50001D00, wire 0x50001E00); retention verified across OTAs 11-23 (881). One-time migration from
  0x500000f4 (builds 804..864) is a TODO to drop.
- 11-23: fry 1 counted Rc day (17.74 mΩ), Ro ≥3 days (20.13); flat 0 days. Dim November days rarely pass the quality gate.
**Why:** November currents (11–19 A) make the excess-power criterion slow, and offset drift risks nuisance derates in summer.
**How to apply:** don't set `wire_protect=2` before a current-independent ΔR warning and warn-only observation over
high-current days. Fab accepted that a shared-section (Rb) fault can derate via the fast path.
