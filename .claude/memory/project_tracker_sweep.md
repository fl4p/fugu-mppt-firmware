---
name: project_tracker_sweep
description: MPPT tracker and global sweep on fry/flat — loss budget, sweep verdict/return pass, pace + c_in, boot cap-lift sweep, no-MPP backoff, rejected rev_k, scan probes, overload-limiter #70, target_duty_cycle
metadata:
  type: project
---

Tools: host sim `test/host-stub/mppt-sim.sh` (wx-* scenarios; offload big sweeps per [[feedback_agent_orchestration]]),
48 Hz Influx telemetry, havan `pv/fugu_console.log`. Tracker docs: website/docs/internals/mppt-tracker.md.

## Tracking-loss budget
Measured 11-08..14. Calm P&O loss ~0.5–0.6 % (half offset, half dither); periodic sweep 0.2–0.6 Wh each (~0.25 %). The big
loss is a wrong local peak until the next 30-min sweep: 13/49 sweeps gained >5 % (fry up to +65 %). Cause 1: limiter release
forced `resetDirection(up)` each time (sim `shade-cv` 69 → 99.7 %); fixed 3e6cbb5/59df7b7 (sub-250 ms D-only limits leave
the tracker alone), live 11-14, not field-validated (check per-sweep gain stats). Open: sim at 1470 sps with Vout cap ≤0.5 V above battery loops in sweep-no-mpp (old and new code). Cause 2: moving shade (fry ~08:45 on the 72 V peak, 57 V 17–21 % better) needs a rescan.
**Why:** daily fry/flat energy ratio swings ±20 % (different arrays, ~0.65 on sunny days); use per-window/per-sweep stats.

## Fast/slow mode and rev_k
REJECTED 11-18 (Fab): do NOT land the noise-derived reversal threshold `rev_k`; fixed rule stays. Slow mode limit-cycles in
variable light (~40 % of time, ~30 s episodes) but gains are bounded by the ~0.5 % calm loss. Continuous period fade lost in
sim (−0.25 pp). Measured dusk Iout noise floor ~8–11 mA (not current-scaled) → no k wins on both measured and old profiles.
Code parked on local branch `research/rev-k-noise-threshold` (bec9990f..789bec12); notes doc/research/2026-11-17/18-*,
docs f567f67f. Do not reopen without new low-light evidence. Fab dislikes uncommitted code: park rejected work on a branch.

## Sweep verdict and return pass
Live since 11-19 (742, then 763/770). `tracker.conf::sweep_verify` (0.10): landed vs MPP power beyond the fraction flags
the sweep (`sweep-curve` `flag=light`, dashboard dims it). Return pass: the way back up probes every recorded maximum (≥2)
and lands on the best. 60 s re-sweep only for flagged + ≤1 maximum + unknown n (5 s if landing brighter; cap 1/10 min; a
manual sweep counts toward the cap). Two consecutive verified single-max sweeps learn `pv_substrings=1`.
770 also: NTC excluded from despike, no start while a filter is empty (killed re-sweep ctrl-nan), stopAndBackoff ends a sweep.
Open (Fab): retry under a still-changing sky on single-peak strings costs sim harvest (−1.9/−3.0 pp).

## Sweep pace, c_in, landing trim
Live 11-18 (a5f852f6, 1170e02c). Vin-floor crossing ends the sweep (fry 9.3 s from up to 39 s). `sweep_pace` 4; sweep
subtracts input-cap discharge `c_in·V·dV/dt` (`converter.conf::c_in` 1e-3; at dusk ~3 W fake peak) and trims the landing
onto the captured Vin. 11-19: sweeps 35–40 % shorter, dusk landings fixed for fry (n=1). wx-sweep-rise −2.5 pp accepted by
Fab (expected to FAIL); idle-wake fails since before. Bisect knobs: `sweep_pace 1`, `c_in 0`. CV-pinned sweeps when
terminated are slow and meaningless; skip-sweep fix DECLINED by Fab. fry: +10–15 % power bump at DCM→CCM (duty ~3100–3430).
4258f880 (11-22): Tracker::restart on PsuOff→Mppt (stale tracker after `dc`→`mppt`); last curve kept across aborts. Open: scan Vin reference taken 5 s after a manual hand-back (wx-sweep-scan SIM_ACT=manual timing shifts).

## Boot sweep and cap lift
VALIDATED 11-24 (929). Boot sweep runs under the provisional 26.80 V no-BMS cap → 0 W when the pack is higher. 87306a01:
re-sweep when the cap lifts; a26a3274 + 4012dc42: cap lift clears the no-MPP backoff and starts a sweep at once (flat first
MPP at 20.6 s uptime vs ~46 s). Log: `Vout cap 26.80 V lifted to 29.00 V, no-MPP backoff cleared` → `Start sweep`.
Older lesson (06-09): a cold boot into marginal dawn light captured a garbage MPP at near-max duty (1.3 W @4010) and flat
stuck at duty 1/CV until `restart`; avoid OTA reboots near dusk.

## No-MPP backoff
Live since 11-21 (a90529d5). No-MPP backoff doubles 30..300 s, re-sweep cap, battery return clears it; `ota` works without
the RT loop (ota.cpp timing via esp_timer). Bench: vconv `bat open` does not drop Vout with converter off; use `bat short`
then `bat N`.

## Peak-scan probes
- `tracker.conf::scan_near_v` (2.0 V, log-only `scan near:` line), live as 912 on 11-23 to test flat sitting 1.5–5 V below
  Vmpp at ≥100 W (31 % of bins). 11-24 Codex review: it cannot see 1.5–3 V offsets and never runs ≥300 W (scan skipped when
  nc==0); v2 (bracketed ref, near-only run) being built, note reopened 28206d64.
- Share estimate overestimates from an all-conducting shaded maximum (docs model: 10.4 vs 9.7 V; probe lands ~6 % high,
  `kProbeLow 0.97` in peak_scan.h) → may hide a lower peak <~6 % better. Option: trust 0.82·Voc/n alone when m0==n. Needs a
  sim sweep over shading cases first.

## Overload limiter wrong side (#70)
Deliberately NOT fixed (Fab 11-17, re-confirmed 11-24). Below Vmpp, IoutCTRL/PowerCTRL lowering duty raises power →
`Iout-OC` trip, 30 s backoff, sweep recovers. Sim reproduces (7fef64ae, `wx-ovl-shade*`, not in default list); field: 0
trips. **Why:** fix needs a noisy slope sign under limiting. **How to apply:** don't fix unasked; revisit if Iout-OC shows
in logs. Then: sim → reverse/sweep in MpptController::update() → regress wx-* → bench → Fab's OK.

## target_duty_cycle
2026-08-10 (35f83ec0). `tracker.conf::target_duty_cycle` = fraction of pwmMaxDriver; non-zero → manualPwm at boot with
persistent `manualTarget` (cleared only by `dc 0`), startSweep skipped. `clearBootTarget()` on `mppt`/`sweep`/ota-fail/
measure-coil. fboost 2499 counts → 0.6138 ([[reference_bench_rigs]]).
