---
name: Diode-emulation tests TODO
description: Diode-emulation unit tests status; boot_refresh_ns was silently regressed 1500→500 ns and the test was tautological — both found/fixed 2026-08-11, committed 452761c6 (final default 2000 ns)
created: 2026-06-15T12:00:00.000Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_00fc47d03ffeOjgi2pCRIFkyJz
---

# Diode-emulation tests TODO

After the 2026-05-21 diode-emulation review of `src/buck.h`, we still owe unit tests for the edge cases.

**Why:** the fixes were applied without test coverage; these are sensor-noise / boundary conditions that are hard to hit on hardware but easy to assert in a unit test.

**How to apply:** add to `test/` (Unity, on-target or host-stub). Cover at least:
- **convRatioWCE clamp (the fix):** buck at M≈0.99 and the `vout>=vin` fallback (and boost equivalents) must NOT make `rectCtrlRatio()`/`pwmRectRatioDCM` negative. Assert `pwmRectRatioDCM >= 0` and that DCM `pwmRectMax` does not wrap to the full CCM complement.
- **DCM/CCM decision + hysteresis:** `computeDCM` band [1.8, 2.0]·il, plus the `il < 0.1` force-DCM path.
- **rectCtrlRatio** matches doc eqs: buck `1/M-1`, boost `1/(M-1)`.
- **boot_refresh_ns → pwmRectMin count conversion:** `pwmRectMin == ceil(ns·1e-9·fsw·pwmMax)`; default 2000 ns at 39 kHz; boost gives `pwmRectMin == 0`.

Status: #1 convRatioWCE clamp DONE; cheap part of #2 DONE (`+N`/`-N` routed through `setTargetDutyCycle`, RT core sole PWM writer); `boot_refresh_ns` conf DONE (was hardcoded `MinDutyCycleLS=0.06`); magic constants named DONE.

Tests WRITTEN (in `test/test_buck.cpp`, registered in `test/main.cpp`): ratio clamp below/at unity, never-negative sweep, DCM rectMax not full-CCM near unity, sync-rect-off thresholds, sync-rect active in DCM, boot_refresh_ns→pwmRectMin (default/scaled/boost=0), boost ratio clamp. Compile-verified via `RUN_TESTS=1 idf.py build`. NOT yet run on-target — needs a bench device ($ESPPORT); the suite actuates LEDC/gate pins so use a safe board, not a live charger.

**boot_refresh_ns default regression (found+fixed 2026-08-11, committed 452761c6):** an unrelated cleanup commit ("remove tele bench", 434efa14) silently cut the `boot_refresh_ns` default in `src/buck.h` from 1500 to 500 ns — a 3x reduction in HS bootstrap refresh (80 counts, ~2% duty instead of the intended ~6% at 39 kHz / 4069-count MCPWM). MEASURED on fbuck with output open: DCM(H|L|Lm)=1506|80|80, LS on-time pinned at the 80-count minimum; scope showed a clean hard turn-off but NO fast turn-on — just a slow resonant ramp, which is what a starved HS gate drive looks like. With no load the LS body diode never conducts, so `pwmRectMin` is the ONLY thing recharging the bootstrap cap.

The final default is **2000 ns** (not 1500): 1500 merely restored the legacy 6% (~244 counts), but Fab established 300 counts as the measured safe floor on fbuck. 2000 ns gives 320 counts at 39.3 kHz / 4069-count MCPWM. Committed in **452761c6** (`buck.h:570`).

The test `test_buck_bootstrap_min_default` was **tautological** — it recomputed the expectation with the same `500e-9` constant the code used, so it compared the default against itself and held green while the default sat at a third of the required refresh. Fixed in 452761c6: the test now asserts against `0.06 * pwmMaxDriver` (the legacy `MinDutyCycleLS` intent), independent of whatever ns default is chosen, with a 5% tolerance. This catches drift in either direction.

Find the regression with `git log -S "boot_refresh_ns\", 500" -- src/buck.h`.

Still-open review findings (not yet fixed, may want tests too): full single-writer for #2 (`setManualRect`/`enableSyncRect` still actuate PWM from core 0 in manual mode), and the throw in `pwmPerturbFractional` on the RT path. See [[project_perf_h_noninline_odr]] for the single-TU header constraint when adding test files.
