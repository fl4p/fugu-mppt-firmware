*this document is an LLM generated placeholder*

# CV hold current regulation

Issue [#72](https://github.com/fl4p/fugu-mppt-firmware/issues/72) concerns CV-hold
simulation failures that depend on BMS sampling phase. The final candidate changes
only the output-current controller's error reference. Charger logic, PWM timing,
configured gains, protections and simulation thresholds stay unchanged.

## Current-error reference

The old controller used `1 - Iout / smoothed_Iout_max`. Its response per ampere of
error increased as the charger reduced its current limit: at 4 A it was ten times
the response at 40 A. In the investigated hold this produced repeated current
dips and a mean current below the requested limit.

The candidate uses `(smoothed_Iout_max - Iout) / 40 A`. Both inputs are divided by
the named `IoutCtrlRefA`, and the smoothed controller disables relative
normalization. Its optional constructor argument defaults to the old behavior
for all other callers. The 40 A value is a fixed control-gain reference, independent
of battery configuration.

At a constant, settled 40 A target the old and new laws agree. At a changing
target their derivative terms differ; the new law differentiates the scaled
ampere error. The unit regression checks that moving-target arithmetic explicitly.
The smaller response at low limits can also change which competing limiter wins.

## Diagnostic evidence

Two contributors were separated in scratch experiments: a fast current pulse
when an LS-window clamp released the simulated gate driver's HS interlock delay,
and current-limiter chatter superimposed on a depressed mean. Reserving an LH
gap removed the high pulse but left a low excursion. Reducing Iout-controller
gain removed the latter in the tested fixture.

The fixed reference passes the two initial cases with the original charger and
normal PWM timing, so neither the experimental LH reservation nor the earlier
charger gain-scheduling candidate is included.

| Mac `cv-hold-pair` | Hold band (V) | Entry peak (V) | Verdict |
|---|---|---|---|
| Unmodified `main` `2fc28a71`, phase 1 | 3.492..3.510 | 3.507 | FAIL |
| Fixed reference, original charger, default phase | 3.500..3.502 | 3.505 | PASS |
| Fixed reference, original charger, phase 1 | 3.499..3.501 | 3.507 | PASS |

Printed voltages are rounded to 1 mV; the baseline's own assertion reports a
failure, so its printed 3.510 V is not evidence of equality to the limit.
The baseline above was rebuilt on current `main`; initial candidate results came
from the earlier scratch checkout. Final validation uses the candidate on current
`main`.

Local evidence: `/tmp/cv72-current-main-baseline-phase1.log`,
`/tmp/cv72-lh/RESULTS.md`, and `/tmp/cv72-lh/logs/cand_preA_lh0_{default,phase1}.log`.

## Validation

- The new host regression checks equal ampere-error gain at 1/4/40 A, exact
  smoothed-target and derivative responses, zero-target behavior, default relative
  behavior for other callers, and the sign used by the limiter's past-limit logic.
- Forcing the new constructor to ignore `normalize=false` in a scratch copy
  makes five regression checks fail. The unit test does not instantiate MPPT;
  its fixed reference is repeated locally, so integration needs simulator coverage.
- Worker host suite: 38 PASS, one existing XFAIL (`bms-cutoff-test`).
- Pending on the final candidate: 26 phase/lag cases, all stock MPPT scenarios,
  Linux host units, ESP-IDF build and independent final review.

No test threshold was widened. Stock simulator cases include report-only cases;
completion of the suite does not assert every reported statistic or every limiter
selection transition. No hardware run or output-waveform measurement has been
made. Issue #72 remains open pending final validation.
