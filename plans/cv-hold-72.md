*this document is an LLM generated placeholder*

# CV hold current regulation

Issue [#72](https://github.com/fl4p/fugu-mppt-firmware/issues/72) concerns CV-hold
simulation failures that depend on BMS sampling phase. The candidate changes the
output-current controller's error reference and reduces the CV hold's per-frame
battery-current correction. Forecast logic, PWM timing, configured converter PD
gains, protections and simulation thresholds stay unchanged.

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
normal PWM timing. Neither the experimental LH reservation nor the earlier
forecast gain-scheduling candidate is included.

| Mac `cv-hold-pair` | Hold band (V) | Entry peak (V) | Verdict |
|---|---|---|---|
| Unmodified `main` `2fc28a71`, phase 1 | 3.492..3.510 | 3.507 | FAIL |
| Fixed reference, original charger, default phase | 3.500..3.502 | 3.505 | PASS |
| Fixed reference, original charger, phase 1 | 3.499..3.501 | 3.507 | PASS |

Re-running phase 1 on final candidate `f0ebdf43`, based on current `main`
`2fc28a71`, also passes at 3.499..3.501 V with a 3.507 V entry peak. Its log is
`/tmp/cv72-final-candidate-phase1.log`.

Printed voltages are rounded to 1 mV; the baseline's own assertion reports a
failure, so its printed 3.510 V is not evidence of equality to the limit.
The baseline above was rebuilt on current `main`; initial candidate results came
from the earlier scratch checkout. Final validation uses the candidate on current
`main`.

Local evidence: `/tmp/cv72-current-main-baseline-phase1.log`,
`/tmp/cv72-lh/RESULTS.md`, and `/tmp/cv72-lh/logs/cand_preA_lh0_{default,phase1}.log`.

## Delayed battery-current feedback

The larger timing grid found residual failures with `lagI=1`: the BMS sends the
previous current frame's value, four seconds old in this fixture, while cell
voltage is current. Arrival timestamps do not expose that sample age.

At cell phase 4 s, the fixed-reference candidate reaches 3.522028 V at 210.1279 s.
Both converters closely track their commanded limits at that peak. Excursions
above 3.515 V last about nine seconds and recur about 24 seconds apart. This is
different from the earlier millisecond converter pulse.

The paired diagnostic changes only the battery-current correction `G` from 0.4
to 0.2. The delayed case then passes at 3.499..3.501 V with a 3.510 V entry peak;
the default case also passes, at 3.498..3.503 V with a 3.508 V entry peak. The
roughly 24-second recurrence disappears. These two cases support a candidate,
not acceptance of the full timing grid.

For two converters with ideal current tracking, a fixed target and inactive
caps, a one-frame delay gives `x[k+1] = x[k] + 2G*(target - x[k-1])`. The pole
magnitude falls from about 0.894 to 0.632 when G is halved. This explains why the
old stability condition `2G < 1` did not guarantee adequate damping. It is an
approximation, not a stability proof of the coupled cell loop, converter dynamics,
caps or asynchronous updates. The remaining slow excursion is not explained by
that inner-loop model.

The production candidate retains the fixed reference and uses G = 0.2. A new
stock `cv-hold-pair-lag` case fixes cell phase at 4 s and delays current by one
frame, sharing the original pair's entry, band and latch assertions. Its empty
steady-window check rejects a run with no samples after `holdAt + 300 s`.

Diagnostic logs and traces: `/tmp/fugu-cv72-lag/`, including `lag_pC4.csv`,
`g02_pC4.log` and `build_g02.log`. No instrumented getters or trace hooks are
included in the production candidate.

## Validation

- The new host regression checks equal ampere-error gain at 1/4/40 A, exact
  smoothed-target and derivative responses, zero-target behavior, default relative
  behavior for other callers, and the sign used by the limiter's past-limit logic.
- Forcing the new constructor to ignore `normalize=false` in a scratch copy
  makes five regression checks fail. The unit test does not instantiate MPPT;
  its fixed reference is repeated locally, so integration needs simulator coverage.
- Worker host suite: 38 PASS, one existing XFAIL (`bms-cutoff-test`).
- Independent review of `f0ebdf43` found no blockers and reproduced the five
  failures under the constructor mutation.
- At `f0ebdf43`, the ESP-IDF 6.0.3 ESP32-S3 build passed. The image is `0x1b8060` bytes with 4% of
  the smallest application partition free. No cycle-level cost measurement was
  made; the phase-1 simulation reports zero allocations and frees in its RT loop.
- The documentation build with a fresh cache and the cross-link check passed.
- Linux [stock suite](https://github.com/fl4p/fugu-mppt-firmware/actions/runs/46038412150):
  186 simulator scenarios completed without failures; host units 38 PASS and one
  existing XFAIL. The separately pinned
  [unit run](https://github.com/fl4p/fugu-mppt-firmware/actions/runs/46038398604)
  confirms the same counts.
- Linux [26-case phase/lag grid](https://github.com/fl4p/fugu-mppt-firmware/actions/runs/46038398925):
  20 PASS, six FAIL. All 13 cases with no extra current-frame delay pass. With
  `lagI=1` (one four-second-old current frame, not a one-second delay), phase 1
  fails low at 3.486 V; phases 3, 4, 5, 7 and 8 exceed the entry limit, peaking at
  3.521–3.524 V. These runs use Ubuntu 26.04, Clang 21.1.8, x86_64, glibc 2.43,
  image `20261122.9.1`, and candidate `f0ebdf43`.
- macOS at `f0ebdf43`: 23 PASS, three FAIL (delayed-current phases 3, 4 and 5).
  One case reused the root's run with captured exit 0; the worker ran the other
  25. A corresponding baseline grid has not been run, so the residual failures
  have not been classified as pre-existing or introduced.
- The G = 0.2 candidate passes the default pair and new delayed pair locally.
  Restoring G = 0.4 in an isolated copy makes the same delayed-pair harness fail
  at a 3.522 V entry peak. The focused charger and PD host tests pass.
- After merging `main` at `d0cf88d8`, candidate `cbbb692b` passes the delayed
  pair with entry 3.510 V and band 3.499..3.501 V. The merged G = 0.4 calibration
  still fails at entry 3.522 V; all three exact host-step assertions also fail
  under that mutation. The shortened run fails the empty-window guard.
- The merged ESP32-S3 build passes (`0x1b8210` bytes, 4% partition space free),
  as do the fresh-cache docs build, rendered-page check and cross-link check.
  Both 26-case grids, the 190-case stock suite and Linux host units remain pending.

### Empty-window guard review

The pair assertions now require `lo <= hi` before accepting the steady band.
The unobserved window retains the reversed sentinels (9, 0), which fail this
check; NaN bounds also fail the comparison. This is a nonempty-window check,
not validation of every plant sample. Adding this requirement cannot convert an
existing failure to a pass, and the voltage limits remain unchanged.

Both pair scenarios invoke it. A shortened `SIM_T=300` delayed-pair run fails
with `no steady hold samples`; a full run passes. There is no result cache.
Final validation records the candidate revision, including the simulator and
BMS producer, alongside the controller; diagnostic and reused logs remain
identified separately. The guard exposes missing evidence rather than relaxing
the voltage assertions.

The host benchmark could not resolve its incremental cost above noise (largest
observed delta 0.097 ns/iteration, not a latency upper bound). The verified pass
path is a comparison and conditional branch, executed once at simulator shutdown;
it adds no firmware RT work. Evidence: `/tmp/cv72-needbench/README.md`.

No test threshold was widened. Stock simulator cases include report-only cases;
completion of the suite does not assert every reported statistic or every limiter
selection transition. No hardware run or output-waveform measurement has been
made. Issue #72 remains open; the candidate is not ready to land on `main`.
