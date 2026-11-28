*this document is an LLM generated placeholder*

# CV hold current regulation

Validated source `795f402e33d35784ce8e6325c15a05a333be7e13` addresses
[issue #72](https://github.com/fl4p/fugu-mppt-firmware/issues/72) with two control
changes. The full Linux stock suite and both platform phase grids pass.

## Control changes

The output-current PD controller uses a fixed 40 A error reference:
`e = (smoothed_target - measured_current) / 40 A`. Previously it divided by the
smoothed current target, increasing its response to an ampere of error as the
charger reduced that target. At a 4 A hold the old proportional gain was ten
times its value at 40 A. In the investigated simulation this contributed to
current ripple and a mean current below the commanded limit.

The 40 A reference preserves the old gain at a **settled** 40 A target. It is
not a hardware maximum or a battery-current setting. Moving-target derivative
behavior differs; the regression test checks that sequence explicitly. Other
controllers retain their existing relative-error normalization through the
constructor's default argument.

The CV hold's per-current-frame correction changes from `G = 0.4` to `G = 0.25`.
The simulated BMS can deliver the previous current frame, four seconds old,
while delivering a current cell-voltage frame. With the fixed-reference change
alone, some delayed-current phases produced seconds-long entry excursions while
both converters tracked their limits. Reducing the feedback correction removed
that recurrence in the tested case.

For two units with ideal tracking, a fixed target and inactive caps, the delayed
inner loop is `x[k+1] = x[k] + 2G*(target - x[k-1])`. Its pole magnitude is about
0.894 at G = 0.4 and 0.707 at G = 0.25. This approximation excludes the outer
cell loop, converter dynamics, caps and asynchronous updates; it is not a full
stability proof.

G = 0.2 passed the delayed phase-4 case but failed entry limits at other phases.
A bounded G = 0.25 probe passed all five competing cases: phase 0 with/without
extra current delay, delayed phase 4, phase 9 without delay, and delayed default
phase 5.1. Their printed entry peaks were 3.507–3.513 V. These probes motivated
the candidate; the complete grids determine acceptance.

The existing forecast, converter gain settings, PWM timing and voltage gates
are unchanged. The rejected forecast-gain and dead-time experiments are absent
from the final change.

## Permanent regressions

- `pd-control-test`: equal ampere-error response at 1/4/40 A; settled-40 A
  compatibility; exact moving-target EWMA/P/D sequence; finite zero-target
  response; default relative normalization for other callers; limiter sign.
- `charger-term-test`: exact steps derived from the fixture parameters:
  -0.700 A with the look-ahead and +0.116667 A without it, within 1 mA.
- `cv-hold-pair-lag`: cell phase 4 s, current delayed by one four-second frame.
- `cv-hold-pair-phase0`: cell phase 0, no added current delay.

Both new scenarios share the original pair's entry limit (eoc + 15 mV), steady
band (eoc ±10 mV), hold and latch checks. All three reject an empty steady window.
The stock list contains 191 unique scenarios, including these regressions.

## Validation

All final runs use candidate `795f402e`; it includes `main` at `d0cf88d8`.

| Check | Result |
|---|---|
| Linux 26 phase/current-delay cases | 26/26 PASS: [run 46048183784](https://github.com/fl4p/fugu-mppt-firmware/actions/runs/46048183784) |
| Linux full stock and host suite | 191 scenarios PASS; host units 38 PASS, one known XFAIL: [run 46048183356](https://github.com/fl4p/fugu-mppt-firmware/actions/runs/46048183356) |
| Pinned Linux host units | 38 PASS, one known XFAIL, zero failures: [run 46048183647](https://github.com/fl4p/fugu-mppt-firmware/actions/runs/46048183647) |
| macOS 26 phase/current-delay cases | 26/26 PASS, fresh runs with captured exit 0 |
| Focused PD and charger host tests | PASS, independently reviewed |
| Three permanent pair scenarios, macOS | PASS; printed entry peaks 3.507 / 3.509 / 3.509 V |
| ESP-IDF 6.0.3, ESP32-S3 | PASS; image 0x1b8210 bytes, 4% application-partition space free |
| Documentation | Fresh-cache build, cross-links, rendered values and independent source check PASS |

The grid uses current phase 1.3 s, cell phases 0–11 s plus default 5.1 s, each
with and without one added current-frame delay. Cell/current periods remain
12/4 s. This is 26 cases per platform, not every possible BMS timing pattern.
The known XFAIL is `bms-cutoff-test`. Linux uses x86_64, Clang 21.1.8,
GCC 15.2.0 and glibc 2.43; the pinned phase image is Ubuntu 26.04
`20261116.202.1`, and the pinned unit image is `20261122.9.1`.

Known-bad calibrations:

- Forcing the PD constructor to ignore `normalize=false` fails five checks.
- Restoring G = 0.4 fails the delayed-pair entry gate (printed peak 3.522 V)
  and the exact host-step assertions.
- Restoring G = 0.2 with the final phase-0 harness exits 1 on the entry gate
  (printed peak 3.516 V). The final G = 0.25 case passes.
- On the final phase-0 case, `SIM_T=300` exits 1 with `no steady hold samples`.
  The farther-tail `SIM_T=0` case also exits 1, with no hold and no steady samples.

Earlier candidates are retained as evidence, not passing acceptance results:
fixed-reference/G = 0.4 (`f0ebdf43`) passed 20/26 Linux and 23/26 Mac cases;
G = 0.2 (`cbbb692b`) passed 25/26 Linux and 22/26 Mac cases. All G = 0.2 steady
bands passed; its remaining failures were entry peaks. A corresponding full
unmodified-main phase grid was not run.

## Empty-window guard review

1. Missing steady samples retain reversed bounds (9, 0), so `lo <= hi` fails.
   NaN bounds also fail the comparison. This does not validate every plant sample.
2. The added requirement cannot turn an existing failure into a pass; the
   original voltage limits still reject excursions beyond their bounds. Both
   the shortened and zero-duration cases reject missing observation windows.
3. All three pair names invoke the shared guard and are registered once. Full
   runs pass; the shortened run provides the empty-window calibration.
4. Validation records the code revision, simulator/BMS producers, configuration,
   binary fingerprints and compiler. An input-only fingerprint is insufficient.
5. There is no result cache; failed verdicts and exit codes remain in the logs.
6. Full runs, scratch diagnostics and earlier reused evidence are identified
   separately. The final grids use fresh cases; no probe result substitutes for one.
7. The shortened run demonstrates the missing-evidence failure; controller
   mutations independently demonstrate the targeted behavioral failures.
8. No voltage threshold was relaxed. The controller response changes, and the
   guard rejects missing evidence rather than suppressing a warning.

The host benchmark could not resolve incremental guard cost above noise
(largest observed delta 0.097 ns/iteration, not a latency upper bound). The
verified pass path is a comparison and conditional branch once at simulator
shutdown. It adds no firmware RT work. See `/tmp/cv72-needbench/README.md`.

## Evidence and limits

Final binary: `/tmp/fugu-cv72-final/out/mppt-sim`, SHA-256
`ecd9437ffa96d494da318663db3647a42b8b49d8a4465b9f4e431bb56ad8b674`.
Final Mac artifacts: `/tmp/cv72-final26-mac-grid/` (Apple Clang 17.0.0, arm64).
Final Linux artifacts and per-case provenance: `/tmp/fugu-cv72-repro/final3/`
and `/tmp/fugu-cv72-repro/landing-summary.md`.
Build log: `/tmp/fugu-cv72-g025-build.log`. Earlier grids and diagnostics:
`/tmp/fugu-cv72-repro/`, `/tmp/cv72-final-mac-phase/`,
`/tmp/cv72-g02-mac-grid/`, `/tmp/cv72-g025-clean/`,
`/tmp/cv72-g025-mac-cases/`, and `/tmp/cv72-final-knownbad-g02/`.

Independent review checks the final source, shared gates, test registration and
documentation. The unit test does not instantiate MPPT or assert which competing
limiter wins. Stock scenarios check outcomes, but some reported statistics are
not acceptance assertions. No hardware run or output-waveform measurement has
been performed. These checks resolve the reproduced simulation failures within
the tested scope.
