# Issue 72 — CV-hold look-ahead gain selection (sim-only, not closed)

Candidate fix `22981613` (`src/charger.h`) + regression in `test/host-stub/charger-term-test.cpp`.
Simulator-only; no field claim. Scenario thresholds are byte-identical to before. **Issue 72 is
not closed: one bounded phase variant still fails (§6).**

## 1. Diagnosis

`_cvHoldStep` adds a rise look-ahead (`AHEAD_S = 36 s`, cell frames 12 s apart in this fixture) and selected `K_UP = 0.5` /
`K_DOWN = 1.5` from the **forecast** error. When `36·rise` exceeds the measured error, the forecast
drives `err` negative while the cell is still **below** `cv_eoc`, so the code applied the 3×
`K_DOWN` magnitude — three times the below-target gain. The baseline forecast fires 11 times on
the instrumented Mac `cv-hold-pair` trajectory, all during approach (42–162 s); six of those
forecasts change the error's sign. The forecast reduces the target current by 1.3–2.7 A per frame.

The paired experiments implicate the approach corrections in the later, decaying hold transient;
the precise contribution of polarization and the other controller dynamics remains a hypothesis.
The ±10 mV gate window
opens only at `hold+300 s` and samples the decaying ring, so the verdict flips side with the
build trajectory (Linux clang 21.1.8 fails high, Apple clang fails low, with comparable excursions).

Examined but **not established** as causes: the asynchronous V/I subtraction, `R_OHM = 1.5 mΩ`, and
an idealized 2-state eigenvalue estimate (|λ| ≈ 0.83–0.94) that is not a stability proof of the
full nonlinear loop.

## 2. Fix

Take the gain from the measured error before the forecast; the forecast still shifts `err`. `err ≤ 0`
is unchanged (`K_DOWN`).

For a fixed nonzero forecast, this introduces a downward correction step at measured error zero:
`(K_DOWN - K_UP) * forecast * S`. The old law was continuous at forecast error zero. This is a
tradeoff, not removal of a discontinuity; the boundary regression deliberately pins `K_DOWN`.

```cpp
const float gain = err > 0 ? K_UP : K_DOWN;   // measured error, not the forecast's sign
if (err < 2 * APPROACH_V && dtOk && rise > RISE_VS) err -= AHEAD_S * rise;
_cvIbat = fminf(ibat + gain * err * ..., ibatCap);
```

Rejected scratch variants (bounded, not landed): a hard floor of the positive-error forecast at 0
(worsens recloud re-entry to +17.1 mV); the continuous `gain(e)·e − K_UP·F` (phase 1 still fails,
default entry 3.515); `K_UP = 1.0` (phase 1 still fails, default band worsens).

## 3. Evidence

**Mac, `cv-hold-pair`** (same flags; baseline via a scratch copy of the pre-fix expression):

| | entry peak | band over `holdAt+300 … end` | verdict |
|---|---|---|---|
| baseline | 3.507 | 3.489..3.507 | FAIL low |
| candidate | 3.512 | 3.499..3.501 | PASS |

`cv-hold-load` PASS (peak 3.512); `cv-hold-recloud` PASS (re-entry +11.0 → +6.8 mV).

**Linux** (clang 21.1.8, glibc 2.43, two Ubuntu 26.04 image revisions): old `283c0248` and current
`38ae2865` are byte-identical on the cv-hold summaries (pair FAIL 3.496..3.515, load FAIL 3.515,
recloud PASS). At `22981613` **all ten CV scenarios pass** (pair 3.498..3.502, entry 3.512; load
3.512; recloud +7.4 mV). CI run 46019362503 is red solely on the old `test_hold_rise_frame_spacing`
mode 0, replaced here. Root's ESP-IDF build passes.

**Full `charger-term-test`, fix on vs off:** only four printed lines move — mode 0 cut 1.12 → 0.373 A,
approach recovery 312 → 308 s, 40 A entry peak 3.505 → 3.508, sibling peak 3.506 → 3.508 (both
≤ 3.515). Every other assertion passes on both.

**Regression** (`test_hold_lookahead_gain_from_measured_error`), exact step `G·gain·eff·S`, ±1 mA
(`S = 93.333 A/V`):

| case | measured err | gain | step (pre-fix → fix) |
|---|---|---|---|
| P below `cv_eoc`, forecast | +10 mV | K_UP | −1.120 → **−0.373 A** |
| Z at `cv_eoc`, forecast | 0 mV | K_DOWN | −1.680 A (unchanged) |
| O above `cv_eoc`, forecast | −5 mV | K_DOWN | −1.960 A (unchanged) |
| G below `cv_eoc`, flat (rise gate) | +10 mV | K_UP | +0.187 A (unchanged) |

`test_hold_rise_frame_spacing` mode 0 now asserts the gain-derived cut 0.373 A instead of `> 0.5 A`.
Pre-fix source fails the new suite (2 FAILED, rc 1); the fix passes (rc 0). The test is a 12 s-frame
fixture; the function uses each frame's actual `dt`.

## 4. Guard checklist

1. **Unevaluable input** — no new input; gain is a pure function of the measured cell error. N/A.
2. **Monotonicity** — below `cv_eoc` the gain is `K_UP`, never `K_DOWN`. The forecast can still drive
   `eff` negative (reducing current) at the `K_UP` magnitude; it cannot switch the gain.
3. **Preconditions** — `_cvHoldStep` also runs its blind-decay branch on stale data (age > `BLIND_S`),
   unchanged by this fix.
4. **Source of truth** — gain from `params.cv_eoc − v` (measured), not the forecast.
5. **Persistence** — no cache or latch added.
6. **Provenance** — N/A.
7. **Known-bad calibration** — pre-fix source fails the P assertion and mode 0 (observed, rc 1).
8. **Fix vs mute** — the control law changed; no warning suppressed, no threshold weakened.

## 5. Cost

One comparison reordered per cell frame; the firmware binary size remains `0x1b7e50` bytes.
No cycle-level cost measurement was made. `charger-term-test` wall time was 0.24 s in the worker run.

## 6. Bounded phase / lag variants (`cv-hold-pair`, Mac, candidate vs pre-fix)

| variant | pre-fix | candidate |
|---|---|---|
| default (`phaseC 5.1`) | FAIL 3.489..3.507 | PASS 3.499..3.501 |
| `phaseC 0` | PASS 3.500..3.501 | PASS 3.500..3.501 |
| `phaseC 1` | FAIL 3.492..3.510 | **FAIL 3.495..3.512** |
| `phaseC 7` | FAIL 3.487..3.503 | PASS 3.490..3.504 |
| `phaseC 9` | PASS 3.499..3.501 | PASS 3.499..3.501 |
| `lagI 1` | not run | PASS 3.499..3.501 |
| `phaseC 7 + lagI` | not run | PASS 3.500..3.501 |

`phaseC 1` fails on both sides of the fix. The candidate's printed upper excursion is larger;
the pre-fix 3.510 V print is rounded and does not establish the exact margin. This remains an
unresolved case. Entry margin at the default phase is 3 mV (3.512 vs 3.515) against 8 mV before.

## 7. Limits

- No field behaviour is claimed or verified.
- The ring is reduced, not removed; `phaseC 1` still exceeds ±10 mV. No claim that issue 72 is closed.
- The candidate is Mac-validated and Linux CI-validated on the ten stock CV scenarios; the phase/lag
  variants above are Mac-only.

## 8. Reproduce

```sh
# candidate (tracked HEAD):
OUT=/tmp/cv72-candidate-repro bash test/host-stub/mppt-sim.sh cv-hold-pair cv-hold-load
SIM_BMS=1.3,1,0 /tmp/cv72-candidate-repro cv-hold-pair
# baseline: clean worktree at the fix's parent, then the same command
git worktree add --detach /tmp/fugu-cv72-base 22981613^
# regression suite (candidate): exits 0
test/host-stub/host-tests.sh charger-term-test
```
