# Runtime-adjustable switching frequency (`pwm-freq`) — MCPWM only

Issue: [fl4p/fugu-mppt-firmware#62](https://github.com/fl4p/fugu-mppt-firmware/issues/62)

## Context

`pwm_freq` is read exactly once, at `src/buck.h:816`, so the only way to change the switching
frequency is `set-config board.conf pwm_freq <hz>` + `restart`. On the recirculating power-loop
rig a restart of the buck is a reversed shutdown — fbuck drops to zero while fboost stays pumping
at `dc 2499` — and the loop collapses (measured on flu 2026-09-04: Vout 27.2 V → 7.1 V over two
minutes, tracker engaged on the way back up, repeated OV trips until the operator broke the loop).

This blocks `~/dev/pv/ee/plans/BENCH-flu-fsw-sweep.md`, which needs **eleven** frequency changes
(25/32/39/48/60/75 kHz with repeated 39 kHz drift controls) interleaved against a reference point.
Eleven reboots is eleven loop collapses, and each one also perturbs the thermal state the
interleave exists to control. Beyond that one run, every frequency-dependent measurement on the
rig — core loss, gate-drive loss, EMI spectra vs `fsw` — has the same problem.

Outcome: a `pwm-freq` console verb that changes the frequency live, glitch-free, preserving the
operating point, or refuses with a reason and leaves the converter untouched. RAM-only;
`board.conf::pwm_freq` stays the boot value.

Decisions taken with the user:
- **No mode gate.** Works in any mode (MPPT / PSU / PV / manual). Duty counts are rescaled so the
  operating point survives; the tracker's captured MPP is invalidated.
- **Ripple:** only the existing `fsw·L0·0.95 ∈ (1, 20)` guard, applied as a *refusal*, not a throw.
  The operator owns the ripple budget.

## The one finding that shrinks this a lot

The issue (item 4) and a first read of `bestTiming()` both suggest `resolutionHz` moves with
frequency, so every ns-derived count would need re-deriving. **It does not, anywhere in the legal
range.** `bestTiming()` (`src/pwm/mcpwm_timing.h:13`) raises the prescaler only when
`160e6/presc/freq > 65535`; at the lowest legal frequency (5 kHz) that is 32000, so **the prescaler
is always 1 and `resolutionHz` is always 160 MHz** across 5 kHz–500 kHz.

Consequences:
- The tick is 6.25 ns at every frequency, so `getPwmTickRate()` (`src/buck.h:760`, which returns
  `resolutionHz` on MCPWM) does not move.
- `rectRefreshTicks`, `pwmRectMin`, `rectOnOffset` and `dtHlTicks`/`dtLhTicks` are **already
  frequency-invariant** and must be left alone. Their *duty fractions* change; that is physics.
- The prescaler cannot be changed on a running timer anyway (`mcpwm_timer_set_period` writes the
  period register only), so the implementation must compute ticks from the **live** `resolutionHz`,
  not from `bestTiming(hz).resolution_hz`, and refuse if the two ever disagree (unreachable today,
  cheap insurance).

What actually has to change is a much shorter list: `periodTicks`, `pwmMax`, the live comparators,
`pwmCtrlMax`, `pwmFrequency`, `fL`, and the duty targets held elsewhere as raw counts.

## Design

Mirror the runtime dead-time path (`dt`) exactly — it already solves the same problem (core-0
producer, RT-core applier, TEZ-latched writes, envelope recompute) and is the reviewed precedent.

### 1. `MCPWM_SyncLeg::setPeriod()` — `src/pwm/mcpwm.h`

Add next to `setPeriodTicks` (line 376):

```cpp
// Full period change: register write + the members derived from it. Validated, unlike the raw
// trim below, which bsync uses for ±1-tick dither and which must NOT move periodTicks/pwmMax.
esp_err_t setPeriod(uint16_t ticks);   // ticks > dtHlTicks_, > dtLhTicks_, >= some floor
                                       // -> mcpwm_timer_set_period(); periodTicks = ticks;
                                       //    pwmMax = ticks - dtLhTicks_
```

Keep `setPeriodTicks()` as-is — bsync's 1 kHz sigma-delta (`src/sync/bsync.cpp:151`) depends on it
not touching the members. Extend its comment: the "never below the init periodTicks" contract is
now "never below the *current* `periodTicks`".

### 2. `SynchronousConverter` — `src/buck.h`

**Mailbox**, alongside `dtReqWord`/`dtAckWord` (line 142). Period ticks need 16 bits, so pack
`[31:16] seq | [15:0] ticks`; same latest-value single-producer contract, same rationale comment.

**Producer, core 0** — `const char *requestPwmFrequency(uint32_t hz, uint16_t &ticks)`, returning
`nullptr` on success or a static reason. Refusals, in order:

| # | Refusal | Source |
|---|---|---|
| 1 | not the MCPWM driver | `drvDtResolutionHz() == 0` — LEDC has no period register and a power-of-two `pwmMax` (`src/pwm/ledc.h:21`); explicit refusal, never a silent no-op |
| 2 | `hz` outside (5e3, 5e5) | same bounds as the boot assert, `buck.h:817` |
| 3 | prescaler would have to change | `bestTiming(hz).resolution_hz != mcpwmDrv.resolutionHz` |
| 4 | `fsw·L0·0.95` outside (1, 20) | `buck.h:819-821`, as a refusal — an out-of-range `pwm_freq` currently prevents boot entirely, which looks exactly like a bad flash. With `L0=80e-6` the lower bound bites below ~13.2 kHz |
| 5 | dead-time above 1/32 of the new period | mirrors `requestDeadTimeNs` (`buck.h:615`) and `parseDt`'s `dtCeil` (`buck.h:180`) |
| 6 | `rectRefreshTicks + dtHl > newTicks/4` | `buck.h:616`; this is what caps the top of the range — `pwmRectMin` is a fixed count while the period shrinks (at 75 kHz: 330 of 2119, fine; ~150 kHz is where it bites) |
| 7 | wired sync configured (`sync_role != none`) | the leader's pulse comparators are absolute ticks set once in `initSyncOut` (`mcpwm.h:230`) and the follower's period is baked 2 ticks short (`mcpwm.h:112`); neither has a re-arm path. Also `wsyncQualifyLine` assumes leader and follower share `pwm_freq` (`buck.h:259`) |

`ticks = lround((double) resolutionHz / hz)` — from the live resolution.

**RT applier** — `float applyPendingPwmFreqRt()`, returning the scale actually applied (0 = nothing
done), called next to `applyPendingDeadTimeRt()`.

The ordering is the heart of it. Both the period register and the comparators latch on TEZ, so if
no period boundary falls between the writes the change is atomic; if one does, a single period runs
mixed values. That transient must be safe in both directions:

- **Period shrinks (frequency up)** — write the rescaled+clamped comparators **first**, then the
  period. The bad interleaving is one period of small comparators against the old large period:
  lower duty for 25 µs, gate order intact. The reverse order is exactly the hazard
  `setPeriodTicks`' comment warns about — a comparator left above the new period never fires and
  the LS gate stays high for a full cycle.
- **Period grows (frequency down)** — write the **period first**, then the comparators. The bad
  interleaving is old (small) comparators against the new large period: again lower duty for one
  period, and the LS→HS band is *wider* than required, not narrower.

Then, unconditionally on success:

```
r         = (float) newTicks / (float) oldTicks
pwmCtrl, pwmRect, pwmRectMax, manualRect(if >=0)  *= r, then clamp to the new envelope
driverPwmMax = mcpwmDrv.pwmMax                 // = newTicks - dtLhTicks
pwmCtrlMax   = isBoost ? newMax*0.9f : newMax - pwmRectMin - 1
pwmFrequency = hz
fL           = hz * coilL0 * InductivityDcBias  // coilL0 is already retained, buck.h:128
// rectRefreshTicks, pwmRectMin, rectOnOffset, dtHl/dtLh: UNCHANGED — fixed times at a fixed tick
```

Clamp `pwmCtrl` to `pwmCtrlMax` and `pwmRect` into `[pwmRectMin, newMax - pwmCtrl - 1]` after
scaling (the `-1` is the cmpLS-below-period rule, `buck.h:926`). A rescaled duty can land above the
new `pwmCtrlMax` when the frequency rises, because `pwmRectMin` does not scale; report it rather
than hiding it.

Everything else keyed to `pwmMax` follows automatically — the fade-in and ramp steps are all
`driverPwmMax/N` (`buck.h:982,999,1003,1010`), the manual ramp is `pwmMaxDriver()/512`
(`mppt.cpp:327`), the control slew normalizes to `pwmCtrlMax` (`mppt.cpp:263`). No arrays are
indexed by pwm count (the `pwmPowerTable` in `tracker.h:196` is commented out).

**Public accessors** for the console: `getPeriodTicks()`, `getPwmResolutionHz()` (promote the
existing private `drvPeriodTicks()`/`drvDtResolutionHz()`), and

```cpp
// Realized fsw. bestTiming()'s actual_freq is res/ticks in uint32 integer division and truncates
// (4103 ticks reports 38995, true 38995.86); pwm-dump prints the REQUESTED freq. This is the only
// honest source.
[[nodiscard]] double realizedFreqHz() const;   // (double) resolutionHz / periodTicks
```

### 3. Duty targets held outside the converter — `src/mppt.h` / `src/mppt.cpp`

`manualTarget` (`mppt.h:200`) and `targetPwmCnt` (`mppt.cpp:368`) are **raw counts**, and
`mppt.cpp:349` ramps `pwmCtrl` back toward `manualTarget` every tick. Rescaling only the converter
would have the manual ramp immediately undo the change — the single most important consequence for
the bench, and not in the issue's list.

Add `MpptController::applyPendingPwmFreqRt()`:

```
float r = converter.applyPendingPwmFreqRt();
if (r > 0) {
    manualTarget  = clamp(round(manualTarget * r), 0, converter.pwmCtrlMax);
    targetPwmCnt  = round(targetPwmCnt * r);
    tracker.maxPowerPoint = {};        // captured MPP is a raw count (tracker.h:9) -> stale
}
```

Call it from `src/main.cpp:598`, replacing the bare `converter.applyPendingDeadTimeRt()` line with
the pair (both must run outside the `NewData` branch so a change also lands while idle).

### 4. Console — `src/cli.cpp`

`cmdPwmFreq`, modelled on `cmdDeadTime` (`cli.cpp:194-253`) including the strtof full-token parse
and the 1 s RT-ack wait on `esp_timer_get_time()` (not `wallClockMs()` — a wedged RT loop would
stop the very clock measuring it):

```
pwm-freq          -> report
pwm-freq <hz>     -> change live, or FAIL with a reason
```

Two guards before the request, matching `dt`'s house style of keeping policy in the CLI:
- `isMeasuring()` → `"pwm-freq: busy measuring"` (`cli.cpp:200`).
- bsync service Running → refuse. It caches `nomPeriod_`/`ticksPerUs_` at `onStart()`
  (`bsync.cpp:230`) and dithers the same register at 1 kHz; after a frequency change it would
  drive the period straight back to the old value. Tell the operator to `svc off bsync` first.

Report line — realized, not requested:

```
pwm-freq 38995.86 Hz (period 4103 ct, res 160000000 Hz) pwmMax=4089 hs_off=2499 dt hl=62.5 lh=87.5 ns maxHS=3758
```

Also add `period_ticks=` to `pwm-dump` (`cli.cpp:1939`) and switch its `freq=` to the realized
value — `BENCH-flu-fsw-sweep.md:44,163` explicitly reads `period_ticks` as the source of truth and
warns off both `actual_freq` and the requested `pwm_freq`.

Register next to `dt` at `cli.cpp:1897`, inside the same `HAVE_MCPWM` guard.

### 5. Docs

- `doc/Console.md` — the verb, RAM-only, the refusal list, the `svc off bsync` prerequisite.
- `doc/Configuration.md:44` — note that `pwm_freq` is the *boot* value and `pwm-freq` overrides it
  for the session.
- Add a line to `doc/dev-notes/beacon-sync.md` and `doc/dev-notes/wired-sync.md` recording that a
  runtime frequency change is refused while either sync owns the period.

## Files

| File | Change |
|---|---|
| `src/pwm/mcpwm.h` | `setPeriod()`; amend the `setPeriodTicks` contract comment |
| `src/buck.h` | mailbox, `requestPwmFrequency()`, `applyPendingPwmFreqRt()`, `realizedFreqHz()`, public period/resolution accessors |
| `src/mppt.h`, `src/mppt.cpp` | `MpptController::applyPendingPwmFreqRt()` — rescale `manualTarget`/`targetPwmCnt`, reset captured MPP |
| `src/main.cpp:598` | call it from the RT loop |
| `src/cli.cpp` | `pwm-freq` command + registration; `pwm-dump` reports realized freq + `period_ticks` |
| `doc/Console.md`, `doc/Configuration.md`, `doc/dev-notes/{beacon,wired}-sync.md` | document |
| `test/test_pwm.cpp` | tests below |

## Verification

**Host / unit** (`test/test_pwm.cpp` already sweeps `{20k, 39k, 100k}` at line 446):
1. `bestTiming()` returns prescaler 1 and `resolution_hz == 160e6` across 5 kHz–500 kHz — the
   assumption the whole design rests on.
2. `requestPwmFrequency` refuses each of the seven cases above and leaves every member untouched.
3. Round-trip: 39 k → 75 k → 25 k → 39 k returns `periodTicks` to 4103 and the duty *fraction* to
   within one count; dead-time ticks, `rectRefreshTicks` and `rectOnOffset` are bit-identical.
4. Ordering: with a fake leg recording the write sequence, assert comparators-before-period on a
   shrink and period-before-comparators on a grow.

**On-target** (`RUN_TESTS=1 idf.py -B build-tests build flash monitor`) — the same on real MCPWM,
plus `mcpwmdump` register readback of `CFG0` after each change.

**Bench, in this order** (bench units flash without asking; fry/flat need confirmation first):

1. **fbuck, output open, low Vin.** Flash, `dc 800`, then walk 39 → 48 → 60 → 75 → 39 → 32 → 25 →
   39 kHz. After each: `pwm-freq` and `pwm-dump` (expect `period_ticks` = 4103/3333/2667/2133/
   4103/5000/6400/4103), `dt` (expect hl/lh **unchanged in ns**, 62.5/87.5 — the invariance claim),
   and `status`. No `ADC error`, no backoff, no reboot in the console log.
2. **Scope both gates** at 25 kHz and 75 kHz: confirm the dead-band is still ~62.5/87.5 ns and that
   no cycle shows both gates high or LS stuck high for a full period. Take one capture across the
   change itself (`etc/pico_capture.py`) to prove the transient is glitch-free.
3. **Refusals**, each expected to leave the converter running untouched: `pwm-freq 4000`,
   `pwm-freq 600000`, `pwm-freq 12000` (fails the `fsw·L0` guard at `L0=80e-6`),
   `pwm-freq 39000` while `svc on bsync`, and on a `pwm_driver=ledc` profile.
4. **flu on the power loop**, only after 1–3 pass: fboost held at `dc 2499`, walk the eleven-point
   sweep grid, and confirm the loop never unloads — `BLE_ESP32_INA228_2/_4` Vin/Vout stay inside a
   few percent across each change, which is the whole point of the issue.
