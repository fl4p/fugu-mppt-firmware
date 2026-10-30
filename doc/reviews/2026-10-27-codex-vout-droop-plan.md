*this document is an LLM generated placeholder*

# Codex review of plans/vout-droop.md (2026-10-27, HEAD 51c1b95)

## ACCESS

- Live InfluxDB query succeeded: fry’s latest `Uo` was **26.4786 V**, timestamp **2026-10-27T19:55:46.291Z**.
- Playwright fetched [Droop speed control](https://en.wikipedia.org/wiki/Droop_speed_control); `<title>` was **“Droop speed control - Wikipedia”**. No human check or fabchrome access occurred.
- A seven-day aggregate query timed out; subsequent two-day and matched-minute queries succeeded.

**The plan is not ready for implementation followed by live OTA.** Droop remains a plausible sharing mechanism, but the proposed clamp does not establish recovery or safe behavior across charging regimes, and the stability/test arguments need correction.

Reviewed the uncommitted plan against committed HEAD **`51c1b951`**. Other uncommitted changes were excluded. No source files were edited, builds run, or devices commanded.

Live telemetry also challenges the proposed identification model. For **2026-10-27**, matching `mean(Uo)` and `mean(I)` by device in one-minute buckets gave:

| UTC window | Matched buckets | Median `Uo_fry − Uo_flat` | Mean `I_fry` | Mean `I_flat` |
|---|---:|---:|---:|---:|
| 00:00–06:00 | 358 | +20.36 mV | −0.007 A | −0.007 A |
| 11:00–17:00 | 341 | +490.38 mV | −0.015 A | 4.951 A |

These are matched buckets, not synchronous measurements. At 12:00, the difference was **508.46 mV**, with flat reporting **4.851 A**. Under the assumed common bus, constant calibration offset, and output-current interpretation, subtracting the overnight offset would imply approximately **−101 mΩ** for flat’s cable—physically inconsistent with that model.

Consequently, **cable resistance and operating calibration mismatch remain unverified**. Also, `I` is the selected *physical* current sensor, which can be input current when Iout is virtual ([src/mppt.cpp:513](src/mppt.cpp:513), [src/mppt.h:369](src/mppt.h:369)). Verify deployed sensor mapping and voltage behavior before fitting resistance. I did not verify deployed `vout_rl`.

## FINDINGS

### 1. P1 — The bus-tracking floor does not eliminate persistent zero output. **(c) Real design defect**

[Plan:77–86](plans/vout-droop.md:77)

A permitted steady state is:

- A maintains the bus and supplies the load.
- B’s EOC feedback pushes its pin downward.
- B’s clamp holds its pin at `Vbus_measured − 20 mV`.
- B supplies zero current, so its droop term is zero.

B’s target remains below its measured output. Nothing requires it to join A. This can persist while A supplies over 100 W; testing that B takes over **after A drops out** does not test sharing while A remains present.

The upper bound creates another contradiction. With real bus **26.75 V**, B reading **100 mV high**, and fallback **26.8 V**:

- tracking floor = **26.83 V**;
- permitted upper bound = **26.8 V**.

Both constraints cannot hold. Enforcing the upper bound leaves B’s real target at **26.7 V**, below the bus—even though the inter-unit offset could satisfy the plan’s 125 mV allowance.

There is also an independent restart path: fresh termination blocks rearming a disabled converter, and the stuck watchdog excludes `chargeHold()` ([src/main.cpp:1159](src/main.cpp:1159), [src/main.cpp:845](src/main.cpp:845)). A pin-only change does not resolve that.

**Required correction:** specify separate sharing, takeover, and restart guarantees, including behavior when tracking and safety bounds conflict. Do not supersede load-following Phase 1 based on the proposed dropout test.

### 2. P1 — Removing authority gating restores upward windup during unavailable PV. **(c) Real design defect**

[Plan:81–86](plans/vout-droop.md:81)

The existing `_loadFollowStep` authority gate protects against **upward** windup too. Its comment explicitly describes a disabled nighttime converter integrating house load toward `Vbat_max`, then charging at dawn ([src/charger.h:442](src/charger.h:442)).

For a zero-current hold target and `ibat = −8 A`, each fresh frame adds the capped **20 mV**. Allowing this while disabled can move a 26.5 V pin to 29 V in **125 frames**. A lower bound at `bus − margin` does nothing to prevent it.

“The hold pin” is not a defined independent upper bound: the current hold controller modifies `vpack_pin` itself, bounded by `Vbat_max` ([src/charger.h:450](src/charger.h:450)). An existing test explicitly requires the pin to remain unchanged without authority ([test/test_charger.cpp:756](test/test_charger.cpp:756)).

History confirms two different failures:

- **`32540e3`** introduced authority gating to stop downward EOC ratcheting without plant response.
- **`12d5876`** introduced terminated yielding after upward release caused reinjection and approximately 60-second limit cycling.

**Required correction:** distinguish a healthy idle sibling from disabled, PV-starved, and actuator-limited units. Define directional anti-windup, explicit upper bounds, and filter/glide state handling.

### 3. P1 — The claimed α bound is not a stability certificate for this controller. **(c) Real design defect**

[Plan:58–64](plans/vout-droop.md:58)

`1 − α(1 + G)` assumes the terminal voltage reaches each new target instantaneously. The firmware instead includes:

- normalized PD with derivative memory;
- accumulated duty corrections;
- median-of-three voltage feedback normally, EWMA during sweeps;
- minimum selection across limiters and the MPPT tracker;
- asymmetric duty-step limits.

See [src/pd_control.h:34](src/pd_control.h:34), [src/mppt.cpp:107](src/mppt.cpp:107), and [src/mppt.cpp:270](src/mppt.cpp:270).

Even the plan’s proposed **first-order-lag surrogate** invalidates its no-ringing assertion. For normalized current perturbation `x` and filtered current `z`, consider:

```text
z_next = (1 − α)z + αx
x_next = (1 − β)x − βG·z_next
```

With `G=4`, `β=0.1`, and the prescribed `α=0.2`, its poles are:

```text
0.810 ± 0.253j
```

That is damped ringing. These are illustrative surrogate poles, **not measured firmware poles**.

**Required correction:** establish the inner-loop dynamics and timing, then validate the coupled system across limiter transitions, slew saturation, sweep feedback, and asymmetric PV availability. Cable resistance alone cannot determine a defensible filter span.

### 4. P2 — Sharing arithmetic mixes current difference with deviation from equal sharing. **(a) Arithmetic error**

[Plan:13](plans/vout-droop.md:13), [Plan:111](plans/vout-droop.md:111)

For equal effective resistance `Z = Rc + Rd`, with both units conducting:

```text
I1 − I2 = ΔV / Z
I1 − Itotal/2 = ΔV / (2Z)
```

The proposed `I_diff ≈ ΔV/(2Z)` is wrong if `I_diff` means the difference between currents.

Thus:

- 50 mV mismatch with **5 mΩ per cable** gives **10 A current difference**, or ±5 A around equal sharing.
- 125 mV with **25 mΩ per branch** gives **5 A current difference**. That tolerance is consistent with this definition.
- At total output current ≤5 A, that allowed mismatch can leave one unit at zero. Both exceed 25% only when total output exceeds 10 A.

Therefore, the calibration allowance cannot support the unconditional sharing acceptance criterion at low load.

There is a second arithmetic problem: twice the stated **α=0.2** bound is **0.4**, giving pole **−1** in the plan’s instantaneous model—marginal oscillation, not growing divergence ([Plan:112](plans/vout-droop.md:112)).

### 5. P2 — The absorption guarantee assumes a controller that the code does not implement. **(c) Real design defect**

[Plan:65–68](plans/vout-droop.md:65)

The EOC branch is not a bidirectional integrator that increases the pin whenever cells fall below target. It runs only when `vcell_high >= v_eoc`, then subtracts the nonnegative error. Below target, execution instead reaches float/glide or bulk behavior ([src/charger.h:567](src/charger.h:567), [src/charger.h:595](src/charger.h:595)).

So:

- This EOC branch cannot simply wind upward to cancel droop.
- Bulk and hold behavior may restore voltage, but only within their bounds.
- Termination depends on a current-dependent line, not solely on reaching `cv_eoc` ([src/charger.h:198](src/charger.h:198)).
- Output current includes house loads; it need not approach zero when battery charging ends.

For hypothetical synchronized bidirectional outer integrators, common-mode compensation would **preserve** the differential sharing benefit while restoring bus voltage—provided neither saturates. That is not proof of the existing EOC state machine’s behavior.

**Required correction:** replace “the pack does not end lower” with a headroom condition and actual termination tests, including sustained loads, stale BMS data, and unequal saturation. The **0.4 V** subtraction is correct; its claimed consequence is unproven.

### 6. P1 — The safety predicate accepts invalid normalized targets and silently removes droop on bad data. **(c) Real design defect**

[Plan:114](plans/vout-droop.md:114), [Plan:140–147](plans/vout-droop.md:140)

`Vset ≤ Vout_max()` is insufficient because the PD divides by its target.

For example, `pin=28 V`, `Rd=0.1 Ω`, and finite corrupted filtered current `300 A` produce `Vset=−2 V`. At measured 28 V:

```text
normalized error = 1 − 28/(−2) = +15
```

The CV controller requests an increase rather than enforcing the voltage limit. The proposed inequality still passes. This demonstrates a missing precondition, not that such a sensor fault necessarily escapes every independent protection.

Likewise, changing valid 20 A to NaN removes **0.4 V** of droop at 20 mΩ, raising the target. That is an explicit degraded mode, not monotone verified safety.

**Required correction:** require a finite, positive effective target; define stale-data detection and recovery; prevent filter poisoning; and test zero, negative targets, overflow, NaN, infinity, and recovery trajectories. Record degraded operation explicitly.

The checklist also omits implementation/input provenance for model results, applicable persistence behavior, actual invocation checks, and runtime-cost measurement. Mark nonapplicable cache items explicitly rather than treating the shortened checklist as complete.

### 7. P1 — The host-test design can pass without exercising the real failures. **(c) Real design defect**

[Plan:108–128](plans/vout-droop.md:108)

A standalone lag model cannot establish equivalence to the firmware’s authority, termination, filtering, and actuator state machines. In particular:

- `chargeHold()` bypasses the `iout > 0.5 A` requirement, although duty and enabled checks remain ([src/main.cpp:892](src/main.cpp:892)).
- A dropout-only test misses the persistent-idle equilibrium in finding 1.
- Equal pins need not give equal currents with unequal cable resistances.
- A stiff source plus resistance does not reproduce per-cell termination, BMS frame timing, stale data, or night-to-dawn transitions.
- Filter initialization/reset across shutdown, sweep, and reenabling is unspecified.
- “N ticks” is undefined, and a build alone establishes no stability.

**Required correction:** exercise production controller/state-machine code against the plant model. Include steady sibling operation, both historical authority failures, cold/partial holds overnight, conflicting bounds, stale sensors/BMS, unequal cables and PV power, and startup after termination. Retain the mathematical model as a supporting check.

### 8. P2 — The live activation procedure does not activate the proposed parameter. **(c) Real design defect**

[Plan:129–135](plans/vout-droop.md:129)

`set-config` writes the configuration file; it does not reload charger parameters ([src/cli.cpp:1506](src/cli.cpp:1506)). Charger parameters load through `begin()` ([src/charger.h:331](src/charger.h:331)).

As written, OTA with droop zero followed by `set-config` can leave the running feature disabled.

**Required correction:** specify reboot/reinitialization or an implemented runtime setter, verify the active value and effective target, and define rollback. Assess sharing using verified output currents and independently establish PV headroom; a genuinely PV-starved unit cannot satisfy the proposed percentage criterion.

## SURVIVED

- **(b) No inherent P-only steady-state droop was established.** The important assumption happens to hold in the unsaturated CV regime because PD output **increments duty**; fractional increments accumulate ([src/buck.h:1355](src/buck.h:1355)). With default `Kp=1500` and slew factor `0.025`, the proportional contribution is `dD/dt = 37.5·e`. At 28 V, a persistent 10 mV error commands approximately **0.0134 duty/s**. It does not require a finite voltage error to sustain duty. Ideal steady-state error is zero; quantization, switching, and limit cycles require separate measurement.

- **(b) The individual equilibrium formula survives conditionally.** With settled CV operation, available power, positive current, and consistent voltage frames, `I=(pin−Vbus)/(Rc+Rd)` is correct. Include sensor offset in the effective pin and clip for zero-current/PV/current limits.

- **A fixed high-reading Vout offset does not by itself make the tracking clamp inject current.** At zero current, `measured_bus = real_bus + offset`; choosing `pin=measured_bus−margin` gives a real target of `real_bus−margin`. The offset cancels. An unlimited A/B upward ratchet does **not** follow from that ideal clamp alone. The actual concerns are dynamic lag, independent upward integration, transitions, and conflicting bounds.

- **The 20 mΩ × 20 A = 0.4 V calculation survives.** Twenty milliohms is a plausible candidate for investigation, not a validated selection. The live telemetry makes calibration/sensing verification the appropriate first step. Natural cable droop could suffice if residual mismatch is sufficiently small; the present data cannot establish that.

- **The direct `Vout_max()` consumer audit found no hidden sweep/watchdog target comparison.** Sweep stopping uses the selected limiter’s actual target ([src/mppt.h:1337](src/mppt.h:1337)); CV mode and `cv_lim_idx` follow limiter selection. The watchdog uses headroom/current/hold state. Periodic sweep suppression remains consequential because CV and charge hold suppress it ([src/mppt.cpp:90](src/mppt.cpp:90)).

- **PSU/PV-sim exclusion and unchanged independent OV protection are compatible with the proposed insertion point.** They do not prove that the combined slow-path changes cannot raise operating voltage. Explicitly bypassing all new behavior at droop zero is sound; its bit-identical behavior still needs the proposed golden comparison.
