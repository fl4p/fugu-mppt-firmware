*this document is an LLM generated placeholder*

# Vout droop for parallel chargers on a shared bus

Status: **revised after Codex review** (`doc/reviews/2026-10-27-codex-vout-droop-plan.md`).
Step 0 (calibration) blocks everything else. Part B (anti-windup) is a design sketch, not
ready to implement.

## Problem

fry and flat charge one 8S LFP bank. Each closes its own CV loop on its own Vout sensor. When
the bank is CV-limited, the unit whose setpoint is higher **in real volts** takes the current and
the other sits at ~0 A. Only the cable resistance between each sense point and the bus shares
current. For two branches with effective resistance Z each (both conducting) and a setpoint
mismatch ΔV:

    I1 - I2         = ΔV / Z
    I1 - Itotal/2   = ΔV / (2Z)

With Z = 5 mΩ of cable, 50 mV of mismatch is already a 10 A difference.

**Live, 2026-10-27 12:00-12:10 UTC (InfluxDB `mppt`, 10-min means):** fry Uo = 26.888 V,
I = -0.02 A, P = -0.4 W. flat Uo = 26.392 V, I = 4.86 A, P = 128 W. Overnight (both at 0 A), the
median Uo difference was +20 mV (Codex, matched 1-min buckets). On one bus, flat should read
*higher* than fry while it drives current. Instead fry reads ~0.5 V higher, so the midday gap is
not cable drop. It is either a current- or temperature-dependent sensing error, or the two units
do not see the same node. Today's asymmetry is happening again with exactly this gap.

Two slow loops make it worse:

- **The authority gate** (`src/main.cpp:888-899`, `src/charger.h:534`). A terminated unit at ~0 A
  yields to `Vbat_fallback - vout_offset_max` (≈ 26.2 V) while the bus is at ≈ 26.75 V
  (2026-06-30, 2026-07-07). This is Phase 1 of `plans/load-following.md`, still open.
- **Integrator history.** `32540e3` added authority gating to stop downward EOC ratcheting on a
  unit with no plant response. `12d5876` added the terminated yield after upward release caused
  re-injection and a ~60 s limit cycle. `_loadFollowStep` is also authority-gated so a disabled
  converter does not integrate the night load up to `Vbat_max` (`src/charger.h:442`, test at
  `test/test_charger.cpp:756`). Any change must keep all three protections.

## Goal

When the bank is CV-limited and both units have PV headroom, both carry current, with no
communication. Sharing is only meaningful above a minimum bank current: with an allowed mismatch
ΔV and branch resistance Z, both units conduct only when `Itotal > ΔV / Z`.

Out of scope: master/follower over a link, bsync phase interleaving, and exact proportional
sharing.

## Step 0: calibration and sensing (blocks everything, read-only first)

Droop bounds the imbalance; it does not remove it. A 0.5 V mismatch would need Z ≈ 50 mΩ for a
10 A difference, which costs 1 V of setpoint at 20 A. Droop is therefore not the fix while the
mismatch is this large.

1. Read-only `get-config sensor.conf vout_rl`, `sensor` and `get-config sensor.conf` on both
   units. Confirm the deployed divider values and which physical sensor feeds telemetry `I`.
   `I` can be the input sensor when Iout is virtual (`src/mppt.cpp:513`, `src/mppt.h:369`).
2. Plot the fry−flat Uo difference over a whole day against each unit's I, `ntc_temp` and
   `mcu_temp`. Decide between a fixed offset, a current-dependent offset (a shunt or ground path
   in the sense line) and a temperature-dependent offset.
3. Fix the cause (recalibration, sense wiring). This needs Fab's go-ahead on the live units.
   Then re-measure: if the residual |ΔV| at midday is below ~20 mV, the natural cable droop may
   already share enough, and parts A and B may be unnecessary. Decide that from data before
   writing firmware.
4. Only if droop is still needed, estimate Z per unit from a period when both conduct, using the
   verified current signal.

### Step 0 results (2026-10-28, telemetry only)

Data: InfluxDB `ha_van`, 1-min means for 2026-10-21 to 2026-10-28. Sources: `mppt` (fry,
flat) and `V`/`A` `bat_caravan_voltage`/`_current` (BMS pack). Scripts:
`/tmp/step0-scripts/*.py`.

- **Telemetry `I` is Iout on both units** (median P/(Uo·I) = 1.00).
- **fry sat at 0 A** for 94–99 % of flat's charging minutes on five of the seven days.
- **Each Vout minus the BMS pack voltage:**

  | state                       | fry − BMS     | flat − BMS        |
  |-----------------------------|---------------|-------------------|
  | both idle (n=5828)          | +0.037 V      | +0.016 V          |
  | flat only converting (1332) | +0.045…+0.076 | **−0.33…−0.46**   |
  | both converting (299)       | +0.071        | −0.17             |

  Idle, the two units agree within 20 mV, so the static gain is fine. While converting, flat's
  terminal must be *above* the idle fry (it sources the current). Yet flat reads 0.3–0.45 V low,
  stepping in at 0.1 A without scaling with current. The error is present in both CCM (−0.34 V
  at 5 A) and DCM (−0.37 V), and is smaller when fry also converts. fry reads correctly while
  converting.
- **Conclusion:** this week's 0 W asymmetry comes from **flat's Vout sensing reading low while
  its own converter switches**. flat regulates its real terminal voltage ~0.4 V above its
  intent, and fry's pin, which is correct in real volts, sits below the bus. Neither droop nor a
  gain trim fixes a switching-dependent offset. The cause is in flat's Vout sense path: an
  INA226 bus-voltage input via a 47k/46k divider, coupled to switching through ground or
  layout.
#### Console check (2026-10-30, read-only, telnet through havan over Tailscale)

Captures are in `doc/research/2026-10-30-step0-console-{fry,flat}.txt`.

- **Firmware.** Both units run `fry-brk1-133-g4fc1ddab`, built 2026-06-18, with 55 days of
  uptime. None of the September charger fixes are live.
- **Deployed `sensor.conf`.** flat has `vout_rl=47e3` (nominal; the May trim is *not* deployed),
  and fry has `42137`. Idle readings agree within 20 mV, so the nominal value is correct: **do
  not trim flat.**
- **Board differences.** `ina22x_resistor` is 0.5 mΩ on fry and 1.0 mΩ on flat. flat also has
  `skip_assert=1`. Both use the INA226 for Vout and Iout.
- **Console log, 2026-10-28 to 10-30.** Line by line, flat − fry Vout while flat converts:
  median −0.29 V, IQR −0.42 to −0.22 V, n = 1795. The spread is the same in CCM and DCM. This
  confirms the Influx result.
- **Second mechanism: termination divergence.** On 2026-10-30 at 08:04, flat released
  termination by DoD (its counter read 42.00 Ah). fry's counter read 41.58 Ah, so fry stayed
  `TERMINATED` with `Vout_max=26.20` (the authority-yield floor) at 0 W, while flat ran MPPT
  at a 29 V limit. Each unit integrates `ahSinceFull` on its own, so release times drift apart.
  During shared termination, the unit with authority holds 26.8 V and the other yields to
  26.2 V. Because of the sense error, flat holds its 26.8 V target ~0.3 V high in real volts, so
  it always wins authority and fry always loses.
- **So the asymmetry has two causes:**
  1. The authority-yield floor plus per-unit termination state make the outcome all-or-nothing.
     That is firmware, and it is still present in `main` (`src/charger.h:534`).
  2. flat's switching-dependent Vout error decides who wins.

  Fixing (2) alone turns a fixed loser into a race. Fixing (1) alone still lets flat over-drive.
- **Side observations.** flat logged "Loop latency high (49 Hz, then 20 Hz)", windows 1 and 2
  of 3, with no shutdown. It coincided with the config dump over telnet. ADC
  `resetPeripherals` lines follow sweep calibrations, which is normal.

- **Superseded:** the deployed `sensor.conf` on flat and fry (resolved above). The `config/dl/` copies date from
  2025-12 and use the old role keys, so they are not what runs now. Neither device is reachable
  from the home LAN (192.168.178.x).
- **Next steps:** read-only `get-config sensor.conf` and `sensor` on both units from the van LAN.
  Then, with permission, a bench or scope check of flat's INA226 VBUS node while switching
  (`etc/scope.py`). Also check whether fry's board differs (divider placement, RC filter).

## A. Droop on the CV setpoint (only if step 0 says so)

New key `charger.conf::vout_droop` [Ω], default 0. When it is 0, none of the new code runs.

    Vset = charger.Vout_max() - vout_droop * clamp(Iout_droop, 0, Iout_max)

applied to the VoutController target only, never to `psuVsetpoint`.

- **Target validity.** PD_Control normalizes by its target (`src/pd_control.h:34`). A
  non-positive or tiny target inverts the CV limiter: pin 28 V, R_d 0.1 Ω and a corrupt 300 A
  give Vset = -2 V and an error of +15. So clamp Iout_droop to [0, Iout_max], cap the droop at a
  fixed maximum (e.g. 0.5 V), and require Vset > 0.9·Vout_max().
- **Degraded mode.** If Iout_droop is NaN or stale, the droop is removed, which *raises* Vset by
  up to the droop cap. That is today's behaviour, so it is not unsafe against OV, but it is a
  degraded mode. Log it once, and do not call it "monotone safe".
- **Filter.** Use a dedicated EWMA. Reset it on converter disable, sweep start and re-enable, so
  stale current is never applied after a restart.
- **Stability.** The earlier `1 - α(1+G)` bound assumed the terminal reaches Vset instantly.
  That is wrong: even a first-order inner lag (β = 0.1, G = 4, α = 0.2) gives poles at
  0.81 ± 0.25j, which rings. Stability must come from the real inner loop: PD increments to duty
  (`src/buck.h:1355`, Kp = 1500 and slew 0.025 → dD/dt ≈ 37.5·e), median-of-3 feedback, min()
  across limiters, and asymmetric slew. Measure the CV step response on a bench unit first, then
  pick the span.
- **Cost.** At 20 mΩ and 20 A the setpoint drops 0.4 V. The EOC branch only pulls down
  (`src/charger.h:567`); bulk and float targets do not wind up to cancel droop. So absorption
  ends lower unless headroom remains. Check termination with sustained house load, since Iout
  includes the load. Do not claim "the pack does not end lower".
- **Activation.** `set-config` writes the file but does not reload charger params
  (`src/cli.cpp:1506`; params load in `Charger::begin()`, `src/charger.h:331`). Activation
  therefore needs a restart, or a runtime setter added like `vset`. Verify the live value through
  a status/get line, and roll back with `vout_droop=0` plus a restart.

## B. Replacing the authority yield (design sketch, NOT ready)

The first draft clamped `vpack_pin ≥ Vout_meas - margin` on the non-driving unit and re-enabled
the integrators there. The review showed three problems with that:

1. **Idle is still a valid steady state.** The pin sits at bus - margin with zero current and
   zero droop, and nothing makes the unit join. A test where unit A drops out does not show that
   the units share while A is present.
2. **Bounds can conflict.** Bus 26.75 V and a unit reading 100 mV high give a floor of 26.83 V
   against an upper bound of 26.8 V. The rule for which bound wins is missing.
3. **Upward windup at night.** Ungating `_loadFollowStep` lets a disabled unit integrate the
   house load by +20 mV per frame up to `Vbat_max` (125 frames from 26.5 V). The existing test
   forbids this.

Needed before implementing:

- Distinguish *idle but able* (enabled, PV headroom, duty above minimum, current ≈ 0) from
  *disabled*, *PV-starved* and *actuator-limited*. Only the first may track the bus.
- Make anti-windup directional, with separate rules for downward (EOC) and upward (load-follow)
  moves, and an explicit upper bound for each regime.
- Define what happens to the filter and glide state at each transition.
- Add a separate restart path. A fresh termination blocks re-arming a disabled converter, and
  the stuck watchdog excludes `chargeHold()` (`src/main.cpp:1159`, `:845`), so a change to the
  pin alone cannot bring a stopped unit back.

Do not mark load-following Phase 1 as superseded until this exists.

## Tests

Run the production `Charger` code and the MPPT limiter selection against a plant model, not a
standalone lag model. Use `test/test_charger.cpp` style for the charger and a host stub for the
coupled run. Cases:

- **Sharing and arithmetic.** Steady sibling operation with unequal Z and unequal PV, checking
  I1−I2 = ΔV/Z, both conducting only above the threshold current.
- **Historical failures.** Both past authority failures: downward ratchet (`32540e3`) and
  re-injection limit cycle (`12d5876`).
- **Holds overnight.** Cold and partial hold overnight, with no upward windup (keep the
  `test_charger.cpp:756` assertion).
- **Conflicting bounds**, stale BMS, stale or NaN Iout, and a corrupt huge Iout. Assert the
  effective target stays above 0.9·Vout_max() and the controller output sign is correct.
- **Restart.** Start-up after termination, dawn start, and filter reset on re-enable.
- **Off path.** With `vout_droop=0`, a golden trajectory equal to the current code.
- **Stability.** The instability case at an α where the measured inner loop rings or diverges
  (known-bad). Define the "pick up within N ticks" bound from measured loop timing.

## Live acceptance (fry/flat, with confirmation, after bench)

Acceptance criteria apply only while both units have PV headroom and the bank current is above
the sharing threshold:

- each unit carries > 25 % of the current;
- neither sits at 0 W for more than 10 min while the other is above 100 W *and* the idle one has
  headroom (check `Ui`/sweep data);
- no `vpPin` oscillation;
- no ADC or lag errors;
- termination still latches with house load present.

## Guard checklist

1. **Unevaluable input.** NaN or stale Iout removes droop and is logged as a degraded mode. A
   NaN bus measurement freezes the clamp. A target that is non-positive or below 0.9·Vout_max()
   is refused.
2. **Monotonicity.** Larger Iout never raises Vset, within the clamps. The far tail (corrupt huge
   current) is bounded by the droop cap.
3. **Preconditions.** `vout_droop` must be finite and in [0, 0.1] Ω (`assert_throw`). The active
   value must be checked after restart. Test the call site, not just the helper.
4. **Source of truth.** Telemetry `I` may not be Iout (step 0).
5. **Persistence.** None, except the conf key. Filter state is reset on every transition.
6. **Provenance.** Log the droop value and degraded-mode entry and exit.
7. **Known-bad.** Corrupt-current, instability and night-windup cases must fail without the fix.
8. **Fix vs mute.** Droop changes the actual current split. Check this in InfluxDB per device,
   not from a log line.
9. **Runtime cost.** Measure the `rtcount` of the extra multiply and EWMA in `mppt.update`.

## Firmware fix 1: shared deficit for the termination release (implemented 2026-10-30, not deployed)

Cause (see the Step 0 console check): each unit integrated `ibat` on its own. On 10-30 flat
reached the 42 Ah release first. Its charging then pulled fry's deficit back down (41.58 Ah and
falling), so fry stayed `TERMINATED` at the yield floor until the next full charge.

Change:
- New `mqtt.conf::bat_charge_topic` (batmon-ha: `bat_caravan/mosfet_status/capacity_ah`, Ah
  remaining).
- `CoulombCounter` samples it on every cell frame and takes the full-point reference from the
  sample on the latching frame. While the sample is fresh (< 120 s), the deficit since full is
  `ref - charge`, signed and never clamped. Otherwise the integrator is used, as before.
- The release now compares the deepest deficit since full (`updateDepth()`). With the
  integrator this is equivalent to the old check, because the check runs every frame.
- `status` shows the source as `(bms)` or `(ibat)`, and the DoD line shows the depth.
- Files: `src/etc/coulomb_counter.h`, `src/charger.h`, `src/cli.cpp`, docs (`reference/config/mqtt.md`,
  `guide/charging/termination.md`), and `conf-editor.html`.

Verification:
- Host test `test/host-stub/coulomb-test.cpp` passes. The integrator baseline reproduces the
  deadlock (known-bad), BMS mode releases both units on the same frame, and the deficit is
  signed and unclamped. It falls back to the integrator when the BMS charge is stale and when
  there is no topic.
- Firmware (`build-chg`, 12 % slot free) and test image (`build-chg-tests`) build.
- The on-target test `test_mqtt_bat_charge_drives_deficit` is written but **not run** (no bench
  board attached).

Guard checklist:
1. **Unevaluable input.** An invalid payload is ignored. A stale or absent charge falls back to the
   integrator, which is today's behaviour. A stale charge at the moment of a full makes the
   reference the first later fresh sample, so the release comes late. The voltage-floor backstop
   still applies.
2. **Monotonicity.** Depth only grows until the next full charge, so a deeper discharge never
   un-releases.
3. **Preconditions.** Sampling sits in `setVcellHigh`, the MQTT cell callback. The on-target test
   covers that path but has not been run.
4. **Source of truth.** The BMS counter. Its drift is common to both units and re-referenced at
   every full charge.
5. **Persistence.** RAM only. After a reboot, the reference is the first sample, so units that
   rebooted at different times disagree until the next shared full charge.
6. **Provenance.** `status` prints the source.
7. **Known-bad.** The integrator baseline deadlocks in the host test.
8. **Fix vs mute.** It changes the release quantity itself.
9. **Runtime cost.** A few float operations per BMS frame on core 0, none on the RT core.

Residual risk: units that latched on different frames (for example, one rebooted in between)
hold different references. The authority-yield floor (`src/charger.h:534`) is unchanged, so
during *shared* termination flat still wins because of its Vout error.

Deploy (needs Fab):
- set `mqtt.conf bat_charge_topic` on fry and flat;
- OTA both. Their June firmware is far behind `main`, so follow the `-n`/`-m` OTA procedure,
  `dc 0` at high power, and confirm in the after-table.
