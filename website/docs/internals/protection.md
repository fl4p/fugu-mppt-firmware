---
title: Protection
sidebar_position: 2.5
---

# Protection

Protection stops the converter when a measurement leaves its safe range. It runs in the real-time loop on core 1
before the [control loop](control-loop.md) step, and it is separate from the PD limiters: a limiter reduces duty to
hold a value at its limit, while a protection trip turns the half-bridge off.

## Check scheduling

The real-time loop calls two functions on every new sample while the converter is switching, in MPPT, PSU and
manual PWM modes. The control step that follows runs only on a new Vout sample:

- `mppt.protect()` checks the fast electrical faults on the latest sample.
- `mppt.protectLf()` checks supply voltage, battery detection, temperature and the wire-fault latch.

Both run on the same sample, so `protectLf()` also runs after `protect()` has tripped. Both are skipped while the
converter is off or while the sensors calibrate; calibration holds the converter off. While it is off, the
[start conditions](#start-conditions) decide whether it may start again.

Watchdogs in `src/main.cpp` sit outside these functions and cover the sampler and the loop:
[ADC stall](#adc-and-loop-watchdogs), missing samples, loop latency and an output stuck near zero.

## Trip and backoff

Most trips call `shutdownDcdc(who, seconds)`, which:

1. Releases the LV power-good output and opens the backflow switch.
2. Disables the converter (both gates off).
3. With a backoff above 0: ends a running sweep, arms the backoff timer and logs `backoff <n>s [<who>]`.

The log line appears only when the backoff is newly armed or made longer, and carries ` mid-sweep` when a sweep was
running. A repeated trip does not extend a running backoff unless it asks for a longer one.

A failed check also makes the loop call `stopAndBackoff(4)`:

- It holds the next start for at least 4 s (100 ms in PSU mode). This timer is not reported as a start-block reason.
- If the converter is still on or a sweep is running, it calls `shutdownDcdc` with a 4 s backoff, logged as
  `backoff 4s [stopAndBackoff]`.

The converter restarts once both timers have expired and the [start conditions](#start-conditions) pass, so a 5 s or
30 s trip restarts after 5 s or 30 s. In MPPT mode the restart is a new sweep. It waits while a fresh BMS report says
the battery is full, and up to 12 s for the first BMS report when a BMS source is configured.

A sweep that finds no usable power ends with `backoff 30s [sweep-no-mpp]`, logged at info level. It uses the same
mechanism but is not a fault. Each further sweep without usable power doubles the backoff (30, 60, 120, 240 s, then
300 s); a sweep that finds an MPP resets it to 30 s. A battery that comes back during the backoff ends it and resets it
to 30 s: the output voltage was outside the battery range (below 2.5 V per cell or above 1.03 × `vout_max`) and has been
back inside it for 1 to 2 s. A battery connected all along, as at dawn, never leaves that range. After a sweep the Vout cap
ended, the cap rising above Vout ends the backoff too and starts a sweep at once; see [MPPT tracker](mppt-tracker.md).

### PSU mode

In PSU mode, including the PV simulator, output over-voltage (`Vout-OV`) and supply under-voltage (`supply-UV`) trips retry
fast, because a load step can cause them:

| Trips within 60 s of the first trip in a window | Backoff |
|---|---|
| 1–4 | 100 ms |
| 5–8 | The trip's normal backoff |
| More than 8 | 30 s, and the converter stays latched off (`psu-latch`) |

Re-issuing `psu <V>` or `pv <isc> <voc>`, or leaving PSU mode, clears the latch and the trip count.

Trips with their own backoff, such as `Vin-OV`, `Iout-OC`, `temp`, `temp-stale` or `wire-trip`, keep it in PSU mode.
Stops without one (loop latency, calibration failure) may retry after 100 ms in PSU mode, once their start condition
clears.

## Trip reference

The table lists the checks in the order `protect()` and `protectLf()` evaluate them. Thresholds use the
[`limits.conf`](../reference/config/limits.md) values in their converter roles (Vin max, Vout max, Iin max, Iout max);
the side keys `hv_max`, `lv_max`, `hv_i_max` and `lv_i_max` map to these roles through `converter.conf::topo`.

| `who` tag | Condition | Backoff |
|---|---|---|
| `hw-OV` | INA226 bus over-voltage alert, or the alert could not be armed, see [Hardware over-voltage alert](#hardware-over-voltage-alert) | 5 s |
| `psu-setpoint-infeasible` | PSU setpoint no longer reachable: a boost setpoint below Vin, or a setpoint above the OV limit | 30 s, latched |
| `Vin-OV` | Vin sample > Vin max | 5 s |
| `BMS-cutoff` | The signature of a BMS opening its switch: Iout median below 25 % of its average (average > max(1 A, 5 % of Iout max)), Vout median more than 2 % of `Vbat_max` above its average, and Vin median above its average. Buck, battery voltage known, not PSU mode | 30 s |
| `Vout-OV` | Vout sample > the [OV threshold](#output-over-voltage-threshold) | 5 s |
| `Iin-OC` | Iin sample > 1.3 × Iin max | 5 s |
| `Iout-OC` | Iout sample > 1.5 × Iout max, median > 1.25 × Iout max, or average > 1.15 × Iout max | 30 s |
| `revI-highAvg` | The last two samples of the physical current sensor below −1 A while the average Iout is above 10 A. Not in forced PWM or during `selftest bflow` | 5 s |
| `revI-ewm` | Average current of the physical current sensor below −1 A | 5 s |
| `Vout>Vin-avg` | Buck: average Vout > 1.25 × (average Vin + 1 V) | 5 s |
| `Vout>2Vin` | Buck: Vout sample > 2 × (Vin + 0.5 V) | 5 s |
| `short` | Average Iout > `iout_short` while average Vout < 1 V | 30 s |
| `highI-bflowOpen` | Boards with a backflow switch: more than 6 A while the backflow switch is open or the duty cycle is below 10 % | 5 s |
| `highD-bflowOpen` | Boards with a backflow switch: duty cycle above 33 % with the backflow switch open, unless the DCM current estimate stays below 1 A. A buck sweep that reaches this point at near-zero current ends the sweep instead | 5 s |
| `highI-noSyncRect` | More than 6 A with synchronous rectification off | 5 s |
| `minpulse-M` | Buck in DCM: Vout/Vin above the low-side minimum-pulse stop ratio, or reverse current with Vout/Vin above the start ratio, for 4 fresh Vout samples (at once if DCM is entered above the stop ratio). See [Diode Emulation](diode-emulation.md#ls-minimum-pulse-near-unity-ratio) | 5 s |
| `Vr-sensor-fail` | Duty cycle well above what Vout and Iout imply while the current stays below 1 % of Iout max: a sensor or half-bridge fault. `reverse_current_paranoia` only; not in PSU mode while Vout is between half the setpoint and the setpoint | 5 s |
| `supply-UV` | Board supply, max(Vin, Vout) − 0.3 V, below 9 V. Skipped in manual PWM | 5 s |
| — | Not PSU mode, no battery voltage known (`charger.conf::vout_max`, `vset` or an earlier detection), and none can be detected from Vout. Starts a new sensor calibration. No `backoff` line outside a sweep; during a sweep the sweep ends with `backoff 4s [stopAndBackoff] mid-sweep` | 4 s |
| `temp-stale` | A fitted NTC or MCU temperature sensor is open, shorted or silent: 3 readings in a row outside −40…150 °C (about 9 s), or no valid reading for 90 s. A shorted NTC trips `temp` instead if the ADC saturates below the supply rail. An unfitted NTC never trips | 5 s, then until a valid reading |
| `temp` | NTC or MCU temperature > `temp_max`, logged as `Temp … > …°C, shutdown` | 5 s |
| `wire-trip` | Wire-fault guard tripped (`wire_protect = 2` only), see [Output wire-fault protection](../reference/config/limits.md#output-wire-fault-protection) | until `wire-reset` or reboot |

### Other stops

These stops come from outside `protect()` and `protectLf()`:

| `who` tag | Condition | Backoff |
|---|---|---|
| `ctrl-nan` | A PD controller output is not finite while the converter switches above 1 % duty | 5 s |
| `calib` | Sensor calibration is running. No `backoff` line | none |
| `stop` | A switch to manual PWM at duty 0 that stops at once, without a ramp: `ota <url>`, coil measurement, backflow switch test. No `backoff` line | none |
| `stopAndBackoff` | An OTA push over BLE stops the converter before it halts the sampler | 5 s, start held 10 s |
| `calib-fail` | Sensor calibration fails. The converter is already off, so a `backoff` line appears only when a sweep was calibrating: the sweep ends with `backoff 4s [calib-fail] mid-sweep` | Start held 4 s (100 ms in PSU mode) |

Two latches act outside the checks as well. Neither logs a `backoff` line, and both hold until `selftest clear`:

- `hs-short`: the [high-side short check](#high-side-short-check) runs on every sample of an idle buck and turns all
  gates off itself, logged as `HS short (…): …`.
- `bf-fail`: a failed [backflow switch test](#backflow-switch-test) only refuses the next start.

### Sub-trip reverse current

Some reverse-current conditions reduce rectification instead of tripping:

- Two output-current samples below −1 A with a low average: open the backflow switch and set the low-side FET to its
  minimum pulse.
- A buck with `reverse_current_paranoia` and a filtered current below −0.05 A: the same, to prevent the low side from
  boosting energy back into the input.

The converter keeps switching in both cases.

While `selftest bflow` runs, both reductions and the `highD-bflowOpen` check are suspended, because the test drives
reverse current on purpose. The test stops the converter itself when a sample falls below its trip threshold, −0.3 A
with the backflow switch off or −0.5 A with it on. `revI-highAvg` is suspended as well. All other trips stay active,
including `revI-ewm` and `highI-bflowOpen`. See
[Backflow switch test](#backflow-switch-test).

### High-side short check

A shorted high-side FET connects the input to the output whatever the duty cycle. A buck checks for it while off.
While switching, a short stops the converter within milliseconds through other trips: each low-side pulse shorts the
input capacitors through both FETs, so Vin collapses and `Vout>2Vin` or an over-current trip follows.

With both switches off, the high-side body diode blocks current from the input to the output. A forward
output current while the converter is off therefore means the high-side FET conducts. This check runs on every new
sample of a buck that has been off for at least 0.5 s, only on boards that declare `board.conf::bflow_bat=1`: a backflow
switch between the converter output and the battery, and the current sensor on the battery side. Without that switch,
the battery discharges into the panel through the high-side body diode at night, so the check could not learn its zero.

The check judges the raw current, before the idle offset calibration subtracts its zero. That calibration accepts a
zero of up to 5 % of Iout max, so it would otherwise absorb a small short. The check keeps its own zero, learnt while no
current can flow: the backflow switch commanded off, Vin more than 2 V below Vout, and the raw current within half the
large-current threshold below (5 % of Iout max, at least 0.3 A).

A sample is suspect when, with $I$ the current above that zero and the window $w$ = max(1.5 V, 4 % of Vout):

- $I$ exceeds 10 % of Iout max (at least 0.6 A), whatever the voltages, or
- $I$ exceeds max(0.3 A, 1 % of Iout max), Vout is above 5 V, and Vin lies within −$w$ and
  $w$ + 0.05 Ω × $I$ of Vout, or
- Vout is above the [OV threshold](#output-over-voltage-threshold) and Vin lies within the same window, at any current.
  This covers a battery that has been disconnected, for example by its BMS: the shorted high side then holds the
  output at the panel voltage with no current flowing. Off in PSU mode.

The window covers two voltage dividers with 1-2 % gain error each (2.4 V at 60 V).

Suspect samples hold the start (`hs-check`). They latch the fault after a hold that depends on the criterion:

| Criterion | Hold | Trade-off |
| --- | --- | --- |
| current | 3 s | a short passes panel current to the battery, which a healthy idle converter cannot |
| voltage only | 60 s | with a real short and the battery disconnected, the loads see the panel voltage for about 60 s before the latch; a false latch needs `selftest clear` |

The voltage-only hold is 60 s with or without the [low-side clamp](#low-side-clamp): an earlier latch would only turn the
gates off, which cannot stop a shorted high side, and Vin is then far above the clamp's 9 V limit.

Vin leaving the window before the hold ends drops the suspicion without a latch. A short holds Vout at Vin, while
another charger on a shared output bus rarely holds the battery above the OV threshold for a whole minute.

#### Latched fault

A latch:

- commands the backflow switch off and holds it off,
- refuses every start (MPPT, PSU and manual PWM), and
- turns all gates off. With the low-side clamp armed, the clamp then waits for a low input voltage.

The latch is written to flash right away (the converter is off then) and is restored after a reboot, with all gates
off. A stored record from an older firmware keeps its latches; one that cannot be read counts as a latched high-side
short. A latch stored on a board that now runs as a boost is dropped. Only `selftest clear` clears it, on any topology.
After a clear, the start stays blocked until a clean sample has been seen.

Unevaluable samples (any input NaN) give no verdict and hold the start; they never count as clean. A start within 0.5 s
of a stop, for example a sweep restart, is not held, because the converter was switching up to then. Any other start
waits for a clean sample.

#### Low-side clamp

The clamp turns the low-side FET on and holds the high side off, so the input current flows through the shorted high
side and the low side instead of into the battery. It is off by default.

At a high input voltage the clamp would discharge the input capacitors through the shorted high side and the low side,
at a peak current above the low-side pulse rating. It therefore engages only at night: a latch turns all gates off and
leaves the clamp pending. The clamp is armed when all of these hold:

- `board.conf::hs_clamp=1` and `bflow_bat=1`, a backflow switch is fitted and the converter is a buck,
- the output current sensor can read reverse current (see [Toggle variant](#toggle-variant)), because the clamp is
  released on battery current flowing backwards,
- not PSU mode, where a stiff supply would be shorted,
- the latch came from the voltage criterion, or the check has learnt its zero, so a sensor offset cannot clamp,
- the blocks phase of the [backflow switch test](#backflow-switch-test) passed within the last 36 hours (a toggle
  variant pass does not count), no `bf-fail`
  is latched, and
- no clamp has met battery current since the last blocks pass.

| State | Gates | Leaves the state |
| --- | --- | --- |
| pending | all off | to the ramp once Vin (EWMA) has stayed below 9 V for 60 s without a break; a NaN or a sample at or above 9 V restarts the 60 s. To all gates off when a check every 250 ms finds the clamp no longer armed, or on a hardware over-voltage alert |
| ramp | high side held low, low side pulsed at the switching frequency | to the hold after 20 ms; to all gates off on battery current, a gate shutdown from any other path, a hardware over-voltage alert, or the clamp no longer armed |
| hold | low side on, high side off | to all gates off on battery current, a hardware over-voltage alert, or when the clamp is no longer armed |

The ramp widens the low-side pulse linearly from 200 ns to the full period over 20 ms. The 200 ns is the realized
on-time: where the gate driver adds a hardware dead-time to the low-side turn-on, the commanded pulse is longer by that
dead-time. The width is updated once per sample, so the ramp is a staircase, about ten steps at 450 samples per
second. One step advances the ramp by at most 4 ms, so a stalled loop cannot skip it. Both gate drivers (MCPWM and
LEDC) support it.

Battery current below −0.5 A when the ramp is about to start means the backflow switch already conducts. The ramp
then does not start; all gates stay off and the clamp is disarmed as after a release.

While ramping or clamped, a current below −0.5 A, or more than 0.5 A above its value at the start of the ramp, for two
finite samples in a row releases the clamp and turns all gates off. Such a current means the backflow switch conducts
and the clamp is shorting the battery. NaN samples hold the count. A release is written to flash right away and
discards the last blocks pass, so the clamp stays disarmed, across reboots and `selftest clear`, until a new blocks
pass. If the write fails, the clamp stays disarmed until the next reboot and the write is retried with a back-off.

Every 250 ms the hold is re-applied, in case something else forced the gates off, after checking the conditions
again; it does not start a new ramp. If they no longer hold (PSU mode, the pass older than 36 hours, a new `bf-fail`),
the clamp is released and all gates turn off.

:::warning

Validate `hs_clamp` on a bench before enabling it, with a shorted high side on a board that may be destroyed. Grounding
the switch node at once rings the output capacitor through the coil. On boards where the backflow switch source follows
that node and its gate is only pulled to ground through a resistor, the ring can drive the source well below ground and
turn the backflow switch on, so the battery feeds the ring. The 9 V limit and the pulse-width ramp are meant to avoid
that ring. Record the backflow switch source voltage and its gate-source voltage with an oscilloscope while the ramp
runs, at an input voltage just below 9 V.

:::

Limits of the checks:

- Without a learnt zero, a sensor offset above the 0.3 A threshold reads as a short when Vin is near Vout (dawn, dusk).
  A board that boots at such a time can latch falsely, without the clamp. The zero is learnt during the first night.
- Below 9 V and with a per-period ramp, the clamp's peak current is about a third of the low-side pulse rating
  (circuit simulation).
- The 9 V limit relies on the Vin reading. A divider whose reading floors near 9 V in the dark never arms the clamp.

### Backflow switch test

`selftest bflow` checks a battery-side backflow switch in two phases. Both take the converter over in manual mode, ramp
the duty up in diode emulation until the coil current is continuous, then engage forced PWM (complementary low side)
and step the duty down until D × Vin is 1 V below Vout. Each step waits a quarter of the power sensors' settling time
(20-500 ms), so a run takes 10-30 s on fast filters and up to about 3 minutes on slow ones:

| Phase | Backflow switch | Pass | Fail |
|---|---|---|---|
| blocks | Commanded off at least 20 ms before switching | No sample below −0.3 A and a minimum of at least −0.2 A | A sample below −0.3 A, or a minimum below −0.2 A |
| conducts | On | A sample below −0.5 A | No such sample by the end of the ramp |

The thresholds are trip levels, not bounds: one duty count moves the current by up to about 1 A on a low-resistance
path. The blocks phase engages forced PWM at once and steps down faster while the current exceeds 2 A, because the
switch's body diode carries the forward current. It also engages at 4.5 A or at the duty cap without waiting for CCM; the 4.5 A limit stays clear of
the 6 A `highI-bflowOpen` trip. The conducts phase engages only in CCM, after 150 ms for the switch's gate supply: in
DCM, D × Vin is below Vout and forced PWM would pull reverse current at once.

The ramp's cap is the proven-forward duty or, with the switch on, the duty the converter ran at before the test,
whichever is higher. The conducts ramp may climb past it: leaving DCM takes more current than staying in CCM (half
the coil ripple below 0.9 × the current instead of 1 ×), so a live point that held CCM need not reach it again after
the restart. Above the cap, the ramp continues one count per step, at most 1/8 further, while all of these hold:

- the forward current stays within the search limit: the lowest of 5 A, half of `limits.conf` `lv_i_max` and the
  charger's current limit. 5 A keeps 1 A to the 6 A level the firmware allows through the switch's body diode. The
  control loop checks every sample (median of 3) and stops the converter above it,
- Vin stays at least 1.2 × Vout and 3 V above it (it falls as the duty passes the panel's maximum power point),
- the climb takes at most 60 s.

A breach aborts the run without a verdict, as does reaching 1/8 above the cap without CCM.

A blocks FAIL ends the run: the conducts phase is not run on a switch that leaks backwards. The FAIL also discards the
last blocks pass, which disarms the low-side clamp. Both are written to flash as soon as the converter has stopped.

A run aborts, without a verdict, when a protection trips, the input voltage collapses, the mode changes, a high-side
short latches, a step times out or `selftest abort` is given. After a mode change the new mode stays; otherwise the previous mode is restored. The
test keeps backoffs and the charger's latches as they were.

A failed phase latches `bf-fail`: every start is refused until `selftest clear`, also after a reboot.

#### Toggle variant

The two phases need a current sensor that reads reverse current, and a battery. The test runs a toggle variant
instead when either is missing:

- the output current sensor cannot read below −1 A: an INA226 current channel counts as bidirectional; another ADC
  only when its transform rises with the raw value and the raw zero, less the learnt zero offset, maps below −1 A (an
  offset amplifier). The ADS1x15 driver reads single-ended, so it counts like the internal ADC.
- Vout (battery side) reads below 2 V when the test starts: no battery. The Vout divider bleeds an open battery side
  toward 0 V; a stale charge on the output capacitors makes the test refuse (Vout implausible) until it has decayed.

The variant holds forced PWM at a fixed duty and turns the switch off and on 3 times. Each reading waits for the
settling time of the current, Vin and Vout filters plus 20 ms after the edge, at least 60 ms. The test refuses to start when that wait
exceeds 1 s (sensor filters too slow for the toggle).

The criterion is y = D × Vin − Vout, the voltage across the loop. With the switch on, y is the resistive drop. While
off, a blocking switch adds its body-diode drop, or, on a stiff source, the current collapses instead.

| Case | Operating point | Pass | Fail |
| --- | --- | --- | --- |
| battery | ramp as above with the switch on, to CCM with at least 1 A (0.5 A at the duty cap), then forced PWM | every off reading: y at least 0.1 V above both on neighbours, or the current below 30 % of both | an off reading that does neither, with steady neighbours |
| no battery | switch off, low side at its minimum pulse (an asynchronous buck), the duty climbing from its minimum until Vout has risen 1 V (at most to D × Vin = Vout + 3 V); the real-time loop then engages forced PWM, the duty goes above the output and ramps to min(Vin/2, 0.8 × Vbat max) − 3 V | every off reading: y at least 0.1 V above both on neighbours | an off reading below that, with steady neighbours |

Steady neighbours means the two on readings around an off reading agree: y within 0.15 V and, with a battery, the
current within 15 %. Otherwise the run aborts without a verdict.

A switch stuck closed steps 0.00 V. The 0.1 V threshold lies below a hot body diode's drop at a few hundred
microamperes (0.15–0.35 V) and above the error of a settled reading. Without a battery, each turn-off also raises the
duty by 1 V, so a blocking switch leaves the output one diode drop below the new D × Vin.

The variant never lowers the duty with the switch on. The real-time loop owns the switch and these guards, because
reverse current through the closed switch is invisible to a one-direction sensor:

- with a battery, a stop when y falls below −0.3 V, and with the switch on, when the current falls below 30 % of the
  last on reading. The current check starts after the median filter's depth plus the gate's turn-on time in fresh
  current samples (3 + 10 ms × the sample rate).
- without a battery, a stop at a current above 0.5 A, at Vin 3 % above its value at the start, which only energy
  from the output side can cause, and with the switch on in forced PWM at y below −1 V. Vin and y use the raw
  samples, two in a row, not the filtered values.

Each stop aborts the run, as does a Vin drop of 1 % before a turn-on. Without a battery, the test runs only on
request, never as the daily run. It refuses when BMS cell data is fresh, when a battery has been seen since boot
(BMS data, or Vout above 9 V with the converter off for 2 minutes, so a pack may sit behind an open BMS), when Vbat max is unknown, or when
min(Vin/2, 0.8 × Vbat max) is below Vout + 5 V. It aborts when BMS data arrives or Vout does not rise within 3 s
(a pack behind an open divider). Without the low side, a switch stuck closed cannot pull reverse
current from such a pack. A raised output after a no-battery run does not count as a battery. A FAIL latches `bf-fail` like a blocks FAIL. A PASS counts as a blocks pass for the
verified state, but does not arm the [low-side clamp](#low-side-clamp).

A pack that reconnects without BMS data while the switch is on, at a voltage above D × Vin, back-feeds until Vin rises
(a few samples). With a battery, a sudden rise of the battery voltage right after a turn-on is unobservable for up to
the current check's hold.

**Daily run.** Boards with `board.conf::bflow_bat=1` run the test on their own once a day, at the first opportunity
when all of these hold for 60 s:

- MPPT mode, no limiter active, no sweep, no backoff, a battery present,
- the live operating point in CCM, with the current at least 1.5 × the CCM boundary (half the coil ripple), the
  current the test needs: with the switch on, its ramp may climb to this live duty,
- Vin at least 1.2 × Vout and 3 V above it,
- no rectifier calibration, coil measurement, OTA or USB drive.

The next run is due 20 hours after the last run with two verdicts, at the first opportunity with enough power. An
aborted conducts phase does not count as a run; the blocks pass it may have produced still counts for the verified
state. An aborted run retries after 30 minutes, at most four times, then waits 20 hours. A refused start waits
5 minutes. The time of the last blocks pass is stored in flash. Without a pass for 36 hours, for example after days
without sun, the switch counts as unverified: the low-side clamp is off, but conversion continues.

### Output over-voltage threshold

The `Vout-OV` threshold depends on the mode, in this order:

1. An explicit limit from the console command `ovset`: min(`ovset`, Vout max).
2. PSU mode: setpoint × 1.03 (× 1.5 without `reverse_current_paranoia`), capped at Vout max. The PV simulator uses its
   open-circuit voltage instead of the setpoint. After the setpoint is lowered, the old setpoint stays the base until
   Vout falls inside the new band.
3. A known battery voltage: `Vbat_max` × 1.03 (× 1.5 without `reverse_current_paranoia`), capped at
   Vout max.
4. Otherwise Vout max.

If `Vout-OV` trips keep recurring with no clean protection pass for more than 20 s (not PSU mode), the firmware clears
the detected `Vbat_max` and detects it again after a new sensor calibration. A `Vbat_max` set with the console command `vset` is kept, and the firmware logs an error instead.

### Hardware over-voltage alert

With `sensor.conf::ina22x_ov` set, the INA226 compares its bus voltage against that limit in hardware and raises its
ALERT pin. With the MCPWM gate driver, the ALERT interrupt forces the gates off immediately, without waiting for the
real-time loop. The loop then trips with `hw-OV`. An alert that arrives while the converter is off is discarded, so it
cannot block the next start. While the [low-side clamp](#low-side-clamp) is pending, ramping or held, the alert is kept
instead and releases the clamp, turning all gates off.

The loop also trips `hw-OV` when the alert cannot be armed or the INA226 has lost its alert configuration, for example
after an I2C error or a chip reset. Without the MCPWM gate driver the firmware warns at boot that the alert cannot force
the gates off. See [`sensor.conf`](../reference/config/sensor.md).

### Overshoot on battery disconnect

When a BMS opens the battery path while the buck delivers current, the coil current has nowhere to go but the output
capacitor. Stopping the switches does not stop it: the low-side body diode keeps the coil discharging into the output.
The output therefore rises to about

$$
V_\text{pk} \approx V_0 + I \sqrt{L / C_\text{out}}
$$

with $V_0$ the battery voltage and $I$ the output current at the moment the battery opens. Over-voltage trips, in
software or hardware, cannot keep the output below this value.

Simulated peaks for an 8S LFP pack (`Vbat_max` 29 V) charged at 28 A from 64 V, with nothing else on the output:

| L | C<sub>out</sub> | Vout sampled at 200 Hz | Vout sampled at 5 kHz |
|---|---|---|---|
| 30 µH | 470 µF | 38.0 V | 35.5 V |
| 30 µH | 560 µF | 37.3 V | 34.9 V |
| 100 µH | 470 µF | 41.4 V | 41.4 V |

The coil energy sets the peak: a faster trip lowers it by at most a few volts, and only for small coils. Absorbing that
energy ($\tfrac{1}{2} L I^2$, about 12 mJ at 30 µH and 28 A) or adding output capacitance lowers it further. A fast trip
still limits the damage, because a high side that keeps switching with the low side off drives Vout on towards Vin.

The table comes from the host simulator. A measurement with the battery at 28.5 V showed 36 V at the output for
400 ms. Loads on the charger side of the BMS reduce the peak, but they are also exposed to it.

## ADC and loop watchdogs

The ADC stall, ADC reset, no-sample and calibration checks run in the real-time loop. The loop-latency and stuck-output
watchdogs run in the low-frequency loop on core 0.

| Watchdog | Condition | Action |
|---|---|---|
| ADC stall | The ADC reports an error, or no fresh sample for 200 ms outside calibration | Reset the ADC peripherals (at most every 300 ms). After 800 ms of stall, if the converter is switching and no backoff runs: stop with a 5 s backoff and hold the start for 16 s. After 60 s: restart the system |
| ADC reset | A requested ADC reset | Stop during the reset; restart 6 s after it (30 s if the reset failed) |
| No sample | No sample at all 20 s after boot | Disable the converter, switching or not, and log `Never got a sample! Please check ADC`. After 15 min: restart the system |
| Loop latency | While switching: Vout sample rate below half its nominal rate for 3 consecutive 3 s windows. Always on, unless Vout is a virtual sensor | Stop with a 4 s backoff (100 ms in PSU mode) |
| Calibration failure | Sensor calibration fails | `stopAndBackoff(4, "calib-fail")`: hold the start for 4 s (100 ms in PSU mode). During a sweep, end the sweep and log `backoff 4s [calib-fail] mid-sweep` |
| Stuck output | Average Iout within ±0.3 A despite input headroom (buck: average Vin more than 8 V above average Vout; boost: more than 2 V above Vin min), outside manual PWM and a no-MPP backoff. In PSU mode Vout must also be more than 2 V below the setpoint; otherwise the charger must not be holding the charge (for example terminated, in a CV hold or too cold) | After 30 s: release the CV-floor latch, which also clears the backoff. After 2 h: restart the system |

The latency watchdog ignores manual PWM, the first 2 s after a calibration, and the first samples after boot. A single
slow window logs a warning without stopping the converter.

## Start conditions

While the converter is off, `startBlockReason()` must return no reason before the loop starts it. In MPPT mode the
low-frequency status line logs the blocking reason as `START blocked: <reason>`, when it changes and at most every 30 s
otherwise. `temp-stale` is not logged before the first input-voltage sample, when every sensor is still empty.

The temperature and supply checks use a margin above their trip level, so the converter does not restart right at the
threshold. Manual PWM skips this table, but `restart`, `hs-short`, `hs-check`, `bf-fail`, `conf-save`, `backoff`,
`wire-trip`, `temp-stale`, `temp`, `minpulse-M` and `idle profile` also hold it off; `backoff` and `wire-trip` also stop
a running manual PWM. The table lists the reasons in the order they are checked:

| Reason | Blocks the start while |
|---|---|
| `psu-latch` | PSU mode latched off after repeated trips |
| `restart` | A system restart is under way. Also holds manual PWM |
| `hs-short` | The [high-side short check](#high-side-short-check) latched. Also holds manual PWM |
| `hs-check` | The high-side short check has not yet seen a clean sample after 0.5 s off, or a sample is suspect. Also holds manual PWM |
| `bf-fail` | The [backflow switch test](#backflow-switch-test) failed. Also holds manual PWM |
| `conf-save` | A rectifier-calibration result or a `pwr pwm` setting is being saved |
| `psu-boost-setpoint` | PSU mode, boost: the setpoint is below Vin |
| `psu-ov-conflict` | PSU mode: the setpoint conflicts with the OV limit |
| `backoff` | The backoff timer is running |
| `wire-trip` | The wire-fault guard is tripped |
| `temp-stale` | A fitted NTC or MCU temperature sensor has given 3 invalid readings in a row (outside −40…150 °C), no valid reading for 90 s, or none since boot |
| `temp` | NTC or MCU temperature > `temp_max` − 3 °C, or no MCU temperature |
| `Vin-Vout` | Not in PSU mode: a buck needs average Vin > average Vout + 1 V, a boost needs average Vin < average Vout + 1 V |
| `minpulse-M` | A buck start at the current Vout (or the PSU setpoint, if higher) would run the low-side minimum pulse above its start ratio |
| `Vout-still-OV` | PSU mode: Vout is still above 98 % of the OV threshold |
| `supply-UV` | Board supply below 9.5 V |
| `calibrating` | Sensor calibration is running |
| `filters-empty` | A Vin, Vout, Iin or Iout filter has not refilled after a calibration or ADC reset |
| `idle profile` | The idle power profile holds the converter off; the loop asks it to leave |

## Configuration

| Setting | Effect |
|---|---|
| [`limits.conf`](../reference/config/limits.md) | Voltage, current and temperature limits, short-circuit current, `reverse_current_paranoia`, wire-fault guard |
| [`charger.conf`](../reference/config/charger.md)`::vout_max` | Battery voltage that sets the output OV threshold |
| [`sensor.conf`](../reference/config/sensor.md)`::ina22x_ov` | INA226 hardware over-voltage alert |
| [`board.conf`](../reference/config/board.md)`::bflow_en`, `bflow_sd` | Backflow switch pin; a board with neither has no backflow switch |
| [`board.conf`](../reference/config/board.md)`::bflow_bat`, `hs_clamp` | Idle high-side short check and backflow switch test; low-side clamp |
| `ovset` (console) | Explicit output OV limit, RAM only |
