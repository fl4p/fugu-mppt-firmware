---
title: MPP Tracker
sidebar_position: 3
---

# MPP Tracker

The tracker finds and follows the maximum power point (MPP) of the solar input by moving the converter duty cycle.
It runs inside the [control loop](control-loop.md) only while no limiter is active.

## Phases

Tracking runs in three phases:

1. Global sweep: the duty cycle rises from 0 and the tracker captures the MPP. The sweep ends when a limiter
   becomes active (input undervoltage, output overvoltage, overcurrent, power limit) or the duty reaches its maximum.
2. Fast tracking: perturb & observe (P&O) around the captured MPP.
3. Slow tracking: P&O with a lower rate, smaller steps, and a smaller mean tracking error. A major change in
   power (clouds, partial shading) switches back to fast tracking.

A new global sweep runs every 30 minutes so the tracker does not stay on a local maximum, which happens with
partially shaded strings. A sweep takes 20–60 s, depending on the loop rate. Sweeping too often or too slowly
costs yield.

In low light, power changes between P&O steps stay below the tracker's thresholds, so P&O cannot follow an
MPP that moves while the light changes (see [Perturb & observe](#perturb--observe)). If the last sweep
captured less than 5 % of `limits.conf::p_max`, the tracker therefore starts another sweep once power has
doubled since that sweep, at most every 2 minutes. At dawn this adds a few short sweeps while the power is
still low.

The following state diagram shows the transitions between the phases:

```mermaid
stateDiagram-v2
    [*] --> Sweep: boot / sweep command / 30 min / low-light power x2
    Sweep --> FadeToMPP: limiter active or max duty
    Sweep --> Backoff: best power < 0.2 % of p_max
    FadeToMPP --> Fast: target duty reached
    Fast --> Slow: no new MPP for 30 s
    Slow --> Fast: power change
    Backoff --> Sweep: next start
```

## Global sweep

A sweep (`startSweep()` in `src/mppt.h`) runs these steps:

1. It disables the converter, clears the fade target, and resets the PD controllers.
2. It starts a sensor calibration. The periodic re-sweep doubles as periodic calibration.
3. Once calibration is done, it raises the duty each tick. `sweep_speed` caps the step, and the distance to the
   nearest limit scales it, so the ramp slows down when approaching a V/I/P limit.
4. It records the sample with the highest smoothed power as the MPP. It ignores samples below 0.2 % of
   `limits.conf::p_max` (2 W at 1 kW, `sweepMinPower()`), so a sweep in marginal light cannot "peak" at a
   near-maximum-duty phantom point.

The sweep stops as soon as any limiter produces a control mode (CV, CC, or CP), including the bounce at the maximum
duty. The captured MPP duty becomes the target, and the loop fades to it with a bounded step before handing over to
P&O.

If the best captured power is below that floor, the sweep commits no target. The converter shuts down for 30 s and
resumes through the normal start path. The firmware logs this backoff at info level as
`backoff 30s [sweep-no-mpp]`, not as a fault. A source that cannot deliver that power, such as a current-limited
bench supply, repeats this cycle indefinitely.

At the end of a sweep, the firmware writes "MPP Scan done" to the LCD and logs
`Stop sweep ... MPP=(<W>,<duty>,<V>)`.

## Perturb & observe

`Tracker::update()` (`src/tracker.h`) averages power and Vin over one tracker period, then compares the period
mean with the last reference power. These rules set the direction:

- If the power change exceeds the absolute or relative threshold and is negative, the direction reverses.
- For 150 ms after a reversal, the tracker does not judge the power change. The reference power follows the
  measurement instead. The filtered power still lags behind the old direction, and the input capacitor
  discharges while Vin falls and charges while it rises, so the output power dips right after a turn toward
  higher Vin. A decision in that window would reverse again, mostly toward lower Vin.
- If Vin moved by more than 5 % since the last reversal, the direction reverses as well. This handles cloud
  recovery, because tracking a light transient can drift far from the MPP. The 5 % refers to the higher of the
  two voltages, so a leg down and the following leg up have the same length.
- Below 1 W, the tracker always increases duty.

In low light the power change across one 5 % leg near the MPP is about 1 % of the power, below the
thresholds and close to the current-sensor noise. The tracker then oscillates within the 5 % band around
where it is. Both rules above keep that oscillation free of a bias toward the battery voltage: with a band
measured from each leg's start (5 % down, then 5 % of the lower voltage up), every cycle ended 0.25 % lower,
and the re-reversals after a turn walked Vin down in the same way.

The tracker keeps its own MPP record. It resets the record after 5 min, or when the current power is below 85 % of
the recorded MPP and the record is more than 30 s old.

Fast and slow mode use these parameters:

| Parameter              | Fast mode | Slow mode |
|------------------------|-----------|-----------|
| Update rate            | 30 Hz     | 2 Hz      |
| Step (control value)   | 0.5       | 0.02      |
| Min. power step, abs.  | 1.0 W     | 0.5 W     |
| Min. power step, rel.  | 1.5 %     | 0.15 %    |

The tracker enters slow mode when power is above 2 W (0.5 W to stay, hysteresis), no new MPP was found for 30 s,
and the direction reversed within the last 45 s. A tracker that moves steadily in one direction stays in fast mode.
At the slow-mode step, one pass across the band in which the power drops by the slow-mode threshold takes about
35 s; with a shorter limit the tracker fell back to fast mode in the middle of that pass.

On entry to slow mode, the duty jumps back to the MPP duty captured in fast mode. Output-power measurement (for
example, with a hall sensor) can be too poor and non-linear for slow P&O alone.

The console command `speed <x>` (0 ≤ x < 10) scales the tracker step (default 1.0).

## When the tracker does not sweep

The tracker skips the periodic re-sweep under these conditions:

| Condition                                             | Why                                                           |
|-------------------------------------------------------|---------------------------------------------------------------|
| PSU / PV-sim mode                                     | No MPP to track; the output is regulated to a setpoint        |
| A sweep is already running                            |                                                               |
| Charge hold (terminated, partial-charge ceiling, too cold) | Ramping duty from 0 would dump a charge pulse into a full pack |
| Control mode is CV (output-voltage limited)           | Same: the pack is full or the output is disconnected          |
| A protection backoff is pending                       | A sweep into an active trip timer would stall                 |

When a BMS cell-voltage topic is configured, the sweep at start-up and after a shutdown also waits for the BMS. It
is skipped while termination is active and the BMS data is fresh, and it waits up to 12 s for the first termination
decision. A fresh-to-stale BMS edge re-arms that wait at most once per 10 min, so a flapping BMS
cannot block the sweep indefinitely. See [Charge Termination](../guide/charging/termination.md).

`tracker.conf::target_duty_cycle > 0` skips the sweep entirely and holds a fixed duty (manual PWM).

## Configuration

The tracker reads these keys from `tracker.conf`:

| Key                                                                  | Default | Effect                                               |
|----------------------------------------------------------------------|---------|------------------------------------------------------|
| [`sweep_speed`](../reference/config/tracker.md)                      | 4.0     | Sweep speed; scales the per-tick cap and the limit-tracking gain (min 0.1) |
| [`target_duty_cycle`](../reference/config/tracker.md)                | 0.0     | Fixed duty fraction; > 0 disables sweep and P&O      |

Console: `sweep` starts a global sweep, `mppt` returns to automatic tracking, `speed <x>` scales the P&O step. See
[Console](../reference/console.md).
