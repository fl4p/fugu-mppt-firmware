---
title: Protection
sidebar_position: 2.5
---

# Protection

Protection stops the converter when a measurement leaves its safe range. It runs in the real-time loop on core 1
before the [control loop](control-loop.md) step, and it is separate from the PD limiters: a limiter reduces duty to
hold a value at its limit, while a protection trip turns the half-bridge off.

## Check scheduling

The real-time loop calls two functions on every control step while the converter is switching, in MPPT, PSU and
manual PWM modes:

- `mppt.protect()` checks the fast electrical faults on the latest sample.
- `mppt.protectLf()` checks supply voltage, battery detection, temperature and the wire-fault latch.

Both run in the same step, so `protectLf()` also runs after `protect()` has tripped. Both are skipped while the
converter is off or while the sensors calibrate; calibration holds the converter off. While it is off, the
[start conditions](#start-conditions) decide whether it may start again.

Three watchdogs in `src/main.cpp` sit outside these functions and cover the sampler itself:
[ADC stall](#adc-and-loop-watchdogs), missing samples and loop latency.

## Trip and backoff

Most trips call `shutdownDcdc(who, seconds)`, which:

1. Releases the LV power-good output and opens the backflow switch.
2. Disables the converter (both gates off).
3. With a backoff above 0: ends a running sweep, arms the backoff timer and logs `backoff <n>s [<who>]`.

The log line appears only when the backoff is newly armed or made longer, and carries ` mid-sweep` when a sweep was
running. A repeated trip does not extend a running backoff unless it asks for a longer one.

A failed check also makes the loop call `stopAndBackoff(4)`:

- It holds the next start for at least 4 s (100 ms in PSU mode). This timer is not reported as a start-block reason.
- If the check did not stop the converter itself, it stops it with a 5 s backoff logged as
  `backoff 5s [stopAndBackoff]`.

The converter restarts once both timers have expired and the [start conditions](#start-conditions) pass, so a 5 s or
30 s trip restarts after 5 s or 30 s. In MPPT mode the restart is a new sweep. It waits while a fresh BMS report says
the battery is full, and up to 12 s for the first BMS report when a BMS source is configured.

A sweep that finds no usable power ends with `backoff 30s [sweep-no-mpp]`, logged at info level. It uses the same
mechanism but is not a fault. Each further sweep without usable power doubles the backoff (30, 60, 120, 240 s, then
300 s); a sweep that finds an MPP resets it to 30 s. A battery that comes back during the backoff ends it and resets it
to 30 s: the output voltage was outside the battery range (below 2.5 V per cell or above 1.03 × `vout_max`) and has been
back inside it for 1 to 2 s. A battery connected all along, as at dawn, never leaves that range.

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
| `revI-highAvg` | The last two samples of the physical current sensor below −1 A while the average Iout is above 10 A. Not in forced PWM | 5 s |
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
| — | Not PSU mode, no battery voltage known (`charger.conf::vout_max`, `vset` or an earlier detection), and none can be detected from Vout. Starts a new sensor calibration; no `backoff` line is logged | 4 s |
| `temp-stale` | A fitted NTC or MCU temperature sensor is open, shorted or silent: 3 readings in a row outside −40…150 °C (about 9 s), or no valid reading for 90 s. A shorted NTC trips `temp` instead if the ADC saturates below the supply rail. An unfitted NTC never trips | 5 s, then until a valid reading |
| `temp` | NTC or MCU temperature > `temp_max`, logged as `Temp … > …°C, shutdown` | 5 s |
| `wire-trip` | Wire-fault guard tripped (`wire_protect = 2` only), see [Output wire-fault protection](../reference/config/limits.md#output-wire-fault-protection) | until `wire-reset` or reboot |
| `hs-short` | Latched high-side short, see [High-side short check](#high-side-short-check). Logged as `HS short (…): …`, no `backoff` line | until `selftest clear` |
| `bf-fail` | Latched failure of the [backflow switch test](#backflow-switch-test). No `backoff` line | until `selftest clear` |

### Sub-trip reverse current

Some reverse-current conditions reduce rectification instead of tripping:

- Two output-current samples below −1 A with a low average: open the backflow switch and set the low-side FET to its
  minimum pulse.
- A buck with `reverse_current_paranoia` and a filtered current below −0.05 A: the same, to prevent the low side from
  boosting energy back into the input.

The converter keeps switching in both cases.

While `selftest bflow` runs, both reductions and the `highD-bflowOpen` check are suspended, because the test drives
reverse current on purpose. The test stops the converter itself when a sample falls below its trip threshold, −0.3 A
with the backflow switch off or −0.5 A with it on. All other trips stay active, including `revI-ewm` and
`highI-bflowOpen`. See
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
zero of up to 5 % of the sensor's maximum current, so it would otherwise absorb a small short. The check keeps its own
zero, learnt while no current can flow: the backflow switch commanded off and Vin more than 2 V below Vout.

A sample is suspect when, with $I$ the current above that zero and the window $w$ = max(1.5 V, 4 % of Vout):

- $I$ exceeds 10 % of the sensor's maximum current (at least 0.6 A), whatever the voltages, or
- $I$ exceeds max(0.3 A, 1 % of the maximum current), Vout is above 5 V, and Vin lies within −$w$ and
  $w$ + 0.05 Ω × $I$ of Vout, or
- Vout is above the [OV threshold](#output-over-voltage-threshold) and Vin lies within the same window, at any current.
  This covers a battery that has been disconnected, for example by its BMS: the shorted high side then holds the
  output at the panel voltage with no current flowing. Off in PSU mode.

The window allows for two voltage dividers with 1-2 % gain error each, which can disagree by 2.4 V at 60 V.

Suspect samples hold the start (`hs-check`). They latch the fault after a hold that depends on the criterion:

| Criterion | Hold | Trade-off |
| --- | --- | --- |
| current | 3 s | a short passes panel current to the battery, which a healthy idle converter cannot |
| voltage only | 60 s | with a real short and the battery disconnected, the loads see the panel voltage for about 60 s before the latch; a false latch needs `selftest clear` |

The voltage-only hold is 60 s with or without the [low-side clamp](#low-side-clamp). That criterion needs Vout above the
OV threshold, so Vin is far above the 9 V at which the clamp may engage. A latch turns the gates off, which cannot
stop the shorted high side, so an earlier latch would not take the panel voltage off the loads.

A short holds Vout at Vin indefinitely. On a shared output bus, another charger can push the battery above the OV
threshold while the panel voltage passes through the window, but rarely for a whole minute. Vin leaving the window
before the hold ends drops the suspicion without a latch.

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
- not PSU mode, where a stiff supply would be shorted,
- the latch came from the voltage criterion, or the check has learnt its zero, so a sensor offset cannot clamp,
- the blocks phase of the [backflow switch test](#backflow-switch-test) passed within the last 36 hours, no `bf-fail`
  is latched, and
- no clamp has met battery current since the last blocks pass.

| State | Gates | Leaves the state |
| --- | --- | --- |
| pending | all off | to the ramp once Vin (EWMA) has stayed below 9 V for 60 s without a break; a NaN or a sample at or above 9 V restarts the 60 s. To all gates off when a check every 250 ms finds the clamp no longer armed |
| ramp | high side held low, low side pulsed at the switching frequency | to the hold after 20 ms; to all gates off on battery current, a gate shutdown from any other path, a hardware over-voltage alert, or the clamp no longer armed |
| hold | low side on, high side off | to all gates off on battery current, or when the clamp is no longer armed |

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
- A battery driven above the OV threshold by another charger, while the panel voltage passes through the window, latches
  the voltage criterion after 60 s.
- The clamp still discharges the input capacitors through the shorted high side and the low side. Below 9 V and with a
  per-period ramp, circuit simulation puts the peak at about a third of the low-side pulse rating; the per-sample
  staircase was not simulated.
- The 9 V limit relies on the Vin reading. A divider whose reading floors near 9 V in the dark never arms the clamp.
- A short that latches by day keeps the panel voltage on the output until the night.
- A backflow switch stuck closed turns the hold into a battery short for two samples, until the release.

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
switch's body diode carries the forward current. It also engages at 4.5 A without waiting for CCM, to stay clear of
the 6 A `highI-bflowOpen` trip. The conducts phase waits 150 ms before engaging, for the switch's gate supply.

A blocks FAIL ends the run: the conducts phase is not run on a switch that leaks backwards. The FAIL also discards the
last blocks pass, which disarms the low-side clamp. Both are written to flash as soon as the converter has stopped.

A run aborts, without a verdict, when a protection trips, the input voltage collapses, the mode changes, a high-side
short latches, a step times out or `selftest abort` is given. After a mode change the new mode stays; otherwise the previous mode is restored. The
test keeps backoffs and the charger's latches as they were.

A failed phase latches `bf-fail`: every start is refused until `selftest clear`, also after a reboot.

**Daily run.** Boards with `board.conf::bflow_bat=1` run the test on their own once a day, at the first opportunity
when all of these hold for 60 s:

- MPPT mode, converting at least 0.5 A, no limiter active, no sweep, no backoff,
- Vin at least 1.2 × Vout and 3 V above it,
- no rectifier calibration, coil measurement, OTA or USB drive.

The next run is due 20 hours after the last run with two verdicts, so the time of day drifts to the first opportunity
of the morning. An aborted run retries after 30 minutes, at most four times, then waits 20 hours. A refused start waits
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

With `board.conf::ina22x_ov` set, the INA226 compares its bus voltage against that limit in hardware and raises its
ALERT pin. With the MCPWM gate driver, the ALERT interrupt forces the gates off immediately, without waiting for the
real-time loop. The loop then trips with `hw-OV`. An alert that arrives while the converter is off is discarded, so it
cannot block the next start.

The loop also trips `hw-OV` when the alert cannot be armed or the INA226 has lost its alert configuration, for example
after an I2C error or a chip reset. Without the MCPWM gate driver the firmware warns at boot that the alert cannot force
the gates off. See [`board.conf`](../reference/config/board.md).

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

A faster trip lowers the peak by at most a few volts, and only for small coils. Absorbing the coil energy
($\tfrac{1}{2} L I^2$, about 12 mJ at 30 µH and 28 A) or adding output capacitance lowers it further. A fast trip still
matters if the high side keeps switching with the low side off: Vout then climbs towards Vin.

These figures come from the host simulator, not from measurements. Loads on the charger side of the BMS reduce the peak,
but they are also exposed to it.

## ADC and loop watchdogs

These checks run in the real-time loop whether or not the converter is switching. They stop the converter only when it
is switching.

| Watchdog | Condition | Action |
|---|---|---|
| ADC stall | The ADC reports an error, or no fresh sample for 200 ms outside calibration | Reset the ADC peripherals (at most every 300 ms). After 800 ms of stall: stop with a 16 s backoff. After 60 s: restart the system |
| ADC reset | A requested ADC reset | Stop during the reset; restart 6 s after it (30 s if the reset failed) |
| No sample | No sample at all 20 s after boot | Disable the converter and log `Never got a sample! Please check ADC`. After 15 min: restart the system |
| Loop latency | Sample rate below `sensor.conf::expected_hz` for 3 consecutive 3 s windows. Off when `expected_hz` is 0, the default | Stop with a 5 s backoff (100 ms in PSU mode) |
| Calibration failure | Sensor calibration fails | Hold the start for 4 s (100 ms in PSU mode) |

The latency watchdog ignores manual PWM, the first 2 s after a calibration, and the first samples after boot. A single
slow window logs a warning without stopping the converter.

## Start conditions

While the converter is off, `startBlockReason()` must return no reason before the loop starts it. In MPPT mode the
low-frequency status line logs the blocking reason as `START blocked: <reason>`, when it changes and at most every 30 s
otherwise. `temp-stale` is not logged before the first input-voltage sample, when every sensor is still empty.

The temperature and supply checks use a margin above their trip level, so the converter does not restart right at the
threshold. Manual PWM skips this table, but `hs-short`, `hs-check`, `bf-fail`, `temp-stale` and `temp` also hold it off. The table lists the reasons in
the order they are checked:

| Reason | Blocks the start while |
|---|---|
| `psu-latch` | PSU mode latched off after repeated trips |
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
| `idle profile` | The idle power profile holds the converter off; the loop asks it to leave |

## Configuration

| Setting | Effect |
|---|---|
| [`limits.conf`](../reference/config/limits.md) | Voltage, current and temperature limits, short-circuit current, `reverse_current_paranoia`, wire-fault guard |
| [`charger.conf`](../reference/config/charger.md)`::vout_max` | Battery voltage that sets the output OV threshold |
| [`board.conf`](../reference/config/board.md)`::ina22x_ov` | INA226 hardware over-voltage alert |
| [`board.conf`](../reference/config/board.md)`::bflow_bat`, `hs_clamp` | Idle high-side short check and backflow switch test; low-side clamp |
| [`sensor.conf`](../reference/config/sensor.md)`::expected_hz` | Minimum sample rate for the latency watchdog |
| `ovset` (console) | Explicit output OV limit, RAM only |
