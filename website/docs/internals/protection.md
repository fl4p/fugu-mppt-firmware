---
title: Protection
sidebar_position: 2.5
---

# Protection

Protection stops the converter when a measurement leaves its safe range. It runs in the real-time loop on core 1
before the [control loop](control-loop.md) step, and it is separate from the PD limiters: a limiter reduces duty to
hold a value at its limit, while a protection trip turns the half-bridge off.

## Where the checks run

The real-time loop calls two functions on every pass while the converter is switching, in MPPT, PSU and manual PWM
modes:

- `mppt.protect()` checks the fast electrical faults on the latest sample.
- `mppt.protectLf()` checks supply voltage, battery detection, temperature and the wire-fault latch.

Neither runs while the converter is off. While it is off, the [start conditions](#start-conditions) decide whether it
may start again.

Three watchdogs in `src/main.cpp` sit outside these functions and cover the sampler itself:
[ADC stall](#adc-and-loop-watchdogs), missing samples and loop latency.

## Trip and backoff

A trip calls `shutdownDcdc(who, seconds)`, which:

1. Disables the converter (both gates off).
2. Opens the backflow switch and releases the LV power-good output.
3. Ends a running sweep.
4. Arms a backoff timer and logs `backoff <n>s [<who>]`.

A failed check also makes the loop call `stopAndBackoff(4)`, which delays the next start by 4 s. The converter
restarts once both timers have expired and the start conditions pass. In MPPT mode the restart is a new sweep, which
waits while the battery is reported full.

A repeated trip does not extend a running backoff unless it asks for a longer one. The `who` tag in the log line names
the check that tripped; the [trip reference](#trip-reference) lists them.

### PSU mode

In PSU mode, including the PV simulator, output over-voltage (`Vout-OV`) and supply under-voltage (`supply-UV`) trips retry
fast, because a load step can cause them:

| Trips in the last 60 s | Backoff |
|---|---|
| 1–4 | 100 ms |
| 5–8 | The trip's normal backoff |
| More than 8 | 30 s, and the converter stays latched off (`psu-latch`) |

All other trips keep their normal backoff in PSU mode.

## Trip reference

The table lists the checks in the order `protect()` and `protectLf()` evaluate them. Thresholds use the
[`limits.conf`](../reference/config/limits.md) values in their converter roles (Vin max, Vout max, Iin max, Iout max);
the side keys `hv_max`, `lv_max`, `hv_i_max` and `lv_i_max` map to these roles through `converter.conf::topo`.

| `who` tag | Condition | Backoff |
|---|---|---|
| `hw-OV` | INA226 bus over-voltage alert, see [Hardware over-voltage alert](#hardware-over-voltage-alert) | 5 s |
| `psu-setpoint-infeasible` | PSU setpoint no longer reachable: a boost setpoint below Vin, or a setpoint above the OV limit | 30 s, latched |
| `Vin-OV` | Vin sample > Vin max | 5 s |
| `BMS-cutoff` | Output current collapses while Vout and Vin jump, the signature of a BMS opening its switch (buck, battery mode) | 30 s |
| `Vout-OV` | Vout sample > the [OV threshold](#output-over-voltage-threshold) | 5 s |
| `Iin-OC` | Iin sample > 1.3 × Iin max | 5 s |
| `Iout-OC` | Iout sample > 1.5 × Iout max, median > 1.25 × Iout max, or average > 1.15 × Iout max | 30 s |
| `revI-highAvg` | Two reverse-current samples below −1 A while the average output current is above 10 A | 5 s |
| `revI-ewm` | Average current of the physical current sensor below −1 A | 5 s |
| `Vout>Vin-avg` | Buck: average Vout > 1.25 × (average Vin + 1 V) | 5 s |
| `Vout>2Vin` | Buck: Vout sample > 2 × (Vin + 0.5 V) | 5 s |
| `short` | Average Iout > `iout_short` while average Vout < 1 V | 30 s |
| `highI-bflowOpen` | Boards with a backflow switch: more than 6 A while the backflow switch is open or the duty cycle is below 10 % | 5 s |
| `highD-bflowOpen` | Boards with a backflow switch: duty cycle above 33 % with the backflow switch open, unless the DCM current estimate stays below 1 A | 5 s |
| `highI-noSyncRect` | More than 6 A with synchronous rectification off | 5 s |
| `minpulse-M` | Buck in DCM: Vout/Vin above the low-side minimum-pulse ratio, see [Diode Emulation](diode-emulation.md#ls-minimum-pulse-near-unity-ratio) | 5 s |
| `Vr-sensor-fail` | Duty cycle well above what Vout and Iout imply while the output current stays near zero: a sensor or half-bridge fault (`reverse_current_paranoia` only) | 5 s |
| `supply-UV` | Board supply, max(Vin, Vout) − 0.3 V, below 9 V. Skipped in manual PWM | 5 s |
| — | Battery voltage cannot be detected from Vout (MPPT without `charger.conf::vout_max`). Starts a new sensor calibration | 4 s |
| — | NTC or MCU temperature > `temp_max` | 5 s |
| — | Wire-fault guard tripped, see [Output wire-fault protection](../reference/config/limits.md#output-wire-fault-protection) | until `wire-reset` |

### Reverse current below the trip level

Some reverse-current conditions reduce rectification instead of tripping:

- Two output-current samples below −1 A with a low average: open the backflow switch and set the low-side FET to its
  minimum pulse.
- A buck with `reverse_current_paranoia` and a filtered current below −0.05 A: the same, to prevent the low side from
  boosting energy back into the input.

The converter keeps switching in both cases.

### Output over-voltage threshold

The `Vout-OV` threshold depends on the mode, in this order:

1. An explicit limit from the console command `ovset`: min(`ovset`, Vout max).
2. PSU mode: setpoint × 1.03 (× 1.5 without `reverse_current_paranoia`), capped at Vout max. The PV simulator uses its
   open-circuit voltage instead of the setpoint.
3. Battery mode with a known battery voltage: `Vbat_max` × 1.03 (× 1.5 without `reverse_current_paranoia`), capped at
   Vout max.
4. Otherwise Vout max.

If the over-voltage persists for more than 20 s in battery mode, the firmware clears the detected `Vbat_max` and
detects it again. A `Vbat_max` set with the console command `vset` is kept, and the firmware logs an error instead.

### Hardware over-voltage alert

With `board.conf::ina22x_ov` set, the INA226 compares its bus voltage against that limit in hardware and raises its
ALERT pin. With the MCPWM gate driver, the ALERT interrupt forces the gates off immediately, without waiting for the
real-time loop. The loop then trips with `hw-OV`. An alert that arrives while the converter is off is discarded, so it
cannot block the next start. See [`board.conf`](../reference/config/board.md).

## ADC and loop watchdogs

These checks run in the real-time loop whether or not the converter is switching.

| Watchdog | Condition | Action |
|---|---|---|
| ADC stall | The ADC reports an error, or no fresh sample for 200 ms outside calibration | Reset the ADC peripherals (at most every 300 ms). After 800 ms of stall: stop with a 16 s backoff. After 60 s: restart the system |
| No sample | No sample at all 20 s after boot | Disable the converter and log `Never got a sample! Please check ADC`. After 15 min: restart the system |
| Loop latency | Sample rate below `sensor.conf::expected_hz` for 3 consecutive low-frequency windows (~3 s each) | Stop with a 4 s backoff |
| Calibration failure | Sensor calibration fails | Stop with a 4 s backoff |

The latency watchdog ignores manual PWM, the first 2 s after a calibration, and the first samples after boot. A single
slow window logs a warning without stopping the converter.

## Start conditions

While the converter is off, `startBlockReason()` must return no reason before the loop starts it. In MPPT mode the
low-frequency status line logs the blocking reason as `START blocked: <reason>`. The temperature and supply checks use a margin above their trip level, so the
converter does not restart right at the threshold:

| Reason | Blocks the start while |
|---|---|
| `psu-latch` | PSU mode latched off after repeated trips |
| `backoff` | The backoff timer is running |
| `wire-trip` | The wire-fault guard is tripped |
| `temp` | NTC or MCU temperature > `temp_max` − 3 °C |
| `Vin-Vout` | Not in PSU mode: Vin is not at least 1 V above Vout (buck) or below Vout (boost) |
| `minpulse-M` | A buck start would run the low-side minimum pulse above its ratio limit |
| `Vout-still-OV` | PSU mode: Vout is still above 98 % of the OV threshold |
| `supply-UV` | Board supply below 9.5 V |
| `calibrating` | Sensor calibration is running |

## Configuration

| Setting | Effect |
|---|---|
| [`limits.conf`](../reference/config/limits.md) | Voltage, current and temperature limits, short-circuit current, `reverse_current_paranoia`, wire-fault guard |
| [`charger.conf`](../reference/config/charger.md)`::vout_max` | Battery voltage that sets the output OV threshold |
| [`board.conf`](../reference/config/board.md)`::ina22x_ov` | INA226 hardware over-voltage alert |
| [`sensor.conf`](../reference/config/sensor.md)`::expected_hz` | Minimum sample rate for the latency watchdog |
| `ovset` (console) | Explicit output OV limit, RAM only |
