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
mechanism but is not a fault.

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

### Sub-trip reverse current

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
threshold. Manual PWM skips this table, but `temp-stale` and `temp` also hold it off. The table lists the reasons in
the order they are checked:

| Reason | Blocks the start while |
|---|---|
| `psu-latch` | PSU mode latched off after repeated trips |
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
| [`sensor.conf`](../reference/config/sensor.md)`::expected_hz` | Minimum sample rate for the latency watchdog |
| `ovset` (console) | Explicit output OV limit, RAM only |
