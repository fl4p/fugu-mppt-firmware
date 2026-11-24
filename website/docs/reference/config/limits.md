---
title: limits.conf
sidebar_position: 3
---

# limits.conf

`limits.conf` holds the protection cutouts. The firmware reads it at boot, so reboot after `set-config` (see
[When changes take effect](index.md#when-changes-take-effect)). A missing required key or a failed check stops setup
and the converter stays off; `conf-check` reports the same errors without a reboot.

Voltage and current ratings of the hardware are named by board side, HV or LV. `converter.conf::topo`
maps them to the converter roles:

| side key   | buck (in = HV, out = LV) | boost (in = LV, out = HV) |
|------------|--------------------------|---------------------------|
| `hv_max`   | Vin max                  | Vout max                  |
| `lv_max`   | Vout max                 | Vin max                   |
| `hv_i_max` | Iin max                  | Iout max                  |
| `lv_i_max` | Iout max                 | Iin max                   |

A missing limit names the key and the role it plays, for example
`limits.conf: missing hv_max (Vin max under topo=buck)`. `vin_min`, `iout_short`, and `p_max` describe the
source, the output, and the power path, not a side, and keep their role names. The battery limit
`charger.conf::vout_max` is also unaffected: the battery is always on the output.

An old role key for one of the four side ratings fails setup at boot and names its replacement, for
example
`limits.conf: vout_max is no longer read; with topo=boost use hv_max`. See
[Migrating from role keys](sensor.md#migrating-from-role-keys).

| key                        | unit | type  | default | description                                       |
|----------------------------|------|-------|---------|---------------------------------------------------|
| `hv_max`                   | V    | float | (required) | Maximum HV-side voltage before protection cutout       |
| `lv_max`                   | V    | float | (required) | Maximum LV-side voltage before protection cutout       |
| `vin_min`                  | V    | float | (required) | Input-voltage regulation floor; must be below the Vin max. In MPPT, PSU and PV emulator mode the Vin controller reduces duty to keep Vin ≥ `vin_min` (a buck in DCM also keeps Vin ≥ about 1.18 × Vout, see [Diode emulation](../../internals/diode-emulation.md#ls-minimum-pulse-near-unity-ratio)). Not a trip, and not applied in manual PWM. The hard supply undervoltage stop is fixed at ~9 V |
| `hv_i_max`                 | A    | float | (required) | Maximum HV-side current                           |
| `lv_i_max`                 | A    | float | (required) | Maximum LV-side current                           |
| `iout_short`               | A    | float | (required) | Output short-circuit threshold: Iout above this with Vout below 1 V |
| `p_max`                    | W    | float | (required) | Maximum power; sets thermal derating and the sweep's minimum MPP power (0.2 %). Vin max × Iin max must lie between `p_max` and 4 × `p_max` |
| `temp_max`                 | °C   | float | 90      | NTC or MCU temperature that shuts the converter down; 20 < x < 120 |
| `temp_derate`              | °C   | float | (required) | Temperature where power derating begins (linear to 0 at `temp_max`); must be below `temp_max` |
| `reverse_current_paranoia` |      | bool  | 1       | Enable aggressive reverse-current protection; also sets the derived output OV threshold to 1.03 × the charge target or PSU setpoint (1.5 × when 0), capped at the output's side max |
| `wire_protect`             |      | byte  | 1       | Output wire-fault protection: 0 off, 1 warn only, 2 warn, derate and trip. See [Output wire-fault protection](#output-wire-fault-protection) |
| `wire_p_warn`              | W    | float | 2       | Excess wiring power that raises a warning. Needs 0 < `wire_p_warn` ≤ `wire_p_derate` ≤ `wire_p_trip` |
| `wire_p_derate`            | W    | float | 4       | Excess wiring power that caps the output current at √(`wire_p_derate`/ΔR) |
| `wire_p_trip`              | W    | float | 8       | Excess wiring power that stops the converter until `wire-reset` or a reboot |
| `wire_dr_min`              | mΩ   | float | 4       | Minimum excess resistance ΔR for any level; twice this ΔR as a day's median raises the day warning. Must be > 0 |
| `wire_r_max`               | mΩ   | float | 0       | Absolute cap on the learned converter-to-pack resistance baseline; 0 = none, must be ≥ 0 |

## Output wire-fault protection

A loose or corroded connection between the converter and the battery heats up with the extra power
ΔR·I² it dissipates. The protection compares the output-path resistance against a baseline learned
on the board itself and grades the excess power:

| level  | condition                                                  | action |
|--------|------------------------------------------------------------|--------|
| warn   | excess power ≥ `wire_p_warn` and ΔR ≥ `wire_dr_min`         | log line, telemetry `wf`, MQTT alert |
| derate | excess power ≥ `wire_p_derate` (and ΔR ≥ `wire_dr_min`)     | output current capped at √(`wire_p_derate`/ΔR) |
| trip   | excess power ≥ `wire_p_trip`, or the excess drop still ≥ 1.25 × `wire_p_derate` for 5 consecutive minutes while derated | converter stopped until `wire-reset` or reboot |

The ΔR floor keeps normal temperature drift of the copper (about 0.4 %/K) from raising a level at high
current. In PSU mode the output is not taken for the battery, and the protection gets no samples. Three estimates
feed the grading:

- **Excess drop per BMS frame.** The output-to-pack voltage drop minus the baseline resistances times
  the present output and battery current, minus a voltage offset, averaged over one minute while
  `Iout` ≥ 5 A. Needs the pack voltage and battery current over MQTT (`mqtt.conf`
  `bat_voltage_topic`, `ibat_topic`). A level needs 5 consecutive one-minute means above its
  threshold.
- **Slope.** The slope of those drops against the output current, against the baseline resistance.
  Evaluated once a minute; a level needs 30 consecutive minutes above its threshold.
- **Sweep fit.** The median output resistance of the last 5 qualifying sweeps (telemetry `Ro`)
  against its own baseline, so up to 2 outlying sweeps out of 5 don't move it. Needs no [BMS data](../../guide/charging/bms-integration.md) and
  at least 3 sweeps since boot. It is graded at most once per 20 charging minutes, so a burst of
  sweeps within a few minutes counts once. A level needs 3 consecutive graded values above its
  threshold, which takes at least 40 charging minutes; a level drops only when all of the last 5
  sweeps are below.

The converter's and the BMS's voltage readings differ by an offset that drifts by tens of millivolts
over hours and days. Against a fixed offset that drift would read as excess resistance at high current,
so the offset comes from a running regression of the one-minute drops against the output current
(memory of about 120 minutes with output current): an offset carries no current, a resistance shows
as slope. The regression needs at least 2 A of spread in the output current before the excess drop and
the slope are evaluated.

A minute whose excess power off the regression line reaches half of `wire_p_warn` (a step)
doesn't enter it, and nothing enters it while a level is held, so a fault can't fade into the offset.

The slope and the sweep fit are graded at the highest one-minute output current of the last one to
two charging hours. Any estimate can raise the level. A level drops only after every estimate that
still has data has stayed below 75 % of the thresholds for 30 minutes. A trip never drops by itself.
An estimate counts as having data until 15 (excess drop), 60 (slope) or 360 (sweep fit) charging
minutes pass without a new value; time without output current does not age it, so a night neither
releases nor invalidates the day's verdict.

A rise of the shared bus-to-pack resistance (`Rb`) only warns: the chargers can't relieve that
section while a load discharges the pack through it. When the output and battery currents move
together (a single charger, steady loads), the converter can't tell a fault in the shared section
from one in its own wire, and the excess-drop and slope estimates grade it as its own.

On days with little current a fault can stay below `wire_p_warn`. At the end of each day after the
learning phase, the median excess resistance of the day's one-minute excess drops (with fewer than 4
excess-drop values that day: of the day's sweep fits against their baseline, once that baseline holds 3 days) is checked on its own: at 2 × `wire_dr_min` or more
it raises a warning, regardless of current. The day warning never derates or trips, survives a
reboot, and clears after a judged day below 1.5 × `wire_dr_min`.

When no estimate has data (no BMS data and no qualifying sweep) and no level is held, telemetry `wf`
reports -1 and the MQTT alert reports `unverified`, also after more than 3 charging days without any
data; a held level stays. Missing data never reads as `ok`. Telemetry `Pw` is sent only from an
estimate that produced a value within the last 2 minutes.

### Baseline and learning

The baseline is the median of the daily medians of the last 14 good days. A day is good for the
resistance baseline when it has at least 4 samples of the slow fit taken while `Iout` ≥ 5 A, at
least 300 BMS frames at `Iout` ≥ 5 A, and a standard deviation of `Iout` of at least 2 A among those
frames. Below that spread the fit can't tell the voltage offset from the resistance, and the drift of
the offset biases the slope. The sweep baseline needs at least 2 sweeps on the day. A day without
enough data neither enters the baseline nor counts as learned.

The first 7 good days are a learning phase that only warns. On a converter with BMS data a day counts
when the resistance part was good; without BMS data, when the sweeps were. During learning every good
day enters the baseline, so a wrong first day is outvoted by the median once two more good days are
in.

An estimate is graded only once its baseline holds at least 3 stored days: the excess drop, the slope
and the shared-section (`Rb`) check need 3 days in the resistance baseline, the sweep fit 3 days in
the sweep baseline. A configured `wire_r_max` counts as a reference for the excess drop and the
slope once one day is stored; until 3 days are stored they grade against `wire_r_max` alone. An estimate
that is not graded holds and raises no level and shows as `stale, baseline n/3 days` in the `wire`
command; with no other estimate graded, `wf` reports -1. Every good day is still stored while
learning.

After learning, a day's median enters only when it is at most `wire_dr_min` above the current
baseline and at most half of `wire_dr_min` above the lowest baseline since learning ended, so a slowly
growing fault isn't learned in. The resistance part of a day is also rejected when more than half of its
excess-drop minutes reached half of `wire_dr_min`.

A day ends at the first hour without conversion at least 12 hours after the previous end, or after 36
hours. After a cold start (power-up) the first such hour ends the day as soon as the day counts. The
open day and the offset regression are kept in RAM that survives a restart, a crash or watchdog reset
and a firmware update (unless the update changes how they are stored), but not a power loss, so
restarts during the day don't discard its data and the excess drop is evaluated again from the first
minute after a restart.

The baseline is kept in NVS and written at most once a day, only while the converter is stopped;
reprovisioning the config partition doesn't touch it. `wire-reset` clears the trip, the baseline and
the open day and restarts learning.
