---
title: "Charge Termination"
sidebar_position: 2
---


# Charge Termination

`Li_ChgTerminationCondition` in `src/charger.h` implements a charge-termination
line for LFP and other lithium chemistries, as described in
[Charging Marine Lithium Battery Banks](https://nordkyndesign.com/charging-marine-lithium-battery-banks/).
It works on the highest cell voltage and the pack current reported by the [BMS](bms-integration.md), so
the charger can keep every cell below the BMS over-voltage cut-off (and the
voltage transient of that disconnect), give the cell balancer time, and avoid a
trickle charge into a full pack.

The model places the cell at `cv_float` (the "float" voltage) at zero current and
at `cv_eoc` (the absorption voltage) when the charging current equals the
tail-current threshold. Between those two operating points, a straight line in
the (current, voltage) plane defines the termination boundary. A cell voltage
above the line at the present current means the cell is "done". Below it, the
cell is "still charging".

The line can be restated as an apparent series resistance:

```
r = (cv_eoc - cv_float) / (tail_c_rate * Cbat)
```

With the firmware defaults (`cv_eoc` 3.5 V, `cv_float` 3.325 V, `tail_c_rate`
0.05) and a 280 Ah pack, r = 0.175 V / 14 A = 12.5 mΩ.

With that resistance, the per-current termination voltage becomes:

```
v_term(ibat) = cv_float + ibat * r       (clamped at cv_eoc)
```

The charger declares termination when the highest reported cell voltage
is above `v_term` on two consecutive BMS cell frames (`termCond.update()` runs
once per cell frame) while the pack is not discharging (`ibat` above
−0.02 × `tail_c_rate` × `bat_c`). The line is evaluated only while both the cell
voltages and `ibat` are fresh. The `cv_ceiling` backstop (see
[Related parameters](#related-parameters)) also needs two consecutive frames,
regardless of current.

The current that places the line is the recent peak of the battery current, decaying with a time
constant of 2 minutes, not the instantaneous value. When the charge current drops suddenly, for example
under a passing cloud, the cell voltage needs minutes to relax, and an instantaneous line would
drop below it at once and end the charge early.

## The `tail_c_rate` parameter

`tail_c_rate` is the current, as a fraction of capacity, at which the
termination line reaches `cv_eoc`: the tail current at the absorption voltage.
You can set it in `charger.conf:tail_c_rate`. It defaults to `0.05` (LFP).

The following table lists typical values per chemistry:

| Chemistry                            | `tail_c_rate` |
|--------------------------------------|---------------|
| LFP                                  | `0.05`        |
| Sanyo/Panasonic NCR18650GA (67 mA / 3500 mAh, [spec sheet copy at a reseller](https://www.orbtronic.com/content/Datasheet-specs-Sanyo-Panasonic-NCR18650GA-3500mah.pdf)) | `~0.02` |
| EVE INR18650                         | `0.033`       |

The values apply at the firmware's `cv_eoc`. For LFP, the Nordkyn article's
tail currents of 0.033C (older cells) and 0.05C (newer cells) are given at
3.65 V; its own table puts the tail at 3.5 V at 0.015C and 0.023C. The default
0.05C at 3.5 V therefore terminates deliberately earlier than the Nordkyn line
(see [Related parameters](#related-parameters)). The EVE INR18650 value has no
cited source; check it against the cell's datasheet.

A larger `tail_c_rate` is safer. It lowers `r` and flattens the termination
line, so `v_term` is lower at any current below the `cv_eoc` clamp and
termination triggers at a higher current, earlier in the constant-voltage
phase. The cell is left less fully charged but with more headroom against
over-charge.

A smaller `tail_c_rate` charges more fully with a tighter margin. The charger
holds CV down to a smaller tail current before it terminates. The cell ends up
closer to its true full-charge state, but the cell-voltage tolerance is
narrower.

For unknown cells, the default `0.05` is the conservative choice. Termination
may be slightly early for chemistries that prefer a smaller tail. The tail rate
leaves the voltage limit unchanged: for any `tail_c_rate`, the termination line
is capped at `cv_eoc`, and `cv_ceiling` is a current-independent per-cell
backstop above it.

## Recharge hysteresis (DoD-based release)

The charger releases termination after a set amount of net discharge, not on
cell voltage alone. LFP cells discharge nearly flat: a 1 % SoC drop from full
can take voltage from 3.45 V down to 3.35 V. Releasing termination on
`vcell_high < cv_float` alone therefore causes micro-cycling at the top of
charge. The charger oscillates between "terminated" and "charging" every few
minutes while the cells barely move.

To measure that discharge, the charger integrates `-ibat` over time (positive =
pack discharging) and tracks Ah-since-the-last-full event. Termination releases
when either of these conditions holds:

- Ah condition (primary): `ahSinceFull > recharge_dod * Cbat`. With the
  default `recharge_dod = 0.20` and a 280 Ah pack, this is ~56 Ah of net
  discharge before recharge is permitted.
- Voltage floor (fallback): `vcell_high < cv_float - recharge_vfloor_band`
  (band default 0.05 V) for 4 consecutive BMS cell frames. That is 3.275 V with
  the firmware defaults and 3.32 V with `cv_float=3.37`. This condition catches
  integrator drift, a wrong `Cbat`, or a missing BMS, any of which would
  otherwise leave the charger stuck terminated while the pack is genuinely
  empty.

With `mqtt.conf:bat_charge_topic` set, the charger takes the deficit from the
BMS remaining-charge counter instead, as long as that value is fresh (a frame
within the last 120 s). The charge is sampled on every cell frame, and the
reference is the sample on the frame that latched termination. Chargers that
share one BMS therefore compute the same deficit and release on the same cell
frame. With independent integrators, small differences let one charger release
first. Its charging then pulls the other's deficit back below the threshold, so
that one stays terminated until the next full charge. The release compares the
deepest deficit since the last full charge, so a sibling that starts charging
cannot undo a release that is already due. If the BMS charge goes stale, the
integrator takes over.

While a charger is terminated, its full point follows the BMS charge up. On a
cell frame where the BMS deficit is negative (a sibling charged the pack past
the reference), the reference moves to that frame's BMS charge and the deepest
deficit is cleared. Two chargers on one BMS that both see that peak thus end on
the same reference, even when only one of them latched again after more
charge, for example on the cell ceiling or a hold timeout. A charger that
misses the peak frame (stale BMS data) keeps its lower reference. The
`ibat` integrator fallback does not follow. After the release the reference
stays where it is, so charge added during a recharge does not delay the next
release. The raised reference is the one saved for a reboot.

Some BMSs re-seed their remaining charge, for example to 90 % of the nominal
capacity after a reconnect. The charger treats a step as a re-seed when no
battery current could cause it: the change since the previous BMS value
exceeds 200 A (the pack maximum) times the time between the two values, plus
0.5 Ah, and that time is at most 120 s. The time counts as at least 12 s (one BMS
publish period), so values that arrive late, in a batch or twice still pass.

- A re-seed is logged as `BMS charge re-seed ...`. Its step is replaced by the
  `ibat` estimate for that one window (the integrated `ibat`, plus the last
  `ibat` for time without data, limited to 200 A), and the difference is kept
  as an offset.
- Every other change passes unchanged, whatever the `ibat` rate or timing.
- The error at a re-seed is that of the `ibat` estimate over the one window,
  at most the change in current times the window.

The reference saved for a reboot is stored in the BMS's own scale together
with the offset, so an absorbed step does not reappear after a restart.

The last BMS value, the offset and the time of that value are also kept in RTC
memory across a software reset or a panic, task and interrupt watchdog
included (not a power loss, brownout or other reset). The first value after a reboot is checked against it
with the same bound. The time between the two comes from the RTC slow-clock
counter and includes the reboot. A re-seed during a reboot is then replaced by
the `ibat` received since the boot, or 0 A without one. The RTC time counts as
unknown, and the value as real, when it is shorter than the time since boot
(the counter restarted).

Some re-seeds are taken as real charge or discharge:

- a re-seed after a gap of more than 120 s between BMS values, a reboot
  included;
- a re-seed during a reboot after a power loss, brownout or other reset that
  may restart the RTC clock, or with an unknown RTC time;
- a re-seed during a reboot when the reference was last saved by a firmware
  without this check (after a rollback);
- a re-seed small enough for 200 A to explain, for example up to 1.2 Ah
  within 12 s, or a 3.8 Ah step after a 69 s gap.

A downward one can release termination early; the next full charge sets a new
reference. An upward one delays the release by its size. While terminated, the
part of it above the reference moves the reference up instead; that delay is
then at most the deficit before the step. A single wrong value above the
reference that is taken as real keeps the reference raised after the value
returns, so the release comes that much early (up to 1.2 Ah at the 12 s
spacing).

The charger re-zeroes the integrator on every termination event. The
integrator counts the "deficit since the last known full" and recalibrates
itself each cycle, so it does not have to be a precise long-term SoC gauge.

The integrator and the termination latch are RAM-only. After a reboot, the
charger is not terminated, so it charges until termination latches again
(within two cell frames if the pack is still full). That termination re-zeroes
the counter, and from then on both release conditions apply as usual. The BMS
reference of the last full charge is kept in RTC memory across a software reset
(not a power loss), so the BMS-based deficit continues from that full point.

Termination needs the pack current, which takes about eight `ibat` frames to
settle after a boot. If a cell frame in that gap shows the highest cell within
30 mV of `cv_eoc`, the charger limits its output current to 0.25 A until the
termination line has run on two cell frames. A sibling charger may hold the
pack at the top meanwhile, and a full-power start would push it over
`cv_ceiling`. Without an `ibat_topic` this idle ends 120 s after boot.

You can set `recharge_dod` in `charger.conf:recharge_dod`. A higher value
releases later: deeper discharges between full charges, fewer cycles, and more
time at mid-SoC. A lower value releases sooner: more frequent topping, more
cycle wear, and less time below 100 %. LFP off-grid systems commonly run
0.10–0.30.

## Parallel chargers on one pack

When several chargers feed one pack and the pack is terminated, usually only
one of them drives the bus. A converter drives the bus while it is enabled
and above its minimum duty. The others sit at minimum duty, and charging into
a full pack would only fight that converter. A charger that does not drive
the bus therefore pins its target at the float floor
(`vout_max_fallback - vout_offset_max`) and delivers no current.

While the BMS reports the pack discharging (pack current below −1 A), the load
is larger than what the driving charger supplies. The idle charger then joins
at the float target (`vout_max_fallback`) and helps cover the load. It returns
to the floor once the pack current rises above +0.5 A, or when the BMS current
goes stale. The join target is fixed, so a converter that is off overnight
does not build up a higher setpoint.

Set `bat_charge_topic` so that all chargers leave termination together (see
[Recharge hysteresis](#recharge-hysteresis-dod-based-release)).

## Related parameters

These parameters also shape termination:

- `cv_eoc`: absorption voltage, where the line meets the tail current
  (default 3.5 V)
- `cv_float`: float voltage, where the line meets zero current (default
  3.325 V)
- `cv_ceiling`: hard per-cell ceiling, default `cv_eoc + 0.05`. It latches
  termination after 2 consecutive frames at or above it, regardless of current.
- `bat_c`: effective pack capacity in Ah. For parallel packs, use the summed
  Ah (e.g. 2P 280 Ah → 560). The model assumes `Cbat` is the parallel-effective
  capacity.
- `recharge_dod`: DoD threshold for releasing termination (see above).
- `vout_max_fallback`: pack output limit without BMS data and float target
  after termination, default `n_cells × cv_float`. The cell count is
  `floor(vout_max / cv_eoc)`, or detected at boot with `vout_max=auto`.

Setup fails and the converter stays off if `cv_eoc` is below `cv_float`,
`cv_ceiling` is below `cv_eoc`, or `cv_hold_band` is not positive. With the
[CV hold](#cv-hold) or [top balancing](#top-balancing) enabled, `cv_hold_band`
must also be smaller than `cv_eoc − cv_float`.

The Nordkyn article states the manufacturer termination condition for LFP as
3.65 V at 0.033C (older cells) or 0.05C (newer cells), and about 3.37 V as the
resting voltage of a fully charged cell, where the termination current is zero.
The firmware defaults deliberately lie below that line: 3.5 V at 0.05C, which
the article calls a conservative charging voltage, and 3.325 V at zero current,
its suggested voltage for supplying loads without charging. Termination thus
comes earlier, trading some capacity for less time at full charge.

:::warning Set `bat_c` on any battery system
If `bat_c` is missing, the termination line and the EOC feedback on the highest
cell are disabled. The output is held at the pack-level `vout_max_fallback`
(default `N_cells × cv_float`), open-loop on the Vout reading, like a stale BMS.
Only the hard `cv_ceiling` latch can still flag termination, and it does not
lower the output. The Ah recharge condition is disabled; only the voltage floor
releases termination. The pack floats near `cv_float` per cell and does not
reach full, and the highest cell is not individually limited.
:::

## Stale cell data

Cell voltages older than 180 s count as stale, and the output is limited to
`vout_max_fallback`, open-loop on the Vout reading. If the data goes stale
before termination (with `bat_c` set) and the last highest cell was below
`cv_float` + 50 mV, the limit does not drop at once: it starts at the bus
voltage plus 0.1 V (at most the previous limit) and ramps down to
`vout_max_fallback` over 600 s, so the charger does not fall below a charging
pack.

## Absorption target vs. termination line

The termination line only *decides* termination. It is not a setpoint.

The EOC feedback loop (`BatteryCharger::_updatePackVoltagePinning`) lowers the
pack-voltage pin whenever the highest cell is above its target. That target is
the fixed `cv_eoc` while charging and `cv_float` once terminated. The pin may
fall to `vout_max_fallback − vout_offset_max` (default 0.6 V below the float
target), so a converter whose Vout reading is high still pulls the highest
cell down.

The target must not be the current-dependent `v_term`. Lowering the pin
reduces the current, which lowers `v_term`, which lowers the pin again, until
the current is zero and the line trips at `cv_float`, a premature termination.

## CV hold

At a high charge current the internal resistance raises the highest cell
above the line long before the pack is full. With `cv_hold_max_s` > 0 (default 0, off) the
line crossing does not terminate. The charger enters a CV hold instead: it
holds the highest cell at `cv_eoc` and lets the current taper.

The crossing enters the hold only with the highest cell within `cv_hold_band`
of `cv_eoc`, where the tail exit can end it. At a low `ibat` the line sits near
`cv_float`: a load or a cloud takes most of the PV current, or a restart or a
BMS data gap has cleared the line's peak-current memory. A cell still relaxing
from a higher current then crosses it far below the top. Below the band the
charger keeps charging, and the [approach](#approach) limits the current on
the way up.

The hold regulates the pack charge current from BMS data only. The output
voltage limit stays at `vout_max` as a protection ceiling, so an error of the
converter's own Vout reading cannot shift the regulated cell voltage. Two
loops run on the BMS frames:

- Cell loop, on each cell frame: the pack-current target is the measured
  `ibat` plus $K (cv\_eoc - vcell\_high)/r$, where
  $r = (cv\_eoc - cv\_float)/(tail\_c\_rate \cdot bat\_c)$ is the cell
  resistance implied by the termination line (12.5 mΩ for 280 Ah with the
  defaults, see above).
  $K$ is 1.5 with the cell above `cv_eoc` and 0.5 below it.
- Look-ahead, part of the cell loop: the polarization of a nearly full cell
  builds over minutes, so a cell below `cv_eoc` that still rises has no room
  left. Within 60 mV of `cv_eoc`, a cell that rose faster than 0.25 mV/s
  (3 mV per 12 s frame) on two frame pairs in a row, beyond the ohmic step of
  the `ibat` change (2.5 mΩ), uses its voltage projected 36 s ahead at that
  rate instead of the present one. $K$ follows the sign of this projected
  error, so a cell still below `cv_eoc` can get the downward gain. The rate is
  taken over the frames' arrival times. A pair less than 4 s or more than 30 s
  apart (lost frames, a burst) starts the count of two pairs again.
- Current loop, on each `ibat` frame: the converter's output-current limit
  moves by 0.4 × (target − `ibat`). Upward steps are at most 2 A, and the limit
  stays at most 2 A above the converter's own output current, so it does not
  wind up while the PV power is short and the returning sun does not push the
  cell over `cv_eoc`. The limit never goes below 0.25 A.
- During an MPP sweep the current loop holds its limit: the sweep starts from
  zero output current, which would otherwise pull the limit down to 2 A. A hold
  or approach that begins during a sweep starts at the tail current.

The output current includes the house load ($i_{out} = i_{bat} + i_{load}$);
regulating `ibat` with the limit as the integrator lets a load step settle
within a few `ibat` frames, without waiting for the cell voltage to move. The
limit starts at the output current when the hold begins, or keeps the limit of
the [approach](#approach), so entering the hold does not step the output. In PSU mode the limit stays where it is. A colder or
hotter pack than the temperature policy allows lowers the target to its
pack-current limit.

The target is the constant `cv_eoc`, for the same reason as in
[Absorption target vs. termination line](#absorption-target-vs-termination-line):
a target that follows the current would walk the current down to zero.

### Approach

Near the top of charge the cell voltage keeps rising for minutes after the
current drops, and cell frames arrive seconds apart. A hold that starts at
full current overshoots `cv_eoc`. The charger therefore starts the current limit
before the line crossing. When the highest cell comes within 30 mV of
`cv_eoc` while the pack current is above the tail current
(`tail_c_rate × bat_c`, the line's current at `cv_eoc`), the output-current
limit drops at once to the output current scaled by tail current / `ibat`. The
cell loop then raises it again at the slow gain. The approach lasts until the
hold begins, termination latches, the BMS data goes stale, or the pack
discharges. It applies only where a line crossing enters the hold
(`cv_hold_max_s` > 0 or a balance run).

A hold that begins at a low `ibat` starts without an approach limit, and its
cell loop alone would let the limit follow the output current when the sun
returns. Such a hold keeps the cut pending and applies it once, the same way,
when the pack current rises above the tail current with the cell in the band.

The hold ends in one of these ways:

- Tail: the BMS pack current stays below `tail_c_rate × bat_c` and the highest
  cell stays within `cv_hold_band` (default 0.02 V) of `cv_eoc` for
  `tail_hold_s` (default 180 s) without a break. The band test rejects a low
  current caused by clouds or a load: then the cell falls below the band and
  the tail timer restarts. The charger terminates and marks the pack full.
- Timeout: the hold has lasted `cv_hold_max_s`. The charger terminates and logs
  a warning.
- Ceiling: the highest cell reaches `cv_ceiling` on 2 consecutive frames. The
  charger terminates at once, as without the hold.
- Stale BMS data: the cell voltage or the pack current is older than 180 s. The
  charger terminates without marking the pack full and holds the float
  fallback. A [balance run](#top-balancing) pauses instead.
- Sun gone: the highest cell stays below `cv_float` for 4 consecutive frames.
  The charger returns to charging without terminating, so a cloudy afternoon
  does not end in a false "full".

Without BMS frames the hold runs blind: neither loop nor the ceiling check
sees the pack. Once the newest cell or current frame is older than 20 s, the
current limit decays from `ibat_max` towards its 0.25 A floor with a 20 s time
constant (with the default `ibat_max` of 20 A, below 1 A about 80 s after
the last frame), during the hold and its approach. The
floor is safe however many chargers share the pack, and it matches where the
180 s expiry ends anyway. Fresh frames resume the regulation from the decayed
limit, so the current ramps back up at the upward step limit instead of
stepping. The 180 s expiry and its exits do not change.

The hold has two known overshoot limits:

- Re-entry: a BMS data gap or a cloud dip lets the highest cell fall below
  the approach band, and its polarization relaxes. When the data or the sun
  comes back, the polarization rebuilds faster than the cell loop lowers the
  current, and the cell overshoots `cv_eoc` by about 20 mV (measured). The
  peak stays below the ceiling cutoff (3.55 V with `cv_eoc` at 3.50 V).
- Cycling load: a load that switches on and off faster than the cell frames
  arrive can push the highest cell above `cv_ceiling` between two frames,
  where the ceiling check does not see it. In simulation, a 900 W load
  cycling 30 s on and 30 s off took the cell to 3.557 V with no frame above
  the ceiling. Field data has not shown this so far.

When the hold ends in termination, the output voltage limit glides from the
bus voltage as the converter reads it to the float voltage, so the output does
not step. After termination the float and recharge rules above apply
unchanged. `status` reports `CV HOLD (<seconds>s, ilim <amps>A)` during the
hold.

Parallel chargers on one pack each run their own hold on the same BMS data,
without coordination. The gains are sized for that:

- All chargers compute the same pack-current target from the same frames, so
  the cell loop gain does not add up. Its proportional part has a gain per
  frame of $K R/r$, with $R$ the cell's resistance within a frame, and must
  stay below 1 with a frame of lag: downward for $R < r/1.5$ (8.3 mΩ at
  280 Ah with the defaults), upward for $R < 2r$.
- A model fitted to a field charge of a 245 Ah LFP pack near the top has
  2.1 mΩ at once plus a polarization that follows the current with a 410 s
  time constant and grows with the charge put in. At a 3.50 V hold its
  incremental resistance is about 23 mΩ once settled, but it builds by less
  than 3 % in a 12 s frame, so $R$ is about 3 mΩ, well inside both bounds.
  The slow upward gain keeps the polarization that relaxes after a current cut
  from being read as room for more current.
- The look-ahead lies outside that bound. It acts only near `cv_eoc` and
  only on a sustained rise, and lowers the target. Each lower current shrinks
  the next rise, so the look-ahead lets go by itself. The measured `ibat` step
  is taken out of the rise, so a current step is not read as a rise or fall of
  the cell.
- Each charger's limit moves `ibat` one for one, so N chargers sum to a gain of
  0.4 N per `ibat` frame: 0.8 for two, stable also when the BMS reports the
  current one frame late.
- A charger whose Vout sensor reads wrong regulates the same pack current as
  the others; the reading only matters at the `vout_max` ceiling.

## Top balancing

An active cell balancer equalizes the cells only while they sit at the top of
the charge curve, where the voltage separates them. With `balance_interval` > 0
(days, default 0, off) the charger periodically holds the pack there long
enough for the balancer to work. Time at the top ages LFP cells, more so when
warm (see [LFP longevity](../../internals/lfp-longevity.md)), so the run is
infrequent, bounded, and deferred when the pack is warm.

A run is due once the last completed run is `balance_interval` days old. While
it is due:

- The partial-charge ceiling does not hold, as when `full_charge_interval`
  expires. Holiday mode defers the run instead (see below), so its ceiling
  stays.
- A crossing of the termination line within `cv_hold_band` of `cv_eoc` enters
  the [CV hold](#cv-hold), also with `cv_hold_max_s` = 0. The tail and the
  `cv_hold_max_s` timeout do not end it.
- The time the highest cell spends within `cv_hold_band` of `cv_eoc` adds up
  across charge sessions (a cloudy day, a sunset) until it reaches
  `balance_hold_s` (default 7200 s).
- With `mqtt.conf:cell_voltages_delta_topic` configured, the run also completes
  when the BMS cell spread is at or below `balance_delta` (default 0.01 V) on 3
  frames in a row while the cell is in band. A delta older than 60 s does not
  count.

When the run completes, the hold ends through its normal exits: with
`cv_hold_max_s` = 0 the charger terminates at once and marks the pack full.
The ceiling and sun-gone exits of the hold apply unchanged; they do not
complete the run. A run that stops on the ceiling, in the hold or in its
[approach](#approach), stays pending with its in-band time but is deferred for
a quarter of `balance_interval`, at least 20 h, so the next charge ends
normally (the `balance` reason is `ceiling`). `balance now` clears the
deferral; a software reset keeps it.

Stale BMS data pauses the run instead of terminating:

- The charger leaves the hold and holds the float fallback, as a termination
  would: the output voltage limit glides from the bus voltage to the float
  voltage and the hold's current limit is dropped. It does not mark the pack
  full.
- The paused time counts neither as in-band time nor as session time.
- With fresh cell and current data again, the hold resumes on the next cell
  frame. Its current limit restarts at the
  output current and ramps up, so the output does not step.
- Data still stale 600 s into the pause (780 s after the last frame)
  terminates the charger as outside a run. The run stays pending with its
  in-band time and resumes on the next charge.

A broker or BMS-publisher restart of a few minutes thus costs the run only the
gap.

Limits and deferral:

- A single session ends after 2 × `balance_hold_s` in the hold (for example
  when the PV cannot keep the cell in band) through the normal hold exits. The
  run stays pending with its in-band time. A software reset during a session
  does not restart this count.
- The run is deferred while holiday mode is on, while the hottest fresh pack
  sensor is above 35 °C (the top of the LFP storage range; it resumes below
  33 °C), and while the BMS cell or current data is stale. A deferred run that
  is already holding ends the hold through its normal exits and keeps its
  progress; stale data pauses it instead (see above). With `mqtt.conf:bat_temp_topic` configured but no fresh
  temperature (each sensor expires after one hour), the run is deferred as well;
  without a configured temperature topic there is no temperature deferral.
- A pack that is already terminated when the run becomes due is not released
  for it; the run happens on the next charge.
- The age of the last run, the in-band time, the session time and a ceiling
  deferral survive a software reset. After
  a power loss the age counts from boot, so a run is never due right after
  power-on.

The console command `balance` shows the state (off, days to the next run, due,
running or the deferral reason, and the in-band time); `balance now` makes a
run due, `balance skip` marks it done. During a run `status` reports
`CV HOLD balance (<in band>/<balance_hold_s>s, ilim <amps>A)`. Parallel
chargers on one pack each run their own balance hold on the same BMS data, with
the same current regulation as the CV hold.

## Partial-charge ceiling (`partial_charge`)

The charger can stop short of full to lower the average SoC, which is the
best-evidenced LFP lifetime lever (see
[LFP longevity](../../internals/lfp-longevity.md)). Recharge hysteresis alone
keeps the pack in the 80–100 % window.

The partial-charge ceiling works as follows:

- After a termination (the only point where the Ah counter is known to be at
  zero), charging stops once `ahSinceFull <= (1 - partial_charge) * Cbat`. The
  charger then holds the pack there by load-following: on every BMS `ibat`
  frame, the pin steps by up to 20 mV (4 mV/A, 0.2 A deadband) so the pack
  current goes to a small target and the converter covers the load only.
- The charger trims the target by the Ah error (±1 A at 2 Ah off the ceiling),
  so a BMS current offset inside the deadband cannot walk the SoC away over the
  week.
- The hold starts from the measured bus voltage and only steps while the
  converter drives the bus (not at night).
- The pin may fall to
  `n_cells * (cv_float - recharge_vfloor_band) - vout_offset_max` and rise to
  `Vbat_max`. If the load exceeds the PV, the pin sits at `Vbat_max` and the
  pack discharges normally.
- The hold releases when the deficit exceeds the ceiling by `recharge_dod`
  (the pack cycles between `partial_charge - recharge_dod` and
  `partial_charge`).
- Every `full_charge_interval` days, the charger drops the ceiling and charges
  the pack to full for BMS balancing.
- A software reset (restart, crash, OTA update) keeps the age of the last full
  charge and the BMS remaining charge at that point in RTC memory. With
  `mqtt.conf:bat_charge_topic` reporting, the hold continues after such a reset
  from the BMS charge. Otherwise, and after a power loss, the deficit counter
  starts unknown and the charger charges to full first.
- The hold needs live BMS data. If the cell-voltage or the `ibat` stream stops
  (180 s), the charger drops the hold and falls back to its ordinary behaviour.
  It re-engages the hold when the data returns.
- Unlike termination, the hold does not block the dawn start. The converter
  has to run to serve the loads, and the pin keeps the pack current at zero.

`status` reports `PARTIAL HOLD (load-following)`, the ceiling, and the age of
the last full charge. The periodic re-sweep and the stuck watchdog treat the
hold like termination.

The configuration must satisfy `0 < recharge_dod < partial_charge`. Otherwise,
the release band would reach below 0 % SoC. Without `bat_c`, the ceiling is
disabled.

### Holiday auto mode (`holiday_charge`)

A pack that is cycled every day benefits from charging to full; a pack that
barely moves for days, for example while the owner is away, only sits at the
top. With `holiday_charge` set, the charger detects such quiet periods and
applies a ceiling automatically:

- The charger measures the mean discharge current of the pack in consecutive
  24 h windows on its own uptime clock: the Ah discharged while the BMS `ibat`
  was below −0.2 A, divided by the time it was. Charging is ignored, so a full
  charge is not counted as activity, and neither is discharge caused by the
  hold itself while it drains the pack down to the ceiling. A window is quiet
  when this mean (taken over at least 2 h) stays below `holiday_ibat`.
- A window is only evaluated when live `ibat` data covered at least 80 % of it
  (a gap of more than 60 s between frames counts as not covered) and, if the
  pack discharged for 2 h or more, the `ibat` reading changed in at least 5 %
  of the frames (a stuck reading is not quiet). A covered window with almost no
  discharge counts as quiet. A window that fails these
  checks neither counts nor resets the streak. Before holiday mode is on, two
  such windows in a row reset the streak; once it is on, missing data never
  ends it (without fresh BMS data the hold is off anyway, and the next
  evaluated window decides).
- After `holiday_days` quiet windows in a row, the ceiling `holiday_charge`
  applies (the lower of `holiday_charge` and `partial_charge` if both are set).
  It works like the partial-charge ceiling above, including the hold and the
  periodic full charge every `full_charge_interval` days, but the hold
  releases `holiday_band` (instead of `recharge_dod`) below the ceiling.
- With `holiday_charge=0.6` and the default `holiday_band` of 0.1, the pack
  cycles between about 50 and 60 % SoC. That lies on the same calendar-aging
  plateau as lower storage levels and below the step near 70 % (see
  [LFP longevity](../../internals/lfp-longevity.md)), and it keeps several
  days of base-load reserve. The hold refills towards the ceiling at no more
  than 1 A, so a base load that discharges more overnight than that refills
  during the day slides the pack down to the release point, where normal
  charging takes it back up to the ceiling.
- Holiday mode ends when a window closes with a mean at or above
  `holiday_ibat`, or earlier, as soon as the running window has discharged for
  3 h at a mean above 1.2 × `holiday_ibat`. The pack then charges to full again
  (or to `partial_charge`).
- The holiday state, the quiet-day streak and the console override survive a
  software reset (RTC memory), not a power loss.

Set `holiday_ibat` to about 1.5 times the pack's idle discharge current (standby
consumers, BMS, inverter idle), measured from the BMS current on days when the
installation is unattended. Below the idle current holiday mode never engages;
far above it, normal use counts as quiet. There is no default: the mode
refuses to load without it.

The console command `holiday on|off|auto` overrides the detection: `on` applies
the holiday ceiling now, `off` disables it, and `auto` returns to the detection.
`status` shows the holiday state, the mode, the quiet-day streak, the
unevaluated windows and the discharge time and mean current of the running
window. The configuration must satisfy `0 < holiday_band < holiday_charge`;
without `bat_c` the auto mode is disabled.

## Pack temperature (`bat_temp_*`)

When `mqtt.conf:bat_temp_topic` is configured (up to four BMS sensors), the
charger limits the pack current by temperature:

- If the coldest sensor is below `bat_temp_min` (default 0 °C), the same
  load-follower as the partial hold holds the pack current at zero. Loads are
  still served from PV, and discharging a cold pack is fine. The cold hold has
  no voltage floor, so a cold pack may sit at any SoC. The hold releases 2 °C
  above `bat_temp_min`.
- If the hottest sensor is above `bat_temp_derate` (45 °C), the *pack* current
  limit `ibat_max` scales linearly to zero at `bat_temp_max` (55 °C), and the
  load-follower regulates the BMS pack current to that limit. Because it
  regulates the pack current and not this converter's output, it holds on a
  shared bus, where neither converter can tell the load from the sibling.
- Without a live `ibat` (no topic, or the stream stopped for 180 s), both
  fall back to an output-current limit: 0.25 A when cold, and the derated
  `ibat_max` when hot.

The charger drops a sensor whose reading is outside −40…100 °C. Each sensor
expires an hour after its last frame. With none left, the policy switches off
(as without a sensor). Without a topic, nothing changes.
