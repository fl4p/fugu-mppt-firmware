---
title: "Rectifier timing calibration"
sidebar_position: 9
description: "rect-cal sweeps the low-side window across the inductor zero crossing on a running buck and stores the timing correction rect_offset_ns."
---

# Rectifier timing calibration

`rect-cal` measures the [timing correction](diode-emulation.md#timing-correction-rect_offset_ns)
`coil.conf::rect_offset_ns` on a running buck (`src/rect_cal.h`). It runs the
[bench LS sweep](diode-emulation.md#bench-measurement) itself.

It lowers HS to `0.55·M` of the
period and holds it there. That point draws well below the panel's MPP current, so `V_i` sits on
the flat, voltage-source part of the PV curve near `V_oc`. At the MPPT operating point itself the
panel limits the power, the LS window hardly changes the output current, and the located peak
lands late, toward a longer LS.

## Preconditions and automatic runs

A run needs:

- buck topology;
- the converter on in MPPT mode (not in PSU or PV emulator mode or manual PWM, no ADC calibration
  running);
- no sweep or [peak scan](mppt-tracker.md), no forced PWM, no manual LS count, no queued dead-time change, no limiter
  active;
- sync rect and backflow switch on;
- a physical `I_out` sensor reading at least 0.3 A (averaged);
- averaged `V_i > V_o + 1 V`;
- an HS count at or above the calibration HS of `0.55·M`, which must itself lie above the HS
  minimum.

`rect-cal` further needs DCM and at most 4 A. At the calibration point the run further
needs more than 0.15 A and LS headroom: the sweep cap, the smallest of the complementary limit, 90 %
of the off-time limit `(1 − V_o/V_i)·T` and the ideal window + 2.5 µs, must reach at least the ideal
window + 1.25 µs (`LS headroom`).

With `coil.conf::rect_cal_auto=1` (default) the firmware starts a run (`rect-cal`, not `force`) by
itself while the offset is uncalibrated, at most once a day, not in the first 10 minutes after
boot and only at a steady operating point above 5 W. A refused start is retried after 10 minutes, a failed run after a day.
A bound is retried only while the operating `V_i` is below `2·V_o` (the calibration point sits
higher on the PV curve, so this is necessary, not sufficient) and not within 7 days of a bound
result since boot: a board that always operates above `2·V_o` gives a bound on every run and
would otherwise spend a run of 35–40 s every day.

## Run start

`rect-cal` starts from a DCM operating point at 0.3–4 A. `rect-cal force`
starts from any MPPT operating point above the calibration HS, including CCM at full power. Both
take the duty over from the tracker and ramp HS to the calibration point by `pwmMax/512` counts
per control sample (about 0.3 s from a full-power operating point at ~450 samples/s).

Before the first HS step the LS is pinned at its minimum (`pwmRectMin`), and it stays there through the ramp
and the settling phase. The converter is then a non-synchronous buck: the body diode carries the
freewheel current and no reverse current can build, however fast the current falls through the
CCM/DCM boundary and however late the CCM/DCM decision follows it. Lowering the duty only moves
every limiter away from its limit (power, currents and `V_o` fall, `V_i` rises), so the ramp cannot
make a limiter act. `V_i` rises toward `V_oc`; the run stops at 97% of the input over-voltage trip
(`limits.conf::hv_max`) before the trip itself.

## Settling

At the calibration HS the run waits until the converter reports DCM and two
consecutive 0.5 s windows (each after 150 ms settling) agree: `I_out` within 0.02 A + 2%, `V_i`
and `V_o` within 0.3%. It gives up 12 s after the start: `HS ramp` if HS has not reached the
calibration point, `not in DCM`, or `unsettled`. A settling window with `V_o` at or below 0.5 V or
`V_i` not above `V_o + 1 V` fails with `Vin<=Vout`, and a non-finite averaged reading in any
window with `bad sample`.

## LS sweep

The firmware computes the ideal window from the averaged voltages of the last settling
window (`(V_i/V_o − 1)·HS`, without the worst-case ratio margin) and steps LS from 0.5× to 1.4× of
it in 24 steps. Each step settles for 150 ms and averages `I_out` for 500 ms. Between steps, a
reference LS of 0.6× ideal is held for 400 ms. Each point is the step's current minus the mean of
the two references around it, which cancels any change of the source that is linear in time, such
as slowly changing irradiance. Each point is placed relative to the ideal window of its own
averaged `V_i` and `V_o`, so a voltage drift moves the coordinate with the zero crossing instead of
reading as a timing error.

The coarse sweep stops one step after the first clear drop, a step at least 10 mA below the step
before and 20 mA below the best point so far. If the step after it is still 10 mA below the best
point, the drop stands. If it is back up, the drop is measured again: a second low reading confirms
it (a ring dip keeps the lower of both), otherwise it was an outlier, its point is replaced and the
coarse sweep continues.

After a confirmed drop the run measures 10 more points spread evenly over
the four coarse intervals before the drop, never later than it, which places points about every
20–30 counts where the bend starts. The sweep also stops when a step's averaged output current
falls below −0.05 A.

The LS stays within 2.5 µs of the ideal window and within 90% of the off-time volt-second limit
`(1 − V_o/V_i)·T` (see [switch-node ringing](#switch-node-ringing)); a run whose cap leaves less
than half of that 2.5 µs beyond the ideal window fails with `LS headroom`.

## Signal size

With the LS held `Δt` past the inductor zero crossing, reverse current builds to
`V_o·Δt/L` and returns through the HS body diode at the slope `(V_i − V_o)/L`. The output loses

$$
\Delta I_{out} \approx \frac{f_{sw}\, V_o\, V_i}{2 L\,(V_i - V_o)}\,\Delta t^2 .
$$

With the LS turned off `Δt` early, the body diode carries the rest of the fall at the slope
`(V_o + V_f)/L`, and the output loses only `ΔI_out ≈ f_sw·V_f·Δt²/(2L)`, 60–90× less at `V_f` = 0.7 V. Both
terms are independent of the operating current: the peak is located by the curvature of the late side,
which is set by `L`, `V_i` and `V_o`. With 6.25 ns counts and `V_o` = 26.5–27.8 V:

| `L` | `V_i` | `I_out` at the calibration point | 50 counts late | 100 counts late | 200 counts late | 1.4× ideal | early side, 0.5× ideal |
|---|---|---|---|---|---|---|---|
| 56 µH | 51–70 V | 0.9–1.2 A | 1.4–2.1 mA | 5.8–8.3 mA | 23–33 mA | 140–186 mA | 2.5–4.7 mA |
| 80 µH | 62–78 V | 0.7–0.9 A | 1.0–1.2 mA | 3.8–4.8 mA | 15–19 mA | 118–139 mA | 2.6–3.7 mA |

A higher operating current widens the swept range in counts (the ideal window scales with the
peak current) but does not steepen the curve, so the calibration point stays at `0.55·M`. A sweep
that has not yet passed the zero crossing shows only the early side, a few mA in total, which is
within the noise of a short run.

These are lossless estimates. Measured late sides can be much steeper, and the early side can fall
across the sweep instead of rising by a few mA. The run therefore assumes neither the size of the knee nor a flat early side (see
[locating the zero crossing](#locating-the-zero-crossing)).

## Voltage changes

In DCM at fixed HS the output current is
`D²·V_i·(V_i − V_o)/(2·f_sw·L·V_o)`. 1% of `V_o` changes it by about 1.5–2.2% (13–19 mA at the
calibration point), more than the signal within 100 counts of the peak. A step of the battery
voltage during one measurement window would therefore read as a current peak. Two measures keep
voltage changes out of the result:

- Every window current is scaled to the voltages of the settling window with that model,
  `I·g(V_i0, V_o0)/g(V_i, V_o)` with `g = V_i·(V_i − V_o)/V_o`.
- The averaged `V_o` of each step must lie within 0.2% of the mean of its two references, and
  `V_i` within 0.3%. The windows are equally spaced, so a linear drift passes this check and a
  step does not. A step that fails is measured again, up to 6 times per run, so a single bus
  transient costs about one second; the seventh failure ends the run (`V step`). A `V_o` reading
  that is off by more than 0.2% on both tries at the same LS, in the same direction and within
  0.2% of each other, while `V_i` passes, follows the LS window rather than the bus and ends the
  run at once (`sense follows LS`, see [sense path](#sense-path)).

The knee itself moves `V_i`. Reverse current lowers the power drawn from the panel by the
fraction `ΔI/I` of the measured (unscaled) output current drop, and on the PV curve
`ΔV_i/V_i = S·ΔI/I` with `S = |dV/dI|·I/V`, the panel's slope relative to its static
resistance. `S` is 1 at the MPP and smaller on the voltage-source side where the run operates:
about 0.05–0.07 at an `I_sc` of 1.5–2 A with 0.6 A drawn near a 76 V `V_oc`. There, a 160 mA knee
lifts `V_i` by 0.4%, and the next step by about 1%, so a fixed 0.3% limit rejects exactly the
signal the run looks for.

A step's `V_i` may therefore exceed its references by 0.3% plus
`0.25·ΔI/I`, but only upward and only by what the step's own measured current drop explains. A
model error cannot create that allowance: if the plant current does not follow `V_i` at all, the
raw current shows no drop and the limit stays 0.3%. `V_o` gets no allowance: the knee moves it by
`R_bus·ΔI`, under 0.1% for a 0.4 A knee on a 40 mΩ bus.

`V_i` of the reference windows must stay within 2% of the settling value (`Vin shift`): a cloud
lowers `V_oc` by about 2% for a 30% drop in irradiance. `V_o` may drift by up to 5% (`Vout
shift`): on a battery bus shared with other chargers and loads, `V_o` moves with their current,
independent of the run. A window whose voltage scaling factor `g(V_i0, V_o0)/g(V_i, V_o)` lies
outside 0.8–1.25 fails the run (`V shift`): the voltages may move the modelled current by at most a
25 % rise or a 20 % drop against the settling window.

These checks reject `V_o` dips of 0.3 % and more during the steps past the crossing.

## Sense path

Before the zero crossing the LS window can only raise `I_out` by the body-diode
loss it recovers, a few mA (0.3% of the current or less between 0.5× and 1× ideal), and reverse
current only lowers it. A current reading that climbs with the LS window therefore comes from the
sense path: switching noise coupled into a current and voltage monitor that samples during the LS
on-time. Such a monitor usually disturbs its voltage channel as well, so a run that trusts neither
reading is refused rather than corrected.

Once both of two consecutive coarse points up to the
ideal window read more than `20 mA + 0.4%·I` (`I`: the settling current) above the higher of the
first two points, the run ends with `Iout follows LS`. The bound grows with the current because
the diode loss does; the switch-node ring adds 10–15 mA on an 80 µH board and noise does not
scale. `rect-cal status` prints the largest rise and its bound.

## Switch-node ringing

While both switches are off, the inductor rings with the switch-node
capacitance `C` (FET output capacitances, driver and layout) at `Z_0 = √(L/C)`; the mechanism and
the field fits of `C` and `Q` are in [Switch-node ringing in
DCM](../lab/coil-inductance.md#switch-node-ringing-in-dcm). In DCM this is the ring around `V_o`
that ripples `I_out` against the duty. [DCM coil ringing](dcm-ringing.mdx) covers the tank and the
node capacitance per board.

When the LS turns off with a reverse current `I_r`, the node
swings up from 0 V; it reaches `V_i`, where the HS body diode clamps it and the remaining current
returns to the input, only if `I_r·Z_0 ≥ √((V_i − V_o)² − V_o²)`. A smaller reverse current rings
back into the inductor instead and is not lost. The steep part of the late side therefore starts a
fixed reverse current after the zero crossing, about 0.35 A at `V_i` = 78 V, `V_o` = 26.5 V and
`Z_0` = 126 Ω (80 µH, 5 nF), which is 170 counts of 6.25 ns.

The coil current left over from the
ring when HS turns on shifts the next switching period. This tilts the early side by a few mA and,
past the bend, makes the curve non-monotonic: the loss reaches a minimum about 250 counts past the
crossing, recovers, and turns into a gain around 1.2× ideal. The converter model with this node is
described in [vconv.conf](../reference/config/vconv.md#switch-node-ring).

HS also turns on into the ring. The coil current it finds there, up to about `±V_o/Z_0` depending on
the ring phase, adds to the peak current and moves the zero crossing by that current times `L/V_o`,
up to `±√(LC)`. The window `(V_i/V_o − 1)·HS` does not contain it. The phase follows the off-time
after the crossing, `T − HS·V_i/V_o`, so the shift differs at every operating point and no constant
offset can follow it. At the run's calibration point the crossing lies up to about 0.4 µs before
and 0.75 µs after the ideal window. Holding the LS past the crossing changes the ring
current HS turns on into: with a negative ring current the output current bends up at the crossing;
with a positive one it stays flat past the crossing and bends down only at the clamp onset.

Two consequences follow for the run:

- A downward bend fit alone finds the steep loss, between the zero crossing and the clamp onset,
  and misses an upward bend at the crossing (see [locating the zero crossing](#locating-the-zero-crossing)). Points past the
  loss minimum pull the fit far early, which is why the sweep stops one step after the first clear
  drop and the fit ends at the floor of the first dip. After the margin, the applied offset lands
  before the crossing.
- An LS longer than the off-time `(1 − V_o/V_i)·T` leaves reverse current in the inductor when
  HS turns on. The converter then runs as a synchronous buck backward from the battery, and the
  reverse current grows by several amperes within a few periods until the reverse-current guard
  clamps the LS. Because the calibration HS is set from the voltages before the ramp, while `V_i`
  rises toward `V_oc` afterward, this limit can sit at only 1.3× the ideal window, inside a
  0.5–1.4× sweep. The 90% cap keeps the sweep below it.

With the cap and the stop rule, the LS ends at most 2.5 µs past the ideal window, so the peak
reverse current is bounded by `V_o·2.5 µs/L` plus the LS turn-off delay: about 1 A at 80 µH and
1.5 A at 51 µH with `V_o` = 26.5 V. A sweep that stops one coarse step past the first clear drop
reaches about half of that.

## Run aborts

The run is aborted, and the LS returned to diode emulation, on any of these:

- `rect-cal abort` (`aborted`);
- a limiter (`limiting`);
- a mode change (`mode change`, `not in MPPT mode`), a sweep or peak scan (`sweeping`) or a
  forced-PWM request (`forced PWM`);
- the input voltage above 97% of its trip (`Vin near max`);
- leaving DCM (`left DCM`) or a change of HS (`HS moved`) after the settling phase;
- 60 s run time (`timeout`);
- any protection action (`protection`): a shutdown, the backflow switch or sync rect turned off,
  or the LS clamped to its minimum.

The protection check runs right before each
LS update of the run, after the protection pass of the same sample, so the run never raises an LS
that a protection has just lowered.

Afterwards HS fades back to the operating point it started from, unless the converter is off,
the backflow switch is open (reverse-current guard), the mode changed or a sweep or peak scan is
running; then the tracker continues from where the run stopped.

## Locating the zero crossing

A DCM triangle of peak current `p` carries
`p²·L·V_i/(2·V_o·(V_i − V_o))` per period and ends `p·L/V_o` after HS turns off. This holds
whatever coil current HS turned on into (to second order) and however long HS really conducted, so
the measured current places the actual zero crossing `zc` without knowing the ring phase: the run
uses the mean current of its steps at 0.8–1× ideal (below the crossing), the PWM period and the
coil inductance of the converter model, `0.95·L0`.

Since `zc ∝ √L`, an inductance error of `ε`
moves `zc` by `ε/2` of the window. With the model inductance equal to the plant's, `zc` reads
somewhat early: the ring returns about `C·V_o` per period from the output to the node, which the run
cannot subtract without the node capacitance. A model inductance above the plant's moves `zc` late.

The run then fits a line with a quadratic bend from `x_0` on,
`y = c_0 + c_1·d + c_2·min(0, d)² + c_3·max(0, d)²` with `d = x − x_0`, and searches `x_0` over the
sweep. Two bends are fitted: a downward one (`c_2 ≤ 0`, `c_3 < c_2`) over all points up to a dip
floor (below) and, if the highest of them lies past `zc`, an upward one (`c_3 > max(c_2, 0)`)
through that point. Each must exceed 10 mA and 4× its residual RMS at the last point, counting
only the bend it adds to the early side's own curvature; otherwise a concave early side and a
noise point past `zc` pass as an upward bend. The earlier bend is the crossing in commanded counts.

The line takes any slope of the early side, so `x_0` is where the curve starts to bend, not where
it is highest. With a falling early side, the maximum sits at the start of the sweep and a peak
fit lands far before the crossing (about −190 counts on the 80 µH field data, where the bend fit
finds −9 and +47).

Past the bend the ring can turn the current back up, and the points of a recovery pull the
quadratic, and with it `x_0`, early. Once two consecutive points lie at least 10 mA below the
highest point and a later one rises 10 mA above the lowest point since, the fit uses the points up
to that lowest one, the floor of the first dip. An upward bend at the crossing comes before any
dip and stays in. A dip followed by a recovery smaller than 10 mA is fitted in full. Ending at the
floor leaves few points past the bend, so the fit can land after the knee when a dip floor and a
partial recovery follow it closely.

`zc − ideal` contains the ring current at HS turn-on, which no constant offset can follow, but also
the extra charge of an HS that conducts longer than commanded and half of a voltage sensing error
(`zc` depends half as strongly on `V_o` as the window does). The run therefore learns
`learned = x_0 − max(zc, ideal)`: the delays of driver, gate and sensing between the commanded and
the real LS turn-off, rounded early. An HS that conducts `τ` longer than commanded moves the real
crossing by `τ·V_i/V_o`, of which the result contains about `τ`, which leaves the LS early.

The run is rejected, and the previous value kept, in any of these cases:

- fewer than 6 points up to the dip floor (`few points`);
- no step at 0.8–1× ideal (`no points below ideal`);
- `|zc − ideal| > 2.5 µs` (`charge model`: `L0` or the current reading far off);
- no bend (`no knee`: the bend at the last point under 10 mA or under 4× the residual RMS);
- `|x_0 − max(zc, ideal)| > 2.5 µs` (`implausible`).

Otherwise the applied offset is
`learned − max(125 ns, 0.3·|learned|)`, or `learned − max(250 ns, 0.3·|learned|)` when the fit
ended at a dip floor, at most +125 ns. With the larger floor, the offset applied in that
partial-recovery case stays before the knee. The margin grows with the offset,
because a large offset comes from large unmodelled delays; a fixed fraction of `learned` would move
toward a longer LS when `learned` is negative. The result takes effect at once.

## Late results

A late LS turn-off builds reverse current at `V_o/L` per second. When the LS turns
off, that current drains through the HS body diode into the input at `(V_i − V_o)/L`, so it takes
`M/(1 − M)` times the late time to return to zero: one late time at `M = 0.5`, four at `M = 0.8`.
Near `D = M` the 2% ratio margin of the diode-emulation window (about `0.02·T` = 0.5 µs at 39 kHz)
is all that keeps the LS from filling the off-time, and a `V_o` reading 1.5–2% low (or `V_i` as much
high) uses it up. A positive offset added on top then either reaches the complementary limit
`T − HS − 1`, where the LS turns off only when HS turns on and the converter runs backward from the
battery in forced CCM, or leaves a reverse current that has not drained when HS turns on. An
offset of +125 ns with `V_o` read 1.7 % low is enough for both.

The firmware therefore applies a positive offset only as far as it leaves the LS off for `T/32`
(0.8 µs at 39 kHz) plus the drain time `offset·M/(1 − M)` before the next HS turn-on, not at all
where the window without it already ends later, and not at all above `M = 0.8` (worst-case ratio),
where the drain takes more than four times the offset. Together, the three parts keep any offset
up to +250 ns from turning a positive mean output current negative with readings up to 2.5 % off.
The window without an offset has its own [drain limit](diode-emulation.md#reverse-current-drain-near-d--m); it
applies on top of the offset rule.

The run applies a late result up to +125 ns, and only from a calibration point below `V_i = 2·V_o`
(see [bounds](#bounds)). The gain of a late correction is small: the body diode carrying the last 0.5 µs of the fall costs
about 1 mW. A stored `rect_offset_ns` above +125 ns (compared in counts at the active tick rate) is
used as +125 ns and counts as a bound; so does a `measure-coil ls apply` result, which as a bench
tool does not apply the `V_i = 2·V_o` rule of the run.

## Bounds

Above `V_i = 2·V_o` the reverse current must first lift the switch node to `V_i`
before it is lost, which can put the downward bend after the zero crossing by a lag that the node
capacitance sets. Driver and gate delays explain tens of nanoseconds, and a single run scatters by
about 0.2 µs. A late result from a calibration point above `V_i = 2·V_o`, or one whose applied
offset would exceed +125 ns below it, is therefore a bound, not a measurement: the run applies the
shorter of the limit (0 above `V_i = 2·V_o`, +125 ns below) and the offset in use, and never moves an
active offset later. A shorter measured offset stays as it is and keeps its state. Otherwise the
value is stored with `rect_offset_cal=2`, which `rect-cal status` shows as `bound` also after a
reboot.

## Saving the result

Writing the result to `coil.conf` (`rect_offset_ns`, `rect_offset_drv`, `rect_offset_cal`) waits
until the converter has been off without interruption for 10 s with a valid output-power reading
below 1 W, typically at night: a flash write that erases a sector stalls both cores, including the
control loop, for tens of milliseconds. Any converter start, however short, restarts the 10 s.

The control loop then blocks converter starts until the write is done: it grants the request,
the low-frequency task admits the write within milliseconds and rewrites `coil.conf` once, and
starts are allowed again as soon as it finishes, however long the write takes. A grant that is not
admitted within 8 s is withdrawn. Only starts are blocked; protection and shutdown are not.

The new `coil.conf` is written to `coil.conf.cft` and renamed over the old file, which LittleFS does
atomically, so a reset during the save leaves either the old or the new file complete; a leftover
`.cft` file is deleted at boot. Console configuration commands (`set-config`, `del-config`,
`svc`) write the same way; FTP uploads and the USB config drive have their own write paths. A start
that falls into a save is delayed by the save, a fraction of a second on an ESP32-S3. A failed or
withdrawn save is retried a minute later; successful writes are at least 10 minutes apart.

An unsaved value meets restarts and edits as follows:

- A software restart (`restart`, an OTA update over Wi-Fi or BLE including its timeout, the network
  watchdog, the task watchdog) writes it at once, without the 10 s off time: it first blocks
  converter starts and switches the converter off, so the stall cannot hit a running converter.
- Restarts from the control loop (ADC stall, no sample) and resets that skip the restart path
  (panic, brown-out, hardware watchdog, the no-output watchdog) lose it.
- Editing a `rect_offset*` key of `coil.conf` (`set-config`, `del-config`) or committing the USB
  config drive drops it, so the edit is kept.

What is written is always the offset in use at that moment together with its state (measured, bound or
uncalibrated): if the [runtime backstop](diode-emulation.md#runtime-backstop) has shortened a fresh result before it was saved, the shorter
value is saved as uncalibrated.

## Failure reasons

A run ends without a result, and the previous value stays, on any refusal, abort or failure.
`rect-cal status` and the log name the reason.

A refused start names the first unmet [precondition](#preconditions-and-automatic-runs):
`converter off`, `boost`, `not in MPPT mode`, `sweeping`, `forced PWM`, `sync rect/backflow off`,
`no Iout sensor`, `manual LS`, `dead-time change queued`, `limiting`, `not in DCM`,
`Iout outside 0.3..4 A` (`Iout below 0.3 A` with `force`), `Vin<=Vout+1` or
`HS below calibration point`.

A started run fails with one of the [abort](#run-aborts) reasons or with `HS ramp`, `not in DCM`,
`unsettled`, `Vin<=Vout`, `low I`, `LS headroom`, `bad sample`, `Vin shift`, `Vout shift`,
`V shift`, `V step`, `sense follows LS`, `Iout follows LS`, `few points`, `no points below ideal`,
`charge model`, `no knee` or `implausible`.
