---
title: Power-loop rig
sidebar_position: 4
---

# Power loop

The power-loop rig connects two Fugu devices in a loop for hardware measurements, such as waveforms of gates, switch
nodes, and coil current, or conversion efficiency.

One device is a boost converter, and the other is a buck converter. Wire the loop as follows:

1. Connect an external power supply to the boost input. The boost generates the solar voltage.
2. Connect the boost output to the buck input.
3. Connect the buck output back to the boost input and the PSU.

The following diagram shows the loop:

```mermaid
flowchart LR
    PSU["External PSU (current-limited)"] --> BI["Boost input"]
    BI --> BO["Boost output = bus"]
    BO --> KI["Buck input"]
    KI --> KO["Buck output"]
    KO --> BI
```

Both converters run forced PWM. Configure the boost with these settings:

```
# boost/converter.conf:
topo=boost
forced_pwm=1

# boost/charger.conf:
vout_max=75


# boost/limits.conf:
vin_min=27 
hv_max=80
```

A current sensor on the low side (GND) shows a current that is much lower than the actual current flowing. To prevent
issues with CCM/DCM detection, enable forced PWM on the buck converter as well:

```
# buck/converter.conf:
topo=buck
forced_pwm=1

# buck/charger.conf:
vout_max=29

# buck/limits.conf:
hv_max=85
vin_min=72      # only for the stiff-source setup: pins the operating "solar" voltage
                # (the bench buck profile config/lab/buck_bench ships vin_min=10.5)
lv_max=60
```

Before closing the loop, read back each converter with a bare `sync` (expect `forced pwm armed`).

The low-side current sensors of both devices don't work in this loop. The loop works with only a little current from
the external supply. Put a low current limit on that supply, but note that it caps the power put into the loop, not
the loop current (see [Pitfalls](#pitfalls)).

## Bring-up order

Both converters run forced PWM, the mode with no reverse-current blocking. Diode emulation has no input it can trust,
because the current sensors are unusable. The *mismatch* of the two conversion ratios against the loop resistance
sets the loop current:

    I_loop = (D_buck·V_bus − V_psu) / R_loop        R_loop = DCR + 2*Rds, i.e. milliohms

While the buck's duty is still below `V_psu / V_bus`, the buck runs backwards. It boosts the bus and feeds the
external supply, which in CV cannot sink. A duty ramp from 0 in forced PWM therefore drives reverse current through
the loop before it reaches the operating point. This failure mode shows up as "the buck boosts at low duty".

Bring the converters up in this order:

1. Ramp the boost first, with the supply current-limited. Its output cap starts at the supply voltage, so
   `D_zero = 1 − Vin/Vout = 0` and every duty step pushes forward. You can ramp it straight to the target ratio.
2. Ramp the buck's duty with its low side diode-emulating. Below `D = Vout/Vin`, the LS body diode blocks most of
   the loop current. The LS bootstrap keep-alive (`board.conf::boot_refresh_ns`, 2 µs) still puts `−Vout` across
   the coil every period, so `ΔI = Vout·t/L ≈ 1 A` peak recirculates through the HS body diode, about a watt. That
   is 0.05 % of what the gate prevents, and it grows linearly with `boot_refresh_ns/L`.
3. Once the duty is past the ratio (the converter is in CCM and conducting forward), switch the low side to
   complementary. In CCM, diode and synchronous rectification are the same operating point, so this transition is a
   non-event. At any lower duty, the transition is a short circuit between the two nodes through the coil.

`converter.conf::fpwm_gate` (default on) does steps 2 and 3 by itself. With the gate on, `forced_pwm=1` is a
*request*, and the firmware holds the low side diode-emulating until the gate engages. A bare `sync` reports
`armed`/`engaged` and the duty the gate is waiting for. See
[converter.conf](../reference/config/converter.md#fpwm_gate-fpwm_gate_margin).

The crossing itself is abrupt. Against a stiff loop, `dI/dD = Vin/R_loop` ≈ 2000 A per unit duty. The converter is
in CCM within a fraction of a percent of duty, and the whole operating range lies within a few percent. That is why
the gate's margin has to be small and the loop current is trimmed in single PWM counts.

The gated ramp gives no synchronous rectification on this rig. The diode-emulation clamp drops the low side to its
minimum when the current sensor reads below 10 mA (`SyncRectOffCurrent`), and on this topology the sensor reads ~0 by
construction. The loop current therefore freewheels through the LS body diode for the whole gated part of the ramp,
which puts tens of watts in that FET at the engage point. For this reason, the ramp holds while the gate counts out
`fpwm_gate_hold` instead of climbing through it. Keep the hold short unless you have a reason to lengthen it.

:::danger The gate does not guarantee forward current
`D₀` is the duty at zero coil current, `err` the worst-case error of the measured voltage ratio and
`margin` is `fpwm_gate_margin`. While the duty is climbing the gate engages only above
`D₀+err+margin` (provably forward). If the duty stands still for `fpwm_gate_hold` it engages from
`D₀−err`, which can be below zero current.
Once engaged it stays engaged down to `D₀−err−margin`. In this loop the output is always stiff, so
expect reverse current of tens of amps at the settled engage point and during ramp-down (see
*Know what the settled path costs* in
[converter.conf](../reference/config/converter.md#fpwm_gate-fpwm_gate_margin)). Watch loop current
with an independent clamp, and have a way to cut the PSU and the loop before starting.
:::

## PV-sim source (solar-array-simulator)

To exercise the buck's tracker, run the boost in `mode=pv` (see
[converter.conf](../reference/config/converter.md) and the `pv` console command). A stiff CV source has no maximum
power point, so the tracker has nothing to find. In `mode=pv`, the boost output follows a PV curve V=f(Iout) with a
real MPP at `pv_k`·`pv_voc`, and the buck tracks it like a panel. The bench boost profile is `config/lab/boost_pv`.
To enter the mode at runtime without re-provisioning, use `pv <isc> <voc> [k]`.

The following caveats are specific to this rig:

* The boost can only emulate the curve above its input voltage. The setpoint is clamped to Vin+0.5 V, so the steep
  near-Isc branch is truncated. If Vin rises to within 0.5 V of Voc, the feasibility latch shuts the mode down.
* A Vin rise mid-run truncates the curve silently (floor clamp) rather than faulting, until it reaches Voc−0.5.
* Below the Vin floor, no firmware limit controls the current. The boost body diode passes through at Vout ≈ Vin,
  and these boards have no panel-disconnect switch. Keep the external supply's current limit low, but it caps power,
  not loop current (see [Pitfalls](#pitfalls)).
* The emulated "solar" power recirculates through the loop, and the external supply only covers losses. Watch the
  boost's Iin against `lv_i_max` (Iin max in a boost) when raising `pv_isc`.

## Pitfalls

The rig has three pitfalls:

* Low-side current sensing reads ~0. On this topology, a current sensor on the low side (GND) of either converter
  reads ~0 by construction, much lower than the current actually flowing. The diode-emulation clamp
  (`SyncRectOffCurrent`, 10 mA) then keeps the low side at its minimum, so synchronous rectification cannot work from
  that sensor. Run forced PWM, and don't trust the reported currents on this rig.
* Forced PWM has no reverse-current blocking. Both converters run `forced_pwm=1`. While the buck duty is below
  `V_psu / V_bus`, the buck runs backwards, boosting the bus and feeding the external supply. Low duty is the
  dangerous end, and the current-based trips cannot be counted on here (the sensors read ~0). Keep `fpwm_gate`
  enabled. It blocks the destructive ramp from 0, but it does not prove forward current. A settled or descending duty
  may run in reverse (see [Bring-up order](#bring-up-order)).
* Shutdown must run in reverse order. Bring the boost duty down first, then the buck to zero. The opposite order
  unloads a still-pumping boost into its reverse-current trip.

:::danger The PSU current limit is not a loop-current limit
Keep a low PSU current limit. It caps the power put into the loop, not the loop current.
Circulating current is set by the ratio mismatch and can reach tens of amps within that power
budget. The limit does nothing against returned energy or discharge of the bus capacitance. The
firmware's current-based trips are blind on this rig, but its voltage trips still act. Provide a
separate way to break the loop (switch or fuse sized for the loop current), watch loop current with
an independent clamp, and discharge the bus before rewiring.
:::

### Measurement traps

Avoid these traps when you measure on the rig:

* Confirm that the buck really runs forced PWM (a bare `sync` reads it back). Without forced PWM, the buck silently
  runs DCM, and any loss analysis that assumes CCM is wrong for every point.
* Use scope ground springs, not ground clips. A ground clip fabricated a ~15 MHz mode that outranked the real
  switch-node ring in a mode/spectrum ranking and led to a retracted conclusion.
* Record which board and which probes produced each capture. Identity assumed from context misidentified a probe for
  a whole session.
* A ranked mode table is not an identification. Read the modes below the first one, because an artifact can outrank
  the real signal.
