---
title: Power-loop rig
sidebar_position: 4
---

# Power Loop

Put two Fugu devices in a loop to do hardware measurements, e.g. waveforms of gates, switch nodes and coil current, or
conversion efficiency measurements.

One device is a boost converter, the other a buck converter.
Connect an external power supply to the boost input, which will generate the solar voltage.
Connect boost output to buck input and buck output back to boost input / PSU.

```mermaid
flowchart LR
    PSU["External PSU (current-limited)"] --> BI["Boost input"]
    BI --> BO["Boost output = bus"]
    BO --> KI["Buck input"]
    KI --> KO["Buck output"]
    KO --> BI
```

```
# boost/converter.conf:
topo=boost
forced_pwm=1
pwm_driver=mcpwm

# boost/charger.conf:
vout_max=75


# boost/limits.conf:
vin_min=27 
vout_max=80
```


If the current sensor is on the low side (GND), it will show a current that is much lower than the actual current flowing.
Enable forced pwm buck converter to prevent issues with CCM/DCM detection: 
```
# buck/converter.conf:
topo=buck
forced_pwm=1
pwm_driver=mcpwm

# buck/charger.conf:
vout_max=29

# limits.conf:
vin_max=85
vin_min=72      # only for the stiff-source setup: pins the operating "solar" voltage
                # (the bench buck profile config/lab/fbuck_lab_bench ships vin_min=10.5)
vout_max=60
```

Before closing the loop, read back each converter with a bare `sync` (expect `forced pwm armed`).


* Device low-side current-sensor will not work (both buck & boost)
* the power loop will work with only a little current from the external supply
* put a low current limit on the external supply. It caps the power put into the loop, not the loop current
  (see [Pitfalls](#pitfalls))

## Bring-up order

Both converters run forced PWM (the current sensors are unusable, so diode emulation has no input
it can trust), and that is exactly the mode that has no reverse-current blocking. The loop current
is set by the *mismatch* of the two conversion ratios against the loop resistance:

    I_loop = (D_buck·V_bus − V_psu) / R_loop        R_loop = DCR + 2*Rds, i.e. milliohms

so while the buck's duty is still below `V_psu / V_bus` it does not merely fail to deliver — it
runs backwards, boosting the bus and feeding the external supply, which in CV cannot sink. A duty
ramp from 0 in forced PWM therefore slams the loop with reverse current before it ever reaches the
operating point. That is the failure mode you see as "the buck boosts at low duty".

The order that works:

1. Boost first, with the supply current-limited. Its output cap starts at the supply voltage, so
   `D_zero = 1 − Vin/Vout = 0` and every duty step pushes forward — it can be ramped straight to
   the target ratio.
2. Buck with the **low side diode-emulating**, and ramp its duty. Below `D = Vout/Vin` the LS body
   diode blocks the loop current — not quite nothing: the LS bootstrap keep-alive
   (`board.conf::boot_refresh_ns`, 2 µs) still puts `−Vout` across the coil every period, so
   `ΔI = Vout·t/L ≈ 1 A` peak recirculates through the HS body diode, about a watt. That is 0.05 %
   of what the gate prevents, and it grows linearly with `boot_refresh_ns/L`.
3. Only once the duty is past the ratio — i.e. the converter is in CCM and conducting forward —
   switch the low side to complementary. In CCM diode and synchronous rectification are the same
   operating point, so this transition is a non-event; done at any lower duty it is a short circuit
   between the two nodes through the coil.

Do not expect the crossing itself to be gentle. Against a stiff loop `dI/dD = Vin/R_loop` ≈ 2000 A
per unit duty, so the converter is in CCM within a fraction of a percent of duty and the whole
operating range lives inside a few percent — which is why the gate's margin has to be small, and
why the loop current is trimmed in single PWM counts.

One thing the gated ramp does *not* give you on this rig: synchronous rectification. The
diode-emulation clamp drops the low side to its minimum when the current sensor reads below 10 mA
(`SyncRectOffCurrent`), and on this topology the sensor reads ~0 by construction — so the loop
current freewheels through the LS **body diode** for the whole gated part of the ramp, tens of
watts in that FET at the engage point. That is why the ramp is held while the gate counts out
`fpwm_gate_hold` rather than climbing through it, and why the hold should not be made long without
reason.

`converter.conf::fpwm_gate` (default on) does steps 2 and 3 by itself: `forced_pwm=1` is then a
*request*, and the firmware holds the low side diode-emulating until the gate engages. A bare `sync`
reports `armed`/`engaged` and the duty the gate is waiting for. See
[converter.conf](../reference/config/converter.md#fpwm_gate-fpwm_gate_margin).

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

A stiff CV source has no maximum power point, so the buck's tracker can't actually be
exercised. Instead run the boost in `mode=pv` (see [converter.conf](../reference/config/converter.md)
and the `pv` console command): its output follows a PV curve V=f(Iout)
with a real MPP at `pv_k`·`pv_voc`, and the buck tracks it like a panel. Bench boost
profile: `config/lab/fboost_pv`; runtime entry without re-provisioning: `pv <isc> <voc> [k]`.

Caveats specific to this rig:

* The boost can only emulate the curve **above its input voltage** — the setpoint is
  clamped to Vin+0.5 V, so the steep near-Isc branch is truncated. If Vin rises to
  within 0.5 V of Voc the feasibility latch shuts the mode down.
* **Below the Vin floor no firmware limit controls the current**: the boost body diode
  passes through at Vout ≈ Vin and these boards have no panel-disconnect switch. Keep the
  external supply's current limit low, but it caps power, not loop current (see
  [Pitfalls](#pitfalls)).
* The emulated "solar" power recirculates through the loop; the external supply only
  covers losses. Watch the boost's Iin against `iin_max` when raising `pv_isc`.
* A Vin rise mid-run truncates the curve silently (floor clamp) rather than faulting,
  until it reaches Voc−0.5.

## Pitfalls

* **Low-side current sensing reads ~0.** On this topology a current sensor on the low side (GND)
  of either converter reads ~0 by construction, much lower than the current actually flowing. The
  diode-emulation clamp (`SyncRectOffCurrent`, 10 mA) then keeps the low side at its minimum, so
  synchronous rectification cannot work from that sensor — run forced PWM and do not trust the
  reported currents on this rig.
* **No reverse-current blocking in forced PWM.** Both converters run `forced_pwm=1`, the mode with
  no reverse-current blocking. While the buck duty is below `V_psu / V_bus` the buck runs
  backwards, boosting the bus and feeding the external supply. Low duty is the dangerous end, and
  the current-based trips cannot be counted on here (the sensors read ~0). Keep `fpwm_gate`
  enabled. It blocks the destructive ramp from 0, but it does not prove forward current: a settled
  or descending duty may run in reverse (see [Bring-up order](#bring-up-order)).
* **Shut down in reverse order**: bring the boost duty down first, then the buck to zero.
  Reversing it unloads a still-pumping boost into its reverse-current trip.

:::danger The PSU current limit is not a loop-current limit
Keep a low PSU current limit. It caps the power put into the loop, not the loop current.
Circulating current is set by the ratio mismatch and can reach tens of amps within that power
budget. The limit does nothing against returned energy or discharge of the bus capacitance. The
firmware's current-based trips are blind on this rig, but its voltage trips still act. Provide a
separate way to break the loop (switch or fuse sized for the loop current), watch loop current with
an independent clamp, and discharge the bus before rewiring.
:::

### Measurement traps

* **Confirm the buck really runs forced PWM** (a bare `sync` reads it back). Without it the buck
  silently runs DCM, and any loss analysis that assumes CCM is wrong for every point.
* **Scope ground springs, not ground clips.** A ground clip fabricated a ~15 MHz mode that outranked
  the real switch-node ring in a mode/spectrum ranking and led to a retracted conclusion.
* **Record which board and which probes** produced each capture. Identity assumed from context
  misidentified a probe for a whole session.
* **A ranked mode table is not an identification** — read the modes below the first one; an
  artifact can outrank the real signal.
