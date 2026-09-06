
put 2 Fugu devices in a loop to do hardware measurements, e.g. waveforms of gates, switch nodes and coil current or
conversion effiencency measurements.

One Device is a buck converter the other the buck converter
Connect an external power supply to the boost input, which will generate the solar voltage.
Connect boost output to buck input and buck output back to boost input / PSU.

```
# boost/converter.conf:
boost=1
vout_max=75


# boost/limits.conf:
vin_min=27 
vout_max=80
```


If the current sensor is on the low side (GND), it will show a current that is much lower than the actual current flowing.
Enable forced pwm buck converter to prevent issues with CCM/DCM detection: 
```
buck/converter.conf:
boost=0
forced_pwm=1
vout_max=29

# limits.conf:
vin_max=85
vin_min=72      # only for the stiff-source setup: pins the operating "solar" voltage
                # (the checked-in fbuck_lab_bench profile ships vin_min=10.5)
vout_max=60
```


* Device low-side current-sensor will not work (both buck & boost)
* the power loop will work with only a little current from the external supply
* put a low current limit on the external supply to protect the devices

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
*request*, and the firmware holds the low side diode-emulating until the effective duty passes the
measured ratio while the duty is still climbing, engaging once it is provably forward — or, if the
duty settles at the ratio first (the normal boot state here, where the loop is still open and the
converter parks at `D = Vout/Vin`), once it has stood still for `fpwm_gate_hold`. Either way it lets
go on the way down, so `dc 0` also ramps down cleanly. A bare `sync` reports
`armed`/`engaged` and the duty the gate is waiting for. See
[Configuration.md](Configuration.md#fpwm_gate-fpwm_gate_margin).

## PV-sim source (solar-array-simulator)

A stiff CV source has no maximum power point, so the buck's tracker can't actually be
exercised. Instead run the boost in `mode=pv` (see [Configuration.md](Configuration.md)
converter.conf and the `pv` console command): its output follows a PV curve V=f(Iout)
with a real MPP at `pv_k`·`pv_voc`, and the buck tracks it like a panel. Profile:
`config/lab/fboost_pv`; runtime entry without re-provisioning: `pv <isc> <voc> [k]`.

Caveats specific to this rig:

* The boost can only emulate the curve **above its input voltage** — the setpoint is
  clamped to Vin+0.5 V, so the steep near-Isc branch is truncated. If Vin rises to
  within 0.5 V of Voc the feasibility latch shuts the mode down.
* **Below the Vin floor no firmware limit controls the current**: the boost body diode
  passes through at Vout ≈ Vin and these boards have no panel-disconnect switch. The
  external supply's current limit is the real backstop — keep it low.
* The emulated "solar" power recirculates through the loop; the external supply only
  covers losses. Watch the boost's Iin against `iin_max` when raising `pv_isc`.
* A Vin rise mid-run truncates the curve silently (floor clamp) rather than faulting,
  until it reaches Voc−0.5.
