
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
