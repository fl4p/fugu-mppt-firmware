---
title: vconv.conf
sidebar_position: 18
---

# vconv.conf

`vconv.conf` configures the virtual converter plant, which replaces the power stage and ADC with a simulated PV source, battery, and passives. It applies to `CONFIG_FUGU_WITH_VCONV` builds with `sensor.conf` `adc = vconv`. `src/adc/sensor_setup.cpp` and `src/adc/vconv.h` read it. For an example, see [`config/lab/vconv_mock`](../../../../config/lab/vconv_mock/conf/vconv.conf).

The simulation takes the inductor from `coil.conf::L0` and the topology from `converter.conf::topo` (`buck`/`boost`). This file doesn't set either.

The file accepts the following keys.

| key             | unit | type  | default | description                                                                 |
|-----------------|------|-------|---------|-----------------------------------------------------------------------------|
| `isc`           | A    | float | 8.0     | PV short-circuit current                                                    |
| `voc`           | V    | float | 40.0    | PV open-circuit voltage                                                     |
| `pv_k`          |      | float | 0.8     | V_mpp / Voc                                                                 |
| `v_bat`         | V    | float | 28.0    | Battery sink voltage (`v_bat=0`, `r_bat=1e9` ≈ open output)                 |
| `r_bat`         | Ω    | float | 0.05    | Battery series resistance                                                   |
| `vbat_ac_amp`   | V    | float | 0.0     | Ripple amplitude (peak) on V_bat; 0 = off                                   |
| `vbat_ac_freq`  | Hz   | float | 100     | Ripple frequency                                                            |
| `vbat_ac_shape` |      | int   | 0       | 0 = sine (inverter), 1 = full-wave \|sin\| (rectifier load), 2 = spiky pulse |
| `c_in`          | F    | float | 470e-6  | Input capacitance                                                           |
| `c_out`         | F    | float | 470e-6  | Output capacitance                                                          |
| `ring_c`        | F    | float | 0       | Buck switch-node capacitance; 0 = off. Rings with the coil at f = 1/(2π·√(L0·C)) while both switches are off, see [Switch-node ring](#switch-node-ring) |
| `ring_q`        |      | float | 20      | Quality factor of that ring (amplitude decays as e^(−πf·t/Q))               |
| `adc_freq`      | Hz   | int   | 3000    | Sampling tick (falls back to `board.conf::adc_fake_freq`, then 3000)        |
| `noise_vin`     | V    | float | 0       | Gaussian ADC noise σ on Vin; 0 = deterministic                              |
| `noise_vout`    | V    | float | 0       | Gaussian ADC noise σ on Vout                                                |
| `noise_iout`    | A    | float | 0       | Gaussian ADC noise σ on Iout                                                |
| `noise_ntc`     |      | float | 0       | Gaussian ADC noise σ on the NTC channel                                     |
| `ntc_v`         |      | float | 0.9     | Constant NTC channel value (25 °C equivalent)                               |

## Switch-node ring

With `ring_c > 0` the buck plant models the switch node while both switches are off. The node
capacitance `C` rings with the coil about `Vout`, damped by a conductance `1/(Q·Z0)` from the node
to the output (`Z0 = √(L0/C)`), and the body diodes clamp the node at `−Vf` and `Vin + Vf`. While
clamped, the coil current ramps back to zero at the clamp voltage and the ring restarts from rest;
charge through the high-side body diode returns to the input. The off phase starts from the node
voltage the last switch left: 0 V after a low-side phase, `Vin` without one. The plant solves each
ring segment in closed form, so the cost does not depend on the ring frequency.

Both switch edges are hard: the high side charges the node to `Vin` from the input at turn-on
(loss `½C(Vin − Vnode)²`), and the low side discharges it to 0 V at its turn-on (loss `½C·Vin²`). A
real dead time lets the coil carry most of the second part to the output instead, about `C·Vin·fsw`
of output current (15 mA at 78 V, 5 nF, 39 kHz), see [Coil inductance, remaining
biases](../../lab/coil-inductance.md#remaining-biases).

Fitted values on two field boards: `C` = 4.9 nF, `Q` = 25 at 80 µH (254 kHz) and 4.7 nF, `Q` = 15
at 51 µH (326 kHz) from the `Iout` ripple of DCM sweeps; 5 nF, `Q` = 40 at 80 µH from the
late-side tails of rect-cal sweeps. The ring shapes two measurements:

- the `Iout` ripple versus duty in DCM, which `measure-coil` fits out, see [Switch-node ringing in
  DCM](../../lab/coil-inductance.md#switch-node-ringing-in-dcm);
- the non-monotonic late side of a rect-cal sweep, where a reverse current at low-side turn-off
  lifts the node to `Vin`, see *Switch-node ringing* in [Diode emulation,
  self-calibration](../../internals/diode-emulation.md#self-calibration-rect-cal).

The console command `vconv set ring_c <F>` / `vconv set ring_q <Q>` changes them at runtime.
