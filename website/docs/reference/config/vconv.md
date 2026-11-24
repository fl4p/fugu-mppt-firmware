---
title: vconv.conf
sidebar_position: 18
---

# vconv.conf

`vconv.conf` configures the virtual converter plant ([Simulation](../../development/simulation.md)), which replaces the power stage and ADC with a simulated PV source, battery, and passives. It applies to `CONFIG_FUGU_WITH_VCONV` builds with `sensor.conf` `adc = vconv`. `src/adc/sensor_setup.cpp` and `src/adc/vconv.h` read it. For an example, see [`config/lab/vconv_mock`](../../../../config/lab/vconv_mock/conf/vconv.conf).

The simulation takes the inductor from `coil.conf::L0` (50 µH when unset) and the topology from `converter.conf::topo` (`buck`/`boost`). This file doesn't set either.

In these builds the sensor map comes from `sensor.vconv.conf` when that file exists, else from `sensor.conf`, so a real board's `sensor.conf` can stay in place. The firmware does not drive the `board.conf` backflow switch pins; `bflow_model` simulates the switch instead.

The file accepts the following keys.

| key             | unit | type  | default | description                                                                 |
|-----------------|------|-------|---------|-----------------------------------------------------------------------------|
| `pv_isc`        | A    | float | 8.0     | PV short-circuit current                                                    |
| `pv_voc`        | V    | float | 40.0    | PV open-circuit voltage                                                     |
| `pv_k`          |      | float | 0.8     | V_mpp / Voc                                                                 |
| `v_bat`         | V    | float | 28.0    | Battery sink voltage (`v_bat=0`, `r_bat=1e9` ≈ open output)                 |
| `r_bat`         | Ω    | float | 0.05    | Battery series resistance (floor 1 µΩ)                                      |
| `vbat_ac_amp`   | V    | float | 0.0     | Ripple amplitude (peak) on V_bat; 0 = off                                   |
| `vbat_ac_freq`  | Hz   | float | 100     | Ripple frequency                                                            |
| `vbat_ac_shape` |      | int   | 0       | 0 = sine (inverter), 1 = full-wave \|sin\| (rectifier load), 2 = spiky pulse; other values = sine |
| `plant_c_in`    | F    | float | 470e-6  | Input capacitance of the plant; the real board's is `converter.conf::c_in`                                                           |
| `plant_c_out`   | F    | float | 470e-6  | Output capacitance                                                          |
| `ring_c`        | F    | float | 0       | Buck switch-node capacitance; 0 = off. Rings with the coil at f = 1/(2π·√(L0·C)) while both switches are off, see [Switch-node ring](#switch-node-ring) |
| `ring_q`        |      | float | 20      | Quality factor of that ring (amplitude decays as e^(−πf·t/Q)), minimum 0.5  |
| `ring_v`        | V    | float | 0       | Input voltage at which `ring_c` applies; > 0 makes the node capacitance scale as (`ring_v`/Vin)^`ring_cexp`, 0 = constant |
| `ring_cexp`     |      | float | 0.6     | Exponent of that scaling (MOSFET Coss falls with voltage)                   |
| `ring_ct`       |      | float | 1       | Share of the node capacitance that the hard switch edges charge and discharge, see [Switch losses and timing](#switch-losses-and-timing) |
| `diode_vf`      | V    | float | 0       | Body-diode forward drop on both clamps, in the dead time and on the reverse-current return to the input |
| `t_dead`        | s    | float | 0       | Dead time before each rising gate edge (HS off → LS on, LS off → HS on); the coil freewheels through a body diode |
| `t_hs_ext`      | s    | float | 0       | High-side on-time extension (turn-off minus turn-on delay)                  |
| `r_hs`          | Ω    | float | 0       | Resistance in series with the coil while the high side conducts (Rds(on), coil DCR, input-cap ESR share) |
| `r_ls`          | Ω    | float | 0       | Resistance in series with the coil while the low side conducts             |
| `t_sw`          | s    | float | 0       | High-side turn-off current fall time; the loss is paid from the input       |
| `pwm_clk`       | Hz   | int   | 0       | Gate-driver timing the PWM shim emulates, read at boot: 0 = LEDC (80 MHz, power-of-two period: 2047 counts at 39 kHz), else the MCPWM timer clock (160000000: the MCPWM period of 4103 counts at 39 kHz; the plant runs at the realized frequency in whole Hz, 38995). `rect_offset_ns` and `boot_refresh_ns` are converted at this tick rate |
| `bflow_model`   |      | int   | 0       | 1 = backflow switch between the output node and the battery: on, it conducts both ways; off, it is a diode (0.7 V) toward the battery. Vout and Iout then read the battery side (terminal voltage, battery current). The firmware's backflow switch drives it (`selftest bflow`); `vconv set bflow_model` at runtime |
| `bflow_stuck`   |      | int   | 0       | Fault on that switch: 0 none, 1 stuck open, 2 stuck closed (`bflow_model` only) |
| `i_unidir`      |      | int   | 0       | 1 = the Iout channel clips at 0, like a one-direction current amplifier: `selftest bflow` runs its toggle variant and `hs_clamp` is ignored. `vconv set i_unidir` at runtime |
| `hs_short`      |      | int   | 0       | 1 = buck high side conducts permanently; with the low side on as well, the input capacitor is shorted to ground. 2 = the same, and the low side never turns on. For the [high-side short check](../../internals/protection.md#high-side-short-check) |
| `adc_freq`      | Hz   | int   | 3000    | Sampling tick (falls back to `board.conf::adc_freq`, then 3000)        |
| `noise_vin`     | V    | float | 0       | Gaussian ADC noise σ on Vin; 0 = deterministic                              |
| `noise_vout`    | V    | float | 0       | Gaussian ADC noise σ on Vout                                                |
| `noise_iout`    | A    | float | 0       | Gaussian ADC noise σ on Iout                                                |
| `noise_iin`     | A    | float | 0       | Gaussian ADC noise σ on Iin (channel 3)                                     |
| `noise_ntc`     |      | float | 0       | Gaussian ADC noise σ on the NTC channel                                     |
| `ntc_v`         | V    | float | 0.9     | Constant NTC channel voltage; 0.9 V reads about 1 °C, 1.65 V is 25 °C; `vconv set ntc_v` changes it at runtime |
| `sun_day_s`     | s    | float | 0       | Length of the simulated day; 0 = off (static PV), minimum 1. See [Sun and clouds](#sun-and-clouds) |
| `sun_night_s`   | s    | float | day/4   | Length of the night after each day; a negative value = `sun_day_s`/4        |
| `sun_peak`      |      | float | 1.0     | Clear-sky irradiance at noon, fraction of the `pv_isc`/`pv_voc` conditions (clamped to 0..1) |
| `cloud_rate`    | 1/min | float | 0      | Mean cloud starts per minute of clear sky; 0 = no clouds                    |
| `cloud_depth`   |      | float | 0.7     | Maximum irradiance dip of a cloud (clamped to 0..1)                         |
| `cloud_len_s`   | s    | float | 20      | Mean cloud duration (clamped to 1..600)                                     |
| `cloud_edge_s`  | s    | float | 3       | Time constant of the cloud edges                                            |
| `sun_seed`      |      | int   | 1       | Cloud random seed; 0 = 1                                                    |

## Renamed keys

Four keys were renamed so they read as what they configure and stop colliding with
[converter.conf](converter.md)'s keys:

| old key   | new key       | why                                                              |
|-----------|---------------|------------------------------------------------------------------|
| `isc`     | `pv_isc`      | the `pv_` family `pv_k` already used, matching converter.conf      |
| `voc`     | `pv_voc`      | as above                                                          |
| `c_in`    | `plant_c_in`  | the plant's capacitor, not the real board's `converter.conf::c_in` |
| `c_out`   | `plant_c_out` | the plant's capacitor                                             |

An old key without its new one stops setup and names the replacement. `vconv set` takes the new
names too.

## Sun and clouds

With `sun_day_s` > 0 the plant scales the PV curve by an irradiance G between 0 and 1, so the tracker
sees sunrise, noon, sunset, night and passing clouds. `pv_isc`, `pv_voc` and `pv_k` stay the full-sun
values.

- **Clear sky.** A day of `sun_day_s` seconds followed by a night of `sun_night_s` seconds, repeating.
  During the day G_clear = `sun_peak` · sin(π·t/`sun_day_s`)^1.3, a flattened half-sine; at night
  G_clear = 0.
- **Clouds.** In clear sky a cloud starts with probability `cloud_rate`/60 per second. Each cloud
  dips the irradiance by a depth drawn uniformly from 0.3·`cloud_depth` to `cloud_depth` and lasts
  an exponentially distributed time with mean `cloud_len_s`. The cloud factor follows the dip
  through a first-order low-pass with time constant `cloud_edge_s`, so edges are soft. A seeded
  random generator makes a run repeatable for one `sun_seed`.
- **PV curve.** G = G_clear · cloud factor sets Isc = `pv_isc` · G and
  Voc = `pv_voc` · (1 + 0.04 · ln G): a string loses about 4 % of its open-circuit voltage per e-fold
  of irradiance. Below G = 0.001 Voc falls linearly toward zero (floors 0.1 mA and 0.5 V), so the
  night looks like a dead panel and the input undervoltage protection acts as at dusk.

The model updates 20 times per simulated second and rebuilds the PV curve only when G has changed
by more than 0.2 %. Over a long run the cloud factor averages about 1 − 0.65 · `cloud_depth` · f,
where f = r·L / (1 + r·L) is the cloudy share of the time for a start rate r = `cloud_rate`/60 and
mean duration L = `cloud_len_s`.

A 10-minute day with clouds:

```
sun_day_s=600
cloud_rate=4
cloud_depth=0.7
cloud_len_s=15
```

The sun and cloud keys change at runtime with `vconv set <key> <value>`. Switching `sun_day_s` from 0 to a
positive value starts the day at sunrise, setting `sun_seed` reseeds the clouds and restarts the
day, and `sun_day_s` 0 restores the full-sun curve. `vconv sun` prints G, G_clear, the cloud factor,
the time of day and the effective Isc and Voc.

## Runtime changes

The firmware reads `vconv.conf` once at boot. The console changes part of the plant in RAM:

- `vconv set <key> <value>`: `plant_c_in`, `plant_c_out`, `r_bat`, `vbat_ac_amp`, `vbat_ac_freq`, `vbat_ac_shape`,
  `diode_vf`, `ring_c`, `ring_q`, `bflow_model`, `bflow_stuck`, `hs_short`, `i_unidir`, `ntc_v`, and the
  sun and cloud keys. `vconv set l <H>` overrides the inductor.
- `vconv pv <isc> <voc> [k]` sets the PV curve, `vconv bat <V|open|short>` the battery.

The other keys (`ring_v`, `ring_cexp`, `ring_ct`, `t_dead`, `t_hs_ext`, `r_hs`, `r_ls`, `t_sw`,
`pwm_clk`, `adc_freq`, `noise_*`) need a reboot. See [console](../console.md).

## Switch-node ring

With `ring_c > 0` the buck plant models the switch node while both switches are off ([DCM coil ringing](../../internals/dcm-ringing.mdx); measured values in [Switch-node capacitance](../../lab/switch-node-capacitance.md)). The node
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
  lifts the node to `Vin`, see [switch-node ringing in the rectifier timing
  calibration](../../internals/rect-cal.md#switch-node-ringing).

The console command `vconv set ring_c <F>` / `vconv set ring_q <Q>` changes them at runtime.

## Switch losses and timing

All keys in this section default to 0 (ideal, lossless switches) and apply to the buck plant only.

- **Dead time** (`t_dead`): the gate driver delays each rising edge. After the high side turns off,
  the coil current freewheels through the low-side body diode (node at `−diode_vf`) until the low
  side turns on; the low-side window shrinks by `t_dead` and its end does not move. A low-side
  pulse shorter than the dead time is swallowed. When the idle time before the next high-side pulse
  is shorter than `t_dead`, that pulse starts late, as in synchronous CCM.
- **Node energy**: with `ring_c > 0` the high side charges the node from the input at turn-on,
  `ring_ct·C·(Vin − Vnode)`, for every pulse whatever the node voltage it starts from. At turn-off a
  positive coil current swings the node down during the dead time along the undamped resonance of
  the coil with `ring_ct·C`, so a small current first gains current while the node is still above
  `Vout`. If the node reaches `−diode_vf` the body diode clamps it; if not, the rest of the node
  energy is lost when the low side turns on. `ring_ct` < 1 accounts for a rise transit that is
  faster than the free ring (nonlinear Coss).
- **Turn-off loss** (`t_sw`): the high-side channel current falls linearly in `t_sw` while the node
  capacitance takes the rest. Small currents turn off almost without loss
  (`I²·t_sw²/(24·ring_ct·C)`, C the node capacitance at that Vin), large ones approach
  `½·Vin·I·t_sw`. A high-side pulse that fills the whole period has no turn-off and no such loss.
- **Conduction** (`r_hs`, `r_ls`): a voltage drop `R·i` against the coil while that switch conducts,
  so the loss lowers the delivered current at a given duty. The loss is `R·∫i²`: the part from the
  current ripple, `R·t·ΔI²/12`, is paid from the input.
- **Diode drop** (`diode_vf`): both ring clamps, the dead-time freewheel, and the reverse current
  that returns to the input through the high-side body diode (node at `Vin + diode_vf`).

`r_hs`, `r_ls` and `t_sw` are effective values: they also carry losses the plant has no term for,
such as avalanche loss at the switch-node overshoot. Parameter sets for two field boards, fitted to
an LTspice half-bridge bench, are in `test/host-stub/plant_boards.h`;
`test/host-stub/vconv-spice-check.sh` compares the plant with that bench.
