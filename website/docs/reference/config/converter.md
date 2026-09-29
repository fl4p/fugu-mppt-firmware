---
title: converter.conf
sidebar_position: 5
---

# converter.conf

Topology.

| key          | unit | type  | default | description                                                |
|--------------|------|-------|---------|------------------------------------------------------------|
| `topo`       |      | enum  | —       | Converter topology: `buck` or `boost`                      |
| `mode`       |      | enum  | mppt    | Operating mode: `mppt` (default), `psu` (constant-voltage supply; see `psu` console command) or `pv` (solar-array-simulator: output follows a PV curve; see `pv` console command) |
| `psu_vout`   | V    | float | —       | PSU mode output voltage setpoint (used when `mode=psu`; range-checked against `vout_max`) |
| `pv_isc`     | A    | float | —       | PV-sim short-circuit current (`mode=pv`; also caps the Iout limiter at 1.1×) |
| `pv_voc`     | V    | float | —       | PV-sim open-circuit voltage (range-checked against `vout_max`; carries the OV threshold and boost feasibility) |
| `pv_k`       |      | float | 0.8     | PV-sim Vmp/Voc curve factor, valid range [0.5, 0.95] |
| `pv_slew`    | V/s  | float | 200     | PV-sim setpoint slew limit (min 10) |
| `pv_iout_span` |    | float | 16      | PV-sim EWMA span of the Iout feedback for the curve (local, separate from `iout_filt_len`) |
| `forced_pwm` |      | bool  | 0       | Force CCM PWM even at light loads (see notes below)        |
| `fpwm_gate`  |      | bool  | 1       | Hold `forced_pwm` off until the duty reaches the voltage ratio (bring-up gate, see notes below) |
| `fpwm_gate_margin` |  | float | 0.01    | Duty margin the gate requires on top of the measured ratio |
| `fpwm_gate_hold` |    | long  | 192     | Samples the gate must see a passing duty before it engages (the ramp is held meanwhile) |
| `pwm_driver` |      | enum  | ledc    | Gate driver: `ledc` or `mcpwm`. Only consulted when the firmware compiles in both (`CONFIG_FUGU_WITH_LEDC` and `CONFIG_FUGU_WITH_MCPWM`); with one compiled it is forced to that one |
| `vout_max`   | V    | float | —       | Legacy output voltage limit (real one is in `limits.conf`) |
| `sync_role`  |      | enum  | none    | Wired MCPWM clock sync (`WITH_WSYNC`): `none`, `leader` (emit TEZ pulse on `board.conf::pwm_sync_pin`) or `follower` (phase-reload timer from that pin). See [Wired Clock Sync](../../internals/sync/wired-sync.md) |
| `sync_phase_deg` | ° | float | 0      | Leader only: pulse offset from its TEZ as an angle (= follower period-start shift; `180` for interleave). Ignored on a follower (reload fixed at 0) |
| `sync_phase_ns` | ns | float | 0      | Leader only: additive trim on `sync_phase_deg`, for wire + receiver propagation delay (a time, so it does not scale with `pwm_freq`) |
| `ctrl_<n>_kp`  |      | float | see below | Proportional gain of control unit `<n>` ∈ {`vin`, `vout`, `iin`, `iout`, `power`} |
| `ctrl_<n>_kd`  |      | float | see below | Derivative gain of unit `<n>`, applied to the **per-sample** error difference |
| `ctrl_<n>_td`  | s    | float | (unset)   | Derivative *time* of unit `<n>`. When set it replaces `_kd` and makes the D component sample-rate invariant |

## Control-loop gains

The five PD units in `src/mppt.h` limit duty against Vin (under-voltage), Vout (over-voltage / CV),
Iin, Iout and power; the smallest response wins each tick. Compiled-in defaults:

| unit    | `_kp` | `_kd` | error is relative to |
|---------|-------|-------|----------------------|
| `vin`   | -100  | -200  | `limits.conf::vin_min` |
| `vout`  | 1500  | 12000 | `charger.Vout_max()` |
| `iin`   | 100   | 200   | `limits.conf::iin_max` |
| `iout`  | 200   | 400   | derived `Iout_max` |
| `power` | 20    | 5     | derived power limit |

These are **not** the whole loop gain — the controller output is scaled into a duty slew rate by a
per-path constant in `mppt.cpp`:

| path | constant | effective scale |
|------|----------|-----------------|
| limiter (`update()`)  | `kCtrlSlewLimit` = 0.025 | 0.025 |

The limiter path is the only duty-slew path: the five PD controllers (Vin, Vout, Iin, Iout, power)
produce a control value each tick, the minimum wins, and `kCtrlSlewLimit` scales it into a duty
step. In PSU mode the same chain runs — when no limiter binds, the Vout controller's output drives
duty toward the setpoint (CV); when a current or power limit binds, it folds back (CC). `mode=pv`
runs the identical chain, but the Vout setpoint is recomputed each tick from the PV curve
V=f(Iout), clamped to [Vin+0.5, min(`pv_voc`, `vout_max`)] and slew-limited by `pv_slew`; the OV
threshold and feasibility checks stay pinned at `pv_voc`, not the moving setpoint.

**`_kd` vs `_td`.** `_kd` multiplies the raw difference between consecutive samples, so its
contribution to duty scales with the loop period: the same `_kd` is a *different* derivative gain
at a different sample rate. Change `sensor.conf::esp32adc1_avg` or `esp32adc1_sr` and every `_kd`
silently retunes. `_td` expresses the same thing as a time (`de/dt`), which is invariant. To port a
board without changing its behaviour, set `_td = (_kd / _kp) * Ts` where `Ts` is that board's
current control period (1 / the Vout sensor's sample rate — the loop runs once per Vout sample).
Setting `_td` makes `_kd` unused. Both are logged at boot (`ctrl vout: Kp=… Td=…`).

Gains live here rather than in `charger.conf` because they are a property of the plant — L, output
capacitance, `pwm_freq`, `pwmMax`, buck-vs-boost — not of the battery.


## Notes & examples

### `forced_pwm`

With the default value `0`, the converter runs in DCM under light load. The controller uses the coil
inductance (`coil.conf::L0`) plus the input and output voltages to decide whether to operate in DCM
or CCM.

Set to `1` to disable diode emulation and always run in CCM — *forced PWM* mode, with these
characteristics:

* less output noise, because the inductor never free-wheels
  (see [wave forms](https://www.nisshinbo-microdevices.co.jp/en/faq/083.html))
* much better output regulation during load changes, useful for a PSU
* lower efficiency: reverse coil current shuttles energy back and forth between output and input
* a buck converter in forced PWM can easily boost voltage from output back to input

Forced PWM is useful if you want to use the converter as power supply.

### `fpwm_gate`, `fpwm_gate_margin`

The DC coil current in forced PWM is `(D·Vin − Vout) / R_loop`, and `R_loop` is only the coil DCR
plus the two `Rds(on)` — milliohms. Zero current sits at `D = Vout/Vin` (boost: `D = 1 − Vin/Vout`),
so while a duty ramp is still *below* that ratio the converter pumps current backwards, out of the
output and into the input. Against a stiff output — a supply, a battery, the input node of a
[power loop](../../lab/power-loop.md) — that is hundreds of amps, and a ramp from duty 0 in forced PWM is
destructive.

`fpwm_gate=1` (the default) keeps the low side diode-emulating until the duty has passed the
measured ratio. Below that duty the body diode blocks, so the ramp cannot boost; the gate engages
once the converter is conducting forward in CCM, where diode and synchronous rectification share
the same operating point and the transition is seamless.

There are two ways to deserve forced PWM, because a duty that is still *moving* and a duty that has
*settled* are different situations:

    engage, duty moving   at  D ≥ D₀ + err + fpwm_gate_margin     ("forward")
    engage, duty settled  at  D ≥ D₀ − err                        ("settled")
    disengage             at  D <  D₀ − err − fpwm_gate_margin
                                          err = r·kErr, kErr = 1.01/0.99 − 1 ≈ 2 %

While the duty is climbing, the only safe test is *provably forward*: a ramp that has not reached
its operating point may be climbing toward a stiff output, and engaging below `D₀` there is the
destructive case. Once the duty has stopped moving for the whole `fpwm_gate_hold`, the picture
changes — a settled converter with no load sits at `D == D₀` **by definition**, that being what zero
current means. Refusing forced PWM there would refuse it exactly where it is both harmless and
wanted: a bench converter parked at the ratio is steerable *only* in forced PWM, because diode
emulation cannot pull the output down. So a settled duty engages from `D₀ − err`, having converged
onto the ratio rather than ramping at it.

Disengage sits a margin below the converge floor so that steering by single counts does not chatter
the gate, while a ramp away from the operating point still drops it promptly. That test is also
re-run inside `pwmPerturb()` against the duty about to be committed, not only once per sample,
because the control loop may step the duty all the way to zero in one go; when it fires there it
also puts the rectifier back into diode emulation in the same call, rather than leaving one more
control interval of complementary low side behind.

**Know what the settled path costs.** A still duty is evidence that the converter has settled, not
that its output is floating — a stiff load, a manual target or plain quantisation can hold `D` below
the true `D₀` just as well. On a stiff output the gate will then engage and *stay* engaged, and the
reverse current is bounded in magnitude but **not in duration**: at 26.2 V / 70.5 V with
`R_loop ≈ 30 mΩ`, a fresh settled engagement at the converge floor allows ≈ −18 A, and an engaged
gate may sit at the disengage floor at ≈ −41 A (≈ −59 A once worst-case ratio error is included).
That is the deliberate price of being able to park a bench converter at the ratio in forced PWM,
where it is steerable; `fpwm_gate_margin` is the knob, and a closed stiff loop wants it small.

The margin is the price of the transition: the engagement current step is

    I_engage ≈ (r·kErr + fpwm_gate_margin) · Vin / R_loop      r = Vout/Vin (buck)

— note the sensor-error term, which at `Vout/Vin = 0.4` is `0.008`, nearly as large as the default
margin, so the real step is about twice what the margin alone suggests. On a 68 V bus with
`R_loop = 30 mΩ` that is roughly 40 A. Raise the margin only if reverse current at engagement is
the bigger worry than forward current.

`fpwm_gate_hold` is counted in **fresh Vin/Vout samples**, not in control-loop passes: the gate is
evaluated on every pass but only advances its counters when the sampler has actually produced new
voltage data, so a stalled sensor cannot be mistaken for a settled operating point.

While the gate is counting out `fpwm_gate_hold`, the duty ramp is **held** (upward only; a retreat
is never blocked) — in manual PWM and in the automatic control loop alike, so a gated boost running
`mode=pv` or MPPT gets the same treatment. The point of the hold is that the voltage filters can
settle: the gate compares an instantaneous duty against EWMA averages `vin_filt_len`/`vout_filt_len`
samples long, and on a moving duty that lag is a steady-state error, not noise. Sizing the hold
below the filter length defeats it.

The gate needs both side voltages above 0.1 V with `Vin > Vout` (buck) resp. `Vout > Vin` (boost);
anything else is not a conversion ratio and it refuses to engage. A light or purely capacitive load,
where `D == Vout/Vin` by definition, reaches forced PWM through the settled path once the duty stops
moving — but only after `fpwm_gate_hold`, so a converter whose duty never settles (an active
regulator hunting) will stay armed. `fpwm_gate=0` (as `config/psu_12v` does) removes the gate
entirely; then never ramp the duty to 0 in forced PWM, because a complementary low side at duty ~0
shorts the output to ground through the coil.

The console reports the state with a bare `sync` (`off`/`armed`/`arming`/`engaged`/`on (ungated)`)
together with the threshold that currently applies, and `sync forced!` engages immediately, bypassing
the gate until the next `disable()` — a shutdown, a sweep or `dc 0` makes forced PWM re-earn its
engagement.
