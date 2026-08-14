---
name: pd-control-loop-gain-anatomy
description: Where the real MPPT PD loop gain lives (Kp is only half of it), the kCtrlSlew constants, the hidden x10 in updateCV, and the Kd->Td migration identity
metadata:
  type: project
---

The five PD limiters (`VinController`, `VoutController`, `IinController`, `IoutCurrentController`,
`powerController` in `src/mppt.h`) do **not** carry the whole loop gain. The controller output `cv`
is dimensionless (all five use `normalize=true`, so `e` is a relative error) and gets turned into a
**duty slew rate** by a per-path constant in `src/mppt.cpp`:

- `kCtrlSlewLimit = 25*2/2000 = 0.025 s^-1` — limiter path, `update()`
- `kCtrlSlewCV = 25*2/10000 = 0.005 s^-1` — CV path, `updateCV()`

`fp = cv * kCtrlSlew * pwmCtrlMax * dt` → units: [-]·[1/s]·[counts]·[s] = counts. Checks out.

**Trap:** `updateCV()`'s "low-duty gate" is not only a slow-down. The normal branch (on-time above
2500 ns) multiplies `fp` by **10**, so the effective CV slew is `0.05 s^-1` — twice the limiter
path, not one fifth of it. The low branches are `x0.01` and `x0.04`. Reading `kCtrlSlewCV` alone
understates the running gain by 10x.

Effective integral gain on normalized duty = `Kp * kCtrlSlew`. E.g. Vout: `1500 * 0.05 = 75/s`, so a
1 % Vout error commands ~75 % duty per second.

**Vin has negative Kp/Kd by design** (`-100 / -200`). It is a *lower*-bound limiter: `e = 1 -
Vin/Vin_min`, so `Vin < Vin_min` gives `e > 0` and `Kp*e < 0` = "reduce duty". The other four are
upper-bound limiters with positive gains. Don't "fix" the minus signs.

**Kd -> Td migration identity: `Td = (Kd/Kp) * Ts`.** Legacy `Kd` multiplies the raw per-sample
difference `de`, so its contribution to *duty* scales with the loop period; `Kp*Td*de/dt` cancels
against the caller's `*dt` and is sample-rate invariant. Verified bit-exact against legacy at `Ts`
for all five controllers, including the negative-Kp Vin unit (Kp cancels out of the identity, so
the sign carries through). `Kd/Kp` is positive for all five (2, 8, 2, 2, 0.25).

`Ts` = one Vout sample period — `loopRTNewData` gates `mppt.update()` on `sensors.Vout->numSamples`,
so the control period follows `sensor.conf::esp32adc1_avg` / `_sr`. Changing those silently retunes
every legacy `Kd`. See [[pd-control-td-and-dt-traps]].
