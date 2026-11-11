*this document is an LLM generated placeholder*

# Host simulator

Runs the real control loop natively against the vconv plant (`src/sim/vconv.cpp`) on a deterministic
sim clock. Compiled unchanged from `src/`: `MpptController` (`mppt.cpp`/`mppt.h`: `protect()`,
`protectLf()`, `update()`, sweep, PD limiters, `shutdownDcdc`), `Tracker`, `BatteryCharger`,
`ADC_Sampler` + `Sensor` filters (notch, median, EWM, offset calibration), `SynchronousConverter`
(diode emulation, `dcmCurrent`) with the `MOCK` gate driver at the MCPWM resolution (4103 counts).

```sh
test/host-stub/sim/build.sh /tmp/fugu-sim                       # firmware from this checkout
SRC=/other/worktree/src test/host-stub/sim/build.sh /tmp/sim-x   # firmware from another tree
/tmp/fugu-sim isc=0.1 voc=66 pv_k=0.7 vbat=25.6 noise_iout=0.02 hours=4 v=1
```

Arguments (`key=value`): `isc voc pv_k` (PV curve), `isc_end` (linear Isc ramp over the run),
`vbat r_bat` (battery), `L` (coil, H; also coil.conf L0), `noise_iout noise_vin noise_vout`
(Gaussian sigma per ADC sample), `iout_off` (raw Iout offset, removed by the sweep-start
calibration), `sps` (ADC rate, default 500), `hours`, `seed`, `v` (0 summary, 1 warnings + sweep and
backoff lines, 2 everything incl. a 3 s status line). `<file>.<key>=<value>` overrides a conf key,
e.g. `tracker.sweep_speed=8` or `sensor.filt_len=10`.

Output: firmware log lines with sim timestamps, then a summary: time converting, energy, mean Vin/P
while tracking, a Vin histogram, every `backoff [reason]` (with `mid-sweep`), selected events and the
highD-bflowOpen trip rate.

## Simplified or stubbed

- Board profile is fry-like (`config/fmetal` values, L0 from `L`), built in memory in `sim.cpp`;
  no BMS/MQTT, so the charger runs on `Vbat_fallback` and the BMS start wait is skipped.
- Sensor wiring re-implements the vconv branch of `setupSensors()` (identity transforms, Iin virtual,
  NTC fixed at 25 C, die at 35 C). One snapshot of all channels per tick (like `ADC_VConv`), no
  round-robin; the plant is stepped once per sample.
- RT/LF glue is a copy of `main.cpp` (`loopRT`, `loopRTNewData`, `stopAndBackoff`, `lfControl`
  charger part, `lfStuckWatchdog`); PSU/manual paths, ADC-stall and loop-latency watchdogs, telemetry,
  LCD and rect-cal persistence are left out. `esp_restart()` rebuilds the firmware objects in place.
- The gate driver is `PWM_Mock`: MCPWM dead time, gate delays and the backflow switch are not in the
  plant (an open backflow switch does not block forward current; vconv models the LS body diode).
- Core-0 tasks (`enqueue_task`) run right after each RT pass.
