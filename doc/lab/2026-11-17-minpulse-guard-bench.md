*this document is an LLM generated placeholder*

# Bench procedure: LS min pulse ratio limit (minpulse-guard), not yet run

Prepared 2026-11-17 for branch `minpulse-guard` (simulator-validated only). Nothing here has been run.
Background: `website/docs/internals/diode-emulation.md#ls-minimum-pulse-near-unity-ratio` and
`doc/research/2026-11-17-minpulse-guard.md`.

## What is tested

1. Start block: a buck does not start in DCM above the start ratio (`START blocked: minpulse-M`).
2. `V_in` floor: in DCM the Vin controller holds `V_in ≥ V_out / M_start` (limiter index 0) and the converter keeps
   running with positive output current while the input is lowered toward `V_out`.
3. Stop: in manual duty (no floor) the converter stops with `backoff 5s [minpulse-M]` when the read ratio reaches
   `M_stop`.

Not tested here: the stop on reverse current between the start and the stop ratio (needs a source on the output that
can be pumped, see "Not covered").

## Board, wiring, locks

- Board: **fbuck** (`config/lab/buck_bench`: diode emulation, `pwm_deadtime_ns=200`, `reverse_current_paranoia=0`).
  fboost stays off. The power loop must be opened for this test: fbuck's input is fed by the Korad, fbuck's output
  goes to a load resistor only. **Confirm with Fab how the rig is to be re-wired before starting**; the procedure
  assumes this wiring and nothing else on fbuck's output (no battery, no PSU, no fboost input).

  ```
  Korad (CV, current-limited) --> fbuck HV input
  fbuck LV output --> R_L (220 ohm, >= 5 W)        nothing else on the output
  ```

  R_L 220 ohm puts 91 mA / 1.8 W at 20 V: DCM over the whole test. A different resistor is fine if the output current
  stays below about 0.2 A at 20 V (DCM).
- Locks (all three, released at the end):

  ```bash
  L=~/.claude/skills/lock/lock.sh
  "$L" list
  "$L" acquire fugu-rig "minpulse-guard bench"
  "$L" acquire korad "minpulse-guard bench"
  "$L" acquire fbuck "minpulse-guard bench"
  # + "$L" acquire scope "..." if the switch node / bootstrap is probed
  ```

- Korad: CV as given per step, **CC 0.5 A**, OVP (if set) 30 V. Why this is safe: the Korad cannot sink current and
  the output has no source, so the only energy the min pulse can pump back into the input is what the output
  capacitors and R_L hold (well under 1 J at 20 V); it can lift `V_in` by a fraction of a volt, far from `hv_max` 85 V.
  The CC limit caps the input power at 0.5 A x 25 V.
- Firmware: fbuck on the `minpulse-guard` build (bench unit, flash pre-authorised). Record `bootinfo` (version, slot).

## Console

Send commands as one script per step (one connection), e.g. over serial or BLE:

```bash
.venv/bin/python3 etc/fugu_console.py --ble --name fbuck --stdin <<'EOF'
hostname
status
EOF
```

Before anything that switches: `hostname` must answer `fbuck`.

## Pre-flight (converter off)

```
hostname
bootinfo
dc 0
sync
dt
pwm-dump
get-config board.conf boot_refresh_ns
get-config converter.conf
sensor
log mppt info
```

- `sync` must report diode emulation (not `forced pwm armed`). `get-config converter.conf` without `forced_pwm=1`.
- From the boot log line `drv=... pwmMax=P minLS=m` (or `pwm-dump` period ticks and `dt`): `T = P + dt_LH ticks`.
  Compute `M_stop = (1 − (m + T/32)/T)·0.99/1.01` and `M_start = M_stop − 0.02`. Expected with 2 us refresh and
  200 ns dead time at 39 kHz: `M_stop ≈ 0.866` (`V_in` < 1.155 `V_out`), `M_start ≈ 0.846` (`V_in` > 1.182 `V_out`).
  Write the computed values down; the pass criteria use them.
- Check that the idle `V_in`/`V_out` readings agree with a DMM within 1 % (both). The limit assumes 1 % per reading.

## Step 1: start block (PSU mode)

Korad 22.0 V. Then:

```
psu 20
```

Wait 10 s, then `status` and read the log. Expected: converter stays off, log `START blocked: minpulse-M` (20 / 22 =
0.91 > `M_start`). In PSU mode the setpoint counts, not only the present `V_out`.

Korad 25.0 V (20 / 25 = 0.80 < `M_start`). Expected: converter starts, `V_out` settles at 20.0 V, `I_out` ≈ 0.09 A
(positive), `pwm_dcm` / DCM in the status line.

## Step 2: V_in floor (PSU mode, input lowered toward V_out)

With `psu 20` running, lower the Korad in steps, holding each 20 s, then back up:

`25.0, 24.5, 24.0, 23.75, 23.5, 23.25, 23.0, 22.5, 22.0, 22.5, 23.0, 23.5, 24.0, 25.0` V

At each step, after 15 s: `status`, `sensor`, and the latest `V=` line (Vin, Vout, Iout, duty, limiter index).

Expected:
- `V_in` ≥ `V_out / M_start` (Korad ≥ ~23.6 V): `V_out` = 20.0 V, Vout limiter (index 1).
- Below that: limiter index 0 (`Vin`), `V_out ≈ M_start·V_in` (e.g. 22.0 V in → ~18.6 V out), still switching, `I_out`
  positive, no `minpulse-M` stop.
- On the way up `V_out` returns to 20.0 V.

## Step 3: stop (manual duty, no floor)

```
psu off
dc 0
```

Korad 23.0 V. Manual duty has no `V_in` floor, so the stop is the only protection. Start low and step up:

```
dc 1500
```

(1500 of ~4100 counts; read `V_out`, it depends on R_L in DCM.) Then raise `dc` in steps of 20 counts, one step per
command script with `sensor` after 3 s, until the converter stops. Use `pwm-dump` to confirm each `dc` landed.

Expected: the converter stops with `... at the LS min pulse ratio limit, shutdown` and `backoff 5s [minpulse-M]` when
`V_out/V_in` (averaged readings) reaches `M_stop` (≈ 19.9 V at 23.0 V in). After the 5 s backoff the manual duty
restarts by itself only once `V_out` has decayed below `M_start·V_in` (≈ 19.5 V at 23.0 V): R_L drains the output
capacitors, so expect a restart, a ramp back to the commanded duty and another stop, a few seconds apart, until
`dc 0`. Each restart must happen below `M_start·V_in` (read `V_out` in the `sensor` line just before the restart);
a restart above it is a failure. Send `dc 0` within 30 s of the first stop.

Repeat once at Korad 26.0 V (stop at ≈ 22.5 V out).

## What to record

- Per step: Korad setpoint and its current readout, `V_in`, `V_out`, `I_out` (firmware), a DMM on `V_out`, duty,
  limiter index, mode, log lines with timestamps.
- Step 3: the last `V_out/V_in` before the stop and the computed `M_stop`; the time between the `dc` step that crossed
  and the stop line.
- If the scope is available (lock `scope`): switch node and HS gate in step 2 at the floor (Korad 22.0 V) and in
  step 3 just below the stop. Check that the HS turns on in every period (no burst pattern) and, with a differential
  probe across the bootstrap capacitor (VB−VS), that its minimum stays above the gate driver's UVLO threshold
  (datasheet value of the fitted driver). The min pulse is unchanged by this firmware, so a burst pattern here is a
  pre-existing bootstrap issue, to be noted, not a guard failure.

## Pass criteria

- Step 1: blocked at 22.0 V, starts at 25.0 V.
- Step 2: no stop; limiter index 0 below the floor; `V_out/V_in` within ±0.01 of `M_start` there; `I_out` never below
  −0.05 A; `V_in` never more than 0.2 V above the Korad setpoint (no pumping into the input).
- Step 3: stop line with `minpulse-M` at a read ratio within ±0.005 of the computed `M_stop`, within 0.5 s of the
  crossing step; no `Reverse I` or `revI-ewm` line before it; no restart while `V_out ≥ M_start·V_in`.

## Abort criteria (then `dc 0`, then Korad output off)

- `V_in` more than 1 V above the Korad setpoint, or the Korad goes into CC unexpectedly.
- `I_out` below −0.2 A for more than 1 s, or any `Reverse I` / `revI-ewm` line.
- `V_out` above 21 V in PSU mode with setpoint 20 V.
- Any ADC error (`ADC stall`, `Never got a sample`, `ina22x: ... timeout`, `Loop latency high`).
- Switch node shows the HS missing pulses (bootstrap starved), or a part gets hot.
- The console stops answering: Korad output off.

## Shutdown

```
dc 0
psu off
```

Korad output off, restore the rig wiring as it was, `"$L" release fbuck`, `korad`, `fugu-rig` (and `scope`).

## Not covered

- The stop on reverse current between `M_start` and `M_stop` and the pump itself need a source on fbuck's output
  that the min pulse can drain (battery or a second supply with a sink path). That setup can carry amps of reverse
  current on the old firmware; it is covered by the simulator (`test/host-stub/minpulse-sim.sh`) only.
- Real reading errors: the limit assumes 1 % per voltage reading; the DMM comparison in pre-flight is the only check.
