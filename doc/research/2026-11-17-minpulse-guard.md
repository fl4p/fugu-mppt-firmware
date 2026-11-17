*this document is an LLM generated placeholder*

# LS min pulse ratio limit near Vin = Vout (minpulse-guard), simulator study

Branch `minpulse-guard`, 2026-11-17. Simulator only, NOT tested on hardware. Bench procedure (prepared, not run):
`doc/lab/2026-11-17-minpulse-guard-bench.md`. Published description: `website/docs/internals/diode-emulation.md`
section "LS minimum pulse near unity ratio". Related: `doc/research/2026-11-17-de-margin-near-unity.md` (branch
de-margin).

## Problem

In buck DCM the LS still gets its minimum pulse `t_min = pwmRectMin` (bootstrap refresh) every period, even when
diode emulation would want 0. During that pulse the coil sees `−V_out`, so it ends the pulse with a reverse current
`i_r = V_out·excess/L` (excess = the part of the pulse after the coil current hit zero). That reverse current drains
through the HS body diode against `V_in − V_out`, which takes `δ·M/(1−M)` of time (δ = the excess).

- Below the carry-over onset `M_c = 1 − t_min/T` the drain finishes inside the period: the mean output current is
  negative but bounded, about `−V_out·t_min²/(2·L·T·(1−M))` for light HS (`t_HS < t_min·M/(2(1−M))`).
  Example flat, 51 µH, 29 V, `t_min` 2 µs, T 25.6 µs, M 0.85: −0.30 A (sim: −0.25 A).
- Above `M_c` the drain does not finish before the next HS pulse, the current carries over and grows each period by
  `V_in·T·(M − M_c)/L`. This is independent of `t_HS`. The steady state is set by resistance only,
  `≈ −V_in·(M − M_c)/R`: amps.

Pump rate (fry 76 µH, 27 V): `M_c + 0.01`: 3.8 A/ms, −1 A in 0.26 ms (about 10 periods); `M_c + 0.005`: 1.9 A/ms.
flat (51 µH) `M_c + 0.01`: 5.6 A/ms. The sampler sees `V_out` at a few hundred Hz (fry ~512 Hz, flat ~1.5 kHz), so a
guard that waits for reverse current reacts after the damage. The guard has to act on the measured ratio, before
`M_c`.

## Design

Ratio limits from config, in `SynchronousConverter` (`src/buck.h`):

```
T       = driverPwmMax + dt_LH ticks
M_stop  = wcef · (31/32 − pwmRectMin/T)      wcef = (1 − 0.01)/(1 + 0.01) = 0.9802 (1 % per voltage reading)
M_start = M_stop − 0.02
```

`wcef` covers `V_out` reading 1 % low and `V_in` 1 % high (the worst case: the read ratio is then below the true
one). `T/32` (0.031 in M) is a margin for reading errors beyond 1 %: flat's `V_out` reads 1.1–1.7 % low while
switching. The ring current at HS turn-on moves the zero crossing by only ~0.002 in M, so it is not the reason for
the margin.

| config | M_c | M_stop (Vin below) | M_start (Vin above) |
|---|---|---|---|
| mock, no dead time (2 µs, 4103 ticks) | 0.9218 | 0.8729 (true worst 0.8905) | 0.8529 |
| dead time 200 ns | 0.9149 | 0.8661 (1.155·Vout) | 0.8461 (1.182·Vout) |

Margin between the stop ratio and the onset at 27 V out: about 1 V of `V_in`.

Active only when exposed: buck, DCM (`dcmHysteresis`), not forced PWM. Boost is excluded (`minPulseExposed()`
false): in a boost the LS is the main switch and the min pulse is the HS rectifier, which has no such carry-over at
low ratio; a boost near unity is the de-margin branch's topic.

Three parts:

1. **Start block** (`startBlockReason`, reason `minpulse-M`): a buck does not start while
   `max(V_out, psu setpoint) ≥ M_start·V_in` (EWM readings). The same rule gates a manual-duty start in
   `updateManual()`, including the restart after a backoff.
2. **V_in floor** (`mppt.update()`): in DCM the Vin controller target becomes
   `max(Vin_min, V_out/M_start)`. The tracker keeps running and the floor limits it. The Vin controller is reset when
   the floor switches on or off, because the normalized PD controller's D-term kicks on a setpoint step (that kick
   ended the initial sweep with "no MPP" and a 30 s backoff in an early version, −5.6 pp harvest on wx-dusk).
3. **Stop backstop** (`protect()`, fresh `V_out` samples only): `shutdownDcdc("minpulse-M")` (5 s backoff) after 4
   consecutive fresh samples with read ratio `≥ M_stop`, or with ratio `≥ M_start` and `I_out` EWM `< −0.05 A`.
   When the converter enters DCM (exposed edge) with the ratio already `≥ M_stop`, it stops on that pass without the
   debounce.

Why not a pure stop: in wx-mp-ramp the MPP sits right at the limit for minutes and a stop-only guard cycled 41 times
per run. The floor holds the operating point at `M_start` instead; the stop only fires when the source is too stiff
for the Vin controller (wx-mp-psu: once per run).

Debounce: 4 fresh samples (fry ~8 ms). The EWM inputs (span 20, τ ≈ 10 samples) mean the read ratio lags a step by
~14 samples. A stiff-source step faster than that crosses `M_c` before the stop. The start block and the floor
cover the slow cases (dusk, dawn, ramps).

Code size: +0x340 bytes. `rtprof_map --check`: 79 entries, all in IRAM. The two `buck.h` helpers are
`always_inline` (no flash COMDAT). The only new flash symbol is the cold `protect()` lambda with the log line.
No new conf key. After the review follow-ups: still 11 % free, `rtprof_map --check` 79 entries, 0 not in IRAM.

CCM side: a buck's HS is capped at `pwmCtrlMax = pwmMax − pwmRectMin − 1`, so `D_max ≈ M_c`. A CCM point with forward
current therefore sits at `M < M_c`; DCM can be entered above `M_c` only when a stiff input falls while the
converter is in CCM (current reverses, the current EWM then flags DCM). That entry stops at once.

## Open-loop check (`test/host-stub/minpulse-test.cpp`)

The firmware class (MOCK, 4103 ticks) drives the vconv plant with stiff `V_in`/`V_out`, LS = min pulse, HS swept in
steps of 4 counts (64 for the known-bad scan). Boards fry, flat, fry-hq, flat-hq.

| point | 26.8 V out fry / flat | 13.4 V out fry / flat |
|---|---|---|
| admitted, true M = M_stop/wcef (worst reading error) | min −0.23 / −0.30 A, no drift | −0.11 / −0.13 A |
| known-bad, M_c + 0.015 | −16 / −15 A | −8 / −7.5 A |

Known-bad limit overrides (`MINPULSE_LIMIT`): 0.93 and 0.905 both fail the test (above / at the onset with the
reading error).

## Closed-loop sims (pgx93, mppt-sim)

Base = `c3ce3dc2` (main before the guard) with the new scenario file; guard = `eb6e0f9c`. 19 wx scenarios × fry,
fry-hq, flat, flat-hq × seeds 1–4, SIM_RING = board, SIM_NOISE = fry|flat. 304 paired runs, all on pgx93.
Scripts and full tables: `doc/research/minpulse-guard/` (`job.sh`, `analyze.py`, `results.md`).

New scenarios, `V_oc` track 120 s → 720 s down, hold 60 s, back up by 1380 s, T 1500 s:

- `wx-mp-ramp`: PV panel, V_oc 1.6 → 1.3 × V_bat (26.6 V).
- `wx-mp-psu`: stiff sinking source (k 0.95, I_sc 20 A), V_bat 28.95 V, V_oc 1.25 → 1.02 × V_bat.
- `wx-mp-psu-nosink`: same, non-sinking source (bench PSU).

`on above M_c` = time switching at a plant ratio above `1 − t_min/T`.

### All scenarios (sum over boards and seeds, harvest/limiter means)

| scenario | runs | harvest base / guard [%] | dPV mean / min [Wh] | minpulse-M stops | revI trips base / guard | on above M_c base / guard [s] | Iout min base / guard [A] | limiter base / guard [%] |
|---|---|---|---|---|---|---|---|---|
| wx-cloud, -cv, -cv-ripple, wx-mid, wx-shade*, wx-sweep-* (12) | 192 | identical | ±0.004 / −0.02 | 0 | 0 / 0 | 0 / 0 | identical | identical |
| wx-dawn | 16 | 97.54 / 97.31 | −0.016 / −0.080 | 0 | 0 / 0 | 383.6 / 0.0 | −0.179 / −0.144 | 0.0 / 0.0 |
| wx-dusk | 16 | 98.16 / 98.18 | +0.002 / −0.030 | 0 | 16 / 0 | 540.7 / 0.0 | −1.022 / −0.159 | 0.0 / 4.3 |
| wx-low | 16 | 98.10 / 98.08 | +0.000 / −0.040 | 0 | 0 / 0 | 28.5 / 0.0 | −0.077 / −0.077 | 0.0 / 0.0 |
| wx-low-slow | 16 | 98.47 / 98.44 | −0.001 / −0.020 | 0 | 0 / 0 | 29.1 / 0.0 | −0.077 / −0.077 | 0.0 / 0.0 |
| wx-mp-ramp | 16 | 116.78 / 113.74 | −0.188 / −0.210 | 0 | 0 / 0 | 0.0 / 0.0 | 0.000 / 0.000 | 0.0 / 36.9 |
| wx-mp-psu | 16 | 3.86 / 1.83 | −3.361 / −10.170 | 8 | 333 / 0 | 1628.8 / 0.0 | −7.141 / −0.249 | 12.8 / 7.2 |
| wx-mp-psu-nosink | 16 | 2.40 / 1.93 | −0.729 / −4.350 | 8 | 31 / 0 | 1570.5 / 0.0 | −1.009 / −0.250 | 14.1 / 8.1 |

Guard fires over all wx scenarios: stop 16 (all in wx-mp-psu*, fry/fry-hq, one per run), floor active in wx-dusk
(4.3 %) and wx-mp-ramp (36.9 %), start blocks lower the start count in wx-dawn (270 → 257) and wx-dusk (176 → 161).
None of the 12 scenarios away from unity change.

### Near-unity scenarios per board (sum over 4 seeds)

| scenario | board | PV base / guard [Wh] | out base / guard [Wh] | revI base / guard | stops | starts base / guard | Iout min base / guard [A] | t(I<−0.05 A) base / guard [s] | charge back base / guard [C] |
|---|---|---|---|---|---|---|---|---|---|
| wx-dawn | fry | 24.45 / 24.39 | 24.14 / 24.08 | 0 / 0 | 0 | 68 / 60 | −0.155 / −0.111 | 5.7 / 4.8 | 2.2 / 2.6 |
| wx-dawn | fry-hq | 24.64 / 24.47 | 24.35 / 24.17 | 0 / 0 | 0 | 64 / 53 | −0.156 / −0.108 | 7.0 / 6.5 | 2.3 / 2.8 |
| wx-dawn | flat | 24.51 / 24.60 | 24.24 / 24.32 | 0 / 0 | 0 | 72 / 72 | −0.177 / −0.144 | 6.3 / 5.6 | 1.8 / 1.3 |
| wx-dawn | flat-hq | 24.67 / 24.56 | 24.42 / 24.30 | 0 / 0 | 0 | 66 / 72 | −0.179 / −0.140 | 7.6 / 8.0 | 1.8 / 1.3 |
| wx-dusk | fry | 24.73 / 24.73 | 24.35 / 24.39 | 4 / 0 | 0 | 44 / 36 | −1.022 / −0.146 | 25.2 / 22.5 | 9.1 / 5.0 |
| wx-dusk | fry-hq | 24.71 / 24.72 | 24.34 / 24.39 | 4 / 0 | 0 | 44 / 37 | −1.014 / −0.151 | 26.8 / 21.5 | 9.2 / 4.6 |
| wx-dusk | flat | 24.72 / 24.74 | 24.39 / 24.43 | 4 / 0 | 0 | 44 / 44 | −1.002 / −0.159 | 32.8 / 49.3 | 9.5 / 5.4 |
| wx-dusk | flat-hq | 24.72 / 24.72 | 24.40 / 24.47 | 4 / 0 | 0 | 44 / 44 | −0.999 / −0.154 | 32.9 / 14.0 | 9.5 / 1.9 |
| wx-mp-ramp | fry | 28.86 / 28.08 | 28.61 / 27.83 | 0 / 0 | 0 | 4 / 4 | 0.000 / 0.000 | 0.0 / 0.0 | 0.0 / 0.0 |
| wx-mp-ramp | fry-hq | 28.87 / 28.14 | 28.61 / 27.88 | 0 / 0 | 0 | 4 / 4 | 0.000 / 0.000 | 0.0 / 0.0 | 0.0 / 0.0 |
| wx-mp-ramp | flat | 28.85 / 28.10 | 28.69 / 27.98 | 0 / 0 | 0 | 4 / 4 | 0.000 / 0.000 | 0.0 / 0.0 | 0.0 / 0.0 |
| wx-mp-ramp | flat-hq | 28.83 / 28.08 | 28.69 / 27.96 | 0 / 0 | 0 | 4 / 4 | 0.000 / 0.000 | 0.0 / 0.0 | 0.0 / 0.0 |
| wx-mp-psu | fry | 46.50 / 20.80 | 37.18 / 20.96 | 108 / 0 | 4 | 116 / 12 | −7.141 / −0.167 | 593.2 / 86.6 | 246.2 / 10.7 |
| wx-mp-psu | fry-hq | 56.62 / 28.56 | 37.10 / 20.90 | 108 / 0 | 4 | 116 / 12 | −7.132 / −0.167 | 584.2 / 85.1 | 250.3 / 10.5 |
| wx-mp-psu | flat | 0.01 / 0.00 | −2.49 / −0.51 | 59 / 0 | 0 | 159 / 62 | −2.460 / −0.242 | 1418.2 / 469.7 | 310.3 / 64.1 |
| wx-mp-psu | flat-hq | 0.00 / 0.00 | −2.75 / −0.58 | 58 / 0 | 0 | 162 / 66 | −2.474 / −0.249 | 1494.5 / 547.4 | 343.7 / 73.5 |
| wx-mp-psu-nosink | fry | 30.60 / 27.51 | 39.98 / 22.78 | 12 / 0 | 4 | 20 / 8 | −1.009 / −0.162 | 588.4 / 40.6 | 227.7 / 5.8 |
| wx-mp-psu-nosink | fry-hq | 33.31 / 24.74 | 40.14 / 22.11 | 12 / 0 | 4 | 20 / 8 | −1.003 / −0.169 | 577.3 / 41.6 | 221.4 / 7.1 |
| wx-mp-psu-nosink | flat | 0.00 / 0.00 | −2.35 / −0.54 | 3 / 0 | 0 | 126 / 77 | −1.001 / −0.236 | 1594.8 / 550.6 | 291.2 / 65.8 |
| wx-mp-psu-nosink | flat-hq | 0.00 / 0.00 | −2.50 / −0.54 | 4 / 0 | 0 | 129 / 80 | −1.001 / −0.250 | 1616.4 / 553.0 | 310.1 / 67.8 |

Reading:

- **wx-dusk**: the guard removes all 16 revI trips and the 541 s above the onset. The floor is active 4.3 %, the
  harvest is unchanged.
- **wx-dawn**: 384 s above the onset → 0, no trips either way. Cost 0.23 pp mean harvest (−0.016 Wh mean, worst
  −0.08 Wh per run), mostly from start blocks (starts 270 → 257).
- **wx-mp-ramp**: no negative current with or without the guard (the soft panel never reaches `M_c` in this
  scenario). The floor costs ~0.75 Wh of 28.9 (−2.6 %), the energy between `M_start` and the MPP the base tracked.
- **wx-mp-psu / -nosink, fry**: the guard does what it is for. revI trips 108 → 0, reverse current −7.1 A → −0.17 A,
  charge back 246 → 11 C. The output energy roughly halves because the converter no longer runs at a ratio above
  `M_start`. Base's extra "harvest" there is in part charge pumped back and forth: base PV energy exceeds the output
  by ~9–19 Wh in the sinking case.
- **wx-mp-psu / -nosink, flat**: flat finds no MPP in this scenario with or without the guard (sweep-no-mpp every
  start, harvest 0, net output negative in both). The guard reduces the charge back 310 → 64 C and the reverse
  current −2.5 A → −0.25 A. The residual is the bounded region below `M_c` during the sweeps (−0.30 A calculated at
  51 µH, M 0.85). Why flat's sweeps fail here was not investigated; it is a pre-existing behaviour of this stiff
  source scenario on the 1.5 kHz flat sampler, not caused by the guard.

## Tests

| test | result |
|---|---|
| `test/host-stub/minpulse-test.cpp` (new, includes known-bad) | PASS; known-bad limits 0.93 / 0.905 / 0.90 FAIL as intended (0.90 by the ≥ 0.02 margin check); buck in DCM above the stop ratio exposed, boost not |
| `test/host-stub/minpulse-sim.sh` (new, wx-mp-ramp/psu/psu-nosink/manual/ccm/dusk, fry) | guard PASS; base FAIL (psu, nosink, dusk); first guard version without the follow-ups FAIL on wx-mp-manual (135 stops, 158 starts, −4.7 A, revI trip) and wx-mp-ccm (DCM entry to stop 8.9 ms vs 2.2 ms) |
| rect-cal-test | PASS |
| dcm-current-test | all ok |
| vconv-test | 1309 / 1309 |
| dawn-test | ALL PASS |
| vconv-spice-check | PASS |
| rcal-vin.sh | 192 runs, 0 outside the band, 22 failed (same as baseline), means equal to baseline |
| Unity QEMU | 262 tests, 0 failures, 25 ignored |
| esp32s3 build | ok, 11 % free, +0x340 bytes; rtprof_map --check 0 not in IRAM |
| non-wx Mac sims (a–e, d-*, rcal*) | identical to base except small trajectory differences in b, b-p0; no minpulse-M stops |

## Limitations

- Not tested on hardware. The bench procedure covers the start block, the floor and the stop in manual duty on fbuck;
  the stop on reverse current needs a source on the output and is covered only in the simulator.
- A stiff-source ratio step faster than ~14 samples (EWM lag + debounce) crosses `M_c` before the stop fires.
- DCM entry from CCM above `M_c` (stiff input falling while in CCM): the stop is immediate once the firmware flags
  DCM, but that flag follows the current EWM, so the pump runs for the EWM's lag first. In the sim
  (wx-mp-ccm) the entry was at M 0.918, below `M_c`; the reviewer's 13 A/ms case (M 0.95) was not reproduced
  closed-loop.
- Manual duty: after a `minpulse-M` stop the restart waits for `V_out < M_start·V_in`; with a load that drains the
  output this gives a stop/restart cycle every few seconds until `dc 0` (no floor in manual).
- Below `M_start` the min pulse still gives a bounded negative mean current at light HS (−0.1…−0.3 A, larger for
  smaller L). The guard does not address it; the existing revI-ewm protection and diode emulation do.
- PSU and PV-sim modes: the start block uses the setpoint, so `psu <V>` / a `pv` setpoint above `M_start·V_in` does
  not start (by design, documented in console.md and operating-modes.md); with the floor a PSU output sags to
  `M_start·V_in` when the input drops.
- The 1 % reading error is an assumption, not a measured property of each board.
- The PV plant clamps `V_in` at 1.05·V_oc when non-sinking (pvBlocking); a real bench PSU with output capacitance
  can rise further while charge is pumped back.
- flat in wx-mp-psu*: no MPP in base or guard, not investigated.

## Review follow-up spot check (pgx93, after the rebase onto main 21123e59)

wx-dawn / wx-dusk / wx-mp-ramp, fry, fry-hq, flat, flat-hq, seeds 1–2 (24 runs, tag minpulse-fix): mean harvest
97.33 / 98.12 / 113.83 %, 0 minpulse-M stops, 0 s above `M_c`, 0 revI trips, Iout min −0.144 / −0.159 / 0 A.
The reviewer's combined numbers: dawn 97.33, dusk 98.13.

## Reproduce

```bash
# binaries: build mppt-sim for base (c3ce3dc2 + this branch's mppt-sim.cpp) and guard, mirror with
#   offload build-sim <tree> /tmp/minpulse-sim-{base,wt}
for v in base wt; do for b in fry fry-hq flat flat-hq; do for sc in <19 wx scenarios>; do for s in 1 2 3 4; do
  echo "$v $b $sc $s"; done; done; done; done | xargs -P 18 -L1 doc/research/minpulse-guard/job.sh
python3 doc/research/minpulse-guard/analyze.py /tmp/minpulse-batch/out > results.md
```
