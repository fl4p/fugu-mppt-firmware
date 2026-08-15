*this document is an LLM generated placeholder*

# PV-sim mode: fboost simulates a solar panel at its output

*(reviewed by a Codex agent; 4 blockers + 9 risks folded in below)*

## Context

The power-loop rig feeds fbuck (MPPT DUT) from fboost. Today the "solar" source is a stiff
external PSU / fixed-duty fboost — a CV source with no maximum power point, so the tracker
can't actually be exercised (doc/Automated Bench Tests.md:14-27 points at external
Keysight/Chroma SAS gear). This feature makes fboost regulate its **output** along a PV I-V
curve V = f(Iout), parameterized by Isc, Voc, k = Vmp/Voc, so fbuck sees a real MPP. It
builds on the existing PSU constant-voltage mode (mode enum, RT-owned ticketed mailbox,
protection specializations, cold-start, single setpoint plumbing point) and reuses the PV
curve model already in the vconv simulator (`src/sim/vconv.h:28-54`).

On implementation start, copy this plan into the repo as `plans/pv-sim-mode.md` with the
repo's required first line (`*this document is an LLM generated placeholder*`).

## Design decisions

- **D1 — PSU-mode variant, not a fourth OpMode.** Keep `OpMode::Psu` (src/app_state.h:12).
  Add RT-owned `pvSim` state in `MpptController`; when active, `psuVsetpoint` becomes the
  per-tick computed curve voltage, so most existing readers keep working (CVP target
  src/mppt.cpp:94, stuck watchdog src/main.cpp:752-757, Vr window src/mppt.h:732, cold-start
  src/main.cpp:982-987, trip escalation/latch). Readers that must see **Voc, not the moving
  value**: `computeOvThreshold()`, `currentPsuFeasibility()`, **and `cmdOvset`'s conflict
  check via `getRequestedPsuSetpoint()`** (src/cli.cpp:1310, src/mppt.h:907 — must learn
  `EnablePv` and return Voc, else an operator can set OV between live Vset and Voc and the
  next protection pass hard-latches, src/mppt.h:534).
- **D2 — Control law.** Each tick, before the CVP array: `Vset = pvModel.voltage(Iout_f)`,
  clamped to `[Vin + BoostHeadroomV, min(Voc, limits.Vout_max)]` (floor wins only inside the
  ceiling; if `Vin + headroom >= Voc` the interval is empty — hold Vset = Voc and let the
  Voc-pinned feasibility latch handle it, consistent with the 0.5 V entry gate — **one**
  shared headroom constant, reuse `BoostHeadroomV = 0.5` from src/mppt.h:824, not a second
  1.0 V constant). Then slew-limit (`pv_slew`, default 200 V/s) with **dt clamped to
  ≤ 10 ms** (dtCtrl can be huge after stalls/calibration/backoff since early returns don't
  refresh `lastUs`, src/mppt.cpp:24-30 — an unclamped dt lets Vset jump and the Kd=12000
  normalized PD strip all duty in one update). Write `psuVsetpoint` directly — never
  `setPsuSetpoint()` (src/mppt.h:805-810) per tick, it resets the Vout PD.
  Feedback tap: `sensors.Iout->med3.get()` smoothed by a **dedicated single-pass EWMA inside
  `pvSim` (span ~16)** — NOT the profile's `ewm.avg` tap: fboost's iout span 60 with the
  two-pass EWM (src/math/statmath.h:114,141) is ~59 update intervals of group delay on top
  of stale 450 SPS INA226 data, and *more* filtering there worsens phase margin, it doesn't
  add stability. Keep the smoothing knob local (`pv_iout_span`) so tuning doesn't touch the
  protection filters.
- **D2b — PV-specific current ceiling (blocker fix).** In PSU+PV mode the Iout limiter
  target (src/mppt.cpp:70,102) becomes
  `min(limits.Iout_max, pvSim.model.isc * 1.1f)` — otherwise, once the curve hits the Vin
  floor the converter is a stiff CV source limited only by the hardware 40 A
  (config/lab/fboost/conf/limits.conf). Document explicitly: a boost has intrinsic body-diode
  pass-through at Vout ≈ Vin and this board has no panel-disconnect switch
  (config/lab/fboost/conf/board.conf:22) — below the Vin floor no firmware limit controls
  the current; the external input PSU's current limit is the real backstop there.
- **D3 — Interface.** CLI `pv <isc> <voc> [k]` / `pv off` / `pv` (status), through the PSU
  mailbox with one new `PsuCommand::EnablePv`. **Entry vs update are different (blocker
  fix):** initial entry (not already PV-active) behaves like PSU enable — full feasibility
  check, `setPsuSetpoint(voc)` (one-time PD reset is fine, converter arms from zero).
  An `EnablePv` while already PV-active is an **in-place curve update**: swap model params
  only, keep `psuVsetpoint` where it is and let the slew limiter walk it — no PD reset, no
  jump to Voc. That makes `pv scale <s>` (irradiance knob: re-post with `isc = baseIsc*s`,
  base kept separately so scales don't compound) safe on a loaded converter.
  Boot: `converter.conf mode=pv` + `pv_isc/pv_voc/pv_k/pv_slew`. `pv off` →
  `requestPsuManual(0, -1)` (ramp to 0, Manual) — unlike `psu off`, no MPPT-sweep fallback
  on a bench source.
- **D4 — Model location.** Extract PV math from `src/sim/vconv.h:28-54` into new
  `src/math/pv_model.h` (pure, header-only, NOT gated by WITH_VCONV); add inverse
  `voltage(i)`. `VirtualConverter` embeds a `PvModel` and forwards its public API —
  **including updating `src/sim/vconv.cpp`'s direct `voc_` reads at :110, :131, :137**
  (blocker fix; "vconv.cpp untouched" was wrong).
- **D5 — Profile.** New `config/lab/fboost_pv/` (copy of `config/lab/fboost`); existing
  fboost profile untouched (its `tracker.conf::target_duty_cycle=0.65` forces Manual and
  silently vetoes `mode=psu`/`mode=pv`, src/mppt.cpp:340-355; add an explicit `ESP_LOGE` for
  that combination). Runtime entry from the stock profile also works via the mailbox.

## Steps

### 1. `src/math/pv_model.h` (new) + vconv refactor

```cpp
struct PvModel {
    float isc, voc, k;
    float alpha, norm;                              // Newton-solved, cached
    float vocOverAlpha_, invIscNorm_, expNegAlpha_; // hot-path reciprocals (no HW divide)
    PvModel() { set(8.f, 40.f, 0.8f); }             // derived state ALWAYS initialized
    void set(float isc, float voc, float k);        // Newton solve, from vconv.h:28-45
    float current(float v) const;                   // from vconv.h:47-54
    float voltage(float i) const;                   // NEW inverse
};
```

`voltage(i)`: `i <= 0 → voc`; `x = 1 - i*invIscNorm_`; `x <= expNegAlpha_ → 0`;
else `voc + vocOverAlpha_*logf(x)`, clamped `[0, voc]`. One `logf`/tick, no divides.
Exact inverse of `current()` by construction.

`src/sim/vconv.h`: replace `isc_/voc_/pvK_/pvAlpha_/pvNorm_` with `PvModel pv_;`
(`setPv`→`pv_.set`, `pvCurrent`→`pv_.current`, getters forward). `src/sim/vconv.cpp`:
update the direct `voc_` uses (:110, :131, :137) to the accessor.

Host test (extend `test/host-stub/vconv-test.cpp` or new file): V(0)=Voc, V(≥Isc)=0,
monotone decreasing, round-trip |V(I(v))−v| < 1e-3·Voc over a grid, k ∈ {0.5,0.75,0.85,0.95},
plus a **default-constructed PvModel** sanity test (derived constants valid without set()).

### 2. `src/mppt.h` — state, mailbox, protection/ovset selectors

- `enum class PsuCommand`: append `EnablePv`.
- RT-owned state next to `psuVsetpoint` (src/mppt.h:801):
  `struct { bool active=false; PvModel model; float slewVps=200.f; float baseIsc; EWMA iout; } pvSim;`
  (single-pass EWMA, span from `pv_iout_span`, default ~16).
- Pendings next to `pendingPsuSetpoint`: `std::atomic<float> pendingPvIsc/Voc/K`.
  **Published curve state must be a coherent snapshot** — mirror the mailbox's
  seqlock/version pattern (src/mppt.h:249): version counter + isc/voc/k/active, so
  OTA save (src/cli.cpp:497-528) and measure-coil restore (src/selftest/measure_coil.cpp:258)
  can't read a mixed-generation set during a concurrent `pv scale`. `main.cpp`'s status
  token reads only the published mirror, never RT-owned `pvSim.active`.
- `queuePvCurve(float isc, float voc, float k, bool bootRequest=false)`: precheck **finite**
  isc>0, voc>0, slew, and `0.5f <= k <= 0.95f` (the alpha family can't realize k<0.5 — the
  Newton solver floors at 1e-3 and yields a ~linear curve; k→1 is pathological; don't copy
  `vconv pv`'s NaN-admitting checks at src/cli.cpp:1485), plus
  `validatePsuSetpoint(voc, limits.Vout_max, false, NAN, getExplicitOvLimit())`;
  store pendings; `postPsuCommand(PsuCommand::EnablePv)`.
- `applyPendingPsuCommandRt` (src/mppt.h:919-1001):
  - `EnablePv` requires `freshTelemetry` like `Enable`.
  - If NOT already PV-active: revalidate Voc with fresh Vin
    (`validatePsuSetpoint(voc, …, converter.boost(), vin, ovLimit)` — no-load operating
    point IS Voc); warn if `k*voc < vin + 2` (MPP below boost floor). On success:
    `pvSim.model.set(...)`, seed `pvSim.iout` and `pvSim.baseIsc`, `pvSim.active = true`,
    publish snapshot, `setPsuSetpoint(voc)`, `g_app.opMode = OpMode::Psu`.
  - If already PV-active: **in-place update** — same validation, then swap model params +
    publish; keep `psuVsetpoint`, no PD reset (slew limiter walks to the new curve).
  - Clear `pvSim.active` (+ published snapshot) in every disable path AND on plain `Enable`.
- Protection/CLI pinned on Voc:
  - `computeOvThreshold()` (src/mppt.h:1030-1040): `base = pvActive ? model.voc : psuVsetpoint`.
  - `currentPsuFeasibility()` (src/mppt.h:1007-1011): validate Voc when PV-active; the
    infeasibility latch (src/mppt.h:534-546) then fires only for "Vin within 0.5 V of Voc",
    which matches the empty-clamp-interval case in the tick law.
  - `getRequestedPsuSetpoint()` (src/mppt.h:907): recognize `EnablePv` (return pending Voc);
    when PV-active return published Voc — so `cmdOvset`'s 0.98-margin conflict check
    (src/cli.cpp:1310-1323) guards the top of the curve, not the instantaneous Vset.
  - Vr window and stuck watchdog read the moving setpoint — correct as-is.
- Testable per-tick law: `float pvAdvanceSetpoint(float ioutMed3, float vinFiltered, float dt);`
  (RT-only, clamps dt to ≤10 ms internally, no controller reset).

### 3. `src/mppt.cpp` — per-tick law, Iout ceiling, boot, telemetry

- Iout limiter target (src/mppt.cpp:70): when `psuMode() && pvSim.active`,
  `Iout_max = min(limits.Iout_max, pvSim.model.isc * 1.1f)`.
- In `update()` before the CVP array (src/mppt.cpp:87-89):
  ```cpp
  if (g_app.psuMode() && pvSim.active)
      pvAdvanceSetpoint(sensors.Iout->med3.get(), sensors.Vin->ewm.avg.get(), dtCtrl);
  ```
  Body: `iF = pvSim.iout.add(max(iout,0))`; `vTgt = model.voltage(iF)`;
  `vTgt = min(vTgt, min(model.voc, limits.Vout_max))`; if Vin finite,
  `vTgt = max(vTgt, vin + BoostHeadroomV)` but never above the ceiling (empty interval →
  hold ceiling, feasibility latch owns the fault); `dt = min(dt>0?dt:6e-4f, 0.01f)`;
  `psuVsetpoint += constrain(vTgt - psuVsetpoint, -slewVps*dt, +slewVps*dt)`; publish.
  The Vout CVP at :94 picks it up; limiter min-select, PSU CV branch (:231-243), duty slew
  (:253-266) untouched.
- `begin()` (src/mppt.cpp:354-366): `mode == "pv"` branch — read `pv_isc/pv_voc/pv_k`
  (k default 0.8), `pv_slew` (clamp ≥10), `pv_iout_span`; `queuePvCurve(..., true)`; failure
  → `g_app.setupErr = true`. `ESP_LOGE` when `targetPwmCnt` overrides `mode=pv`.
  **Boot calibration**: at boot `g_app.psuMode()` is still false (OpMode::Psu is assigned
  later by the RT consumer, src/mppt.h:997), so the psuMode() branch at src/mppt.cpp:393
  doesn't fire — make `begin()` recognize a queued boot PSU/PV request explicitly and call
  `sampler.startCalibration()` without `startSweep()` (fixes the existing fragile accident
  for mode=psu too).
- `telemetry()` (src/mppt.cpp:401-487): when `psuMode()`, every 10th point add
  `Vset` = `getPsuSetpoint()`. No sym_line_protocol.h edit — symbols are interned
  dynamically on first addField (src/tele/sym_line_protocol.h:93,168).

### 4. `src/cli.cpp` — `pv` command, status, save/restore

- `cmdPv` next to `cmdPsu`, `cli.addBoundlessCmd("pv", cmdPv)`:
  - no args → status: active/pending, Isc/Voc/k, live Vset/Vout/Iout, trips/latched.
  - `pv <isc> <voc> [k]` → `queuePvCurve` + `waitPsuCommand` ticket wait (src/cli.cpp:93-101),
    errors via `psuErrorText`.
  - `pv off` → `requestPsuManual(0, -1)` + wait.
  - `pv scale <s>` (0 < s ≤ 1.2): re-post EnablePv with `isc = baseIsc*s` (in-place update
    path, no setpoint jump).
- `cmdStatus`: extend the PSU block with curve params when active.
- OTA save/restore (src/cli.cpp:497-528) + measure-coil restore
  (src/selftest/measure_coil.cpp:259-285): read the **versioned snapshot**; restore via
  `queuePvCurve(...)` instead of `queuePsuSetpoint(saved-instantaneous-Vset)`.
- Status-line token (src/main.cpp:828-834): `"PVS"` when the published snapshot says active.

### 5. Conf / docs / config-tool (three places together, per CLAUDE.md)

- `doc/Configuration.md` converter.conf table: `mode` gains `pv`; new rows `pv_isc` (A),
  `pv_voc` (V), `pv_k` (0.5–0.95, default 0.8), `pv_slew` (V/s, default 200),
  `pv_iout_span` (default 16).
- `etc/config-tool/conf-editor.html`: add keys to `META` + converter.conf `FILE_KEYS`
  (regenerate via `etc/config-tool/scrape_conf_keys.py --write`).
- `doc/Power Loop.md`: "PV-sim source" section (also fix the stale `vin_min=72` prose at
  :31 — the actual fbuck lab profile has `vin_min=10.5`). Note the feasibility semantics
  (Vin rise truncates the curve via the floor clamp rather than latching, until Vin+0.5 ≥
  Voc) and the pass-through caveat from D2b. `doc/Automated Bench Tests.md`: pointer that
  fboost PV-sim is the in-house SAS alternative.
- New `config/lab/fboost_pv/conf/`: copy of fboost, `tracker.conf` without
  `target_duty_cycle`, `converter.conf` + `mode=pv`, `pv_isc=5`, `pv_voc=60`, `pv_k=0.8`.
  (Note: 1.5·Voc = 90 clamps at `vout_max=85` — an explicit `ovset` stays the real
  bring-up guard; Vmp=48 is above `vin_min=26`.)

### 6. Tests

- Host: PV model tests (step 1).
- On-target Unity `test/test_pv_sim.cpp` (+ test/main.cpp registration), following
  test/test_psu_mode.cpp:
  - Entry: `queuePvCurve` → `applyPendingPsuCommandRt(true)` → psuMode, Vset==Voc, active.
  - EnablePv deferred on stale telemetry; rejected for voc > Vout_max, ovset conflict,
    boost Voc ≤ Vin+0.5, non-finite/out-of-range k.
  - **In-place update**: second EnablePv while active keeps Vset (no jump to Voc), swaps
    params.
  - `computeOvThreshold()` and `getRequestedPsuSetpoint()` pinned at Voc while
    `psuVsetpoint` sits below Voc; `ovset` conflict path exercises the Voc value.
  - `pvAdvanceSetpoint`: slew clamp, **dt clamp (dt=5 s input → bounded step)**, Vin-floor,
    Voc ceiling, empty-interval hold, V(I) spot values, no PD reset.
  - Iout limiter target = isc·1.1 when active.
  - Mailbox races (last-writer-wins, src/mppt.h:258): pending EnablePv overridden by
    off/manual/plain-Enable and vice versa — overwritten waiter never completes
    (per test/test_psu_mode.cpp:211 pattern); all disable paths clear active.
  - Trip path: simulated load-shed OV trip → 100 ms retry → escalation counter behavior
    with PV active (threshold from Voc; >8 trips/60 s latch, src/mppt.h:381).
  - Save/restore: versioned snapshot round-trips params, restore re-queues the curve.

### 7. Bench validation on fboost→fbuck

State-changing commands only with user confirmation; take locks (serial port, fugu-rig)
first. Sequence:

1. Build (`idf.py -B build-<tag>`), host tests, `RUN_TESTS=1` on-target suite. **app-flash**
   fboost only; do NOT provision `fboost_pv` yet — first bring-up is runtime-CLI on the
   existing profile so the rig's fixed-duty ladders keep working.
2. Read-only preflight: `status`, status line, sensor sanity, fbuck idle. External input PSU
   at usual voltage with a **low current limit** (it is the only real backstop below the
   Vin floor, per D2b).
3. [confirm] `ovset 70` (≥ 0.98-margin above Voc=60 conflict rule → must be > ~61.2; 70
   leaves load-shed headroom — OV trips on a single raw Vout sample, src/mppt.h:562), then
   `pv 2 60 0.8`. No load: expect Vout → 60.0 (Voc).
4. [confirm] Step fbuck manual duty upward → walk down the curve; compare telemetered
   Uo/Iout against PvModel predictions (few %, dominated by the virtual-Iout
   `power_conversion_eff` guess). **Load-step test**: a deliberate fbuck duty step and a
   load-shed (duty to 0) — watch Vset↔Uo lag, overshoot vs the 70 V ovset, limit-cycling
   near the knee. Tuning knobs if it rings: `pv_slew` down, `pv_iout_span` up (local EWMA,
   not the profile filter).
5. [confirm] fbuck `sweep` + MPPT: settles near Vmp=48 V / Pmp≈87 W; re-finds MPP after
   `pv scale 0.5`.
6. Only after stable behavior: provision `fboost_pv`, reboot, verify `mode=pv` cold start
   (calibration without sweep), re-run 4–5, `pv off` → converter to 0.

## Risks

- **Virtual-Iout algebraic coupling (main risk).** fboost has no Iout sensor:
  `Iout = Iin·Vin/Vout·eff` (src/adc/sensor_setup.cpp:239) — Vout in the denominator means
  a falling Vout raises computed Iout and lowers Vset further while INA226 data (~450 SPS)
  is stale. Mitigations: local short EWMA + slew limiter + Vin-floor clamp. Filtering is
  delay, not intrinsic stability — the bench load-step test (step 7.4) is the real gate,
  and the closed-loop behavior should be watched near the knee where curve slope is
  steepest.
- **Moving setpoint through the normalized Vout PD** (kp 1500 / kd 12000): slew + dt clamp
  bound the per-tick D-kick (200 V/s ⇒ ≤0.12 V/tick at 1.7 kHz); `ctrl_vout_td` in the
  profile is the escape hatch. Never reset the PD per tick.
- **Load-shed OV → trip escalation**: stale-high Iout holds Vset low after load removal
  while the converter still carries energy; repeated OV events escalate (100 ms retries,
  latch after >8/60 s). Covered by tests (6) and bench step 7.4; `ovset 70` gives headroom.
- **Below the Vin floor firmware cannot limit current** (body-diode pass-through, no panel
  switch on fboost) — external input PSU current limit is the backstop; documented in
  Power Loop.md.
- **Feasibility semantics**: pinning on Voc means a mid-run Vin rise truncates the curve
  (floor clamp) instead of latching, until Vin+0.5 ≥ Voc. Acceptable for a bench source;
  documented.
- **Power-loop circulation**: emulated "solar" power recirculates, the external PSU only
  supplies losses — keep its current limit low; watch Iin vs `iin_max=40` when raising Isc.

## Critical files

- `src/math/pv_model.h` (new), `src/sim/vconv.h` + `src/sim/vconv.cpp` (delegate + voc_ refs)
- `src/mppt.h` (pvSim state, mailbox EnablePv + in-place update, OV/feasibility/ovset
  selectors, published snapshot)
- `src/mppt.cpp` (per-tick law, Iout ceiling, boot `mode=pv` + calibration, telemetry `Vset`)
- `src/cli.cpp` (cmdPv, status, OTA save/restore) + `src/selftest/measure_coil.cpp`
- `src/main.cpp` (status token)
- `doc/Configuration.md`, `doc/Power Loop.md`, `etc/config-tool/conf-editor.html`,
  `config/lab/fboost_pv/` (new), `test/test_pv_sim.cpp` (new), `test/host-stub/`
