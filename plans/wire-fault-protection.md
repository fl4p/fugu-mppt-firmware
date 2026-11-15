*this document is an LLM generated placeholder*

# Wire-fault protection on the output side (issue #47)

Status: **implemented in warn-only mode (2026-11-12)**, see "Implementation notes" at the end.
Protection (`wire_protect=2`) is not to be enabled before the follow-ups listed there. The measurements it builds on are done and
field-validated (2026-11-12).

## Goal

Detect a degrading or loose connection between converter and battery, and limit the heat it
produces: warn, then derate, then shut down. The measurements already exist as telemetry. This
plan adds the decision logic.

## Signals available

| Signal | What it covers | Update rate | Precision (fry, measured) | Needs |
|---|---|---|---|---|
| `Rc` (WireResistance, `charger.h`) | converter terminal → BMS pack measurement (cable, fuse, BMS path) | refit per BMS frame (~4 s), forgetting time constant ~2.3 h | day medians 12.7–15.2 mΩ over 11-04..11-12 | BMS pack voltage + current over MQTT |
| per-frame excess drop (new, see below) | same path as `Rc` | every BMS frame | residual σ ≈ 15 mV per frame | BMS, a learned baseline |
| `Ro` (sweep fit, `out_impedance.h`) | whole output path including battery internal resistance | one value per qualifying sweep (~7/day in good sun, none on full-battery or dull days) | ±0.5 mΩ per sweep; 21.2–24.0 mΩ within one day | sweeps with ≥ 3 A span |

`Rb` (shared bus → pack) is common to both chargers and to any load on the pack. A sustained
rise above its baseline only raises a warning (section 6, alert source `wire`); it never derates
or trips.

Notes:
- `Rc` alone reacts too slowly to a sudden fault: its forgetting time constant is hours, and a
  fault slowly gets absorbed into the fit. The fast path below avoids that.
- Day-to-day spread of about ±1.5 mΩ is expected from copper temperature alone (0.4 %/K) and
  from the battery part of `Ro` (state of charge, temperature).
- flat produces neither signal until its Vout filter cap is fitted. The logic must treat that
  as "no data", never as "OK".

## Design

### 1. Baseline per board

- Learned, not configured: the median of the last 14 daily medians of `Rc` and of `Ro`.
- A day's median only enters the baseline when it lies within `+wire_dr_min` of the current
  baseline, so a slowly developing fault can't be learned in.
- The first 7 days with enough data are a learning phase: warnings only.
- Optional absolute cap `wire_r_max` (mΩ) for installations that may already be bad when first
  learned.
- Persisted as one NVS blob through the existing `KeyValueStorage` (`src/storage/key-value.h`),
  so a reboot doesn't restart learning. It is written at most once a day and only while the
  converter is disabled (night), because flash writes stall the RT loop by 33–65 ms. Not stored
  in a conf file: reprovisioning a board (`provision.py`) rewrites the conf partition and would
  wipe it or restore a stale baseline from another board's image. A console command `wire`
  shows the baseline and state; `wire-reset` clears it.

### 2. Fast path: per-frame excess drop

On each BMS frame (core 0, `updateWire`), using the frozen baseline fit (offset a, `Rc_base`, `Rb`):

    e = (Vout − Vpack) − (a + Rc_base·Iout + Rb·Ibat)     excess voltage drop
    Pw = e · Iout                                          excess power in the wiring

- `e` is averaged over 1 minute (~15 frames). Per-frame noise of 15 mV gives about 4 mV on the
  mean, or 0.16 mΩ at 25 A, so a few mΩ of fault is visible within a minute.
- Offset drift (the fitted `a` moved 15–31 mV over days) must be tracked slowly, or its
  equivalent at low current gated out: decisions use `Pw` only while Iout ≥ 5 A.

### 3. Slow paths

- **`Rc`:** `Rc − Rc_base`, with hours of time constant. Catches gradual degradation even when
  the fast path's offset tracking hides it.
- **`Ro`:** per sweep, `Ro − Ro_base`. Needs no BMS, so it is the only path on a system without
  BMS data.

### 4. Criterion: excess power, not a resistance ratio

A connector overheats from the extra power dissipated in it, ΔR·I², not from a resistance
ratio. The criterion is the excess power at the present current (fast path), or at the recent
maximum current (slow paths). Proposed defaults, to be tuned in the warn-only phase:

| Level | Condition (sustained, see 5) | fry at 25 A |
|---|---|---|
| warn | Pw ≥ `wire_p_warn` = 2 W and ΔR ≥ `wire_dr_min` = 4 mΩ | ΔR ≈ 4 mΩ (2.5 W) |
| derate | Pw ≥ `wire_p_derate` = 4 W | ΔR ≈ 6.4 mΩ |
| trip | Pw ≥ `wire_p_trip` = 8 W, or still above the derate level after derating | ΔR ≈ 13 mΩ |

The ΔR floor keeps normal temperature drift (±1.5 mΩ) from triggering at high current.

### 5. Persistence

- Fast path: the 1-minute mean above the threshold for 5 consecutive minutes.
- `Ro`: 3 consecutive qualifying sweeps above the threshold.
- `Rc`: above the threshold for 30 minutes.
- Any path may raise the level. Lowering it needs all fresh paths below the threshold minus
  hysteresis for 30 minutes. A trip doesn't release on its own (see 6).

### 6. Actions

- **warn:** an `ESP_LOGW` line on each level change (reaches the console, telnet and the MQTT
  log topic), plus telemetry fields `wf` (level 0–3) and `Pw` (W).
- **derate:** cap the output current at `sqrt(wire_p_derate / ΔR)`. This feeds the existing
  `Iout_max` path in `charger.h` (`min(lim, ioutLim)`) and the `IoutCurrentController`. No new
  control loop is needed. Example: ΔR = 10 mΩ, 4 W → 20 A.
- **trip:** stop the converter and latch until a console command `wire-reset` (after the repair,
  which also restarts baseline learning) or a reboot. The latch is not persisted (Fab,
  2026-11-12). The baseline is, so after a reboot the fast path detects a remaining fault
  again within minutes of current flowing.
- **alert:** each level change is also published on the MQTT alert topic (section 10).

### 7. Missing data

- No path fresh (no BMS, no qualifying sweep): telemetry `wf` = "unverified"; no action.
- The level holds its last state, since missing data must not release a derate or a trip.
- The warn level is kept while data is missing.
- If the protection has been enabled but unverified for more than 3 charging days, warn once,
  so a silent sensor (as on flat) isn't mistaken for a healthy wire.

### 8. Configuration (`limits.conf`)

| Key | Default | Meaning |
|---|---|---|
| `wire_protect` | 1 | 0 off, 1 warn only, 2 warn + derate + trip |
| `wire_p_warn`, `wire_p_derate`, `wire_p_trip` | 2, 4, 8 | W |
| `wire_dr_min` | 4 | mΩ, minimum ΔR for any level |
| `wire_r_max` | 0 | mΩ absolute cap on `Rc`, 0 = none |

The keys go into the config reference page and the conf editor (`META` + `FILE_KEYS`) together.

### 9. Placement and cost

- Everything runs on core 0 in `loopLF` (the `Ro` value is read through its atomic getter). The
  RT core only gets the existing current cap.
- RAM: about 100 bytes (baseline, 14 day medians as `uint16_t` µΩ·10 or similar, state, timers).

## Validation plan

1. **Host tests:** inject a resistance step (+3, +6, +15 mΩ) into the simulated sweep and BMS
   data. Check each level's detection time, the derate current, the trip latch, monotonicity
   (a larger fault never gives a lower level) and the missing-data behaviour.
2. **Replay all real fry data since 11-04** (BMS frames and sweeps) through the logic: zero
   warnings expected. That gives the false-alarm rate at the proposed thresholds.
3. **Field:** deploy with `wire_protect=1` (warn only) to both boards for 1–2 weeks, review `wf`
   and `Pw`, tune, then set `wire_protect=2` on fry.
4. **Known-bad test:** a small series resistor (about 10 mΩ, rated for the current) in fry's
   battery lead for an afternoon. That's a physical intervention for Fab.

### 10. MQTT alert topic (all error sources, not only wire)

- One retained topic per source: `pv/alert/<hostname>/<source>`, e.g. `.../wire`, `.../adc`,
  `.../protect` (OV/OC/temperature trips), `.../latency`. Next to the existing log stream
  `pv/log/<hostname>`.
- Payload JSON: `{"lvl":"ok|warn|derate|trip|unverified","msg":"...","t":<unix s>}`. `ok` clears.
- Retained, so a subscriber (Home Assistant) sees the current state after its own restart.
  One topic per source, so one source clearing doesn't hide another that is still active.
- Home Assistant: one automation on `pv/alert/+/+` notifies on any non-`ok` level; optionally an
  MQTT sensor per source for dashboards.
- Firmware: a small `alert(source, level, fmt, ...)` call on core 0 that publishes only on a
  state change, plus the existing log line. Wire protection is the first user; the existing
  shutdown paths can adopt it one by one.

## Decisions

1. Thresholds 2/4/8 W with a 4 mΩ floor: agreed (Fab, 2026-11-12), tuned in the warn-only phase.
2. Trip latch does not survive a reboot (Fab, 2026-11-12).
3. Baseline storage: NVS (section 1) (Fab, 2026-11-12).
4. Shared path (`Rb`): warn only, published as an alert; no derate or trip (Fab, 2026-11-12).
   The chargers can't relieve that section while a load discharges the pack through it.
5. MQTT alerts: `pv/alert/<hostname>/<source>`, retained JSON, one topic per source (section 10)
   (Fab, 2026-11-12). Wire protection is the first user; the existing shutdown paths adopt it in a
   separate change.

## Implementation notes (2026-11-12)

Code: `src/wire_guard.h` (logic, host-testable), `src/tele/alert.*` (MQTT alert topic), glue in
`charger.h`, `main.cpp`, `mppt.*`, `cli.cpp`. Test: `test/host-stub/wire-guard-test.cpp`.

Deviations from the design above:
- The `Rc` slow path grades the whole fit (offset + `Rc` + `Rb`) against the baseline fit at the
  present operating point. `Rc` alone gave false warnings: the fit splits the drop between `Rc`
  and `Rb` arbitrarily when output and battery current move together (11-12: `Rc` 9 → 24 mΩ,
  `Rb` 5.5 → −1.7 mΩ within 2 h).
- Decision 4 is only partly met (accepted by Fab, 2026-11-12): while charging, a shared-section
  fault is indistinguishable from an own-wire fault, so the fast path can derate or trip on it.
  The separate `Rb` path only warns.
- Baseline learn-in guards: a day must also lie within `wire_dr_min/2` of the lowest baseline
  seen (stops a slow ramp), and a day is rejected when most of its fast-path minutes show
  ≥ `wire_dr_min/2` (stops a fault leaking into the baseline via the fit offset).
- RAM about 730 B (14-day history, day buffers, alert table), flash +13 KB.

Results:
- Replay of all fry data 11-04..11-12: zero warnings; fast-path excess power max 1.55 W against
  a baseline learned from all days.
- Synthetic faults on November currents (11–19 A): +6 mΩ never warned; +15 mΩ warned after
  10 h and derated after 3.5 days. At 28 A (host test) detection takes 5–60 min.

Follow-ups before `wire_protect=2` (status 2026-11-15 below):
1. A current-independent warning (e.g. `Rc`/`Ro` ≥ baseline + 8 mΩ for a day), so a degrading
   connection is reported on low-current days. **Done 2026-11-15.**
2. False-alarm margin at high current: on 11-04 the fast path showed an apparent +3.4–4.5 mΩ at
   18.7 A (offset drift against the baseline), which would warn at 22 A and approach the derate
   level at 30 A. Observe the warn-only phase over high-current days, and consider fitting the
   baseline jointly instead of three separate medians. **Done 2026-11-15 (offset regression).**

## Offset drift and day warning (2026-11-15)

### Field observation

fry, 2026-11-13 11:30–14:00 UTC: telemetry `Pw` 2.2–3.2 W at `Iout` 12–18 A (about +7 mΩ apparent),
no real fault; `wf` stayed 0 only because of the learning phase. On 11-14/11-15 the device's `Pw`
reached 6.7/6.9 W. The device's baseline then rested on one partial afternoon (11-12 14:12 onward,
a = 23 mV, Rc 12.2 mΩ).

`Pw` sat at exactly 2.16185 W from 12:39 to 13:50 on 11-13 while `Iout` moved between 3 and 20 A:
`pw()` reported the worst path that was still *fresh*, and freshness counts charging minutes (15 for
the fast path, 60 for `Rc`, 360 for `Ro`). Cloud transients left too few steady BMS frames for new
fast-path minutes, and `WireResistance` failed its residual gate (σ > 40 mV, telemetry `Rc` absent from
12:28), so a held `Rc`-path value was published for 70 minutes. Holding is right for the level (a
night must not release it) but wrong for telemetry.

### What the data shows

Frames rebuilt from InfluxDB (fry 11-04..11-15, BMS V/A paired within 2 s, converter 1 s means,
`Iout` ≥ 5 A, steady current; 11 950 frames). Pooled fit dv = a + Rc·I + Rb·Ib: a 10.1 mV, Rc 14.46,
Rb 1.47 mΩ, residual σ 39 mV (17–44 mV per day; cloudy days are worst).

- Pinning Rb and fitting e = c + k·I per day (e = residual against the pooled fit): the offset c
  ranges −54..+63 mV between days, the slope k only −2.7..+4.7 mΩ. A 2048-frame running fit gives
  c −84..+38 mV, k −2..+5 mΩ. The drift is mostly offset, not resistance.
- The offset is not a pure function of time: it correlates with pack voltage (pooled +48 mV/V, per
  day 29–106 mV/V, about a 0.2 % gain mismatch between the converter's Vout sense and the BMS), which
  makes it current-correlated within a morning ramp. Not modelled (collinear with I and Ib); a Vout gain
  calibration against the BMS would remove much of it at the source.
- 11-14 shows an offset step of about +40 mV from the morning on (+35 mV at 6 A at 08:30), which a
  fixed baseline offset turns into excess resistance at every current.

### Design choice

The fast path now takes its offset from a forgetting regression of the one-minute frame sums of
(dv − Rb_base·Ib) against Iout (7 sums in doubles, memory `OFS_TAU` = 120 minutes with frames), and
subtracts only its intercept: e = mean(dv − Rb_base·Ib) − c_fit − Rc_base·I. Why this and not the
alternatives:

- **Slow offset tracking from low-current frames** absorbs ΔR·I_low of a real fault and needs a
  separate low-current gate. The intercept of a regression over current is the same idea without
  that bias: a resistance change goes into the slope, which stays in the graded quantity.
- **Grading by slope alone** (ΔR from a fit of e vs Iout over a window) is blind to a step at
  constant current and has ±3 mΩ noise per window; it is kept as the slow `Rc` path instead (the
  regression's slope against Rc_base, graded at the reference current).
- **A jointly fitted baseline** fixes the triple's consistency but not the drift, which is the bulk
  (offset range 120 mV vs. slope range 7 mΩ).

Guards around the regression:
- It is used only with ≥ 150 frames and ≥ 4 A² variance of Iout (else the fast path has no value,
  which can end in `unverified`), and only while |c_fit − a_base| ≤ 0.15 V.
- A minute that lies off the regression line by ≥ `wire_p_warn`/2 (power) is a step, not drift, and
  is not folded in; nothing is folded while a level is held. With only a grade gate (fold unless the
  minute grades above ok) a +3 mΩ step at 28 A tilted the line, lowered the intercept and warned after 25 min; without the hold gate a +15 mΩ
  fault folded in through its low-current minutes and the intercept rose by 28 mV in 3.5 h (fast ΔR
  15 → 11 mΩ).
- Known limitation: a step large enough to be excluded at all currents above ~8 A leaves only
  low-current minutes for the regression; after about a day of such data it is no longer separable
  and the fast path reports nothing (held level stays, alert goes `unverified` if nothing is held).

τ was chosen on the replay (fast-path max Pw over 11-04..11-15): 60 min 1.48 W but day warnings
during learning, 120 min 1.61 W, 240 min 1.91 W, 480 min 2.39 W, 960 min 2.62 W.

Other changes:
- **Day warning (follow-up 1):** at day close, after learning, the median fast-path ΔR of the day
  (decimating 16-sample buffer; without BMS data: median Ro − Ro_base) ≥ 2·`wire_dr_min` sets a
  warning that never derates or trips, persists in the blob's spare byte (`flags`, no version bump),
  and clears after a judged day below 1.5·`wire_dr_min`. No new conf key. Replay day medians after
  learning: −3.3..+1.8 mΩ (threshold 8).
- **Unverified alert (task 3):** `alertLevel()` is gone; the alert follows `wf()`, which is -1 when no
  level is held and no path has data (or none for more than 3 charging days). At boot the alert now
  says `unverified` until the first verdict.
- **Telemetry `Pw`:** only from a path that produced a value within 2 minutes (`PW_AGE_S`); held
  values still drive the level but are no longer published.
- The `Rb` path's cap uses the offset-corrected excess drop; the `Rc` path no longer uses the
  `WireResistance` offset at all (that fit is still the baseline source and telemetry).
- RAM +120 B (`sizeof(WireGuard)` 576 → 696).

### Replay (fry, `test/host-stub/wire-guard-replay.cpp`, data from `etc/wire_replay_fetch.py`)

Clean data 11-04..11-15 (natural learning from 11-04, levels as with `wire_protect=2`):

| | before | after |
|---|---|---|
| fast-path Pw max | 2.57 W (11-14 12:03, 22.6 A, ΔR 5.2 mΩ) | 1.67 W |
| fast minutes ≥ warn (Pw and ΔR) | 1 | 0 |
| minutes at level ≥ 1 / any warning | 0 / 0 | 0 / 0 |
| `Rc`-path Pw max | 3.95 W (11-14, held into 11-15) | 1.44 W (11-05, learning), ≤ 0.56 W after |
| `Ro`-path Pw max (unchanged code) | 4.11 W | 4.11 W |
| day warnings | – | 0 |

From the deploy time (11-12 14:12, the device's one-afternoon baseline): fast 3.12 → 2.16 W, `Rc`
path 4.14 → 0.51 W, `Ro` path 5.80 W (unchanged); the device itself reported up to 6.9 W.

Injected series resistance at real currents (≤ 24.6 A), minutes to warn / derate / trip, before → after:

| onset | +6 mΩ | +15 mΩ |
|---|---|---|
| 11-12 10:00 | 1556 / – / – → same | 69 / 81 / 1559 → 69 / 81 / 1556 |
| 11-13 10:00 | 116 / – / – → 1424 / – / – | 46 / 94 / 119 → 46 / 116 / 1763 |
| 11-14 09:00 | 117 / – / – → same | 37 / 98 / 146 → 39 / 98 / 383 |
| 11-14 12:00 | 203 / – / – → same | 203 / 203 / 203 → same |

The replay doesn't apply the derate cap to the recorded current, so a trip here comes from the
over-derate rule at ≥ 18.3 A. Where "after" is slower, "before" was helped by the positive offset
drift of 11-13/11-14 (day medians of the fast ΔR for the +6 mΩ run, 11-13..11-15: before 5.9, 10.1,
9.8; after 4.7, 5.1, 2.6 mΩ). At ≤ 19 A a +6 mΩ fault dissipates ≤ 2.2 W, at the warn threshold.

Host test (`test/host-stub/wire-guard-test.cpp`, all pass): 30/120 mV offset ramps, ±30 mV waves and
their combination at 30 A raise nothing (the old guard derates on +120 mV and on ramp+wave); +6/+15 mΩ
on top of the drift warn/trip within 5.1 min; +3 mΩ below the floor stays silent; monotonic over
0..30 mΩ (fast and all paths) and over 0..20 mΩ at 12 A for the day warning (+10 mΩ and above warn by
day, never cap); day warning from Ro without BMS, cleared after repair, persisted; `wf`/alert
unverified at boot and after 2 days of lost data while charging; `Pw` absent 3 min after data stops.

Open:
- The `Ro` path reached 4.1 W (7-day baseline) and 5.8 W (one-afternoon baseline) on 11-14 in the
  replay. It did not warn (3 consecutive qualifying sweeps needed), but on 11-14 there were 30 sweeps
  within one hour (10:01–10:59), so 3 consecutive sweeps cover only a few minutes. The replay's sweep
  fits are noisier than the device's (device `Ro` 20.3–23.0 mΩ that day). Consider spacing or a
  median over sweeps before `wire_protect=2`.
- The device is still learning (since 11-12) with a baseline of one partial afternoon; the new code
  keeps the blob layout, so the learned days carry over.
- Field check of the new fast path over high-current days before `wire_protect=2`.
