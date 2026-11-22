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
- ~~The `Ro` path reached 4.1 W (7-day baseline) and 5.8 W (one-afternoon baseline) on 11-14 in the
  replay. It did not warn (3 consecutive qualifying sweeps needed), but on 11-14 there were 30 sweeps
  within one hour (10:01–10:59), so 3 consecutive sweeps cover only a few minutes. The replay's sweep
  fits are noisier than the device's (device `Ro` 20.3–23.0 mΩ that day). Consider spacing or a
  median over sweeps before `wire_protect=2`.~~ Done 2026-11-17, below.
- The device is still learning (since 11-12) with a baseline of one partial afternoon; the new code
  keeps the blob layout, so the learned days carry over.
- Field check of the new fast path over high-current days before `wire_protect=2`.

## Ro path: median and spacing (2026-11-17)

Host-tested and replayed, not deployed.

- Each qualifying sweep enters a ring of the last `RO_MED` = 5 sweeps (int16, 10 µΩ, as the day
  buffers). The graded value is their median against `Ro_base`, so 2 outliers in 5 don't move it,
  while a real shift does once 3 of 5 sweeps carry it.
- A graded value enters the `N_RO` = 3 history at most once per `RO_GAP` = 20 charging minutes; sweeps
  in between only enter the ring. 20 minutes is longer than 5 sweeps take at the fastest rate seen
  (~2 min apart on 11-14), so successive graded values share no sweep, and a level needs the median
  above threshold over at least 40 charging minutes. Nights don't count (charging minutes).
- Below `RO_MED_MIN` = 3 sweeps since boot there is no `Ro` value: the path is not fresh, `wf` stays
  unverified unless another path has data. The boot-sweep rule (wait for a reference current) and
  `STALE_RO` = 360 charging minutes since the last graded value are unchanged.
- Release is asymmetric: the `Ro` path counts as below only when the highest of its 5 sweeps is below
  75 % of the thresholds (`roDown`). Without that, a lagging median (fault in 1–2 of 5 sweeps) could
  release a level raised by the fast path once the fast path goes stale.
- `sizeof(WireGuard)` 696 → 696 (host and xtensa-esp32s3; `roIn` dropped, members moved into padding),
  `sizeof(Blob)` 124 unchanged.

Replay fry 11-04..11-16 (`etc/wire_replay_fetch.py`), max `Ro`-path Pw per day [W], before → after:

| day | natural learning | primed baseline (`-p`) |
|---|---|---|
| 11-05 | 0.33 → 0.19 | 0.53 → 0.33 |
| 11-06 | 0.28 → 0.28 | 0.42 → 0.50 |
| 11-07 | 0.14 → −0.06 | 0.21 → −0.01 |
| 11-09 | 0.96 → 0.04 | 0.99 → 0.07 |
| 11-10 | 0.03 → −0.29 | 0.03 → −0.26 |
| 11-12 | 1.01 → 0.42 | 1.01 → 0.42 |
| 11-13 | 0.28 → −0.22 | 0.30 → −0.20 |
| 11-14 | 4.11 → 0.23 | 4.11 → 0.23 |
| 11-15 | 1.34 → 0.77 | 1.32 → 0.76 |
| 11-16 | 1.00 → 0.67 | 1.00 → 0.67 |

No level in either. From the deploy time (one-afternoon baseline, `-s` 11-12 14:12) the `Ro` path
max drops from 5.71 to 1.90 W on 11-14 and from 2.71 to 2.15 W on 11-15 (that baseline is 3.5 mΩ low
on `Rc`); still no level.

Injected series resistance, `Ro` path only (`-P 4`), minutes to warn / derate / trip, before → after:

| onset | +6 mΩ | +15 mΩ | +100 mΩ |
|---|---|---|---|
| 11-14 09:00 | 117 / – / – → 353 / – / – | 62 / 98 / 383 → 145 / 178 / 1523 | 62 / 62 / 62 → 145 / 145 / 145 |
| 11-12 10:00 | 2864 / – / – → 252 / – / – | 81 / 81 / 3203 → 206 / 206 / 4343 | |

On 11-14 the +15 mΩ warning comes 104 charging minutes after the onset (sweeps at 09:13, 09:43, then
from 10:01; graded at 10:16, 10:49, 11:25). +6 mΩ is at the warn threshold at the day's currents, so
the noise of single sweeps made it look faster before; on 11-12 the median finds it earlier. Trips
come later because 3 graded values must lie above 8 W, i.e. ≥ 23 A at +15 mΩ. With all paths the fast
path still warns +15 mΩ after 39 min (11-14) and 69 min (11-12).

Host test: sweeps every 2 min with 2 of every 5 or 20 % random sweeps +12 mΩ raise nothing (the old
code derates on the 20 % case); +15 mΩ at 15 A warns after 45 min, also with 20 % low outliers;
Ro-only monotonic over 0..100 mΩ; a repaired Ro-only derate releases; 1–2 sweeps after a reboot leave
`wf` unverified. With the simulator's 30-min sweeps the Ro-only +6/+15 mΩ steps take 120 instead of
60 min (3 sweeps to move the median, then 3 graded values).

## Learning across restarts and odd first days (2026-11-20)

Host-tested and replayed, not deployed.

### Field finding

fry, 2026-11-20: `wf` = 1 for about 70 minutes, `Pw` 6.2–6.8 W from the `Rc` (slope) path, no real
fault. The `wire` command showed `baseline: 1 days (learning), Rc 3.13 a 25.7mV Rb 5.00 Ro 20.16
mOhm` and `Rc fresh dR 15.45 mOhm Pw 3.47 W` at `Iref` 15 A. The true `Rc` is about 15–18 mΩ (11-08
field fit about 15 mΩ). The baseline had been one day since at least 11-17.

Three causes:

1. **Restarts discarded the open day.** The day accumulators lived in RAM, and a day closed only at
   the first hour without conversion 12 h after the previous close *or boot* (or after 36 h). The
   console log shows 116 restarts of fry between 11-12 14:15 and 11-20 14:00 (boot stamps from
   uptime; deduplicated within 2 min): the 20 OTAs plus bench sessions and reboot loops, e.g. 50 on
   11-13 17:15–23:50, one every 2.5 min on 11-14 10:01–10:59, 13 on 11-18. After the first close
   (11-13 ~02:48, the afternoon of 11-12) no further day closed.
2. **That one day was poor.** Slow-fit samples were taken whenever the fit was valid, which includes
   the hour after the last BMS frame. The 11-12 day had 5 samples, all from the end of a partial
   afternoon; the replay gets 5.30 mΩ for it (the device 3.13 mΩ).
3. **Learning could not outvote it.** `closeDay` rejected a day whose median exceeded baseline +
   `drMin` also while learning (`accept()` already let everything through while learning, the
   `drMin` check and the fast-path veto did not). Every correct day (~12–15 mΩ) was rejected against
   a 1-day baseline.

### Design

- **Open day in RTC RAM.** The day accumulators moved into `WireGuard::Day` (200 B). `main.cpp`
  copies it every 10 s into `RTC_NOINIT` RAM (`DayRtc`, 208 B: magic with `sizeof(Day)`, FNV-1a
  checksum) and `wireGuardLoad()` restores it at boot, the same pattern as `ChargerRtc`. Times are
  stored as ages, since the guard's clock is uptime and restarts at 0. The restore checks magic,
  checksum and ranges (counts ≤ `DAY_N`, `drSkip < drStep`, step ≥ `DAY_SAMPLE_S`, age ≤ 72 h).
  RTC RAM survives `esp_restart`, panic, task and interrupt watchdog and OTA (if the `Day` layout is
  unchanged; a changed layout changes the magic), but not power-on or brown-out.
  Why not NVS: restarts happen during the day while converting (OTA, crashes), and an NVS write
  stalls the RT loop for 1 ms (append) to 33–65 ms (erase/compaction); RTC RAM costs no flash write
  at all. The NVS blob is still written only after a day closes, with the converter off.
- **Cold start.** Without a restored day (power-up), the first hour without conversion closes the
  day as soon as it counts toward learning, without the 12 h. A midday pause of an hour after a cold
  start can close a partial day once; accepted.
- **Learning.** While `days < LEARN_DAYS`, every good day enters the `Rc` and `Ro` rings (no
  `drMin` check, no fast-path veto), so the median outvotes an odd first day after two more good
  days. After learning the rejection and the anchor are unchanged.
- **Quality gate.** `Rc` (`rcGood`): ≥ `DAY_RC_N` = 4 slow-fit samples, now taken only while
  `Iout` ≥ `I_MIN`; ≥ `DAY_W_MIN` = 300 BMS frames at `Iout` ≥ `I_MIN`; variance of `Iout` among
  those frames ≥ `DAY_VAR_MIN` = `OFS_VAR_MIN` = 4 A². `Ro` (`roGood`): ≥ 2 sweeps (unchanged). A
  day counts toward learning (`dayCounts`) when `rcGood`, or, on a board without any BMS frame that
  day and without an `Rc` baseline, when `roGood`. Reasons: the slow fit separates offset from
  resistance only through the current spread, the same criterion as the offset regression; the
  offset drifts by tens of mV within a day (section of 2026-11-15), so at σ(Iout) = 2 A (range about
  7 A) a 20 mV drift moves the slope by about 3 mΩ, below `drMin`, at 1 A by about 6 mΩ. 4 samples at
  15 min need 45 min of charging with a valid fit; 300 frames are about 25 min of frames. On fry
  11-04..11-20 the good days had σ 2.1–5.5 A; the gate dropped 11-06 (1 sample while charging),
  11-10 (σ 2.0 A), 11-11, 11-17 (2 samples, σ 1.1 A) and 11-18 (`Iout` ≤ 4 A).
- Consequence: a converter whose daily peak stays below about 11 A (σ < 2 A) never finishes
  learning and stays warn-only (host test: counted days over 3 days by peak current 4..28 A in 2 A
  steps: `0000333333333`).
- Blob layout unchanged (`VER` 1): a stored baseline loads as before.
- `wire` console line: today's `Iout` σ and whether the day counts.
- `sizeof(WireGuard)` 696 → 720 B (host and xtensa-esp32s3), RTC RAM +208 B, `Blob` 124 B.
  Firmware (esp32s3, IDF 6.0.3) 1 698 128 → 1 699 440 B (+1 312 B).

### Replay (fry, empty baseline from 11-12 14:00, data to 11-20 16:00)

`test/host-stub/wire-guard-replay.cpp -r` restarts the guard at the given times as the firmware
does: baseline from the last NVS save (`saveDue`, 600 s retry), open day from the RTC copy (where
the code has one; `-R` drops it), slow fit, sweeps and timers lost, uptime clock. Level minutes as
with `wire_protect=2`.

With the 116 restarts from the console log (before → after):

| day | restarts so far | Imax [A] | days | baseline Rc [mΩ] | baseline Ro [mΩ] | max Pw fast / Rc / Ro [W] | minutes at warn |
|---|---|---|---|---|---|---|---|
| 11-12 | 2 | 21.2 | 0 → 0 | – → – | – → – | – / – / – → – / – / – | 0 → 0 |
| 11-13 | 50 | 22.9 | 1 → 0 | 5.30 → – | – → 19.17 | 2.97 / 0.70 / – → – / – / 0.65 | 0 → 0 |
| 11-14 | 79 | 24.6 | 1 → 2 | 5.30 → 11.14 | – → 19.17 | 0.83 / 0.86 / – → 0.85 / 0.51 / 0.45 | 0 → 0 |
| 11-15 | 88 | 23.8 | 1 → 3 | 5.30 → 11.66 | – → 20.33 | – / – / – → – / – / 0.88 | 0 → 0 |
| 11-16 | 90 | 20.5 | 1 → 4 | 5.30 → 11.75 | – → 20.81 | 1.59 / 1.28 / – → 0.55 / 0.18 / 0.95 | 0 → 0 |
| 11-17 | 91 | 9.1 | 1 → 4 | 5.30 → 11.75 | – → 20.46 | – | 0 → 0 |
| 11-18 | 105 | 4.0 | 1 → 4 | 5.30 → 11.75 | – → 20.81 | – | 0 → 0 |
| 11-19 | 110 | 22.0 | 1 → 5 | 5.30 → 11.83 | – → 20.46 | 2.42 / 1.85 / – → 2.53 / 2.02 / 0.32 | 0 → 0 |
| 11-20 | 116 | 20.7 | 1 → 5 | 5.30 → 11.83 | – → 20.46 | 5.94 / 5.43 / – → 1.11 / 1.01 / 0.79 | 47 → 0 |

"Before" reproduces the field state: one learned day, a low `Rc`, and a warning on 11-20 from the
`Rc` path (47 min, 5.4 W; device about 70 min, 6.2–6.8 W).

Summary of the other runs (days learned by 11-20 / final baseline Rc, Ro [mΩ] / minutes at warn):

| restarts | before | after |
|---|---|---|
| none | 8 / 13.46, 20.46 / 0 | 5 / 12.69, 20.46 / 0 |
| 20 OTAs only | 5 / 12.68, 20.10 / 241 | 5 / 12.70, 20.46 / 242 |
| 116, RTC copy lost each time (power loss) | 1 / 5.30, – / 47 | 1 / 15.88, 19.96 / 240 |
| none, from 11-04 (empty baseline) | 16 / 14.33, 21.24 / 0 | 9 / 14.03, 21.24 / 0 |

- The OTA-only run warns on 11-19 for about 4 h in both versions: fast path 6.3 mΩ, `Rc` path
  5.7 mΩ at 20–21 A (2.5 W). The regression slope ran 1–6 mΩ above the baseline that day; it
  depends on when the regression restarted (without the 11-19 OTAs the warning lasts longer, without
  any restart there is none). Not a learning issue; the regression is not kept across a restart.
- With the RTC copy lost on every restart, only 11-16 counts (power loss is not what happened on
  fry; every reset reason in the log, from `bootinfo` replies, was SW, PANIC or INT_WDT).
- From 11-04 the gate makes learning slower (ends 11-15 instead of 11-10) and drops the days listed
  above; the baselines agree within 0.5 mΩ.
- The `Rc` baseline from 11-12 on (11.8–12.7 mΩ) is lower than from 11-04 on (13.8–15.2 mΩ): the
  slow fit's `Rc`/`Rb` split differs between those weeks.

### Host test

New cases in `test/host-stub/wire-guard-test.cpp` (all pass): a fry-like blob (1 day, `Rc`
3.13 mΩ) followed by correct days converges to 13.97 mΩ after two good days and warns no more from
day 3, still learning; a dim first day (7 A peak) and 5.4 A days don't enter the baseline; counted
days are monotonic in the peak current; after learning a +10 mΩ fault stays out of both baselines
for 5 days and holds the level; restarts every 3 h, every 2.5 min for 2 h a day (RTC kept) and at
10:00 and 13:00 with the RTC copy lost all learn 10 days to the right baseline; restarts every 3 h
with the RTC copy lost learn nothing (documented); a restored day closes exactly as without the
restart; a corrupt, foreign or zeroed RTC copy is rejected.

### Guard checklist

1. Unevaluable input: a day below the gate neither enters a baseline nor counts toward learning, so
   `learning()` stays true and the guard stays warn-only; `wire` shows the day's σ and whether it
   counts. An unreadable RTC copy is rejected and starts an empty day.
2. Monotonicity: more samples, frames or current spread can only turn a day good; counted days over
   peak current 4..28 A are non-decreasing. Fault grading is unchanged (existing monotonic tests).
3. Preconditions: `restoreDay()` runs in `wireGuardLoad()` before the first tick; the RTC copy is
   refreshed every 10 s from the same core-0 loop that ticks the guard. Checked in the build (symbol
   `s_wireDayRtc`, 208 B, in RTC slow memory) and in the host replay with the logged restart times.
4. Source of truth: the magic includes `sizeof(Day)`, so a firmware with another layout does not
   restore a foreign copy.
5. Persistence: a restored day only adds data; a failed restore cannot mark anything verified.
   `unvDays` still counts only at a day close.
6. Provenance: `wire` reports days, learning and the day's gate inputs; the boot log says when an
   open day was retained.
7. Known-bad calibration: the 1-day 3.13 mΩ blob warns on day 1 (precondition checked), then
   converges; the real-data replay reproduces the field warning before and removes it after; a
   fault after learning is still rejected and warns.
8. Fix vs mute: the warning disappears because the baseline converges to the measured `Rc`, not
   because a threshold moved. Learning-time warnings still occur while the baseline is wrong.

### Deploy note

The blob layout is unchanged, so fry keeps `days 1, Rc 3.13`. Without `wire-reset` the next good day
gives a 2-day median of about 8 mΩ (the slope path can still warn above about 18 A), the second
good day outvotes it. `wire-reset` after the update drops the bad day at once; it also discards the
open day, so it is best done before the day's charging starts.

## Grading only against a baseline of 3 stored days (2026-11-22)

Host-tested and replayed, not deployed.

### Field finding

While learning, `closeDay` stores a day's `Ro` median whenever the day has ≥ `DAY_RO_N` = 2 sweeps,
also when the day does not count toward `days`. `updateBase` then sets `bRo` to the ring median, so
after one afternoon the `Ro` path graded against a 1-day baseline (fry: 20.1 mΩ). The 11-20 false
warning came the same way through the `Rc` path (1-day `Rc` 3.13 mΩ).

### Change

- `BASE_MIN_DAYS` = 3. `rcBase()` is NAN until `nRc` ≥ 3, unless `rMax` is set; then it is `rMax`
  alone (not `min(bRc, rMax)`, which would bring back the 1-day value). `roBase()` is NAN until
  `nRo` ≥ 3. A NAN baseline leaves `slow()` without a value, so the path stays `has = false`, and
  `roDown` grades NAN to `Ok`.
- Fast path: the same bug. Its excess drop is `y − rcBase()·i` with `y` the drop above the
  regression intercept, i.e. the regression slope against `rcBase()` minute by minute (the 11-20
  replay "before" had fast 5.94 W next to `Rc` 5.43 W). It goes through `rcBase()` and is gated with
  the `Rc` path. The step test of the regression no longer uses `rcBase()` (`|y − r·i|`, the same
  value), so regression learning is unchanged while the path is ungraded.
- `Rb` path: graded `rbIn − bRb` against the 1-day `bRb`; gated on `nRc` ≥ 3 (`rbRef()`), also with
  `rMax`, since `rMax` caps only `Rc`.
- Day warning fallback (no fast-path minutes): `median(d.ro) − roBase()`, so it also needs 3 `Ro`
  days. It only runs after learning.
- Not changed: `ofsFit` still subtracts the 1-day `bRb·ib` and checks `|a − bA|` against the 1-day
  `bA`. With `rMax` set and `nRc` < 3 the slope is therefore graded with a 1-day `Rb` correction; a
  `bRb` error moves the slope by about ΔRb·cov(ib, Iout)/var(Iout). Without `rMax` nothing is graded
  in that state.
- Day storage unchanged: every good day still enters the rings while learning.
- `rMax` as a reference: kept. It is the user's stated upper bound on `Rc`; grading against a value
  at or above the true `Rc` under-reports the excess (a later warning), it cannot raise a false one
  unless `rMax` is set below the real `Rc`, which is equally wrong after learning.
- `wire`: an ungraded path prints `stale, baseline n/3 days`.
- No new member, `Blob` and `VER` unchanged; `sizeof(WireGuard)` 720 → 720 B (host). Firmware
  (esp32s3, IDF 6.0.3) 1 719 760 → 1 719 792 B (+32 B).

### Host test

`testBaseMinDays`: a blob with one `Ro` day 5 mΩ low (Ro path only) and one with one `Rc` day 5 mΩ
low (fast + `Rc`), 28 A days: `wf` −1 on day 0, no level on days 0–2, referenced after 3 days, and
+10 mΩ on day 3 warns after 62 min (Ro) and 5 min (fast + `Rc`). `rMax` 16 mΩ with the low 1-day
`Rc`: no level on day 0, +10 mΩ on day 1 warns after 5 min. Against the old header all three clean
cases warn (known-bad). `testBadFirstDay` (1-day 3.13 mΩ) no longer warns on day 1; its convergence
checks are unchanged. The 3-sweep `Ro` case now builds a 3-day baseline first. All other results
unchanged.

### Replay (fry, empty baseline from 11-12 14:00, data to 11-20 14:36, 116 restarts)

- Clean: minutes at warn 0 → 0. Before, `Ro` was graded on 11-13 (1 day, 0.65 W) and all paths on
  11-14 (2 days); after, `Ro` from 11-15 and fast/`Rc` from 11-16.
- +6/+10/+20 mΩ injected 11-14 12:00, 11-16 12:00, 11-19 12:00: identical before and after
  (first warn 3108/2829/2786, 184/134/134, 114/134/114 min). For the 11-14 fault the old code
  had only 17 fast-path minutes that day (1 at warn) and `Ro` at 2.0 W, so it did not warn on its
  2-day baseline either.
- Without restarts, +10 mΩ at 11-14 12:00: first warn 1343 → 1529 min (+3 h), the cost of not
  grading on a 2-day baseline; at 11-15 12:00: 267 → 263 min.
- The live single-day `Ro` case (after 11-20) is not in this data set.

## Offset regression kept across restarts (2026-11-22)

Host-tested and replayed, not deployed.

### Finding

The regression sums `rg[7]` were lost on every restart. With fry's OTA restarts only (20 from
11-12 14:00 to 11-20 14:36, 3-day baseline gate on), the replay warned 242 min on 11-19
(12:59–17:01, fast 2.57 W and `Rc` 2.51 W at 6.3/5.7 mΩ): restarts at 09:59 and 10:12 left a
regression learned from that morning only, whose offset and slope were off. Without restarts and
with the 116 real restarts it does not warn (fast max 2.30 and 2.53 W).

### Change

- `FitRtc` (`float rg[7]` + FNV-1a seeded with `FIT_MAGIC`, 32 B) in a second `RTC_NOINIT` struct,
  written with the day copy every 10 s (`saveFit`), restored in `wireGuardLoad` (`restoreFit`).
  Rejected (cold regression) on a checksum mismatch, a non-finite sum or negative `rg[0]`/`rg[2]`.
  Power loss still starts cold.
- Stored as float: rounding is ~1e-7 relative; the worst cancellation is `var = rg2/w − mi²`
  (~400 A² minus ~400 A²), an error of ~4e-5 A² against `OFS_VAR_MIN` 4 A², and the slope moves by
  ~1e-7 Ω. Runtime sums stay double.
- `DayRtc`, its magic and checksum, `Blob` and `VER` unchanged, so the open day survives the update
  to this version. Level, histories and paths are not kept: after a restart a level must be
  re-raised (5 fast minutes).
- Firmware (esp32s3, IDF 6.0.3): 1 720 432 → 1 720 768 B (+336 B), DIRAM unchanged, RTC slow
  +32 B (noinit).

### Host test

`testFitRestore`: after a noon restart on day 10 the fast path returns after 1 min (cold: none
within 60 min, the noon current spread is too small); slope −0.837 vs −0.845 mΩ without the restart.
+6 mΩ 2 min before / after the restart warns after 7 / 5 min (cold: 122 / 118 min). Round trip,
corrupt sum, zeroed RAM and re-signed NaN/inf/negative sums checked. Other results unchanged.

### Replay (fry, empty baseline from 11-12 14:00, data to 11-20 14:36)

| restarts | before (min at warn) | after | fast Pw max 11-19 |
|---|---|---|---|
| OTA only (20) | 242 (11-19) | 0 | 3.06 → 2.29 W |
| real (116) | 0 | 0 | 2.53 → 2.38 W |
| real, RTC lost (`-R`) | 0 | 0 | – |

`-F` (day kept, regression dropped) reproduces the old result exactly; without restarts the output is
byte-identical. Injected faults, first warn [min] before → after (OTA / real restarts):

| fault | +6 mΩ | +10 mΩ | +20 mΩ |
|---|---|---|---|
| 11-14 12:00 | 1529→1529 / 3109→3109 | 1529→1529 / 2829→2867 | 1529→1529 / 2786→2829 |
| 11-16 12:00 | 4364→92 / 185→229 | 90→87 / 135→89 | 87→54 / 135→54 |
| 11-19 12:00 | 158→8 / 114→5 | 134→8 / 134→5 | 114→8 / 114→5 |
| 11-19 10:10, restart 10:12 | 124→48 / 124→41 | 124→12 / 124→7 | 124→9 / 124→7 |
| 11-20 12:03, restart 12:13 | 72→72 / 71→71 | 69→21 / 67→20 | 43→21 / 42→6 |
| 11-20 12:10, restart 12:13 | 65→65 / 64→64 | 62→14 / 60→13 | 36→14 / 35→13 |
| 11-19 16:45 (evening) | 0*→1230 / 1229→1229 | 0*→1230 / 1096→1155 | 0*→1010 / 1017→1012 |

\* inside the old false warning. Later in 4 of the 21 real-restart cases (none of the OTA-only ones) (+38 to +59 min): the 11-14
fault (no 3-day baseline until 11-16), +6 mΩ on 11-16 (2.2 W against `wire_p_warn` 2 W, found by
`Ro` instead of `Rc`), +10 mΩ on 11-19 16:45 (next morning). In those the old code was helped by a
restart rebuilding the regression from post-fault data, so the `Rc` slope saw the step; a kept
regression, like one never restarted, holds through the fault (step minutes don't enter it) and
leaves it to the fast path. The no-restart times are 1529, 92 and 1129 min.

## RTC copy at a fixed address (2026-11-22)

### Finding

The OTA 792 → 804 lost the open day although `DayRtc` was unchanged: adding `s_wireFitRtc` moved
`s_wireDayRtc` from 0x50000004 to 0x50000024 (`.rtc_noinit` is filled in link order), so the new
image read the wrong bytes and the checksum failed. Any RTC variable added, removed or reordered in
any component shifts it again; so does `RTC_DATA_ATTR` data, which the linker places before
`.rtc_noinit` in the same region.

### Change

- One struct `WireGuard::Rtc`: `magic` (`RTC_MAGIC`, layout version), `FitRtc`, `DayRtc`. Per-part
  magic and checksum unchanged; a part that fails starts cold alone. New parts go at the end. `Fit`
  comes first because a `Day` size change (`DAY_N`) would move everything after it.
- `main/wire_rtc.ld` (added with `target_linker_script`, no IDF file patched) places input section
  `.rtc_wire` as a `NOLOAD` output section at `ORIGIN(rtc_slow_seg) + LENGTH(rtc_slow_seg) − 0x200`:
  0x50001E00 on the S3, 0x50001DE8 on the classic ESP32 (below its reserved timer/bootloader
  block). The linker fills RTC slow memory from the bottom (`.rtc.data`, `.rtc.bss`,
  `.rtc_noinit`, `.rtc.force_slow`), so nothing it places can move the top. Two `ASSERT`s fail the
  link if `_rtc_force_slow_end` reaches the block or the block outgrows 0x200 B (both checked by
  forcing them).
- Rejected: an ldgen fragment or `SECTION_MAPPINGS(rtc_noinit)` puts it first in `.rtc_noinit`,
  which still moves with RTC data/bss; enabling `CONFIG_ULP_COPROC_ENABLED` to reserve the bottom
  turns on an unrelated feature.
- Failure modes: the address moves only if the end of `rtc_slow_seg` moves (classic ESP32:
  `CONFIG_BOOTLOADER_RESERVE_RTC_SIZE`; S3: none, its reserved block is in RTC fast memory) or the
  0x200 is changed; the copy then fails its checks once and starts cold. A large ULP reserve or
  RTC data growth to ~7.5 KB fails the link instead of overlapping. Moving RTC data to fast memory
  (`CONFIG_ESP32S3_RTCDATA_IN_FAST_MEM`) does not touch the block.
- Migration from 804: a temporary `RTC_NOINIT` struct `s_wireRtcOld` (Fit + Day, 240 B) takes the
  place of the two old variables and lands at 0x50000004 (nm), so `s_chargerRtc` (0x500000F4),
  the flash-probe state (0x50000108) and `.rtc.force_slow` (0x50000128, reloaded by the
  bootloader on every non-deep-sleep reset) keep their 804 addresses too. While the new header is
  not valid, `wireGuardLoad` restores from it, validated by the existing magic and checksums; the
  first 10 s save writes the header. Drop `s_wireRtcOld` once no device runs 804; that update
  shifts `s_chargerRtc` and the flash-probe state (both fail safe).
- Firmware (esp32s3): `fugu-firmware.bin` 1 722 016 B unchanged (`.flash.text` +8 B), `.rtc_noinit`
  unchanged (0x128), `.rtc_wire` +244 B; net +4 B RTC once the migration struct is dropped.
- nm, base: `s_wireFitRtc` 0x50000004, `s_wireDayRtc` 0x50000024. New: `s_wireRtc` 0x50001E00 (244 B);
  with dummy `RTC_NOINIT_ATTR` arrays in `main.cpp` (52 B) and `util.cpp` (28 B) plus an
  `RTC_DATA_ATTR` array (20 B): `s_wireRtcOld` 0x50000004 → 0x5000004C, `s_wireRtc` still 0x50001E00.
- Host test passes; the replay is byte-identical to main (OTA and real restarts, c1..c9 data).
