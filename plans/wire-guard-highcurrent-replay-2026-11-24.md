*this document is an LLM generated placeholder*

# Wire guard on fry's high-current days

Answers the open item in `plans/wire-fault-protection.md` ("Field check of the new fast path over
high-current days before `wire_protect=2`") and its follow-up 2 (the apparent +3.4–4.5 mΩ at high
current on 11-04).

## Fast-path replay over high-current days, 2026-11-24

### Setup

- Code: `refs/heads/main` at 954dcb63 (`src/wire_guard.h`, `src/out_impedance.h`,
  `test/host-stub/wire-guard-replay.cpp`), exported to a scratch dir, built with
  `clang++ -std=gnu++20 -O2`. Unchanged except for a scratch copy of the replay that also writes
  every fast-path minute to a CSV (`-m`, for the persistence statistics below).
- Data: `etc/wire_replay_fetch.py`, fry, 2026-10-25 03:17 (first fry point in InfluxDB) to
  2026-11-24 12:54 UTC, fetched in 4 chunks (merged by the replay). 2.12 M converter seconds,
  623 k BMS frames, 17.6 M sweep samples.
- Restarts: 138 boot times from fry's lines in `pv/fugu_console.log*` on havan (11-11 17:14 on;
  a new boot = ESP log uptime < 300 s with the derived boot time > 2 min after the previous one).
  For 11-12 14:15..11-20 14:00 this gives 111, the earlier write-up counted 116.
  No restart data before 11-11.
- Levels are `level()`, i.e. what `wire_protect=2` would act on once learning has ended.

Runs:

| run | baseline | restarts | learning ends | final baseline Rc / a / Rb / Ro |
|---|---|---|---|---|
| A natural | empty from 10-25 | none | 11-12 | 14.03 mΩ / 20.8 mV / 1.56 / 20.97 mΩ (13 days) |
| B primed (`-p`) | learned over all data, armed from the start | none | – | 13.79 / 38.6 / 1.56 / 20.97 |
| C natural | empty from 10-25 | 138 real | 11-12 | 15.11 / 18.9 / 1.74 / 20.97 (15 days) |
| E primed | as B | 138 real | – | 13.95 / 36.3 / 1.64 / 20.97 |
| D device | empty from the `wire-reset` at 11-20 14:58 | real | not yet | 15.60 / 70.1 / 0.82 / 21.00 (2 days) |

Run D reproduces the device: its `wire` line since 11-23 19:31 reads `baseline: 2 days (learning),
Rc 14.14 a 83.0mV Rb 0.63 Ro 20.19 mOhm`.

### Day selection

Ranked by minutes with a 1-minute mean `Iout` above 15 A (InfluxDB, fry): 11-04 (245), 11-19
(232), 11-22 (187), 11-16 (103), 11-13 (100), 11-06 (70), 11-20 and 11-23 (57 each), 11-12 (56),
11-14 (36). Above 20 A: 11-22 58 min, 11-19 19, 11-14 12. Highest daily peak 25.4 A (11-14). No
day before 11-03 exceeds 14 A. The table holds those days plus every day from 11-15 on.

### Per-day results

Fast path from run B (armed, no restarts) unless noted. "5-min Pw" is the highest Pw that 5
consecutive fast minutes all reached with ΔR ≥ 4 mΩ, i.e. the persistence-relevant value (a level
needs 5 consecutive graded minutes). ΔR med is the median fast-path ΔR over minutes at ≥ 15 A.

| day | Imax [A] | min >15/>20 A | charging / fast min | fast Pw max [W] (ΔR mΩ @ A) | fast Pw max E / C [W] | ΔR med ≥15 A [mΩ] | min ≥ warn / longest run | 5-min Pw [W] | Rc / Ro path Pw max [W] | level | day warn | device Pw max [W], wf>0 | device Rc / Ro median [mΩ] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 11-04 | 21.2 | 245 / 0 | 419 / 261 | 1.34 (3.8 @ 18.7) | 1.34 / 0.60 | 0.27 | 0 / 0 | 0 | 0.86 / 0.68 | 0 | 0 | – | – |
| 11-06 | 23.6 | 70 / 1 | 296 / 192 | 1.11 (3.5 @ 18.4) | 1.11 / 0.84 | 0.24 | 0 / 0 | 0 | 0.35 / 0.49 | 0 | 0 | – | 14.6 / – |
| 11-12 | 24.2 | 56 / 4 | 289 / 68 | 0.74 (1.7 @ 21.0) | 0.65 / 0.46 | 0.21 | 0 / 0 | 0 | 0.50 / 0.36 | 0 | 0 | –, 0 | 9.4 / 20.4 |
| 11-13 | 23.7 | 100 / 3 | 303 / 137 | 1.19 (3.8 @ 17.6) | 1.10 / 0.75 | 0.24 | 0 / 0 | 0 | 0.07 / −0.32 | 0 | 0 | 3.22, 0 | 10.5 / 19.3 |
| 11-14 | 25.4 | 36 / 12 | 241 / 123 | 1.10 (2.2 @ 22.6) | 1.18 / 0.42 | −1.05 | 0 / 0 | 0.13 | −0.22 / 0.06 | 0 | 0 | 6.66, 0 | 10.4 / 21.1 |
| 11-15 | 24.3 | 6 / 1 | 86 / 55 | 0.02 | −0.03 / −0.07 | −2.56 | 0 / 0 | 0 | −0.72 / 0.70 | 0 | 0 | 6.89, 0 | 11.3 / 21.7 |
| 11-16 | 20.7 | 103 / 7 | 308 / 171 | 0.48 (1.4 @ 18.3) | 0.53 / 0.17 | −0.66 | 0 / 0 | 0 | −0.17 / 0.62 | 0 | 0 | 1.52, 0 | 14.4 / 20.2 |
| 11-17 | 9.8 | 0 / 0 | 8 / 2 | −0.08 | 0.07 / 0.02 | – | 0 / 0 | – | held | 0 | 0 | –, 0 | 13.0 / 18.1 |
| 11-18 | 4.5 | 0 / 0 | 0 / 0 | – | – | – | – | – | held | 0 | 0 | –, 0 | 14.8 / 20.5 |
| 11-19 | 22.3 | 232 / 19 | 368 / 201 | 1.77 (4.4 @ 20.0) | 1.89 / 1.28 | 2.37 | 0 / 0 | 0.90 | 1.17 / 0.20 | 0 | 0 | 2.09, 0 | 14.1 / 20.7 |
| 11-20 | 21.0 | 54 / 1 | 362 / 305 | 1.62 (6.9 @ 16.5) | 1.42 / 1.17 | −1.11 | 0 / 0 | 0.28 | 0.90 / 0.77 | 0 | 0 | 6.80, 10 497 pts | 15.2 / 20.0 |
| 11-21 | 23.7 | 5 / 0 | 81 / 40 | 0.17 | 0.17 / 0.11 | 0.19 | 0 / 0 | 0 | 0.57 / 0.11 | 0 | 0 | 0.13, 0 | 11.3 / 18.4 |
| **11-22** | 24.8 | 187 / 58 | 407 / 202 | **3.40 (7.6 @ 21.4)** | 3.58 / 2.89 | **4.08** | **7 / 2** | **1.66** | 1.97 / 0.80 | 0 | 0 | 0.71, 0 | **19.6** / 20.4 |
| 11-23 | 19.8 | 57 / 0 | 229 / 169 | 0.82 (4.2 @ 13.6) | 1.01 / 0.74 | 1.36 | 0 / 0 | 0 | 0.80 / 1.00 | 0 | 0 | –, 0 | 10.4 / 21.6 |
| 11-24 (to 12:54) | 22.4 | 16 / 3 | 68 / 48 | 0.50 (2.3 @ 15.3) | 0.39 / 0.11 | −0.38 | 0 / 0 | 0 | 0.14 / 0.46 (E: Ro 2.04) | 0 | 0 | 0.83, 0 | 11.7 / 21.8 |

Totals over 10-25..11-24, every run (A, B, C, E): minutes at level ≥ 1: **0**; max level 0; day
warnings 0; shared-section (`Rb`) warnings 0. Fast-path Pw max: A 3.29, B 3.40, C 2.89, E 3.58 W,
all on 11-22. Fast minutes with Pw ≥ 2 W and ΔR ≥ 4 mΩ: A 3, B 7, C 1, E 3, never more than 2 in
a row. Before 11-04 (10-25..11-03, ≤ 15.6 A) the fast path stays below 0.9 W.

### Margin against the thresholds

| quantity | worst value (day) | threshold | margin |
|---|---|---|---|
| fast Pw, single minute | 3.58 W (11-22, run E) | derate 4 W, trip 8 W | 0.4 W / 4.4 W |
| fast Pw, 5 consecutive minutes with ΔR ≥ 4 mΩ | 1.66 W (11-22) | warn 2 W | 0.34 W |
| fast ΔR, median at ≥ 15 A | 4.08 mΩ (11-22) | floor `wire_dr_min` 4 mΩ | none |
| day median ΔR | 4.18 mΩ (11-22) | day warning 8 mΩ | 3.8 mΩ |
| `Rc` / `Ro` path Pw | 1.97 / 2.04 W | warn 2 W (30 / 3 graded values) | none sustained |

The ΔR floor is met on 11-22; what kept the fast path from warning was the current: 4–5 mΩ at
17–22 A is 1.2–2.3 W, and the minutes above 2 W did not line up 5 in a row.

Extrapolated with the 11-22 pattern (ΔR ≈ 4.0–4.5 mΩ held for hours): at 25 A 2.5–2.8 W, a
sustained **warn**; at 30 A 3.6–4.1 W, at the **derate** threshold. A derate from this pattern caps
`Iout` at √(4 W / 4.5 mΩ) ≈ 30 A, so the over-derate trip (5 W after derating) and the 8 W trip are
out of reach. fry's peak in this data is 25.4 A; its summer peak is not in InfluxDB (data starts
10-25).

### The 11-04 offset reappearing: 11-22

11-22 is the only day where the high-current apparent resistance comes back, and it is the
day with the most high-current minutes (58 min above 20 A). Fast path 10:44–15:00 UTC: ΔR 4–5 mΩ at
17–22 A, regression slope 2.6–3.9 mΩ above `Rc_base`, intercept −26..−48 mV against the
baseline offset. The device's own slow fit saw it too: telemetry `Rc` median 19.6 mΩ on 11-22 against
10–15 mΩ on the other days, `Rb` −0.38 mΩ.

It looks like the voltage-gain mismatch described in the 2026-11-15 section, not a resistance. A
per-day least-squares fit of `dv − 1.56 mΩ·Ib` over steady frames at ≥ 5 A:

| day | Vpack range [V] | dv = c + R·I: R [mΩ] | dv = c + R·I + k·Vpack: R [mΩ], k [mV/V] |
|---|---|---|---|
| 11-19 | 26.0–27.3 | 16.8 | 13.4, 72 |
| 11-22 | 24.7–27.2 | 14.8 | 12.0, 60 |
| 11-23 | 26.5–27.8 | 13.0 | 13.0, 36 |
| 11-24 | 26.5–27.5 | 12.8 | 12.1, 15 |

With a pack-voltage term the 11-22 resistance falls back to 12 mΩ. On 11-22 the pack started from
24.7 V and rose by 2.5 V during the current ramp, so 60 mV/V adds up to ~150 mV of offset that moves
with current. 11-19 (72 mV/V) shows the same thing at a smaller scale (fast Pw 1.8–1.9 W). The guard's
120-minute offset regression follows the intercept and only partly removes it. A Vout gain
calibration against the BMS, as the 2026-11-15 section suggests, would remove it where it starts.

### Unevaluable data

- **Unevaluable days:** 10-26, 10-27, 10-29, 10-30, 11-08, 11-11 and 11-18 have no fast-path
  minute (peak ≤ 7.7 A, mostly under 5 A); 11-17 has 2. Those are 8 of 31 days; none is a
  high-current day. The guard judges nothing on them, but it does not report them as `unverified`
  in the replay either: the `Rc`/`Ro` paths keep their last values because freshness counts
  charging minutes (design, section 7). The device does report them: telemetry `wf` −1 all day
  on 11-17, 11-18 and 11-23.
- **Partly evaluable:** fast minutes are 50–85 % of charging minutes (≥ 5 A) on good days, less
  where BMS frames were missing: 11-12 68 of 289 (124 charging minutes without a BMS voltage
  frame), 11-09 127 of 354 (108), 11-04 261 of 419 (83). Steady-current gating (|ΔI| < 0.5 A)
  removes most of the rest, so cloudy days (11-15, 11-21) are thin: 55 and 40 fast minutes.
- **Telemetry gaps** in the replay input during charging: 11-12 15:45 40 min, 11-20 09:48 14 min
  (12 A), 11-22 10:31 10 min (16 A); otherwise ≤ 8 min. The device saw those minutes, the replay
  did not.
- **Restart list** is reconstructed from the console log (111 against the 116 counted before for
  the same window) and is missing before 11-11.

### Live telemetry cross-check

- The device has not graded the fast path since the `wire-reset` on 11-20 14:58: it has 2 `Rc`
  days (`BASE_MIN_DAYS` = 3), so its `Pw` since then (≤ 0.83 W) comes from the `Ro` path only.
  11-21 did not count (σ(Iout) 1.6/3.5 A, no slow-fit samples while charging); run D gives the
  same day count and a similar baseline (Rc 15.60 against 14.14 mΩ, a 70 against 83 mV).
- The device's 2-day `Rc` baseline includes 11-22 (one-day value 17.74 mΩ, replay 20.23 mΩ), the
  inflated day above. While learning every good day enters, so it will be outvoted by later days.
- 11-13..11-15 and 11-20 device `Pw` up to 6.9 W are the known pre-fix results (one-afternoon
  baseline, then the 1-day `Rc` 3.13 mΩ baseline; 11-20 `wf` = 1 for 10 497 points). Current main
  replays the same days at ≤ 1.9 W.
- Discrepancy: on 11-23 the device reported `wf` −1 all day, while run D graded the `Ro` path
  (0.72 W). Probably 11 restarts that day left fewer than `RO_MED_MIN` sweeps after some boots;
  the replay's restart times may differ. Not resolved.

### Verdict

On fry's data from 10-25 to 11-24 (peak 25.4 A), current main with `wire_protect=2` would not
have warned, derated or tripped in any of the four replay variants: armed or learning, with or
without the real restarts. The only close day is 11-22: a single minute reached 3.4–3.6 W
against the 4 W derate threshold, the persistent value was 1.66 W against 2 W warn, and the
median ΔR sat right on the 4 mΩ floor. Seen as false trips, `wire_protect=2` is safe for fry at
November currents. A false warning (no action) is plausible on a 25 A day with an 11-22-like pack
voltage swing. A false derate would need about 30 A with that pattern. A false trip is not
reachable from it.

Two conditions apply before switching:

1. Setting `wire_protect=2` now changes nothing yet: the device is learning (2 counted days since
   the 11-20 `wire-reset`; levels are capped at warn until 7). The earliest it arms is about 5
   good days from now; dull days like 11-21 don't count.
2. The 11-22 gain effect grows with current and with the pack-voltage swing. Recheck the
   fast-path margin on the first days above ~27 A, or calibrate Vout gain against the BMS first.

Known-bad check on the same data (primed, injected series resistance from 09:00): +6 mΩ on 11-22
warns after 70 min and derates after 234 min; +10 mΩ warns after 61, derates after 135 and trips
after 234 min. On 11-19, +6 mΩ warns after 137 min; +10 mΩ warns after 93 and derates after
183 min. So the data does not blind the guard. The faster 11-22 derate (+6 mΩ) comes from that
day's +4 mΩ apparent offset adding to the fault.
