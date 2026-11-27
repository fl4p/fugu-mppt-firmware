*this document is an LLM generated placeholder*

# Ripple comb field trial (#37): fry (`ripple_filter=both`) vs flat (notch only)

Analysis run 2026-11-25 08:30–08:50 UTC. Read-only: no commands were sent to fry or flat, and no configs were changed.
Lisbon time = UTC.

## Data and method

- **Telemetry:** InfluxDB `ha_van/mppt`, 2026-11-12 to 2026-11-25 08:31 UTC, aggregated per minute and device
  (about 47 points/s per device): `mean(P)`, point count, the count of points with `cv_lim_idx=1`, and counts of
  `mppt_state` values.
- **Console log:** havan `pv/fugu_console.log` plus `.1`–`.10` (2026-11-13 22:55 to 2026-11-25 08:43). Used for the
  `st=` state, command acks (`OK: dc|psu|measure-coil|sweep|mppt|ota|restart|rect-cal force|...`), `Rebooting`,
  `ota: Download`, measure-coil output and despike WARNs. `elf-archive/ota-owners.jsonl` gives the OTA times.
- **Battery:** `batmon` `bat_caravan` current, mean per minute (same InfluxDB query as `datasets.batmon()`; the module
  itself does not import here because `bmslib` is missing). A **load minute** means pack current < -15 A, i.e. the
  inverter is running.
- **Encodings** (from `src/mppt.cpp` and `src/mppt.h`):
  - `mppt_state`: 0 N/A, 1 CV, 2 CC, 3 CP, 4 MPPT, 5 SWEEP. It is sent on every 20th point.
  - `cv_lim_idx` is the minimum `limIdx` reached during the telemetry interval, and is only present when a limiter
    was active. The indices are 0 Vin, **1 Vout (the cap)**, 2 Iin, 3 Iout and 4 power.
  - A **cap minute** is one where either device has `cv_lim_idx=1` on more than 5 % of its points, or
    `mppt_state=1` on more than 30 % of its state samples. Every other minute is a **tracking minute**.
- **Exclusions** (applied to both devices whenever either one qualifies):
  - `st=MANU` or `PSU`, padded -2/+10 min.
  - State-changing commands (`dc`, `psu`, `measure-coil`, commanded `sweep`, `mppt`, `bf`, `sync`, `restart`, `ota`,
    `rect-cal force`, `sleep`, `±N`), padded -2/+10 min (`sweep` and `sleep` +5 min). `pwm-freq` and
    `rect-cal status` are periodic reads and are kept.
  - Reboots, OTA downloads and every `ota-owners` record (-5/+10 min).
  - Telemetry gaps: fewer than 1400 points/min on either device.
  - Daytime only: either device above 3 W.
- **Ratio:** E_fry / E_flat over the minutes that pass, `sum(P)/60` in Wh.

## 1. Baseline (before the trial)

| day | valid min | E_fry Wh | E_flat Wh | **ratio** | cap min | cap ratio | track ratio | load min (track) | load ratio |
|---|---|---|---|---|---|---|---|---|---|
| 11-12 | 756 | 1463 | 3061 | 0.478 | 113 | 0.26 | 0.534 | 6 | – |
| 11-13 | 859 | 1684 | 3105 | 0.542 | 19 | 0.77 | 0.533 | 0 | – |
| 11-14 | 324 | 577 | 860 | 0.672 | 140 | 1.22 | 0.560 | 18 | 0.29 |
| 11-15 | 351 | 441 | 589 | 0.748 | 0 | – | 0.748 | 14 | 0.71 |
| 11-16 | 507 | 1778 | 2617 | 0.679 | 103 | 0.50 | 0.733 | 0 | – |
| 11-17 | 484 | 255 | 374 | 0.681 | 1 | – | 0.681 | 42 | 0.66 |
| 11-18 | 353 | 221 | 332 | 0.665 | 0 | – | 0.665 | 14 | 0.66 |
| 11-19 | 442 | 2522 | 3258 | 0.774 | 48 | 0.83 | 0.765 | 0 | – |
| 11-20 | 504 | 1449 | 2221 | 0.652 | 53 | 0.27 | 0.725 | 200 | 0.80 |
| 11-21 | 457 | 503 | 673 | 0.748 | 0 | – | 0.748 | 5 | – |
| 11-22 | 490 | 2217 | 3058 | 0.725 | 43 | 0.89 | 0.697 | 0 | – |
| 11-23 | 228 | 437 | 987 | 0.443 | 45 | 0.42 | 0.459 | 0 | – |
| 11-24 before 14:13 | 179 | 311 | 403 | 0.770 | 0 | – | 0.770 | 17 | 0.60 |

The reference set is 11-15 to 11-22, eight days. 11-12/13 are excluded because the hourly ratio was 0.05–0.35 in the
morning, a different regime before the 11-14 changes. 11-23 is excluded because flat was in MANU 11:46–16:26, which
leaves mostly morning minutes, and fry's morning ratio is low.

- Full-day ratio: **0.709 ± 0.045** (sd, n=8), range 0.652–0.774.
- Tracking-only ratio: **0.720 ± 0.036**, range 0.665–0.765.
- Cap ratio: 0.27–1.22. It swings this much because at the cap the battery sets the total and the two converters
  share it by their own Vout readings. fry reaches its Vout limiter far more often than flat: on 11-23, fry was capped
  for 254 min and flat for 89. flat's Vout reads 0.3–0.9 V low while converting (known). The cap ratio is therefore a
  sensor-offset sharing number, not an efficiency, and is useless as a baseline for a few-percent effect.
- Load (inverter) minutes while tracking: 0.60–0.80 on the few days that have them.
- The ratio depends strongly on time of day: 0.2–0.6 at 07–08 h, 0.75–0.89 at 11–13 h, 0.5–0.7 at 15–16 h. The panels
  point in different directions, so any part-day comparison has to be matched by time of day.

### Today (11-25) manual and test periods

- **fry:** `st=MANU` 08:13:51–08:31:56 (measure-coil, `dc` steps). OTA to 1146-g8e8f497b at 08:31:56, reboot
  08:33:46, sweep, then MPPT from 08:34:09.
- **flat:** `st=MANU` 08:22:12–08:35:34 (`dc 0` and `mppt` at 08:33:55–59). OTA to 1146 at 08:37–08:40, back in MPPT
  from about 08:41.
- Both devices therefore came back to MPPT by 08:41, on 1146. The task's "~08:27" start is late for fry, which
  entered MANU at 08:13.
- fry still reports `ripple_filter='both'` (conf print at 08:31). flat's is unset.

## 2. Trial period (fry on the comb since 11-24 14:13)

This section is the 11-25 morning snapshot. For the full 11-25 day and the 11-26 morning, see
[Update 2026-11-26](#update-2026-11-26).

| window | valid min | ratio | cap min | load min | baseline, same window (n=8) | z |
|---|---|---|---|---|---|---|
| 11-24 14:13–17:30 | 94 | **0.599** (track 0.604) | 1 | 0 | 0.694 ± 0.055 (0.644–0.799) | -1.7 |
| same, minute-matched to the 94 trial minutes | 94 | 0.599 | – | – | 0.683 ± 0.041 (0.609–0.730) | -2.1 |
| 11-25 07:28–08:10 | 43 | 0.661 (07 h 0.74, 08 h 0.59) | 0 | 6 | 07 h 0.28–0.57, 08 h 0.22–0.66 | – |

- **fry's ratio did not rise.** In the 11-24 afternoon window it was lower than in all eight baseline afternoons;
  the closest is 11-16 at 0.609, minute-matched. The same day before 14:13 had a high ratio: 0.85–0.89 at
  10–11 h against a baseline of 0.74–0.77. That points to the weather, with partial cloud reaching the two arrays
  differently, rather than the filter.
- **Confounds in the trial window:**
  - The firmware differed. fry ran 964 (comb branch), then 1009 from 15:22. flat ran 958, then 1009 from 15:27, then
    1026 from 16:43.
  - flat ran measure-coil at 16:13 (excluded).
  - 62 of the window's minutes were lost to OTAs and reboots.
  - The two devices only ran the same build from 17:2x on (1027 and later), which was already dusk.
- **Cap and load events:** the trial so far has **one** cap minute and **no** valid load minutes. The battery sat at
  26.5–26.9 V, below the cap. The question the comb was built for, harvest at the Vout cap and with inverter ripple,
  has **not yet been tested in the field**.
- **Verdict:** no evidence of a gain. The afternoon ratio is 1.7–2.1 sd below the baseline, which points the other
  way, but it rests on one partial day with mismatched firmware and weather that likely differed. It cannot be told
  apart from day-to-day variation (sd 0.04–0.055 in a ratio of about 0.7, i.e. ±6–8 %). The sim's predicted cost
  without a cap is 1.5–7 %, smaller than this noise. Several full days with both devices on the same build are
  needed, ideally including cap and inverter hours, and the cap comparison is limited by the Vout offset sharing
  described above.

## 3. Despike WARNs (`clipped N of M sample(s) (k=8.0, mostly <ch>)`)

These are WARN lines per device, channel and day across all available log history. A line is emitted per window, only
when more than 0.5 % of the samples were clipped; flat's windows hold about 4700 samples. The line count is therefore
a threshold-crossing count: flat iout windows typically clip 26–40 of about 4700 (0.55–0.85 %), just above the
threshold.

| day | flat iout | flat vout | flat vin | flat iin | fry iout | fry vin | fry vout |
|---|---|---|---|---|---|---|---|
| 11-18 (despike on 14:38, pre WARN fix) | 4942 | 2643 | 6229 | – | 2361 | 1762 | – |
| 11-19 | 1639 | 146 | 274 | – | 9 | 849 | – |
| 11-20 | 881 | 110 | 2 | – | 270 | 102 | – |
| 11-21 | 57 | 358 | – | – | 77 | 109 | – |
| 11-22 | 188 | 90 | – | – | 24 | 24 | 4 |
| 11-23 | 52 | 31 | – | – | 744 | 133 | – |
| 11-24 | 1089 | 346 | 63 | 3 | 4 | 26 | – |
| 11-25 (to 08:37) | 90 | 11 | – | – | 0 | – | – |

On 11-18/19 there were also ntc WARNs, which were fixed by excluding temperature channels from despike.

### flat iout per hour on the days of interest (P = mean flat power)

| hour | 11-20 | 11-24 | 11-25 |
|---|---|---|---|
| 08 | 701 (142 W) | 467 (123 W) | 90 in 24 min (76 W, partly MANU) |
| 09 | 176 (410 W) | 138 (69 W) | |
| 10 | 4 | 159 (321 W) | |
| 11 | 0 | 7 | |
| 12 | 0 | 59 | |
| 13 | – | 92 | |
| 14 | – | 44 | |
| 15 | – | 27 | |
| 16 | – | 96 | |
| 17+ | – | 0 | |

### flat iout WARN rate by flat power (since 11-19 12:00, lines/h)

| flat P (W) | ≤3 | 3–25 | 25–50 | 50–100 | 100–200 | 200–300 | 300–500 | 500–1000 |
|---|---|---|---|---|---|---|---|---|
| WARN/h | 7 | 51 | 6 | 25 | **138** | **113** | 43 | 20 |

- Load minutes: 4.5 WARN/h, against 51/h without load.
- Cap minutes: 45/h, against 47/h off the cap.
- Excluded test and OTA minutes: 61/h, against 39/h in clean minutes.

### Verdict

- **Onset:** flat's iout WARNs are **not new**. They started with despike=8 on 2026-11-18 14:38 and have appeared
  every sunny day since.
- **Firmware:** they do not follow the firmware changes.
  - The 11-24 burst (467/h at 08 h, 159/h at 10 h) came **before** 1027 (17:2x) and before the despike floor
    3797fa4f (1126 at 23:05).
  - 11-20, on far older firmware, shows the same 08–09 h burst (701/h).
  - After 1126/1127 the 11-25 morning windows clip the same 24–31 of about 4690 samples as before.
- **Power level:** the rate peaks at 100–300 W, most often during the morning ramp. It does not rise at the cap.
- **Load:** inverter load events do not drive it; the rate is lower with the inverter running.
- **Tests:** manual and measure-coil periods raise it only modestly.
- **Other channels:** fry's iout WARNs come almost only at its Vout cap (92/h at the cap, 2/h off it; 744 lines on
  11-23, a long cap day). They vanished on 11-24, but there was no cap during the trial, so this is **not evidence
  for the comb**.
- flat's vout WARNs cluster at low power (25–100 W, up to 102/h).

## 4. Errors, stalls, latency, panic and backoff since 11-24 14:00

These were searched in the log through 2026-11-25 08:43.

- **ADC errors:** no `ADC stall`, `Never got a sample`, `ina22x … timeout!` or `Loop latency high` on either
  device. Also none of `hs_short`, `bf_fail`, `OV alert`, `Guru`, `abort()` or `Backtrace`.
- **fry PANIC reset:**
  - Firmware: 1040-gcce0d6e4.
  - When: between 18:00:25 and 18:03:34 on 11-24, at night with Vin 6.9 V, converter off, st=START.
  - Evidence: `bootinfo` at 18:10:28 reports `reset reason: PANIC`, uptime 411 s. The same boot is reprinted at 18:33
    and 18:34.
  - After it, `flash-suspend … auto: error(crash) … -> caches-off (cached)`: the auto-suspend probe recorded a crash
    and fell back to caches-off.
  - No panic text reached the log. The coredump has not been read, because that would be a device command.
  - It is not in any memory note. The next OTA (1044 at 18:36) reported `auto: pass … -> active`.
- **Dusk shutdowns (normal):**
  - fry 17:28:11: `Vout 26.59 / Vin 30.44 (I 0.02) at the LS min pulse ratio limit, shutdown`, then
    `backoff 5s [minpulse-M]`.
  - flat 17:30:09: the same line at Vin 30.20 V.
  - Both devices then logged `backoff 30/60/120/240s [sweep-no-mpp]` between 17:28 and 17:34.
- **fry 2026-11-25 08:18:29:** `backoff 5s [highD-bflowOpen]`. This was during the MANU measure-coil session.
- **Boot-time holds:** `temperature stale (ntc=0 mcu=1), converter held off` at about 8 s uptime after each reboot or
  OTA: 22 on fry, 17 on flat.
- **MQTT:** `No PING_RESP, disconnected` 4× on fry and 2× on flat between 18:07 and 18:51, around the panic, the OTAs
  and the night tests.
- **Peak loop lag** (`lag=` in the status lines):
  - flat: ≤ 990 µs throughout.
  - fry: 2.0–3.3 ms is normal at the INA226's 512 sps.
  - fry peaks: 8.3 ms at 15:22 (boot of 1009), 12.2 ms from the 17:16 boot of 1027 (held until the next reset), and
    10.7 ms right after today's 1146 boot (08:34:09, after the boot sweep).
  - None triggered a latency shutdown.

## Update 2026-11-26

Rerun at 2026-11-26 10:10 UTC on data through 10:07 UTC, with the same pipeline and exclusions. Read-only as before.

### Firmware and config

| time (UTC) | fry | flat |
|---|---|---|
| 11-25 08:33 / 08:40 | 1146-g8e8f497b | 1146 (fry 1146 while flat still ran 1127, 08:33–08:40) |
| 11-25 09:20 / 09:25 | 1152-g438c1ddf | 1152 (mismatch 09:20–09:25) |
| 11-25 10:33 / 10:38 | 1169-g8d3a5a84 | 1169 (mismatch 10:33–10:38) |
| 11-25 21:52 | OTA to 1180-g5d039229 **failed** | 1169 |

- All three mismatch windows fall inside the OTA exclusion. **From 11-25 10:38 on, both devices ran 1169.**
- The failed 1180 OTA: `reset reason: PANIC` at 21:53:48, 21:55:08 and 21:57:28, about 11 s after each of three
  flashes. `ota-owners` records "rebooted into 1169". The log has no later version print for fry, so fry being on
  1169 since then is inferred from that record. The panic loop on 1180 is a finding in its own right.
- **ripple_filter:** the boot conf prints show fry `'both'` throughout (last at 11-25 21:52). flat reads `''`
  (notch only) at its last query, 11-25 08:29 (`get-config`). No `set-config sensor.conf` was sent to either device
  after that. flat's value is therefore inferred from the absence of changes, not re-read since 08:29.
- **Other changes:**
  - Coil L0: fry 76.5e-6 (unchanged), flat 49.5e-6 (11-25 08:42).
  - Restarts: fry 09:09, flat 09:18.
  - No state-changing commands after 11-25 09:20.
  - 11-26 08:11 (flat) and 08:48 (fry): brief `st=MANU` blips of about 20 s, with no command ack. They come with the
    scan-near probe and power-doubled sweep, and are excluded conservatively.

### 11-25 (full day) and 11-26 (to 10:07)

| window | valid min | ratio | baseline (11-15..22), same minutes | z |
|---|---|---|---|---|
| 11-25 all | 448 | **0.847** | 0.729 ± 0.049 (0.667–0.807) | +2.4 |
| 11-25 tracking | 308 | **0.861** | 0.746 ± 0.045 | +2.6 |
| 11-25 cap | 140 | 0.793 | 0.730 ± 0.072 | +0.9 |
| 11-26 all | 134 | 0.688 | 0.623 ± 0.029 | +2.2 |

The table hides opposite effects by time of day, so the regimes are separated here.

| regime | ratio | flat mean W | baseline, same regime |
|---|---|---|---|
| 11-25 09:00–11:20, uncapped, charging 14–26 A | 0.890 | 352 | 09:00–11:20 on 11-15..23: 0.648–0.767 |
| **11-24 09:00–11:20, before the comb (same firmware on both)** | **0.867** | 230 | — |
| 11-25 11:20–15:15, low light, CV / near-cap sharing | 0.82 (hourly 0.53–1.08) | 95 | — |
| 11-25 15:15–18:00, battery full, both capped | 0.835 | 24 | — |
| 11-26 07:14–08:47, fry stuck on the wrong local peak | **0.151** | 121 | 08 h: 0.22–0.66 |
| 11-26 08:48–10:07, after fry's power-doubled sweep | 0.849 | 391 | 08:48–10:10 on 11-15..23: 0.621–0.693 |

- **The jump to about 0.85–0.89 in the morning predates the comb.** On 11-24 between 09:00 and 11:20, with no comb on
  either device and both on the same build, the ratio was already 0.867 against a baseline of 0.65–0.77. The 11-24
  07 h and 08 h values (0.65, 0.51) also sit at or above the top of the baseline. The step therefore happened between
  11-23 about 10 h and 11-24 07 h. That interval overlaps:
  - firmware 881 → 912;
  - the INA226 key moves on both devices on 11-23 11:03–12:47 (`ina22x_resistor`, `ina22x_conv_time_us`);
  - deletion of `power_conversion_eff` / `conversion_eff` on 11-23 12:31–12:47.

  The resistor values were kept (fry 0.5 mΩ, flat 1.0 mΩ), so a calibration change is not shown. Whether the step is
  real harvest or a measurement shift is unresolved. Either way the baseline before 11-24 is **not a valid control**
  for the comb.
- **11-26 07:14–08:47:** fry ran 12–40 W at Vin 58–75 V, near Voc, while flat made 221 W. The scan-near probe then
  triggered a power-doubled sweep at 08:47, which found the MPP at 44.6 V (175 W, against 48 W before). This is the
  known wrong-local-peak tracker loss. It dominates the 11-26 number; nothing ties it to the comb.

### Cap and inverter periods

- **Cap, 11-25 (140 min):** split into two parts.
  - 11:20–15:15: low light, and flat on its Vout limiter more often than fry, hourly ratio 0.53–1.08.
  - 15:15–17:15: battery full, both capped, about 24 W per device.

  Ratio 0.793, z = +0.9 against matched baseline minutes. The capped energy is small (fry 150 Wh, flat 189 Wh). The
  share at the cap follows the two Vout readings and charger targets, not the filter, so this is uninformative.
- **No cap with high PV power or inverter ripple** has occurred yet, so the scenario the sim targets is still
  untested.
- **Inverter / load:**
  - 11-25: 6 valid minutes around 08:00 (-33 A), ratio 0.63.
  - 11-26: 27 valid minutes, 09:00–10:07, pack current down to -75 A. Load ratio 0.903 against 0.813 for the non-load
    minutes of the same stretch (+0.09).
  - Baseline 11-15..22: load 0.789 vs non-load 0.722 (+0.07), because load minutes fall at favourable times of day.
  - The +0.09 is within that pattern, so there is no detectable inverter-ripple advantage.

### Verdict (11-26)

**No detectable comb effect.**
- The headline +2.4σ rise of fry/flat on 11-25 is real in the data. It already appears on the morning of 11-24,
  before `ripple_filter=both` was set, and coincides with firmware and INA-config changes on both devices on 11-23/24.
- Against the only valid same-config control, 11-24 09:00–11:20 (0.867), the comb days give 0.890 (11-25) and 0.849
  (11-26 after the sweep). That is a difference of -0.02 to +0.02.
- The cap and inverter samples are too few, and confounded by Vout sharing.

### What would make the trial conclusive

- **Freeze firmware and config on both devices.** Every OTA or config migration so far has moved the ratio more than
  the effect under test, and a new pre-comb baseline exists for only about 3 h, from 11-24 morning.
- **Swap or alternate instead of a fixed A/B.** Alternate fry's `ripple_filter` between `both` and `notch` day by day,
  or swap the roles of fry and flat. That cancels panel orientation and the sensor-offset sharing.
- **Number of days.** The day-to-day sd of the matched ratio is about 0.045 (6 %).
  - A 5 % harvest effect needs about 13 days per arm, so about 4 weeks alternating, for a 2σ result.
  - A 3 % effect needs about 40 days per arm.
- **Conditions for the cap claim.** The sim predicts a large gain at the cap with spiky current (94–97 % vs 33–80 %).
  Days with **a full battery around midday and good sun**, ideally **with the inverter running at the cap**, would
  test it in a few such days per arm. Even then the cap share has to be read from each device's own limiter time
  (`cv_lim_idx=1`), and the Vout offset between the two devices fixed or measured first.
- **Tracker losses.** Exclude or track wrong-local-peak periods separately, such as 11-26 07:14–08:47; they dwarf any
  filter effect.

## Reproduction

The scripts are in `/tmp/combtrial/`, which is not persistent:

- `pull.py`: per-minute InfluxDB aggregates. Run it with `.venv/bin/python3` after sourcing
  `.claude/memory/secrets.env`.
- `bat.py`: battery current.
- `desp.py` / `desph.py`: despike WARNs.
- `st.py` / `ev.py`: states and commands from the log.
- `an2.py` – `an6.py`: exclusion mask, ratios, WARN correlation and the time-matched comparison (`an6.py`, 11-26 update). Telemetry is in `m.pkl`, which merges `m1.pkl` and `m2.pkl`.

## Update 2026-11-27: sim per profile and inverter-load exposure

Sim `wx-cloud-cv` (Vout cap), harvest % of cap-limited, notch / both: fry-inv 97.8 / 98.0, china-inv 80.7 / 96.2,
pv8a-inv 33.7 / 91.1, var-inv 21.0 / 25.2, heater-inv 14.1 / 16.0. With fry's own 2 kW inverter the comb gains nothing
at the cap.

Off the cap with fry-inv noise (% of GMPP, notch / both): wx-cloud 77.8 / 85.0, wx-mid 71.1 / 80.5, wx-shade2-move
95.5 / 98.3. With fry and fry-wander noise (no inverter) both filters are within 0.1 pp.

Load estimate per minute: (P_fry + P_flat) / Uo - I_bat (batmon). 11-12..11-27 daytime: 884 of 8641 min above 15 A
(583 above 30 A), carrying 15.4 % of fry's energy. Upper bound of the field gain: ~0.154 x 8 pp = ~1.2 % of fry yield,
less at lower inverter loads than the 2 kW capture.

fry/flat ratio in load>30 A minutes divided by the ratio in load<5 A minutes of the same hours:
11-14 0.76, 11-15 0.86, 11-17 0.95, 11-20 2.03 (78 no-load min, likely cap), all before the comb; 11-26 (fry on
`both`) 0.996. Suggestive (fry lost against flat under load with the notch, not with the comb) but one day, cap minutes
not excluded. Each sunny day with an hour of inverter load adds a data point.

With minutes at the Vout limit dropped (either device > 5 % of its `cv_lim_idx`-carrying points at 1),
`etc/comb-load-report.py`: 11-14 0.65, 11-15 0.86, 11-17 0.96, 11-20 1.12 (notch on both; mean 0.90, sd 0.20);
11-26 0.81 (fry `both`). A first version that divided by all points kept more capped minutes and read 0.79/0.86/0.95/
1.14 and 0.99: the result moves with the cap filter, so it is not separable from noise yet. The report runs daily on
havan for the previous UTC day (user timer `comb-load-report`, 00:30) and appends to `~/pv/comb-report/report.jsonl`;
days with < 20 load minutes say `too few load minutes`.

## Update 2026-11-27: the sim's tracking loss with fry-inv is a noise cliff the field does not show

`wx-mid` with `fry-inv` loses 28.7 % (notch) / 19.4 % (both) to tracking; the tracker wanders (Vin rms 6.5 V, mean
34.6 V vs 49.8 V without the inverter). It is a cliff, not a slope: at `SIM_NOISE_GAIN` 0.5 both filters keep
99.2 / 99.4 %, at 0.25 99.6 / 99.6 %. Halving one channel of the profile: Vin 96.3 / 97.9, Iout 98.0 / 99.4,
Vout 69.1 / 76.7, so the Vin and Iout noise together push it over; Vout noise does not matter here.

Field check, per-minute stddev of fry's telemetry `Ui` (minutes > 50 W, no limiter), median / p90:
sim without inverter 1.1 V rms matches fry's no-load minutes (0.7–1.2 V). Under load > 60 A: 11-20 (notch) n=217,
274 W, 0.80 / 2.50 V; 11-26 (both) n=92, 371 W, 0.79 / 1.56 V; no wander. Only 11-14 (n=13, 85 W, older firmware)
shows 3.70 / 8.83 V. So at 270–380 W the real inverter noise sits below the cliff, and the comb's tracking gain there
is the below-cliff difference (≤ 0.2 pp). Low power with a heavy inverter load is untested beyond those 13 minutes.

Consequence: expected field gain of the comb on fry is ≤ 0.2 pp × 15 % of energy, i.e. not measurable. Sims with the
`fry-inv` profile at gain 1 overstate fry's November conditions; use gain 0.5 or check the field Vin spread first.
