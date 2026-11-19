*this document is an LLM generated placeholder*

# Field evaluation 11-18/11-19: peak scan cost, dusk tracking, trips

Date: 2026-11-19, read-only (no command sent to fry or flat). Data: InfluxDB `ha_van`/`mppt` (48 Hz raw for the
scans, 1 s means for dusk; `P` is output power), console log `pv/fugu_console.log{,.1..8}` on havan (UTC).
Baseline 11-10..11-17.

## Images that ran

| from (UTC) | fry | flat |
|---|---|---|
| 11-18 11:23 / 11:26 | 64b8498a | 64b8498a |
| 11-18 15:20 / 15:18 | 179ae6e9 | 179ae6e9 |
| 11-18 16:25 / 16:28 | a5f852f6-dirty | a5f852f6-dirty |
| 11-18 18:22 / 18:26 | 3f7136ae-dirty (built 18:20; incl. 1170e02c sweep pace + cap-discharge removal) | same |
| 11-19 09:59 / 09:56 | f567f67f | f567f67f |

179ae6e9 itself ran only ~65 min. The 11-18 dusk ran a5f852f6-dirty, which contains 179ae6e9 (drain limit,
min-pulse guard, scan, dusk-trap fix). The fry scans of 11-19 ran 3f7136ae-dirty. All post-OTA reboots are the
four OTAs per device; no other reboot, no panic.

Interventions in the comparison windows: fry 11-18 16:41-17:14 was under manual `dc`/`sweep` control (dusk-landing
measurement, `2026-11-18-fry-dusk-landing.md`), resumed with `mppt` at 16:48, 16:51, 16:56, 17:14. flat 11-18 got
six manual sweeps 16:40-17:11. Baseline days with OTAs at dusk: 11-11 (both, 16:41-16:45), 11-13 (flat every
~10 min, fry from 17:14).

## 1. Scan cost on fry, 11-19

fry learned 6 substrings at 08:38:57 (log also shows `cannot read ConfFile /littlefs/conf/tracker.conf` right
after: the file did not exist; no `write failed` followed, so the save presumably created it; not verified after the
09:59 reboot). flat has not learned (no `learned` line), so scan is off on flat.

11 logged scans 08:40:54-09:53:49, none moved (best probe ratio 0.80-0.91, never above 0.94, so no 60 s retries).
Loss = integral of (linear interpolation of the mean P 2 s before / 2 s after the Vin dip) minus P over the dip
(Vin < 0.95 of the operating Vin, -0.3/+0.5 s). A 1 s pre/post baseline gives the same within 3 J.

| scan | probes | dip [s] | P before -> after [W] | P min [W] | loss [J] | trigger (inferred) |
|---|---:|---:|---|---:|---:|---|
| 08:40:54 | 1 | 1.30 | 102 -> 102 | 82 | 2.8 | Vin drift: P&O moved Vin 59 -> 66 V after the sweep (P steady) |
| 08:48:07 | 2 | 1.96 | 84 -> 86 | 62 | 14.2 | cloud (P 130 -> 81 W) |
| 08:59:17 | 2 | 1.91 | 85 -> 84 | 61 | 17.9 | cloud (P 165 -> 87 W, still falling) |
| 09:07:57 | 2 | 1.93 | 112 -> 111 | 77 | 27.7 | cloud (P 217 -> 116 W) |
| 09:09:04 | 2 | 1.94 | 120 -> 104 | 77 | 29.9 | forced: 30 min re-sweep flagged (landed 153 vs MPP 207 W) |
| 09:10:03 | 1 | 1.37 | 216 -> 216 | 179 | 13.9 | cloud (light back, 100 -> 216 W) |
| 09:12:29 | 2 | 1.92 | 116 -> 120 | 80 | 32.9 | cloud (P 214 -> 119 W) |
| 09:13:29 | 1 | 1.37 | 231 -> 232 | 192 | 18.1 | cloud (light back) |
| 09:43:49 | 1 | 1.41 | 312 -> 315 | 266 | 18.7 | periodic (300 s after the 09:38:47 sweep) |
| 09:48:49 | 1 | 1.41 | 325 -> 325 | 269 | 24.9 | periodic |
| 09:53:49 | 1 | 1.39 | 347 -> 345 | 288 | 25.1 | periodic |

- Total 226 J = 0.063 Wh against 310 Wh harvested 08:38:57-09:59:20: **-0.020 pp**. The sim predicted about
  -0.04 pp; the field cost is in line (half of it).
- Split: 7 trigger-driven (6 cloud, 1 Vin drift), 1 forced, 3 periodic. Between 09:13 and 09:38 (sun, ~250-300 W)
  nothing was logged: periodic attempts without a candidate return silently (lower peaks that cannot beat the
  operating point even at the best current seen), which costs nothing.
- Steady state: one 1-probe scan per 300 s at ~330 W costs ~25 J, 0.08 W or 0.025 % of the power.
- Under clouds a scan costs 14-33 J; drops in P let more candidates pass, so cloud scans use 2 probes.
- The cloud-triggered scans fire while P is still ramping (08:59: 146 -> 111 -> 87 W in the last 10 s), and with
  uniform cloud they cannot find another peak. On cost, the trigger does not need tightening. A longer steady
  requirement (e.g. +-5 % for 10 s) would drop most of the cloud scans; that saves at most ~0.01 pp.

## 2. Dusk, 11-18 vs 11-10..11-17

Window: sunset - 60 min (sunset for Lisbon, 17:22-17:28 UTC) to dark (last time the 60 s mean P >= 0.5 W).
Available power is the sweep MPP (`Stop sweep ... MPP=`), linearly interpolated in time and taken to 0 at dark.
Manual/OTA spans excluded. Two caveats bind this metric: dusk sweeps come every 30 min (2-7 per window), so the
interpolation is weather-dependent by +-15 % (ratios > 1 occur); and below ~10 W the swept MPP is inflated by
input-cap discharge (+50-100 % at 4 W, `2026-11-18-fry-dusk-landing.md`). That bias is the same on all days
compared here (the cap-discharge removal arrived at 18:22).

| day | dev | dark | min | E [Wh] | avail [Wh] | E/avail | far below [min] |
|---|---|---|---:|---:|---:|---:|---:|
| 11-10 | fry | 17:04 | 36 | 1.5 | 4.3 | 0.36 | 14.1 |
| 11-11 | fry | 17:18 | 48 | 14.1 | 34.1 | 0.41 | 5.4 |
| 11-12 | fry | 17:14 | 26 | 3.0 | 7.2 | 0.42 | 5.1 |
| 11-14 | fry | 17:33 | 69 | 14.2 | 17.0 | 0.84 | 26.6 |
| 11-15 | fry | 17:28 | 64 | 36.4 | 49.9 | 0.73 | 7.2 |
| 11-16 | fry | 17:20 | 41 | 5.5 | 7.0 | 0.78 | 7.2 |
| 11-17 | fry | 17:23 | 55 | 1.9 | 4.6 | 0.40 | 12.8 |
| **11-18** | **fry** | 17:24 | 28 | 3.8 | 6.3 | **0.60** | **7.0** |
| 11-10 | flat | 17:26 | 57 | 6.7 | 7.1 | 0.95 | 0.1 |
| 11-11 | flat | 17:33 | 63 | 27.6 | 49.1 | 0.56 | 9.5 |
| 11-12 | flat | 17:32 | 62 | 37.3 | 50.1 | 0.74 | 29.8 |
| 11-15 | flat | 17:31 | 66 | 47.1 | 73.7 | 0.64 | 0.4 |
| 11-16 | flat | 17:25 | 46 | 8.8 | 10.2 | 0.87 | 0.0 |
| 11-17 | flat | 17:05 | 42 | 3.8 | 3.3 | 1.14 | 0.0 |
| **11-18** | **flat** | 17:27 | 64 | 12.7 | 11.1 | **1.15** | **0.0** |

"Far below": Vin < 0.7 x the last sweep's Vmpp and P < 0.5 x the interpolated sweep MPP. 11-13 is left out
(flat OTA'd every ~10 min, fry only 15 min of data), flat 11-14 has no sweep (no reference). Baseline medians:
fry E/avail 0.42, far below 7.2 min (~20 % of the window); flat 0.80 and ~0.3 min.

The energy ratio is too noisy to separate firmware from weather. The point check at each sweep is more direct:
mean P 45-15 s before a dusk sweep against its MPP, and Vin there against the sweep's Vmpp:

| dev, sweep MPP | baseline 11-10..11-17 | 11-18 |
|---|---|---|
| fry, 4-15 W | n=13, P/MPP median 0.30 (IQR 0.09-0.47), 6 of 13 at Vin < 0.7 Vmpp (sitting at 29-30 V) | n=2 (16:41, 17:12, free-running), 0.58 and 0.67 at 50.4 and 52.4 V (Vin/Vmpp 1.03, 0.99) |
| fry, 2-4 W | n=34, median 0.02, 10 at Vin < 0.7 Vmpp | none |
| flat, 4-15 W | n=8, median 0.86 (0.77-0.90), 3 at Vin < 0.7 Vmpp | n=6, median 0.90 (0.89-0.92), all at Vin/Vmpp 0.99-1.02 |

- fry: on every baseline day, once the sweep MPP fell below ~8 W, the pre-sweep operating point sat at 28.6-30.0 V
  with 0.3-0.5 of the swept power. On 11-18 the two free-running dusk sweeps at that light level found fry at
  50-52 V, on the static plateau (52-58 V, measured at 16:45 and 17:12), and at ~5.0 W vs a static MPP of ~4.2-4.5 W
  at 17:12. With the cap inflation, 0.6-0.7 of the swept MPP is at the true MPP.
- fry 11-18 far-below time (7.0 min) is all from 17:14:47 to 17:21:56, after the last manual `mppt` resume at 37 V:
  the tracker went down to the new DCM Vin floor (30.1 V = 1.18 x 25.5 V) and stayed there at 0.8-0.9 W until dark.
  This follows a manual session (the stale tracker state after `mppt` noted in the dusk-landing study), so it does
  not show the free-running behaviour; it is still the same end state as the baseline days.
- flat: slightly better and with no far-below time, but six manual sweeps re-landed it, so read it as unchanged.
- Verdict: one contaminated dusk is not enough. The two free-running fry points support the dusk-trap fix; the
  11-19 dusk (f567f67f) is the first clean test if nobody drives the converters then.

## 3. Events since the OTA (11-18 15:18/15:20 to 11-19 10:00)

| event | fry post | flat post | baseline per day (11-15..11-18) |
|---|---:|---:|---|
| reverse-current (`Reverse I`, `revI-*`) | 0 | 0 | 0 on both since 11-10 (the 51 lines are bench boards) |
| `START blocked: minpulse-M` | 1 (07:12:53, Vin 26.1 / Vout 25.0) | 1 (07:09:18, Vin 27.7) | new |
| min-pulse stop `[minpulse-M]` | 1 (17:25:12, Vin 29.4 / Vout 25.7, I 0.01 A) | 1 (17:26:45, Vin 29.2, I 0.04 A) | new |
| `highD-bflowOpen` trips | 0 | 0 | fry 4-6, flat 4-7, mostly at dawn/dusk |
| ADC stall/error, `Loop latency high`, ina22x timeout | 0 | 0 | 0 (flat had 6 + 10 on 11-12/13) |
| `sweep-no-mpp` backoffs | 25 (7 dusk, 18 dawn) | 43 (19 dusk, 24 dawn) | fry 10-52, flat 17-49 |
| reboots | 4 (all OTA) | 4 (all OTA) | |
| other backoffs | 0 | 0 | |

- The min-pulse stops happen at dark, at the end of conversion, where the baseline days ended with
  `highD-bflowOpen` trips. That replacement is the visible change; nothing abnormal.
- Other W/E lines since the OTA: one boot-time `temperature stale ... held off` per boot (new message, expected),
  one wifi `heap alloc failed` on fry at 16:30 (1700 B, free 24 KB, during a scope capture; ~0.3/day before),
  scope buffer overflows during the manual captures, one MQTT write timeout on fry at 15:31. fry heap
  min-ever 17.2 KB at 09:55 (uptime 15.5 h).
- Dawn `no MPP` loops are not new. Dawn counts per morning, flat: 11-14 13, 11-15 6, 11-16 8, 11-17 20, 11-18 20,
  11-19 24; fry: 11-12 11, 11-13 20, 11-16 8, 11-17 19, 11-18 10, 11-19 18. Spacing was 34-40 s before and is 33 s
  now (shorter sweep, 30 s backoff). flat 11-19 ran 07:08:43-07:19:32 (11 min), against 07:17:44-07:31:55 on 11-18.
  Dusk shows the same loop on both (flat 11-18 19 in 7 min, 11-17 29). fry's dawn sweeps end with `best 0.00W`
  at duty ~1000 while Vin is still 46-56 V.
