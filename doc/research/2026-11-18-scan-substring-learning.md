*this document is an LLM generated placeholder*

# Partial peak scan: substring count learned from the sweeps (2026-11-18)

Simulator and offline field replay only. Nothing here ran on a converter.

The `tracker.conf::pv_substrings` key is gone. `PeakScan` (`src/peak_scan.h`) learns the number of bypass-diode
substrings $n$ from the global sweeps and keeps it in RAM; the scan stays off until then. Published description:
`website/docs/internals/mppt-tracker.md`, "Substring count".

## Rule

During a sweep ($V_{oc}$ down to the battery) `sweepSample()` records local maxima of `power_smooth` below
$0.93\,V_{oc}$ with an 8 % dip on both sides, in the scan's probe arrays (idle during a sweep; 6 slots, more = noise,
the sweep is discarded). At the end (`onSweepMpp()`, sweep MPP at least 5 % of `p_max`):

* maxima below 30 % of the MPP are dropped; the first remaining one (highest voltage) is the full-string maximum;
* count 0 if it is the only one; else the largest $n$ in 2..6 that puts every lower maximum at
  $m - 0.25 .. m + 0.5$ shares of $0.82\,V_{oc}/n$ ($1 \le m < n$), or 1 if none fits;
* learned when two counted sweeps in a row give the same $n \ge 2$; $n$ only grows. One log line:
  `scan: <n> bypass-diode substrings learned from the sweeps`.

Why these numbers (field replay below): the start transient of flat's sweeps is a ~95-100 W maximum at
0.94-0.98 $V_{oc}$ (0.91-0.94 in low light), so maxima above 0.93 $V_{oc}$ are not recorded; the hot unshaded MPP
reaches 0.72 $V_{oc}$ (5.3 of 6 shares), so a single maximum never counts; shaded lower maxima sit 0-0.45 shares
above the grid (strong substrings carry the weak ones' current), hence the asymmetric window.

## Field replay (fry, flat; InfluxDB `ha_van/mppt`, read-only)

Telemetry is fine enough: about 48 Hz, about 500 samples per 10 s sweep. Sweep ends from the console log
(`Stop sweep ... MPP=`) on havan, 2026-11-03..18. Log time and telemetry time agree at offset 0 (the logged MPP
matches the telemetry there, not at 1 or 2 h). Per sweep: last zero-duty sample to the duty maximum; telemetry `P`
is output power (med3), smoothed with med3 + EWM 0.3 to stand in for `power_smooth`. Replayed through the firmware
header (`test/host-stub/scan-learn-replay.cpp`), with `p_max` 1000 W.

| | fry | flat |
|---|---|---|
| sweeps with telemetry | 362 | 320 |
| MPP at least 50 W | 193 | 223 |
| two or more maxima | 27 (11 of 16 days) | 4 |
| count per such sweep | 6: 27 | 6: 1, none fits: 3 |
| learned, fresh boot each day | n = 6 on 3 days (11-05, 11-12, 11-14) | never |
| learned, no reboot 11-03..18 | n = 6 on 11-05 09:21 | never |
| learned a count other than 6 | never | never |

fry's lower maxima sit at 2.93-3.44, 3.91-4.28 and 4.87-5.43 shares of 6 (28 maxima). flat's three no-fit sweeps
are the start transient followed by the MPP (twice, 2026-11-08/09 low light) and a dusk sweep with maxima at 5.65 and
4.41 shares. On fry, 11-07 and 11-13 had two shaded sweeps that were not consecutive and did not learn.

Earlier rule variants on the same data (Python mirror, discarded): a symmetric ±0.3 window learned 5 or 2 on some
days; the learned $V_{mpp}/V_{oc}$ in place of 0.82 learned 5 once; counting a single maximum below 0.92 of
$0.82\,V_{oc}$ took flat's hot unshaded MPP (0.75 $V_{oc}$) for a lower maximum.

## Simulator (exact series plant, pgx93 only)

`test/host-stub/mppt-sim.cpp`, plant `SIM_RING=fry|flat`, measured noise replay `SIM_NOISE=fry|flat`, offsets
0/11/23 s (3 seeds per cell). Variants: off = main without scan; fixed = main with `pv_substrings=6`; learn = this
branch; warm = this branch with `SIM_SCAN_N=6` (learned before the run). Harvest in points of the reachable GMPP
(`== TRACKER ... harvest`). New scenario `wx-shade-learn`: the wx-shade2 shade over the boot sweep and the 1800 s
sweep, clear at 1900-1960 s, back over 2000-2300 s.

| Cell | off | fixed − off | learn − off | learn − fixed (worst..best) | warm − fixed | wrong-peak s off / fixed / learn | scans fixed / learn |
|---|---|---|---|---|---|---|---|
| wx-shade-learn fry | 97.50 | +1.82 | +1.83 | +0.01 (-0.03..+0.05) | 0.00 | 1166 / 58 / 62 | 9.3 / 5.3 |
| wx-shade-learn flat | 97.56 | +1.75 | +1.76 | +0.01 (-0.03..+0.06) | 0.00 | 1167 / 99 / 98 | 10.0 / 6.0 |
| wx-shade2-move fry | 96.01 | +2.79 | 0.00 | -2.79 | 0.00 | 1412 / 212 / 1412 | 5.0 / 0 |
| wx-shade2-move flat | 96.10 | +2.79 | 0.00 | -2.79 | 0.00 | 1411 / 211 / 1411 | 5.0 / 0 |
| wx-shade3-move fry | 75.55 | +23.57 | 0.00 | -23.57 | 0.00 | 1569 / 69 / 1569 | 7.0 / 0 |
| wx-shade3-move flat | 75.54 | +23.62 | 0.00 | -23.62 | 0.00 | 1569 / 69 / 1569 | 7.0 / 0 |
| wx-shade-walk fry | 86.28 | +10.33 | 0.00 | -10.33 | 0.00 | 1160 / 333 / 1160 | 7.7 / 0 |
| wx-shade-walk flat | 86.29 | +10.64 | 0.00 | -10.64 | 0.00 | 1160 / 293 / 1160 | 6.7 / 0 |
| wx-sweep-shade fry | 85.17 | +9.06 | 0.00 | -9.06 (-12.91..-7.14) | 0.00 | 586 / 207 / 586 | 1.0 / 0 |
| wx-sweep-shade flat | 85.19 | +9.03 | 0.00 | -9.03 (-12.76..-7.16) | 0.00 | 586 / 209 / 586 | 1.0 / 0 |
| wx-shade-cloud fry | 96.05 | -0.14 | 0.00 | +0.14 (-0.28..+0.40) | 0.00 | 220 / 128 / 220 | 10.0 / 0 |
| wx-shade-cloud flat | 96.13 | +0.17 | 0.00 | -0.17 (-0.44..+0.09) | 0.00 | 220 / 122 / 220 | 10.7 / 0 |
| wx-shade2 fry / flat (static) | 99.00 / 99.07 | 0.00 | 0.00 | 0.00 | 0.00 | 0 | 1.0 / 0 |
| wx-shade3 fry / flat (static) | 98.97 / 99.04 | -0.05 / -0.03 | 0.00 | +0.05 / +0.03 | 0.00 | 0 | 1.0 / 0 |

* warm and fixed produce byte-identical output in all 48 runs: the learner changes nothing once $n$ is known.
* learn and off produce byte-identical output in all 42 runs outside wx-shade-learn: these scenarios end (900-2400 s) before a
  second shaded sweep, so nothing is learned. That is the first-shade-event cost: from boot, a shade event is
  not scanned until two consecutive sweeps (30 min apart) have seen it. In wx-shade-learn the first scan follows
  the 1812 s sweep in all 6 runs (learned there) and the returning shade is then handled like fixed $n = 6$ (+0.01).
* Default without shade (main default vs this branch, fry plant and noise): wx-cloud, wx-cloud-cv, wx-sun-fry,
  wx-dawn, wx-low, wx-mid and wx-shade2-move produce byte-identical output (the noise-file path line excepted).

## Cost

esp32s3 build vs e2bcaff2: image +400 B, `MpptController` +16 B RAM, `_stopSweep` +378 B (IRAM; the learner is
inlined there), `update()` -11 B. `rtprof_map.py --check`: 79 entries, 0 not in IRAM. No flash writes, no
allocation.

## Open risks

* First shade after a boot: no scan until two consecutive sweeps were shaded (field: fry learned on 3 of 11 shaded
  days with a daily reboot, once without). A shadow that comes and goes within 30 min after a reboot is never
  caught on its first pass. Persisting $n$ (one write when learned) or a default would remove this; both were out
  of scope (RAM only).
* A cloud edge in two consecutive sweeps can fake a lower maximum that fits 6 (75 % of positions fit the
  window): the scan then runs on an unshaded string. Its candidate filter skips most probes in full sun; the cost
  is scan time, not a wrong move.
* A string of 3 substrings with its lower maxima on even multiples of 6 learns 6 (extra probes, nothing missed).
  Strings with more than 6 substrings learn at most 6.
* Field replay uses telemetry output power (med3 + EWM), not the firmware's `power_smooth`; dips near 8 % may
  differ. No converter has run this code.
* `kVmp` (learned $V_{mpp}/V_{oc}$ for probe placement) is unchanged; the learner uses the fixed 0.82.

## Reproduce

```bash
# field: sweep lines from havan's console logs, then telemetry (INFLUX_HA_VAN_USER/_PASS)
python3 doc/research/scan-learn/field_sweeps.py fetch sweeps.txt sweeps.pkl
python3 doc/research/scan-learn/field_sweeps.py export sweeps.pkl > replay.txt
clang++ -std=gnu++20 -O2 -include test/host-stub/esp/sim_prelude.h -I test/host-stub/esp -I src \
    test/host-stub/scan-learn-replay.cpp -o /tmp/scan-learn-replay
/tmp/scan-learn-replay day < replay.txt   # "day": fresh PeakScan per device and date
# sim: one cell
SIM_RING=fry SIM_NOISE=fry SIM_NOISE_OFS=11 offload run --host pgx93 -- <mppt-sim> wx-shade-learn
```
