*this document is an LLM generated placeholder*

# Partial peak scan: substring count learned from the sweeps (2026-11-18)

Simulator and offline field replay only. Nothing here ran on a converter.

`PeakScan` (`src/peak_scan.h`) learns the number of bypass-diode substrings $n$ from one shaded global sweep and
persists it in `tracker.conf::pv_substrings` (a learned value; 0 = learn again after a restart). The scan stays off
until then. Published description: `website/docs/internals/mppt-tracker.md`, "Substring count".

Revision 2 (after review): one sweep instead of two agreeing ones, persisted, learner on core 0, full-string
reference = first maximum with 60 % of the MPP (was: maxima above the MPP ignored).

## Rule

RT side (core 1): `sweepSample()` records local maxima of `power_smooth` below $0.93\,V_{oc}$ with an 8 % dip on both
sides into the scan's probe arrays (idle during a sweep; 6 slots, more = noise). `onSweepMpp()` sets `pending` if the
sweep's MPP is at least 5 % of `p_max`, Voc is valid and the slots did not overflow. Nothing else runs on the RT path.

Core 0 (`loopLF` -> `MpptController::scanLf()`, at most 3 s later; the arrays stay untouched for at least 5 s):

* the MPP must be one of the recorded maxima (98 %);
* the first maximum with at least 60 % of the MPP power is the full-string maximum, earlier ones are ignored;
* the lower maxima are those after it with at least 30 % of the MPP power; none -> nothing learned;
* $n$ = the largest count in 2..6 that puts every lower maximum at $m - 0.25 .. m + 0.5$ shares of $0.82\,V_{oc}/n$
  ($1 \le m < n$); no fit -> nothing learned;
* learned if it exceeds the current $n$ (only grows), logged once (`scan: <n> bypass-diode substrings learned from
  the sweep`), marked unsaved; written to `tracker.conf` by `scanSave()` when `flashWriteSafe()` (flash auto-suspend,
  converter off, or below 5 W) and from `systemRestart()`. Worst case a few writes per device life.

Why these numbers (field replay below): flat's start transient is a ~95-100 W maximum at 0.94-0.98 $V_{oc}$
(0.90-0.94 in low light), so maxima above 0.93 $V_{oc}$ are not recorded and a weaker one does not become the
reference; shaded lower maxima sit 0-0.45 shares above the grid (strong substrings carry the weak ones' current),
hence the asymmetric window. With "MPP = reference" (revision 1) a sweep whose shaded lower maximum is the MPP taught
nothing; the 60 % reference learns those too (fry 11-04 15:51, exact-plant 6-substring string with one substring at
0.6-0.7).

## Behaviour by string (exact series plant, `test_scan_learns_other_counts`, probe over shade 0.5-0.9)

| Substrings | One substring shaded | Two shaded |
|---|---|---|
| 3 | 6 (0.5-0.8), nothing at 0.85+ | - |
| 4 | 5 | 6 |
| 5 | 6 (0.6-0.8; nothing at 0.5) | 5 |
| 6 | 6 (0.6-0.8; nothing at 0.5) | 6 |

Shade of 0.85 and above leaves a single maximum (no learning). The fit cannot tell 3 from 6 or 4/5 from 5/6; the
count picks the probe grid. Cost of a non-6 count: extra or misplaced probes (a better maximum can be missed),
never a wrong move (the scan only moves to a probe that measured more power).

Transient guard: a start transient with less than 60 % of the MPP is not the reference, and one above 0.93 $V_{oc}$
is not recorded. A transient between, with 60 % of the MPP, plus a hot MPP at or below 0.75 $V_{oc}$ still fakes 6
(`test_scan_unshaded_never_learns` asserts this). In the field data the transient is about 100 W, so this needs an
MPP below ~165 W with modules hot enough for $V_{mpp} \le 0.75\,V_{oc}$; no field sweep did it. Time from sweep start
and dip depth do not separate the transient from fry's shaded full-string maximum (both about 4 s, similar dips).

## Field replay with boot resets (fry, flat; InfluxDB `ha_van/mppt`, read-only)

Input: the reviewer's `sweeps-boot.txt` (362 fry, 320 flat sweeps, 2026-11-03..18, 42 fry / 38 flat boots;
`field_sweeps.py` now exports the same boot ids: log time minus uptime). Telemetry ~48 Hz, `P` as med3 + EWM 0.3
for `power_smooth`, log and telemetry time agree at offset 0. Each new boot starts a fresh `PeakScan` with the
saved count. Saved at the next sweep start (converter off) and at the reboot (`systemRestart()`); "crash" = every
reboot unplanned, so only the sweep-start write counts. Shaded = a counted sweep (`pending`) with at least two
maxima of 30 % of the MPP.

| | fry | flat |
|---|---|---|
| shaded sweeps | 27 | 4 |
| covered ($n$ known after the sweep) | 27 | 2 |
| covered, crash model | 27 | 1 |
| sweeps with $n \ne 6$ | 0 | 0 |
| first learned | 11-04 15:51 (lower maximum is the MPP) | 11-12 12:43 |
| saved at the end | 6 | 6 (crash model: 0) |

flat's two uncovered shaded sweeps (11-04 13:39, 11-08 16:14) are a low-light transient at 0.91 $V_{oc}$ (88-91 W)
followed by the MPP at 0.85 $V_{oc}$: the transient is the reference and the MPP (6.2 sixths) fits no grid, so
nothing is learned. fry's lower maxima sit at 2.93-5.43 sixths. The reviewer counted 37/39 fry and 3/7 flat with a
different "shaded" definition; both counts show no wrong $n$.

## Simulator (exact series plant, pgx93 only, tag scan-learn2)

`test/host-stub/mppt-sim.cpp`, plant `SIM_RING=fry|flat`, measured noise replay `SIM_NOISE=fry|flat`, offsets
0/11/23 s (3 seeds per cell), all runs on pgx93. Variants: off = main without scan; fixed = main with
`pv_substrings=6`; learn = this branch from an empty `tracker.conf` key; warm = this branch with
`pv_substrings=6` (learned and persisted earlier). Harvest in % of the reachable GMPP (`== TRACKER ... harvest`).
`wx-shade-learn`: the wx-shade2 shade over the boot sweep and the 1800 s sweep, clear at 1900-1960 s, back over
2000-2300 s. Last column: learned / reloaded `tracker.conf` value per seed (the sim now redirects `rename`/`unlink`
into its littlefs tree, so `scanSave()` really writes).

| Cell | off | fixed − off | learn − off | learn − fixed (worst..best) | warm − fixed | wrong-peak s off / fixed / learn | scans fixed / learn | first scan learn [s] | learned/saved |
|---|---|---|---|---|---|---|---|---|---|
| wx-shade-learn fry | 97.50 | +1.80 | +1.80 | +0.00 | 0 | 1166 / 61 / 61 | 9.3 / 10.3 | 12 ×3 | 6/6 ×3 |
| wx-shade-learn flat | 97.56 | +1.74 | +1.74 | +0.00 | 0 | 1167 / 99 / 99 | 10.0 / 11.0 | 12 ×3 | 6/6 ×3 |
| wx-shade-cloud fry | 96.01 | -0.02 | +0.05 | +0.07 (+0.00..+0.22) | 0 | 220 / 139 / 139 | 10.3 / 11.3 | 15 ×3 | 6/6 ×3 |
| wx-shade-cloud flat | 96.11 | +0.17 | +0.05 | -0.12 (-0.31..+0.00) | 0 | 220 / 145 / 133 | 11.0 / 11.3 | 15 ×3 | 6/6 ×3 |
| wx-shade2 fry (static) | 99.01 | -0.02 | -0.02 | 0.00 | 0 | 0 | 1.0 / 2.0 | 12 ×3 | 6/6 ×3 |
| wx-shade2 flat (static) | 99.06 | 0.00 | 0.00 | 0.00 | 0 | 0 | 1.0 / 2.0 | 12 ×3 | 6/6 ×3 |
| wx-shade3 fry (static) | 98.97 | -0.04 | -0.04 | 0.00 | 0 | 0 | 1.0 / 2.0 | 12 ×3 | 6/6 ×3 |
| wx-shade3 flat (static) | 99.03 | -0.03 | -0.01 | +0.02 (+0.00..+0.04) | 0 | 0 | 1.0 / 0.7 | 12 ×1 | 6/6, -, - |
| wx-shade3-move fry | 75.54 | +23.59 | +7.86 | -15.73 (-23.59..0.00) | 0 | 1569 / 69 / 1069 | 7.0 / 3.3 | 12, 1813 | 6/6, -, 6/6 |
| wx-shade3-move flat | 75.56 | +23.60 | 0.00 | -23.60 | 0 | 1569 / 69 / 1569 | 7.0 / 0 | - | - |
| wx-shade2-move fry | 96.01 | +2.78 | 0.00 | -2.78 | 0 | 1412 / 212 / 1412 | 5.0 / 2.0 | 1813 ×3 | 6/6 ×3 |
| wx-shade2-move flat | 96.11 | +2.78 | 0.00 | -2.78 | 0 | 1411 / 211 / 1411 | 5.0 / 2.0 | 1812 ×3 | 6/6 ×3 |
| wx-shade-walk fry | 86.28 | +10.31 | 0.00 | -10.31 (-10.64..-10.02) | 0 | 1160 / 333 / 1160 | 7.7 / 0 | - | - |
| wx-shade-walk flat | 86.29 | +10.63 | 0.00 | -10.63 (-10.89..-10.36) | 0 | 1160 / 280 / 1160 | 7.0 / 0 | - | - |
| wx-sweep-shade fry | 85.17 | +9.06 | 0.00 | -9.06 (-12.90..-7.14) | 0 | 586 / 207 / 586 | 1.0 / 0 | - | - |
| wx-sweep-shade flat | 85.17 | +10.70 | 0.00 | -10.70 (-12.57..-7.17) | 0 | 586 / 140 / 586 | 1.0 / 0 | - | - |

* warm and fixed produce identical output in all 48 runs (the noise path line and the learned line excepted): once
  $n$ is persisted the learner changes nothing.
* learn − fixed is the one-time cost of a converter that has never seen a shaded sweep: the move/walk/sweep-shade
  scenarios start with an unshaded boot sweep, so the shade is not scanned until a sweep sees it (in the 2400 s
  move runs that sweep is at 1812 s, too late to matter inside the run). With the count persisted this happens once
  per installation, not once per boot.
* Default without shade (main vs this branch, fry plant and noise, pgx93): wx-cloud, wx-cloud-cv, wx-sun-fry, wx-dawn,
  wx-low and wx-mid produce identical output (an ssh retry line in one log excepted). wx-shade2-move now differs:
  the branch learns 6 at the 1812 s sweep and scans afterwards (harvest unchanged at 96.0 %).

## Cost

esp32s3 (IDF 6.0.3) vs main 60fa7dfd, revision 3: IRAM (`.text` in DIRAM) +204 B (135819 -> 136023;
`MpptController::update()` +64 B with `sweepSample` inlined, `PeakScan::claim()` 33 B out of line in IRAM; inlined
twice it cost 120 B more; revision 2 was +100 B, revision 1 +378 B in `_stopSweep`). `scanLf()` 662 B and
`scanSave()` 520 B in flash. `rtprof_map.py --check`: 79 entries, 0 not in IRAM. Classic esp32: IRAM 108499 B
(main 108463), 22.6 kB free. One `tracker.conf` rewrite per learned increase, only when `flashWriteSafe()` or at a
console restart on core 0.

Sim sanity after revision 3 (pgx93, offset 11): warm output equals fixed $n = 6$ on wx-shade-learn fry,
wx-shade-cloud flat, wx-shade2-move fry and wx-shade3 flat; learn output equals revision 2 on the same cells.

## Revision 3 (re-review)

* `systemRestart()` saves only off the RT core: the ADC-stall and no-sample restarts run on core 1 and must not write
  flash or race `loopLF`'s save.
* Handoff: `pending` is a `std::atomic<bool>` (store release, exchange acquire); RT bumps `gen` in `claim()` at sweep
  start and scan start, before it rewrites the maxima. `learn()` copies them, fences, and drops the sweep if `gen`
  moved (`test_scan_learn_drops_rewritten_sweep`).
* `unsaved` is now `n != saved`, `saved` being the value read at boot or last written. `scanSave()` re-reads the key
  first; if it differs from `saved` (set by hand since, e.g. `set-config ... pv_substrings 0`), the learned count is
  not written.
* `pv_substrings` is back in the reference table and the config editor, as a learned value.

Covered as "known before the sweep" (the scan can act on that sweep's shade): fry 26/27, flat 1/4. "Covered" in the
table above counts $n$ known after the sweep.

## Open risks

* A strong start transient (at least 60 % of the MPP, below 0.93 $V_{oc}$) with a hot MPP at or below 0.74 $V_{oc}$
  learns and saves a false 6 for good (until `pv_substrings` is set to 0). Cost on an unshaded string: idle scans,
  about -0.04 pp of harvest; no wrong move.

* First shade on a fresh installation (or after `pv_substrings=0`): not scanned until a sweep shows it. A shadow that
  comes and goes within 30 min after that boot is missed on its first pass.
* A crash or power loss between learning and the next flash-safe moment loses the count (flat in the crash model).
  It is learned again from the next shaded sweep.
* A cloud edge or a strong start transient (above) can fake 6 on an unshaded string; the scan then runs, probes and
  stays. On a 6-substring string that is the right count anyway.
* Strings of 3/4/5 learn 5 or 6 (table above); strings with more than 6 learn at most 6.
* Field replay uses telemetry output power (med3 + EWM), not the firmware's `power_smooth`; dips near 8 % may
  differ. No converter has run this code.

## Reproduce

```bash
# field: sweep lines from havan's console logs, then telemetry (INFLUX_HA_VAN_USER/_PASS)
python3 doc/research/scan-learn/field_sweeps.py fetch sweeps.txt sweeps.pkl
python3 doc/research/scan-learn/field_sweeps.py export sweeps.pkl > replay.txt
clang++ -std=gnu++20 -O2 -include test/host-stub/esp/sim_prelude.h -I test/host-stub/esp -I src \
    test/host-stub/scan-learn-replay.cpp -o /tmp/scan-learn-replay
/tmp/scan-learn-replay < replay.txt         # saved at sweep start and at reboot
/tmp/scan-learn-replay crash < replay.txt   # saved at sweep start only
# sim: one cell (learn); fixed/warm: SIM_CONF=tracker.conf:pv_substrings=6
SIM_RING=fry SIM_NOISE=fry SIM_NOISE_OFS=11 offload run --host pgx93 -- <mppt-sim> wx-shade-learn
```
