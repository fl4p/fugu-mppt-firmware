*this document is an LLM generated placeholder*

# Partial peak scan: substring share and probe placement (2026-11-24)

Simulator only, no device contact. Worktree branch `worktree-agent-a925f52b7b4e70cac` on main 10637be2.

## Question

`PeakScan::update()` estimates one substring's voltage as `sh = 0.5 (Vin/m0 + vs0)` (`vs0 = kVmp Voc / n`,
`m0 = round(Vin / vs0)`) and probes each candidate at `0.97 m sh`. A docs model (fig_pv: 6 substrings, 2 at 40 %)
put the m = 4 probe from the m0 = n maximum 6 % above its target, reading about 3 % low; a probe must beat the
reference by 3 %. How far do probes land from their maxima across shading cases, what does it cost in moves and
harvest, and does a change help? Candidates: A (vs0 alone when m0 = n), B (lower 0.97 to 0.94 / 0.92), C (weight
0.25/0.75 towards vs0), plus others found on the way.

## Result

* The docs-model premise does not hold on the exact series-string plant. From the high-voltage maximum (m0 = n) the
  probe for a better lower maximum lands at a median +2.5 % (worst +8 %; read 0.95..1.0 of the true gain), because
  the lower maximum also sits above its m unshaded shares (strong substrings carry the weak current there too).
  A, B and C pull those probes low and lose hits.
* The real placement error is in the other direction: probes **above** the operating point (m > m0) land
  6-11 % **below** their maximum and read 5-10 % low. A maximum above ours always includes substrings weaker than
  any at the operating point; that lifts it above m shares, and below it the power falls about in proportion to the
  voltage (the weak substring limits the current), so a -10 % placement reads -10 %. Upward moves with a gain below
  about 10 % were missed until the next global sweep (wrong-peak 1360-1470 s of 1800 s in l1, l2, l4, l5).
* **Superseded by round 2 (below): the firmware uses 1.015, not 1.03.** A review found 1.03 overshoots higher
  maxima after heating; round 2 re-ran everything with fixed tooling and picked 1.015.
* First round, adopted then (D6): probes above m0 at `1.03 m sh`, below m0 unchanged at `0.97 m sh`, share unchanged. Closed loop:
  +5.3 points harvest in l2/l4/l5, +2.0..+3.5 in l1/l3/l1d, +1.2 in l2k, no cell worse than -0.06 (a6, extra
  near-miss rescans), mean over 24 cells +1.04 points. A, B94, C75 lose -0.19..-0.34 points on average, up to
  -6.7 points in single runs (a5, a6, shade-walk), because a smaller share also shrinks the current bound
  `m sh iEnv` that decides whether a lower maximum is probed at all ("G not probed").

## Setup

* Plant: the sim's exact series string (`setPvExact`), fry6 geometry (2 modules x 3 substrings, bypass diode
  0.45 V, Voc 80 V, module Vmpp/Voc `k` 0.86 unless SIM_K), board plants `SIM_RING=fry|flat` with their measured
  sensor noise `SIM_NOISE=fry|flat`, offsets 0, 13, 29, 41, 57 s: 10 paired realisations per cell and variant.
* `pv_substrings=6` in tracker.conf for every run (scans need n).
* Host: **vup.fabi.me** for every closed-loop run (pgx93 was fully booked at the start; no mixing). Static check
  on the Mac (deterministic, no noise).
* Harvest: the TRACKER line (plant PV energy / reachable GMPP energy over the run, 2 decimals).
* Per-scan diagnostics (new `== PROBE` line): the plant's maxima at the scan start, each probe's basin, its offset
  from that basin's maximum and its static power ratio to the operating maximum; outcome ok (no move, gain <= 3 %),
  HIT (moved to the global), MISS-place (stayed, the best probe in the global basin read <= 3 % static: placement),
  MISS-noise (stayed though the static read was > 3 %), WRONG / PARTIAL (moved to a worse / better-but-not-global
  maximum), light (light-change abort).

### Variants

The study ran every variant closed loop through `SIM_SCAN_*` knobs in the sim stubs (share weight `W`/`WN`/`WHI`,
`m = n` probe `HI`/`HIVS`). When D6 landed, the firmware kept only the two probe factors as overridable statics
(`PeakScan::probeLow`/`probeHigh`), set by `SIM_SCAN_LOW` / `SIM_SCAN_UP` in `mppt-sim.cpp`; the share and `HI`
knobs were dropped, so A, A2, C75, r0, vs and D1-D4 now reproduce in `scan-static` only (it computes every variant
itself).

| Name | Rule | Env (closed loop, sim default is D6) |
|---|---|---|
| base | `sh` = mean, all probes `0.97 m sh` (firmware before this change) | `SIM_SCAN_UP=0.97` |
| A | `sh = vs0` when m0 = n | static only now |
| A2 | `sh = vs0` when Vin/m0 > vs0 | static only |
| B94 / B92 | all probes at 0.94 / 0.92 (B92 static only) | `SIM_SCAN_LOW=0.94 SIM_SCAN_UP=0.94` |
| C75 | `sh = 0.25 Vin/m0 + 0.75 vs0` | static only now |
| r0 / vs | `sh = Vin/m0` / `sh = vs0` | static only |
| D1 / D2 / D4 | m = n probe at `n vs0` / `n sh` / `1.03 n sh` | static only now |
| **D6** | probes with m > m0 at `1.03 m sh`, below at `0.97 m sh` | default |

### Scenario matrix (`wx-shade-gen`, 1800 s, 300 W clear-sky string)

`SIM_SHADE=k,s,mode[,k2,s0]`: the last k substrings at s, k2 more at (1+s)/2; `a` = shade comes in over
120..420 s after an unshaded boot sweep (operating point stays on the high-voltage maximum, m0 = n), `r` = ramps
from s0 to s (boot sweep shaded, operating point on a lower maximum, m0 < n), `l` = lifts to 1.

| Cell | Shade | Start | Final global vs operating maximum (static) |
|---|---|---|---|
| a1 | 1 at 0.70, a | m0 = 6 | lower max +6 % |
| a2 | 1 at 0.62, a | m0 = 6 | +18 % |
| a3 | 2 at 0.55, a | m0 = 6 | +8 % |
| a4 | 2 at 0.40, a (docs figure case) | m0 = 6 | +47 % |
| a5 | 3 at 0.40, a | m0 = 6 | +11 % (m = 3) |
| a6 | 2 at 0.45 + 1 at 0.725, a | m0 = 6 | +4.6 % (m = 4 with a weak substring) |
| l1 | 1: 0.60 -> 0.80 | m0 = 5 | full string +6 % |
| l2 | 2: 0.45 -> 0.65 | m0 = 4 | +8 % |
| l3 | 2: 0.45 -> 0.72 | m0 = 4 | about +19 % |
| l4 | 1: 0.35 -> 0.55, + 1 at (1+s)/2 | m0 = 4 | m = 5 max +7 % |
| l5 | 2: 0.30 -> 0.45, + 1 at (1+s)/2 | m0 = 3 | +8 % |
| l6 | 2 at 0.45, lifts to 1 | m0 = 4 | unshaded |
| l1d / a3d | l1 / a3 with Voc -6 % over the run (heating, `SIM_VOCDRIFT=-0.06`) | | |
| l2k / a3k | l2 / a3 with module Vmpp/Voc 0.80 (`SIM_K=0.80`) | | |

Plus the existing wx-shade2-move, wx-shade3-move, wx-shade-walk, wx-shade-cloud, wx-sweep-shade, wx-shade2, wx-shade3,
wx-cloud.

## Static placement (`mppt-sim scan-static`, no closed loop)

Grid: module k 0.80 / 0.86; Voc now vs at the sweep +2 / 0 / -4 / -8 %; k = 1..4 substrings at s = 0.30..0.95, with
and without one more at (1+s)/2; every reachable maximum as the operating point (1364 cases per variant); kVmp
learned from the unshaded sweep. Counts: moves to the global maximum / cases where the global is > 3 % better.

| Variant | m0 = n, gain 3-10 % | m0 = n, > 10 % | m0 < n, 3-10 % | m0 < n, > 10 % | wrong / partial (all cases) | global-probe dV median / worst, m0 = n | m0 < n | lowest read |
|---|---|---|---|---|---|---|---|---|
| base | 25/35 | 167/178 | 22/76 | 226/240 | 0/27 | +2.5 / -12.8 % | -6.5 / -12.0 % | 0.880 |
| A | 17/35 | 158/178 | 22/76 | 226/240 | 0/37 | -1.1 / -10.5 % | -6.5 / -12.0 % | 0.898 |
| A2 | 18/35 | 158/178 | 21/76 | 224/240 | 0/36 | -1.1 / -10.5 % | -6.8 / -12.1 % | 0.897 |
| B94 | 19/35 | 166/178 | 15/76 | 211/240 | 0/24 | -0.7 / -7.4 % | -9.3 / -13.8 % | 0.877 |
| B92 | 15/35 | 156/178 | 15/76 | 199/240 | 0/28 | -2.8 / -9.3 % | -11.1 / -15.7 % | 0.859 |
| C75 | 21/35 | 164/178 | 29/76 | 226/240 | 0/30 | +0.6 / +9.1 % | -6.2 / -12.1 % | 0.892 |
| r0 (Vin/m0) | 31/35 | 156/178 | 17/76 | 220/240 | 0/43 | +4.2 / -18.4 % | -7.2 / -12.3 % | 0.846 |
| vs (vs0) | 17/35 | 158/178 | 33/76 | 227/240 | 0/34 | -1.1 / -10.5 % | -5.9 / -12.1 % | 0.897 |
| D2 | 25/35 | 167/178 | 39/76 | 235/240 | 0/18 | +2.5 / -12.8 % | -3.9 / -10.0 % | 0.880 |
| D4 | 25/35 | 167/178 | 48/76 | 236/240 | 0/17 | +2.5 / -12.8 % | -1.1 / -10.0 % | 0.880 |
| **D6** | 25/35 | 167/178 | **61/76** | 234/240 | 0/20 | +2.5 / -12.8 % | -0.8 / +7.1 % | 0.880 |

By Voc drift (all should-move cases, base -> D6): +2 % 122 -> 137 of 150, 0 % 126 -> 140 of 149, -4 % 102 -> 114 of
120, -8 % 90 -> 96 of 110. With kVmp not learned (0.82 default, `SIM_STATIC_KDEF=1`): m0 < n 3-10 % 18 -> 55 of 72,
no column worse. D3/D5 (m = n probe at 1.03 n vs0 / 1.06 n sh) and D7 (up 1.0) were also tried: D3 and D5 fall to
70-89 of 110 at -8 % drift (steep side), D7 is between base and D6.

The remaining m0 = n misses (10 of 35, every variant) are global maxima within 3-5 % whose probe reads 0.4-1.8 %
low (threshold), or downward targets that themselves contain a weak substring (a6: m = 4 at 47.4 V, probe 5 % low).

## Closed loop (vup.fabi.me, 10 paired realisations per cell)

Harvest delta vs base in points of the reachable GMPP: mean (min..max). Full tables incl. standard errors and per
variant probe counts: `doc/research/scan-share/closed-loop-tables.md`.

| Cell | base harvest | **D6** | D4 | A | B94 | C75 |
|---|---|---|---|---|---|---|
| a1 | 99.01 | +0.00 (-0.04..+0.06) | +0.00 | -0.13 (-0.58..+0.04) | -0.00 | +0.01 |
| a2 | 99.49 | -0.01 (-0.04..+0.04) | -0.01 | -0.94 (-1.90..+0.02) | -0.00 | -0.75 (-1.92..+0.02) |
| a3 | 98.76 | -0.01 (-0.03..+0.02) | -0.00 | -0.01 | -0.01 | -0.01 |
| a4 | 98.94 | +0.00 (-0.02..+0.04) | -0.01 | -0.26 (-0.66..+0.05) | -0.12 (-0.73..+0.07) | -0.12 |
| a5 | 99.29 | -0.00 (-0.04..+0.07) | -0.00 | -3.33 (-6.69..+0.04) | -4.44 (-6.71..+0.00) | -3.31 (-6.69..+0.07) |
| a6 | 98.41 | **-0.06** (-0.16..+0.01) | -0.07 | -2.10 (-2.98..+0.06) | -1.86 (-3.00..+0.07) | -1.47 (-2.98..+0.06) |
| l1 | 95.25 | **+1.98** (+0.00..+3.23) | +1.98 | +0.00 | +0.02 | +0.01 |
| l2 | 93.44 | **+5.32** (+5.30..+5.34) | +5.31 | +0.00 | -0.02 | -0.00 |
| l3 | 97.38 | **+2.11** (+2.08..+2.13) | +2.11 | -0.00 | +0.00 | -0.01 |
| l4 | 94.03 | **+5.31** (+5.10..+5.36) | -0.00 | +0.00 | +0.04 | +0.00 |
| l5 | 93.81 | **+5.42** (+0.02..+6.21) | -0.01 | +0.00 | +0.00 | -0.03 |
| l6 | 99.22 | +0.00 (-0.01..+0.02) | -0.01 | +0.00 | -0.02 | -0.01 |
| l1d | 95.26 | **+3.48** (+2.97..+3.78) | +3.48 | +0.00 | -0.00 | +0.00 |
| a3d | 98.77 | +0.01 (-0.02..+0.04) | +0.00 | +0.00 | +0.01 | +0.00 |
| l2k | 98.35 | **+1.23** (+0.32..+1.48) | +1.24 | +0.02 | -0.51 (-1.63..+0.02) | +0.92 |
| a3k | 99.16 | **-0.03** (-0.09..+0.02) | -0.04 | +0.11 (-0.09..+0.73) | +0.12 (-0.05..+0.70) | +0.11 |
| wx-shade2-move | 99.13 | -0.01 (-0.03..+0.01) | -0.01 | -0.11 (-0.31..+0.01) | -0.00 | -0.01 |
| wx-shade3-move | 99.34 | +0.00 (-0.04..+0.02) | -0.01 | -0.01 | -0.01 | -0.00 |
| wx-shade-walk | 96.97 | +0.18 (-0.07..+0.56) | +0.18 | -1.06 (-2.85..+0.06) | -1.51 (-5.43..+0.06) | -0.20 (-0.87..+0.08) |
| wx-shade-cloud | 96.86 | +0.00 (identical) | +0.00 | +0.08 (-0.59..+0.85) | +0.07 (-0.48..+0.93) | +0.20 (-0.36..+1.12) |
| wx-sweep-shade | 99.27 | +0.02 (+0.01..+0.03) | +0.02 | +0.00 | -0.02 | -0.00 |
| wx-shade2 / wx-shade3 | 99.50 / 99.45 | +0.01 / -0.00 | | | | |
| wx-cloud | 99.00 | +0.00 (identical) | +0.00 | +0.03 | +0.02 | +0.04 |
| **mean of 24 cells** | | **+1.04** | +0.59 | -0.32 | -0.34 | -0.19 |

D2 (m = n probe at n sh): +0.15 mean, helps l3 and l2k only.

### Moves and placement, base -> D6 (sums over 10 runs)

| Cell | wrong-peak s (mean) | scans / run | HIT | MISS-place | MISS-noise | global-probe offset | read / true gain |
|---|---|---|---|---|---|---|---|
| l1 | 1406 -> 797 | 21.0 -> 11.3 | 0 -> 8 | 200 -> 95 | 0 -> 0 | -10.2 -> -4.6 % | 0.908 -> 0.965 |
| l2 | 1401 -> 241 | 21.0 -> 5.0 | 0 -> 10 | 200 -> 0 | 0 | -10.9 -> -5.2 % | 0.908 -> 0.965 |
| l3 | 301 -> 63 | 6.0 -> 6.0 | 10 -> 10 | 0 | 0 | -10.1 -> -5.0 % | 0.917 -> 0.969 |
| l4 | 1470 -> 138 | 21.0 -> 7.1 | 0 -> 10 | 200 -> 10 | 0 -> 1 | -10.2 -> -4.4 % | 0.910 -> 0.968 |
| l5 | 1359 -> 155 | 21.9 -> 6.5 | 0 -> 9 | 214 -> 14 | 0 -> 1 | -10.2 -> -4.7 % | 0.911 -> 0.968 |
| l1d | 1405 -> 311 | 21.0 -> 9.5 | 0 -> 10 | 200 -> 5 | 0 -> 8 | -8.3 -> -3.5 % | 0.927 -> 0.975 |
| l2k | 247 -> 45 | 6.2 -> 6.0 | 10 -> 10 | 2 -> 0 | 0 | -8.3 -> -3.3 % | 0.948 -> 0.990 |
| a6 | 442 -> 442 | 11.8 -> 16.8 | 8 -> 8 | 71 -> 71 | 0 | -4.3 % (both) | 0.971 |
| a3k | 257 -> 257 | 6.1 -> 8.7 | 10 -> 10 | 12 -> 12 | 0 | -2.5 % (both) | 0.996 |
| a1-a5, a3d, wx-shade2-move | unchanged | unchanged | unchanged | | | +0.3..+1.8 % | 0.995-0.999 |

No WRONG or PARTIAL move in any gen cell for any variant. wx-shade-walk counts 7 / 4 "wrong" (base / D6): the shade
moves during those scans, so the start-of-scan truth is stale there.

## Regressions and their mechanism

* **A, B, C**: the share also sets the current bound for lower candidates (`m sh iEnv < P (1 + margin)` skips
  them). A smaller share skips a lower maximum that is better (a5: m = 3 at 33 V, +11 %, never probed under A/B94/C75
  in 5-6 of 10 runs; a6 likewise), and lower placement drops reads below the 0.94 near-miss rescan (a2: the move at
  364 s happens at 600 s instead). Static and closed loop agree: none of A, B, C should be adopted.
* **D6**: the higher upward probe reads closer to the start power, so the near-miss rescan (best probe > 0.94) fires
  more often when the higher maximum is just worse: a6 11.8 -> 16.8 scans per 1800 s, a3k 6.1 -> 8.7, costing
  0.03-0.06 points. Static worst upward overshoot +7.1 % (onto the steep side) still reads >= 0.88, no wrong move.
  The full lift (l6) and the unshaded single maximum (wx-cloud) are unchanged.

## Not tested / limitations

* No hardware or field data; the exact series-string plant (fixed bypass-diode drop, no mismatch within a substring)
  sets every number here. Field sweeps (2026-11-15/16) place shaded global maxima at 0.85-0.94 Voc, consistent with
  the plant's high-voltage maxima, but there is no field shade event with a known global peak to replay.
* Only fry6 geometry (n = 6, Voc 80 V), 300 W, one battery voltage. n = 2..5 strings, other Voc/Vout ratios and the
  Vin floor touching low probes were only covered where the grid reached them (k = 4 substrings shaded).
* Voc drift +2 % (Voc rising since the sweep) and -8 % only statically; closed loop only -6 %.
* Downward targets with a weak substring (a6-like) still miss; no variant tested here fixes them without hurting
  the all-strong targets.
* On-target Unity run of `test/test_tracker.cpp` not done (the test image builds). The scan tests were compiled on
  the host against a minimal Unity shim: the new `test_scan_probe_above_weak_peak` passes with the change and fails
  with the old 0.97 rule. `test_scan_timeout_returns_to_start` failed on main without the change: a test bug since
  the near probes (ab966532). The first scan times out correctly (d0 = start duty), but the rig does not return to
  d0 as `mppt.cpp` does, the Ref phase settles only within 1 % after the near probes, and a second scan 5 s after
  the timeout overwrote d0 before the assert. The test now checks d0 and pRef at the first timeout.

## Files

* `src/peak_scan.h`: probe rule (`probeLow` 0.97 below m0, `probeHigh` 1.015 above since round 2; 1.03 in round 1).
* `test/host-stub/mppt-sim.cpp`: `wx-shade-gen`, `scan-static`, `== PROBE` / `== PROBES` lines, `SIM_SCAN_LOW` /
  `SIM_SCAN_UP`.
* `test/test_tracker.cpp`, `test/main.cpp`: `test_scan_probe_above_weak_peak`.
* `website/docs/internals/mppt-tracker.md`, `website/scripts/plots/mppt_tracker.py`: the probe rule (figure
  unchanged, it only probes below m0).
* `doc/research/scan-share/`: `job.sh`, `agg.py`, `static_agg.py`, `results.txt` (raw result lines, 1680 runs),
  `closed-loop-tables.md`.
* Round 2: `doc/research/scan-share/static_cmp.py`, `results-2.txt` (2240 lines, pgx93), `closed-loop-tables-2.md`.

## Reproduce

```bash
T=/tmp/scanshare-tree; rm -rf $T; mkdir -p $T; rsync -a src test config $T/      # offload cannot mirror /Users paths
OUT=/tmp/scanshare-bin $T/test/host-stub/mppt-sim.sh none >/dev/null; ~/bin/offload build-sim $T /tmp/scanshare-bin
cd doc/research/scan-share
for c in a1 a2 a3 a4 a5 a6 l1 l2 l3 l4 l5 l6 l1d a3d l2k a3k wx-shade2-move wx-shade3-move wx-shade-walk \
         wx-shade-cloud wx-sweep-shade wx-shade2 wx-shade3 wx-cloud; do for b in fry flat; do
  for o in 0 13 29 41 57; do for v in base D6 B94; do echo "$c $v $b $o"; done; done; done; done > /tmp/jobs.txt
HOST=vup.fabi.me xargs -P 14 -I{} ./job.sh "{}" < /tmp/jobs.txt > /tmp/results.txt   # ~2 h on vup (14 slots)
python3 agg.py /tmp/results.txt
/tmp/scanshare-bin scan-static > /tmp/static.txt && python3 static_agg.py /tmp/static.txt   # static grid, seconds
```

The 2026-11-24 runs used two binaries built from the tree before and after the `SIM_SCAN_UP` knob (base outputs
byte-identical between them), with the share and `HI` knobs that were dropped at landing (A, C75, D2, D4 closed loop
no longer reproduce). The landed tree with `SIM_SCAN_UP=0.97` matches main before the change byte for byte (wx-shade2,
wx-shade-walk, `== PROBE` lines aside), and its default matches the study binary's D6 (l2, fry, offset 0).


## Round 2: probe factor above the operating point (2026-11-24, evening)

An independent review of round 1 found three problems: (1) after heating (Voc 8 % below the sweep's Voc, module
Vmpp/Voc 0.86) a probe at 1.03 m sh overshoots a higher maximum onto its steep side and misses a move 0.97 found
(static grid: D6 gains 49 cases, loses 2; D7 gains 30, loses 0); (2) the `== PROBE` outcome read the maxima and
basins at the scan start but the probe powers on the plant at the scan end, and counted HIT / WRONG from the
"moving" log before the move had completed; (3) `job.sh` cached outputs by cell / variant / board / offset only, so a
changed binary, scenario or host reused old results. Round 2 fixes the tooling, extends the grid and re-runs
everything.

### Tooling changes

* `job.sh`: output file `<cell>.<var>.<board>.<ofs>.<sha256(BIN)[:12]>.<host>.txt`, offload tag `TAG`; variants
  `uNNNN` (SIM_SCAN_UP = NNNN/1000), `D6` now sets 1.03 explicitly; heating cells h1-h4. Fresh OUTDIR
  `/tmp/scanf-out`.
* `== PROBE`: maxima, basins and the probe powers (`x` ratios) all from the plant at the scan start; the outcome
  from the plant 5 s after the scan went idle (or when a sweep or the next scan starts): HIT when it moved and the
  landed basin is the global one, PARTIAL / WRONG when it moved to another basin better / not better than the start
  basin, else ok / MISS by the global gain at landing. The line adds `landed <V>, G now <V> x<gain>`.
  **The round 1 per-probe counters (HIT / WRONG / MISS columns above) have the old limitation**: they mixed scan-start
  maxima with scan-end probe powers and did not check where the move landed. The harvest numbers are unaffected
  (plant energy).
* `scan-static`: n = 4, 5, 6 substrings (string Voc 80 V for all; `SIM_STATIC_N`), Voc drift +2 / 0 / -4 / -8 / -12 %,
  factor variants u0.980 ... u1.050 (D7 = 1.000, D6 = 1.030), `SIM_STATIC_VARS` filter, a `land <V> x<P/PG>` field
  (the maximum it stays on or moves to, relative to the global). `static_cmp.py`: per-case gains / losses vs base.
  The n = 6, drift >= -8 % subset reproduces round 1 exactly (D6 +49 / -2, D7 +30 / -0, same two cases).
* `wx-shade-gen`: `SIM_SHADE=k,s,mode,k2,s0,tr` (ramp / hold time), modes `u` (unshaded cold boot sweep, step to s0
  at 120 s, to s at 120 + tr) and `v` (as u, k2 substrings unshaded until 120 + tr); `SIM_VOCDRIFT=d,t1,t0` (drift
  from t0 to t1).

### Static grid (Mac, deterministic; 4459 cases per variant)

Gain / loss: the maximum the case lands on is > 0.2 % (of the global) better / worse than under base (0.97).

By Voc drift since the sweep:

| variant | +2 % | +0 % | -4 % | -8 % | -12 % | all |
|---|---|---|---|---|---|---|
| u0.980 | +6 / -0 | +7 / -0 | +7 / -0 | +3 / -0 | +5 / -0 | +28 / -0 |
| u0.990 | +14 / -0 | +14 / -0 | +9 / -0 | +10 / -0 | +6 / -0 | +53 / -0 |
| D7 | +21 / -0 | +21 / -0 | +12 / -0 | +14 / -0 | +7 / -1 | +75 / -1 |
| u1.005 | +24 / -0 | +23 / -0 | +15 / -0 | +15 / -0 | +9 / -4 | +86 / -4 |
| u1.010 | +28 / -0 | +24 / -0 | +18 / -0 | +15 / -0 | +10 / -7 | +95 / -7 |
| u1.015 | +30 / -0 | +26 / -0 | +22 / -0 | +15 / -0 | +10 / -9 | +103 / -9 |
| u1.020 | +31 / -0 | +27 / -0 | +24 / -0 | +15 / -0 | +11 / -13 | +108 / -13 |
| D6 | +34 / -0 | +33 / -0 | +25 / -0 | +12 / -5 | +13 / -24 | +117 / -29 |
| u1.040 | +40 / -0 | +40 / -0 | +25 / -0 | +10 / -14 | +14 / -32 | +129 / -46 |
| u1.050 | +47 / -0 | +41 / -0 | +22 / -2 | +7 / -29 | +16 / -37 | +133 / -68 |

By n:

| variant | n = 4 | n = 5 | n = 6 | all |
|---|---|---|---|---|
| u0.980 | +6 / -0 | +8 / -0 | +14 / -0 | +28 / -0 |
| u0.990 | +12 / -0 | +18 / -0 | +23 / -0 | +53 / -0 |
| D7 | +14 / -1 | +27 / -0 | +34 / -0 | +75 / -1 |
| u1.005 | +17 / -1 | +30 / -3 | +39 / -0 | +86 / -4 |
| u1.010 | +17 / -2 | +33 / -5 | +45 / -0 | +95 / -7 |
| u1.015 | +18 / -2 | +35 / -6 | +50 / -1 | +103 / -9 |
| u1.020 | +17 / -3 | +38 / -8 | +53 / -2 | +108 / -13 |
| D6 | +18 / -8 | +40 / -17 | +59 / -4 | +117 / -29 |
| u1.040 | +19 / -13 | +44 / -24 | +66 / -9 | +129 / -46 |
| u1.050 | +18 / -19 | +45 / -36 | +70 / -13 | +133 / -68 |

By kp:

| variant | kp 0.80 | kp 0.86 | all |
|---|---|---|---|
| u0.980 | +10 / -0 | +18 / -0 | +28 / -0 |
| u0.990 | +24 / -0 | +29 / -0 | +53 / -0 |
| D7 | +31 / -0 | +44 / -1 | +75 / -1 |
| u1.005 | +38 / -1 | +48 / -3 | +86 / -4 |
| u1.010 | +41 / -1 | +54 / -6 | +95 / -7 |
| u1.015 | +46 / -1 | +57 / -8 | +103 / -9 |
| u1.020 | +49 / -2 | +59 / -11 | +108 / -13 |
| D6 | +57 / -4 | +60 / -25 | +117 / -29 |
| u1.040 | +64 / -8 | +65 / -38 | +129 / -46 |
| u1.050 | +65 / -13 | +68 / -55 | +133 / -68 |

Heated cases per n:

| variant | n = 4, -8 % | n = 4, -12 % | n = 5, -8 % | n = 5, -12 % | n = 6, -8 % | n = 6, -12 % |
|---|---|---|---|---|---|---|
| u0.980 | +0 / -0 | +1 / -0 | +0 / -0 | +1 / -0 | +3 / -0 | +3 / -0 |
| u0.990 | +1 / -0 | +1 / -0 | +2 / -0 | +2 / -0 | +7 / -0 | +3 / -0 |
| D7 | +1 / -0 | +1 / -1 | +5 / -0 | +2 / -0 | +8 / -0 | +4 / -0 |
| u1.005 | +2 / -0 | +0 / -1 | +5 / -0 | +3 / -3 | +8 / -0 | +6 / -0 |
| u1.010 | +2 / -0 | +0 / -2 | +5 / -0 | +3 / -5 | +8 / -0 | +7 / -0 |
| u1.015 | +2 / -0 | +0 / -2 | +5 / -0 | +3 / -6 | +8 / -0 | +7 / -1 |
| u1.020 | +1 / -0 | +0 / -3 | +5 / -0 | +3 / -8 | +9 / -0 | +8 / -2 |
| D6 | +0 / -0 | +0 / -8 | +4 / -3 | +3 / -14 | +8 / -2 | +10 / -2 |
| u1.040 | +0 / -1 | +0 / -12 | +3 / -6 | +3 / -18 | +7 / -7 | +11 / -2 |
| u1.050 | +0 / -5 | +0 / -14 | +2 / -14 | +3 / -21 | +5 / -10 | +13 / -2 |

Every loss of every factor is at drift <= -8 %; with drift >= -4 % no factor loses a case. Only 0.98 and 0.99 have
no loss at all; 1.00 loses one case at -12 % (n 4). 1.015 loses none at -8 % and these 9 at -12 % (all HIT under base):

```
n 4 kp 0.86 drift -0.12 k 1 k2 1 s 0.60 op 1/3 m0 3 gT 1.042: HIT -> MISS    (G probe dV -1.2 -> +3.4 %, read 0.996 -> 0.931)
n 4 kp 0.86 drift -0.12 k 1 k2 1 s 0.65 op 1/3 m0 3 gT 1.093: HIT -> MISS    (-0.8 -> +3.8 %, 0.998 -> 0.919)
n 5 kp 0.80 drift -0.12 k 1 k2 1 s 0.60 op 0/3 m0 3 gT 1.208: HIT -> PARTIAL x0.998 (-0.5 -> +4.1 %)
n 5 kp 0.86 drift -0.12 k 1 k2 1 s 0.40 op 0/3 m0 3 gT 1.042: HIT -> MISS    (-2.1 -> +2.5 %, 0.991 -> 0.969)
n 5 kp 0.86 drift -0.12 k 1 k2 1 s 0.60 op 0/3 m0 3 gT 1.172: HIT -> PARTIAL x0.975 (-0.7 -> +3.9 %)
n 5 kp 0.86 drift -0.12 k 1 k2 1 s 0.70 op 1/3 m0 4 gT 1.069: HIT -> MISS    (-0.8 -> +3.8 %, 0.997 -> 0.913)
n 5 kp 0.86 drift -0.12 k 1 k2 1 s 0.75 op 1/3 m0 4 gT 1.111: HIT -> MISS    (-0.5 -> +4.1 %, 0.999 -> 0.906)
n 5 kp 0.86 drift -0.12 k 2 k2 1 s 0.45 op 1/3 m0 3 gT 1.068: HIT -> MISS    (-1.0 -> +3.6 %, 0.998 -> 0.944)
n 6 kp 0.86 drift -0.12 k 2 k2 1 s 0.55 op 1/3 m0 4 gT 1.096: HIT -> MISS    (-0.9 -> +3.7 %, 0.998 -> 0.933)
```

The two round 1 regressions (n 6, kp 0.86, -8 %, k 1 k2 1, s 0.65 op 0/3 and s 0.75 op 1/3) are HIT under 1.015.

### Closed loop (pgx93 only, 10 paired realisations, fresh outputs)

Binary sha256 `377f519ec794...` (this tree with `SIM_SCAN_UP` per variant), built with `offload build-sim`, every
run on **pgx93** (`--host pgx93`, tags scanf-1 / scanf-2 / scanf-3; two runs that failed on ssh were re-run on pgx93).
28 cells: the 24 of round 1 plus four heating cells reproducing the static regressions of 1.03 in closed loop:

| Cell | Shade | Heating |
|---|---|---|
| h1 | unshaded boot sweep; 120 s: 1 at 0.20 + 1 at 0.60 (op moves to m = 4); 720 s: 1 at 0.65 + 1 at 0.825 | Voc -8 % over 420..700 s |
| h2 | as h1 with 1 at 0.30, the other unshaded (op to m = 5); 720 s: 1 at 0.75 + 1 at 0.875 | -8 % |
| h3 / h4 | h1 / h2 | -12 % |

With the shade ramping in (mode r) or a shaded boot sweep, closed loop moved before the overshoot case arose, and the
learned Vmpp/Voc (0.82 default after a shaded sweep) put the probes lower than the static grid; the cold unshaded
sweep and the step shade were needed to reach the static cases. Even so, closed-loop probe placement sits about
2-3 % lower than the static grid's for the same case (h1: static 1.03 probe +3.1 %, closed loop +0.5 %), which is why
1.00 / 0.99 gain little in closed loop (l2, l4, l1d) though they gain statically.

Harvest delta vs base (0.97), points of reachable GMPP energy, mean (min..max) over the 10 pairs:

| Cell | base harvest | u0990 | u1000 | u1010 | u1015 | u1020 | u1025 | D6 |
|---|---|---|---|---|---|---|---|---|
| a1 | 99.02 | +0.01 (-0.02..+0.03) | +0.00 (-0.02..+0.03) | -0.00 (-0.01..+0.02) | +0.00 (-0.02..+0.02) | +0.01 (-0.02..+0.03) | +0.01 (-0.02..+0.03) | +0.00 (-0.02..+0.03) |
| a2 | 99.52 | -0.00 (-0.02..+0.01) | -0.00 (-0.03..+0.02) | -0.00 (-0.03..+0.03) | +0.00 (-0.01..+0.02) | -0.00 (-0.03..+0.02) | -0.00 (-0.04..+0.01) | -0.01 (-0.04..+0.01) |
| a3 | 98.77 | -0.01 (-0.03..+0.03) | +0.00 (-0.03..+0.04) | -0.00 (-0.03..+0.04) | -0.00 (-0.03..+0.04) | -0.01 (-0.06..+0.03) | +0.00 (-0.02..+0.04) | -0.00 (-0.04..+0.05) |
| a4 | 98.89 | -0.00 (-0.03..+0.03) | +0.01 (-0.02..+0.03) | +0.00 (-0.03..+0.02) | -0.00 (-0.03..+0.03) | -0.01 (-0.05..+0.02) | -0.01 (-0.02..+0.02) | -0.00 (-0.02..+0.02) |
| a5 | 99.29 | -0.00 (-0.11..+0.11) | +0.00 (-0.07..+0.09) | +0.00 (-0.08..+0.08) | -0.00 (-0.07..+0.10) | -0.00 (-0.09..+0.07) | +0.00 (-0.04..+0.06) | +0.00 (-0.08..+0.09) |
| a6 | 98.04 | -0.00 (-0.01..+0.00) | +0.00 (-0.01..+0.01) | +0.00 (+0.00..+0.01) | +0.00 (+0.00..+0.01) | -0.01 (-0.04..+0.01) | -0.05 (-0.17..+0.00) | -0.06 (-0.17..+0.01) |
| l1 | 95.25 | +0.00 (-0.02..+0.05) | +0.01 (-0.01..+0.04) | +0.01 (-0.01..+0.03) | +0.01 (-0.02..+0.04) | +0.01 (-0.02..+0.05) | +0.62 (-0.01..+1.85) | +1.25 (+0.00..+2.87) |
| l2 | 93.43 | +0.01 (-0.02..+0.04) | +0.00 (-0.03..+0.05) | +2.50 (+0.01..+5.35) | +4.39 (+2.57..+5.34) | +4.93 (+4.19..+5.31) | +5.27 (+5.03..+5.36) | +5.32 (+5.30..+5.36) |
| l3 | 97.39 | +1.93 (+1.59..+2.11) | +2.08 (+2.04..+2.10) | +2.08 (+2.04..+2.11) | +2.09 (+2.04..+2.13) | +2.09 (+2.07..+2.12) | +2.10 (+2.06..+2.14) | +2.10 (+2.08..+2.12) |
| l4 | 94.03 | -0.01 (-0.04..+0.03) | -0.01 (-0.07..+0.02) | +1.02 (-0.07..+3.43) | +3.54 (-0.02..+5.32) | +4.32 (+2.90..+5.33) | +5.12 (+4.13..+5.36) | +5.28 (+5.08..+5.38) |
| l5 | 93.80 | +0.01 (-0.07..+0.07) | +1.79 (-0.11..+5.41) | +4.74 (+0.02..+5.73) | +5.31 (+0.01..+6.00) | +5.33 (+0.01..+6.00) | +5.36 (+0.02..+5.99) | +5.41 (+0.02..+6.19) |
| l6 | 99.22 | +0.00 (-0.01..+0.02) | +0.00 (-0.02..+0.03) | +0.00 (-0.01..+0.02) | +0.01 (-0.01..+0.02) | -0.00 (-0.02..+0.01) | +0.01 (+0.00..+0.02) | -0.00 (-0.02..+0.01) |
| l1d | 95.26 | +0.00 (-0.02..+0.02) | +0.03 (-0.04..+0.29) | +1.45 (+0.31..+2.57) | +1.98 (+1.22..+2.77) | +2.80 (+2.12..+3.56) | +3.47 (+2.96..+3.77) | +3.57 (+3.12..+3.77) |
| a3d | 98.77 | +0.00 (-0.02..+0.04) | +0.00 (-0.01..+0.04) | +0.01 (-0.01..+0.03) | +0.01 (-0.01..+0.03) | +0.00 (-0.02..+0.04) | +0.01 (-0.02..+0.04) | +0.01 (-0.02..+0.04) |
| l2k | 98.35 | +0.95 (+0.02..+1.45) | +0.96 (+0.03..+1.44) | +1.23 (+0.33..+1.52) | +1.23 (+0.34..+1.50) | +1.24 (+0.33..+1.52) | +1.24 (+0.34..+1.51) | +1.24 (+0.35..+1.52) |
| a3k | 99.09 | -0.01 (-0.07..+0.03) | -0.01 (-0.04..+0.02) | -0.01 (-0.05..+0.04) | +0.01 (+0.00..+0.02) | -0.00 (-0.03..+0.06) | -0.01 (-0.06..+0.03) | -0.02 (-0.11..+0.03) |
| h1 | 98.86 | -0.01 (-0.05..+0.02) | -0.02 (-0.05..+0.01) | -0.01 (-0.04..+0.01) | -0.01 (-0.06..+0.04) | -0.12 (-1.08..+0.01) | -0.34 (-1.11..-0.01) | -0.47 (-1.10..-0.01) |
| h2 | 97.28 | +1.77 (-0.01..+3.56) | +1.77 (-0.01..+3.55) | +1.76 (-0.01..+3.54) | +1.76 (-0.01..+3.55) | +1.76 (-0.01..+3.54) | +1.75 (-0.21..+3.55) | +1.42 (-3.54..+3.56) |
| h3 | 98.28 | +0.06 (-1.04..+1.79) | -0.06 (-1.87..+1.77) | -0.05 (-1.86..+1.74) | -0.19 (-1.82..+1.80) | -0.62 (-1.16..+0.62) | -0.22 (-1.12..+0.60) | -0.90 (-1.81..+0.05) |
| h4 | 98.55 | -0.19 (-1.03..+0.16) | -0.16 (-0.90..+0.51) | -0.56 (-3.64..+0.18) | +0.03 (-0.35..+0.37) | -0.09 (-1.37..+0.87) | -0.72 (-3.51..+0.62) | -0.80 (-3.17..+0.36) |
| wx-shade2-move | 99.12 | +0.00 (-0.01..+0.02) | +0.00 (-0.02..+0.01) | +0.01 (-0.03..+0.04) | +0.01 (-0.01..+0.02) | +0.01 (-0.01..+0.02) | -0.00 (-0.02..+0.01) | +0.01 (-0.01..+0.02) |
| wx-shade3-move | 99.35 | -0.00 (-0.02..+0.02) | -0.00 (-0.01..+0.01) | +0.00 (-0.02..+0.02) | +0.00 (-0.02..+0.02) | -0.00 (-0.05..+0.03) | +0.01 (-0.02..+0.04) | +0.01 (-0.01..+0.03) |
| wx-shade-walk | 97.04 | -0.20 (-0.62..+0.03) | -0.02 (-0.31..+0.53) | -0.23 (-0.88..+0.54) | -0.34 (-2.44..+0.54) | -0.20 (-0.89..+0.03) | -0.08 (-0.88..+0.51) | -0.22 (-2.31..+0.55) |
| wx-shade-cloud | 96.90 | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) |
| wx-sweep-shade | 99.27 | +0.02 (+0.01..+0.02) | +0.02 (+0.01..+0.03) | +0.03 (+0.02..+0.03) | +0.02 (+0.01..+0.04) | +0.03 (+0.01..+0.03) | +0.03 (+0.02..+0.04) | +0.03 (+0.02..+0.04) |
| wx-shade2 | 99.49 | +0.00 (-0.04..+0.04) | +0.01 (-0.02..+0.04) | +0.01 (-0.03..+0.04) | +0.00 (-0.03..+0.05) | +0.00 (-0.02..+0.03) | +0.00 (-0.03..+0.03) | +0.01 (-0.01..+0.04) |
| wx-shade3 | 99.45 | +0.00 (-0.02..+0.02) | +0.00 (+0.00..+0.01) | +0.00 (-0.02..+0.02) | +0.00 (-0.01..+0.02) | -0.00 (-0.02..+0.02) | +0.00 (-0.01..+0.01) | +0.00 (-0.02..+0.02) |
| wx-cloud | 99.02 | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) | +0.00 (+0.00..+0.00) |
| **mean of 28 cells** | | **+0.16** | **+0.23** | **+0.50** | **+0.71** | **+0.77** | **+0.84** | **+0.83** |
| worst cell mean | | -0.20 (wx-shade-walk) | -0.16 (h4) | -0.56 (h4) | -0.34 (wx-shade-walk) | -0.62 (h3) | -0.72 (h4) | -0.90 (h3) |

Standard errors (agg.py, `scan-share/closed-loop-tables-2.md`): h3 1.03 -0.90 se 0.19, h4 1.03 -0.80 se 0.36, h1
1.03 -0.47 se 0.16 (real regressions: 5-6 PARTIAL landings on the m = 6 maximum per 10 runs in h1 / h3). The other
negative cell means are within noise: wx-shade-walk (all factors -0.02..-0.34, single pairs -2.4..+0.6 without
factor dependence; the moving shade makes it event-chaotic), h4 1.01 -0.56 se 0.36, h3 1.015 -0.19 se 0.35. In round
1 (vup) D6 was +0.18 on wx-shade-walk.

Landed outcomes (sum over 10 runs, new classification) for the cells that move:

| Cell | factor | wrong-peak s | scans / run | HIT | MISS-place | MISS-noise | PARTIAL |
|---|---|---|---|---|---|---|---|
| l2 | 0.97 / 1.015 / 1.03 | 1401 / 445 / 241 | 21.0 / 8.0 / 5.0 | 0 / 10 / 10 | 200 / 24 / 0 | 0 / 10 / 0 | 0 |
| l4 | | 1470 / 578 / 144 | 21.0 / 13.0 / 7.2 | 0 / 8 / 10 | 200 / 86 / 11 | 0 / 0 / 1 | 0 |
| l1d | | 1405 / 773 / 281 | 21.0 / 16.5 / 9.5 | 0 / 10 / 10 | 200 / 85 / 4 | 0 / 5 / 4 | 0 |
| h1 | | 32 / 32 / 32 | 11.0 / 16.0 / 15.2 | 20 / 20 / 17 | 0 | 0 | 0 / 0 / 5 |
| h2 | | 546 / 27 / 130 | 13.0 / 6.2 / 7.5 | 15 / 20 / 19 | 90 / 0 / 17 | 0 / 0 / 1 | 0 |
| h3 | | 86 / 86 / 142 | 15.0 / 11.6 / 11.4 | 20 / 19 / 17 | 1 / 0 / 2 | 0 | 0 / 2 / 6 |
| h4 | | 136 / 113 / 356 | 6.5 / 6.1 / 7.4 | 20 / 20 / 20 | 5 / 1 / 23 | 0 | 0 |

No WRONG landing in any cell except wx-shade-cloud (8 per variant, identical for all factors: clouds after the move).

### Choice: 1.015

Criterion asked for: maximum mean harvest with no cell mean below -0.1 points vs base and no static regression in the
heated cases. No factor meets it: every factor has a cell mean below -0.1 (wx-shade-walk -0.20 at 0.99, h4 -0.16 at
1.00, noise-level), and every factor above 0.99 has static losses at -12 %. The trade-off:

* 1.03 (round 1): best mean (+0.83) but real heating regressions in closed loop (h1 -0.47, h3 -0.90, h4 -0.80) and
  29 static losses (5 at -8 %).
* 1.025: +0.84 mean, h1 -0.34, h4 -0.72; 1.02: +0.77, h3 -0.62.
* **1.015: +0.71 mean (85 % of 1.03), h1 -0.01, h3 -0.19 (se 0.35), h4 +0.03; static: 0 losses at drift >= -8 % for
  n = 4..6, 9 losses vs 10 gains at -12 %.** It keeps all of l5, l3, l2k and most of l2 (+4.4 of +5.3), l4 (+3.5 of
  +5.3), l1d (+2.0 of +3.6), and drops the extra a6 near-miss rescans of 1.03 (13.7 scans / run as base, 1.03 18.3).
* 1.00 / 0.99: no static loss (0.99) or one (1.00), but closed-loop mean only +0.23 / +0.16: the upward gains of
  l2, l4, l1d are lost.

`test_scan_probe_above_weak_peak` (two substrings at 0.65, +8 % full-string maximum) passes at 1.015 and fails at
0.97 ... 1.01 (host Unity shim).

### Remaining regressions

* Static, heated -12 %: the 9 cases above (gains 10 at -12 %). Heating by 12 % since the last sweep with a higher
  maximum 4-21 % better.
* Closed loop: no cell mean below -0.2 points except wx-shade-walk (-0.34, noise-level, all factors negative there on
  this host).
* The closed-loop vs static placement gap (2-3 %) is the probe's own inputs. Measured 2026-11-24 by forcing each to
  the plant's exact value (paired over the 10 board/offset pairs, `SIM_KVMP_EXACT` / `SIM_VOC_EXACT`, all runs on one
  host; the same tree reproduces this table's l2 row exactly: 0.97 -> 1395.6 s / 93.42 %, 1.03 -> 240.9 s / 98.73 %):

  | cell | base | `kVmp` exact | `voc` exact | both |
  |---|---|---|---|---|
  | l2 (shaded) | 97.35 % | 98.75 % (+1.40) | 98.21 % (+0.86) | 98.77 % (+1.42) |
  | h1 (heated) | 98.83 % | 98.84 % (+0.01) | 98.85 % (+0.02) | 98.84 % (+0.01) |

  `kVmp` dominates. It is learned only from an *unshaded* sweep MPP, and only if `v/voc` lies in `(0.74, 0.86)`
  (`src/peak_scan.h`); a shaded maximum deliberately keeps the last value. With the shade already in at the boot
  sweep it therefore stays at the **0.820 default** while the plant's `kp` is **0.860** - a systematic **-4.7 %** on
  every probe, worth up to ~1.4 points of harvest.
  `voc` is secondary: it is the **Vin at the sweep start** and is never refreshed, so it sits below the true Voc.
  In the heated cells neither costs anything (h1 is already at 98.83 %), so that part of the gap is placement-only.

### Reproduce (round 2)

```bash
T=/tmp/scanf-tree; rm -rf $T; mkdir -p $T; rsync -a src test config $T/
OUT=/tmp/scanf-bin $T/test/host-stub/mppt-sim.sh none >/dev/null; ~/bin/offload build-sim $T /tmp/scanf-bin
cd doc/research/scan-share
for c in a1 a2 a3 a4 a5 a6 l1 l2 l3 l4 l5 l6 l1d a3d l2k a3k h1 h2 h3 h4 wx-shade2-move wx-shade3-move wx-shade-walk \
         wx-shade-cloud wx-sweep-shade wx-shade2 wx-shade3 wx-cloud; do for b in fry flat; do for o in 0 13 29 41 57; do
  for v in base u0990 u1000 u1010 u1015 u1020 u1025 D6; do echo "$c $v $b $o"; done; done; done; done > /tmp/jobs.txt
BIN=/tmp/scanf-bin HOST=pgx93 TAG=scanf-1 OUTDIR=/tmp/scanf-out xargs -P 18 -I{} ./job.sh "{}" < /tmp/jobs.txt > /tmp/results.txt
python3 agg.py /tmp/results.txt                                   # results-2.txt: the 2240 result lines of round 2
/tmp/scanf-bin scan-static > /tmp/static.txt && python3 static_cmp.py /tmp/static.txt --cases   # ~2 s
```

About 13-20 runs per minute on pgx93 (18 slots), 2240 runs in about 2.5 h.
