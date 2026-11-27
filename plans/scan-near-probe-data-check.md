*this document is an LLM generated placeholder*

# Near-point probe: field data check (open, from 2026-11-24)

## Question

Does flat sit below its Vmpp at ≥300 W? The 30-day study put a possible loss of ~22 Wh/day there; the
review of `doc/research/2026-11-24-mppt-tracking-at-limit.md` left it unresolved.

## State (2026-11-27)

- Probe v2 on origin/main as ba83c76c; fry and flat have run it since 11-24 ~17:20 UTC. Check `bootinfo`
  for the build they are on — they are reflashed by other sessions often, so do not rely on a pinned version here.
- Log line, one per periodic or triggered scan:
  `scan near: v0 X V, -2.0V xA, +2.0V xB, ref Pa->Pb W[, drift][, near only]`
  - ratios are against the start power interpolated between before (Pa) and after (Pb)
  - `drift`: Pa and Pb differ by more than 1 %, light changed; exclude
  - `near only`: periodic scan with no distant candidate (bright, full string); these are the ≥300 W samples
  - an out-of-range probe is written `n/a` (no `x`); a regex requiring `x` drops those lines silently
- Calibration (unit test): op point 3 V below Vmpp gives +2V x1.013; at Vmpp both sides ~x0.99.

### Read the archive, not the log

Since 2026-11-27 `fugu-near-collector` on havan subscribes to `pv/log/+` and appends every probe to
`~/pv/near/<UTC date>.jsonl` (one JSON object per probe; `n/a` stored as `null`). Source and deploy recipe:
`etc/havan/README.md`, `etc/havan/near_collector.py`, tests `etc/test_near_collector.py`.

**Analyse the archive.** `~/pv/fugu_console.log*` rotates (~10 MB per file), so older samples vanish and a
count taken from it changes from one day to the next — that is what made two earlier readings of this
question disagree. The archive is durable and is now the source of truth.

## Findings so far (do not re-derive)

- 11-26, flat, every probe at ref ≥300 W: +2V ratios 1.040, 1.005, 0.965, 1.003, 0.971, 0.986, 0.956 —
  **straddling 1** (>1 in 3 of 7; the two non-drift samples 1.003 and 0.986). No consistent below-Vmpp.
  Below 300 W the ratio is also mostly < 1, so there is no power-dependent sign change either.
- An earlier "2 of 2 > 1" rested on a 09:21:56 sample that had rotated out of the log, and on n=2. Beware n=2.
- The CV explanation is refuted: at those samples `cv_lim_idx` was absent and `Uo` 25.5-25.9 V, below the
  26.8 V float target (`cv_float 3.350 x 8`). `cv_lim_idx` is absent for flat across 11-26, so it is not a
  usable discriminator.
- The tracker's own `MPP duty` comparison (n=562, mean −1.2 counts) is self-referential — a low MPP estimate
  makes the gap look small — so it cannot settle the question either way.

## Check (after 3+ clear days of archive)

1. `ssh havan-ts 'cat ~/pv/near/*.jsonl'`; keep `device == "flat"` with `min(ref_before, ref_after) >= 300`
   and `drift == false`. Say which UTC days the archive covers and how many clean samples there are.
2. Per converter: n, share with +2V > 1.005 and > 1.01, share with −2V > 1.005, medians. Same for fry as control.
3. Count `drift` and `near_only` samples: if most bright samples drift, the 1 % threshold or the probe timing
   needs a look.
4. Probe cost: near-only runs every `scan_interval` (300 s), ~0.6 s each; check no rise in tracking loss.
5. The probe needs a sweep on flat: `PeakScan::update()` runs from the return pass after a sweep (`src/mppt.cpp:305`)
   or normal tracking (`:363`, requiring `controlMode == None`). flat is CV-limited when it is, so its probes ride
   the return pass. flat does sweep at high power; a manual `sweep` also works but briefly drops a live converter
   to ~0 W — only with Fab's go-ahead, and take the lock:
   `etc/fugu_console.py --mqtt --name flat --take-lock --lock-note "..." -c sweep`.

## Decision

- +2V consistently > 1 (most clean samples, median ≥ ~1.005) on flat only: the offset is real. Look at the tracker's
  reversal rules and the 5 %-Vin rules in `src/tracker.h`, estimate Wh/day from the ratio, plan a sim-tested fix.
- Both sides < 1 in most clean samples: flat is at its peak; close the question and update the research note.
- Mixed or too few clean samples: wait for more days.

Record the result in `doc/research/2026-11-24-mppt-tracking-at-limit.md`, then mark this plan done.
