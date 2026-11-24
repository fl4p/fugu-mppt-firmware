*this document is an LLM generated placeholder*

# Near-point probe: field data check (open, from 2026-11-24)

## Question

Does flat sit below its Vmpp at ≥300 W? The 30-day study put a possible loss of ~22 Wh/day there; the
review of `doc/research/2026-11-24-mppt-tracking-at-limit.md` left it unresolved.

## State

- Probe v2 on origin/main as ba83c76c; fry and flat run `fry-brk1-1027-g9c3890ea` (or later) since 11-24 ~17:20 UTC.
- Log line, one per periodic or triggered scan:
  `scan near: v0 X V, -2.0V xA, +2.0V xB, ref Pa->Pb W[, drift][, near only]`
  - ratios are against the start power interpolated between before (Pa) and after (Pb)
  - `drift`: Pa and Pb differ by more than 1 %, light changed; exclude
  - `near only`: periodic scan with no distant candidate (bright, full string); these are the ≥300 W samples
- Calibration (unit test): op point 3 V below Vmpp gives +2V x1.013; at Vmpp both sides ~x0.99.

## Check (after 3+ clear days)

1. `ssh havan-ts 'grep -ah "flat.*scan near:" ~/pv/fugu_console.log*'`, keep lines with min(Pa,Pb) ≥ 300 W and no `drift`.
2. Per converter: n, share with +2V > 1.005 and > 1.01, share with -2V > 1.005, medians. Do the same for fry as control.
3. Count `drift` lines: if most bright samples drift, the 1 % threshold or the probe timing needs a look.
4. Probe cost: near-only runs every `scan_interval` (300 s), ~0.6 s each; check no rise in tracking loss.

## Decision

- +2V consistently > 1 (most clean samples, median ≥ ~1.005) on flat only: the offset is real. Look at the tracker's
  reversal rules and the 5 %-Vin rules in `src/tracker.h`, estimate Wh/day from the ratio, plan a sim-tested fix.
- Both sides < 1 in most clean samples: flat is at its peak; close the question and update the research note.
- Mixed or too few clean samples: wait for more days.

Record the result in `doc/research/2026-11-24-mppt-tracking-at-limit.md`, then delete this plan or mark it done.
