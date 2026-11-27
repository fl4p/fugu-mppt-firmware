*this document is an LLM generated placeholder*

# CV hold: known overshoot issues (2026-11-27)

Two known limits of the CV hold (`src/charger.h`, origin/main as of a7dee6e8). A hold-current "re-entry cap" was
built against the first one and rejected after review; its record is below so the work is not repeated blind.

## 1. Re-entry overshoot after a BMS gap or a cloud

Status: open, accepted. Stays below `cv_ceiling`.

When a BMS data gap (stale pause) or a cloud dip lets the highest cell fall below the approach band, its
polarization relaxes (~110 mV after 4-5 min near 0 A). When data or sun return, the polarization rebuilds at
~1.2 mV/s at 15-20 A, faster than the cell loop and look-ahead cut the current, and the cell overshoots `cv_eoc`.

Field, 2026-11-25 (cv_eoc 3.500, ceiling 3.550): re-entries peaked at 3.521 V and 3.519 V, about +20 mV. The
10 s averaged BMS record `test/host-stub/data/bms-top-2026-11-25.csv` shows 3.5191 V after the 11:21..11:26 BMS
gap and 3.5168 V after the 12:00..12:05 cloud (the 10 s average can miss the instantaneous peak).

Model: `lfp-pack-test` replays that record (`test_reentries`). The fitted pack (11-23) follows the cloud window
(model max 3.5187 vs field 3.5168) but is optimistic at the 11:04 entry (-46 mV) and after the gap (-22 mV): it
rebuilds polarization 2-3x slower than the field. `SIM_PACK_TAU` scales tauP; x0.35 matches the rebuild but
overstates the build-up from bulk (+86 mV at the cloud).

Sim on origin/main firmware (mppt-sim, 2 x 700 W, pack 5 Ah above the fit's 0, bat_c 245, measured BMS cell
intervals). Criteria: re-entry peak <= cv_ceiling - 10 mV (3.540 V), back within 2 mV of cv_eoc within 1200 s
after the event.

| scenario | event | tauP | re-entry peak | margin to ceiling | back at cv_eoc |
|---|---|---|---|---|---|
| cv-hold-regap | 280 s no BMS frames at 800 s | x0.35 | 3.5129 V (+12.9 mV) | 37.1 mV | 77 s |
| cv-hold-recloud | light 5 % for 300 s at 800 s, 200 s back | x0.35 | 3.5302 V (+30.2 mV) | 19.8 mV | 72 s |
| cv-hold-recloud-early | same cloud at 300 s | x1 | 3.5053 V (+5.3 mV) | 44.7 mV | 91 s |

Variants (not checked in): `SIM_PACK_TAU=1` regap 3.5082 V, recloud 3.5096 V; cloud 600 s 3.5038 V; cloud
back over 30/60/100 s at 5 % light 3.502/3.503/3.506 V.

Found while probing: a cloud to 0 % light (`SIM_RECLOUD=300,30,0.0`, tauP x0.35) reaches 3.5576 V, over the
ceiling, with no latch. There the cell falls below cv_float, the hold exits on "sun gone", the units return in bulk
at 2 x 20 A and the next hold entry overshoots. With tauP x0.6 the same case peaks at 3.523 V, with x1 at 3.5105 V.
It is the same mechanism as issue 2 (high-current return after a sun-gone exit) and depends on the tauP x0.35 plant,
which is known to overstate the build-up from bulk.

## 2. Cycling load: ceiling exceeded between cell frames

Status: open, existing on origin/main, not seen in field data so far.

A load cycling faster than the cell-frame interval near the top pushes the cell above the ceiling between two
frames. Reproduction (Codex review, rerun 2026-11-27 on origin/main firmware):

    SIM_PACK_TAU=1 SIM_LOAD=900,30,30,800 SIM_CONF=charger.conf:bat_c=280 <mppt-sim> cv-hold-recloud

3.5574 V, 0 latches (3.5589 V with bat_c 245). The trace: sun-gone exits near 957 s, bulk, approach control while
the load is on; at 1430 s the load drops out and the pack current jumps from ~14 A to 41-45 A. The peak falls
between frames: the adjacent delivered frames read 3.480 V (1429.4 s) and 3.506 V (1441.9 s), so zero frames are
above the ceiling and no ceiling rule on frames can catch it.

Rejected fix: a single-frame ceiling latch. At cv_ceiling + 10 mV (3.560 V) it does not catch 3.557 V even with
continuous measurement; at cv_eoc + 10 mV (3.510 V) normal entry peaks (3.518/3.526 V in sim) trip it and defer
balancing by 20 h or more.

## Rejected: hold-current re-entry cap

Commits a90e17c4 ("cv hold re-entry: cap ibat at the last current held at cv_eoc until the cell is back there") and
5bd42fb8 ("learn only in a settled hold, grow when it stalls the cell, explicit resume"); not on main. The cap
remembered the pack current of a settled hold and limited ibat to it after the cell left the approach band, growing
by 0.02 A/s while ibat sat at it and the cell did not rise. In sim it cut the re-entry peaks (fast plant: gap
3.5207 -> 3.5018 V, cloud 3.5327 -> 3.5092 V) but slowed the return to 5-9 min.

Codex review of 5bd42fb8 (second round) found:

- NaN-slope growth: an out-of-range frame interval sets the slope to NaN, the max() keeps the previous finite slope
  and growth does not require a valid interval, so worse input loosens the cap (5.24 -> 5.84 A at dt 30.001 s,
  7.64 A at 120 s). Cell frames at alternating 12/60 s after the cloud: 3.5154 V, failing the peak check.
- Noise-stalled recovery: with +-1.2 A alternating BMS current noise and 4 s or 12 s frames, first return after
  1271/1273 s vs 82/87 s without the cap; the cap sat at 5.52 A from 1140 s to at least 2460 s. No finite return
  guarantee.
- Sibling reboot: cloud at 800 s, unit 2 reboots at 1000 s: 3.5191 V with the cap vs 3.5104 V without. The
  survivor keeps its cap at its floor while the rebooted unit, without one, supplies 11.5-15.1 A in bulk.
- Cycling load (issue 2): 3.5574 V on both parent and cap; not caused by the cap, but it leaves the ceiling
  unenforced between frames either way.

Decision (2026-11-27): the cap is dropped. The re-entry scenarios, the 11-25 record and its replay are kept as tests
of the current firmware.
