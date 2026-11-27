*this document is an LLM generated placeholder*

# CV-hold post-dip cap (from 3168f2e0) vs origin/main 21626ec2

Evaluation only; the cap was not landed (cv-hold-known-issues.md). Patch (not kept): `/tmp/capeval-wt-cap.patch` (cap in `src/charger.h`, host test
`test_bal_cloud_dip_cap`, sim knobs `SIM_IBAT_NOISE=A` (alternating +-A on BMS ibat frames) and
`SIM_CELL_DT=s[,s..][@t]` (cell frame intervals from t) in `test/host-stub/lfp_pack.h` / `mppt-sim.cpp`; sim only).

## Setup

- A = origin/main + sim knobs, B = A + cap. Both built with `OUT=/tmp/capeval-{A,B} test/host-stub/mppt-sim.sh`,
  same Mac, run in pairs (2 processes) by `/tmp/capeval-run.sh TAG SCENARIO [ENV=V..]`; logs in `/tmp/capeval-runs/`.
- Metrics from the sim's `== CVHOLD ... re-entry` line: peak after the event, seconds from the event's end to the
  cell back at cv_eoc - 2 mV.

## Key finding: the cap is mostly the tail current

The cap is `max(iTail, _cvIeoc)`. In the re-entry scenarios iTail = 0.05 C x 245 Ah = 12.25 A, while the hold
current is ~5 A and `_cvIeoc` is 3.7-5.4 A, so the cap is always iTail. An instrumented build (`CAPBIND` print)
showed it binding on 5-7 cell frames per unit per run, clipping raw targets of 12.3-15.1 A by at most 2.8 A.
Peak differences between A and B (a few mV either way) are perturbations of a sensitive loop, not a systematic
effect. With the field's default tail_c_rate (0.05) and 245-280 Ah, the cap only limits ramps above 12-14 A.

## Results (peak V / back s / verdict)

| case | A (main) | B (main + cap) |
|---|---|---|
| recloud | 3.5110 / 152 / PASS | 3.5071 / 165 / PASS |
| recloud dt4 | 3.5070 / 141 | 3.5079 / 138 |
| recloud dt12 | 3.5105 / 136 | 3.5120 / 168 |
| recloud dt4 noise 1.2 A | 3.5052 / 92 | 3.5090 / 159 |
| recloud dt12 noise 1.2 A | 3.5075 / 123 | 3.5133 / 122 |
| regap dt4 | 3.5031 / 95 | 3.5021 / 138 |
| regap dt12 | 3.5137 / 80 | 3.5012 / 151 |
| regap dt4 noise 1.2 A | 3.5036 / **709** | 3.5043 / 86 |
| regap dt12 noise 1.2 A | 3.5222 / 77 | 3.5041 / 100 |
| recloud + unit 2 reboot 1000 s | 3.5033 / 92 | 3.5035 / 92 |
| same + noise 1.2 A | 3.5154 / 82 | 3.5141 / 87 |
| regap + reboot 1000 s | 3.5051 / 152 / FAIL (no resume 60 s) | 3.5014 / 142 / FAIL (same) |
| recloud, cells 12/60 s from 800 s | 3.5437 / 103 / **FAIL** peak | identical, FAIL |
| regap, cells 12/60 s from 800 s | 3.5258 / 126 | identical |
| recloud-early, cells 12/60 s from 300 s | 3.5220 / 185 | identical |
| PACK_TAU=1 regap / recloud / early | 3.5061/154, 3.5055/131, 3.5086/140 | 3.5139/140, 3.5078/146, 3.5143/159 |
| PACK_TAU=0.35 regap / recloud / early | 3.5017/78, 3.5110/152, 3.5067/157 | 3.5042/147, 3.5071/165, 3.5203/147 |

All noise/tau rows PASS on both. Default `cv-hold-*` (pass/fail identical on A and B):
pair FAIL both (3.489..3.507 after hold + 300 s vs +-10 mV), offset/gap/load/reboot/mask/day/regap/recloud/
recloud-early PASS both; entry/run peaks equal to 1 mV except recloud-early 3.513 (A) / 3.514 (B), day 3.5065 both.

Host test `test_bal_cloud_dip_cap` (main's pack model, no fast branch): A FAIL (crosses cv_eoc at 19.4 A, peak
3.509), B PASS (9.7 A, peak 3.504, back after 238 s vs 141 s).

The regap+reboot FAIL is an artifact of the combination (unit 2 reboots inside the gap, so it never "pauses").
The 12/60 s FAIL is on main itself: 60 s cell frames let the cell cross from 3.47 V to 3.54 V between frames.

## By reading

- Tail timer: the tail test is skipped below the band in hold mode, and the cap is released at the band, so the
  cap cannot starve or run the timer. PV-limited below iTail: the cap (>= iTail) never binds. A cell that never
  reaches the band holds until cv_hold_max_s, as on main.
- Noise: `_cvIeoc` takes noisy ibat, but the floor iTail dominates; there is no growth term, slope or learned
  floor, so no stalled recovery (worst B return 168 s vs A's 709 s).
- `_cvIeoc` across a stale pause: kept (the resume path does not reseed). Reboot: not in ChargerRtc, NAN, and
  `fmaxf(iTail, NAN)` = iTail; both units carry the same floor, so no survivor-vs-rebooted asymmetry. A fresh hold
  entered without a cut at high ibat seeds a high `_cvIeoc`, making the cap inert until a frame at cv_eoc.
