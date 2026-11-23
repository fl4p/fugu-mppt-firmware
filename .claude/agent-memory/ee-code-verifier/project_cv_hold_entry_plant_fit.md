---
name: cv-hold-entry-plant-fit
description: Field-fit 8s 280Ah LFP top-of-charge plant (11-23 overshoot) and the CV-hold cell-loop gain bounds K*R_f/r
metadata:
  type: project
---
Fit to the 2026-11-23 fry+flat 40 A hold entry (peak 3.528 V): rc 1 mOhm + polarization 2 mOhm/τ30 s, OCV knee +0.05 V/Ah,
imbalance 40 mV, voc0 3.318 (`topOfCharge()` in test/host-stub/charger-term-test.cpp). Frame-scale R_f ≈ 1.7-3 mOhm,
r = (cv_eoc-cv_min)/(tail_c·Cbat) ≈ 10.7 mOhm, so the HEAD K=1 loop gain ≈ 0.16, which is why the response was slow.

**Why:** the cell loop is ibat_target = ibat + K·err/r, stable with 1-frame lag for K·R_f/r < 1 (for 2 units, α≈1).
Commit 48c9203c (worktree branch, untested on HW) uses K_DOWN 1.5 (R_f < 7.1 mOhm) and K_UP 0.5, plus a one-shot cut to I_tail 30 mV below cv_eoc.

**How to apply:** the 40 A sim margin is only 2 mV (3.513 vs 3.515). On stiff 7-10 mOhm plants, a sibling booting below
the band still gives a one-frame transient of about 3.58 V. Check field peaks after deployment.
