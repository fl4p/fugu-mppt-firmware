---
name: cv-pin-needs-tracker-reset
description: At the battery Vout cap the P&O tracker must be reset on every P-exceeded limiter touch, or it reads the limiter's brake as its own dP<0 and hunts below the cap
metadata:
  type: project
---

update() in mppt.cpp: limiter touches at the CV cap are short (<250 ms) but P-exceeded (Kp*e<0).
tracker.update() reverses on dP<0 (tracker.h), and the power drop caused by the limiter's brake
looks like its own step's result. Old code pinned CV by resetDirection(false) on limit and
resetDirection(true) on release.

Review of 3e6cbb5 (2026-11-14, branch flash-autosus): skipping the reset for all touches <250 ms
fixed the shade-cv lower-peak walk (69 -> 99.7 % of the higher peak) but cut CV-cap charge current
in sim scenario c by ~7-8 % (6.00 -> 5.56 A, ripple variant 6.71 -> 6.19 A), duty hunting 8 counts
below the cap.

**How to apply:** a "touch" filter for the tracker should key on D-only (Kp*_prevE >= 0), not just
duration; A/B with a parent build via `git archive HEAD~1` + copy the new sim/stubs, then
SIM_FINE=30,60 and average Iout from the `fine:` lines.
