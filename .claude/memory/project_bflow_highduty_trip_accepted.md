---
name: project_bflow_highduty_trip_accepted
description: "E MPPT: High duty cycle with open backflow switch" on fry/flat since the join-at-float OTA (10-31) is a known, ACCEPTED nuisance trip — Fab: "just leave it" (2026-11-04)
metadata:
  type: project
---

Since aebc8e4 (terminated sibling joins at float), fry/flat log `E MPPT: High duty cycle with open backflow switch!` + 5 s backoff a few times/day (7 on 11-01; never before 10-31). Mechanism: bflow is current-gated (mppt.cpp:316 below 0.05 A, mppt.h:722 reverse < −0.05 A); joining at 26.8 V creates high-duty/~0-current states (sibling holds bus, or dusk Vin≈Vbat), bflow opens at D>0.33 and the guard at mppt.h:759 trips.

**Why:** Fab decided to leave it (2026-11-04); the guard is protective, cost is only the backoff.
**How to apply:** don't report it as a new error or regression in fry/flat checks; don't propose a fix unprompted. A duty-ramp-before-bflow-open fix was sketched but needs a bench test. Related: [[project_charger_shared_bms_and_flat_vout_cal]].
