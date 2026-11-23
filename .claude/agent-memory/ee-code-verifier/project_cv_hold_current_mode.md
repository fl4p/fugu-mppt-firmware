---
name: cv-hold-current-mode
description: CV hold (charger.h _cvHoldStep) regulates pack current via an Iout limit since 2026-11-23; gain argument, load-off transient, floor trickle
metadata:
  type: project
---

CV hold no longer moves a Vout pin (fb17aa5f, 2026-11-23): pin = Vbat_max, cell frame sets ibat target = ibat_raw + K(cv_eoc - vcell_high)/r
(r = line resistance (cv_eoc-cv_float)/(tail_c_rate*bat_c), ~10-12 mOhm/cell for 280 Ah), ibat frame moves _cvIlim by G(target-ibat), lim <= iout+2 A.

**Why:** flat's INA226 Vout reads -0.1..-0.9 V only while converting; the old pin loop clamped at Vbat_fallback in that wrong frame and the cell hit cv_ceiling.

**How to apply:**
- Outer gain per frame = K*R_dyn/r; real R_dyn ~1 mOhm/cell -> ~0.1/frame, so expect slow settling and ~10 mV ramp lag at the LFP knee; K up to ~1.5 stays stable if R_dyn <= r.
- Load removal while PV-limited gives an ohmic cell spike until the next ibat frames (inherent to current mode; Vout pin rejected it better).
- IOUT_LIM_FLOOR 0.25 A/unit is an unremovable trickle in the hold; ceiling latch is the backstop.
- Terminated + hot pack: load-follow still regulates ibat UP to the derated limit (pre-existing, not fixed).
