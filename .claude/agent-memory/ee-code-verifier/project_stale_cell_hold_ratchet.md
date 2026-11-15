---
name: stale-cell-hold-ratchet
description: Any BMS-stale pin derived from vout_avg at glide entry ratchets up via stuck-watchdog releaseVoutPinning re-entry; pack-V cap never bounds the highest cell
metadata:
  type: project
---

Reviewed 2026-11-12 (uncommitted, /tmp/integ integ-ota-1110): stale-cell "hold at vout_avg+0.1 V for 600 s" in
`_updatePackVoltagePinning`'s `!batDataOk` branch.

- `releaseVoutPinning` (stuck watchdog, 30 s at |Iout|<0.3 A, not gated by chargeHold while stale) resets
  `_fallbackGlide`; re-entry recomputes the target from the now-higher vout_avg → +~0.08 V per release.
  Host sim: 27.40 → 28.43 V in 600 s. A single charger self-ratchets too once the pack tapers to <0.3 A.
- vout_avg: EWMA span 60 fed from lfControl every 3 s (lfPeriod) with the sensor ewm → τ ≈ 90 s.
- fmetal charger.conf cv_eoc=3.57 → a "v_eoc − 0.05" gate lets the hold arm at vcell_high 3.52 (knee).
- A pack-voltage cap can't bound one cell: cap − 7·V_others (e.g. 27.4 − 7·3.38 = 3.74 V).

**How to apply:** any stale-data setpoint must be latched per stale episode (keyed to vcell_high_ticks),
not recomputed after a release; gate on an absolute plateau voltage, not relative to cv_eoc. See
[[pd-control-loop-gain-anatomy]] for why parallel CV sources leave one converter at 0 A.
