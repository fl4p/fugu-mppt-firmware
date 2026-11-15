---
name: project_highd_bflowopen_lowlight_trip
description: "fry's highD-bflowOpen trip storm (1361 on 2026-11-11) comes from a fixed duty>0.33 guard hit at low light, not from a fault"
metadata:
  node_type: memory
  type: project
  originSessionId: 5705d5de-ac35-448c-b5a3-418631e9f63a
  modified: 2026-11-11T14:06:54.242Z
---

Diagnosed 2026-11-11 (read-only). `src/mppt.h` ~l.760 shuts down when the backflow switch is open and duty > 0.33. Below 0.05 A, `mppt.cpp` ~l.313 opens bflow (it re-closes above 0.2 A). At low light fry sits at 0.05–0.1 A with Vin ~28–37 V, Vout ~25.6 V, and its DCM duty is ~0.34+, so it trips, backs off 5 s, recalibrates, re-sweeps, and repeats.

- Trips only happen at dawn and dusk on normal days (11-09/10: 07h and 16–17h, ~100–170 per hour). 11-11 was dim all day: fry 1361, flat 62. flat makes ~0.3 A, which stays above the hysteresis.
- fry's iout offset of −0.61 A has been stable since at least 11-04; it is not new.
- fry/flat energy ratio is ~0.65 on sunny days because the arrays differ.

**Why:** the guard assumes a high duty means high current. With a battery output, though, current is only high as D approaches Vout/Vin, and here Vout/Vin is ~0.8.
**Fix (2026-11-11, commit 944f541 on integ-ota-1110, NOT OTA'd):** `SynchronousConverter::dcmCurrent()` gives I = D²·Vin·(Vin−Vout)/(2·fL·Vout), or NAN when D ≥ M within the ratio error, for a boost, or for bad voltages. The trip now fires only when the duty is above 0.33 and that estimate is not < 1 A. It is host-tested in `test/host-stub/dcm-current-test.cpp`, where it lands within −3/+9% of the vconv model.
Follow-up 19c9409: after review, the gate is D < 0.9·M (a 0.95·M gate let a 4%-low Vin / 2%-high Vout reading escape into CCM in the host test).
**Bench, vconv on the S3 bench board, 2026-11-11:**
- Low light (Isc 0.12 A, Voc 66 V, Vbat 25.6 V): the sweep reached D ≈ 0.7 with bflow open and did not trip; MPPT then held 6 W at ~50 V.
- Positive control (lv_i_factor 0.01, Isc 3 A): it tripped on every sweep, Idcm 2.45–3.66 A.
Deployed 2026-11-11 15:51-15:55 (19c9409). The dusk trips continued at the same rate, ~4/min, all at Idcm=nan: Vin collapses to ~28 V, so the sweep reaches 0.9·M.
- 8546c97+0b5c23b: a sweep that reaches the ratio limit with valid Vin>Vout>1 V and measured I<0.05 A now ends via _stopSweep (commit the MPP and drop the duty to it, or a 30 s sweep-no-mpp backoff) instead of tripping. vconv bench: dusk Isc 0.01 gave "ending it" plus a 30 s backoff and 0 highD; with the sensor lying it still trips.
- 0b5c23b was OTA'd to flat and fry at 16:42-16:44, rollback VALID.
- The host simulator `test/host-stub/sim/` (the real MpptController plus the vconv plant, 4 h in ~15 s; `build.sh`, `scenarios.sh OLD NEW`) reproduced the old loop at 330–420 trips/h. It also found a bug in 0b5c23b: protect() returning false made loopRT call stopAndBackoff(4), which drops the committed MPP (~250 shutdowns/h). Fixed in 3e0539c (sim: ~0 trips, 99% tracking). 3e0539c was OTA'd to flat and fry on 11-11 at ~20:04, rollback VALID.
**How to apply:** on 11-12, check fry's highD and "ending it" counts per hour against 11-11 (1361 highD). Expect highD trips only near CCM after that. Related: [[project_bflow_highduty_trip_accepted]].
