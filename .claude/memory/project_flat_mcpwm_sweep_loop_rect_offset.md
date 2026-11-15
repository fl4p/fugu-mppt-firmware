---
name: project_flat_mcpwm_sweep_loop_rect_offset
description: flat midday re-sweeps looped (highD-bflowOpen) after MCPWM migration; rect_offset_ns 875 set to 0 on 11-09; firmware fix deferred
metadata:
  node_type: memory
  type: project
  originSessionId: 5705d5de-ac35-448c-b5a3-418631e9f63a
  modified: 2026-11-09T21:40:36.123Z
---

After the LEDC->MCPWM migration (flat 11-08 18:49), every 30-min re-sweep on flat looped 4-12 min between 10:00 and 15:00 on 11-09 (~54 min at 0 W). Telemetry: in DCM, as D approaches Vout/Vin, current collapses (3.7 -> 0.5 A -> negative), the low-current check opens bflow and the duty>0.33 guard trips. On LEDC the current held, and the converter jumped to CCM.

Hypothesis: `coil.conf rect_offset_ns=875` was calibrated on LEDC as a net correction including dead time. On MCPWM (interlock, no dt) the DCM LS window (buck.h:1283) hits the period-H-1 clamp (buck.h:1296) below D=M, so the converter runs forced CCM with D*Vin < Vout. fry (offset 0) didn't loop.

2026-11-09 21:39: flat `rect_offset_ns` set 875 -> 0 + restart (Fab approved). Legacy `rect_offset=78` is still in the file (ignored).

**Why:** the loop cost ~25% of flat's midday yield.
**How to apply:** check flat's 10:00-15:00 sweeps on 11-10 for `highD-bflowOpen` loops before calling it fixed. Fab wants the firmware fix considered later: keep the DCM LS window from reaching forced CCM below D=M, and/or recalibrate rect_offset_ns per driver. Related: [[project_runtime_selectable_pwm_driver]], [[project_bflow_highduty_trip_accepted]].

**Host repro (2026-11-10):** `/tmp/simrepro/sweep.cpp` (not committed) runs buck.h from 84a48eb in MOCK mode (src snapshot in /tmp/simsrc with mock.h given name/stop + pwmMax 4103, logging.h stubbed) against the vconv cycle model + PV curve. The ideal plant never trips. With flat's documented 0.4 V low Vout reading, a sweep trips once the LS extension reaches ~220 counts (offset + unmodelled delays) at Isc 8-11.5 A, 260 at 6 A, never at 4 A. Same signature as the field: current collapses at x≈0.95 while Vin rises above Voc. So the field needs ~80+ counts (~0.5 µs) of real extra LS time beyond the 140 offset; offset 0 leaves ~140 ct of margin under the same assumptions.
