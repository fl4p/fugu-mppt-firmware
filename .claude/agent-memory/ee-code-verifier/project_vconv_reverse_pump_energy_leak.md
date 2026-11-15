---
name: vconv-reverse-pump-energy-leak
description: vconv buck c<0 branch never credits HS-body-diode charge to iIn; energy destroyed; inflates wx-dawn harvest 80 -> 94 %
metadata:
  type: project
---

src/sim/vconv.cpp stepOneCycleBuck: phase 3 with c < 0 (LS min pulse drove reverse current) sends areaOff
through the HS body diode, but iInAvg_ = areaHS only. Probe 2026-11-14 (Isc 0.07 A, Vin~29, Vout 26.6):
Pin +2.0 W, Pout -3.3 W. This makes the high-duty low-light plateau (PV at Isc, Vin hugging Vout) look
catastrophic, so P&O is repelled. Crediting it (iIn += areaOff when c<0) drops wx-dawn from 94.5 % to
~80 % (P&O drifts up the flat plateau to pwmCtrlMax, Vin ~29 V vs MPP ~44 V). The ring branch (ring_f>0)
credits the clamp charge (qIn), so ring-on dawn runs show the trap. Not fixed: would change baselines.

**Why:** matches field flat dawn stuck at ~4010 counts / 1.3 W (June).
**How to apply:** don't trust wx-dawn/low-light harvest numbers from the default plant; fix the plant and
re-baseline before evaluating any low-light tracker change. See [[dcm-switch-node-ring]].
