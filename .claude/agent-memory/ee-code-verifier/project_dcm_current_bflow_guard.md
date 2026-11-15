---
name: dcm-current-bflow-guard
description: highD-bflowOpen guard gated on dcmCurrent() (commit 944f541) - where its safety margin is thin
metadata:
  type: project
---

dcmCurrent(vh,vl) = D²·vh·(vh−vl)/(2·fL·vl), NAN unless D < M·0.98. Boundary current = vl(1−M)/(2fL).
Where that is < 1 A (fry fL 2.96: Vin < ~33 V at 25.6 V out; flat/fmetal lower), the NAN gate, not the
1 A cap, is binding, so CCM detection rests on 2 % ratio margin + diode drops (~2.6 % at 31 V).
vconv: past CCM onset, current rises ~3 A per 1 % duty. Backflow switch is on the INPUT (panel_sd),
not output. Diode drops, EWM lag on an up-sweep and L0·0.95 all err toward overestimating/tripping;
HS turn-off delay (realised on-time > commanded) is the one that underestimates.

**How to apply:** when reviewing changes to this guard, check the low-Vin band and voltage-ratio error.
