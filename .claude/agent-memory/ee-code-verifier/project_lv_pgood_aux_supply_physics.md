---
name: lv-pgood-aux-supply-physics
description: Fugu2 LV_PGOOD (GPIO35) only removes the HV aux path; Bat+ diode path is always on, so release is always safe and a wrong assert self-heals by brownout reset
metadata:
  type: project
---

Fugu2 aux node = diode-OR of Solar+ (Q8 GT800N10D3 source follower, gate R43 200k from Solar+, R44 1M G-S, C31 10nF, 62V zener, D11 SS310), Bat+ (D12 SS310), Vusb (D7). LV_PGOOD high -> Q9 BSS123 grounds Q8 gate -> HV path off. Bat+ path is never switched.

- Release is always safe; the only hazard is "asserted while Bat+ is gone" -> aux collapses -> brownout reset -> R45 10k pulldown restores HV path. No latch, worst case an unplanned reboot.
- Q8 gate recovery after release: tau = (200k||1M)*10nF ~ 1.7 ms, ~0.8 ms to reach Vaux+Vth at 40 V Solar+.
- +12V gate rail: J8 "12V PSU" module = open-pe/buck100-LM5163 (Fab, 2026-10-09; the XL7005A link in the schematic was stale and is now replaced). Its dropout at 12 V out is unverified, check the LM5163 datasheet before reasoning about it. Gate drivers UCC21330BQDRQ1 (UVLO grade unverified). lv_pgood_v default 10.5 V (Fab).
- Firmware review 2026-10-08 (LvPgood, uncommitted): NaN/low lv_pgood_v asserts unconditionally; no per-sensor freshness, stopAndBackoff skips shutdownDcdc when converter already disabled.

**How to apply:** when reviewing aux-supply/LV_PGOOD changes, judge fail-safety by "can Bat+ vanish while asserted", not by release paths.
