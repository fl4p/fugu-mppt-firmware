---
name: DCM ring frequency ~1.5 MHz; active damping needs non-comparator mechanism on MCPWM
description: DCM SW-node ring for fry/flat coils (L=50-80µH, C_oss~100-200pF) is ~1.3-2 MHz; T_ring/4≈125-192ns. MCPWM has only 2 comparators/operator (both used for HS/LS), so LS re-trigger active damping needs a different mechanism (2nd operator, dead-time submodule, or SW one-shot). LEDC has 2 channels (duty/hpoint, one pulse/period) and can't do it either. doc/DCM Ringing.md updated 2026-08-11 with all corrections.
created: 2026-08-11T10:11:04.822Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_00fb315cbffekO0yL5QTw8UwxV
---

The DCM SW-node ring for this hardware is ~1.3-2 MHz, not "tens to hundreds of MHz" (that applies to the HF transition ring from parasitic loop inductance). The DCM ring involves the main inductor L (50µH flat / 80µH fry) × C_oss (~100-200pF), giving f_r ≈ 1/(2π√(LC)) ≈ 1.3-2 MHz. T_ring/4 ≈ 125–192 ns.

**Why:** The lower frequency makes firmware active damping timing-feasible (within MCPWM tick resolution at 39 kHz / ~4000 counts, tick ≈ 6.4 ns). However, the ESP32-S3 MCPWM has only 2 comparators per operator (`SOC_MCPWM_COMPARATORS_PER_OPERATOR=2`), both already allocated to HS and LS edges (`cmpHS_`, `cmpLS_` in `src/pwm/mcpwm.h`). There is no spare comparator for an LS re-trigger pulse. Active damping would need a different mechanism: a second operator's comparator cross-triggered, the dead-time submodule, or a software-triggered one-shot event. LEDC uses 2 channels (duty/hpoint registers, one pulse per period each) and also cannot generate an additional LS pulse.

**How to apply:** doc/DCM Ringing.md was updated 2026-08-11 with: (1) HF transition ring vs DCM coil ring distinction, (2) corrected frequency 1.3-2 MHz, (3) existing `rect_offset_ns` partial damping via reverse current, (4) `sync forced` / `forced_pwm=1` existing forced-CCM, (5) MCPWM 2-comparator constraint and LEDC channel limitation, (6) `boot_refresh_ns` is bootstrap refresh not ring damping, (7) T_ring/4 corrected to 125-192 ns, (8) flat rect_offset corrected to +78 (not +57). No commit in this session — doc edits only.
