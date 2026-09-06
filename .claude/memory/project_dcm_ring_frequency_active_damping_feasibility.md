---
name: DCM ring frequency is per-board (flu measured 320 kHz); active damping needs non-comparator mechanism on MCPWM
description: DCM SW-node ring frequency is per-board and set by how many FETs sit on the node. MEASURED: flu 320 kHz, T_ring/4 ~780 ns (2026-09-06). The ~1.3-2 MHz / 125-192 ns figure is an UNVERIFIED ESTIMATE for fry/flat only and is an order of magnitude too high for any board with paralleled 100 V FETs. MCPWM has only 2 comparators/operator (both used for HS/LS), so LS re-trigger active damping needs a different mechanism (2nd operator, dead-time submodule, or SW one-shot). LEDC has 2 channels (duty/hpoint, one pulse/period) and can't do it either. doc/DCM Ringing.md updated 2026-08-11 with all corrections.
created: 2026-08-11T10:11:04.822Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: project
  originSessionId: ses_00fb315cbffekO0yL5QTw8UwxV
---

**CORRECTED 2026-09-06 -- the ~1.3-2 MHz figure was never measured and does not apply to this hardware generally.** Canonical: `doc/DCM Ringing.md`.

The DCM ring involves the main inductor L and the whole switch-node capacitance C_sw, and **C_sw is dominated by how many FETs are paralleled on the node**, so f_r is per-board:

- **flu: 320.2 kHz, C_sw 2.98-3.04 nF, T_ring/4 ~780 ns** -- MEASURED 2026-09-06 (LS-gate-triggered, cosine fit; `pv/ee/plans/RESULTS-flu-csw-ring-20260906.md`).
- fry/flat: ~1.3-2 MHz, T_ring/4 125-192 ns -- **estimate, unverified**, from an assumed C_oss of 100-200 pF.
- fbuck: **unmeasured.** A previously quoted 3.34 nF / ~574 ns was retracted 2026-09-06.

It is still true that this is not the HF transition ring (tens to hundreds of MHz, from parasitic loop inductance).

**Do not time an LS re-trigger from a constant.** At flu's ~780 ns valley, a delay hard-coded to the fry/flat 125-192 ns fires near the ring's peak and pumps energy in rather than clamping it out.

**Why:** The lower frequency makes firmware active damping timing-feasible (within MCPWM tick resolution at 39 kHz / ~4000 counts, tick ≈ 6.4 ns). However, the ESP32-S3 MCPWM has only 2 comparators per operator (`SOC_MCPWM_COMPARATORS_PER_OPERATOR=2`), both already allocated to HS and LS edges (`cmpHS_`, `cmpLS_` in `src/pwm/mcpwm.h`). There is no spare comparator for an LS re-trigger pulse. Active damping would need a different mechanism: a second operator's comparator cross-triggered, the dead-time submodule, or a software-triggered one-shot event. LEDC uses 2 channels (duty/hpoint registers, one pulse per period each) and also cannot generate an additional LS pulse.

**How to apply:** doc/DCM Ringing.md was updated 2026-08-11 with: (1) HF transition ring vs DCM coil ring distinction, (2) corrected frequency 1.3-2 MHz, (3) existing `rect_offset_ns` partial damping via reverse current, (4) `sync forced` / `forced_pwm=1` existing forced-CCM, (5) MCPWM 2-comparator constraint and LEDC channel limitation, (6) `boot_refresh_ns` is bootstrap refresh not ring damping, (7) T_ring/4 corrected to 125-192 ns, (8) flat rect_offset corrected to +78 (not +57). No commit in this session — doc edits only.
