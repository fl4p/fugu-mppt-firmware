---
name: reference-lfp-longevity-research
description: "Where the 2026-09-05 LFP charging/cycle-management longevity research lives, its Codex-reviewed conclusions, and the literature archive path (~/dev/pv/ee/lit/bat)"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 023edbe7-5660-4788-ac93-c4d61fe20a8a
  modified: 2026-09-05T12:59:13.553Z
---

Research write-up: `doc/LFP Longevity Research.md` in fugu-mppt-firmware (findings with locators, source access log, Codex review applied in §6a; review transcript in `doc/reviews/2026-09-05-codex-lfp-longevity-research.md`). Canonical cross-project claim: `~/dev/kb/battery/lfp-graphite-ages-fastest-at-high-average-soc.md`. Literature archive (authorized by Fab 2026-09-05): `/Users/fab/dev/pv/ee/lit/bat/` with INDEX.md + SHA256SUMS; the two Naumann Elsevier PDFs there are owner-supplied licensed files, gitignored, never commit.

Headline (primaries inspected: Keil 2016 JES, Preger 2020 JES, Naumann 2018/2020 + dissertation, Zsoldos 2024 JES, EVE LF280K spec B, Rauhala 2018):
- Time at high SoC is the best-evidenced lifetime lever; the fast-fade boundary is cell-specific (57–73 % across cells), not a transferable 70 %.
- No sustained hold at EoC voltage; a low ~3.4 V float is unevidenced either way.
- `recharge_dod` does NOT lower average SoC (0.2 → pack lives 80–100 %, the worst window). Lowering it needs a partial-charge ceiling + periodic full charge for balancing — a feature the firmware lacks.
- 3.55 vs 3.65 V cutoff: untested; 3.65 V is EVE's standard CV target.
- Never charge below 0 °C; derate above ~45 °C. Charge rate ≤0.5 C is a weak lever (only S7 varied charge rate).

**Why:** a first draft claimed hysteresis alone keeps idle SoC ≤70 %; the Codex review caught the arithmetic. See [[project-recharge-after-full-periodic-sweep]].
