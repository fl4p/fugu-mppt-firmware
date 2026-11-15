---
name: feedback_delegate_implementation_to_opus_agents
description: Implementation work goes to Opus subagents; the main session briefs, verifies, commits and deploys
metadata:
  type: feedback
---

Always delegate implementation work (writing/changing firmware, simulator or tool code) to subagents running on Opus (`model: "opus"`), instead of implementing in the main session.

**Why:** Fab, 2026-11-14, during the fugu tracker/charger/heap work: "always delegate opus agents for implementation work". Earlier the same day he asked for an "opus agent" for the INA226 HS fix. My reading (not stated by him): keep the main session's context for coordination, measurement and review.

**How to apply:** brief the agent with goal, files, constraints (worktree, no device contact unless intended, no commits unless told, wait in foreground), then verify its result myself (build, tests, diff) before committing or flashing. Diagnosis, measurements, device operations and small doc/memory edits stay with me.
