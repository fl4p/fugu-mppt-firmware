---
name: feedback-no-feature-branches-always-main
description: "Never create feature/fix branches in Fab's repos — commit directly to main, always"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 07f59db9-3384-490a-8c10-3cd1515d8404
  modified: 2026-09-06T21:40:06.130Z
---

Never create a feature or fix branch. Commit everything directly to `main`.

Stated 2026-09-06, emphatically ("no feature/fix branches please, never! all on main")
after I had parked unvalidated firmware fixes on `fix/deferred-bench-items` to keep them
out of reach of another session's OTA.

**Why:** Fab works this repo with many concurrent agent sessions and a shared `main`;
side branches fragment that and get lost. The reflex to isolate risky work on a branch
is the wrong tool here.

**How to apply:** commit to `main` even when the work is unvalidated on hardware. Say so
in the commit message instead — mark it "NOT validated on hardware" so anyone about to
OTA `fry`/`flat` can see it. Isolation is communicated in the message and in the
`fugu-rig` lock note, never in the branch topology. Related: [[feedback-commit-author-user-name]],
[[feedback-confirm-before-flashing]].
