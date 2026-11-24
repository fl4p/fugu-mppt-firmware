---
name: feedback_workflow_git
description: Git and writing rules for this repo — commit author Fabian, always main, push by sha, safe partial staging, no obvious comments, README hand-written, published docs curated
metadata:
  type: feedback
---

## Commit as Fabian
2026-05-25. Every commit/amend (also submodules, e.g. etc/fugu): `git commit --author="Fabian <fabian@schlieper.email>"`, no Co-Authored-By.
**Why:** a local `user.name=claude` override once polluted history and made GitKraken commit as claude too.
**How to apply:** pass `--author` always; if `git config user.name` shows claude, `git config --unset user.{name,email}` (local).

## No feature/fix branches
2026-09-06, emphatic ("never! all on main").
**Why:** many concurrent sessions share `main`; side branches fragment work and get lost.
**How to apply:** commit unvalidated work to `main`, mark it "NOT validated on hardware" in the message; isolation goes in the message / `fugu-rig` lock note, not branches.

## Push by sha in the shared checkout
2026-11-24: rebuilt "HEAD" after a rejected push and pushed another session's commit (8f785e40).
**Why:** HEAD in the shared checkout can move between two tool calls.
**How to apply:** `sha=$(git rev-parse HEAD)` in the same command as the commit; push/commit-tree/cherry-pick by that sha. On rejected push rebuild via temp index on origin/main (`git read-tree origin/main; git apply --cached`), never rebase the shared worktree.

## No `--unidiff-zero` partial staging
acbb6b5 broke HEAD CMake (blocks landed inside `set(BLE_SRC ...)` / `if($ENV{FUGU_BAT_V})`); fixed in 1e5c022.
**Why:** zero-context hunks have no anchor; skipping foreign hunks shifts the rest. Worktree builds stayed green.
**How to apply:** build the index from `git show HEAD:<file>` + anchor-asserted scripted replacements (`git hash-object -w` + `git update-index --cacheinfo`); verify the staged tree (`write-tree` → `commit-tree` → `worktree add --detach /tmp/...`, symlink submodules, copy `dependencies.lock`, OWN managed_components — a symlinked one gets re-resolved and broke the main checkout).

## No obvious comments
Fab removed `// printf_mux mirrors to telnet/MQTT...` entirely.
**Why:** the identifier already says it; restating/justifying is noise.
**How to apply:** default to zero comments; annotate only non-obvious intent; omit rather than shorten.

## README is hand-written
2026-11-24 ("docs are mostly AI slop").
**Why:** README technical sections are Fab's original text; more authoritative than the docs site.
**How to apply:** small README edits only (stale facts, "Details:" links); never trim to summary + link.

## Published docs: no scratchpad structure
2026-09-29 (internals/lfp-longevity). Rules now in CLAUDE.md "Documentation site".
**Why:** pages published verbatim from doc/ research notes read as "AI slop" (Bottom line lead, "Independent review", access log).
**How to apply:** research record stays in doc/research/; write the page as a survey with verified numbered references (battery lit archive ~/dev/pv/ee/lit/bat); no LLM-placeholder line on published pages. See [[reference_lfp_longevity_research]].
