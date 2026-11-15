---
name: feedback_no_unidiff_zero_partial_staging
description: Partial staging with `git diff -U0` + `git apply --cached --unidiff-zero` misplaced CMake blocks and broke HEAD; replay edits onto HEAD and build the staged tree instead
metadata:
  type: feedback
---

Staging only my hunks in a file that also had someone else's uncommitted edits, via `git diff -U0` hunk
selection + `git apply --cached --unidiff-zero`, put blocks at wrong lines (acbb6b5: scope/FTP/HA ifs
landed inside `set(BLE_SRC ...)`, defines inside `if($ENV{FUGU_BAT_V})`). The worktree builds were green,
so nothing caught it until a reviewer read `git diff --cached`. Fixed in 1e5c022.

**Why:** zero-context hunks have no anchor; skipping foreign hunks shifts the line numbers of the rest.
**How to apply:** build the index from `git show HEAD:<file>` + scripted, anchor-asserted replacements
(`git hash-object -w` + `git update-index --cacheinfo`). Then verify the exact staged tree: `git write-tree`
→ `git commit-tree` → `git worktree add --detach /tmp/...`, symlink submodules, copy `dependencies.lock`,
and give it its OWN managed_components. A symlinked managed_components gets re-synced (fresh resolve
pulled led_strip 3.1.0 and dropped tinyusb in the main checkout).
