---
name: feedback_agent_orchestration
description: Agent/session orchestration — Opus subagents implement in background, Paseo not tiles, session-find, codex exec stdin/DNS traps, offload sims to pgx93, read kb first
metadata:
  type: feedback
---

## Delegate implementation to Opus agents
2026-11-14, Fab: "always delegate opus agents for implementation work".
**Why:** keeps the main session's context for coordination, measurement and review (my reading).
**How to apply:** code changes go to `model: "opus"` subagents briefed with goal, files, constraints (worktree, no device contact unless intended, no commits unless told, wait in foreground); I verify (build, tests, diff), commit, flash. Diagnosis, measurements, device ops, small doc/memory edits stay with me.

## Long agents in background
2026-11-15: a foreground Opus agent blocked the session for minutes; Fab interrupted it by accident, leaving half-done edits.
**Why:** a foreground agent makes the session look unresponsive.
**How to apply:** anything over ~1 min with `run_in_background: true`, tell Fab it runs; foreground only when the next step needs a short agent's result.

## Paseo, not tiles
2026-11-16, Fab: "dont use the tiles here, we are in paseo".
**Why:** restored tiles had no agent process; the tile list missed Paseo agents.
**How to apply:** use `mcp__paseo__*` (list_agents, get_agent_activity, send_agent_prompt, archive_workspace); continue a closed agent with send_agent_prompt; never headless `claude --resume <id> -p` (invisible in Paseo, can collide with a live transcript).

## session-find
2026-11-17. `~/dev/bin/session-find <uuid-prefix|peer-name|pid|title>` prints the SendMessage name, uds socket, pid, status, cwd, first prompt; no arg lists live sessions. Registry `~/.claude/sessions/<pid>.json`. Paseo agent ids are not session UUIDs; match by title + cwd.

## codex exec traps
- stdin hang (2026-08-15): in background it blocks forever on "Reading additional input from stdin". Always `codex exec ... < /dev/null > out.txt 2>&1`; don't pipe through `tail`. Diagnose via `ps -o etime,time` + no rollout file in `~/.codex/sessions/YYYY/MM/DD/`.
- DNS (intermittent; worked 2026-08-26): sometimes `getaddrinfo` fails in the sandbox ("nodename nor servname provided"); `dangerouslyDisableSandbox` and mDNSResponder restarts don't help. Probe first: `timeout 60 codex exec --skip-git-repo-check "reply with exactly OK" < /dev/null`. If blocked, don't retry; write a script for Fab to run in Terminal, tee to /tmp, read back; extract from the LAST heading.
- Launch rules otherwise: `codex-review` skill.

## Offload sim batches
2026-11-15 (Mac at load 100-170). Rules: see global CLAUDE.md "Heavy CPU batch work". Project-bound: brief subagents with no sim batches on the Mac, `--host pgx93`, par=18; `build-sim` rebuilds in place, so not while a batch uses it; a subagent once killed another session's jobs with `pkill -f "offload run"`.

## kb notes
Read pushed/relevant ~/dev/kb notes before forming hypotheses; TOMBSTONED = retracted. See global CLAUDE.md "Cross-project knowledge base".
