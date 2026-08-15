---
name: codex-exec-stdin-hang
description: codex exec hangs forever reading stdin when run as a background shell command — always redirect < /dev/null
metadata: 
  node_type: memory
  type: reference
  originSessionId: 46c7e808-20eb-4e70-8a46-f280e1c16d5a
  modified: 2026-08-15T08:45:16.124Z
---

`codex exec "<prompt>"` prints `Reading additional input from stdin...` and blocks forever
when stdin is a non-TTY pipe that never closes — which is what the agent Bash tool provides
for background (`run_in_background`) commands. Observed 2026-08-15: 61 min elapsed, 0.09 s
CPU, no `~/.codex/sessions` rollout file ever created. Foreground runs may work by accident.

**How to apply:** always launch it as
`codex exec --sandbox read-only "<prompt>" < /dev/null > out.txt 2>&1`
and read `out.txt`; also avoid piping through `tail` (buffers all output until exit, so
progress is invisible). Diagnose a suspected hang via CPU time (`ps -o etime,time`) plus
absence of a new rollout file under `~/.codex/sessions/YYYY/MM/DD/`.
