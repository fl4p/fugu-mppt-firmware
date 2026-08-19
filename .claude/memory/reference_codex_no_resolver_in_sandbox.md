---
name: codex-no-resolver-in-sandbox
description: codex exec cannot run inside the Claude Code sandbox — no system resolver, so getaddrinfo fails; run it from a real Terminal
metadata:
  type: reference
---

`codex exec` launched from Claude Code's Bash tool always dies with
`failed to lookup address information: nodename nor servname provided` /
`stream disconnected before completion`, however many times it retries.

**Why:** the sandbox has no route to the macOS system resolver. Measured 2026-08-19:
`host chatgpt.com` succeeds (direct DNS to a nameserver) while `getaddrinfo` fails; `scutil --dns`
and `dscacheutil` return nothing; `pgrep` reports "Cannot get process list" and
`sysmond service not found`. `dangerouslyDisableSandbox: true` does NOT help, and neither does
restarting mDNSResponder — it is an environment property, not a broken daemon. `!`-prefixed
commands run in the same environment, so they fail too.

**How to apply:** don't burn attempts retrying. Write the review prompt into a shell script and
have Fab run it from a normal Terminal, teeing to a file (`/tmp/...`) that can then be read back
— file access from the sandbox works fine. Extract from the LAST heading; codex repeats its final
block. See also [[codex-exec-stdin-hang]] (`< /dev/null`).
