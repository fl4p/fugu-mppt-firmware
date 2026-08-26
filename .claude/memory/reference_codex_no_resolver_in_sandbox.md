---
name: codex-no-resolver-in-sandbox
description: codex exec sometimes cannot resolve DNS inside the Claude Code sandbox; PROBE with a one-liner first — it worked fine on 2026-08-26
metadata:
  type: reference
---

**Probe before assuming this applies** — it is intermittent, not permanent. On 2026-08-26 a bare
`timeout 60 codex exec --skip-git-repo-check "reply with exactly OK" < /dev/null` returned `OK`
immediately from inside the sandbox, and a full 20-minute adversarial review ran to completion.
Cost of the probe is ~5k tokens; cost of wrongly believing it is blocked is punting the work to
the user.

When it IS blocked, `codex exec` from Claude Code's Bash tool dies with
`failed to lookup address information: nodename nor servname provided` /
`stream disconnected before completion`, however many times it retries.

**Why:** the sandbox has no route to the macOS system resolver. Measured 2026-08-19:
`host chatgpt.com` succeeds (direct DNS to a nameserver) while `getaddrinfo` fails; `scutil --dns`
and `dscacheutil` return nothing; `pgrep` reports "Cannot get process list" and
`sysmond service not found`. `dangerouslyDisableSandbox: true` does NOT help, and neither does
restarting mDNSResponder — it is an environment property, not a broken daemon. `!`-prefixed
commands run in the same environment, so they fail too.

**How to apply:** probe once (above). If it answers, just run the review. If it fails, don't burn attempts retrying. Write the review prompt into a shell script and
have Fab run it from a normal Terminal, teeing to a file (`/tmp/...`) that can then be read back
— file access from the sandbox works fine. Extract from the LAST heading; codex repeats its final
block. See also [[codex-exec-stdin-hang]] (`< /dev/null`).
