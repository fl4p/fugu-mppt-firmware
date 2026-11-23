---
name: Wokwi CI token
description: Wokwi CI credential: where it lives, how to use it, and that user.tok is not it
created: 2026-11-23T09:46:30.243Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: reference
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

`$WOKWI_CLI_TOKEN` — real value in `.claude/memory/secrets.env` (gitignored; see
`secrets.env.example`).

Use with `wokwi-cli` (installed at `/Users/fab/bin/wokwi-cli`) to run the simulator headless.
`~/.wokwi/user.tok` is the *license* (VSCode extension etc.), not the CI token — they're different.

The project's `wokwi.toml` points at `build/` (checked 2026-11-23); the classic-ESP32 board in
`diagram.json` needs `build-esp32/`, so symlink it — see [[project_wokwi]].
