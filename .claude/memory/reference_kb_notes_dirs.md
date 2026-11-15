---
name: ~/dev/kb/ knowledge-base notes are auto-pushed and must be read before deriving
description: User maintains curated kb notes (e.g. ~/dev/kb/esp32/, ~/dev/kb/tooling/) that the harness pushes into the session when commands touch that tooling; read the relevant ones before forming hypotheses
created: 2026-10-10T16:50:36.627Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: extraction
  type: reference
  originSessionId: 01a126b5-53ca-7ed1-88ca-d145893f1b0c
---

The user maintains a curated knowledge base at ~/dev/kb/ with per-topic directories (e.g. esp32/, tooling/). The harness pushes a list of relevant note paths into the session when a command touches that tooling area, with the standing instruction to read any that bear on the task BEFORE forming a hypothesis. Note statuses include verified / unverified / TOMBSTONED markers.

User turn (2026-10-10): "kb notes in esp32/ (pushed once this session because the command touches esp32 tooling). Read any that bear on what you are about to derive, BEFORE forming a hypothesis: - ~/dev/kb/esp32/boot-and-ota.md ... - ~/dev/kb/esp32/usb-console-and-recovery.md ..."

**Why:** these notes contain hard-won bench measurements and corrections (e.g. ESP32-S3 USB reset/recovery traps) that contradict common assumptions; ignoring them leads to re-derived wrong hypotheses.

**How to apply:** when an esp32 (or other kb-covered) task begins, read the pushed notes that are relevant before answering; cite or respect TOMBSTONED entries as retracted.
