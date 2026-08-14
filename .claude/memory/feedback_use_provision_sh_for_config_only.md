---
name: Use provision.sh for config-only changes, don't full flash
description: When only the littlefs config changed (not firmware), use ./provision.sh <config> not idf.py flash
created: 2026-08-12T22:12:36.001Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: feedback
  originSessionId: ses_00aed49c6ffeDARNnXjfrKzIY3
---

Use `./provision.sh <config_name>` to reflash only the littlefs partition when the firmware hasn't changed. Don't do a full `idf.py flash` just to update the config.

**Why:** Full flash rewrites bootloader + app + littlefs + otadata — unnecessary and slower when only the conf files changed. provision.sh writes just the littlefs partition.

**How to apply:** If only files under `config/` changed, run `./provision.sh <config>` (e.g. `./provision.sh psu/boost80V`). Only do `idf.py flash` when the firmware binary itself changed (src/ changes).
