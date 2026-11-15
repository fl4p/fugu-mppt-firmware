---
name: project-wire-guard-warn-only
description: "Issue #47 wire-fault guard committed 1b96f1f, deployed to fry+flat 2026-11-12 as c0bf8e0, wire_protect=1 warn only; two follow-ups block enabling protect"
metadata:
  node_type: memory
  type: project
  originSessionId: e30d4973-fdd8-4b7f-a285-59288d26932b
  modified: 2026-11-12T14:16:06.209Z
---

Wire-fault guard (`src/wire_guard.h`, alerts `src/tele/alert.*`, plan `plans/wire-fault-protection.md`) committed to main as 1b96f1f. Default `limits.conf wire_protect=1` (warn only): telemetry `wf`/`Pw`, retained MQTT `pv/alert/<host>/wire`, console `wire`/`wire-reset`. Baseline in NVS blob "wire". DEPLOYED 2026-11-12 14:13/14:15 UTC to fry+flat as c0bf8e0 (main = merge of integ-ota-1110 + guard; version string has -dirty from submodules), via etc/ota-via-relay.sh on havan 192.168.1.200 + MQTT console. Both rollback-valid; retained pv/alert/{fry,flat}/wire seen. Learning phase started 11-12 (7 days, warn only).

**Why:** on November currents (11–19 A) the excess-power criterion is slow (+6 mΩ never warns, +15 mΩ warns after 10 h); and the fast path showed +3.4–4.5 mΩ offset drift on 11-04, a nuisance-derate risk at summer currents.
**How to apply:** don't set `wire_protect=2` before (1) a current-independent ΔR warning and (2) warn-only observation over high-current days. Fab accepted that a shared-section (`Rb`) fault can derate/trip via the fast path. Related: [[project-ro-tracker-step-not-viable]].
