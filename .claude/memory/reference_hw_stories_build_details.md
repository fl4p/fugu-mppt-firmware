---
name: reference-hw-stories-build-details
description: "Per-board hardware build details (fry, flat, flu, fboost, ...) live in ~/dev/ee/hw/Fugu2/HW Stories/<board>.md — check there before guessing what a board has fitted"
metadata:
  node_type: memory
  type: reference
  originSessionId: e30d4973-fdd8-4b7f-a285-59288d26932b
  modified: 2026-11-10T09:30:13.197Z
---

`/Users/fab/dev/ee/hw/Fugu2/HW Stories/` has one Markdown file per physical board (`flat.md`, `flu.md`, `fboost.md`, `ftall.md`, `fugu1.md`, `banana.md`, …) with photos. Each records that board's as-built details: FETs, gate drive, coil (core, turns, L), input caps, heat sink, components not fitted, and dated incident/fault sections.

Read the board's file before reasoning about its hardware (e.g. a sensor offset may be a missing part, as with flat's Vout divider cap, [[project-ro-tracker-step-not-viable]]). Record new hardware findings there as a dated section, matching the existing style.
