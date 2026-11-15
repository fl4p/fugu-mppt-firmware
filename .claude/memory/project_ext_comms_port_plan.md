---
name: project_ext_comms_port_plan
description: "External comms port (VE.Direct pinout, CAN/RS-485 via isolated adapters) — HW plan committed in Fugu2 repo, firmware scoping deferred"
metadata:
  node_type: memory
  type: project
  originSessionId: 0779bad4-595b-48c6-9cbd-94e145901318
  modified: 2026-09-29T09:21:58.298Z
---

HW plan: `/Users/fab/dev/ee/hw/Fugu2/plans/ext-comms-port.md` (committed ab3f865, 2026-09-29). Firmware draft (committed 62afc30, software scoping deferred by Fab): `plans/ext-serial-port.md` in this repo.

Key decisions (Fab): 4-pin JST-PH VE.Direct pinout, **no ID pin — wrong-adapter safety handled in software** (both reviewers called that a blocker; Fab accepted the residual risk). TX on GPIO36, RX on GPIO37 (existing J5). V+ from 3.3 V rail. Port + USB-C go on the bottom edge beside the power terminals (wall mount, terminals face down, no connectors on the upper side: water risk). RS-485 adapter echoes (/RE low). mcu-head header 2×7→2×9. USB1 pin header removed. Board redesign with more area + extra Cin bulk.

Found during review: 32-bit µs wrap made stale BMS cell voltage (180 s) and NTC/MCU temp (90 s) read fresh again every ~71.6 min. FIXED e8f2a71 (2026-09-29): ~1.05 s ticks via wallClockTicks()/setWallClockUs(); ibat/temp from BMS were already safe (_fresh()). Unity test built, NOT run on target (no board).

**Why:** multi-session design; firmware phase will resume later.
**How to apply:** read both plans before touching this; don't reopen the no-ID-pin decision.
