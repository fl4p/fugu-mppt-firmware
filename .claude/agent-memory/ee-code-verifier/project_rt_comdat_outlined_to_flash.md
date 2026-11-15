---
name: rt-comdat-outlined-to-flash
description: Header-inline RT helpers (RectCal::feed, RectCalCtl::hold/drop, rectCalHold) silently get outlined into flash when they grow; rtprof_map --check cannot see it
metadata:
  type: project
---

Growing an inline header function used on the RT core can make GCC emit it out of line as a weak
COMDAT symbol in flash (0x42...). `etc/rtprof_map.py --check` only checks symbols NAMED in the .lf
fragments, so it still reports "0 not in IRAM". Seen 2026-11-14: RectCal::feed/abort,
RectCalCtl::hold went to flash; force-inlining hold then pushed MpptController::rectCalHold out instead.

**Why:** a flash-resident RT function stalls the control loop for the whole flash op (tens of ms).

**How to apply:** after touching RT header code, `xtensa-esp32s3-elf-nm -C <elf> | grep <Class>` and look
for 0x42 addresses; fix with `__attribute__((always_inline))` (single call site) per
doc/flash-write-rt-latency.md. Then scan the IRAM callers' l32r literal pools for 0x42xxxxxx values
(pyelftools; float constants like 0x42c80000=100.0f are false positives).
