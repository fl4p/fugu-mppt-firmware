---
name: project-idf6-dual-build
description: "Firmware builds on ESP-IDF 6.0.3 as well as 5.5 (default); how, and the IDF 6 traps that bit"
metadata:
  node_type: memory
  type: project
  originSessionId: 280678b9-fa11-42c7-a1a4-02b532e3a84b
  modified: 2026-09-29T07:47:56.936Z
---

Commit 3c8a7fa (2026-09-29): builds on IDF 6.0.3 (worktree /Users/fab/dev/esp/idf6.0.3) and 5.5; **5.5 stays the default**.
Needs esp-ota-ble d11c4a2 + esp-bootguard 7ad687d + idf-devtools 4622195 (all local commits, push status: ask).

- Build IDF 6 in its own dir + sdkconfig: `-B build-idf6 -DSDKCONFIG=build-idf6/sdkconfig`. managed_components/dependencies.lock are shared → arduino-esp32 re-downloads when switching versions.
- Traps: root CMakeLists must APPEND to CMAKE_CXX_FLAGS (IDF 6 puts picolibc -specs there; overwrite → __bufio_* link errors); IDF 6 defaults COMPILER_DISABLE_DEFAULT_ERRORS=n; idf_ext.py extensions can't redefine `flash` (wrap callback in place, return 'version').
- Size: was ~29 KB free (IDF 6) / ~88 KB (5.5); ed9f647 (2026-09-29) safe set → 254 KB / 297 KB free. Canonical: doc/image-size-reduction.md. NEVER BT_CTRL_BLE_MASTER=n (kills all BLE connections). Scan-off + -Os on core-0 comps not yet bench-checked (BLE pairing/OTA, rt-stats lag).
- Wokwi run on IDF 6: S3 board + BLE off works (classic esp32 build is broken on both IDFs: GLITCH_FILTER_CLK_SRC_DEFAULT). Not yet run on hardware.

Related: [[project-wokwi-esp32-setup]]
