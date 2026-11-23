---
name: NAT router wedge modes + liveness watchdog
description: NAT router wedges (STA-stuck, OTA lockup), the liveness watchdog that self-recovers, and how to tell it apart from a dead converter
created: 2026-11-23T09:46:51.553Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

The gateway for fry/flat (they sit on its 192.168.4.x AP, port-forwarded as
192.168.1.231:232-235) is an ESP32 running the NAT router + BLE-proxy firmware. Current address
**192.168.1.231** (older notes say 192.168.1.173 — it moved; confirm with `ping`/`/appinfo`). It runs
`nimble-ble-proxy` (`/Users/fab/dev/ha/nimble-ble-proxy`, ESP32-S3) and is also the ESPHome
`--ble-proxy` host. Firmware upstream is `dchristl/esp32_nat_router_extended`; the patched fork is
[[reference_fl4p_nat_router_fork]].

**Failure A — STA-stuck, AP alive (2026-05-22→23, .173, unpatched upstream).** The STA drops off the
upstream WiFi while the soft-AP stays up, so `http://192.168.4.1/` still answers from its AP. It can
stay wedged for many hours. Hitting the web UI appears to revive it (any `esp_wifi_*` call from a
non-event-loop path kicks the driver out of the stuck state) — consistent with the tight
`esp_wifi_connect()`-in-`STA_DISCONNECTED`-handler radio wedge.

**Failure B — full lockup under an OTA (2026-06-11, .231).** An OTA of fry through the router wedged
it ~13% into the ~1.6 MB download: the HTTP server saw BrokenPipe, then *both* converters went silent
on the broker, the havan log collector stopped, telnet via NAT timed out, and the router answered
neither ping nor its web UI (so the web-UI revival trick didn't apply — physical power-cycle). The
devices were fine, still converting on the AP; fry stayed on the old image. Same class as A, but
total.

**Liveness watchdog (deployed 2026-06-11).** `components/liveness_wdt/`, wired in `main.cpp` after
`wifi_sta::start_and_wait_for_ip()`. A task probes upstream reachability (STA default gateway :80 and
broker 192.168.1.200:1882) with non-blocking connect+select; a cycle fails only if **both** are
unreachable (one filtered/down target must not cause reboots); after `threshold` consecutive failures
it `esp_restart()`s. Config: `CONFIG_NBP_LIVENESS_WDT=y`, `_INTERVAL_SECS=30`, `_THRESHOLD=6`
(~3 min). Runtime: `GET /liveness` (state json), `POST /liveness?enabled=0|1&interval=N&threshold=N`
(NVS-persisted, live off-switch), `POST /liveness?test=1` (force reboot to prove the path);
`last_ok_s:null` means still inside the 90 s boot grace. A task-based watchdog is needed because the
IDF Task-WDT never fires on a WiFi/coex livelock where tasks live but IP is dead.

**Gotchas that bit while building it.**
- A new IDF component plus a new CONFIG symbol added together: the first `idf.py build` leaves the
  component lib **empty** (its CMake `if(CONFIG_...)` saw a stale symbol) while the caller's `#if`
  sees it true → undefined-reference link errors. Run `idf.py reconfigure` first.
- A `.cpp` whose top-level `#ifdef CONFIG_*` guard precedes any esp header compiles to **nothing** —
  IDF does not force-include sdkconfig.h. Put `#include "sdkconfig.h"` first.
- Router OTA: `curl --data-binary @build/nimble_ble_proxy.bin http://192.168.1.231/update` (reboots on
  success). The pre-fix image lacked OTA-quiesce, so the upload can crawl or drop under coex
  contention — just retry (a 2nd attempt went at 107 KB/s). Verify with `/appinfo` (elf_sha256
  changes) and `/liveness` returning 200. fry/flat ride the reboot on the AP.

**How to apply.** When fry/flat go silent, check the router before blaming the firmware: if the router
answers ping but the converters can't reach the broker, or nothing answers at all, it is the router.
Both telnet-over-NAT OTA (`etc/ota.py`) and MQTT-triggered OTA pull the image through it, so both can
wedge it — confirm `ping 192.168.1.231` before declaring a converter dead.
