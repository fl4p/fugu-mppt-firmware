---
name: project_nat_router
description: NAT router .231 (nimble-ble-proxy) wedge modes incl. OTA-through-it lockup, liveness watchdog endpoints, router OTA recipe, fl4p fork branches, paused .231 inverter switch
metadata:
  type: project
---

Gateway for fry/flat (their 192.168.4.x AP, forwarded as 192.168.1.231:232-235) is an ESP32-S3 running `nimble-ble-proxy` (`/Users/fab/dev/ha/nimble-ble-proxy`), also the ESPHome `--ble-proxy` host. Older notes say .173: it moved; confirm with ping / `/appinfo`.

## Wedge modes
- A, 2026-05-22 (.173, upstream fw): STA drops off upstream WiFi while the soft-AP lives, for hours; hitting the web UI revived it (driver stuck in `esp_wifi_connect()`-in-disconnect-handler).
- B, 2026-06-11 (.231): an OTA of fry pulled through the router locked it fully at ~13 % (no ping, no web UI) — power-cycle needed. Converters kept converting, fry stayed on the old image.
- **How to apply:** when fry/flat go silent, `ping 192.168.1.231` before blaming the converter; telnet-NAT and MQTT-triggered OTAs both pull through it and can wedge it.

## Liveness watchdog
Deployed 2026-06-11. `components/liveness_wdt/`: probes STA gateway :80 and broker 192.168.1.200:1882; a cycle fails only if both fail; `esp_restart()` after `THRESHOLD=6` x `INTERVAL_SECS=30` (~3 min). `GET /liveness` (`last_ok_s:null` = in 90 s boot grace), `POST /liveness?enabled=0|1&interval=N&threshold=N` (NVS), `POST /liveness?test=1` (forced reboot). Task-based because the IDF TWDT never fires on a WiFi/coex livelock.
- Build gotchas: new component + new CONFIG symbol together leaves the lib empty on first build — `idf.py reconfigure` first. A `.cpp` whose top-level `#ifdef CONFIG_*` comes before any esp header compiles to nothing: include `sdkconfig.h` first.
- Router OTA: `curl --data-binary @build/nimble_ble_proxy.bin http://192.168.1.231/update`; retry if it crawls; verify `/appinfo` elf_sha256 changed and `/liveness` 200.

## fl4p/esp32_nat_router_extended fork
Fork of `dchristl/esp32_nat_router_extended` (the earlier upstream firmware). Branches (each with a `driver/gpio.h` include fix for IDF 5.5): `liveness-watchdog`, `upload-firmware-button` (`POST /uploadfw`), `editable-ota-url`, `all-fixes` (flash target). S3 with 4 MB flash needs `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y` + `board_upload/board_build.flash_size = 4MB` (else size-mismatch panic). ~92.5 % of the 1.5 MB slot. `pio run -e esp32-s3`. Only bench S3 9c:13:9e:f2:8b:50 ran it; the field router now runs nimble-ble-proxy.

## .231 inverter switch (paused 2026-11-17)
GPIO14 inverter ext-switch needs a high-side stage (port needs pull-up to BAT+; low-side TK6R7A10PL alone cannot switch it); Fab building a TP2104 stage. GPIO14 ON, boot policy `last`. Resume from `/Users/fab/dev/ha/nimble-ble-proxy/plans/2026-11-17-inverter-switch-handoff.md` (a4afa5f). Reach .231 only via havan; notify c4 before and after any .231 restart.
