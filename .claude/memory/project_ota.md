---
name: project_ota
description: OTA lessons beyond CLAUDE.md — fleet OTA history/owner check, silent download failures, BLE-push gotchas (ota-ble verb, skip-on-same-version, mbufs), stale :9000 server, esptool --diff-with
metadata:
  type: project
---

Rollback, boot watchdog, `ota_mqtt.py`, bootloader OTA and radio release: see CLAUDE.md "OTA". Generic ESP32 OTA/boot facts: Canonical: ~/dev/kb/esp32/boot-and-ota.md and ~/dev/kb/protocol/ota-success-needs-a-changed-slot-not-a-reboot.md.

## Fleet OTA history and owner check
Latest recorded: 2026-11-24 ~15:30 UTC fry+flat on 1009-g44e16128 (origin/main, `etc/fleet-build.sh 44e16128`; see [[project_charger_termination]]). Before that, 13:47 UTC: both on 958-ga52c7d7c (= origin/main 99004a3a + cherry-picked 783c87c5/954dcb63; a52c7d7c is on no branch, local main and origin had diverged; bin sha b414ffef). Earlier 11-23/24: 826 -> 840 -> 869 -> 881 (flat VERIFY FAILED, 1 fault line) -> 894-gcc30ca19 -> 911-g4f1e4879 -> 926 -> 929 -> 933/954 (flat measure-coil tests, L 49.7/48.4 uH) -> 956 -> 958 -> 1009.
- **Why:** several sessions OTA'd fry/flat independently the same day despite a coordinator.
- **How to apply:** before building, read `elf-archive/ota-owners.jsonl` + `bootinfo`. Compare commits by subject, not hash (rebases: dabb7a37 etc. were already in 894 under other shas).
- Every reboot resets holiday "today: discharging" counters (11.5 h lost on 926) — candidate fix: keep them in RTC.
- ChargerRtc grew 40->48 B in 4f1e4879; ota_mqtt's RTC check then reports `s_chargerRtc resized 0x28->0x30` — expected.
- telnet.conf `trusted=1` set on fry+flat 11-23 (since 894 telnet without it is read-only). tlscon bench: ~24 KB heap per session, too much for fry/flat.
- Config cleanup 11-23: panel_sd, board.conf ina22x_resistor/ov/conv_time/range, sensor.conf expected_hz/power_conversion_eff/conversion_eff deleted; bflow_sd=40 on both.

## MQTT console vs telnet transport
2026-11-12: OTA over the NAT telnet tunnel ran ~14 KB/s and hit the (then 120 s) OTA watchdog at 97-98 %; the same image over the MQTT console took 37-40 s (42-45 KB/s). The NAT port mapping also flipped between connections ([[reference_device_access]]). Prefer MQTT. 08cf227 raised OTA_TIMEOUT_SECONDS to 5 min.
- **Silent failure (11-12):** fry's OTA stopped at 1 %, no error, back on MPPT on the old image; retry 10 min later worked (58 s). The `App:` line `ota` prints is the OLD version — confirm with `bootinfo` after reboot.

## Conversion stop during OTA
FIXED fw 116 (2026-06-11), refined 11-12: app `ota` latches manual 0, waits for the converter to disable, halts the ADC, restores the previous mode if the download fails. Do NOT `dc 0` manually (it skips that restore).
- Lesson that explains it: at ~300 W the flash erase/write stalls the RT loop and the latency/no-sample watchdog reset fry mid-download (87 %, 47 %); reset reason is serial-only, so MQTT just shows uptime reset + old version. Related [[project_rt_loop_latency]].

## Bricked by a bad OTA (2026-05-28, resolved)
GPIO-ISR deadlock image ([[project_crash_rootcauses]]) hung in setup() on fry & flat: no rollback, no watchdog over setup(). Fixed by 97f47a8 + 9ed2078 (now described in CLAUDE.md "OTA safety net"). Recovery was serial only: units wired to havan `/dev/ttyACM0`, esptool + bins staged at `havan:~/fugu_flash/`; flash bootloader+parttable+ota_data+app, omit littlefs (0x3b9000) and nvs (0x9000) to keep config.

## etc/ota.py and the :9000 server
`ota.py` builds `http://<ip>:9000/build/fugu-firmware.bin` and reads `Last-Modified` from a HEAD. `err <ip> flat 'last-modified'` = whatever listens on :9000 serves the wrong dir (404). Since 11-14 `ensure_http_server()` starts one when :9000 is free but REUSES a stale one. Fix: `lsof -nP -iTCP:9000 -sTCP:LISTEN`, `lsof -a -p <pid> -d cwd`, kill, rerun from repo root; check `curl -sI` gives 200 + Last-Modified.
- Verify the image's gate driver in `build/config/sdkconfig.h` before OTAing a converter (gitignored sdkconfig decides MCPWM).

## OTA over BLE (etc/ota_ble.py)
Validated 2026-05-29 on live flat (1.79 MB, rollback=VALID). Firmware `src/etc/ota_ble.cpp`; FW char `6E400004` (write-no-rsp), control `ota-ble begin <size> <sha256>|end|abort`, status lines `OTAB READY/CRED/PROG/OK/FAIL`. CLI verb is **`ota-ble`**; `otab` is only the log tag (the old host bug). RING_CAP 8 KB (bench board has no PSRAM). All OTA state mutates on the net loop; disconnect only sets an abort flag.
- **Same-version skip prints success:** `☑️ skip: already at …` then `✅ success` without pushing. After a rebuild with the same version string (`-dirty`) pass `-f`; never grep out `skip`.
- One BLE OTA at a time per host radio (two in parallel stalled at ~11 %).
- A failed push leaves the receiver active (`OTAB FAIL already-active`): send `ota-ble abort` over the console, then retry.
- NimBLE mbuf exhaustion after many macOS reconnects: first write fails ATT 17 "Insufficient Resource" — reboot the device first.
- macOS caches GATT per address: after a GATT change `blueutil -p 0 && blueutil -p 1`. `find_device_by_name` misses devices (local_name omitted); ota_ble.py matches both names.
- Takes ~57 s clean, 2-3 min with drops: run in background, not under a 120 s tool timeout. Wait ~15 s after a reboot before scanning.
- A no-BLE image pushed over BLE kills the BLE path; ota_ble.py scans the image for BLE markers and asks first. Size-check WITH_BLE builds against the 0x1c9000 slot.

## esptool --diff-with
Needs esptool >= 5.2 (entry point `esptool`, v4 is `esptool.py`; v4 misreads the snapshot path as an address). `etc/idf-devtools/flash-diff.sh` version-gates it; if it full-flashes, check `$ESPTOOL` is v5. (IDF 5.5 shipped v4.11; IDF 6 ships v5.)
