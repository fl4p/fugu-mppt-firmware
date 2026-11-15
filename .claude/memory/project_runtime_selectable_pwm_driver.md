---
name: project_runtime_selectable_pwm_driver
description: Gate driver is a BUILD-TIME Kconfig choice since 1e5c022 (2026-10-09); converter.conf::pwm_driver is obsolete (warn only). Runtime LEDC/MCPWM selection is gone.
metadata:
  node_type: memory
  type: project
  originSessionId: d05671fd-aaeb-4962-a292-4102649291ac
  modified: 2026-10-10T16:49:19.888Z
---

Since 1e5c022 (2026-10-09) the gate driver is `choice FUGU_GATE_DRIVER`: `FUGU_GATE_LEDC` |
`FUGU_WITH_MCPWM` (default) | `FUGU_WITH_VCONV`; LEDC is the default only on classic ESP32
(`default ... if IDF_TARGET_ESP32`). `FUGU_WITH_LEDC` now only means the LEDC peripheral (fan PWM, `anaw`);
the LEDC gate driver `select`s it. Code: `WITH_LEDC_GATE` picks the legacy slot in `buck.h`; both
drivers compiled = `#error`.

`converter.conf::pwm_driver` is dropped; a leftover key logs a WARN after drvInit (user chose warn, not
refuse). fry/flat actually run the **LEDC** gate driver (fry's littlefs had `pwm_driver=ledc`; the 2026-10-31 OTA was built with `CONFIG_FUGU_GATE_LEDC=y`). Build LEDC for them unless the user decides to switch.

**Why:** the user found the runtime key + LEDC/MCPWM/VCONV dependency web confusing.
**How to apply:** `etc/ota.py` (a2a46be) and `etc/ota_ble.py` (edac4a8) probe the device with a no-arg
`pwm-freq` and ask before pushing across a driver change (`--allow-driver-change` skips the prompt).
Probe validated 2026-11-04 on live fry+flat via Tailscale tunnel: both reply `pwm-freq: n/a` (LEDC) -> 'other'; an MCPWM build/ -> 'mismatch', as intended. The full ota.py/ota_ble.py push flow is still unrun. Reused sdkconfigs keep their old MCPWM bit; only fresh ones get the per-target default.
Old LEDC-default profiles (fugu1_esp32s3, f2_test, solar-boost, psu_12v) now run MCPWM on S3 builds.

**MCPWM migration prep (2026-11-08):** user waived the split-dt bench check, MCPWM is the default for fry/flat now.
Image built clean from 84a48eb in worktree `../fugu-wt-mcpwm/build-mcpwm` (sha fd530975…, same version string as the
deployed LEDC image -> ota.py needs -f). Live config read 11-08: both HiLi, pins hi21/li14/sd47, no dead-time keys
(driver-chip interlock). flat coil.conf rect_offset_ns=875 (+legacy rect_offset=78), converter.conf pwm_driver=mcpwm.
fry has NO rect_offset_ns (legacy rect_offset=100 ignored -> 0 today), converter.conf pwm_driver=ledc (will warn).
**flat migrated to MCPWM 2026-11-08 18:48 UTC** (telnet `ota http://192.168.1.200:9000/fugu-firmware-mcpwm-84a48eb.bin`,
image served by `python3 -m http.server` in havan:~/fugu-ota, pid in http.pid). Verified: period_ticks=4103 @160 MHz,
rect_offset 875 ns = 140 ct, rollback confirmed at 22.6 s, ELF archived. Done at night (0 W), so NOT yet seen under load.

**fry migrated to MCPWM 2026-11-09 09:35 UTC** at ~217 W (ota ramps down itself). period_ticks=4103, rect_offset 0 ns,
rollback confirmed 22 s, steady loop lag 4.6-4.9 ms (= LEDC). One-off 42 ms lag peak right after boot = OTA-valid
otadata flash write while converting (inferred from timing; reset-lag cleared it, no recurrence). flat ran 358-475 W
on MCPWM on 11-09 morning, no ADC/latency errors. BOTH NOW MCPWM: build fry/flat with the default config.
Obsolete keys deleted 2026-11-12 ~14:30 UTC via del-config (pwm_driver both, coil rect_offset both, flat converter.conf boost+vout_max). del-config while converting ~120 W peaked flat loop lag at 62 ms with no trip.
havan:~/fugu-ota still holds the image (server stopped). Worktree removed.

