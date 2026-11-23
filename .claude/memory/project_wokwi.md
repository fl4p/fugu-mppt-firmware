---
name: Wokwi: setup, no continuous ADC, telnet wall
description: Running the firmware in Wokwi: BLE off, build dir, littlefs override, no continuous ADC, telnet-forward wall
created: 2026-11-23T09:46:22.848Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

`diagram.json` is a classic-ESP32 devkit-c-v4 board, but the project defaults to esp32s3. Running it
in Wokwi needs (re-verified 2026-05-29):

1. **BLE off** — Wokwi's BT emulation hangs IDLE0 and trips the TWDT. The old `WITH_BLE=0` env is
   rejected now (Kconfig migration); use a fragment: `printf 'CONFIG_FUGU_WITH_BLE=n\n' >
   sdkconfig.wokwi`, built with `SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.wokwi"`.
2. **Separate build dir** — `IDF_TARGET=esp32 idf.py -B build-esp32 set-target esp32 build`
   (`. ./idf-export.sh` exports esp32s3, so override on the line).
3. **littlefs override** — `CMakeLists.txt` hardwires the esp32 littlefs to `config/fugu1/fugu1_esp32`
   (real ADC + real WiFi, never joins Wokwi-GUEST). Temporarily point it at
   `config/lab/wokwi_mock_esp32` (adc=fake, `ssid_Wokwi=Wokwi-GUEST`, telnet on). There is **no**
   auto-pick of a wokwi config.
4. **IRAM overlay** — see [[project_esp32_classic_target]].
5. **`wokwi.toml` points at `build/`**, not `build-esp32/`. After building: `mv build build.s3.stash
   && ln -s build-esp32 build`, so the symlink matches diagram.json and wokwi-cli's `./build`
   autodetect doesn't conflict.
6. **The BLE-off build was broken**: `cli.cpp`'s coredump path calls `bleConsoleAwaitTxDrain()`,
   but `main/CMakeLists.txt` sets `BLE_SRC ""` with BLE off, so `console_ble.cpp` (the no-op stub)
   isn't compiled → undefined reference. Fix: guard the call with `#ifdef WITH_BLE`.

**No continuous ADC.** `sensor.conf adc=esp32adc1` configures fine in Wokwi (init + attenuation
logged) but **no samples ever arrive** ("Never got a sample! Please check ADC"). Wokwi implements
the ADC RTC peripheral but not the DMA/continuous controller + notification ISR that
`ADC_ESP32_Cont` (`src/adc/adc_esp32_cont.h`) uses. Keep `adc=fake` in Wokwi; real-ADC simulation
would need a timer-polled `ADC_ESP32_Oneshot` backend. Found 2026-05-25 wiring pots to GPIO
36/34/39.

**Telnet-forward caveat (the wall).** For a network console, run the gateway with an explicit
forward — `wokwigw-darwin_arm64 --forward 2323:10.13.37.2:23` (deterministic DHCP 10.13.37.2; the
`wokwi.toml [[net.forward]]` alone did not take effect) — then `wokwi-cli --timeout 200000
--timeout-exit-code 0 .`. But that forward **RSTs a sustained telnet connection after 1-2 commands**
(sim WiFi RSSI ~-90 dBm), so a socket-transport e2e test can't complete on Wokwi. Drive the UART via
wokwi-cli stdin instead.

CI access: the Wokwi CLI credential note (memory search "wokwi").
