---
name: project_wokwi
description: Running the firmware in Wokwi: BLE off, classic-esp32 build dir + build/ symlink, littlefs override, no continuous ADC, telnet-forward wall, CI token pointer
metadata:
  type: project
---

## Setup
Re-verified 2026-05-29. `diagram.json` is a classic-ESP32 devkit-c-v4; project defaults to esp32s3.
1. **BLE off**: Wokwi BT emulation hangs IDLE0 → TWDT. `printf 'CONFIG_FUGU_WITH_BLE=n\n' > sdkconfig.wokwi`, build with `SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.wokwi"`.
2. **Classic build dir**: recipe in CLAUDE.md Build (`build/esp32`); note says `build-esp32` (pre-dates the `build/<name>` rule).
3. **littlefs override**: esp32 littlefs is hardwired to `config/fugu1/fugu1_esp32` (real ADC/WiFi); temporarily point `FUGU_LITTLEFS_SRC` at `config/lab/wokwi_mock_esp32` (adc=fake, `ssid_Wokwi=Wokwi-GUEST`, telnet on). No auto-pick.
4. **IRAM overlay**: [[project_esp32_classic_target]].
5. **`wokwi.toml` points at `build/`** (checked 2026-11-23): stash `build` and symlink it to the esp32 build dir.
6. BLE-off build once failed: `cli.cpp` coredump path called `bleConsoleAwaitTxDrain()` while `console_ble.cpp` wasn't compiled; fix was `#ifdef WITH_BLE`.

## No continuous ADC
2026-05-25: `adc=esp32adc1` inits but never yields samples ("Never got a sample!"): Wokwi lacks the DMA/continuous controller + ISR used by `ADC_ESP32_Cont`. Keep `adc=fake`; real-ADC sim would need a timer-polled oneshot backend.

## Telnet wall
Gateway needs an explicit forward: `wokwigw-darwin_arm64 --forward 2323:10.13.37.2:23` (`wokwi.toml [[net.forward]]` alone didn't work), then `wokwi-cli --timeout 200000 --timeout-exit-code 0 .`. The forward RSTs a sustained telnet session after 1-2 commands (sim RSSI ~-90 dBm), so socket e2e tests can't complete; drive the UART via wokwi-cli stdin.

## CI token
`$WOKWI_CLI_TOKEN`, value in `.claude/memory/secrets.env` (gitignored; `secrets.env.example`). `wokwi-cli` at `/Users/fab/bin/wokwi-cli`. `~/.wokwi/user.tok` is the extension license, not the CI token.
