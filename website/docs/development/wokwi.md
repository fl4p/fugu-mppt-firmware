---
title: Wokwi simulator
sidebar_position: 7
description: "The firmware can run in the Wokwi simulator."
---

# Wokwi simulator

*Original hand-written note: [doc/dev-notes/simulator wokwi.md](https://github.com/fl4p/fugu-mppt-firmware/blob/main/doc/dev-notes/simulator%20wokwi.md).*

The firmware can run in the Wokwi simulator. The simulation is configured in `diagram.json` and `wokwi.toml`. For the
options in `wokwi.toml`, see the [Wokwi project configuration](https://docs.wokwi.com/vscode/project-config) docs.

`diagram.json` selects the simulated board, a classic ESP32 devkit:

```
 "parts": [ { "type": "board-esp32-devkit-c-v4", "id": "esp", "top": 0, "left": 0, "attrs": {} } ],
```

The image must therefore be an `esp32` build, while the project defaults to `esp32s3`. `wokwi.toml` loads
`build/fugu-firmware.elf` and `build/flasher_args.json`. A classic-ESP32 build lives in its own directory
(`build/esp32/`, see [Build](build.md#build-directories)), so point these two paths at that directory.

## Board configuration

`config/lab/wokwi_mock_esp32` is the profile for the simulated classic ESP32; `config/lab/wokwi_mock` is the same
profile with ESP32-S3 pins. The classic-ESP32 profile has these properties:

* `adc=fake`: Wokwi has no continuous-ADC (DMA) controller, so the internal-ADC backend gets no samples.
* `adc_fake_freq=1221`, a lower mock sample rate for better simulation speed.
* Telnet, FTP, scope and BLE are enabled. Wi-Fi credentials are not part of the profile; add the simulator's network
  with `wifi-add`. See the Wokwi docs on
  [the private gateway](https://docs.wokwi.com/guides/esp32-wifi#the-private-gateway). `wokwi.toml` forwards
  `localhost:2323` to the device's telnet port 23.

`idf.py flash` on an `esp32` build writes `config/fugu1/fugu1_esp32` into the littlefs image (`FUGU_LITTLEFS_SRC` in
the top-level `CMakeLists.txt`), and the simulator loads that image through `flasher_args.json`. Set
`FUGU_LITTLEFS_SRC` to `config/lab/wokwi_mock_esp32` for a simulator build.

The simulation ran at a speed ratio of 14-28% when last measured (2026-03), which is quite slow.

## Wokwi CLI (CI)

The Wokwi CLI runs the simulation from the command line, for example in CI. Follow the
[CLI installation guide](https://docs.wokwi.com/wokwi-ci/cli-installation), then start the simulation from the repo root:

```bash
wokwi-cli .                                   # reads wokwi.toml + diagram.json from the repo root
# or explicitly: wokwi-cli --elf build/fugu-firmware.elf .
```

## Debugging

Start the simulator and its debugger from VS Code, as described in
[Start the debugger](https://docs.wokwi.com/vscode/debugging#start-the-debugger).

To decode a backtrace, pass its `PC:SP` pairs to `xtensa-esp32-elf-addr2line` with the build ELF:

```
xtensa-esp32-elf-addr2line -pfiaC -e build/fugu-firmware.elf <pc>:<sp> <pc>:<sp> ...
```
