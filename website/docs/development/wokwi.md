---
title: Wokwi simulator
sidebar_position: 7
---

# Wokwi simulator

The firmware can run in the Wokwi simulator. The simulation is configured in `diagram.json` and `wokwi.toml`. For the
options in `wokwi.toml`, see the [Wokwi project configuration](https://docs.wokwi.com/vscode/project-config) docs.

`diagram.json` selects the simulated board, a classic ESP32 devkit:

```
 "parts": [ { "type": "board-esp32-devkit-c-v4", "id": "esp", "top": 0, "left": 0, "attrs": {} } ],
```

The image must therefore be an `esp32` build, while the project defaults to `esp32s3`. `wokwi.toml` loads the image
from `build/`.

## Board configuration

The `config/lab/wokwi_mock` configuration has these properties:

* It connects to the mock WiFi. See the Wokwi docs on
  [the private gateway](https://docs.wokwi.com/guides/esp32-wifi#the-private-gateway).
* It uses a lower-frequency interrupt timer for better performance.
* The simulation runs at a speed ratio of 14-28%, which is quite slow.

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
