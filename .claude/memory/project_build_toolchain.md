---
name: project_build_toolchain
description: Build/toolchain traps (IDF 6 only, newlib-nano no %hh/%ll, perf.h ODR, cli.cpp CMake branch, clangd, IDF venv vs aioesphomeapi) plus host tooling (fugu-py repo, e2e runner, conf-editor, SimpleCLI ;;)
metadata:
  type: project
---

## IDF 6 only
2026-11-23, Fab: IDF 6.0.3 only, 5.5 dropped (see CLAUDE.md Build). Don't add 5.5 shims or ask to install 5.5; any `FUGU_IDF=idf5.5` selector left is unmaintained.
IDF 6 traps (3c8a7fa port, 2026-09-29):
- Root CMakeLists must APPEND to CMAKE_CXX_FLAGS (picolibc `-specs` live there; overwrite → `__bufio_*` link errors).
- IDF 6 defaults COMPILER_DISABLE_DEFAULT_ERRORS=n.
- idf_ext.py can't redefine `flash`: wrap the callback in place, return 'version'.
- NEVER `BT_CTRL_BLE_MASTER=n` (kills all BLE connections). Size work: doc/image-size-reduction.md.

## newlib-nano printf
`CONFIG_LIBC_NEWLIB_NANO_FORMAT=y` saves ~37 KB flash (image is tight against the ~1.87 MB OTA slot); `%f` still works.
**Why:** nano silently misparses C99 length modifiers, no compile error.
**How to apply:** no `%hh`/`%ll`/`%z`/`%j`/`%t`, no `PRIx64`/`PRIu8`. Drop `hh` (varargs promote to int); 64-bit → `(long)`+`%ld` or hi/lo words (`%lX%08lX`).

## perf.h ODR
`src/etc/perf.h` *defines* `print_real_time_stats_1s_task(void*)` non-inline, so only one TU (main.cpp) may include it.
**How to apply:** from another TU forward-declare `void print_real_time_stats_1s_task(void *);` instead of including perf.h (hit when splitting cli.cpp).

## cli.cpp in CMake
Command table, `cmd*` handlers, `setupCli()`, `handleCommand()` live in `src/cli.cpp`/`cli.h`; they `extern` main.cpp globals (`loopLF`, `stopAndBackoff` made non-static). `cli.cpp` is listed in `main/CMakeLists.txt` **only in the default `else()` branch**, not for `RUN_TESTS`/`MAIN_SRC` builds (externs would be unresolved). ESP_LOG tag stays `"main"`.

## clangd for xtensa
Committed `.clangd` strips xtensa-only flags (`-mlongcalls`, `-fno-tree-switch-conversion`, `-fstrict-volatile-bitfields`, `-fno-shrink-wrap`, `-mtext-section-literals`, `--param=*`), `CompilationDatabase: build`. Newlib headers need clangd launched with `--query-driver=/Users/fab/.espressif/tools/**/xtensa-esp32s3-elf-*` (a launch flag, not a `.clangd` field; wired in `.vscode/settings.json`). CLion ignores query-driver: its global toolchain must use `xtensa-esp32s3-elf-gcc/g++` (repo profiles name toolchain `esp-idf-v5.5`, may be stale). Benign noise: `invalid target "xtensa-esp-elf"`, one `__block attribute not allowed`, `--check` `DefineOutline ==> FAIL`.

## IDF venv vs aioesphomeapi
Recorded on IDF 5.5: the IDF tools venv pins `cryptography<45`; `aioesphomeapi` (for `fugu_console.py --ble-proxy`) needs `>=48`. Symptom: `idf-export.sh` python-deps check fails on cryptography.
**How to apply:** never install console/esphome deps into the IDF venv. If it breaks: `pip install 'cryptography>=2.1.4,<45' --only-binary cryptography` + `pip uninstall aioesphomeapi` there; run `--ble-proxy` from `./.venv`. Unverified whether IDF 6's constraint differs.

## fugu-py repo
`etc/fugu/` is a separate git repo (`fl4p/fugu-py`), cloned in, not a submodule: edits there are commits to that repo. Transports/Console layout: see CLAUDE.md "Console & debugging surfaces". Firmware console verb is `svc` (not `service`). MQTT console: firmware subscribes `pv/log/<hostname>/cmd` and emits OK/ERR via untagged `UART_LOG` (dodges mqttLogCallback's self-filter); `MqttTransport(writable=False)` is a read-only monitor. `FuguDevice.get_conf_value` returns `group(2)`. The rpi's `pwr-metering/ate/fugu/` is a divergent copy patched separately.

## e2e tests
`etc/e2e-test/` (not repo root), custom subprocess runner, not pytest. `run_e2e.py` `SPECS` maps tests to clusters (console/mock/destructive/power/wifi; `--list`, `--dry-run`, `--cluster`, `--serial/--telnet`). `_harness.py`: `Results`, `wait_for`, `EventLog`, `Recorder`; no `make_console`, each `test_*.py` builds its own `Console` (idiom in `test_nettools.py`) and is a standalone script returning 0/1.

## conf-editor
- OK/ERR markers carry trailing whitespace: match with `includes("OK: "+cmd)`, never `endsWith()` (caused false "no marker" upload failures while the command applied). Keep the upload panel's live per-line log.
- 2026-08-15: `scrape_conf_keys.py --write` drops hand-maintained FILE_KEYS (`ctrl_*_kp/kd/td`, `bsync.conf`, `ble.conf` security keys, `tele.conf` `adv_ms`/`binary`/`ble`). **How to apply:** hand-edit META/FILE_KEYS (sorted, one file per entry); if you ran `--write`, diff and restore before committing.

## SimpleCLI semicolons
Vendored parser (`components/SimpleCLI/SimpleCLI/src/c/parser.c:211`): `;;` splits lines; a single `;` stays inside space-split word tokens. `script-set` joins args and splits on `;`, so `;` cannot appear in values sent through it (use FTP).
