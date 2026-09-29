---
title: Build
sidebar_position: 3
---

*this document is an LLM generated placeholder*

# Build

How the ESP-IDF build is put together: feature flags, sdkconfig layering, build directories per target and variant,
and how to keep rebuilds fast. Installing ESP-IDF is covered in [Getting started](../guide/getting-started/index.mdx).

## Quick start

```bash
. $IDF_PATH/export.sh              # ESP-IDF 5.5+, once per shell
idf.py set-target esp32s3          # once per build dir
idf.py build
idf.py -p $ESPPORT app-flash       # firmware only, keeps the board config
```

## Feature flags

Compile-time features are Kconfig options `CONFIG_FUGU_WITH_*` (menu **"Fugu MPPT firmware"**, defined in
`main/Kconfig.projbuild`). The full table and the build environment variables (`RUN_TESTS`, `MAIN_SRC`, `FUGU_BAT_V`,
`FUGU_DEVICE`) are in [Build options](../guide/getting-started/build-options.md).

## sdkconfig layering

`sdkconfig` is generated and gitignored; the tracked sources are `sdkconfig.defaults` plus fragments. The top-level
`CMakeLists.txt` resolves `CONFIG_FUGU_WITH_BLE`, `CONFIG_FUGU_WITH_NETW` and `CONFIG_FUGU_WITH_SPROFILER` *before*
`project()` runs Kconfig, by reading the defaults chain and any existing `sdkconfig`, and then:

| Condition | Effect |
|---|---|
| `CONFIG_FUGU_WITH_BLE=y` | appends `sdkconfig.ble` (NimBLE host and radio tuning) |
| `CONFIG_FUGU_WITH_NETW=n` | appends `sdkconfig.no_netw` |
| `CONFIG_FUGU_WITH_SPROFILER=n` | excludes the `esp32-semihosting-profiler` component |
| target `esp32` | ESP-IDF adds `sdkconfig.defaults.esp32` |

Later fragments win. Pass your own with `SDKCONFIG_DEFAULTS`, quoting the list:

```bash
SDKCONFIG_DEFAULTS="sdkconfig.defaults;my.frag" idf.py -B build-myvariant build
```

`sdkconfig.vconv_s3` is the prepared fragment for a virtual-converter build on ESP32-S3; see
[Build options](../guide/getting-started/build-options.md#control-loop-work-without-hardware). If the generated
`sdkconfig` looks wrong, delete it and rebuild.

:::warning One sdkconfig per project root
`-B` only moves the build directory. The generated `sdkconfig` stays in the project root and is shared by every
build dir, so building a different target or variant rewrites it. Give a variant its own file with
`-DSDKCONFIG=<path>`, or use `etc/matrix_build.sh`, which isolates each variant.
:::

## Build directories

| Directory | Command | Purpose |
|---|---|---|
| `build/` | `idf.py build` | Default, ESP32-S3 |
| `build-esp32/` | `idf.py -B build-esp32 set-target esp32`, then `idf.py -B build-esp32 build` | Classic ESP32 |
| `build-tests/` | `RUN_TESTS=1 idf.py -B build-tests build` | Unity test runner, see [Testing](testing.md) |
| `build-<variant>/` | `SDKCONFIG_DEFAULTS=… idf.py -B build-<variant> build` | Any other flag combination |

All `build-*` directories are gitignored.

:::note Flashing includes a config image
`idf.py flash` also writes a littlefs image built from `config/lab/dry_mock` (ESP32-S3) or
`config/fugu1/fugu1_esp32` (ESP32), set by `littlefs_create_partition_image()` in `CMakeLists.txt`. On a configured
board use `idf.py app-flash`. Both commands archive the build ELF for later coredump decoding (`idf_ext.py`).
:::

## Variant matrix

`etc/matrix_build.sh` builds the common variants, each in its own symlinked project root under
`build-matrix-roots/<name>/` so they do not share `sdkconfig`, `managed_components/` or `dependencies.lock`:

| Variant | Target | BLE | NETW |
|---|---|:---:|:---:|
| `s3-baseline` | esp32s3 | on | on |
| `s3-noble` | esp32s3 | off | on |
| `s3-nonetw` | esp32s3 | on | off |
| `s3-headless` | esp32s3 | off | off |
| `esp32-ble` | esp32 | on | on |
| `esp32-noble` | esp32 | off | on |

Logs go to `build-matrix-logs/<name>.log`; the summary lists exit code and `fugu-firmware.bin` size per variant.
The script sources `./idf-export.sh`, a local, gitignored helper that sources ESP-IDF's `export.sh`; create your own
or source ESP-IDF before running it.

## Build speed and size

- Source-only edits under `src/` rebuild minimally. Edits to `sdkconfig.defaults`, `CMakeLists.txt`,
  `main/idf_component.yml` or Kconfig regenerate `sdkconfig.h` and recompile almost everything.
- Enable ccache with `IDF_CCACHE_ENABLE=1`; see [Build speed](build-speed.md) for setup and habits.
- `main/CMakeLists.txt` compiles the real-time path (`main.cpp`, `mppt.cpp`, `adc_esp32_cont.cpp`) with the default
  `-O2` and cold code (console, services, telemetry) with `-Os`.
- Check the image against the OTA slot with `idf.py size`; see [Binary size](binary-size.md).
