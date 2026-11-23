---
title: Build
sidebar_position: 3
---

# Build

This page explains how the ESP-IDF build is put together: feature flags, sdkconfig layering, build directories per
target and variant, and how to keep rebuilds fast. [Getting started](../guide/getting-started/index.mdx) covers
installing ESP-IDF.

## Quick start

These commands set up the shell, build the firmware, and flash it:

```bash
. $IDF_PATH/export.sh              # ESP-IDF 6.0 (5.5 also works), once per shell
idf.py set-target esp32s3          # once per build dir
idf.py build
idf.py -p $ESPPORT app-flash       # firmware only, keeps the board config
```

## Feature flags

Compile-time features are Kconfig options `CONFIG_FUGU_WITH_*` (menu "Fugu MPPT firmware", defined in
`main/Kconfig.projbuild`). [Build options](../guide/getting-started/build-options.md) has the full table and the build
environment variables (`RUN_TESTS`, `MAIN_SRC`, `FUGU_DEVICE`).

## sdkconfig layering

`sdkconfig` is generated and gitignored. The tracked sources are `sdkconfig.defaults` plus fragments.

The top-level `CMakeLists.txt` resolves `CONFIG_FUGU_WITH_BLE`, `CONFIG_FUGU_WITH_NETW`, `CONFIG_FUGU_WITH_USB_MSC`,
and `CONFIG_FUGU_WITH_SPROFILER` *before* `project()` runs Kconfig. It reads the defaults chain and then the live
sdkconfig, and the later value wins. The live sdkconfig is the file given with `-DSDKCONFIG=<path>`, else the
project-root `sdkconfig` if it exists. The resolved options then select fragments and components:

| Condition | Effect |
|---|---|
| `CONFIG_FUGU_WITH_BLE=y` | appends `sdkconfig.ble` (NimBLE host and radio tuning) |
| `CONFIG_FUGU_WITH_NETW=n` | appends `sdkconfig.no_netw` |
| `CONFIG_FUGU_WITH_USB_MSC=y` | appends `sdkconfig.usb_msc` (USB-Serial-JTAG console off); `main/idf_component.yml` adds `espressif/tinyusb` |
| `CONFIG_FUGU_WITH_SPROFILER=n` | excludes the `esp32-semihosting-profiler` component |
| target `esp32` | ESP-IDF adds `sdkconfig.defaults.esp32` |

Later fragments win. To pass your own fragments, set `SDKCONFIG_DEFAULTS` and quote the list:

```bash
SDKCONFIG_DEFAULTS="sdkconfig.defaults;my.frag" idf.py -B build/myvariant build
```

`sdkconfig.vconv_s3` is the prepared fragment for a virtual-converter build on ESP32-S3. See
[Build options](../guide/getting-started/build-options.md#control-loop-work-without-hardware) for its use. Build it in its own
directory with its own `sdkconfig`:

```bash
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.vconv_s3" idf.py -B build/vconv -DSDKCONFIG=build/vconv/sdkconfig build
```

If the generated `sdkconfig` looks wrong, delete it and rebuild.

The defaults only seed options that the existing `sdkconfig` does not set yet, so a changed or newly added default
is ignored in a build directory that already has one. Configure then prints a warning listing each key whose
`sdkconfig` value contradicts the defaults chain (`sdkconfig.defaults*`, their `.<target>` variants and the appended
fragments), with the fix: delete that `sdkconfig` and reconfigure. A fresh `sdkconfig` can contradict a default by
itself, when another option `select`s the symbol or the default value is invalid; those keys are recorded in
`<sdkconfig>.fugu-baseline` when the file is generated and are not reported.

:::warning One sdkconfig per variant
`-B` only moves the build directory. Without `-DSDKCONFIG`, the generated `sdkconfig` stays in the project root and
every build dir shares it, so building a different target or variant rewrites it.

Pass `-DSDKCONFIG=<build-dir>/sdkconfig` with every command of a variant. ESP-IDF then generates that file instead,
and the top `CMakeLists.txt` reads it (not the root `sdkconfig`) to decide the BLE, no-netw and USB MSC fragments and
the profiler exclusion. A relative path is taken from the project root. The fragment list still comes from `SDKCONFIG_DEFAULTS`,
so pass the variant's `CONFIG_FUGU_WITH_*` there too.

`etc/matrix_build.sh` avoids the shared file entirely by building each variant in its own project root.
:::

## Build directories

Each target or variant has its own build directory:

| Directory | Command | Purpose |
|---|---|---|
| `build/` | `idf.py build` | Default, ESP32-S3 |
| `build/esp32/` | `IDF_TARGET=esp32 idf.py -B build/esp32 -D SDKCONFIG=build/esp32/sdkconfig build` | Classic ESP32 |
| `build/tests/` | `RUN_TESTS=1 idf.py -B build/tests build` | Unity test runner, see [Testing](testing.md) |
| `build/<variant>/` | `SDKCONFIG_DEFAULTS=… idf.py -B build/<variant> -D SDKCONFIG=build/<variant>/sdkconfig build` | Any other flag combination |

Put every extra build directory under `build/`, not next to it as `build-<name>/`. `build/` is gitignored.
`idf.py fullclean` on the default directory deletes everything in `build/`, the variant directories included.

:::note Flashing includes a config image
`idf.py flash` also writes a littlefs image built from `config/lab/dry_mock` (ESP32-S3) or
`config/fugu1/fugu1_esp32` (ESP32), set by `littlefs_create_partition_image()` in `CMakeLists.txt`. On a configured
board, use `idf.py app-flash`. Both commands archive the build ELF for later coredump decoding (`idf_ext.py`).
:::

## Variant matrix

`etc/matrix_build.sh` builds the common variants, each in its own symlinked project root under
`build/matrix-roots/<name>/`, so they don't share `sdkconfig`, `managed_components/`, or `dependencies.lock`:

| Variant | Target | BLE | NETW |
|---|---|:---:|:---:|
| `s3-baseline` | esp32s3 | on | on |
| `s3-noble` | esp32s3 | off | on |
| `s3-nonetw` | esp32s3 | on | off |
| `s3-headless` | esp32s3 | off | off |
| `esp32-ble` | esp32 | on | on |
| `esp32-noble` | esp32 | off | on |

The script writes logs to `build/matrix-logs/<name>.log`. Its summary lists the exit code and `fugu-firmware.bin`
size per variant.

The script sources `./idf-export.sh`, which sources ESP-IDF's `export.sh` from `FUGU_IDF_PATH`, from
`$FUGU_ESP_DIR/<version>`, or from `../../esp/<version>` relative to the main checkout (default version `idf6.0.3`,
`FUGU_IDF` picks another).

## Build speed and size

Rebuild time and image size depend on these points:

- Source-only edits under `src/` rebuild minimally. Edits to `sdkconfig.defaults`, `CMakeLists.txt`,
  `main/idf_component.yml`, or Kconfig regenerate `sdkconfig.h` and recompile almost everything.
- Enable ccache with `IDF_CCACHE_ENABLE=1`; see [Build speed](build-speed.md) for setup and habits.
- `main/CMakeLists.txt` compiles the real-time path (`main.cpp`, `mppt.cpp`, `adc_esp32_cont.cpp`) with the default
  `-O2` and cold code (console, services, telemetry) with `-Os`.
- Check the image against the OTA slot with `idf.py size`; see [Binary size](binary-size.md).
