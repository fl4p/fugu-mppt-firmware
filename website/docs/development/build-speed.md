---
title: Build speed
sidebar_position: 4
---

# Build speed

## Why a one-line edit triggers a multi-minute rebuild

The build has ~1568 object files (arduino-esp32 + IDF components dominate). Almost every TU
includes the generated `sdkconfig.h`, so any edit to `sdkconfig.defaults`, `CMakeLists.txt`,
`main/idf_component.yml`, or a `Kconfig` regenerates `sdkconfig.h` / reconfigures CMake and ninja
invalidates ~all 1568 TUs → near-full recompile. Source-only edits under `src/` already rebuild
minimally; it's config/CMake churn that's expensive.

## The big lever: ccache

ESP-IDF wires ccache as the compile launcher when `IDF_CCACHE_ENABLE=1`
(`$IDF_PATH/tools/cmake/project.cmake` ~L578: `set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)`).
With it, a config-triggered full recompile becomes a cache replay for every TU whose preprocessed
input + flags didn't actually change — the majority when one unrelated `CONFIG_*` is flipped. First
build after install is full (populates cache); subsequent ones are much faster.

Setup:
```bash
brew install ccache
ccache -M 10G          # arduino+idf objects are large
```
Add to the script you use to export ESP-IDF:
```sh
export IDF_CCACHE_ENABLE=1
export CCACHE_NOHASHDIR=1        # hit across git worktrees / build paths
export CCACHE_BASEDIR="$PWD"
```
`CCACHE_NOHASHDIR`/`CCACHE_BASEDIR` matter because worktree-isolated builds live at different
absolute paths — without them each worktree is a cold cache.

Verify: cmake prints `ccache will be used for faster recompilation`; watch hit-rate with `ccache -s`.

## Habits that avoid full rebuilds

- Don't edit `sdkconfig.defaults` while iterating — it forces a reconfigure + `sdkconfig.h` regen
  every time. Use `idf.py menuconfig` (touches only `sdkconfig`) for transient flag flips, or keep
  config stable and edit only `src/`. Throwaway diagnostics (e.g. heap poisoning) are better as a
  temporary `sdkconfig.local`-style toggle than a tracked `sdkconfig.defaults` change.
- Don't delete `sdkconfig` or re-run `set-target` unless it actually looks wrong.

## Already fine

ninja generator (not make), the per-file `-Os`/`-O2` split in `main/CMakeLists.txt`, and the
aggressive arduino managed-dep trimming (`override_path` stubs + `EXCLUDE_COMPONENTS`).
