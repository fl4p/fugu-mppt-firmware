---
title: Build speed
sidebar_position: 4
---

# Build speed

## Rebuild cost

Edits to config and CMake files are expensive, while source-only edits under `src/` rebuild
minimally. The build has about 1,500 object files (1,568 in an earlier count; the number varies with the feature flags), and arduino-esp32 and the IDF components dominate them.
Almost every TU includes the generated `sdkconfig.h`.

An edit to `sdkconfig.defaults`, `CMakeLists.txt`, `main/idf_component.yml`, or a `Kconfig`
regenerates `sdkconfig.h` or reconfigures CMake. Ninja then invalidates almost every TU, which means a
near-full recompile.

## ccache

With ccache, a config-triggered full recompile becomes a cache replay for every TU whose preprocessed
input and flags didn't change. That covers most of them when one unrelated `CONFIG_*` is flipped.
ESP-IDF wires ccache as the compile launcher when `IDF_CCACHE_ENABLE=1`
(ESP-IDF 6.0.3 `$IDF_PATH/tools/cmake/project.cmake` ~L615: `set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)`).

The first build after you install ccache is a full build that populates the cache. Later builds are
much faster.

To set up ccache, install it and raise the cache size:
```bash
brew install ccache
ccache -M 10G          # arduino+idf objects are large
```
Then export these variables in your shell profile, or before sourcing `./idf-export.sh` (which does not set them):
```sh
export IDF_CCACHE_ENABLE=1
export CCACHE_NOHASHDIR=1        # hit across git worktrees / build paths
export CCACHE_BASEDIR="$PWD"
```
Worktree-isolated builds live at different absolute paths, so `CCACHE_NOHASHDIR`/`CCACHE_BASEDIR`
matter. Without them, each worktree starts with a cold cache.

To verify the setup, check that cmake prints `ccache will be used for faster recompilation`. Watch
the hit rate with `ccache -s`.

## Habits that avoid full rebuilds

These habits keep rebuilds small:

- Keep `sdkconfig.defaults` unchanged while iterating, because each edit forces a reconfigure and a
  `sdkconfig.h` regeneration. For transient flag flips, use `idf.py menuconfig`, which touches only
  `sdkconfig`. Alternatively, keep the config stable and edit only `src/`.
- Make throwaway diagnostics, such as heap poisoning, a temporary `sdkconfig.local`-style toggle
  rather than a tracked `sdkconfig.defaults` change.
- Delete `sdkconfig` or re-run `set-target` only when it looks wrong.

## Optimizations already in place

The build already uses the following:

- the ninja generator (not make)
- the per-file `-Os`/`-O2` split in `main/CMakeLists.txt`
- aggressive trimming of arduino managed dependencies (`override_path` stubs and `EXCLUDE_COMPONENTS`)
