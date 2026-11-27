# virtual-fat12

A small, dependency-free C++17 library that emulates a FAT12 volume over a set of in-memory files.
It exists so an embedded device (originally an ESP32-S3 power converter) can expose a configuration
folder to a host as a USB mass-storage drive without an on-device FAT filesystem.

`build()` lays the files out contiguously in a flat root directory and keeps them immutable. After
that, host writes land in RAM: the FAT and root directory are copied into RAM, and changed data
sectors are kept in a small overlay. `collect()` parses the host's view of the volume back into a
list of files, and `diff()` compares that against the files the volume was built from. The library
never touches a real filesystem, a USB stack, or any platform API.

## Public API

Everything lives in `namespace vfat` (`include/vfat/vfat.h`).

- `File { std::string name; std::string data; }`
- `Diff { std::vector<File> upsert; std::vector<std::string> removed; bool empty(); }`
- `class Volume`
  - `Volume(uint16_t maxOverlaySectors = 128, Naming naming = {})`
  - `bool build(std::vector<File> files, uint32_t volumeId)` — lays the files out; `false` if some
    were skipped (untracked name, a name with a byte FAT forbids or over 255 bytes, data over 64 KiB,
    a case-insensitive duplicate, or no space), the rest are still built. Whatever it builds,
    `collect()` reads back.
  - `bool read(uint32_t lba, uint8_t *buf)` — one 512-byte sector; `false` if out of range.
  - `bool write(uint32_t lba, const uint8_t *buf)` — `false` on out-of-range or full overlay.
  - `bool collect(std::vector<File> &out, std::string &err)` — the host's view of all tracked
    entries; `false` (with `err`) if any tracked entry is inconsistent.
  - `const std::vector<File> &baseline()` — the files the volume was built from.
  - `size_t overlaySectors()`
  - geometry constants: `SECTOR`, `TOTAL_SECTORS`, `FAT_SECTORS`, `ROOT_ENTRIES`, `ROOT_SECTORS`,
    `FAT1`, `FAT2`, `ROOT`, `DATA`, `CLUSTERS`, `MAX_FILE_BYTES`.
- `Diff diff(const std::vector<File> &base, const std::vector<File> &host)` — name comparison is
  case-insensitive; a changed file keeps its base name.

## Naming policy

Which root-directory entries count as files is a policy, passed to the constructor:

```cpp
struct Naming {
    const char *ext = nullptr;                          // tracked extension, with the dot
    bool (*valid)(const std::string &name) = nullptr;   // strict name check, or nullptr
};
```

- `ext` is compared case-insensitively as a suffix of a long name. An 8.3 short name without a
  long-name entry carries at most three extension characters, so it is matched against the
  truncated `ext` (`".conf"` -> `"CON"`).
- `ext` must outlive the `Volume` (a string literal is fine); it is stored, not copied.
- Names must be printable ASCII. `build()` skips a name with a control byte, a byte above `0x7E`, a
  FAT-reserved character (`" * / : < > ? \ |`), or a trailing space or dot, and `collect()` fails on
  such an entry rather than returning a name that differs from what was written. A long name may
  hold a byte the 8.3 alias may not (a space, `+`); those become `_` in the alias.
- `ext == nullptr` tracks every non-hidden entry (names not starting with `.`).
- `valid`, when set, must accept every tracked name; a tracked name that fails it makes `collect()`
  fail rather than dropping the file silently.
- Untracked and hidden entries are ignored, but their cluster chains are still validated, so an
  untracked entry cross-linked with a tracked one is caught.

## Scope

- Flat root directory only: no subdirectories are created or tracked.
- Long file names are supported through LFN directory entries; each file's 8.3 short name is
  derived from the file name's own base and extension.
- Geometry: 4096 sectors of 512 bytes (2 MiB), 1 sector per cluster, 4067 data clusters, 2 FATs of
  12 sectors each, 64 root entries (4 sectors). The volume is FAT12 (`CLUSTERS < 4085 - 16`).
- FAT2 reads mirror FAT1; writes to FAT2 are ignored (the second FAT is not maintained).
- Limits: at most 64 KiB per tracked file and 255 bytes per tracked name; the overlay holds 128
  written sectors (64 KiB) by default. A write past the overlay fails and is not dropped. The boot
  sector is immutable, and the volume label is fixed to `FUGU_CONF`.

## Build and test

As a standalone project (run from this directory, i.e. the package root):

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/examples/host/vfat-example
```

`test/test_vfat.cpp` is a dependency-free executable (plain `assert`, no framework). It covers the
build/collect/diff round trip, policy enforcement in `build()` and `collect()`, cross-linked and
out-of-range clusters, size/chain mismatches, looping chains, orphan and broken LFN entries,
duplicate names, the overlay cap (including that an unchanged write takes no slot), FAT2 reading as
FAT1, and the default `Naming{}` that tracks every non-hidden entry.

As an ESP-IDF component: drop the directory under `components/`; its `CMakeLists.txt` registers
`src/vfat.cpp` with `include/` as a public include directory. No other build integration is needed.

## Licence

Apache License 2.0 — see `LICENSE`. Extracted from
[fugu-mppt-firmware](https://github.com/fl4p/fugu-mppt-firmware); Copyright 2026 Fabian Schlieper
(see `NOTICE`).
