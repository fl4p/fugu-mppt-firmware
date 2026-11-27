# Standalone pmini dependency

The generic MTP2 implementations now live in the existing standalone pmini
repository. PMA2, telemetry schemas, transport batching, deadlines and raw/tamp
fallback remain here. This extraction does not change the wire format or defaults.

`third_party/pmini.lock.json` pins the exact upstream commit and SHA-256 of each
vendored file. The C++ wrappers retain existing build/test paths. Browser pages
load `pmini.js` before `teledec.js`; Node resolves it through CommonJS. Keep both
files together when distributing the config tool. The PMA2 decoder still enforces
its 8192-byte envelope bound before calling the generic MTP2 decoder.

The Python receiver has a separate pin in `etc/fugu/pmini.lock.json`: distribute
`pmini.py`, `_pmini_mtp2.py` and `PMINI-LICENSE` together. Its compatibility import
keeps existing `teledec.py` and receiver deployment APIs intact. Receivers already
running the old decoder remain wire-compatible and need no coordinated rollout.

To refresh a snapshot, check out the intended pmini revision, copy exactly the
source-to-path mappings listed in each lock file, and update revision and hashes.
Run upstream CMake/CTest and `tests/test_portable.py`, then fugu's native
`test/host-stub/pmini-tests.sh`, Python `test_teledec_pmini.py`, and config-tool
Node tests. Review changes to the generic codec independently of PMA2.

The upstream repository is [fl4p/pmini](https://github.com/fl4p/pmini), private
because its existing history includes vehicle-derived example profiles. Vendored
snapshots keep public firmware builds reproducible without private-repository
access. Publish upstream and fugu-py commits before the firmware integration.

## Extraction validation (2026-11-27)

Prepared from firmware `283c0248` and standalone pmini `47e3db9`, in isolated
clones. The generic native source/header and strict Python implementation were
copied unchanged. JS is the same MTP2 decoder with its PMA2-specific packet-size
cap removed and a standalone UMD export; PMA2 keeps that cap at its boundary.

- Standalone CMake/CTest native golden/invalid-input/append-CRC test passed.
- Existing standalone host run: 190 passed, 36 board/live tests deselected.
- Portable suite after adding fuzz: 4 passed, including deployed golden vectors,
  200 deterministic native round trips (including 128x64), every-bit CRC rejection,
  and 1000 CRC-valid mutation cases compared between Python and JavaScript.
- Portable suite also passed with ASAN/UBSAN native builds.
- Fugu native codec, PMA2 adapter, packet batching and MQTT host tests passed,
  both regular and ASAN/UBSAN builds. Python PMA2: 5 passed.
- Config-tool Node/jsdom suite: 393 passed, zero skips. Browser UMD dependency
  loading and website tool-copy packaging were exercised.
- Synthetic host benchmark (100x24 words, 1000 calls, `-O2`): 76-byte shard,
  38.3 us/encode. This includes no packet assembly and is not a device timing.

No new device flash, receiver deployment or full firmware build was performed.
The native wrapper cross-compiled warning-clean for ESP32 and ESP32-S3. The live
deployment remains unchanged.

## Review closure

Independent review verified codec/Python byte identity and reran portable,
PMA2 and browser-loader tests. It found that CMake Release disabled the inherited
assert-based native test. The test now explicitly keeps assertions active, and
CI runs the Release configuration. A deliberately corrupted golden compiled
with `-DNDEBUG` exits with an assertion failure; the correct golden passes and
reports 75 encoded bytes. Release CTest took 0.22 s on the host.

Test-check review: malformed inputs still fail explicitly; invalid dimensions
remain rejected at zero and the upper boundary; CMake invokes the real encoder;
the test checks exact wire bytes and the dependency manifests hash actual source;
no verification state is cached; host/cross-compile/device verification remain
distinguished above; the known-bad golden fails; assertions were restored rather
than removing the check. The native codec itself is unchanged.
