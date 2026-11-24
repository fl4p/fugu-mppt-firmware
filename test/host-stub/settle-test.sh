#!/bin/bash
# Build and run settle-test.cpp against the host-sim stubs.
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-/tmp/settle-test}
clang++ -std=gnu++20 -O2 -fexceptions -Wno-format -Wno-deprecated-volatile \
    -include "$ROOT/test/host-stub/esp/sim_prelude.h" -I "$ROOT/test/host-stub/esp" -I "$ROOT/src" \
    -DSIM_REPO_ROOT="\"$ROOT\"" \
    "$ROOT/test/host-stub/settle-test.cpp" "$ROOT/test/host-stub/esp/sim_stubs.cpp" "$ROOT/test/host-stub/esp/sim_alloc.cpp" -o "$OUT"
"$OUT"
