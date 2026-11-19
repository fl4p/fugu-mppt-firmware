#!/bin/bash
# Build and run the tracker noise-threshold host test. TRK_OLD=<tracker.h> builds it against another header.
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-/tmp/tracker-noise-test}
INC=()
if [ -n "$TRK_OLD" ]; then
    OLD=$(mktemp -d)
    cp "$TRK_OLD" "$OLD/tracker.h"
    INC=(-I "$OLD")
fi
clang++ -std=gnu++20 -O1 -fexceptions -Wno-format -Wno-deprecated-volatile \
    -include "$ROOT/test/host-stub/esp/sim_prelude.h" "${INC[@]}" -I "$ROOT/test/host-stub/esp" -I "$ROOT/src" \
    "$ROOT/test/host-stub/tracker-noise-test.cpp" "$ROOT/test/host-stub/esp/sim_stubs.cpp" -o "$OUT"
"$OUT"
