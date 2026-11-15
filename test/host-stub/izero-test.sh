#!/bin/bash
# Build and run the Iout zero-calibration host test. IZERO_OLD=1 runs it against HEAD's sampling.h.
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-/tmp/izero-test}
INC=()
if [ -n "$IZERO_OLD" ]; then
    OLD=$(mktemp -d)
    mkdir -p "$OLD/adc"
    git -C "$ROOT" show HEAD:src/adc/sampling.h > "$OLD/adc/sampling.h"
    INC=(-DIZERO_OLD -I "$OLD" -I "$ROOT/src/adc")
fi
clang++ -std=gnu++20 -O1 -fexceptions -Wno-format -Wno-deprecated-volatile \
    -include "$ROOT/test/host-stub/esp/sim_prelude.h" "${INC[@]}" -I "$ROOT/test/host-stub/esp" -I "$ROOT/src" \
    -DSIM_REPO_ROOT="\"$ROOT\"" \
    "$ROOT/test/host-stub/izero-test.cpp" "$ROOT/test/host-stub/esp/sim_stubs.cpp" -o "$OUT"
"$OUT"
