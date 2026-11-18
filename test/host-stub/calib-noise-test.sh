#!/bin/bash
# Build and run the calibration noise-check host test. CALIB_REF=<rev> builds it against that revision's sampling.h.
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-/tmp/calib-noise-test}
INC=()
if [ -n "$CALIB_REF" ]; then
    OLD=$(mktemp -d)
    mkdir -p "$OLD/adc"
    git -C "$ROOT" show "${CALIB_REF}:src/adc/sampling.h" > "$OLD/adc/sampling.h"
    INC=(-DCALIB_OLD -I "$OLD" -I "$ROOT/src/adc")
fi
clang++ -std=gnu++20 -O1 -fexceptions -Wno-format -Wno-deprecated-volatile \
    -include "$ROOT/test/host-stub/esp/sim_prelude.h" "${INC[@]}" -I "$ROOT/test/host-stub/esp" -I "$ROOT/src" \
    -DSIM_REPO_ROOT="\"$ROOT\"" \
    "$ROOT/test/host-stub/calib-noise-test.cpp" "$ROOT/test/host-stub/esp/sim_stubs.cpp" -o "$OUT"
"$OUT"
