#!/bin/bash
# Build the host closed-loop sim (mppt-sim.cpp) and run the given scenarios (default: all).
#   test/host-stub/mppt-sim.sh [scenario ...]
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-/tmp/mppt-sim}
clang++ -std=gnu++20 -O2 -fexceptions -Wno-format -Wno-deprecated-volatile \
    -include "$ROOT/test/host-stub/esp/sim_prelude.h" -I "$ROOT/test/host-stub/esp" -I "$ROOT/src" \
    -DSIM_REPO_ROOT="\"$ROOT\"" \
    "$ROOT/test/host-stub/mppt-sim.cpp" "$ROOT/test/host-stub/esp/sim_stubs.cpp" \
    "$ROOT/src/mppt.cpp" "$ROOT/src/adc/sensor_setup.cpp" "$ROOT/src/sim/vconv.cpp" "$ROOT/src/math/float16.cpp" -o "$OUT"
[ $# -eq 0 ] && set -- sanity a b c d-cloud d-cloud-ramp d-bus04 d-bus08 d-bus12 d-noise e e-bleed a-p0 b-p0 c-p0 e-p0 d-cloud-p0 d-cloud-ramp-p0 d-bus12-p0 d-noise-p0
for s in "$@"; do
    echo "######## $s"
    "$OUT" "$s"
done
