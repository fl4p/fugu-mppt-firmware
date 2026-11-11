#!/bin/sh
# Build the host simulator. SRC selects the firmware tree (default: this checkout's src/); the plant
# (src/sim/vconv.*) always comes from this checkout.
#   test/host-stub/sim/build.sh [out] ; SRC=/tmp/sim-old/src test/host-stub/sim/build.sh /tmp/sim-old.bin
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=${SRC:-$HERE/../../../src}
OUT=${1:-/tmp/fugu-sim}
clang++ -std=gnu++20 -O2 -fexceptions -DMOCK=1 -w -include "$HERE/shim/sim_prelude.h" \
    -I "$HERE/shim" -I "$SRC" "$HERE/sim.cpp" "$SRC/mppt.cpp" "$HERE/../../../src/sim/vconv.cpp" "$SRC/math/float16.cpp" -o "$OUT"
