#!/bin/bash
# vconv plant (fry/flat board parameters) against the LTspice bench numbers; see vconv-spice-check.cpp.
#   test/host-stub/vconv-spice-check.sh [-v]
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-/tmp/vconv-spice-check}
clang++ -std=gnu++20 -O2 -I "$ROOT/src" "$ROOT/test/host-stub/vconv-spice-check.cpp" "$ROOT/src/sim/vconv.cpp" -o "$OUT"
"$OUT" "$@"
