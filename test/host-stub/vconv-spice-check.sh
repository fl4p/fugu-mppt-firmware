#!/bin/bash
# vconv plant (fry/flat board parameters) against the LTspice bench numbers; see vconv-spice-check.cpp.
# The fitted plant must pass, the energy audit (Pout <= Pin) must pass, and the known-bad plants below must fail
# (self-test of the bands).
#   test/host-stub/vconv-spice-check.sh [-v]
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-/tmp/vconv-spice-check}
clang++ -std=gnu++20 -O2 -I "$ROOT/src" "$ROOT/test/host-stub/vconv-spice-check.cpp" "$ROOT/src/sim/vconv.cpp" -o "$OUT" || exit 2
"$OUT" "$@"
rc=$?
[ $rc = 0 ] || { echo "fitted plant: FAIL"; exit 1; }
clang++ -std=gnu++20 -O2 -I "$ROOT/src" "$ROOT/test/host-stub/vconv-energy-audit.cpp" "$ROOT/src/sim/vconv.cpp" -o "$OUT-audit" || exit 2
"$OUT-audit" || { echo "energy audit: FAIL"; exit 1; }
[ -n "$SIM_PLANT" ] && exit 0
bad=0
while read -r name plant; do
    SIM_PLANT="$plant" "$OUT" > "$OUT.$name.log" 2>&1
    r=$?
    n=$(grep -c '^FAIL:' "$OUT.$name.log")
    if [ $r = 1 ]; then echo "self-test $name: fails as expected ($n checks)"; else echo "self-test $name: PASSES (exit $r), bands too loose"; bad=1; fi
done <<'EOF'
ideal fry:ideal=1,flat:ideal=1
t_sw_x2 fry:t_sw=200e-9,flat:t_sw=200e-9
ring_q_60 fry:ring_q=60,flat:ring_q=60
EOF
exit $bad
