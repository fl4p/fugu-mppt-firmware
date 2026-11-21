#!/bin/bash
# Build the host closed-loop sim (mppt-sim.cpp) and run the given scenarios (default: all).
#   test/host-stub/mppt-sim.sh [scenario ...]
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-/tmp/mppt-sim}
clang++ -std=gnu++20 -O2 -fexceptions -Wno-format -Wno-deprecated-volatile \
    -include "$ROOT/test/host-stub/esp/sim_prelude.h" -I "$ROOT/test/host-stub/esp" -I "$ROOT/src" \
    -DSIM_REPO_ROOT="\"$ROOT\"" \
    "$ROOT/test/host-stub/mppt-sim.cpp" "$ROOT/test/host-stub/esp/sim_stubs.cpp" "$ROOT/test/host-stub/esp/sim_alloc.cpp" \
    "$ROOT/src/mppt.cpp" "$ROOT/src/adc/sensor_setup.cpp" "$ROOT/src/sim/vconv.cpp" "$ROOT/src/math/float16.cpp" "$ROOT/src/selftest/bflow_test.cpp" -o "$OUT"
[ $# -eq 0 ] && set -- sanity a b c d-cloud d-cloud-ramp d-bus04 d-bus08 d-bus12 d-noise e e-bleed a-p0 b-p0 c-p0 e-p0 d-cloud-p0 d-cloud-ramp-p0 d-bus12-p0 d-noise-p0 \
    wx-cloud wx-cloud-n1 wx-cloud-cv wx-cloud-cv-ripple wx-shade2 wx-shade3 wx-shade2-move wx-shade3-move wx-shade-walk wx-shade-cloud wx-shade-learn \
    wx-dawn wx-dusk wx-low wx-low-slow wx-mid wx-calfail wx-sweep-fall wx-sweep-rise wx-sweep-fall-n1 wx-sweep-rise-n1 wx-sweep-one wx-sweep-shade wx-sweep-gust wx-sweep-preempt wx-sweep-learn wx-sweep-learn-off wx-sweep-scan wx-sweep-return rcal rcal-bus rcal-cloud rcal-ov rcal-low rcal-low-bus rcal-sun rcal-flat idle-wake \
    hs-short hs-short-clamp hs-short-bfclosed hs-night hs-night-abort hs-cal hs-dawn hs-dawn-nozero hs-volt hs-volt-clamp hs-run hs-run-lsdead hs-run-nodecl hs-unplug hs-glitch \
    bf-ok bf-open bf-closed bf-nodecl bf-daily bf-daily-flat bf-daily-fail bf-daily-then-closed bf-abort bf-abort-cmd
# every scenario runs; the exit status reports whether any failed
rc=0
for s in "$@"; do
    echo "######## $s"
    "$OUT" "$s" || { e=$?; rc=1; echo "######## $s exit $e"; }
done
exit $rc
