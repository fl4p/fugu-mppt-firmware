#!/bin/bash
# Build the host closed-loop sim (mppt-sim.cpp) and run the given scenarios (default: all).
#   test/host-stub/mppt-sim.sh [scenario ...]
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
# per checkout: a shared path is rebuilt by other trees mid-run
OUT=${OUT:-/tmp/mppt-sim.$(printf %s "$ROOT" | cksum | cut -d" " -f1)}
clang++ -std=gnu++20 -O2 -fexceptions -Wno-format -Wno-deprecated-volatile \
    -include "$ROOT/test/host-stub/esp/sim_prelude.h" -I "$ROOT/test/host-stub/esp" -I "$ROOT/src" \
    -DSIM_REPO_ROOT="\"$ROOT\"" \
    "$ROOT/test/host-stub/mppt-sim.cpp" "$ROOT/test/host-stub/esp/sim_stubs.cpp" "$ROOT/test/host-stub/esp/sim_alloc.cpp" \
    "$ROOT/src/mppt.cpp" "$ROOT/src/adc/sensor_setup.cpp" "$ROOT/src/sim/vconv.cpp" "$ROOT/src/math/float16.cpp" "$ROOT/src/selftest/bflow_test.cpp" -o "$OUT"
[ $# -eq 0 ] && set -- sanity a b c d-cloud d-cloud-ramp d-bus04 d-bus08 d-bus12 d-noise e e-bleed a-p0 b-p0 c-p0 e-p0 d-cloud-p0 d-cloud-ramp-p0 d-bus12-p0 d-noise-p0 \
    wx-cloud wx-cloud-n1 wx-cloud-cv wx-cloud-cv-ripple wx-shade2 wx-shade3 wx-shade2-move wx-shade3-move wx-shade-walk wx-shade-cloud wx-shade-learn \
    wx-dawn wx-dusk wx-low wx-low-slow wx-mid wx-calfail wx-sweep-fall wx-sweep-rise wx-sweep-fall-n1 wx-sweep-rise-n1 wx-sweep-one wx-sweep-shade wx-sweep-gust wx-sweep-preempt wx-sweep-learn wx-sweep-learn-off wx-sweep-scan wx-sweep-return wx-bms-late wx-bms-none wx-bms-full wx-bms-high wx-ovl-shade-hold wx-ovl-sun wx-ovl-ctl rcal rcal-bus rcal-cloud rcal-ov rcal-low rcal-low-bus rcal-sun rcal-flat idle-wake \
    hs-short hs-short-clamp hs-short-unidir hs-short-bfclosed hs-night hs-night-dsp hs-night-abort hs-night-ov hs-night-ovpend hs-night-ovhold hs-night-race hs-night-race0 hs-cal hs-dawn hs-dawn-nozero hs-volt hs-volt-clamp hs-run hs-run-lsdead hs-run-nodecl hs-unplug hs-glitch \
    bf-rec bf-ok bf-ok-dcm bf-open bf-closed bf-nodecl bf-uni-ok bf-uni-closed bf-uni-open bf-uni-abort bf-uni-offdrop bf-uni-cloud bf-uni-dcm bf-uni-slow bf-uni-soft-ok bf-uni-soft-closed bf-nobat-ok bf-nobat-closed bf-nobat-reconnect bf-nobat-packsense bf-nobat-bms bf-nobat-sawbat bf-daily bf-daily-uni bf-daily-flat bf-daily-fail bf-daily-then-closed bf-abort bf-abort-cmd bf-flat-ccm bf-flat-ccm-open bf-flat-ccm-grow bf-flat-ccm-vinfall mc-flat mc-flat-lowf mc-flat-ramp \
    dcd-stop-flat dcd-stop-fry dcd-slow-flat dcd-slow-fry dcd-ccm-flat dcd-ccm-fry \
    cv-hold-pair cv-hold-offset cv-hold-gap cv-hold-load cv-hold-reboot cv-hold-mask cv-hold-day
# every scenario runs; the exit status reports whether any failed
rc=0
for s in "$@"; do
    echo "######## $s"
    "$OUT" "$s" || { e=$?; rc=1; echo "######## $s exit $e"; }
done
exit $rc
