#!/bin/bash
# Build and run the host unit tests in test/host-stub (not mppt-sim, see mppt-sim.sh).
#   test/host-stub/host-tests.sh [-j N] [name ...]    default: all, N = 4
#   test/host-stub/host-tests.sh --list
# Exit status is non-zero if any test failed to build or run; logs go to $OUT (default /tmp/host-tests.<cksum>).
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT" || exit 2
CXX=${CXX:-clang++}
OUT=${OUT:-/tmp/host-tests.$(printf %s "$ROOT" | cksum | cut -d" " -f1)}
H=test/host-stub
SIM="-include $H/esp/sim_prelude.h -I $H/esp -I src -DSIM_REPO_ROOT=\"$ROOT\" -Wno-format -Wno-deprecated-volatile"
VC=src/sim/vconv.cpp

# name | compile flags | extra sources | run args
TESTS=(
  "bms-cutoff-test|-std=gnu++17 -fexceptions -I $H -I src|$VC|"
  "calib-noise-test|-std=gnu++20 -O1 -fexceptions $SIM|$H/esp/sim_stubs.cpp|"
  "charger-term-test|-std=gnu++20 -fexceptions -I $H -I $H/arduino-shim -I src||"
  "coil-fit-test|-std=gnu++20 -O2 -I src|$VC|"
  "con-guard-test|-std=gnu++20 -fexceptions -I $H -I src||"
  "conf-write-test|-std=gnu++20 -fexceptions -I $H -I src||"
  "conv-side-test|-std=gnu++17 -fexceptions -I $H -I src||"
  "coulomb-test|-std=gnu++20 -fexceptions -I $H -I src||"
  "dawn-test|-std=gnu++17 -fexceptions -I $H -I src|$VC|"
  "dcm-current-test|-std=gnu++20 -O2 -fexceptions -DMOCK=1 -I $H -I src|$VC|"
  "despike-test|-std=gnu++17 -I .||"
  "hs-short-test|-std=c++20 -I src||"
  "idle-guard-test|-std=gnu++20 -I src||"
  "integrator-test|-std=gnu++20 -fexceptions -I $H -I src||"
  "izero-test|-std=gnu++20 -O1 -fexceptions $SIM|$H/esp/sim_stubs.cpp|"
  "mailbox-test|-std=gnu++20 -O2 -I src||"
  "mcpwm-timing-test|-std=gnu++17||"
  "minpulse-test|-std=gnu++20 -O2 -fexceptions -DMOCK=1 -I $H -I src|$VC|"
  "mpsc-ring-test|-std=gnu++20 -O1 -pthread||"
  "mqtt-service-test|-std=gnu++20 -fexceptions -I $H/mqtt-stub -I $H -I src||"
  "net-guard-test|-std=gnu++20 -fexceptions -I $H -I src||"
  "out-impedance-test|-std=gnu++17 -O2 -fexceptions -I $H -I src||"
  "plot-test|-std=gnu++17 -fexceptions -I $H -I src||"
  "pv-shade-test|-std=gnu++20 -fexceptions -I $H -I src|$VC|"
  "rect-cal-test|-std=gnu++20 -O2 -fexceptions -DMOCK=1 -I $H -I src|$VC|"
  "ripple-freq-test|-std=gnu++17 -O2 -I src -I test -I $H/esp||"
  "service-test|-std=gnu++17 -fexceptions -I $H -I src||"
  "settle-test|-std=gnu++20 -O2 -fexceptions $SIM|$H/esp/sim_stubs.cpp $H/esp/sim_alloc.cpp|"
  "tele-mqtt-test|-std=gnu++20 -fexceptions -DWITH_TELE_BINARY -DWITH_TELE_MQTT -I $H/mqtt-stub -I $H -I src||"
  "tracker-window-test|-std=gnu++20 -O2 -fexceptions -DMOCK=1 -I $H -I src||"
  "vconv-test|-std=gnu++17 -fexceptions -I $H -I src|$VC|"
  "vedirect-test|-std=gnu++20||"
  "vfat-test|-std=gnu++20 -I src|src/usb/vfat.cpp|"
  "wire-guard-test|-std=gnu++20 -O2 -fexceptions -I src||"
)

# Known failures, still built and run and reported as XFAIL with the reason. "fail:" must fail (a pass is XPASS
# and fails the run); "may:" depends on the platform's libm and may pass.
known_fail() {
  case $1 in
    bms-cutoff-test) echo "fail:open battery at 3 A not detected since fb64bb1a removed the plant's LC ring-up" ;;
    rect-cal-test) echo "may:bus square 0.19 % worst case at the -45 ct limit; one more run completes with some libm sin()" ;;
  esac
}
# Not listed: scope-test.cpp (endless streaming prototype), converter-test.cpp (does not build) and the tools overshoot-plant, scan-learn-replay,
# vconv-energy-audit, wire-guard-replay, vconv-spice-check (ngspice).

if [ "$1" = --list ]; then
  for t in "${TESTS[@]}"; do echo "${t%%|*}"; done
  exit 0
fi
J=4
[ "$1" = -j ] && { J=$2; shift 2; }
mkdir -p "$OUT"

one() {
  IFS='|' read -r name flags srcs args <<< "$1"
  local log="$OUT/$name.log" t0=$SECONDS
  # shellcheck disable=SC2086
  if ! eval "$CXX $flags $H/$name.cpp $srcs -o $OUT/$name" > "$log" 2>&1; then
    echo "BUILD-FAIL $name"; return 1
  fi
  # shellcheck disable=SC2086
  local known; known=$(known_fail "$name")
  if (cd "$ROOT" && "$OUT/$name" $args) >> "$log" 2>&1; then
    [ "${known%%:*}" = fail ] && { echo "XPASS $name: drop it from known_fail"; return 1; }
    echo "PASS $name ($((SECONDS - t0)) s)"
  else
    local e=$?
    [ -n "$known" ] && { echo "XFAIL $name (exit $e): ${known#*:}"; return 0; }
    echo "FAIL $name (exit $e)"; return 1
  fi
}

SEL=("$@")
rc=0
pids=()
for t in "${TESTS[@]}"; do
  n=${t%%|*}
  if [ ${#SEL[@]} -gt 0 ] && [[ " ${SEL[*]} " != *" $n "* ]]; then continue; fi
  while [ "$(jobs -rp | wc -l)" -ge "$J" ]; do sleep 0.2; done
  one "$t" &
  pids+=($!)
done
for p in "${pids[@]}"; do wait "$p" || rc=1; done
[ $rc -ne 0 ] && echo "logs: $OUT"
exit $rc
