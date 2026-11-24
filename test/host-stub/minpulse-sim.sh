#!/bin/bash
# Closed-loop check of the buck LS min pulse ratio limit in mppt-sim (fry board plant): Vin through 1.25..1.02 x Vout
# and back from a weak panel (wx-mp-ramp), a stiff sinking source (wx-mp-psu, manual duty: wx-mp-manual), a bench PSU
# (wx-mp-psu-nosink), a charging battery entering DCM from CCM near unity (wx-mp-ccm), and
# wx-dusk. Per run, from the == REVERSE / == SUMMARY lines:
#   no time switching at a plant ratio above the min pulse's carry-over onset 1 - t_min/T
#   no revI-ewm trip, plant output current (20 ms means) above -0.5 A
#   at most 3 minpulse-M stops (no chatter)
#   wx-mp-manual (manual duty): at most 3 converter starts (the restart after a stop waits for the start ratio)
#   wx-mp-ccm: the stop follows the converter's CCM -> DCM entry within 5 ms (no debounce when DCM is entered above
#   the stop ratio)
#   test/host-stub/minpulse-sim.sh      SIM_BIN=<prebuilt mppt-sim> skips the build; -j via JOBS (default 4)
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
JOBS=${JOBS:-4}
if [ -z "$SIM_BIN" ]; then
    SIM_BIN=${OUT:-/tmp/mppt-sim.$(printf %s "$ROOT" | cksum | cut -d" " -f1)}
    OUT=$SIM_BIN "$ROOT/test/host-stub/mppt-sim.sh" sanity > /dev/null
fi
export SIM_BIN
check() {
    out=$(SIM_V=1 "$SIM_BIN" "$1")
    awk -v sc="$1" '
        function num(re) { return match($0, re) ? substr($0, RSTART, RLENGTH) : "" }
        /^== REVERSE/ { im = num("mean min -?[0-9.]+"); sub(/.* /, "", im)
                        ta = num("t_min/T\\) [0-9.]+"); sub(/.* /, "", ta)
                        st = num("stops [0-9]+"); sub(/.* /, "", st)
                        ns = num("starts [0-9]+"); sub(/.* /, "", ns) }
        /^== SUMMARY/ { rev = ($0 ~ /revI/) }
        /converter: CCM -> DCM/ { tDcm = $1 }
        /backoff 5s \[minpulse-M\]/ && lag == "" { lag = sprintf("%.1f", ($1 - tDcm) * 1000) }
        END { bad = ""
              if (im == "") bad = bad " no-REVERSE-line"
              if (ta + 0 > 0) bad = bad " above-onset=" ta "s"
              if (rev) bad = bad " revI-trip"
              if (im + 0 < -0.5) bad = bad " Iout=" im
              if (st + 0 > 3) bad = bad " stops=" st
              if (sc == "wx-mp-manual" && ns + 0 > 3) bad = bad " starts=" ns
              if (sc == "wx-mp-ccm" && (lag == "" || lag + 0 > 5)) bad = bad " DCM-entry-to-stop=" lag "ms"
              printf "%-18s Iout min %7s A, above onset %5s s, minpulse-M stops %s, starts %s, revI %d%s  %s\n", sc, im,
                     ta, st, ns, rev, (sc == "wx-mp-ccm" ? ", DCM entry to stop " lag " ms" : ""),
                     (bad == "" ? "ok" : "FAIL" bad) }' <<< "$out"
}
export -f check
printf '%s\n' wx-mp-ramp wx-mp-psu wx-mp-psu-nosink wx-mp-manual wx-mp-ccm wx-dusk | xargs -P "$JOBS" -I{} bash -c 'check {}' > /tmp/minpulse-sim.$$
cat /tmp/minpulse-sim.$$
! grep -q FAIL /tmp/minpulse-sim.$$
rc=$?
rm -f /tmp/minpulse-sim.$$
exit $rc
