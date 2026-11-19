#!/bin/bash
# rect-cal regression on the SPICE-fitted board plants (SIM_RING fry, fry-hq, flat, flat-hq): `rect-cal force` from
# panels with Voc 48 / 68 / 79.6 V (calibration Vin about 1.6 / 2.4 / 2.9 x Vout), Isc 3 / 6 A, fill factor 0.72 / 0.80,
# ADC noise seeds 1..N. The plant has no driver or gate delay, so the timing truth of the offset is 0; its zero
# crossing at the calibration point (truth, printed by mppt-sim) is off ideal by the ring current at HS turn-on.
# Pass band per finished run, in ns:
#   applied <= +125 (RectCal::LateMaxNs), and <= 0 from a calibration point above Vin = 2 Vout (a bound there)
#   learned >= -1250: the body diode carries at most the last 1.25 us of the fall (11 mW at 0.7 V, 51 uH, 39 kHz)
#   |charge zc - plant zero crossing| <= 300: the charge-based reference (RectCal::zc) tracks the plant
# per board: learned spread (sd) <= 350 ns (the knee referenced to ideal alone scatters by about 450 ns with the ring),
# mean learned within 150 ns of its baseline (BASE, 4 seeds on the Mac; seeds 1-2 vs 3-4 differ by up to 104 ns per
# board), and at least 75 % of the runs finish (a failed run keeps the previous offset). All boards: mean learned within
# 100 ns of the pooled baseline. A bias drift of +200 ns fails the pooled mean, +400 ns every board. The bound share
# (rect_offset_cal 2) is printed, not checked.
#   test/host-stub/rcal-vin.sh [seeds]       seeds default 4 (the baselines need them); SIM_BIN=<prebuilt mppt-sim> skips
#                                            the build; -j via JOBS (default 4)
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SEEDS=${1:-4}
JOBS=${JOBS:-4}
if [ -z "$SIM_BIN" ]; then
    SIM_BIN=${OUT:-/tmp/mppt-sim}
    OUT=$SIM_BIN "$ROOT/test/host-stub/mppt-sim.sh" sanity > /dev/null
fi
export SIM_BIN
run1() {
    set -- $1
    case $1 in fry*) hz=513 ;; *) hz=1469 ;; esac
    out=$(SIM_RING=$1 SIM_ADC_HZ=$hz SIM_SEED=$5 SIM_PV="$3,$2,$4" "$SIM_BIN" rcal-pv)
    awk -v b="$1" -v voc="$2" -v isc="$3" -v k="$4" -v s="$5" '
        /^# gate timing/ { for (i = 1; i < NF; ++i) if ($i == "count") cn = $(i + 1) }
        /^== rect-cal: (done|failed)/ { st = ($3 == "done") ? "done" : substr($0, index($0, "failed: ") + 8); sub(/ after.*/, "", st) }
        /^== rect-cal: ideal/ { for (i = 1; i < NF; ++i) { if ($i == "learned") le = $(i + 1); if ($i == "applied") ap = $(i + 1) }
                                lv = $NF }
        /^== truth/ { vr = $5 / $7; for (i = 1; i < NF; ++i) { if ($i == "crossing") zc = $(i + 1); if ($i == "ideal" && !id) id = $(i + 1);
                                                 if ($i == "zc") zq = $(i + 1) } sub(/,$/, "", zc); tr = zc - id }
        END { gsub(/ /, "_", st)
              if (st == "") st = "no-run"
              if (le == "") le = ap = lv = "-"
              printf "%s %s %s %s %s %s %s %s %s %s %s %s %s\n", b, voc, isc, k, s, st, le, ap, (tr == "") ? "-" : tr, (zq == "") ? "-" : zq - tr,
                     lv, vr, cn }' <<< "$out"
}
export -f run1
jobs=()
for b in fry fry-hq flat flat-hq; do for voc in 48 68 79.6; do for isc in 3 6; do for k in 0.72 0.80; do
    for s in $(seq 1 "$SEEDS"); do jobs+=("$b $voc $isc $k $s"); done
done; done; done; done
printf "%s\n" "${jobs[@]}" | xargs -P "$JOBS" -I{} bash -c 'run1 "{}"' | sort -k1,1 -k2,2n -k3,3n -k4,4 -k5,5n | awk '
    # counts are converted with the run's count duration (# gate timing: 6.25 ns MCPWM, 12.53 ns LEDC)
    BEGIN { split("fry 162 fry-hq 234 flat -231 flat-hq -190", bl, " ")
            for (i = 1; i < 8; i += 2) base[bl[i]] = bl[i + 1]; pooled = -9; printf "%-8s %5s %4s %5s %4s %-22s %9s %9s %9s %9s\n", "board", "Voc", "Isc", "k", "seed",
            "result", "learned", "applied", "truth", "zc-truth" }
    { r = $6; bad = ""
      if (r == "done") {
          ns = $13; le = $7 * ns; ap = $8 * ns; tr = $9 * ns; dz = $10 * ns
          if (ap > 125.5 || (ap > 0 && $12 > 2)) bad = bad " applied>" (($12 > 2) ? 0 : 125)
          if (le < -1250) bad = bad " learned<-1250"
          if (dz > 300 || dz < -300) bad = bad " |zc-truth|>300"
          n[$1]++; nb[$1] += ($11 == 2); sl[$1] += le; sl2[$1] += le * le; if (!($1 in amin) || ap < amin[$1]) amin[$1] = ap; if (!($1 in amax) || ap > amax[$1]) amax[$1] = ap
          printf "%-8s %5s %4s %5s %4s %-22s %6.0f ns %6.0f ns %6.0f ns %6.0f ns %s\n", $1, $2, $3, $4, $5, r, le, ap, tr, dz, (bad == "") ? "ok" : "FAIL" bad
      } else { printf "%-8s %5s %4s %5s %4s %-22s %9s %9s %9s %9s failed\n", $1, $2, $3, $4, $5, r, "-", "-", "-", "-"; nf++ }
      runs++; if (bad != "") fails++ }
    END { for (b in n) { m = sl[b] / n[b]; sd = sqrt(sl2[b] / n[b] - m ^ 2); bs = nb[b] / n[b]; f = ""
              if (sd > 350) f = f " sd>350"
              if (!(b in base) || m - base[b] > 150 || m - base[b] < -150) f = f sprintf(" mean off baseline %+d", base[b])
              if (f != "") fails++
              N += n[b]; S += sl[b]
              printf "%-8s %d done: learned mean %+.0f ns sd %.0f, %d bounds (%.0f %%), applied %+.0f..%+.0f ns%s\n", b, n[b], m, sd,
                     nb[b], 100 * bs, amin[b], amax[b], (f == "") ? "" : " FAIL" f }
          pm = N ? S / N : 0; pf = pm - pooled > 100 || pm - pooled < -100; fails += pf
          printf "all boards: learned mean %+.0f ns (baseline %+d)%s\n", pm, pooled, pf ? " FAIL off baseline > 100" : ""
          printf "%d runs, %d outside the band, %d failed (offset kept)\n", runs, fails, nf
          exit (fails > 0 || nf > runs / 4) }'
