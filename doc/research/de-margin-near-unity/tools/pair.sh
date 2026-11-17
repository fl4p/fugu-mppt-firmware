#!/bin/bash
# Paired old/new firmware rule over the same points (dewin fw, streamed, no files):
#   pair.sh <dewin-old> <dewin-new> <board> <vo_err> <vi_err> <stepFar> [near=600] [vinMax=80]   (DEWIN_OFF_NS passes)
# Prints counts of points with mean Iout < 0 (window = LS above the 321-count min pulse), points the new rule turns
# negative where the old one is >= 0, and the cost where the LS differs: extra loss Vin*Iin - Vout*Iout [mW] and the
# efficiency drop [pp] (both Iout > 0.05 A) per true-M bin.
OLD=$1 NEW=$2 B=$3 VO=$4 VI=$5 SF=$6 NEAR=${7:-600} VMAX=${8:-80}
paste -d' ' <(DEWIN_DUMP=1 "$OLD" "$B" "$VO" "$VI" "$SF" "$NEAR" "$VMAX" fw 2>/dev/null | grep '^P ') \
            <(DEWIN_DUMP=1 "$NEW" "$B" "$VO" "$VI" "$SF" "$NEAR" "$VMAX" fw 2>/dev/null | grep '^P ') |
awk -v b="$B" -v vo="$VO" -v vi="$VI" -v off="${DEWIN_OFF_NS:-0}" '
function bin(m) { return m < 0.5 ? 0 : m < 0.7 ? 1 : m < 0.8 ? 2 : m < 0.9 ? 3 : 4 }
{
  vin=$3; vout=$4; hs=$5; lso=$6; ioo=$8; iio=$9; lsn=$16; ion=$18; iin=$19
  if ($3 != $13 || $5 != $15) { print "MISMATCH " NR; exit 1 }
  n++; m = vout / vin; k = bin(m)
  if (ioo < 0) { if (lso > 321) { ow++; if (ioo < owm) owm = ioo } else { om++; if (ioo < omm) omm = ioo } }
  if (ion < 0) { if (lsn > 321) { nw++; if (ion < nwm) nwm = ion } else { nm++; if (ion < nmm) nmm = ion } }
  if (ion < 0 && ioo >= 0) { reg++; if (ion < regm) { regm = ion; regw = sprintf("Vin %.1f Vo %.1f HS %d LS %d->%d", vin, vout, hs, lso, lsn) } }
  if (lso != lsn) {
    d[k]++; dl = (vin*iin - vout*ion) - (vin*iio - vout*ioo); sl[k] += dl; if (dl > ml[k]) ml[k] = dl
    if (ioo > 0.05 && ion > 0.05) { de = (vout*ioo/(vin*iio) - vout*ion/(vin*iin)) * 100; if (de > me[k]) me[k] = de }
    if (ioo - ion > mdrop) mdrop = ioo - ion
  }
}
END {
  printf "%-8s off %4s Vo %+5.1f%% Vi %+5.1f%%: %6d pts | window Iout<0 old %5d (%+.3f A) new %5d (%+.3f A) | min pulse old %5d (%+.3f) new %5d (%+.3f) | new<0 where old>=0: %d (%+.3f A %s) | max Iout drop %.3f A\n",
    b, off, vo*100, vi*100, n, ow, owm, nw, nwm, om, omm, nm, nmm, reg, regm, regw, mdrop
  printf "%-8s off %4s Vo %+5.1f%% Vi %+5.1f%%:   changed LS per M bin <0.5/0.5-0.7/0.7-0.8/0.8-0.9/>0.9: n", b, off, vo*100, vi*100
  for (k = 0; k < 5; k++) printf " %d", d[k]
  printf " | extra loss max [mW]"; for (k = 0; k < 5; k++) printf " %.1f", ml[k]*1e3
  printf " mean [mW]"; for (k = 0; k < 5; k++) printf " %.2f", d[k] ? sl[k]/d[k]*1e3 : 0
  printf " | eta drop max [pp]"; for (k = 0; k < 5; k++) printf " %.3f", me[k]
  printf "\n"
}'
