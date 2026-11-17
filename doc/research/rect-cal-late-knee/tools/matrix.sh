#!/bin/bash
# usage: matrix.sh <sim-bin> <out.tsv> [seeds]
BIN=$1 OUTF=$2 SEEDS=${3:-3}
jobs=()
for b in ${BOARDS:-fry fry-hq flat flat-hq}; do
 for voc in ${VOCS:-58 68 79.6}; do for isc in 3 6; do for k in 0.72 0.80; do for s in $(seq 1 $SEEDS); do
  jobs+=("$b $voc $isc $k $s")
 done; done; done; done
done
run1() {
  set -- $1
  b=$1 voc=$2 isc=$3 k=$4 s=$5
  case $b in fry*) hz=513;; *) hz=1469;; esac
  out=$(SIM_RING=$b SIM_ADC_HZ=$hz SIM_SEED=$s SIM_PV="$isc,$voc,$k" "$BIN" rcal-pv 2>&1)
  st=$(sed -n 's/^== rect-cal: \(done\|failed: [^a]*\) after.*/\1/p' <<< "$out" | head -1)
  [ -z "$st" ] && st=$(grep -m1 '^== rect-cal: ' <<< "$out" | cut -c1-60)
  l=$(grep -m1 '^== rect-cal: ideal' <<< "$out")
  t=$(grep -m1 '^== truth' <<< "$out")
  hs=$(sed -n 's/.*cal HS \([0-9]*\) reached.*/\1/p' <<< "$out" | head -1)
  ideal=$(sed -n 's/.*ideal \([0-9.]*\) .*/\1/p' <<< "$l"); peak=$(sed -n 's/.* peak \([0-9.]*\) .*/\1/p' <<< "$l")
  le=$(sed -n 's/.*learned \([-+0-9]*\).*/\1/p' <<< "$l"); ap=$(sed -n 's/.*applied \([-+0-9]*\).*/\1/p' <<< "$l")
  bound=$(grep -c '(bound)\|clamped' <<< "$l")
  read vi vo zc tr i0 zq <<< "$(sed -n 's/.*plant Vin \([0-9.]*\) Vout \([0-9.]*\) at HS [0-9]*: zero crossing \([-0-9.na]*\) ct, \([-+0-9.na]*\) ct.*HS turn-on \([-+0-9.na]*\) A; charge zc \([-+0-9.na]*\) ct.*/\1 \2 \3 \4 \5 \6/p' <<< "$t")"
  echo -e "$b\t$voc\t$isc\t$k\t$s\t${st// /_}\t$hs\t$ideal\t$vi\t$vo\t$tr\t$i0\t$zq\t$peak\t$le\t$ap\t$bound"
}
export -f run1; export BIN
printf "%s\n" "${jobs[@]}" | xargs -P 4 -I{} bash -c 'run1 "{}"' > "$OUTF.tmp"
( echo -e "board\tvoc\tisc\tk\tseed\tstatus\ths\tideal\tvin\tvout\ttruth\ti0\tzcq\tpeak\tlearned\tapplied\tbound"; sort "$OUTF.tmp" ) > "$OUTF"; rm "$OUTF.tmp"
wc -l "$OUTF"
