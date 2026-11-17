#!/bin/bash
# usage: simrun.sh <mppt-sim binary> <out.tsv>   (env GAINS = Vout reading gains, VIN_GAIN; runs on pgx93 via offload)
# mppt-sim scenarios near Vin = Vout on the fry plant (ring on) with a Vout reading gain error.
BIN=$1 OUTF=$2
jobs=()
for g in ${GAINS:-1 0.983 0.975}; do
  for s in "wx-sun-fry VOC=48 P=5" "wx-sun-fry VOC=48 P=15" "wx-sun-fry VOC=44 P=15" "wx-sun-fry VOC=52 P=30" \
           "wx-shade3 -" "wx-dawn -" "wx-dusk -" "wx-cloud-cv -" "wx-low -"; do
    jobs+=("$g $s")
  done
done
run1() {
  set -- $1
  g=$1 sc=$2; shift 2
  env=(SIM_RING=fry "SIM_CONF=sensor.conf:lv_v_gain=$g,sensor.conf:hv_v_gain=${VIN_GAIN:-1}" SIM_VBAT=28)
  for kv in "$@"; do [ "$kv" != - ] && env+=("SIM_$kv"); done
  out=$(env "${env[@]}" ~/bin/offload run --host pgx93 --tag demargin-sim -- "$BIN" "$sc" 2>&1)
  sum=$(grep -m1 '^== SUMMARY' <<< "$out" | sed 's/  */ /g')
  oe=$(sed -n 's/.*output energy \([0-9.]*\) Wh.*/\1/p' <<< "$out" | head -1)
  rev=$(grep -c 'Low current\|Reverse\|Low I, set LS' <<< "$out")
  echo -e "$g\t$sc ${*}\t$oe\t$rev\t$sum"
}
export -f run1; export BIN VIN_GAIN
printf "%s\n" "${jobs[@]}" | xargs -P 16 -I{} bash -c 'run1 "{}"' | sort > "$OUTF"
