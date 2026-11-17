#!/bin/bash
# job line: build board gain vin mode k
set -- $1
b=$1 board=$2 g=$3 vi=$4 mode=$5 k=$6
conf="sensor.conf:lv_v_gain=$g,sensor.conf:hv_v_gain=$vi"
env=(SIM_RING=$board SIM_VBAT=28)
case $mode in
  gauss) conf="$conf,vconv.conf:noise_vout=0.03,vconv.conf:noise_iout=0.02"; env+=(SIM_SEED=$k);;
  meas) nb=${board%-hq}; env+=(SIM_NOISE=$nb SIM_NOISE_OFS=$k);;
  clean) ;;
esac
env+=("SIM_CONF=$conf")
f=/tmp/demargin/dusk/out/$b-$board-$g-$vi-$mode-$k.txt
env "${env[@]}" ~/bin/offload run --host pgx93 --tag demargin-dusk -- /tmp/demargin/msim-$b wx-dusk > $f 2>&1
s=$(grep -m1 '^== SUMMARY' $f | sed -E 's/.*harvest +([0-9.]+) %.*trips ([0-9]+).*/\1 \2/')
echo "$b $board $g $vi $mode $k $s"
