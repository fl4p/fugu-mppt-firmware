#!/bin/bash
# job line: build scenario board vout_gain vin_gain mode k
set -- $1
b=$1 sc=$2 board=$3 g=$4 vi=$5 mode=$6 k=$7
conf="sensor.conf:lv_v_gain=$g,sensor.conf:hv_v_gain=$vi"
env=(SIM_RING=$board SIM_VBAT=28)
case $sc in wx-dusk|wx-dawn) ;; *) env=(SIM_RING=$board);; esac
case $mode in
  gauss) conf="$conf,vconv.conf:noise_vout=0.03,vconv.conf:noise_iout=0.02"; env+=(SIM_SEED=$k);;
  meas) nb=${board%-hq}; env+=(SIM_NOISE=$nb SIM_NOISE_OFS=$k);;
  clean) ;;
esac
env+=("SIM_CONF=$conf")
f=/tmp/dusk/out/$b-$sc-$board-$g-$vi-$mode-$k.txt
if [ ! -s $f ] || ! grep -q '^== SUMMARY' $f; then
  env "${env[@]}" ~/bin/offload run --host pgx93 --tag dusk-trap -- /tmp/dusk/msim-$b $sc > $f 2>&1
fi
s=$(grep -m1 '^== SUMMARY' $f | sed -E 's/.*harvest +([0-9.]+) %.*trips ([0-9]+).*/\1 \2/')
echo "$b $sc $board $g $vi $mode $k $s"
