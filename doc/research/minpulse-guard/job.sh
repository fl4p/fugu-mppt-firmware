#!/bin/bash
# job.sh VARIANT BOARD SCENARIO SEED -> /tmp/minpulse-batch/out/VARIANT/BOARD/SCENARIO-sSEED.txt
v=$1 b=$2 sc=$3 s=$4
d=/tmp/minpulse-batch/out/$v/$b; mkdir -p $d
f=$d/$sc-s$s.txt
[ -s $f ] && grep -q "== REVERSE" $f && exit 0
case $b in fry*) nz=fry ;; *) nz=flat ;; esac
SIM_RING=$b SIM_NOISE=$nz SIM_NOISE_OFS=$((s * 37)) SIM_SEED=$s ~/bin/offload run --host pgx93 --tag minpulse-wx -- /tmp/minpulse-sim-$v $sc > $f 2>&1
