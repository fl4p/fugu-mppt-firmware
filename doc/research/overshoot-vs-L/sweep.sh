#!/bin/bash
# Overshoot sweep: Cout x L0 x trip rate x sample phase, scenario a (battery opens at ~28 A).
#   sweep.sh BIN OUTDIR [NPHASES] [RUNNER]   RUNNER e.g. "offload run --host pgx93 --tag ov --"
BIN=$1 OUT=$2 NPH=${3:-32} RUN=${4:-}
P=${P:-4}
mkdir -p "$OUT" && cd "$OUT" || exit 1
for C in 470 560; do for L in 28 30 35 40 45 50 60 70 80 100; do for hz in 200 450 5000; do
    for ((k = 0; k < NPH; k++)); do echo "$C $L $hz $((k * 1000000 / NPH / hz))"; done
done; done; done > jobs.txt
export BIN RUN
xargs -P "$P" -L 1 bash -c 'f=C$0_L$1_$2_$3.log; grep -q "^== peak" $f 2>/dev/null && exit 0; SIM_PHASE_US=$3 SIM_ADC_HZ=$2 SIM_CONF="coil.conf:L0=$1e-6,vconv.conf:plant_c_out=$0e-6" $RUN $BIN a > $f 2>&1; exit 0' < jobs.txt
for f in C*.log; do
    printf "%s " $(basename "$f" .log | tr _ ' ' | tr -d CL)
    awk '/^== first protection/{w=$4; t=$(NF-1)} /^== peak/{printf "%s %s %s", w, t, $(NF-1)}' "$f"
    grep -q "fL > 1" "$f" && printf "REJECT"
    echo
done > results.txt
wc -l < results.txt
