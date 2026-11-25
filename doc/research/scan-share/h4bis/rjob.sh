#!/bin/bash
# runs ON pgx93 (inside offload hold): rjob.sh "tree variant board ofs [EXTRA=..]" -> /tmp/h4bis/rout/<tree>.<key>.txt
set -- $1
tree=$1 var=$2 board=$3 ofs=$4; shift 4
BIN=/tmp/h4bis/bin-$tree
f=/tmp/h4bis/rout/$tree.h4.$var.$board.$ofs.txt
case $var in base) up=0.97;; *) up=$(awk -v x="${var#u}" 'BEGIN{printf "%.3f", x/1000}');; esac
mkdir -p /tmp/h4bis/rout
[ -s "$f" ] && grep -q '^== PROBES' "$f" && exit 0
env -i PATH=/usr/bin:/bin SIM_CONF=tracker.conf:pv_substrings=6 SIM_RING=$board SIM_NOISE=$board SIM_NOISE_OFS=$ofs \
    SIM_SHADE=1,0.75,v,1,0.3,600 SIM_VOCDRIFT=-0.12,700,420 SIM_SCAN_UP=$up "$@" "$BIN" wx-shade-gen > "$f.tmp" 2>&1
mv "$f.tmp" "$f"
