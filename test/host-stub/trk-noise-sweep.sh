#!/bin/bash
# Tracker noise-robustness sweep: scenarios x replayed noise profiles x noise gain, in parallel.
#   test/host-stub/trk-noise-sweep.sh RESULTS.txt [jobs]   (OUT=<sim binary>, default as mppt-sim.sh)
# Build the sim first (test/host-stub/mppt-sim.sh sanity). Compare two result files with trk-sweep-diff.py.
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export OUT=${OUT:-/tmp/mppt-sim.$(printf %s "$ROOT" | cksum | cut -d" " -f1)}
RES=${1:?results file}
J=${2:-4}
SCEN=${SCEN:-"wx-mid wx-dusk wx-low wx-wob-fry wx-cloud"}
NOISE=${NOISE:-"fry flat"}
GAIN=${GAIN:-"0.3 1 3"}
OFS=${OFS:-"0"}   # noise replay offsets [s], SIM_NOISE_OFS
# one file per run, joined at the end: parallel appends to one file interleaved records
TMPD=$(mktemp -d "${TMPDIR:-/tmp}/trk-sweep.XXXXXX")
export TMPD
i=0
for s in $SCEN; do for n in $NOISE; do for g in $GAIN; do for o in $OFS; do i=$((i + 1)); echo "$i $s $n $g $o"; done; done; done; done |
    xargs -P "$J" -L1 bash -c 'o=$(SIM_NOISE=$2 SIM_NOISE_GAIN=$3 SIM_NOISE_OFS=$4 "$OUT" "$1" 2>&1 | grep -E "^== (SUMMARY|TRACKER)"); printf "### %s noise=%s gain=%s ofs=%s\n%s\n" "$1" "$2" "$3" "$4" "$o" > "$TMPD/$(printf %05d "$0")"'
cat "$TMPD"/* > "$RES"
rm -r "$TMPD"
bad=$(awk '/^###/{if (h && n != 1) b++; h = 1; n = 0} /^== SUMMARY/{n++} END{if (h && n != 1) b++; print b + 0}' "$RES")
echo "done: $RES ($(grep -c '^###' "$RES") runs, $bad without exactly one SUMMARY)"
