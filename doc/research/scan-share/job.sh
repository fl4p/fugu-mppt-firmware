#!/bin/bash
# One closed-loop run of the scan-share study: job.sh "cell variant board ofs" -> one result line.
# BIN: sim binary built by `offload build-sim` (default /tmp/scanshare-bin), HOST: offload host (one per comparison),
# TAG: offload tag, OUTDIR: raw outputs, cached per binary sha256 and host (a run with a PROBES line is not repeated).
# Variants: base (probes above the operating point at 0.97, the rule before 11-24), D6 (1.03), uNNNN (SIM_SCAN_UP
# NNNN / 1000, e.g. u1015, the firmware default since round 2), tNN (SIM_SCAN_HEAT NN / 100, t0: off; base also off). A, C75, D2, D4 used share / HI knobs dropped when D6 landed
# (doc/research/2026-11-24-scan-share-estimate.md); they now exit with an error.
set -- $1
cell=$1 var=$2 board=$3 ofs=$4
BIN=${BIN:-/tmp/scanshare-bin} HOST=${HOST:-vup.fabi.me} OUTDIR=${OUTDIR:-/tmp/scanshare-out} TAG=${TAG:-scanshare}
sha=$(shasum -a 256 "$BIN" | cut -c1-12)
key="$cell.$var.$board.$ofs"
f=$OUTDIR/$key.$sha.$HOST.txt
env=(SIM_CONF=tracker.conf:pv_substrings=6 SIM_RING=$board SIM_NOISE=$board SIM_NOISE_OFS=$ofs)
sc=wx-shade-gen
case $cell in
  a1) env+=(SIM_SHADE=1,0.70,a);;
  a2) env+=(SIM_SHADE=1,0.62,a);;
  a3) env+=(SIM_SHADE=2,0.55,a);;
  a4) env+=(SIM_SHADE=2,0.40,a);;
  a5) env+=(SIM_SHADE=3,0.40,a);;
  a6) env+=(SIM_SHADE=2,0.45,a,1);;
  l1) env+=(SIM_SHADE=1,0.80,r,0,0.60);;
  l2) env+=(SIM_SHADE=2,0.65,r,0,0.45);;
  l3) env+=(SIM_SHADE=2,0.72,r,0,0.45);;
  l4) env+=(SIM_SHADE=1,0.55,r,1,0.35);;
  l5) env+=(SIM_SHADE=2,0.45,r,1,0.30);;
  l6) env+=(SIM_SHADE=2,0.45,l);;
  l1d) env+=(SIM_SHADE=1,0.80,r,0,0.60 SIM_VOCDRIFT=-0.06);;
  a3d) env+=(SIM_SHADE=2,0.55,a SIM_VOCDRIFT=-0.06);;
  l2k) env+=(SIM_SHADE=2,0.65,r,0,0.45 SIM_K=0.80);;
  a3k) env+=(SIM_SHADE=2,0.55,a SIM_K=0.80);;
  # heating: unshaded cold boot sweep, shade steps to an intermediate state at 120 s (the scan moves to a lower
  # maximum), Voc falls 8 / 12 % over 420..700 s, the shade steps to the static 1.03 regression cases at 720 s
  h1) env+=(SIM_SHADE=1,0.65,u,1,0.2,600 SIM_VOCDRIFT=-0.08,700,420);;
  h2) env+=(SIM_SHADE=1,0.75,v,1,0.3,600 SIM_VOCDRIFT=-0.08,700,420);;
  h3) env+=(SIM_SHADE=1,0.65,u,1,0.2,600 SIM_VOCDRIFT=-0.12,700,420);;
  h4) env+=(SIM_SHADE=1,0.75,v,1,0.3,600 SIM_VOCDRIFT=-0.12,700,420);;
  *) sc=$cell;;
esac
case $var in
  base) env+=(SIM_SCAN_UP=0.97 SIM_SCAN_HEAT=0);;
  B94) env+=(SIM_SCAN_UP=0.94 SIM_SCAN_LOW=0.94);;
  D6) env+=(SIM_SCAN_UP=1.03);;
  u[0-9][0-9][0-9][0-9]) env+=(SIM_SCAN_UP=$(awk -v x="${var#u}" 'BEGIN { printf "%.3f", x / 1000 }'));;
  t[0-9]*) env+=(SIM_SCAN_HEAT=$(awk -v x="${var#t}" 'BEGIN { printf "%.2f", x / 100 }'));;
  *) echo "variant $var: not in this sim (scan-static only)" >&2; exit 1;;
esac
mkdir -p "$OUTDIR"
if [ ! -s "$f" ] || ! grep -q '^== PROBES' "$f"; then
  if [ -n "$DIRECT" ]; then env "${env[@]}" "$BIN" $sc > "$f" 2>&1 # on the host, inside `offload hold`
  else env "${env[@]}" ~/bin/offload run --host "$HOST" --tag "$TAG" -- "$BIN" $sc > "$f" 2>&1; fi
fi
awk -v key="$key" '
  function num(re) { return match($0, re) ? substr($0, RSTART, RLENGTH) : "" }
  /^== SUMMARY/ { h = num("harvest +[0-9.]+"); sub(/.* /, "", h); w = num("wrong-peak +[0-9.]+"); sub(/.* /, "", w); sw = num("sweeps +[0-9]+"); sub(/.* /, "", sw) }
  /^== TRACKER/ { hh = num("harvest [0-9.]+ %"); sub(/harvest /, "", hh); sub(/ %/, "", hh) }
  /^== SCAN [^l]/ { sc = num("[0-9]+ scans"); sub(/ scans/, "", sc); mv = num("[0-9]+ moved"); sub(/ moved/, "", mv); ls = num("scanning [0-9.]+ s"); sub(/scanning /, "", ls); sub(/ s/, "", ls) }
  /^== PROBES/ { split($0, a, "|"); pr = a[1]; sub(/.*wx-[a-z0-9-]+ +/, "", pr); gsub(/ /, "", pr); gsub(/-/, "", pr); dv = num("mean [0-9.]+ %"); sub(/mean /, "", dv); sub(/ %/, "", dv); rg = num("true gain [0-9.]+"); sub(/true gain /, "", rg) }
  END { printf "%s h=%s h2=%s wrong=%s sweeps=%s scans=%s moved=%s scantime=%s probes=%s dv=%s readgain=%s\n", key, h, hh, w, sw, sc, mv, ls, pr, dv, rg }' "$f"
