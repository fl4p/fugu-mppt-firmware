#!/bin/bash
# job: bin drv sc board rd mode k x   (rd = vout_gain:vin_gain, x = extra tag list joined by +, or -)
set -- $1
bin=$1 drv=$2 sc=$3 board=$4 rd=$5 mode=$6 k=$7 x=$8
key="$bin.$drv.$sc.$board.$rd.$mode.$k.$x"
f=/tmp/sim4103-out/$key.txt
nb=$board; [ "$nb" = - ] && nb=${sc##*-}; nb=${nb%-hq}; nb=${nb%-off}
env=()
[ "$board" != - ] && env+=(SIM_RING=$board)
case $drv in ledc) env+=(SIM_DRV=ledc);; mcpwm0) env+=(SIM_PLANT=$nb:rect_offset_ns=0);; esac
conf=""
g=${rd%%:*} vi=${rd##*:}
[ "$g" != 1 ] && conf="$conf,sensor.conf:lv_v_gain=$g"
[ "$vi" != 1 ] && conf="$conf,sensor.conf:hv_v_gain=$vi"
case $mode in
  gauss) conf="$conf,vconv.conf:noise_vout=0.03,vconv.conf:noise_iout=0.02"; env+=(SIM_SEED=$k);;
  meas) env+=(SIM_NOISE=$nb SIM_NOISE_OFS=$k);;
  rep) env+=(SIM_NOISE=$nb SIM_NOISE_OFS=$((k * 37)) SIM_SEED=$k);;
esac
for t in ${x//+/ }; do case $t in
  v28) env+=(SIM_VBAT=28);;
  scan6) conf="$conf,tracker.conf:pv_substrings=6";;
  slowf) conf="$conf,sensor.conf:lv_i_filt_len=60,sensor.conf:lv_v_filt_len=60"; env+=(SIM_ADC_HZ=200);;
  verb) env+=(SIM_V=1);;
esac; done
[ -n "$conf" ] && env+=("SIM_CONF=${conf#,}")
if [ ! -s "$f" ] || ! grep -q '^== SUMMARY' "$f"; then
  env "${env[@]}" ~/bin/offload run --host pgx93 --tag sim4103 -- /tmp/sim4103-bin-$bin $sc > "$f" 2>&1
fi
awk -v key="$key" '
  function num(re) { return match($0, re) ? substr($0, RSTART, RLENGTH) : "" }
  /^== SUMMARY/ { h = num("harvest +[0-9.]+"); sub(/.* /, "", h); tr = num("trips [0-9]+ \\[[^]]*\\]"); sub(/^trips /, "", tr); gsub(/ /, "", tr)
                  off = num("off +[0-9.]+ s"); sub(/^off +/, "", off); sub(/ s/, "", off); sw = num("sweeps +[0-9]+"); sub(/.* /, "", sw) }
  /^== REVERSE/ { im = num("mean min -?[0-9.]+"); sub(/.* /, "", im); ta = num("t_min/T\\) [0-9.]+"); sub(/.* /, "", ta)
                  st = num("stops [0-9]+"); sub(/.* /, "", st); ns = num("starts [0-9]+"); sub(/.* /, "", ns) }
  /^== TRACKER/ { tl = num("tracking loss [0-9.]+ %"); sub(/tracking loss /, "", tl); sub(/ %/, "", tl)
                  hh = num("harvest [0-9.]+ %"); sub(/harvest /, "", hh); sub(/ %/, "", hh) }
  /^== SCAN [^l]/ { sc = num("[0-9]+ scans"); sub(/ scans/, "", sc) }
  /^== VERDICT/ { vd = ($0 ~ /: PASS/) ? "PASS" : "FAIL" }
  END { printf "%s h=%s h2=%s trips=%s off=%s sweeps=%s onset=%s stops=%s starts=%s imin=%s tl=%s scans=%s verdict=%s\n", key,
        h, hh, tr, off, sw, ta, st, ns, im, tl, sc, vd }' "$f"
