#!/bin/bash
# Wait for a converter's low-light window (P > Pmin, Imin <= I <= Imax from InfluxDB), then take its lock and run
# noise_capture.sh once. Detachable: it names itself as the lock's owner process.
#   test/host-stub/noise_capture_wait.sh <device> <telnet-fwd-port> <deadline HH:MM local> [secs] [Imin] [Imax] [Pmin]
# Polls every 3 min until the deadline; exits 0 after one capture, 5 if the window never came.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
DEV=${1:?device, fry or flat}; TP=${2:?telnet fwd port}; DEADLINE=${3:?HH:MM}
SECS=${4:-60}; IMIN=${5:-0.05}; IMAX=${6:-0.5}; PMIN=${7:-1}
L=~/.claude/skills/lock/lock.sh
export AGENT_LOCK_OWNER_PID=$$
set -a; . "$ROOT/.claude/memory/secrets.env"; set +a
DL=$(date -j -f "%Y-%m-%d %H:%M" "$(date +%Y-%m-%d) $DEADLINE" +%s)

while [ "$(date +%s)" -lt "$DL" ]; do
    read -r P I < <(curl -sG --max-time 15 -u "$INFLUX_HA_VAN_USER:$INFLUX_HA_VAN_PASS" \
        "${INFLUX_URL:-http://tm.fabi.me:8086}/query" --data-urlencode db=ha_van \
        --data-urlencode "q=SELECT mean(P), mean(I) FROM mppt WHERE device='$DEV' AND time > now() - 2m" |
        python3 -I -c 'import json,sys
try:
    v = json.load(sys.stdin)["results"][0]["series"][0]["values"][0]; print(v[1], v[2])
except Exception: print("nan nan")')
    echo "$(date +%H:%M:%S) $DEV P=$P I=$I"
    if python3 -I -c 'import sys, math
try:
    p, i, pm, lo, hi = map(float, sys.argv[1:6])
except ValueError:
    sys.exit(1)
sys.exit(0 if (math.isfinite(p) and math.isfinite(i) and p > pm and lo <= i <= hi) else 1)' "$P" "$I" "$PMIN" "$IMIN" "$IMAX"; then
        "$L" run "$DEV" "low-light noise capture, ${SECS}s scope stream, read-only" --timeout 300 -- \
            "$ROOT/test/host-stub/noise_capture.sh" "$TP" "$SECS"
        rc=$?
        echo "capture rc=$rc"
        [ $rc -eq 0 ] && exit 0
        [ $rc -eq 3 ] && exit 3   # wrong/no banner: port moved, do not keep hammering
    fi
    sleep 180
done
echo "window never came before $DEADLINE"
exit 5
