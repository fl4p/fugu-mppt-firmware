#!/bin/bash
# Headless low-light sensor-noise capture from a NAT-routed converter, for noise_profile.py.
#   test/host-stub/noise_capture.sh <telnet-fwd-port> <seconds> [outroot]
# <telnet-fwd-port>: the router's forwarded telnet port (232..235); the scope sits at +10 (242..245).
# Forwards both through havan (ssh alias havan-ts) to 127.0.0.1:1<port>, confirms the hostname from
# the telnet banner, records all channels with adcscope, then saves the InfluxDB telemetry of the
# capture window (P, Ui, Uo, I) as telemetry.json next to it, for noise_profile.py's scaling.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TP=${1:?telnet forwarded port, e.g. 232}
SECS=${2:-60}
OUTROOT=${3:-/tmp/noise-cap}
SP=$((TP + 10))
LT=1$TP; LS=1$SP
ROUTER=${NAT_ROUTER:-192.168.1.231}
JUMP=${NAT_TUNNEL_HOST:-havan-ts}

fwd() { # local port, remote port: ensure an ssh -L forward exists
    if ! lsof -nP -iTCP:"$1" -sTCP:LISTEN >/dev/null 2>&1; then
        ssh -f -N -o ConnectTimeout=10 -o BatchMode=yes -L "$1:$ROUTER:$2" "$JUMP" || { echo "tunnel $1->$2 failed"; exit 2; }
    fi
}
fwd "$LT" "$TP"; fwd "$LS" "$SP"

HOST=$(cd "$ROOT" && PYTHONPATH=etc timeout 15 .venv/bin/python3 -c "
import asyncio
from fugu_console import probe_welcome
print(asyncio.run(probe_welcome('127.0.0.1', $LT, 6.0)) or '')" 2>/dev/null | tail -1)
[ -n "$HOST" ] || { echo "no telnet banner on 127.0.0.1:$LT (device off or port moved)"; exit 3; }
echo "device on port $TP: $HOST"

T0=$(date -u +%Y-%m-%dT%H:%M:%SZ)
OUT="$OUTROOT/$HOST/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$OUT"
(cd "$ROOT" && .venv/bin/python3 etc/scope.py --ip 127.0.0.1 --port "$LS" --capture "$SECS" --out "$OUT") >"$OUT/adcscope.log" 2>&1
RC=$?
T1=$(date -u +%Y-%m-%dT%H:%M:%SZ)
tail -5 "$OUT/adcscope.log"
[ $RC -eq 0 ] || { echo "adcscope capture failed rc=$RC"; exit 4; }

set -a; . "$ROOT/.claude/memory/secrets.env"; set +a
curl -sG --max-time 20 -u "$INFLUX_HA_VAN_USER:$INFLUX_HA_VAN_PASS" "${INFLUX_URL:-http://tm.fabi.me:8086}/query" \
    --data-urlencode db=ha_van \
    --data-urlencode "q=SELECT P, Ui, Uo, I FROM mppt WHERE device='$HOST' AND time >= '$T0' AND time <= '$T1'" \
    >"$OUT/telemetry.json"
.venv/bin/python3 -I - "$OUT/telemetry.json" <<'EOF'
import json, sys, statistics as st
d = json.load(open(sys.argv[1]))
try:
    s = d["results"][0]["series"][0]; col = {c: i for i, c in enumerate(s["columns"])}
    for c in ("P", "Ui", "Uo", "I"):
        v = [r[col[c]] for r in s["values"] if r[col[c]] is not None]
        print(f"  {c}: n={len(v)} mean={st.mean(v):.3f} min={min(v):.3f} max={max(v):.3f}")
except (KeyError, IndexError):
    print("  no telemetry points in the window", d)
EOF
echo "capture: $OUT"
