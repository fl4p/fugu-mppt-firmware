#!/usr/bin/env bash
# Serve a firmware image from a relay host on the device LAN, for when this host cannot reach it.
#   etc/ota-via-relay.sh [bin]        stage + serve (default build/fugu-firmware.bin)
#   etc/ota-via-relay.sh stop         stop the server early
# Env (or etc/nat.env): OTA_RELAY_SSH  ssh destination of the relay (default $NAT_TUNNEL_HOST, else havan-ts)
#                       OTA_RELAY_IP   relay address as the device sees it (required)
#                       OTA_RELAY_PORT http port (9000), OTA_RELAY_TTL server lifetime in s (600)
# Prints the `ota <url>` to send through the console; the server exits after the TTL.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
env_file=$here/nat.env
if [ ! -f "$env_file" ]; then
    common=$(git -C "$here" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || true)
    [ -n "$common" ] && env_file=$(dirname "$common")/etc/nat.env
fi
from_env() { [ -f "$env_file" ] && sed -n "s/^$1=//p" "$env_file" | tail -n1; }
dest=${OTA_RELAY_SSH:-$(from_env OTA_RELAY_SSH)}
dest=${dest:-${NAT_TUNNEL_HOST:-$(from_env NAT_TUNNEL_HOST)}}
dest=${dest:-havan-ts}
ip=${OTA_RELAY_IP:-$(from_env OTA_RELAY_IP)}
port=${OTA_RELAY_PORT:-9000}
ttl=${OTA_RELAY_TTL:-600}
rdir=/tmp/fugu-ota-relay

if [ "${1:-}" = stop ]; then
    ssh "$dest" "pkill -f '[h]ttp.server $port --directory $rdir' ; rm -rf $rdir"
    exit 0
fi
[ -n "$ip" ] || { echo "OTA_RELAY_IP not set (env or $env_file)" >&2; exit 2; }
bin=${1:-build/fugu-firmware.bin}
[ -f "$bin" ] || { echo "no image: $bin" >&2; exit 2; }
sha=$(shasum -a 256 "$bin" | cut -d' ' -f1)
name=$(basename "$bin")

ssh "$dest" "mkdir -p $rdir && pkill -f '[h]ttp.server $port --directory $rdir' ; true"
scp -q "$bin" "$dest:$rdir/$name"
rsha=$(ssh "$dest" "sha256sum $rdir/$name" | cut -d' ' -f1)
[ "$rsha" = "$sha" ] || { echo "staged image sha mismatch: $rsha" >&2; exit 1; }
ssh "$dest" "nohup timeout $ttl python3 -m http.server $port --directory $rdir --bind $ip >/dev/null 2>&1 &"
sleep 1
ssh "$dest" "curl -fsS -o /dev/null http://$ip:$port/$name" || { echo "relay server not answering" >&2; exit 1; }

echo "sha256 $sha"
echo "serving for ${ttl}s on $dest; send through the console:"
echo "ota http://$ip:$port/$name"
