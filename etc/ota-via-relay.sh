#!/usr/bin/env bash
# Serve a firmware image from a relay host on the device LAN, for when this host cannot reach it.
#   etc/ota-via-relay.sh [bin]            stage + serve (default build/fugu-firmware.bin)
#   etc/ota-via-relay.sh status <token>   is that server still up (exit 0), plus its access log
#   etc/ota-via-relay.sh stop <token>     stop that server and delete its staging dir
# Each run stages into its own dir on the relay (the token) as <name>-<sha8>.bin and serves it on
# the first free port of 9000-9099 ($OTA_RELAY_PORT), so parallel runs never touch each other.
# Env (or etc/nat.env): OTA_RELAY_SSH  ssh destination of the relay (default $NAT_TUNNEL_HOST, else havan-ts)
#                       OTA_RELAY_IP   relay address as the device sees it (required for staging)
#                       OTA_RELAY_PORT http port or lo-hi range (9000-9099), OTA_RELAY_TTL server lifetime in s (600)
# Output: `sha256 <hex>`, `token <dir>`, `port <n>` and the `ota <url>` line to send through the console.
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
port=${OTA_RELAY_PORT:-$(from_env OTA_RELAY_PORT)}
ttl=${OTA_RELAY_TTL:-600}
root=/tmp/fugu-ota-relay

token_ok() { [[ "${1:-}" =~ ^$root/[0-9a-f]{8}-[A-Za-z0-9]{6}$ ]] || { echo "bad token: ${1:-<none>}" >&2; exit 2; }; }
case "${1:-}" in
stop)
    token_ok "${2:-}"
    ssh "$dest" "d=$2; [ -f \$d/pid ] && kill \$(cat \$d/pid) 2>/dev/null; sleep 0.3; rm -rf \$d"
    exit 0 ;;
status)
    token_ok "${2:-}"
    exec ssh "$dest" "d=$2; tail -n 5 \$d/log 2>/dev/null; if [ -f \$d/pid ] && kill -0 \$(cat \$d/pid) 2>/dev/null; then echo \"up port \$(cat \$d/port)\"; else echo down; exit 1; fi" ;;
esac
[ -n "$ip" ] || { echo "OTA_RELAY_IP not set (env or $env_file)" >&2; exit 2; }
bin=${1:-build/fugu-firmware.bin}
[ -f "$bin" ] || { echo "no image: $bin" >&2; exit 2; }
sha=$(shasum -a 256 "$bin" | cut -d' ' -f1)
base=$(basename "$bin" .bin)
name=$base-${sha:0:8}.bin
port=${port:-9000-9099}
lo=${port%-*}
hi=${port#*-}

srv=$(mktemp -t fugu-ota-serve)
trap 'rm -f "$srv"' EXIT
cat >"$srv" <<'EOF'
import functools, http.server, os, shutil, signal, sys, time
d, ip, lo, hi, ttl = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), float(sys.argv[5])
class H(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *a):
        with open(os.path.join(d, 'log'), 'a') as f:
            f.write('%s %s %s\n' % (time.strftime('%H:%M:%S'), self.address_string(), fmt % a))
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
try:
    for p in range(lo, hi + 1):
        try:
            s = http.server.ThreadingHTTPServer((ip, p), functools.partial(H, directory=os.path.join(d, 'www')))
            break
        except OSError:
            pass
    else:
        sys.exit('no free port in %d..%d' % (lo, hi))
    with open(os.path.join(d, 'port'), 'w') as f:
        f.write(str(p))
    s.timeout = 1
    end = time.time() + ttl
    while time.time() < end:
        s.handle_request()
finally:
    shutil.rmtree(d, ignore_errors=True)
EOF

# new dir; drop dirs of servers that died without cleaning up
dir=$(ssh "$dest" "mkdir -p $root && for o in $root/*/pid; do [ -f \$o ] && ! kill -0 \$(cat \$o) 2>/dev/null && rm -rf \$(dirname \$o); done; d=\$(mktemp -d $root/${sha:0:8}-XXXXXX) && mkdir \$d/www && echo \$d")
token_ok "$dir"
scp -q "$srv" "$dest:$dir/serve.py"
scp -q "$bin" "$dest:$dir/www/$name"
if ! out=$(ssh "$dest" "d=$dir; [ \"\$(sha256sum \$d/www/$name | cut -d' ' -f1)\" = $sha ] || { echo 'staged image sha mismatch'; rm -rf \$d; exit 1; }
nohup python3 \$d/serve.py \$d $ip $lo $hi $ttl >\$d/err 2>&1 </dev/null & echo \$! >\$d/pid
for i in \$(seq 40); do
  [ -s \$d/port ] && curl -fsS -m 3 -o /dev/null http://$ip:\$(cat \$d/port)/$name 2>/dev/null && { cat \$d/port; exit 0; }
  kill -0 \$(cat \$d/pid) 2>/dev/null || break; sleep 0.25
done
echo \"relay server not answering: \$(cat \$d/err 2>/dev/null)\"; kill \$(cat \$d/pid) 2>/dev/null; rm -rf \$d; exit 1"); then
    echo "$out" >&2
    exit 1
fi
port=$out

echo "sha256 $sha"
echo "token $dir"
echo "port $port"
echo "serving for ${ttl}s on $dest; stop with: $0 stop $dir; send through the console:"
echo "ota http://$ip:$port/$name"
