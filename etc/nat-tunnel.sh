#!/usr/bin/env bash
# Forward the NAT_TELNET endpoints (etc/nat.env) through an ssh jump host: etc/nat-tunnel.sh [up|down|status]
# Local port = 12000 + remote port. Jump host: $NAT_TUNNEL_HOST (env or nat.env), default ssh alias havan-ts.
# Ports are not bound to a device: confirm with `hostname` after connecting.
set -u
here=$(cd "$(dirname "$0")" && pwd)
env_file=$here/nat.env
if [ ! -f "$env_file" ]; then
    common=$(git -C "$here" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)
    [ -n "$common" ] && env_file=$(dirname "$common")/etc/nat.env
fi
nat_telnet=${NAT_TELNET:-}
jump=${NAT_TUNNEL_HOST:-}
if [ -f "$env_file" ]; then
    [ -z "$nat_telnet" ] && nat_telnet=$(sed -n 's/^NAT_TELNET=//p' "$env_file" | tail -n1)
    [ -z "$jump" ] && jump=$(sed -n 's/^NAT_TUNNEL_HOST=//p' "$env_file" | tail -n1)
fi
jump=${jump:-havan-ts}
[ -n "$nat_telnet" ] || { echo "NAT_TELNET not set (env or $env_file)" >&2; exit 2; }

sock=${TMPDIR:-/tmp}/fugu-nat-tunnel-$(id -u).sock
fwd=()
map=()
for hp in ${nat_telnet//,/ }; do
    lp=$((12000 + ${hp##*:}))
    fwd+=(-L "127.0.0.1:$lp:$hp")
    map+=("127.0.0.1:$lp -> $hp")
done

up() { ssh -S "$sock" -O check "$jump" 2>/dev/null; }
show() { printf '  %s\n' "${map[@]}"; echo "connect: etc/fugu_console.py --ip 127.0.0.1:<port>, then check \`hostname\`"; }

case ${1:-up} in
up)
    if up; then
        echo "tunnel via $jump already up"
    else
        ssh -f -N -M -S "$sock" -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 "${fwd[@]}" "$jump" || exit 1
        echo "tunnel via $jump up"
    fi
    show ;;
down)
    if up; then ssh -S "$sock" -O exit "$jump" 2>/dev/null; echo "tunnel down"; else echo "no tunnel"; fi ;;
status)
    if up; then echo "tunnel via $jump up"; show; else echo "no tunnel"; exit 1; fi ;;
*)
    echo "usage: $0 [up|down|status]" >&2; exit 2 ;;
esac
