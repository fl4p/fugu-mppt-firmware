#!/bin/bash
# variant-0 identity: run the unmodified build and the study build, compare all output except "== ADAPT"
#   ident.sh <dir with mppt-sim-base, mppt-sim-study> <scenario>:<noise|none> ...
D=$1; shift
mkdir -p "$D/ident"
one() {
    local sc=${1%%:*} nz=${1#*:} b
    for b in base study; do
        if [ "$nz" = none ]; then env -u SIM_NOISE "$D/mppt-sim-$b" "$sc"
        else SIM_NOISE=$nz SIM_NOISE_OFS=6 "$D/mppt-sim-$b" "$sc"; fi 2>&1 |
            grep -v '^== ADAPT' | sed 's#/tmp/mppt-sim-[A-Za-z0-9]*##g' > "$D/ident/$sc.$nz.$b"
    done
    cmp -s "$D/ident/$sc.$nz.base" "$D/ident/$sc.$nz.study" && echo "SAME $1" || echo "DIFF $1"
}
export -f one; export D
printf '%s\n' "$@" | xargs -P 4 -I{} bash -c 'one {}'
