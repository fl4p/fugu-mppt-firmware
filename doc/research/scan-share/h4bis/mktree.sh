#!/bin/bash
# mktree.sh NAME: finishes /tmp/h4bis/t-NAME (already extracted, sim overlaid): e7a1376d probe rule where missing,
# fix-ls-hold.patch (unless NOHOLD=1), local build to /tmp/h4bis/bin-NAME.
set -e
name=$1
T=/tmp/h4bis/t-$name
cd $T
grep -q probeHigh src/peak_scan.h || patch -s -p1 < /tmp/h4bis/probe-rule.patch
[ -n "$NOHOLD" ] || grep -q 'scan.hold =' src/mppt.cpp || patch -s -p1 < /tmp/h4bis/fix-ls-hold.patch
OUT=/tmp/h4bis/bin-$name $T/test/host-stub/mppt-sim.sh none > /tmp/h4bis/build-$name.log 2>&1 || true
[ -x /tmp/h4bis/bin-$name ] || { echo "build failed $name"; tail -20 /tmp/h4bis/build-$name.log; exit 1; }
echo "$name $(shasum -a 256 /tmp/h4bis/bin-$name | cut -c1-12)"
