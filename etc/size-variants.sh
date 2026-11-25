#!/bin/bash
# Build config variants and compare image size, app partition free space and the DROM page margins.
#   etc/size-variants.sh [-j N] [--base 'sdkconfig.defaults;sdkconfig.d/fleet'] VARIANT ...
#   VARIANT = name:ITEM[,ITEM...]    ITEM = CONFIG_X=value | COPTS=<main component options, space separated>
#   etc/size-variants.sh os:CONFIG_COMPILER_OPTIMIZATION_SIZE=y,COPTS=-O2 noftp:CONFIG_FUGU_WITH_FTP=n
# Builds the base first (variant "base"), each into build/sz/<name>, with ninja -j N (default 4).
# The fragment is written to build/sz/<name>/variant.defaults; logs to build/sz/<name>/build.log.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || exit 2
J=4
BASE="sdkconfig.defaults;sdkconfig.d/fleet"
while [ $# -gt 0 ]; do
  case $1 in
    -j) J=$2; shift 2 ;;
    --base) BASE=$2; shift 2 ;;
    -h|--help) sed -n '2,7p' "$0"; exit 0 ;;
    *) break ;;
  esac
done
[ $# -gt 0 ] || { sed -n '2,7p' "$0"; exit 2; }
command -v idf.py >/dev/null || . ./idf-export.sh >/dev/null 2>&1; command -v idf.py >/dev/null || exit 2
PY=python3; [ -x .venv/bin/python3 ] && PY=.venv/bin/python3
LAYOUT=etc/idf-devtools/image_layout.py

build() { # name items
  local name=$1 items=$2 d=build/sz/$1 copts= frag
  mkdir -p "$d"; frag=$d/variant.defaults; : > "$frag"
  local IFS=,
  for it in $items; do
    case $it in
      COPTS=*) copts=$(printf %s "${it#COPTS=}" | tr ' ' ';') ;;
      CONFIG_*=*) printf '%s\n' "$it" >> "$frag" ;;
      '') ;;
      *) echo "size-variants: bad item '$it' in $name" >&2; return 2 ;;
    esac
  done
  unset IFS
  rm -f "$d/sdkconfig"  # an existing sdkconfig would win over the defaults
  { idf.py -B "$d" -DSDKCONFIG="$d/sdkconfig" "-DSDKCONFIG_DEFAULTS=$BASE;$ROOT/$frag" \
      "-DFUGU_MAIN_COPTS=$copts" reconfigure && ninja -C "$d" -j "$J"; } > "$d/build.log" 2>&1
}

row() { # name
  "$PY" "$LAYOUT" "build/sz/$1" --json | "$PY" -c '
import json, sys
r = json.load(sys.stdin); n = r["size"]; base = int(sys.argv[2])
e = next(x for x in r["edges"] if x["kind"] == "IROM")
free = "?" if r["free"] is None else r["free"]
print("%-16s %9d %+8d %8s %#9x %8d %8d" % (sys.argv[1], n, n - base, free, e["hdr"], e["up"], e["down"]))' "$1" "$2"
}

printf '%-16s %9s %8s %8s %9s %8s %8s\n' variant bytes delta free irom drom+ drom-
rc=0
if build base ""; then
  base=$(stat -f %z build/sz/base/fugu-firmware.bin 2>/dev/null || stat -c %s build/sz/base/fugu-firmware.bin)
  row base "$base"
else
  echo "base: build failed, see build/sz/base/build.log"; exit 1
fi
for v in "$@"; do
  name=${v%%:*}; items=${v#*:}; [ "$items" = "$v" ] && items=
  [ "$name" = base ] && { echo "size-variants: 'base' is reserved" >&2; rc=2; continue; }
  if build "$name" "$items"; then row "$name" "$base" || rc=1
  else echo "$name: build failed, see build/sz/$name/build.log"; rc=1; fi
done
exit $rc
