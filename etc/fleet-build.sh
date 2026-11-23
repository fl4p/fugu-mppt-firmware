#!/usr/bin/env bash
# Build the fry/flat production image (sdkconfig.fleet, ESP-IDF 6.0.3) of a commit in a clean, reused
# detached worktree, so no uncommitted edit of the main checkout ships.
#   etc/fleet-build.sh [git-rev]      default HEAD of the main checkout
# Env: FLEET_WT  worktree (default <main checkout>/../fugu-fleet-wt), build dir build-fleet inside it.
# Prints the .bin path, version and sha256; feed it to `etc/ota_mqtt.py --bin <path>`.
set -eu
main=$(git -C "$(dirname "$0")" rev-parse --path-format=absolute --git-common-dir)
main=$(dirname "$main")
wt=${FLEET_WT:-$(dirname "$main")/fugu-fleet-wt}
rev=$(git -C "$main" rev-parse --verify "${1:-HEAD}^{commit}")

[ -e "$wt/.git" ] || git -C "$main" worktree add -q --detach "$wt" "$rev"
git -C "$wt" checkout -q -f --detach "$rev"
git -C "$wt" clean -q -ffd
git -C "$wt" -c protocol.file.allow=always submodule update -q --init --recursive --force
git -C "$wt" submodule -q foreach --recursive 'git clean -q -ffd'
dirt=$(git -C "$wt" status --porcelain --ignore-submodules=none)
if [ -n "$dirt" ]; then
    echo "fleet-build: $wt is dirty after checkout of ${rev:0:8}:" >&2
    echo "$dirt" >&2
    exit 1
fi

# a cached sdkconfig would override a changed sdkconfig.fleet/defaults
rm -f "$wt/build-fleet/sdkconfig" "$wt/build-fleet/sdkconfig.fugu-baseline"
(
    cd "$wt"
    . ./idf-export.sh esp32s3 >/dev/null
    idf.py -B build-fleet -DSDKCONFIG=build-fleet/sdkconfig \
        -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.fleet" build
)

bin=$wt/build-fleet/fugu-firmware.bin
ver=$(python3 - "$bin" <<'EOF'
import struct, sys
d = open(sys.argv[1], 'rb').read(0x200)
o = d.find(struct.pack('<I', 0xABCD5432))
print(d[o + 0x10:o + 0x30].split(b'\0')[0].decode() if o >= 0 else '?')
EOF
)
sha=$(shasum -a 256 "$bin" | cut -d' ' -f1)
echo "fleet-build: commit $rev"
echo "bin     $bin"
echo "version $ver"
echo "sha256  $sha"
case $ver in *dirty*) echo "fleet-build: version is -dirty, the build modified tracked files:" >&2
    git -C "$wt" status --porcelain --ignore-submodules=none >&2; exit 1 ;; esac
