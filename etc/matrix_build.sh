#!/usr/bin/env bash
# Build the WITH_* x target matrix into isolated per-variant project roots so that
# concurrent variants (or other idf.py runs in this tree) don't trample each other's
# managed_components/, dependencies.lock, or sdkconfig. Each variant gets a symlink
# farm under build/matrix-roots/<name>/ pointing at the real sources, plus its own
# managed_components/, dependencies.lock, sdkconfig, and build/.
set -u
cd "$(dirname "$0")/.."

. ./idf-export.sh >/dev/null 2>&1

PROJ_ROOT="$(pwd -P)"
results=()
log_dir="build/matrix-logs"
roots_dir="build/matrix-roots"
mkdir -p "$log_dir" "$roots_dir"

# Top-level entries each variant root needs to see. Anything not listed (build dirs,
# managed_components, dependencies.lock, sdkconfig*) stays per-variant.
ROOT_LINKS=(
    CMakeLists.txt
    main src components config test
    partitions.csv
    sdkconfig.defaults sdkconfig.defaults.esp32
    sdkconfig.ble sdkconfig.no_netw
    idf-export.sh
)

make_root() {
    local name="$1"
    local root="$roots_dir/$name"
    rm -rf "$root"
    mkdir -p "$root"
    for entry in "${ROOT_LINKS[@]}"; do
        [ -e "$PROJ_ROOT/$entry" ] && ln -s "$PROJ_ROOT/$entry" "$root/$entry"
    done
    echo "$root"
}

_yn() { [ "$1" = "1" ] && echo y || echo n; }

run_one() {
    local name="$1" target="$2" with_ble="$3" with_netw="$4"
    local root; root="$(make_root "$name")"
    local bdir="$root/build"
    local log="$log_dir/$name.log"

    echo "=== [$name] target=$target WITH_BLE=$with_ble WITH_NETW=$with_netw ==="

    # Feature flags moved from WITH_* env vars to Kconfig: seed this variant's CONFIG_FUGU_WITH_*
    # via a fragment layered onto sdkconfig.defaults. The top CMakeLists reads the same chain.
    printf 'CONFIG_FUGU_WITH_BLE=%s\nCONFIG_FUGU_WITH_NETW=%s\n' \
        "$(_yn "$with_ble")" "$(_yn "$with_netw")" > "$root/sdkconfig.matrix"

    (
        export IDF_TARGET="$target"
        export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.matrix"
        idf.py -C "$root" -B "$bdir" -DIDF_TARGET="$target" set-target "$target"
        idf.py -C "$root" -B "$bdir" build
    ) >"$log" 2>&1
    local rc=$?

    local size="?"
    if [ -f "$bdir/fugu-firmware.bin" ]; then
        size=$(stat -f %z "$bdir/fugu-firmware.bin" 2>/dev/null || stat -c %s "$bdir/fugu-firmware.bin")
    fi
    results+=("$name|rc=$rc|size=$size|log=$log")
    echo "    rc=$rc size=$size"
}

run_one "s3-baseline"  esp32s3 1 1
run_one "s3-noble"     esp32s3 0 1
run_one "s3-nonetw"    esp32s3 1 0
run_one "s3-headless"  esp32s3 0 0
run_one "esp32-ble"    esp32   1 1
run_one "esp32-noble"  esp32   0 1

echo ""
echo "=== SUMMARY ==="
for r in "${results[@]}"; do echo "$r"; done
