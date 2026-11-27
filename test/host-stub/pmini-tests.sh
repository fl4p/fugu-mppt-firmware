#!/bin/sh
# Native codec, adapter, batching, and MQTT integration. No device access.
set -eu
cd "$(dirname "$0")/../.."
out=$(mktemp -d /tmp/fugu-pmini-tests.XXXXXX)
trap 'rm -rf "$out"' EXIT HUP INT TERM
cxx=${CXX:-clang++}
cc=${CC:-clang}
flags='-O2'
if [ "${SANITIZE:-0}" = 1 ]; then flags='-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer'; fi
inc=components/tamp/tamp/_c_src/tamp
for name in common compressor; do
    "$cc" $flags -I "$inc" -c "$inc/$name.c" -o "$out/$name.o"
done
"$cxx" $flags -std=c++17 -Wall -Wextra -Werror -I . -I src/tele src/tele/pmini_codec.cpp test/host-stub/pmini-codec-test.cpp -o "$out/codec"
"$out/codec"
"$cxx" $flags -std=c++17 -Wall -Wextra -Werror -I . -I src/tele src/tele/pmini_packet.cpp test/host-stub/pmini-packet-test.cpp -o "$out/adapter"
"$out/adapter"
for name in tele-packet-test tele-mqtt-pmini-test; do
    "$cxx" $flags -std=c++20 -fexceptions -DWITH_TELE_BINARY -DWITH_TELE_MQTT -DCONFIG_FUGU_WITH_TELE_BINARY -DCONFIG_FUGU_WITH_TELE_MQTT -DCONFIG_FUGU_WITH_TELE_PMINI \
        -I test/host-stub/mqtt-stub -I test/host-stub -I src -I "$inc" \
        "test/host-stub/$name.cpp" src/tele/tele_packet.cpp src/tele/pmini_packet.cpp src/tele/pmini_codec.cpp src/tele/tamp_compress.cpp \
        "$out/common.o" "$out/compressor.o" -o "$out/$name"
    "$out/$name"
done
