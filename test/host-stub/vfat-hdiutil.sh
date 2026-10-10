#!/bin/bash
# Round trip of the emulated config drive through the macOS FAT driver:
# build image -> fsck -> mount -> edit/create/delete/atomic-save + junk -> unmount -> fsck -> diff.
# Run from the repo root: test/host-stub/vfat-hdiutil.sh [confdir]
set -euo pipefail

CONF=${1:-config/fmetal/conf}
W=$(mktemp -d /tmp/vfat-rt.XXXXXX)
BIN=$W/vfat-test
clang++ -std=gnu++20 -I src -o "$BIN" test/host-stub/vfat-test.cpp src/usb/vfat.cpp
"$BIN"
"$BIN" gen "$CONF" "$W/img.bin"
fsck_msdos -n "$W/img.bin" >/dev/null

MP=$(hdiutil attach -imagekey diskimage-class=CRawDiskImage -nobrowse -mountrandom "$W" "$W/img.bin" | awk -F'\t' 'END{print $NF}')
mkdir "$W/expect"
for f in "$CONF"/*.conf; do cmp -s "$f" "$MP/$(basename "$f")" || { echo "FAIL: $f differs on mount"; exit 1; }; done

printf '# edited\n' >>"$MP/charger.conf"
cp "$MP/tele.conf" "$MP/.tele.conf.sb-tmp"; echo "x=1" >>"$MP/.tele.conf.sb-tmp"; mv "$MP/.tele.conf.sb-tmp" "$MP/tele.conf"
echo "new=1" >"$MP/new.conf"
rm "$MP/scope.conf"
xattr -w com.example.test hello "$MP/board.conf"
echo junk >"$MP/notes.txt"
mkdir "$MP/sub"; echo y >"$MP/sub/x.conf"
cp "$MP/charger.conf" "$MP/tele.conf" "$MP/new.conf" "$W/expect/"
hdiutil detach "$MP" >/dev/null

fsck_msdos -n "$W/img.bin" >/dev/null
mkdir "$W/got"
"$BIN" apply "$CONF" "$W/img.bin" "$W/got" | tee "$W/apply.txt"
grep -q '^removed scope.conf$' "$W/apply.txt"
[ "$(grep -c '^upsert' "$W/apply.txt")" = 3 ] || { echo "FAIL: expected 3 upserts"; exit 1; }
diff -r "$W/expect" "$W/got"
rm -rf "$W"
echo "vfat-hdiutil: passed"
