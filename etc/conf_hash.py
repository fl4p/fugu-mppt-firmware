#!/usr/bin/env python3
"""Reference implementation of the firmware's `conf-hash` fingerprint.

Recomputes the `conf` hash and the per-file hashes from a directory, with the same definition and
the same wifi.conf exclusion as src/conf_hash.cpp, and prints them in the console's line format:

    conf sha256=<64hex> files=<N> bytes=<M> excluded=wifi.conf
    conf-file <relpath> sha256=<16hex> bytes=<n>

(the device line also carries `boot=`, which only the device knows.)

DIR may be the conf directory itself, or a directory that contains `conf/` (a config/ profile such
as config/fmetal, or a littlefs dump from etc/dump_littlefs.py); in the latter case `conf/` is used.

Hash definition (byte-exact, not parsed keys):
  files  = every regular file below the conf dir, recursively, as a '/'-separated relative path,
           except the top-level "wifi.conf" (it holds the Wi-Fi PSK)
  order  = relative path, bytewise ascending (UTF-8 bytes)
  sha256 = SHA-256 over, per file in order:  relpath b"\\0" decimal(len) b"\\0" raw-bytes
The per-file hash is SHA-256 of the raw file bytes (what `sha256sum` prints); the line shows the
first 16 hex digits.

The NVS hash (`nvs sha256=...`) covers NVS namespace "storage": entries sorted by key, each
contributing  key b"\\0" typename b"\\0" decimal(len) b"\\0" value-bytes  (typename u8 i8 u16 i16
u32 i32 u64 i64 str blob; integers little-endian at native width, strings without the NUL).
nvs_hash() implements it for a host-side list of entries.

Usage:
    etc/conf_hash.py config/fmetal
    etc/conf_hash.py littlefs-dump --json
"""
import argparse
import hashlib
import json
import os
import stat
import struct
import sys
from pathlib import Path

EXCLUDED = "wifi.conf"


class ConfHashError(Exception):
    pass


def resolve_conf_dir(d: Path) -> Path:
    d = Path(d)
    if not d.is_dir():
        raise ConfHashError(f"not a directory: {d}")
    sub = d / "conf"
    return sub if sub.is_dir() else d


def list_files(conf_dir: Path):
    """Relative '/' paths of the hashed files, in hash order."""
    out = []

    def walk(rel: str):
        full = conf_dir / rel if rel else conf_dir
        for name in os.listdir(full):
            r = f"{rel}/{name}" if rel else name
            st = os.stat(conf_dir / r)  # follows symlinks, as a host copy may contain them
            if stat.S_ISDIR(st.st_mode):
                walk(r)
            elif stat.S_ISREG(st.st_mode):
                if r != EXCLUDED:
                    out.append(r)
            else:
                raise ConfHashError(f"not a regular file: {conf_dir / r}")

    walk("")
    out.sort(key=lambda p: p.encode("utf-8"))
    return out


def conf_hash(conf_dir: Path):
    """Return (sha256_hex, [(relpath, file_sha256_hex, nbytes), ...], total_bytes)."""
    conf_dir = Path(conf_dir)
    h = hashlib.sha256()
    files = []
    total = 0
    for rel in list_files(conf_dir):
        data = (conf_dir / rel).read_bytes()
        h.update(rel.encode("utf-8") + b"\0" + str(len(data)).encode("ascii") + b"\0")
        h.update(data)
        files.append((rel, hashlib.sha256(data).hexdigest(), len(data)))
        total += len(data)
    return h.hexdigest(), files, total


_INT_FMT = {"u8": "<B", "i8": "<b", "u16": "<H", "i16": "<h",
            "u32": "<I", "i32": "<i", "u64": "<Q", "i64": "<q"}


def nvs_value_bytes(typename: str, value) -> bytes:
    if typename in _INT_FMT:
        return struct.pack(_INT_FMT[typename], value)
    if typename == "str":
        return value.encode("utf-8") if isinstance(value, str) else bytes(value)
    if typename == "blob":
        return bytes(value)
    raise ConfHashError(f"unknown NVS type {typename!r}")


def nvs_hash(entries):
    """entries: iterable of (key, typename, value). Returns the `nvs sha256` hex digest."""
    h = hashlib.sha256()
    for key, typename, value in sorted(entries, key=lambda e: e[0].encode("utf-8")):
        v = nvs_value_bytes(typename, value)
        h.update(key.encode("utf-8") + b"\0" + typename.encode("ascii") + b"\0"
                 + str(len(v)).encode("ascii") + b"\0" + v)
    return h.hexdigest()


def format_lines(sha, files, total):
    lines = [f"conf sha256={sha} files={len(files)} bytes={total} excluded={EXCLUDED}"]
    lines += [f"conf-file {rel} sha256={fsha[:16]} bytes={n}" for rel, fsha, n in files]
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", help="conf dir, or a dir containing conf/ (profile or littlefs dump)")
    ap.add_argument("--json", action="store_true", help="print JSON instead of console lines")
    args = ap.parse_args(argv)
    try:
        conf_dir = resolve_conf_dir(Path(args.dir))
        sha, files, total = conf_hash(conf_dir)
    except (ConfHashError, OSError) as e:
        print(f"conf sha256=unknown excluded={EXCLUDED} error={e}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps({"conf_dir": str(conf_dir), "sha256": sha, "files": len(files), "bytes": total,
                          "excluded": EXCLUDED,
                          "per_file": [{"path": r, "sha256": s, "bytes": n} for r, s, n in files]},
                         indent=2))
    else:
        print("\n".join(format_lines(sha, files, total)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
