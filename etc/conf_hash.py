#!/usr/bin/env python3
"""Reference implementation of the firmware's `conf-hash` fingerprint.

Recomputes the `conf` hash and the per-file hashes from a directory with the same definition as
src/conf_hash.cpp (definition: src/conf_hash.h, pure parts: src/conf_hash_core.h) and prints them
in the console's line format:

    conf sha256=<64hex> files=<N> bytes=<M> redacted=<R> excluded=wifi.conf
    conf-file <relpath> sha256=<16hex> bytes=<n> redacted=<r>

(the device line also carries `boot=`, which only the device knows). On any error it prints
`conf sha256=unknown excluded=wifi.conf error=<reason>` and exits 1.

DIR is the conf directory. If DIR has no *.conf file at its top level but has a `conf/`
subdirectory (a config/ profile root such as config/fmetal, or a littlefs dump from
etc/dump_littlefs.py), `conf/` is hashed instead. The hashed directory is printed to stderr.

Definition (see src/conf_hash.h for the normative text):
  files   every regular file below the conf dir, recursively (more than 8 nested directory
          levels is an error), as a '/'-separated relative path, except the top-level wifi.conf. A file ending
          in ".new" (USB-drive commit staging) is an error. Symlinks and other non-regular entries
          are errors (they are not followed; littlefs has none).
  order   relative path, bytewise ascending
  content each file split into lines at b"\\n" only (kept), each line redacted by redact_line()
          (secret rule: any '=' whose preceding bytes contain a secret substring or end in _key
          redacts the rest of the line; else URI userinfo up to the last '@' on the line);
          a line over 4096 bytes (excluding b"\\n") is an error
  sha256  SHA-256 over, per file in order:  relpath b"\\0" decimal(len(content)) b"\\0" content
The per-file hash is SHA-256 of the redacted content: it equals `sha256sum` of the file only when
redacted=0. `bytes` are redacted lengths.

The NVS hash (`nvs sha256=...`) covers NVS namespace "storage": entries sorted by key, each
contributing  key b"\\0" typename b"\\0" decimal(len) b"\\0" value  (typename u8 i8 u16 i16 u32 i32
u64 i64 str blob; integers little-endian at native width, strings without the NUL, URI userinfo
redacted in str values); a secret-key entry contributes only  key b"\\0redacted".  nvs_hash()
implements it for a host-side list of entries.
"""
import argparse
import hashlib
import json
import os
import stat
import struct
import sys
from pathlib import Path

EXCLUDED = b"wifi.conf"
STAGING_SUFFIX = b".new"
MARKER = b"<redacted>"
MAX_DEPTH = 8
MAX_LINE = 4096
# Keep in sync with CONF_HASH_SECRET_SUBSTRINGS / CONF_HASH_SECRET_SUFFIX in src/conf_hash_core.h.
SECRET_SUBSTRINGS = (b"pass", b"psk", b"passkey", b"password", b"secret", b"token")
SECRET_SUFFIX = b"_key"
_SPACE = b" \t\n\v\f\r\0"  # what conf.h trim() strips


class ConfHashError(Exception):
    pass


def _b(s) -> bytes:
    return s if isinstance(s, bytes) else s.encode("utf-8")


def is_secret_key(key) -> bool:
    k = _b(key).lower()  # bytes.lower() is ASCII-only, like the firmware
    return any(s in k for s in SECRET_SUBSTRINGS) or k.endswith(SECRET_SUFFIX)


def redact_uris(s: bytes):
    """Over-redacting URI userinfo: left to right, for each b"://", if any '@' follows on the same
    line (up to the next b"\\n" or the end), everything after b"://" up to the LAST such '@' becomes
    the marker and scanning resumes after the marker (else right after b"://")."""
    n = 0
    pos = 0
    while True:
        p = s.find(b"://", pos)
        if p < 0:
            return s, n
        a = p + 3
        e = s.find(b"\n", a)
        if e < 0:
            e = len(s)
        at = s.rfind(b"@", a, e)
        pos = a
        if at >= 0:
            s = s[:a] + MARKER + s[at:]
            pos = a + len(MARKER)
            n += 1


def redact_line(line: bytes):
    """Redact one line given WITHOUT its b"\\n". Returns (line, n_redactions).

    For each b"=" left to right, left = the line bytes before it; if left.lower() (ASCII) contains a
    secret substring, or left with trailing whitespace removed ends with b"_key", everything after
    this b"=" becomes the marker. A superset of the firmware's logical-line and fgets-chunk views of
    a key; over-redaction is accepted. Otherwise URI userinfo is redacted."""
    lower = line.lower()
    eq = lower.find(b"=")
    while eq >= 0:
        left = lower[:eq]
        if any(s in left for s in SECRET_SUBSTRINGS) or left.rstrip(_SPACE).endswith(SECRET_SUFFIX):
            return line[:eq + 1] + MARKER, 1
        eq = lower.find(b"=", eq + 1)
    return redact_uris(line)


def redact(data: bytes):
    """Return (redacted_content, n_redactions); lines split at b"\\n" only, terminators kept."""
    out, n = [], 0
    parts = data.split(b"\n")
    for i, line in enumerate(parts):
        last = i == len(parts) - 1
        if len(line) > MAX_LINE:
            raise ConfHashError("line too long")
        if last and not line:
            break
        r, k = redact_line(line)
        out.append(r if last else r + b"\n")
        n += k
    return b"".join(out), n


def resolve_conf_dir(d) -> Path:
    d = Path(d)
    if not d.is_dir():
        raise ConfHashError(f"not a directory: {d}")
    sub = d / "conf"
    if sub.is_dir() and not any(p.is_file() and p.name.endswith(".conf") for p in d.iterdir()):
        return sub
    return d


def list_files(conf_dir):
    """Relative b'/' paths of the hashed files, in hash order."""
    root = os.fsencode(conf_dir)
    out = []

    def walk(rel: bytes, depth: int):
        full = os.path.join(root, rel) if rel else root
        for name in os.listdir(full):
            r = rel + b"/" + name if rel else name
            p = os.path.join(root, r)
            st = os.lstat(p)
            if stat.S_ISDIR(st.st_mode):
                if depth >= MAX_DEPTH:
                    raise ConfHashError(f"too deep: {os.fsdecode(p)}")
                walk(r, depth + 1)
            elif stat.S_ISREG(st.st_mode):
                if r.endswith(STAGING_SUFFIX):
                    raise ConfHashError(f"staging file present: {os.fsdecode(r)}")
                if r != EXCLUDED:
                    out.append(r)
            else:
                raise ConfHashError(f"not a regular file: {os.fsdecode(p)}")

    walk(b"", 0)
    out.sort()
    return out


def conf_hash(conf_dir):
    """Return (sha256_hex, [(relpath_bytes, sha256_hex, nbytes, nredacted), ...], total_bytes,
    total_redacted). Hashes and byte counts are over the redacted content."""
    root = os.fsencode(conf_dir)
    h = hashlib.sha256()
    files = []
    total = red_total = 0
    for rel in list_files(conf_dir):
        with open(os.path.join(root, rel), "rb") as f:
            raw = f.read()
        try:
            data, nred = redact(raw)
        except ConfHashError as e:
            raise ConfHashError(f"{e} in {os.fsdecode(rel)}") from None
        h.update(rel + b"\0" + str(len(data)).encode("ascii") + b"\0")
        h.update(data)
        files.append((rel, hashlib.sha256(data).hexdigest(), len(data), nred))
        total += len(data)
        red_total += nred
    return h.hexdigest(), files, total, red_total


_INT_FMT = {"u8": "<B", "i8": "<b", "u16": "<H", "i16": "<h",
            "u32": "<I", "i32": "<i", "u64": "<Q", "i64": "<q"}


def nvs_value_bytes(typename: str, value) -> bytes:
    if typename in _INT_FMT:
        return struct.pack(_INT_FMT[typename], value)
    if typename in ("str", "blob"):
        return _b(value) if isinstance(value, str) else bytes(value)
    raise ConfHashError(f"unknown NVS type {typename!r}")


def nvs_hash_counted(entries):
    """entries: iterable of (key, typename, value). Returns (hex digest, n_redactions)."""
    h = hashlib.sha256()
    n = 0
    for key, typename, value in sorted(entries, key=lambda e: _b(e[0])):
        if is_secret_key(key):  # the firmware never reads a secret entry
            h.update(_b(key) + b"\0redacted")
            n += 1
            continue
        v = nvs_value_bytes(typename, value)
        if typename == "str":
            v, k = redact_uris(v)
            n += k
        h.update(_b(key) + b"\0" + typename.encode("ascii") + b"\0" + str(len(v)).encode("ascii") + b"\0" + v)
    return h.hexdigest(), n


def nvs_hash(entries):
    return nvs_hash_counted(entries)[0]


def _show(b: bytes) -> str:
    return b.decode("utf-8", "backslashreplace")


def format_lines(sha, files, total, redacted):
    lines = [f"conf sha256={sha} files={len(files)} bytes={total} redacted={redacted} "
             f"excluded={_show(EXCLUDED)}"]
    lines += [f"conf-file {_show(rel)} sha256={fsha[:16]} bytes={n} redacted={r}"
              for rel, fsha, n, r in files]
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", help="conf dir, or a profile root / littlefs dump containing conf/")
    ap.add_argument("--json", action="store_true", help="print JSON instead of console lines")
    args = ap.parse_args(argv)
    try:
        conf_dir = resolve_conf_dir(args.dir)
        print(f"# hashing {conf_dir}", file=sys.stderr)
        sha, files, total, redacted = conf_hash(conf_dir)
    except (ConfHashError, OSError, RecursionError) as e:
        print(f"conf sha256=unknown excluded={_show(EXCLUDED)} error={e}")
        return 1
    if args.json:
        print(json.dumps({"conf_dir": str(conf_dir), "sha256": sha, "files": len(files), "bytes": total,
                          "redacted": redacted, "excluded": _show(EXCLUDED),
                          "per_file": [{"path": _show(r), "sha256": s, "bytes": n, "redacted": k}
                                       for r, s, n, k in files]}, indent=2))
    else:
        print("\n".join(format_lines(sha, files, total, redacted)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
