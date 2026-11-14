#!/usr/bin/env python3
"""Host twin of the firmware's `conf-hash` (src/conf_hash.cpp); must agree byte for byte.

    etc/conf_hash.py DIR

prints the device's `conf-file <name> sha256=<16 hex>` lines and `conf sha256=<64 hex> files=<N>`
(the device line also has `boot=`). DIR is a conf dir, or a profile root / littlefs dump with a
`conf/` subdir. Definition: SHA-256 over the top-level *.conf files except wifi.conf, sorted
bytewise, each fed as  name '\\0' redacted-content '\\0'.  The per-file hash is over the redacted
content. Redaction per line (split at '\\n'): if the ASCII-lowercased line contains a SECRET word,
everything after the first '=' following it becomes <redacted>; otherwise the bytes between the
first '://' and the last '@' after it do.
"""
import hashlib
import os
import sys

EXCLUDED = b"wifi.conf"
SECRET = (b"pass", b"psk", b"secret", b"token", b"_key")
MARKER = b"<redacted>"


def redact_line(line: bytes) -> bytes:
    lo = line.lower()
    ends = [lo.find(w) + len(w) for w in SECRET if w in lo]
    if ends:
        eq = line.find(b"=", min(ends))
        if eq >= 0:
            return line[:eq + 1] + MARKER
    a = line.find(b"://")
    if a >= 0:
        at = line.rfind(b"@", a + 3)
        if at >= 0:
            return line[:a + 3] + MARKER + line[at:]
    return line


def redact(data: bytes) -> bytes:
    return b"\n".join(redact_line(line) for line in data.split(b"\n"))


def resolve_conf_dir(d: str) -> str:
    sub = os.path.join(d, "conf")
    if os.path.isdir(sub) and not any(n.endswith(".conf") for n in os.listdir(d)):
        return sub
    return d


def conf_hash(d):
    """Return (hex digest, [(name, hex digest), ...])."""
    h = hashlib.sha256()
    files = []
    for name in sorted(n for n in os.listdir(os.fsencode(d)) if n.endswith(b".conf") and n != EXCLUDED):
        with open(os.path.join(os.fsencode(d), name), "rb") as f:
            r = redact(f.read())
        h.update(name + b"\0" + r + b"\0")
        files.append((name, hashlib.sha256(r).hexdigest()))
    return h.hexdigest(), files


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    try:
        sha, files = conf_hash(resolve_conf_dir(argv[0]))
    except OSError as e:
        print(f"conf sha256=unknown error={e}")
        return 1
    for name, fsha in files:
        print(f"conf-file {os.fsdecode(name)} sha256={fsha[:16]}")
    print(f"conf sha256={sha} files={len(files)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
