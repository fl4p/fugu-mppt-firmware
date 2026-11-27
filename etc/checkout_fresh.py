#!/usr/bin/env python3
"""How far a git checkout (or a revision) is from origin/main.

  checkout_fresh.py rev [--repo R] [--strict] [--no-fetch] REV
    exit 0 REV contains origin/main (or is behind it, with a warning, without --strict), 1 REV is behind
    origin/main and --strict, 2 REV or origin/main unknown. A failed fetch falls back to the cached origin/main and says so.
"""
import argparse
import os
import subprocess
import sys

UPSTREAM = 'origin/main'
TOOL_PATHS = ('etc/ota_mqtt.py', 'etc/ota_common.py', 'etc/ota-via-relay.sh', 'etc/fugu_console.py',
              'etc/migrate_side_keys.py', 'etc/checkout_fresh.py', 'etc/protected-devices', 'etc/fugu',
              'etc/idf-devtools')
TOOL_SUBMODULES = ('etc/fugu', 'etc/idf-devtools')


def git(repo, *a, timeout=None):
    try:
        r = subprocess.run(['git', '-C', repo, *a], capture_output=True, text=True, timeout=timeout,
                           stdin=subprocess.DEVNULL)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout.strip() if r.returncode == 0 else None


def fetch(repo, timeout=20):
    """Fetch origin main; returns None on success, else why it failed."""
    try:
        r = subprocess.run(['git', '-C', repo, 'fetch', '-q', 'origin', 'main'], capture_output=True,
                           text=True, timeout=timeout, stdin=subprocess.DEVNULL,
                           env={**os.environ, 'GIT_TERMINAL_PROMPT': '0',
                                'GIT_SSH_COMMAND': os.environ.get('GIT_SSH_COMMAND', 'ssh -o BatchMode=yes')})
    except subprocess.TimeoutExpired:
        return f'git fetch timed out after {timeout} s'
    except OSError as e:
        return str(e)
    return None if r.returncode == 0 else (r.stderr.strip().splitlines() or ['git fetch failed'])[-1]


def compare(repo, rev='HEAD', do_fetch=True):
    """{'behind', 'ahead', 'fetch_error', 'upstream'} of rev vs origin/main; None if either is unknown."""
    err = fetch(repo) if do_fetch else 'not fetched'
    up = git(repo, 'rev-parse', '--verify', '-q', UPSTREAM + '^{commit}')
    sha = git(repo, 'rev-parse', '--verify', '-q', rev + '^{commit}')
    if not up or not sha:
        return None
    n = git(repo, 'rev-list', '--left-right', '--count', f'{sha}...{up}')
    if not n:
        return None
    ahead, behind = map(int, n.split())
    return {'sha': sha, 'upstream': up, 'ahead': ahead, 'behind': behind, 'fetch_error': err}


def provenance(c):
    return f"cached {UPSTREAM} {c['upstream'][:8]} ({c['fetch_error']})" if c['fetch_error'] \
        else f"{UPSTREAM} {c['upstream'][:8]}"


def submodules_behind(repo, paths=TOOL_SUBMODULES):
    """[(path, checked-out, pin)] of submodules checked out at an ancestor of the commit HEAD pins;
    None if one is not checked out or its pin is unreadable."""
    out = []
    for p in paths:
        t = git(repo, 'ls-tree', 'HEAD', p)
        if t is None:
            return None
        pin = t.split()
        if len(pin) < 3:
            continue  # not a submodule of this checkout
        sub = os.path.join(repo, p)
        head = git(sub, 'rev-parse', 'HEAD') if os.path.exists(os.path.join(sub, '.git')) else None
        if not head:
            return None
        if head == pin[2]:
            continue
        if git(sub, 'cat-file', '-e', pin[2] + '^{commit}') is None:
            out.append((p, head, pin[2]))  # pin not even fetched: older than it
        elif subprocess.run(['git', '-C', sub, 'merge-base', '--is-ancestor', head, pin[2]],
                          capture_output=True).returncode == 0:
            out.append((p, head, pin[2]))
    return out


def stale_tools(repo, c):
    """Reasons the OTA tooling in `repo` is older than origin/main; [] if current, None if unverifiable.
    c: compare(repo, 'HEAD')."""
    if c is None:
        return None
    why = []
    if c['behind']:
        changed = git(repo, 'diff', '--name-only', f"HEAD...{c['upstream']}", '--', *TOOL_PATHS)
        if changed is None:
            return None
        if changed:
            why.append(f"checkout is {c['behind']} commits behind {provenance(c)}, which changes "
                       + ', '.join(changed.split()))
    subs = submodules_behind(repo)
    if subs is None:
        return None
    for p, head, pin in subs:
        why.append(f'{p} is checked out at {head[:8]}, behind its pin {pin[:8]} '
                   f'(git -c protocol.file.allow=always submodule update {p})')
    return why


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='cmd', required=True)
    r = sp.add_parser('rev')
    r.add_argument('rev')
    r.add_argument('--repo', default='.')
    r.add_argument('--strict', action='store_true', help='refuse (exit 1) when behind')
    r.add_argument('--no-fetch', dest='fetch', action='store_false')
    a = ap.parse_args()
    c = compare(a.repo, a.rev, a.fetch)
    if c is None:
        print(f'checkout-fresh: cannot compare {a.rev} with {UPSTREAM}', file=sys.stderr)
        return 2
    if c['ahead']:
        print(f"checkout-fresh: WARNING {c['sha'][:8]} has {c['ahead']} commits that are not on "
              f"{provenance(c)}: push them first, or the image matches no pushed commit", file=sys.stderr)
    if not c['behind']:
        if c['fetch_error']:
            print(f'checkout-fresh: compared with {provenance(c)}', file=sys.stderr)
        return 0
    msg = f"{a.rev} ({c['sha'][:8]}) is {c['behind']} commits behind {provenance(c)}"
    if a.strict:
        print(f'checkout-fresh: refusing, {msg}; name the revision to use it anyway (e.g. {UPSTREAM})',
              file=sys.stderr)
        return 1
    print(f'checkout-fresh: WARNING {msg}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
