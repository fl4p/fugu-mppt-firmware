#!/usr/bin/env python3
"""Move a shared checkout's branch to NEW (default origin/main) and keep everyone's uncommitted edits.

  etc/realign-checkout.py [-n|--dry-run] [--no-fetch] [--allow-unpushed] [NEW]

For each path that differs between HEAD and NEW:
  unchanged here            -> NEW's version (written, created or deleted; mode follows NEW)
  edited here               -> 3-way merge, base HEAD; any conflict leaves the working file as it is
  untracked, added upstream -> 3-way merge with the upstream version closest to it as the base
  anything else             -> left as it is and reported (deleted on one side, symlinks, file/directory
                               swaps, case-only renames, binary conflicts, mode conflicts, a file edited
                               during the run, a submodule that cannot follow its pin)
A path left as it is keeps a working copy based on the old HEAD, so `git diff` shows upstream's change
reverted there until it is resolved. Backups of written files and merge results with conflict markers go
to .git/realign-backups/<time>/{files,conflicts}/.
Holds the index lock from planning to the end, so no commit lands meanwhile; the branch moves by
compare-and-swap, the files are written, then the new index replaces the old one (signals are held from the
ref move to the index swap; after a SIGKILL there, index.lock holds the new index). Submodules whose checkout is clean and at the old pin follow the new pin.
Refuses (exit 2; only the fetch of NEW has happened) when HEAD has commits with no equivalent in NEW or a
merge commit (--allow-unpushed), the index has staged changes, HEAD is detached, the index is locked, a
merge/rebase/cherry-pick is in progress, sparse or skip-worktree entries exist, or .gitattributes /
core.autocrlf are in use. Exit 0 done, 1 paths left to resolve (the branch moved).
"""
import argparse
import difflib
import os
import signal
import stat
import subprocess
import sys
import tempfile
import time

ENC = 'surrogateescape'


def git(*a, cwd=None, ok=(0,), env=None):
    r = subprocess.run(['git', *a], cwd=cwd, capture_output=True, env=env)
    if r.returncode not in ok:
        raise RuntimeError(f"git {' '.join(a)}: {r.stderr.decode(errors='replace')[:300]}")
    return r


def out(*a, **k):
    return git(*a, **k).stdout.decode('utf-8', ENC).strip()


def zlist(*a):
    return [p for p in git(*a).stdout.decode('utf-8', ENC).split('\0') if p]


def entry(rev, p):
    """(mode, blob bytes) of p in rev; (None, None) if absent; blob None for a non-file (tree, link, gitlink)."""
    s = git('ls-tree', '-z', rev, '--', p).stdout.decode('utf-8', ENC).rstrip('\0')
    if not s:
        return None, None
    m = s.split()[0]
    return m, git('cat-file', 'blob', f'{rev}:{p}').stdout if m in ('100644', '100755') else None


def local(p):
    """('file', bytes, exec) | ('none',) | ('other',) for the working-tree path."""
    try:
        st = os.lstat(p)
    except (FileNotFoundError, NotADirectoryError):
        return ('none',)
    if not stat.S_ISREG(st.st_mode):
        return ('other',)
    with open(p, 'rb') as f:
        return ('file', f.read(), bool(st.st_mode & 0o100))


def merge3(ours, base, theirs):
    """(merged bytes, ok) of a diff3 merge; ok False on any conflict or a merge-file failure (binary)."""
    with tempfile.TemporaryDirectory() as td:
        f = [os.path.join(td, n) for n in ('here', 'base', 'upstream')]
        for fn, data in zip(f, (ours, base, theirs)):
            open(fn, 'wb').write(data)
        r = subprocess.run(['git', 'merge-file', '-p', '--diff3', '-L', 'here', '-L', 'base', '-L', 'upstream',
                            *f], capture_output=True)
    return r.stdout, r.returncode == 0


def closest_upstream(p, wt, old, new):
    """(commit, blob) of the version of p in old..new with the smallest line diff to wt."""
    best = None
    wl = wt.decode('utf-8', ENC).splitlines(True)
    for c in out('log', '--format=%H', f'{old}..{new}', '--', p).split():
        b = entry(c, p)[1]
        if b is None:
            continue
        sm = difflib.SequenceMatcher(None, b.decode('utf-8', ENC).splitlines(True), wl, autojunk=False)
        d = sum(max(i2 - i1, j2 - j1) for op, i1, i2, j1, j2 in sm.get_opcodes() if op != 'equal')
        if best is None or d < best[0]:
            best = (d, c, b)
    return best and best[1:]


def plan_path(p, old, new):
    """(action, snapshot): action ('write', data, exec, how) | ('delete',) | ('chmod', exec) | ('skip',)
    | ('left', why, merged-or-None); snapshot is local(p) at planning time."""
    mo, bo = entry(old, p)
    mn, bn = entry(new, p)
    w = local(p)
    if mo == '160000' and mn == '160000':
        return ('skip',), w
    if w[0] == 'other' or (mo and bo is None) or (mn and bn is None):
        return ('left', 'not a regular file on one side (symlink, directory, submodule), check by hand', None), w
    xo, xn = mo == '100755', mn == '100755'
    if w[0] == 'none':
        if bo is None:
            return ('write', bn, xn, None), w
        return (('skip',) if bn is None else ('left', 'deleted here, changed upstream', None)), w
    data, xw = w[1], w[2]
    if bn is None:
        if bo == data and xw == xo:
            return ('delete',), w
        return ('left', 'deleted upstream, edited here (now untracked)', None), w
    if bo is None:
        if xw != xn:
            return ('left', 'untracked copy here has a different mode than upstream', None), w
        x = xn
    else:
        x = xn if xw == xo else xw   # a local mode change wins over no upstream change; both changed = same
    if data == bn:
        return (('chmod', x) if xw != x else ('skip',)), w
    if data == bo:
        return ('write', bn, x, None), w
    base, how = bo, 'merged'
    if bo is None:
        best = closest_upstream(p, data, old, new)
        if best is None:
            return ('left', 'added upstream, untracked copy here', None), w
        base, how = best[1], f'merged untracked copy (base {best[0][:8]})'
    merged, ok = merge3(data, base, bn)
    if not ok:
        return ('left', 'conflict' if merged else 'merge failed (binary?)', merged or None), w
    return ('write', merged, x, how), w


def submodule_pins(old, new, paths):
    res = []
    for p in paths:
        mo, mn = entry(old, p)[0], entry(new, p)[0]
        if mo == '160000' and mn == '160000':
            pin = lambda r: out('ls-tree', r, '--', p).split()[2]
            res.append((p, pin(old), pin(new)))
    return res


def follow_submodule(p, o, n, dry):
    """(ok, message) of moving submodule p from pin o to pin n."""
    if not os.path.exists(os.path.join(p, '.git')):
        return True, f'{p}: not checked out, left alone'
    head = out('rev-parse', 'HEAD', cwd=p)
    if head == n:
        return True, None
    if head != o:
        return False, f'{p}: at {head[:8]}, neither old pin {o[:8]} nor new pin {n[:8]}'
    if out('status', '--porcelain', cwd=p):
        return False, f'{p}: has uncommitted changes, left at old pin {o[:8]}'
    have = lambda: git('cat-file', '-e', f'{n}^{{commit}}', cwd=p, ok=(0, 1, 128)).returncode == 0
    if dry:
        return True, f'{p}: would move {o[:8]} -> {n[:8]}' + ('' if have() else ' (after fetching it)')
    if not have():
        git('fetch', '-q', 'origin', cwd=p, ok=(0, 1, 128))
        if not have():
            return False, f'{p}: new pin {n[:8]} not fetchable, left at {o[:8]}'
    if out('branch', '--show-current', cwd=p) and \
            git('merge-base', '--is-ancestor', head, n, cwd=p, ok=(0, 1)).returncode == 0:
        git('merge', '-q', '--ff-only', n, cwd=p)
    else:
        git('checkout', '-q', '--detach', n, cwd=p)
    return True, f'{p}: {o[:8]} -> {n[:8]}'


def refusals(a, old, new, branch):
    why = [] if branch else ['HEAD is detached']
    for f in ('MERGE_HEAD', 'CHERRY_PICK_HEAD', 'REVERT_HEAD', 'rebase-merge', 'rebase-apply', 'BISECT_LOG'):
        if os.path.exists(out('rev-parse', '--git-path', f)):
            why.append(f'a merge, cherry-pick, revert, rebase or bisect is in progress ({f})')
    if git('config', '--get', '--bool', 'core.sparseCheckout', ok=(0, 1)).stdout.strip() == b'true' or \
            any(l[:1] == 'S' or l[:1].islower() for l in out('ls-files', '-v').splitlines()):
        why.append('sparse checkout, skip-worktree or assume-unchanged entries (read-tree would drop them)')
    if git('diff', '--cached', '--ita-visible-in-index', '--ignore-submodules=none', '--quiet',
           ok=(0, 1)).returncode:
        why.append('the index has staged changes')
    unpushed = [l[2:] for l in out('cherry', new, old).splitlines() if l.startswith('+')]
    merges = out('rev-list', '--merges', f'{new}..{old}').split()
    if (unpushed or merges) and not a.allow_unpushed:
        for c in unpushed + merges:
            print('  not in NEW:', out('log', '-1', '--format=%h %s', c))
        why.append(f'HEAD has {len(unpushed)} commits and {len(merges)} merges not in {a.new}: push them first, '
                   'or pass --allow-unpushed (they stay reachable from the reflog)')
    if any(os.path.basename(p) == '.gitattributes' for r in (old, new)
           for p in zlist('ls-tree', '-r', '-z', '--name-only', r)) or os.path.exists('.gitattributes') \
            or os.path.exists(out('rev-parse', '--git-path', 'info/attributes')) \
            or git('config', '--get', 'core.attributesFile', ok=(0, 1)).stdout.strip() \
            or git('config', '--get', 'core.autocrlf', ok=(0, 1)).stdout.decode().strip() in ('true', 'input'):
        why.append('.gitattributes, core.attributesFile or core.autocrlf in use (filters/eol not handled)')
    return why


def remove_empty_parents(p):
    d = os.path.dirname(p)
    while d:
        try:
            os.rmdir(d)
        except OSError:
            return
        d = os.path.dirname(d)


def umask():
    m = os.umask(0)
    os.umask(m)
    return m


def apply(plan, bk, top):
    """Write the planned paths (deletions first) if each still matches its snapshot; returns [(path, why)]."""
    failed = []
    for p, (act, snap) in sorted(plan.items(), key=lambda kv: (kv[1][0][0] != 'delete', kv[0])):
        kind = act[0]
        try:
            d = os.path.normpath(os.path.join(top, os.path.dirname(p)))
            if os.path.realpath(d) != d:
                raise OSError('a parent directory is a symlink')
            w = local(p)
            if w != snap:
                failed.append((p, 'changed during the run, left as is'))
                continue
            if w[0] == 'file':
                b = os.path.join(bk, 'files', p)
                os.makedirs(os.path.dirname(b), exist_ok=True)
                open(b, 'wb').write(w[1])
            if kind == 'delete':
                os.remove(p)
                remove_empty_parents(p)
                continue
            if kind == 'write':
                os.makedirs(os.path.dirname(p) or '.', exist_ok=True)
                fd, tmp = tempfile.mkstemp(dir=os.path.dirname(p) or '.', prefix='.realign-')
                try:
                    with os.fdopen(fd, 'wb') as f:
                        f.write(act[1])
                    os.chmod(tmp, 0o666 & ~umask())
                    if local(p) != snap:
                        raise OSError('changed during the run')
                    os.replace(tmp, p)
                except BaseException:
                    if os.path.exists(tmp):
                        os.remove(tmp)
                    raise
            x = act[2] if kind == 'write' else act[1]
            m = os.stat(p).st_mode
            os.chmod(p, (m | (0o111 & ~umask())) if x else (m & ~0o111))
        except OSError as e:
            failed.append((p, f'write failed: {e}'))
    return failed


SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def on_signal(*_):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('new', nargs='?', default='origin/main')
    ap.add_argument('-n', '--dry-run', action='store_true')
    ap.add_argument('--no-fetch', dest='fetch', action='store_false')
    ap.add_argument('--allow-unpushed', action='store_true', help='move although HEAD has commits not in NEW')
    a = ap.parse_args()
    sys.stdout.reconfigure(errors='backslashreplace')
    for sg in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sg, on_signal)
    st = {'moved': False, 'bk': None, 'own': False, 'lock': None, 'branch': None, 'new': None}
    try:
        return run(a, st)
    except BaseException as e:
        moved = st['moved'] or (st['branch'] and st['new'] and
                                git('rev-parse', '-q', '--verify', st['branch'], ok=(0, 1)).stdout.decode().strip()
                                == st['new'])
        if not moved:
            print(f'realign: refusing, {e!r}; nothing changed')
            return 2
        hint = f"; {st['lock']} holds the new index: `mv` it onto the index" if st['own'] else ''
        st['own'] = False                      # the lock now carries the new index, never delete it
        print(f'realign: STOPPED after the branch moved ({e!r}); backup {st["bk"]}{hint}; check `git status`')
        return 1
    finally:
        if st['own'] and os.path.exists(st['lock']):
            os.remove(st['lock'])


def run(a, st):
    top = os.path.realpath(out('rev-parse', '--show-toplevel'))
    os.chdir(top)
    if a.fetch and a.new.startswith('origin/'):
        git('fetch', '-q', 'origin', a.new.split('/', 1)[1])
    index = os.path.abspath(out('rev-parse', '--git-path', 'index'))
    lock = st['lock'] = index + '.lock'
    if not a.dry_run:
        try:
            os.close(os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644))
        except FileExistsError:
            print(f'realign: refusing, {lock} exists (another git command is running); nothing changed')
            return 2
        st['own'] = True
    old, new = out('rev-parse', 'HEAD'), out('rev-parse', '--verify', a.new + '^{commit}')
    branch = git('symbolic-ref', '-q', 'HEAD', ok=(0, 1)).stdout.decode().strip()
    why = refusals(a, old, new, branch)
    if why:
        for w in why:
            print('realign: refusing,', w)
        return 2

    paths = zlist('diff', '--no-renames', '--ignore-submodules=none', '--name-only', '-z', old, new)
    plan, left = {}, []
    folded = {}
    if git('config', '--get', '--bool', 'core.ignorecase', ok=(0, 1)).stdout.strip() == b'true':
        for p in paths:
            folded.setdefault(p.lower(), []).append(p)
    for p in paths:
        if len(folded.get(p.lower(), ())) > 1:
            left.append((p, 'case-only rename on a case-insensitive file system, check by hand', None))
            continue
        act, snap = plan_path(p, old, new)
        if act[0] == 'left':
            left.append((p, act[1], act[2]))
        elif act[0] != 'skip':
            plan[p] = (act, snap)
    subs = submodule_pins(old, new, paths)
    merged = [(p, act[3]) for p, (act, _) in plan.items() if act[0] == 'write' and act[3]]
    print(f'{branch.removeprefix("refs/heads/")} {old[:8]} -> {new[:8]}: {len(paths)} paths differ, '
          f'{len(plan) - len(merged)} updated, {len(merged)} merged, {len(left)} left to resolve')
    for p, how in merged:
        print(f'  {how}: {p}')
    for p, w, _ in left:
        print(f'  LEFT ({w}): {p}')
    if a.dry_run:
        for sm in subs:
            ok, msg = follow_submodule(*sm, dry=True)
            if msg:
                print('  submodule' if ok else '  LEFT submodule', msg)
            left += [] if ok else [(sm[0], msg, None)]
        return 1 if left else 0

    st['bk'] = bk = os.path.join(os.path.abspath(out('rev-parse', '--git-path', 'realign-backups')),
                                 time.strftime('%Y%m%d-%H%M%S'))
    os.makedirs(bk, exist_ok=True)
    print('backup:', bk)
    git('read-tree', new, env=dict(os.environ, GIT_INDEX_FILE=lock))   # the new index, inside our lock
    # From the ref move to the index swap: no signal, and the lock (holding the new index) keeps commits out.
    signal.pthread_sigmask(signal.SIG_BLOCK, SIGNALS)
    try:
        st['branch'], st['new'] = branch, new
        git('update-ref', branch, new, old, ok=(0, 1, 128))
        if out('rev-parse', branch) != new:
            print(f'realign: refusing, update-ref failed ({branch} moved or locked); nothing changed')
            return 2
        st['moved'] = True
        for p, w in apply(plan, bk, top):
            left.append((p, w, None))
            print(f'  LEFT ({w}): {p}')
        os.replace(lock, index)
        st['own'] = False
    finally:
        signal.pthread_sigmask(signal.SIG_UNBLOCK, SIGNALS)
    for p, w, m in left:
        if m:
            d = os.path.join(bk, 'conflicts', p)
            os.makedirs(os.path.dirname(d), exist_ok=True)
            open(d, 'wb').write(m)
    git('update-index', '-q', '--refresh', ok=(0, 1, 128))
    for sm in subs:
        try:
            ok, msg = follow_submodule(*sm, dry=False)
        except RuntimeError as e:
            ok, msg = False, f'{sm[0]}: {e}'
        if msg:
            print('  submodule' if ok else '  LEFT submodule', msg)
        left += [] if ok else [(sm[0], msg, None)]
    if left:
        print('paths LEFT keep a working copy based on the old HEAD: `git diff` shows upstream\'s change reverted '
              'there; resolve them (merge results under the backup\'s conflicts/) before anyone commits them')
    return 1 if left else 0


if __name__ == '__main__':
    sys.exit(main())
