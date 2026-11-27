#!/usr/bin/env python3
"""Host tests for etc/realign-checkout.py; throwaway git repos.
Run: .venv/bin/python3 -m pytest etc/test_realign_checkout.py"""
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'realign-checkout.py')
ENV = dict(os.environ, GIT_AUTHOR_NAME='t', GIT_AUTHOR_EMAIL='t@t', GIT_COMMITTER_NAME='t',
           GIT_COMMITTER_EMAIL='t@t', GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM='1')


def g(repo, *a):
    return subprocess.run(['git', '-C', repo, '-c', 'protocol.file.allow=always', *a], check=True,
                          capture_output=True, text=True, env=ENV).stdout.strip()


def write(repo, path, text):
    p = os.path.join(repo, path)
    os.makedirs(os.path.dirname(p) or repo, exist_ok=True)
    open(p, 'w').write(text)


def commit(repo, files, msg='c'):
    for path, text in files.items():
        write(repo, path, text)
        g(repo, 'add', path)
    g(repo, 'commit', '-qm', msg)
    return g(repo, 'rev-parse', 'HEAD')


def lines(*xs):
    return ''.join(f'{x}\n' for x in xs)


class Realign(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.up, self.bare, self.co = (os.path.join(self.tmp, n) for n in ('up', 'origin.git', 'co'))
        os.makedirs(self.up)
        g(self.up, 'init', '-q', '-b', 'main')
        commit(self.up, {'a.txt': lines(1, 2, 3, 4, 5, 6, 7, 8), 'b.txt': 'b\n', 'gone.txt': 'g\n'})
        g(self.tmp, 'clone', '-q', '--bare', self.up, self.bare)
        g(self.tmp, 'clone', '-q', self.bare, self.co)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def upstream(self, files, rm=()):
        for p in rm:
            g(self.up, 'rm', '-q', p)
        c = commit(self.up, files)
        g(self.up, 'push', '-q', self.bare, 'main')
        return c

    def run_tool(self, *a):
        return subprocess.run([sys.executable, TOOL, *a], cwd=self.co, capture_output=True, text=True, env=ENV)

    def read(self, p):
        return open(os.path.join(self.co, p)).read()

    def head(self):
        return g(self.co, 'rev-parse', 'HEAD')

    def test_clean_and_dirty_paths(self):
        new = self.upstream({'a.txt': lines(1, 2, 3, 4, 5, 6, 7, 'eight'), 'b.txt': 'b2\n', 'c.txt': 'c\n'},
                            rm=['gone.txt'])
        write(self.co, 'a.txt', lines('one', 2, 3, 4, 5, 6, 7, 8))   # dirty, mergeable
        r = self.run_tool()
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(self.head(), new)
        self.assertEqual(self.read('a.txt'), lines('one', 2, 3, 4, 5, 6, 7, 'eight'))
        self.assertEqual(self.read('b.txt'), 'b2\n')
        self.assertEqual(self.read('c.txt'), 'c\n')
        self.assertFalse(os.path.exists(os.path.join(self.co, 'gone.txt')))
        self.assertEqual(g(self.co, 'status', '--porcelain'), 'M a.txt')

    def test_conflict_keeps_working_copy_and_exits_1(self):
        new = self.upstream({'a.txt': lines('up', 2, 3, 4, 5, 6, 7, 8)})
        write(self.co, 'a.txt', lines('here', 2, 3, 4, 5, 6, 7, 8))
        r = self.run_tool()
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn('LEFT (conflict): a.txt', r.stdout)
        self.assertEqual(self.read('a.txt'), lines('here', 2, 3, 4, 5, 6, 7, 8))
        self.assertEqual(self.head(), new)
        bk = r.stdout.split('backup: ')[1].split()[0]
        self.assertIn('<<<<<<< here', open(os.path.join(bk, 'conflicts', 'a.txt')).read())

    def test_edit_already_upstream_resolves(self):
        self.upstream({'a.txt': lines(1, 'x', 3, 4, 5, 6, 7, 8)})
        write(self.co, 'a.txt', lines(1, 'x', 3, 4, 5, 6, 7, 8))
        r = self.run_tool()
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertEqual(g(self.co, 'status', '--porcelain'), '')

    def test_untracked_copy_of_an_upstream_commit_gets_the_later_fix(self):
        # an untracked copy of a file at its first upstream commit, plus a local edit; upstream fixes it later
        mid = lines('p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8')
        self.upstream({'n.txt': mid})
        self.upstream({'n.txt': lines('p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'FIX')})
        write(self.co, 'n.txt', lines('LOCAL', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8'))
        r = self.run_tool()
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertEqual(self.read('n.txt'), lines('LOCAL', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'FIX'))

    def test_unpushed_commit_refuses(self):
        self.upstream({'b.txt': 'b2\n'})
        local = commit(self.co, {'mine.txt': 'm\n'})
        r = self.run_tool()
        self.assertEqual(r.returncode, 2, r.stdout)
        self.assertEqual(self.head(), local)
        r = self.run_tool('--allow-unpushed')
        self.assertEqual(r.returncode, 0, r.stdout)

    def test_cherry_picked_equivalent_is_not_unpushed(self):
        commit(self.co, {'b.txt': 'b2\n'})
        self.upstream({'b.txt': 'b2\n'})
        self.assertEqual(self.run_tool().returncode, 0)

    def test_staged_changes_and_dry_run(self):
        self.upstream({'b.txt': 'b2\n'})
        old = self.head()
        r = self.run_tool('-n')
        self.assertEqual((r.returncode, self.head(), self.read('b.txt')), (0, old, 'b\n'))
        write(self.co, 'a.txt', 'staged\n')
        g(self.co, 'add', 'a.txt')
        self.assertEqual(self.run_tool().returncode, 2)
        self.assertEqual(self.head(), old)

    def test_submodule_at_old_pin_follows(self):
        sub = os.path.join(self.tmp, 'sub')
        os.makedirs(sub)
        g(sub, 'init', '-q', '-b', 'master')
        s1 = commit(sub, {'s': '1\n'})
        g(self.up, 'submodule', 'add', '-q', sub, 'm')
        g(self.up, 'commit', '-qm', 'sub')
        g(self.up, 'push', '-q', self.bare, 'main')
        g(self.co, 'pull', '-q')
        g(self.co, 'submodule', 'update', '-q', '--init')
        s2 = commit(sub, {'s': '2\n'})
        g(os.path.join(self.up, 'm'), 'pull', '-q', 'origin', 'master')
        g(self.up, 'add', 'm')
        self.upstream({'b.txt': 'b2\n'})
        r = self.run_tool()
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertEqual(g(os.path.join(self.co, 'm'), 'rev-parse', 'HEAD'), s2)
        self.assertNotEqual(s1, s2)


    # regressions from review: nothing may be lost silently, partial failure must not split HEAD and index
    def conflict_case(self, base, upstream, here):
        g(self.co, 'rm', '-q', 'a.txt')
        commit(self.co, {'a.txt': base})
        g(self.co, 'push', '-q', 'origin', 'HEAD:main')
        g(self.up, 'pull', '-q', self.bare, 'main')
        self.upstream({'a.txt': upstream})
        open(os.path.join(self.co, 'a.txt'), 'wb').write(here.encode() if isinstance(here, str) else here)
        return self.run_tool()

    def test_deletions_are_not_resolved_away(self):
        for up, here in ((lines('a', 'b', 'C2', 'd', 'e'), lines('a', 'b', 'd', 'e')),
                         (lines('a', 'b', 'd', 'e'), lines('a', 'b', 'c_local', 'd', 'e'))):
            with self.subTest(up=up):
                self.tearDown(); self.setUp()
                r = self.conflict_case(lines('a', 'b', 'c', 'd', 'e'), up, here)
                self.assertEqual(r.returncode, 1, r.stdout)
                self.assertEqual(self.read('a.txt'), here)

    def test_whitespace_edit_is_not_resolved_away(self):
        r = self.conflict_case(lines('if x:', '    a', '    b'), lines('if x:', '    a', '    B'), lines('a', 'b'))
        self.assertEqual((r.returncode, self.read('a.txt')), (1, lines('a', 'b')))

    def test_separator_line_in_base(self):
        r = self.conflict_case('C\n=======\nA\n=======\n', 'B\nB\nB\nA\n', 'C\n')
        self.assertEqual((r.returncode, self.read('a.txt')), (1, 'C\n'))

    def test_no_trailing_newline_kept(self):
        r = self.conflict_case(lines('t', 2, 3, 4, 5, 6, 7, 'e'), lines('T', 2, 3, 4, 5, 6, 7, 'e'),
                               't\n2\n3\n4\n5\n6\n7\nE')
        self.assertEqual((r.returncode, self.read('a.txt')), (0, 'T\n2\n3\n4\n5\n6\n7\nE'), r.stdout)

    def test_binary_conflict_is_left_not_fatal(self):
        r = self.conflict_case('\0bin1\n', '\0bin2\n', b'\0bin3\n')
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn('LEFT', r.stdout)
        self.assertEqual(self.head(), g(self.bare, 'rev-parse', 'main'))

    def test_merge_commit_refuses(self):
        g(self.co, 'checkout', '-q', '-b', 'side')
        commit(self.co, {'b.txt': 'side\n'})
        g(self.co, 'checkout', '-q', 'main')
        g(self.co, 'merge', '-q', '--no-ff', '-m', 'm', 'side')
        old = self.head()
        self.assertEqual(self.run_tool().returncode, 2)
        self.assertEqual(self.head(), old)

    def test_modes(self):
        p = os.path.join(self.up, 'a.txt')
        os.chmod(p, 0o755)
        g(self.up, 'add', 'a.txt')
        self.upstream({'b.txt': 'b2\n'})                      # upstream: a.txt +x, b.txt edited
        os.chmod(os.path.join(self.co, 'b.txt'), 0o755)       # here: b.txt +x
        r = self.run_tool()
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertTrue(os.stat(os.path.join(self.co, 'a.txt')).st_mode & 0o100)
        self.assertTrue(os.stat(os.path.join(self.co, 'b.txt')).st_mode & 0o100)
        self.assertEqual(g(self.co, 'status', '--porcelain'), 'M b.txt')

    def test_symlinks_are_left(self):
        outside = os.path.join(self.tmp, 'PRECIOUS')
        open(outside, 'w').write('keep\n')
        os.symlink(outside, os.path.join(self.co, 'n.txt'))   # untracked symlink where upstream adds a file
        self.upstream({'n.txt': 'upstream\n'})
        r = self.run_tool()
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertEqual(open(outside).read(), 'keep\n')

    def test_held_index_lock_changes_nothing(self):
        self.upstream({'b.txt': 'b2\n'})
        old = self.head()
        open(os.path.join(self.co, '.git', 'index.lock'), 'w').close()
        r = self.run_tool()
        self.assertEqual((r.returncode, self.head(), self.read('b.txt')), (2, old, 'b\n'))

    def test_dir_replaced_by_file_does_not_crash(self):
        commit(self.co, {'x/y': 'y\n'})
        g(self.co, 'push', '-q', 'origin', 'HEAD:main')
        g(self.up, 'pull', '-q', self.bare, 'main')
        self.upstream({'x': 'file\n'}, rm=['x/y'])
        r = self.run_tool()
        self.assertIn(r.returncode, (0, 1), r.stdout + r.stderr)
        self.assertEqual(self.head(), g(self.bare, 'rev-parse', 'main'))
        self.assertEqual(g(self.co, 'diff', '--cached', '--name-only'), '')


    def test_branch_moved_during_run_changes_nothing(self):
        self.upstream({'b.txt': 'b2\n'})
        real = shutil.which('git')
        shim = os.path.join(self.tmp, 'bin')
        os.makedirs(shim)
        open(os.path.join(shim, 'git'), 'w').write(
            f'#!/bin/sh\ncase "$1" in update-ref) {real} update-ref HEAD '
            f'$({real} commit-tree "HEAD^{{tree}}" -p HEAD -m race);; esac\n'
            f'exec {real} "$@"\n')
        os.chmod(os.path.join(shim, 'git'), 0o755)
        env = dict(ENV, PATH=shim + os.pathsep + ENV['PATH'])
        r = subprocess.run([sys.executable, TOOL], cwd=self.co, capture_output=True, text=True, env=env)
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn('update-ref failed', r.stdout)
        self.assertEqual(self.read('b.txt'), 'b\n')
        self.assertFalse(os.path.exists(os.path.join(self.co, '.git', 'index.lock')))


    def shim(self, on, script):
        real = shutil.which('git')
        d = os.path.join(self.tmp, 'bin')
        os.makedirs(d, exist_ok=True)
        open(os.path.join(d, 'git'), 'w').write(f'#!/bin/sh\ncase "$1" in {on}) {script.format(git=real)};; esac\n'
                                                f'exec {real} "$@"\n')
        os.chmod(os.path.join(d, 'git'), 0o755)
        return dict(ENV, PATH=d + os.pathsep + ENV['PATH'])

    def test_edit_during_run_is_kept(self):
        self.upstream({'b.txt': 'b2\n'})
        env = self.shim('read-tree', f"echo late >> '{self.co}/b.txt'")
        r = subprocess.run([sys.executable, TOOL], cwd=self.co, capture_output=True, text=True, env=env)
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn('changed during the run', r.stdout)
        self.assertEqual(self.read('b.txt'), 'b\nlate\n')
        self.assertEqual(self.head(), g(self.bare, 'rev-parse', 'main'))

    def test_case_only_rename(self):
        if subprocess.run(['git', '-C', self.co, 'config', '--bool', 'core.ignorecase'], capture_output=True,
                          text=True).stdout.strip() != 'true':
            self.skipTest('case-sensitive file system')
        g(self.up, 'mv', 'b.txt', 'B.txt')
        self.upstream({})
        r = self.run_tool()
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertTrue(os.path.exists(os.path.join(self.co, 'B.txt')))

    def test_both_set_exec_bit(self):
        os.chmod(os.path.join(self.up, 'b.txt'), 0o755)
        g(self.up, 'add', 'b.txt')
        self.upstream({'a.txt': lines('x')})
        os.chmod(os.path.join(self.co, 'b.txt'), 0o755)
        r = self.run_tool()
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertEqual(g(self.co, 'status', '--porcelain'), '')

    def add_submodule(self):
        sub = os.path.join(self.tmp, 'sub')
        os.makedirs(sub)
        g(sub, 'init', '-q', '-b', 'master')
        commit(sub, {'s': '1\n'})
        g(self.up, 'submodule', 'add', '-q', sub, 'm')
        g(self.up, 'commit', '-qm', 'sub')
        g(self.up, 'push', '-q', self.bare, 'main')
        g(self.co, 'pull', '-q')
        g(self.co, 'submodule', 'update', '-q', '--init')
        commit(sub, {'s': '2\n'})
        g(os.path.join(self.up, 'm'), 'pull', '-q', 'origin', 'master')
        g(self.up, 'add', 'm')

    def test_dirty_submodule_is_left(self):
        self.add_submodule()
        self.upstream({'b.txt': 'b2\n'})
        write(os.path.join(self.co, 'm'), 's', 'dirty\n')
        r = self.run_tool()
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn('LEFT submodule', r.stdout)

    def test_submodule_replaced_by_file_is_left(self):
        self.add_submodule()
        self.upstream({})
        g(self.co, 'pull', '-q')
        g(self.up, 'rm', '-q', 'm')
        self.upstream({'m': 'file\n'})
        r = self.run_tool()
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn('LEFT', r.stdout)


    def test_sigterm_during_ref_move_keeps_head_and_index_together(self):
        import signal, time
        self.upstream({'b.txt': 'b2\n'})
        env = self.shim('update-ref', '{git} "$@"; sleep 2; exit 0')
        pr = subprocess.Popen([sys.executable, TOOL], cwd=self.co, stdout=subprocess.PIPE, text=True, env=env)
        time.sleep(1.5)
        pr.send_signal(signal.SIGTERM)
        so, _ = pr.communicate(timeout=30)
        self.assertEqual(self.head(), g(self.bare, 'rev-parse', 'main'), so)
        self.assertEqual(g(self.co, 'diff', '--cached', '--name-only'), '', so)
        self.assertFalse(os.path.exists(os.path.join(self.co, '.git', 'index.lock')), so)
        self.assertEqual(self.read('b.txt'), 'b2\n')

    def test_merge_in_progress_refuses(self):
        g(self.co, 'checkout', '-q', '-b', 'side')
        commit(self.co, {'s.txt': 's\n'})
        g(self.co, 'checkout', '-q', 'main')
        g(self.co, 'merge', '-q', '-s', 'ours', '--no-commit', 'side')
        self.upstream({'b.txt': 'b2\n'})
        self.assertEqual(self.run_tool().returncode, 2)

    def test_sparse_checkout_refuses(self):
        g(self.co, 'update-index', '--skip-worktree', 'b.txt')
        self.upstream({'b.txt': 'b2\n'})
        r = self.run_tool()
        self.assertEqual(r.returncode, 2, r.stdout)

    def test_busy_lock_at_refresh_is_not_a_failure(self):
        self.upstream({'b.txt': 'b2\n'})
        lk = os.path.join(self.co, '.git', 'index.lock')
        env = self.shim('update-index', f"touch '{lk}'; {{git}} \"$@\"; rc=$?; rm -f '{lk}'; exit $rc")
        r = subprocess.run([sys.executable, TOOL], cwd=self.co, capture_output=True, text=True, env=env)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_failed_fetch_is_refusal(self):
        g(self.co, 'remote', 'set-url', 'origin', os.path.join(self.tmp, 'gone.git'))
        self.assertEqual(self.run_tool().returncode, 2)


if __name__ == '__main__':
    unittest.main()
