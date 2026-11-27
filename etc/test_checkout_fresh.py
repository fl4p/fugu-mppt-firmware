#!/usr/bin/env python3
"""Host tests for etc/checkout_fresh.py and its use in fleet-build.sh and ota_mqtt.py; throwaway git repos.
Run: .venv/bin/python3 -m pytest etc/test_checkout_fresh.py"""
import os
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from types import SimpleNamespace as NS

ETC = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ETC)
import checkout_fresh as C  # noqa: E402

GIT_ENV = dict(GIT_AUTHOR_NAME='t', GIT_AUTHOR_EMAIL='t@t', GIT_COMMITTER_NAME='t', GIT_COMMITTER_EMAIL='t@t',
               GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM='1')


def g(repo, *a):
    return subprocess.run(['git', '-C', repo, '-c', 'protocol.file.allow=always', *a], check=True,
                          capture_output=True, text=True, env={**os.environ, **GIT_ENV}).stdout.strip()


def commit(repo, path, text='x'):
    p = os.path.join(repo, path)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, 'a') as f:
        f.write(text + '\n')
    g(repo, 'add', path)
    g(repo, 'commit', '-qm', path)
    return g(repo, 'rev-parse', 'HEAD')


class Repos(unittest.TestCase):
    def setUp(self):
        os.environ.update(GIT_ENV)
        self.tmp = tempfile.mkdtemp()
        self.up = os.path.join(self.tmp, 'up')     # the "origin" working repo
        self.bare = os.path.join(self.tmp, 'origin.git')
        self.co = os.path.join(self.tmp, 'co')     # the checkout under test
        os.makedirs(self.up)
        g(self.up, 'init', '-q', '-b', 'main')
        commit(self.up, 'etc/ota_mqtt.py')
        g(self.tmp, 'clone', '-q', '--bare', self.up, self.bare)
        g(self.tmp, 'clone', '-q', self.bare, self.co)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def push_upstream(self, path):
        commit(self.up, path)
        g(self.up, 'push', '-q', self.bare, 'main')

    def stale(self):
        return C.stale_tools(self.co, C.compare(self.co))


class Compare(Repos):
    def test_current_checkout_is_fresh(self):
        self.assertEqual(self.stale(), [])
        c = C.compare(self.co)
        self.assertEqual((c['ahead'], c['behind'], c['fetch_error']), (0, 0, None))

    def test_behind_in_tool_paths_is_stale(self):
        self.push_upstream('etc/ota_mqtt.py')
        why = self.stale()
        self.assertEqual(len(why), 1)
        self.assertIn('1 commits behind', why[0])
        self.assertIn('etc/ota_mqtt.py', why[0])

    def test_far_behind_still_stale(self):
        for _ in range(5):
            self.push_upstream('doc/x.md')
        self.push_upstream('etc/fugu_console.py')
        self.assertIn('6 commits behind', self.stale()[0])

    def test_behind_in_other_paths_only_is_not_stale(self):
        self.push_upstream('doc/x.md')
        self.assertEqual(self.stale(), [])
        self.assertEqual(C.compare(self.co)['behind'], 1)

    def test_failed_fetch_uses_cached_upstream_and_says_so(self):
        self.push_upstream('etc/ota_mqtt.py')
        C.compare(self.co)                        # cache the new origin/main
        g(self.co, 'remote', 'set-url', 'origin', os.path.join(self.tmp, 'gone.git'))
        c = C.compare(self.co)
        self.assertTrue(c['fetch_error'])
        self.assertEqual(c['behind'], 1)
        self.assertIn('cached origin/main', C.provenance(c))
        self.assertTrue(self.stale())

    def test_unknown_rev_or_upstream_is_unverified(self):
        self.assertIsNone(C.compare(self.co, 'deadbeef', do_fetch=False))
        g(self.co, 'update-ref', '-d', 'refs/remotes/origin/main')
        g(self.co, 'remote', 'set-url', 'origin', os.path.join(self.tmp, 'gone.git'))
        self.assertIsNone(C.compare(self.co))
        self.assertIsNone(C.stale_tools(self.co, None))


class Submodules(Repos):
    def setUp(self):
        super().setUp()
        self.sub = os.path.join(self.tmp, 'sub')
        os.makedirs(self.sub)
        g(self.sub, 'init', '-q', '-b', 'master')
        self.a = commit(self.sub, 'a')
        self.b = commit(self.sub, 'b')
        g(self.co, 'submodule', 'add', '-q', self.sub, 'etc/idf-devtools')
        g(self.co, 'commit', '-qm', 'sub')        # pins b

    def test_at_pin_is_fresh(self):
        self.assertEqual(C.submodules_behind(self.co), [])

    def test_behind_pin_is_reported(self):
        g(os.path.join(self.co, 'etc/idf-devtools'), 'checkout', '-q', self.a)
        self.assertEqual(C.submodules_behind(self.co), [('etc/idf-devtools', self.a, self.b)])
        self.assertIn('behind its pin', self.stale()[-1])

    def test_ahead_of_pin_is_not_stale(self):
        sub = os.path.join(self.co, 'etc/idf-devtools')
        g(sub, 'checkout', '-q', 'master')
        commit(sub, 'c')
        self.assertEqual(C.submodules_behind(self.co), [])

    def test_uninitialized_submodule_is_unverified(self):
        clone = os.path.join(self.tmp, 'clone')
        g(self.co, 'push', '-q', 'origin', 'HEAD:main')
        g(self.tmp, 'clone', '-q', self.bare, clone)      # no submodule update
        self.assertIsNone(C.submodules_behind(clone))
        self.assertIsNone(C.stale_tools(clone, C.compare(clone)))

    def test_unfetched_pin_is_reported(self):
        sub = os.path.join(self.co, 'etc/idf-devtools')
        g(sub, 'checkout', '-q', self.a)
        pin = commit(self.sub, 'd')              # a pin the checkout's submodule never fetched
        g(self.co, 'update-index', '--cacheinfo', f'160000,{pin},etc/idf-devtools')
        g(self.co, 'commit', '-qm', 'bump')
        self.assertEqual(C.submodules_behind(self.co), [('etc/idf-devtools', self.a, pin)])


class FleetBuild(Repos):
    def setUp(self):
        super().setUp()
        for f in ('fleet-build.sh', 'checkout_fresh.py'):
            shutil.copy(os.path.join(ETC, f), os.path.join(self.co, 'etc', f))
        g(self.co, 'add', 'etc')
        g(self.co, 'commit', '-qm', 'tools')
        g(self.co, 'push', '-q', 'origin', 'HEAD:main')
        g(self.up, 'pull', '-q', '--ff-only', self.bare, 'main')
        self.wt = os.path.join(self.tmp, 'fleet-wt')

    def run_build(self, *a):
        env = {**os.environ, 'FLEET_WT': self.wt, 'LOCK_SH': '/nonexistent', 'PATH': os.environ['PATH']}
        return subprocess.run(['bash', os.path.join(self.co, 'etc/fleet-build.sh'), *a], capture_output=True,
                              text=True, env=env)

    def test_default_head_behind_origin_is_refused_before_checkout(self):
        self.push_upstream('src/x.cpp')
        r = self.run_build()
        self.assertEqual(r.returncode, 1, r.stderr)
        self.assertIn('refusing', r.stderr)
        self.assertFalse(os.path.exists(self.wt))

    def test_named_old_rev_only_warns(self):
        self.push_upstream('src/x.cpp')
        old = g(self.co, 'rev-parse', 'HEAD')
        r = self.run_build(old)
        self.assertIn('WARNING', r.stderr)
        self.assertTrue(os.path.exists(self.wt), r.stderr)  # went on to the worktree (the build itself fails)


class OtaMqtt(Repos):
    """tools_fresh() as main() calls it, against a throwaway REPO_ROOT."""

    @classmethod
    def setUpClass(cls):
        sys.modules.setdefault("serial", types.ModuleType("serial"))
        import ota_mqtt
        cls.M = ota_mqtt

    def check(self, dry=False, allow=False, version='v1-g0000000'):
        args = NS(dry_run=dry, allow_stale_tools=allow, would_refuse=False)
        old, self.M.REPO_ROOT = self.M.REPO_ROOT, self.co
        try:
            return self.M.tools_fresh(args, {'version': version}), args.would_refuse
        finally:
            self.M.REPO_ROOT = old

    def test_fresh_passes(self):
        self.assertEqual(self.check(), (True, False))

    def test_stale_refused_real_and_dry(self):
        self.push_upstream('etc/ota-via-relay.sh')
        self.assertEqual(self.check(), (False, False))
        self.assertEqual(self.check(dry=True), (True, True))
        self.assertEqual(self.check(allow=True), (True, False))

    def test_unverifiable_is_refused(self):
        g(self.co, 'update-ref', '-d', 'refs/remotes/origin/main')
        g(self.co, 'remote', 'set-url', 'origin', os.path.join(self.tmp, 'gone.git'))
        self.assertEqual(self.check(), (False, False))
        self.assertEqual(self.check(allow=True), (True, False))

    def test_main_runs_the_check(self):
        src = open(os.path.join(ETC, 'ota_mqtt.py')).read()
        main = src[src.index('def main():'):]
        self.assertLess(main.index('tools_fresh(args, image)'), main.index('broker_endpoint('))


if __name__ == '__main__':
    unittest.main()
