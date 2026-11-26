#!/usr/bin/env python3
"""Host test for fugu_console.Tunnel with a fake `ssh` on PATH: a failure names ssh's exit code and stderr,
a transient failure is retried, and stderr of a live tunnel is shown on close. Run with `python3 etc/test_console_tunnel.py`."""
import os
import stat
import sys
import tempfile
import time
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.modules.setdefault("serial", types.ModuleType("serial"))

import contextlib  # noqa: E402
import io  # noqa: E402

import fugu_console as fc  # noqa: E402

FAKE = r'''#!{py}
import os, socket, sys, time
cnt = os.environ["FAKE_SSH_COUNT"]
n = int(open(cnt).read() or 0) + 1 if os.path.exists(cnt) else 1
open(cnt, "w").write(str(n))
if n <= int(os.environ["FAKE_SSH_FAIL"]):
    sys.stderr.write("kex_exchange_identification: Connection closed by remote host\n")
    sys.exit(255)
port = int(sys.argv[sys.argv.index("-L") + 1].split(":")[0])
s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("127.0.0.1", port)); s.listen(4)
sys.stderr.write("channel 2: open failed: connect failed\n"); sys.stderr.flush()
time.sleep(30)
'''


def run(fail):
    d = tempfile.mkdtemp()
    ssh = os.path.join(d, "ssh")
    with open(ssh, "w") as f:
        f.write(FAKE.format(py=sys.executable))
    os.chmod(ssh, os.stat(ssh).st_mode | stat.S_IEXEC)
    env = dict(os.environ)
    os.environ["PATH"] = d + os.pathsep + env["PATH"]
    os.environ["FAKE_SSH_COUNT"] = os.path.join(d, "count")
    os.environ["FAKE_SSH_FAIL"] = str(fail)
    try:
        t = fc.Tunnel("fake-host", "10.0.0.1", 1882, timeout=5)
        time.sleep(0.5)
        buf = io.StringIO()
        with contextlib.redirect_stderr(buf):
            t.close()
        assert "ssh: channel 2: open failed" in buf.getvalue(), buf.getvalue()  # stderr of a live tunnel
        return None, int(open(os.path.join(d, "count")).read())
    except RuntimeError as e:
        return str(e), int(open(os.path.join(d, "count")).read())
    finally:
        os.environ.clear()
        os.environ.update(env)


def main():
    err, n = run(fail=1)
    assert err is None and n == 2, (err, n)  # one transient failure, retried
    err, n = run(fail=5)
    assert err and n == 2, (err, n)  # gives up after 2 attempts
    assert "ssh exited 255" in err and "kex_exchange_identification" in err, err
    print("ok")


if __name__ == "__main__":
    main()
