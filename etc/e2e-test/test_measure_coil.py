#!/usr/bin/env python3
"""E2E test: on-device ``measure-coil`` vs the host ``etc/measure_coil.py``.

Both measure the same coil with the same DCM transfer relation
``L = (Vin-Vout)*Vin*D^2 / (2*Vout*fsw*Iout)`` — the host by parsing the streamed sensor average
over the console, the device by reading ``ewm.avg`` directly in firmware. This test runs both
against the **same physical converter under the same sun** and checks they agree, which validates
the firmware port against the (reference) script.

How it works
------------
Both the device command and the script drive the half-bridge and talk over the **one** telnet slot,
so they cannot overlap — the test runs them back-to-back with the control connection released in
between:

  1. ``etc/measure_coil.py`` as a subprocess (opens and closes its own connection); parse ``L = N uH``.
  2. open a ``FuguDevice``, send ``measure-coil l0 …``, watch the streamed result for ``L = N uH``,
     wait for ``done, MPPT restored``, close.
  3. assert ``|L_dev - L_host| / mean <= --tol`` (default 20% — each sweep has ~10-16% IQR and the
     sun drifts between the two runs).

With ``--with-ls`` it additionally runs the LS-timing sweep on both and compares ``peak LS`` and the
``peak-ideal`` offset (looser: the Iout-vs-LS peak is shallow/noisy, see doc §6).

This needs a **real converter with sun** (``Vin > Vout``), running firmware that has ``measure-coil``
(see website/docs/lab/coil-inductance.md). A no-sun / no-DCM run is reported SKIP, not FAIL — the
device prints ``need Vin>Vout`` / ``only N DCM pts`` and the script says the same. fugu139C and other
mock-ADC bench devices produce synthetic numbers; don't use them.

Usage
-----
    python etc/e2e-test/test_measure_coil.py --ip 192.168.4.2
    python etc/e2e-test/test_measure_coil.py --ip 192.168.1.173:232 --steps 14 --with-ls
    python etc/e2e-test/test_measure_coil.py --serial /dev/cu.usbmodem1201 --tol 0.25
"""
import argparse
import os
import re
import subprocess
import sys
import threading
import time

ETC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # repo/etc (holds the fugu pkg)
REPO_DIR = os.path.dirname(ETC_DIR)
sys.path.insert(0, ETC_DIR)
from fugu.transport import SerialTransport, SocketTransport
from fugu.fugu import FuguDevice
from _harness import Results, wait_for

_L_RE = re.compile(r"L = ([\d.]+)\s*uH")                       # both: "L = 49.69 uH"
_PEAK_RE = re.compile(r"peak LS\s*:?\s*(\d+)")                 # host "peak LS  : 553", dev "peak LS 560"
_OFFSET_RE = re.compile(r"peak-ideal\s*([+-]?\d+)\s*ct")       # both: "peak-ideal +87 ct"
# the device/script print these when the operating point can't support the measurement:
_SKIP_MARKERS = ("need Vin>Vout", "need Vin > Vout", "empty duty band", "empty LS band",
                 "DCM pts above", "DCM points", "too few points", "no status lines", "need a DCM")


class Tap:
    """Collects every console line (ANSI-stripped by FuguDevice) with a monotonic timestamp."""

    def __init__(self):
        self._lock = threading.Lock()
        self.lines = []  # (t, text)

    def feed(self, line):
        with self._lock:
            self.lines.append((time.monotonic(), line))

    def since(self, t0):
        with self._lock:
            return [s for (t, s) in self.lines if t >= t0]


# --- host script (reference) -----------------------------------------------------------------

def host_conn_args(args):
    if args.ip:
        return ["--ip", args.ip]
    return ["-p", args.serial]


def run_host(args, mode, steps, dwell_s, hs):
    cmd = [sys.executable, "etc/measure_coil.py", "--yes", "--dwell", str(dwell_s)] + host_conn_args(args)
    if mode == "l0":
        cmd += ["--steps", str(steps)]
    else:
        cmd += ["--ls-sweep", "--hs", str(hs)]
    print(f"  host: {' '.join(cmd)}", flush=True)
    try:
        p = subprocess.run(cmd, cwd=REPO_DIR, capture_output=True, text=True, timeout=args.host_timeout)
    except subprocess.TimeoutExpired:
        return "", "timeout"
    out = p.stdout + "\n" + p.stderr
    return out, None


# --- device command --------------------------------------------------------------------------

def connect_device(args):
    if args.ip:
        host, _, port = args.ip.partition(":")
        return FuguDevice(SocketTransport(host, int(port) if port else SocketTransport.DEFAULT_PORT),
                          block=False)
    return FuguDevice(SerialTransport(args.serial), block=False)


def run_device(dev, tap, mode, steps_or_hs, dwell_ms, args):
    """Send measure-coil, wait for the streamed result, return the raw output text (or None on timeout)."""
    t0 = time.monotonic()
    arg1 = steps_or_hs if steps_or_hs else ""
    reply = dev.console.command(f"measure-coil {mode} {arg1} {dwell_ms}".strip(), timeout=6.0)
    if not reply.ok:
        return None, f"command rejected: {reply.text.strip()}"
    # the sweep streams over many seconds; wait for the terminal "done, MPPT restored" line.
    deadline = (steps_or_hs or 24) * (dwell_ms / 1000.0 + 1.5) + 60
    got = wait_for(lambda: any("done, MPPT restored" in s for s in tap.since(t0)),
                   min(max(deadline, 90), args.dev_timeout))
    text = "\n".join(tap.since(t0))
    return text, (None if got else "no 'done, MPPT restored' before timeout")


def parse_l(text):
    m = _L_RE.search(text or "")
    return float(m.group(1)) if m else None


def is_skip(text):
    return any(mk in (text or "") for mk in _SKIP_MARKERS)


# --- tests -----------------------------------------------------------------------------------

def test_l0(args, res):
    print(f"\n--- L0: device vs host (steps={args.steps}, dwell={args.dwell_ms}ms) ---", flush=True)

    host_out, err = run_host(args, "l0", args.steps, args.dwell_ms / 1000.0, 0)
    if err:
        return res.skip("L0 device~=host", f"host run failed: {err}")
    L_host = parse_l(host_out)
    if L_host is None:
        return res.skip("L0 device~=host", "host produced no L (no sun/DCM?)" if is_skip(host_out)
                        else "could not parse host L")
    print(f"  host L = {L_host:.2f} uH", flush=True)

    time.sleep(args.gap)  # let the host release the slot and MPPT re-settle

    dev = connect_device(args)
    tap = Tap()
    dev.on_message = tap.feed
    try:
        if not wait_for(lambda: dev.pwm_state.ccm is not None, 20):
            return res.skip("L0 device~=host", "device not responding on the control console")
        dev_out, err = run_device(dev, tap, "l0", args.steps, args.dwell_ms, args)
    finally:
        dev.close()
    if err and parse_l(dev_out) is None:
        return res.skip("L0 device~=host", f"device run failed: {err}")
    L_dev = parse_l(dev_out)
    if L_dev is None:
        return res.skip("L0 device~=host", "device produced no L (no sun/DCM?)" if is_skip(dev_out)
                        else "could not parse device L")
    print(f"  device L = {L_dev:.2f} uH", flush=True)

    if not res.check("L0 plausible (1..500 uH)", 1.0 <= L_dev <= 500.0 and 1.0 <= L_host <= 500.0,
                     f"dev={L_dev:.1f} host={L_host:.1f}"):
        return
    rel = abs(L_dev - L_host) / (0.5 * (L_dev + L_host))
    res.check(f"L0 device~=host within {args.tol*100:.0f}%", rel <= args.tol,
              f"dev={L_dev:.2f} host={L_host:.2f} uH  ({rel*100:.1f}% apart)")


def test_ls(args, res):
    print(f"\n--- LS: device vs host (hs={args.hs}, dwell={args.dwell_ms}ms) ---", flush=True)

    host_out, err = run_host(args, "ls", 0, args.dwell_ms / 1000.0, args.hs)
    if err:
        return res.skip("LS peak/offset device~=host", f"host run failed: {err}")
    pk_h = _PEAK_RE.search(host_out)
    off_h = _OFFSET_RE.search(host_out)
    if not (pk_h and off_h):
        return res.skip("LS peak/offset device~=host",
                        "host found no peak (no sun/DCM?)" if is_skip(host_out) else "could not parse host peak")
    peak_host, off_host = int(pk_h.group(1)), int(off_h.group(1))
    print(f"  host peak LS={peak_host} offset={off_host:+d} ct", flush=True)

    time.sleep(args.gap)

    dev = connect_device(args)
    tap = Tap()
    dev.on_message = tap.feed
    try:
        if not wait_for(lambda: dev.pwm_state.ccm is not None, 20):
            return res.skip("LS peak/offset device~=host", "device not responding")
        dev_out, err = run_device(dev, tap, "ls", args.hs, args.dwell_ms, args)
    finally:
        dev.close()
    pk_d = _PEAK_RE.search(dev_out or "")
    off_d = _OFFSET_RE.search(dev_out or "")
    if not (pk_d and off_d):
        return res.skip("LS peak/offset device~=host",
                        "device found no peak (no sun/DCM?)" if is_skip(dev_out) else "could not parse device peak")
    peak_dev, off_dev = int(pk_d.group(1)), int(off_d.group(1))
    print(f"  device peak LS={peak_dev} offset={off_dev:+d} ct", flush=True)

    # the peak is shallow (doc §6), so allow a generous count tolerance.
    res.check(f"LS peak within {args.ls_tol} counts", abs(peak_dev - peak_host) <= args.ls_tol,
              f"dev={peak_dev} host={peak_host} ({abs(peak_dev - peak_host)} apart)")
    res.check(f"LS offset within {args.ls_tol} counts", abs(off_dev - off_host) <= args.ls_tol,
              f"dev={off_dev:+d} host={off_host:+d} ({abs(off_dev - off_host)} apart)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--ip", help="TCP/telnet host[:port] (e.g. 192.168.4.2 or 192.168.1.173:232)")
    g.add_argument("--serial", metavar="DEV", help="serial port")
    ap.add_argument("--steps", type=int, default=14, help="L0 duty steps (both tools)")
    ap.add_argument("--hs", type=int, default=300, help="[--with-ls] high-side count to hold")
    ap.add_argument("--dwell-ms", type=int, default=4000, help="per-step settle, ms (host gets ms/1000 s)")
    ap.add_argument("--tol", type=float, default=0.20, help="max relative L0 disagreement (default 0.20)")
    ap.add_argument("--ls-tol", type=int, default=40, help="[--with-ls] max peak/offset disagreement, counts")
    ap.add_argument("--with-ls", action="store_true", help="also compare the LS-timing/rect_offset sweep")
    ap.add_argument("--gap", type=float, default=15.0, help="seconds between the two runs (slot release + settle)")
    ap.add_argument("--host-timeout", type=float, default=240.0, help="host subprocess timeout (s)")
    ap.add_argument("--dev-timeout", type=float, default=240.0, help="device sweep wait timeout (s)")
    args = ap.parse_args()

    print(f"target={'ip ' + args.ip if args.ip else 'serial ' + args.serial}  steps={args.steps} "
          f"dwell={args.dwell_ms}ms tol={args.tol:.0%}", flush=True)
    res = Results()
    test_l0(args, res)
    if args.with_ls:
        test_ls(args, res)

    if not res.items:
        print("\nNO CHECKS RAN (all skipped — needs a real converter with sun)")
        return 0
    print("\n" + ("ALL CHECKS PASSED" if res.ok() else "SOME CHECKS FAILED"))
    return 0 if res.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
