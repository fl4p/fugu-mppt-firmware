#!/usr/bin/env python3
"""E2E test: the panel (backflow) switch conducts when closed.

With the switch open, input current flows through the MOSFET body diode instead of its channel,
so the drop across the switch rises from ~I*Rds(on) to ~0.6 V. At a fixed duty that lower
converter input voltage shows up at the output: Vout and Iout into a resistive load must fall.

How it works
------------
1. Enter manual PWM (`dc 0`), close the switch and ramp the duty slowly until the load sits at
   ``--vout`` (default 2 V). The ramp stops at ``--d-max`` (default 0.30, below the firmware's
   0.33 open-switch duty trip) and aborts above ``--i-max``.
2. Hold that duty and toggle the switch C O C O ... C. Every open phase is bracketed by two closed
   phases, so a slow drift (a lamp warming up) cannot fake or mask the step.
3. Per phase: settle ``--settle`` s, then average the live status lines for ``--meas`` s.
4. PASS iff, for every open phase, Vout and Iout are below BOTH neighbouring closed phases by more
   than max(absolute floor, 3 sigma of the difference).

A phase is unevaluable (FAIL, never PASS) when it has too few fresh status lines, a line reports
nan / a disabled converter / a non-manual mode, the firmware toggles the switch on its own, or a
commanded toggle is not confirmed by the firmware's "Back-flow switch ..." message. Any shutdown,
backoff, reverse-current or panic line aborts the run. The converter is always parked at `dc 0`.

Needs a converter with Vin > Vout and a resistive load (a lamp) on the output. Do not run it
against a battery or a supply on the output.

Usage
-----
    python etc/e2e-test/test_panel_switch.py --ble fugu-fblack
    python etc/e2e-test/test_panel_switch.py --serial /dev/cu.usbmodem1101 --vout 2.0 --cycles 3
"""
import argparse
import math
import os
import re
import statistics
import sys
import threading
import time

ETC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ETC_DIR)
from fugu.console import Console
from fugu.transport import BleTransport, SerialTransport, SocketTransport
from _harness import PANIC_MARKERS, Results

_F = r"(-?\d+(?:\.\d+)?|nan)"
RE_STATUS = re.compile(
    rf"V=\s*{_F}\s*/\s*{_F}\s+I=\s*{_F}\s*/\s*{_F}A\s.*?st=\s*(\w+),(\d).*?\bN=(\d+)")
RE_BFLOW = re.compile(r"Back-flow switch (enabled|disabled)")
RE_PWMMAX = re.compile(r"\bpwmMax=(\d+)")
RE_HSOFF = re.compile(r"\bhs_off=(\d+)")
ABORT_MARKERS = ("shutdown", "backoff", "Reverse", "bflowOpen", "ADC stall") + PANIC_MARKERS


class Tap:
    """Timestamped console lines plus the parsed status samples."""

    def __init__(self):
        self._lock = threading.Lock()
        self.lines = []    # (t, line)
        self.samples = []  # (t, dict)

    def __call__(self, line):
        t = time.monotonic()
        with self._lock:
            self.lines.append((t, line))
            m = RE_STATUS.search(line)
            if m:
                vin, vout, _, iout, st, en, n = m.groups()
                self.samples.append((t, dict(vin=float(vin), vout=float(vout), iout=float(iout),
                                             st=st, en=int(en), n=int(n))))

    def lines_since(self, t0):
        with self._lock:
            return [(t, l) for (t, l) in self.lines if t >= t0]

    def samples_between(self, t0, t1):
        with self._lock:
            return [s for (t, s) in self.samples if t0 <= t < t1]

    def last_sample(self, since):
        with self._lock:
            s = [s for (t, s) in self.samples if t >= since]
        return s[-1] if s else None


class Abort(Exception):
    pass


def open_console(args, tap):
    if args.ble:
        tr = BleTransport(name=args.ble)
    elif args.ip:
        host, _, port = args.ip.partition(":")
        tr = SocketTransport(host, int(port) if port else SocketTransport.DEFAULT_PORT)
    else:
        tr = SerialTransport(args.serial)
    return Console(tr, eol="\n", on_line=tap, wait_banner=bool(args.ip))


def valid(s):
    return (s is not None and s["st"] == "MANU" and s["en"] == 1
            and all(math.isfinite(s[k]) for k in ("vin", "vout", "iout")))


def check_abort(tap, since):
    for _, l in tap.lines_since(since):
        if any(m in l for m in ABORT_MARKERS) and not RE_STATUS.search(l):
            raise Abort(l.strip())


def cmd(con, c, timeout=6.0):
    r = con.command(c, timeout=timeout)
    if not r.ok:
        raise Abort(f"'{c}' rejected: {r.text.strip()[:200]}")
    return r


def fresh_sample(tap, since, timeout=15.0):
    """First valid status line strictly after `since` (a replayed or pre-change line is stale)."""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        s = tap.last_sample(since)
        if valid(s):
            return s
        time.sleep(0.2)
    raise Abort("no fresh valid status line (converter not running in manual mode?)")


def set_bf(con, tap, state, confirm=True):
    """Command the switch; with `confirm`, require the firmware's report of the transition."""
    t0 = time.monotonic()
    cmd(con, f"bf {int(state)}")
    want = "enabled" if state else "disabled"
    t_end = t0 + 3.0
    while time.monotonic() < t_end:
        for _, l in tap.lines_since(t0):
            m = RE_BFLOW.search(l)
            if m:
                if m.group(1) != want:
                    raise Abort(f"bf {int(state)}: firmware reported '{l.strip()}'")
                return
        time.sleep(0.1)
    if confirm:
        raise Abort(f"bf {int(state)}: no 'Back-flow switch {want}' confirmation")


def ramp_to_vout(con, tap, args, pwm_max):
    t0 = time.monotonic()
    cmd(con, "dc 0")
    time.sleep(1.5)
    s0 = tap.last_sample(t0)
    vin = s0["vin"] if s0 and math.isfinite(s0["vin"]) else float("nan")
    if not (vin > args.vout + 1.0):
        raise Abort(f"need Vin > Vout+1 V (Vin={vin})")
    d_max_cnt = int(args.d_max * pwm_max)
    step = max(4, round(pwm_max * args.step))
    cnt = max(1, round(pwm_max * 0.5 * args.vout / vin))
    first = True
    while True:
        if cnt > d_max_cnt:
            raise Abort(f"reached D={args.d_max} without Vout>={args.vout} V (load missing?)")
        t = time.monotonic()
        cmd(con, f"dc {cnt}")
        if first:
            set_bf(con, tap, False, confirm=False)  # may already be open
            set_bf(con, tap, True)
            first = False
        time.sleep(args.ramp_dwell)
        check_abort(tap, t)
        s = fresh_sample(tap, t + 0.5)
        print(f"  ramp dc={cnt} D={cnt / pwm_max:.3f}  Vin={s['vin']:.2f} Vout={s['vout']:.3f} "
              f"Iout={s['iout']:.3f}", flush=True)
        if s["iout"] > args.i_max:
            raise Abort(f"Iout {s['iout']:.2f} A > --i-max {args.i_max}")
        if s["vout"] >= args.vout:
            return cnt
        cnt += step


def measure_phase(con, tap, state, args, confirm=True):
    t0 = time.monotonic()
    set_bf(con, tap, state, confirm)
    t1 = t0 + args.settle
    t2 = t1 + args.meas
    while time.monotonic() < t2:
        check_abort(tap, t0)
        time.sleep(0.2)
    check_abort(tap, t0)
    for _, l in tap.lines_since(t0 + 0.5):
        m = RE_BFLOW.search(l)
        if m and (m.group(1) == "enabled") != state:
            raise Abort(f"switch toggled by firmware during phase: '{l.strip()}'")
    ss = tap.samples_between(t1, t2)
    bad = [s for s in ss if not valid(s)]
    if bad:
        raise Abort(f"unevaluable status line in phase: {bad[0]}")
    if len(ss) < args.min_samples:
        raise Abort(f"only {len(ss)} status lines in {args.meas}s (need {args.min_samples})")
    ns = [s["n"] for s in ss]
    if ns != sorted(ns) or len(set(ns)) != len(ns):
        raise Abort("status-line sample counter N not strictly increasing (replayed/stale lines)")
    out = {}
    for k in ("vin", "vout", "iout"):
        v = [s[k] for s in ss]
        out[k] = (statistics.fmean(v), statistics.stdev(v) / math.sqrt(len(v)))
    out["n"] = len(ss)
    return out


def park(con, args, tap):
    """`dc 0`, reconnecting once if the link dropped: never leave the converter switching."""
    for attempt in range(2):
        try:
            if con.command("dc 0", timeout=6.0).ok:
                print("  parked: dc 0", flush=True)
                break
        except ConnectionError:
            pass
        finally:
            con.close()
        if attempt == 0:
            print("  link down, reconnecting to park the converter", flush=True)
            try:
                con = open_console(args, tap)
            except Exception as e:
                print(f"  reconnect failed: {e}", flush=True)
                break
    else:
        print("  WARNING: could not confirm dc 0 — converter state unknown", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--serial", metavar="DEV")
    g.add_argument("--ip", help="TCP/telnet host[:port]")
    g.add_argument("--ble", metavar="NAME", help="BLE name, e.g. fugu-fblack")
    ap.add_argument("--vout", type=float, default=2.0, help="load voltage to hold (V)")
    ap.add_argument("--d-max", type=float, default=0.30, help="duty ceiling of the ramp")
    ap.add_argument("--i-max", type=float, default=4.0, help="abort above this Iout (A)")
    ap.add_argument("--step", type=float, default=0.002, help="ramp step, fraction of pwmMax")
    ap.add_argument("--ramp-dwell", type=float, default=2.0, help="seconds per ramp step")
    ap.add_argument("--cycles", type=int, default=3, help="open phases (each bracketed by closed)")
    ap.add_argument("--settle", type=float, default=3.0, help="seconds after a toggle before averaging")
    ap.add_argument("--meas", type=float, default=12.0,
                    help="seconds averaged per phase (status lines come every ~3 s)")
    ap.add_argument("--min-samples", type=int, default=3, help="status lines required per phase")
    ap.add_argument("--min-dv", type=float, default=0.005, help="absolute Vout drop floor (V)")
    ap.add_argument("--min-di", type=float, default=0.002, help="absolute Iout drop floor (A)")
    args = ap.parse_args()
    if not 0 < args.d_max < 0.33:
        ap.error("--d-max must be in (0, 0.33): above 0.33 the firmware trips on an open switch")

    res = Results()
    tap = Tap()
    con = open_console(args, tap)
    phases = []
    try:
        r = cmd(con, "pwm-dump")
        m = RE_PWMMAX.search(r.text)
        if not m:
            raise Abort("pwm-dump reported no pwmMax")
        pwm_max = int(m.group(1))
        r = cmd(con, "sync")
        if "engaged" in r.text or "ungated" in r.text:
            raise Abort(f"forced PWM is engaged ({r.text.strip()}); run with diode emulation")

        print(f"\n--- ramp to Vout={args.vout} V (pwmMax={pwm_max}, D<={args.d_max}) ---", flush=True)
        cnt = ramp_to_vout(con, tap, args, pwm_max)
        r = cmd(con, "pwm-dump")
        m = RE_HSOFF.search(r.text)
        if not m or int(m.group(1)) != cnt:
            raise Abort(f"pwm-dump hs_off does not match commanded dc {cnt}: {r.text.strip()}")

        print(f"\n--- toggle at dc={cnt} (D={cnt / pwm_max:.3f}) ---", flush=True)
        for i in range(2 * args.cycles + 1):
            state = i % 2 == 0
            p = measure_phase(con, tap, state, args, confirm=i > 0)  # ramp left it closed
            phases.append((state, p))
            print(f"  {'closed' if state else 'open  '}  Vin={p['vin'][0]:.3f}  "
                  f"Vout={p['vout'][0]:.4f}±{p['vout'][1]:.4f}  Iout={p['iout'][0]:.4f}±{p['iout'][1]:.4f}"
                  f"  (n={p['n']})", flush=True)
    except (Abort, ConnectionError) as e:
        res.check("run completed", False, f"{type(e).__name__}: {e}")
        for _, l in tap.lines_since(0)[-15:]:
            print(f"    | {l}", flush=True)
    finally:
        park(con, args, tap)

    if len(phases) == 2 * args.cycles + 1:
        for i in range(1, len(phases), 2):
            before, op, after = phases[i - 1][1], phases[i][1], phases[i + 1][1]
            for k, floor, unit in (("vout", args.min_dv, "V"), ("iout", args.min_di, "A")):
                drops = [c[k][0] - op[k][0] for c in (before, after)]
                sig = max(math.hypot(c[k][1], op[k][1]) for c in (before, after))
                need = max(floor, 3 * sig)
                res.check(f"open #{i // 2 + 1}: {k} below both closed neighbours", min(drops) > need,
                          f"drops {drops[0] * 1e3:+.1f}/{drops[1] * 1e3:+.1f} m{unit}, need >{need * 1e3:.1f}")
        cl = [p for s, p in phases if s]
        opn = [p for s, p in phases if not s]
        dvin = statistics.fmean(p["vin"][0] for p in cl) - statistics.fmean(p["vin"][0] for p in opn)
        dvout = statistics.fmean(p["vout"][0] for p in cl) - statistics.fmean(p["vout"][0] for p in opn)
        print(f"\n  mean drop when open: Vout {dvout * 1e3:.1f} mV, Vin sensor {dvin * 1e3:.1f} mV", flush=True)

    print("\n" + ("ALL CHECKS PASSED" if res.ok() else "SOME CHECKS FAILED"))
    return 0 if res.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
