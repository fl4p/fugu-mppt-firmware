#!/usr/bin/env python3
"""Measure the buck inductor L without a current probe, using only Vin/Vout/Iout.

Drives the firmware console (`dc`, `get-config`, `sensor`, the streamed status line) over
serial/TCP/BLE via the `fugu` package. With the output on a battery (Vout clamped) the only
DC-observable that depends on L is the discontinuous-conduction-mode (DCM) transfer relation.
Running the converter at a series of low duty cycles (deep in DCM) and reading the averaged
sensors back, each point yields

    L = (Vin - Vout) * Vin * D**2 / (2 * Vout * fsw * Iout)        [buck, DCM, Vout clamped]

where D = pwmCtrl/period is the high-side on-fraction and fsw the switching frequency, both as
realized by the driver (read from the `pwm-dump` console command). This is the
inverse of the firmware's own rippleCurrent() (src/buck.h). We sweep duty upward from deep DCM
toward the CCM/DCM boundary, fit L across the DCM points, and report the median.

Caveats (see doc): result scales directly with the Iout calibration and with the PWM period; dead-time and
DCR bias L low by a few %. The reported L is the *physical* inductance — put it in coil.conf as L0
(the firmware re-applies the 0.95 InductivityDcBias itself).

Requires `pyserial` (serial) and/or `bleak` (BLE).

Examples:
    python etc/measure_coil.py --ip 192.168.4.2
    python etc/measure_coil.py -p /dev/cu.usbmodem1101 --steps 12 --i-max 1.5
    python etc/measure_coil.py --ip 192.168.4.2 --fsw 39000 --pwm-max 1024 --yes
    python etc/measure_coil.py --ip 192.168.4.2 --bidir   # sweep up+down, settle-check
    python etc/measure_coil.py --ip 192.168.4.2 --ls-sweep --hs 300   # LS-timing peak at HS=300
"""

import argparse
import os
import re
import statistics
import sys
import threading
import time

try:
    from fugu.transport import SerialTransport, SocketTransport, BleTransport
    from fugu.console import Console
except ImportError:
    from etc.fugu.transport import SerialTransport, SocketTransport, BleTransport
    from etc.fugu.console import Console

# Status line (src/main.cpp loopLF):
#   V=24.5/27.30 I= 1.2/ 1.50A   40.9W 25C40C 400sps 0/s DCM(H|L|Lm)= 240| 120| 130 st= MPPT,1 ...
_V_RE = re.compile(r"V=\s*(-?\d+\.?\d*)/\s*(-?\d+\.?\d*)")
_I_RE = re.compile(r"I=\s*(-?\d+\.?\d*)/\s*(-?\d+\.?\d*)A")
_HLM_RE = re.compile(r"(CCM|DCM)\(H\|L\|Lm\)=\s*(\d+)\|\s*(\d+)\|\s*(\d+)")
_RANGE_RE = re.compile(r"out of range \[0,(\d+)\]")
# `pwm-dump` (src/cli.cpp): realized frequency, timer period and the driver's pwmMax
_PWM_DUMP_RE = re.compile(r"freq=(\d+\.?\d*)\s+nominal=\d+\s+period_ticks=(\d+)\s+pwmMax=(\d+)")
_CONF_RE = re.compile(r"=\s*'([^']*)'\s*$")
# `sensor avg` (src/cli.cpp cmdSensor) prints one compact line: "sens: vin=24.5000 iout=1.2300 ...":
_SENS_LINE_RE = re.compile(r"sens:\s")
_SENS_PAIR_RE = re.compile(r"(\w+)=(-?\d+\.?\d*)")

MIN_DUTY_LS = 0.06  # src/buck.h MinDutyCycleLS -> pwmCtrlMax = pwmMax*(1-0.06) for a buck


class StatusTap:
    """Parses streamed status lines into the latest (vin, vout, iout, H, mode) snapshot."""

    def __init__(self):
        self._lock = threading.Lock()
        self.samples = []  # (t, vin, vout, iout, H, mode, rectLS)

    def on_line(self, line: str):
        mv, mi, mh = _V_RE.search(line), _I_RE.search(line), _HLM_RE.search(line)
        if not (mv and mi and mh):
            return
        snap = (time.monotonic(), float(mv.group(1)), float(mv.group(2)),
                float(mi.group(2)), int(mh.group(2)), mh.group(1), int(mh.group(3)))
        with self._lock:
            self.samples.append(snap)

    def collect(self, since: float):
        with self._lock:
            return [s for s in self.samples if s[0] >= since]


def median(xs):
    return statistics.median(xs) if xs else float("nan")


def ascii_plot(rows, width=58, height=15):
    """Print an L(uH)-vs-H scatter of the DCM rows collected so far."""
    pts = [(r[0], r[6] * 1e6) for r in rows if r[5] == "DCM" and r[6] == r[6]]
    if len(pts) < 2:
        return
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    if x1 == x0 or y1 == y0:
        return
    grid = [[" "] * width for _ in range(height)]
    for h, l in pts:
        cx = round((h - x0) / (x1 - x0) * (width - 1))
        cy = round((l - y0) / (y1 - y0) * (height - 1))
        grid[height - 1 - cy][cx] = "*"
    med = median(ys)
    my = height - 1 - round((med - y0) / (y1 - y0) * (height - 1))
    for c in range(width):
        if grid[my][c] == " ":
            grid[my][c] = "-"
    print(f"\n  L/uH        ({len(pts)} DCM pts, median {med:.1f} uH dashed)")
    for r in range(height):
        lab = y1 - (y1 - y0) * r / (height - 1)
        print(f"{lab:6.1f} |" + "".join(grid[r]))
    print("       +" + "-" * width)
    print(f"        {x0:<6}" + " " * (width - 12) + f"{x1:>6}   H (PWM ct)\n")


def connect(args):
    if args.ip:
        host, _, port = args.ip.partition(":")
        t = SocketTransport(host, int(port) if port else SocketTransport.DEFAULT_PORT)
        eol = "\n"  # telnet wants \n (CLAUDE.md)
    elif args.ble:
        t = BleTransport(name=args.ble if isinstance(args.ble, str) else "fugu")
        eol = "\r\n"
    else:
        port = args.port or os.environ.get("ESPPORT")
        if not port:
            sys.exit("no serial port; pass --port/--ip/--ble or set $ESPPORT")
        t = SerialTransport(port)
        eol = "\r\n"
    tap = StatusTap()
    return Console(t, eol=eol, on_line=tap.on_line), tap


def get_conf(con, file, key):
    for ln in con.command(f"get-config {file} {key}", timeout=4.0):
        m = _CONF_RE.search(ln)
        if m:
            return m.group(1)
    return None


def query_pwm_timing(con):
    """(realized fsw Hz, period ticks, driver pwmMax) from `pwm-dump`, or None. The period matches the
    firmware's getPwmTickRate() = fsw * period: MCPWM reports period_ticks; LEDC reports 0 there and
    counts time in pwmMax ticks."""
    for ln in con.command("pwm-dump", timeout=4.0):
        m = _PWM_DUMP_RE.search(ln)
        if m:
            period, pwm_max = int(m.group(2)), int(m.group(3))
            return float(m.group(1)), period or pwm_max, pwm_max
    return None


def query_pwm_ctrl_max(con):
    r = con.command("dc 999999999", timeout=4.0)
    for ln in r:
        m = _RANGE_RE.search(ln)
        if m:
            return int(m.group(1))
    return None


def read_sensor_avgs(con, timeout=1.0):
    """Parse the `sensor avg` compact line into {name: ewm.avg} — the firmware's
    notch+median+EWMA DC average (offset-corrected for current), one line, cheap to poll at
    ~10 Hz (unlike the status line, which prints raw `last` for Vin/Vout). {} on a transport
    error or if the line is absent."""
    try:
        lines = con.command("sensor avg", timeout=timeout)
    except Exception:
        return {}
    for ln in lines:
        if _SENS_LINE_RE.search(ln):
            return {k: float(v) for k, v in _SENS_PAIR_RE.findall(ln)}
    return {}


def steady_read(con, tap, dwell):
    """Hold the operating point for `dwell` s. Average Vin/Vout/Iout from the firmware EWMA
    (poll `sensor` across the dwell, median the snapshots — the status line only carries raw
    `last` voltages); take H/mode/rect from the status line (exact control state)."""
    t0 = time.monotonic()
    settle = min(2.0, 0.4 * dwell)  # let the EWMA catch up after the duty step
    deadline = t0 + dwell
    vins, vouts, iouts = [], [], []
    while time.monotonic() < deadline:
        a = read_sensor_avgs(con)
        if time.monotonic() >= t0 + settle and all(k in a for k in ("vin", "vout", "iout")):
            vins.append(a["vin"]); vouts.append(a["vout"]); iouts.append(a["iout"])
        time.sleep(0.1)  # ~10 Hz poll
    pts, waited = tap.collect(t0 + 0.5), 0.0
    while not pts and waited < 4.0:  # ensure >=1 status line for H/mode/rect (lfPeriod ~3 s)
        time.sleep(0.5); waited += 0.5
        pts = tap.collect(t0 + 0.5)
    if not (vins and pts):
        return None
    return dict(vin=median(vins), vout=median(vouts), iout=median(iouts),
                H=int(median([p[4] for p in pts])), mode=pts[-1][5],
                rect=int(median([p[6] for p in pts])), n=len(vins))


RECOVER_BUDGET = 120.0  # seconds to keep retrying a dropped console before giving up


def safe_command(con, cmd, timeout=4.0):
    """con.command() with the lib's fast retries plus a long reconnect budget, reduced to a bool.

    Returns True if the command went through, False if the device stays unreachable (caller should
    end the run gracefully on the data collected so far)."""
    try:
        con.command(cmd, timeout=timeout, retry=True, recover=RECOVER_BUDGET)
        return True
    except Exception as e:
        print(f"  device unreachable on {cmd!r}: {e}")
        return False


def run_sweep(con, tap, hvals, period, fsw, i_max, dwell, label="", plot_every=0):
    """Step the duty over `hvals`, measure L at each, return (rows, last_safe_H).

    Each row is (H, D, vin, vout, iout, mode, L, label). Stops on Iout exceeding `i_max` or on
    CCM; `last_safe_H` is the highest in-limit DCM point reached (the start point for a reverse
    sweep). `label` tags rows and prints as a 1-char direction marker.
    """
    rows, safe_h = [], None
    for H in hvals:
        if not safe_command(con, f"dc {H}"):
            print(f"  device unreachable at H={H}; ending {label} sweep with data so far")
            break
        s = steady_read(con, tap, dwell)
        if not s:
            continue
        D = s["H"] / period
        vi, vo, io = s["vin"], s["vout"], s["iout"]
        L = (float("nan") if io <= 0.02 or vi <= vo
             else (vi - vo) * vi * D * D / (2.0 * vo * fsw * io))
        rows.append((s["H"], D, vi, vo, io, s["mode"], L, label))
        luh = "" if L != L else f"{L * 1e6:7.2f}"
        print(f"  {label[:1] or ' '}{s['H']:>5} {D:>6.3f} {vi:>6.2f} {vo:>6.2f} {io:>6.3f} "
              f"{s['mode']:>4} {luh:>7}")
        if plot_every and len(rows) % plot_every == 0:
            ascii_plot(rows)
        if io > i_max:
            print(f"  Iout {io:.2f} > i-max {i_max}; stopping {label} sweep")
            break
        if s["mode"] == "CCM":
            print(f"  entered CCM; stopping {label} sweep")
            break
        safe_h = s["H"]
    return rows, safe_h


def bidir_verdict(rows, ifloor):
    """Compare up- vs down-sweep L at matched duties to separate settling lag from physics."""
    sel = lambda lab: {r[0]: r[6] for r in rows
                       if r[7] == lab and r[5] == "DCM" and r[6] == r[6] and r[4] >= ifloor}
    up, dn = sel("up"), sel("dn")
    common = sorted(set(up) & set(dn))
    print()
    if len(common) < 3:
        print(f"bidir: only {len(common)} matched duties — can't judge (run wider/slower)")
        return
    diffs = [dn[h] - up[h] for h in common]
    med = median([up[h] for h in common] + [dn[h] for h in common])
    signed, absol = sum(diffs) / len(diffs), sum(abs(d) for d in diffs) / len(diffs)
    print(f"bidir settle-check: {len(common)} matched duties")
    print(f"  up->dn bias  : {signed * 1e6:+6.2f} uH ({signed / med * 100:+.0f}%)  "
          f"[lag if consistent sign]")
    print(f"  mean |up-dn| : {absol * 1e6:6.2f} uH ({absol / med * 100:.0f}%)")
    worst = max(common, key=lambda h: abs(dn[h] - up[h]))
    print(f"  worst H={worst}: up={up[worst] * 1e6:.1f} dn={dn[worst] * 1e6:.1f} uH")
    if abs(signed) / med > 0.05:
        print("  -> consistent direction bias = NOT settled; raise --dwell / lower step")
    elif absol / med > 0.05:
        print("  -> up/dn disagree without a consistent sign = noisy/under-settled")
    else:
        print("  -> up~=dn at each duty: any wiggle is pinned to duty (physical, e.g. SR timing)")


def _solve3(A, b):
    """Solve a 3x3 linear system by Gaussian elimination; None if singular."""
    M = [A[i][:] + [b[i]] for i in range(3)]
    for i in range(3):
        p = M[i][i]
        if abs(p) < 1e-12:
            return None
        M[i] = [v / p for v in M[i]]
        for k in range(3):
            if k != i:
                f = M[k][i]
                M[k] = [M[k][j] - f * M[i][j] for j in range(4)]
    return [M[i][3] for i in range(3)]


def asym_peak_fit(xs, ys, grid=400):
    """Least-squares fit of two half-parabolas sharing an apex (x0, y0):
    y = y0 - a*(x0-x)^2 for x < x0, y = y0 - b*(x-x0)^2 for x >= x0, with a, b >= 0.
    x0 is grid-searched over the interior of the sweep. Returns (x0, y0, a, b) or None."""
    n, best = len(xs), None
    if n < 5:
        return None
    lo, hi = xs[1], xs[-2]
    syy = sum(y * y for y in ys)
    for k in range(grid + 1):
        x0 = lo + (hi - lo) * k / grid
        su = sv = suu = svv = r1 = r2 = 0.0
        nl = nr = 0
        for x, y in zip(xs, ys):
            if x < x0:
                u = (x0 - x) ** 2; su += u; suu += u * u; r1 -= u * y; nl += 1
            else:
                v = (x - x0) ** 2; sv += v; svv += v * v; r2 -= v * y; nr += 1
        if nl < 2 or nr < 2:
            continue
        r0 = sum(ys)
        p = _solve3([[n, -su, -sv], [-su, suu, 0.0], [-sv, 0.0, svv]], [r0, r1, r2])
        if not p or p[1] < 0 or p[2] < 0:
            continue
        sse = syy - (p[0] * r0 + p[1] * r1 + p[2] * r2)
        if best is None or sse < best[0]:
            best = (sse, x0, p[0], p[1], p[2])
    return best and best[1:]


def ls_sweep(con, tap, hs, pwm_max, period, fsw, args):
    """Hold HS, sweep the low-side on-count, find the Iout peak = optimal LS (zero-crossing) timing.

    The peak location is gain-independent (a multiplicative Iout error doesn't move it); comparing it
    to the firmware's voltage-based prediction (rectCtrlRatio*HS) reveals a fixed timing offset
    (dead-time / gate delay). The peak *curvature* gives a cross-check L (which, like the duty sweep,
    still scales with the Iout gain).
    """
    pwm_rect_min = round(pwm_max * MIN_DUTY_LS)
    if not safe_command(con, f"dc {hs}"):  # settle HS with automatic LS
        print("device unreachable while setting HS")
        return
    s = steady_read(con, tap, max(args.dwell, 4.0))
    if not s:
        print("no status while setting HS")
        return
    vi, vo = s["vin"], s["vout"]
    if vi <= vo or s["mode"] != "DCM":
        print(f"need a DCM point with Vin>Vout (got {s['mode']}, Vin={vi:.1f} Vout={vo:.1f}); lower --hs")
        return
    auto_ls = s["rect"]
    ideal_ls = (vi / vo - 1.0) * hs  # rectCtrlRatio(M) * HS, from measured voltages
    ls_lo = max(pwm_rect_min, round(args.ls_lo * ideal_ls))
    ls_hi = min(pwm_max - hs, round(args.ls_hi * ideal_ls))
    step = max(1, (ls_hi - ls_lo) // max(1, args.ls_steps - 1))
    print(f"HS={hs} D={hs / period:.3f} Vin={vi:.2f} Vout={vo:.2f}  "
          f"ideal_LS={ideal_ls:.0f} auto_LS={auto_ls}  sweep {ls_lo}..{ls_hi}")
    print(f"\n  {'LS':>5} {'Iout':>7} {'Vin':>6} {'Vout':>6} {'mode':>4}")

    rows, peak_io = [], -1e9
    for ls in range(ls_lo, ls_hi + 1, step):
        if not safe_command(con, f"dc {hs} {ls}"):
            print("  device unreachable; ending LS sweep with data so far")
            break
        s = steady_read(con, tap, args.dwell)
        if not s:
            continue
        io = s["iout"]
        rows.append((s["rect"], io))
        print(f"  {s['rect']:>5} {io:>7.3f} {s['vin']:>6.2f} {s['vout']:>6.2f} {s['mode']:>4}")
        peak_io = max(peak_io, io)
        if io > args.i_max:
            print(f"  Iout {io:.2f} > i-max; stopping")
            break
        if peak_io > 0.05 and io < 0.8 * peak_io and s["rect"] > ideal_ls:
            print("  passed the peak; stopping (limit reverse current)")
            break

    if len(rows) < 4:
        print("\ntoo few points to locate the peak")
        return
    xs, ys = [r[0] for r in rows], [r[1] for r in rows]
    pk = max(range(len(ys)), key=lambda i: ys[i])
    # The peak is strongly asymmetric: a shallow body-diode side and a steep reverse-current side,
    # both quadratic about the zero-crossing point. A single parabola is dragged toward the flat side.
    fit = asym_peak_fit(xs, ys)
    ls_peak, Lc = float(xs[pk]), None
    if fit:
        ls_peak, _, _, b = fit
        if b > 0:  # steep side: reverse charge ~ Vo*Vin/(2L(Vin-Vo)) * t^2 per period (V_f neglected)
            Lc = vo * vi / (2 * b * fsw * period ** 2 * (vi - vo))
    print()
    print(f"peak LS  : {ls_peak:.0f}   (Iout_peak {ys[pk]:.3f} A at raw argmax {xs[pk]}, "
          f"{'two-sided fit on %d pts' % len(xs) if fit else 'fit failed, raw argmax'})")
    print(f"ideal LS : {ideal_ls:.0f}   (rectCtrlRatio*HS from measured M)")
    print(f"auto  LS : {auto_ls}   (firmware applied)")
    print(f"offset   : peak-ideal {ls_peak - ideal_ls:+.0f} ct ({(ls_peak - ideal_ls) / hs * 100:+.1f}% of HS)"
          f"   peak-auto {ls_peak - auto_ls:+.0f} ct")
    if Lc:
        print(f"L (peak curvature) = {Lc * 1e6:.1f} uH   (cross-check; carries Iout gain like the duty sweep)")
    else:
        print("  (curvature L skipped: no two-sided fit)")
    print("  nonzero peak-ideal = fixed timing offset (dead-time / gate delay) for rectCtrlRatio")

    if getattr(args, "apply", False):
        if not (0 < pk < len(ys) - 1):
            print("  --apply skipped: peak at a sweep edge (never rolled over); widen --ls-hi")
            return
        cur = get_conf(con, "coil.conf", "rect_offset_ns") or "0"
        # rect_offset is added to the offset-free firmware formula (pwmCtrl*rectCtrlRatio == ideal_ls),
        # so the target is absolute: don't fold in cur or the applied auto_ls (the latter is perturbed
        # by reverse-current pullback / transients and misreads as the min-LS floor).
        lim = pwm_max // 8
        new_off = max(-lim, min(lim, round(ls_peak - ideal_ls - args.apply_margin)))
        # Store as a time (ns) so the calibration is independent of PWM resolution: counts/sec = fsw*period.
        tick_rate = fsw * period
        new_ns = round(new_off / tick_rate * 1e9)
        print(f"  --apply: coil.conf rect_offset_ns {cur} -> {new_ns}  "
              f"({new_off} ct @ {tick_rate / 1e6:.0f} MHz tick; peak-ideal {ls_peak - ideal_ls:+.0f} ct"
              f" - margin {args.apply_margin} ct; effective next boot)")
        safe_command(con, f"set-config coil.conf rect_offset_ns {new_ns}")
        print(f"  readback rect_offset_ns = {get_conf(con, 'coil.conf', 'rect_offset_ns')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-p", "--port", help="serial port")
    g.add_argument("--ip", help="TCP/telnet host[:port]")
    g.add_argument("--ble", nargs="?", const="fugu", help="BLE NUS device name (default fugu)")
    ap.add_argument("--fsw", type=float, help="switching freq Hz (default: realized, from pwm-dump)")
    ap.add_argument("--pwm-max", type=int, help="PWM period in timer ticks (default: from pwm-dump)")
    ap.add_argument("--steps", type=int, default=10, help="duty steps across the DCM band")
    ap.add_argument("--lo", type=float, default=0.25, help="start duty as fraction of boundary M")
    ap.add_argument("--hi", type=float, default=0.9, help="end duty as fraction of boundary M")
    ap.add_argument("--i-max", type=float, default=2.0, help="abort a step if Iout exceeds this (A)")
    ap.add_argument("--dwell", type=float, default=5.0, help="seconds to settle per step (>3 s)")
    ap.add_argument("--bidir", action="store_true",
                    help="sweep up then back down; compare L at matched duties (settle-check)")
    ap.add_argument("--plot-every", type=int, default=0,
                    help="print an ASCII L-vs-H plot every N points during the sweep")
    ap.add_argument("--ls-sweep", action="store_true",
                    help="hold HS, sweep low-side count; find the Iout peak (optimal LS timing)")
    ap.add_argument("--hs", type=int, help="[--ls-sweep] HS count to hold (default: --lo*M*period)")
    ap.add_argument("--ls-steps", type=int, default=24, help="[--ls-sweep] number of LS steps")
    ap.add_argument("--ls-lo", type=float, default=0.5, help="[--ls-sweep] start LS / ideal_LS")
    ap.add_argument("--ls-hi", type=float, default=1.4, help="[--ls-sweep] end LS / ideal_LS")
    ap.add_argument("--apply", action="store_true",
                    help="[--ls-sweep] write the measured offset to coil.conf::rect_offset_ns (effective next boot)")
    ap.add_argument("--apply-margin", type=int, default=12,
                    help="[--ls-sweep --apply] counts to keep below the Iout peak (reverse-current safety)")
    ap.add_argument("--restore", choices=["mppt", "off"], default="mppt",
                    help="what to do when done (default: re-enable MPPT)")
    ap.add_argument("--yes", action="store_true", help="skip the 'this moves power' confirmation")
    args = ap.parse_args()

    con, tap = connect(args)
    try:
        if not con.wait_ready(timeout=20):
            sys.exit("device not responding")

        pwm_ctrl_max = query_pwm_ctrl_max(con)
        if not pwm_ctrl_max:
            sys.exit("could not read pwmCtrlMax (dc range)")
        print('args=', ', '.join(f'{k}={v}' for k, v in args.__dict__.items() if v))
        timing = query_pwm_timing(con)
        if timing:
            fsw_rt, period_rt, pwm_max = timing
        else:
            fsw_rt = float(get_conf(con, "board.conf", "pwm_freq") or 0)
            period_rt = pwm_max = round(pwm_ctrl_max / (1.0 - MIN_DUTY_LS))
            print("WARNING: no pwm-dump (older firmware); period estimated from the dc range and fsw from"
                  " board.conf. Times and L can be off by a few %; pass --fsw and --pwm-max.")
        fsw, period = args.fsw or fsw_rt, args.pwm_max or period_rt
        if not (5e3 < fsw < 5e5):
            sys.exit(f"bad fsw={fsw}; pass --fsw")
        l0 = get_conf(con, "coil.conf", "L0")
        print(f"fsw={fsw:.2f} Hz  period={period} ticks ({fsw * period / 1e6:.2f} MHz)  pwmMax={pwm_max}"
              f"  pwmCtrlMax={pwm_ctrl_max}  coil.conf L0={l0}")

        idle = steady_read(con, tap, max(args.dwell, 4.0))
        if not idle:
            sys.exit("no status lines; is the console streaming? try a longer --dwell")
        vin, vout = idle["vin"], idle["vout"]
        print(f"idle: Vin={vin:.2f} Vout={vout:.2f}  M=Vout/Vin={vout / vin:.3f}")
        if vin <= vout + 1.0:
            sys.exit("need Vin > Vout (sun/headroom) for a buck DCM sweep")

        if not args.yes:
            ans = input("This drives the converter (moves power into the battery). Proceed? [y/N] ")
            if ans.strip().lower() not in ("y", "yes"):
                sys.exit("aborted")

        if args.ls_sweep:
            hs = args.hs or max(2, round(args.lo * (vout / vin) * period))
            try:
                ls_sweep(con, tap, hs, pwm_max, period, fsw, args)
            finally:
                safe_command(con, "mppt" if args.restore == "mppt" else "dc 0")
            return

        m0 = vout / vin
        h_lo = max(2, round(args.lo * m0 * period))
        h_hi = min(pwm_ctrl_max, round(args.hi * m0 * period))
        if h_hi <= h_lo:
            sys.exit("empty duty band; check --lo/--hi")
        step = max(1, (h_hi - h_lo) // max(1, args.steps - 1))

        rows = []
        print(f"\n  {'':1}{'H':>5} {'D':>6} {'Vin':>6} {'Vout':>6} {'Iout':>6} {'mode':>4} {'L_uH':>7}")
        try:
            up_lbl = "up" if args.bidir else ""
            rows_up, safe_h = run_sweep(con, tap, range(h_lo, h_hi + 1, step),
                                        period, fsw, args.i_max, args.dwell, up_lbl,
                                        args.plot_every)
            rows += rows_up
            if args.bidir and safe_h:
                rows_dn, _ = run_sweep(con, tap, range(safe_h, h_lo - 1, -step),
                                       period, fsw, args.i_max, args.dwell, "dn")
                rows += rows_dn
        finally:
            safe_command(con, "mppt" if args.restore == "mppt" else "dc 0")

        # fit only points whose current is well above the Iout offset/quantization floor; near
        # zero current a small sensor offset blows the estimate up (L proportional to 1/Iout).
        imax = max((r[4] for r in rows if r[5] == "DCM"), default=0.0)
        ifloor = max(0.15, 0.2 * imax)
        dcm = [r for r in rows if r[5] == "DCM" and r[6] == r[6] and r[4] >= ifloor]
        ls = sorted(r[6] for r in dcm)
        print()
        if len(ls) < 3:
            print(f"only {len(ls)} DCM points above {ifloor:.2f} A; widen the band or wait for sun")
            if all(r[5] == "CCM" for r in rows) and rows:
                print("all points CCM -> forced_pwm may be on (converter.conf); DCM needed")
            return
        med = median(ls)
        q1, q3 = ls[len(ls) // 4], ls[(3 * len(ls)) // 4]  # rough IQR
        spread = (q3 - q1) / med * 100.0
        print(f"L = {med * 1e6:.2f} uH   ({len(ls)} DCM pts, Iout>={ifloor:.2f}A, IQR {spread:.0f}%)")
        print(f"  -> set coil.conf L0={med:.3e}   (was {l0})")
        print("  note: scales with Iout calibration & pwmMax; dead-time/DCR bias ~few % low")
        if args.bidir:
            bidir_verdict(rows, ifloor)
    finally:
        try:
            con.transport.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
