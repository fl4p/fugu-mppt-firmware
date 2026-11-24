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
toward the CCM/DCM boundary and fit L across the DCM points. The switch node rings after the coil
current reaches zero, which makes the per-point L oscillate with the dead time; fit_l0() fits that
periodic term (ring frequency scanned) and reports the mean over whole ring periods.

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
import math
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
    try:
        from etc.fugu.transport import SerialTransport, SocketTransport, BleTransport
        from etc.fugu.console import Console
    except ImportError:  # estimator-only use (etc/test_measure_coil_fit.py) without the fugu package
        SerialTransport = SocketTransport = BleTransport = Console = None

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


# --- L0 estimator (src/selftest/coil_fit.h is the on-device copy: keep the two identical) ---------
FIT_MIN_PTS = 16
RING_F_LO, RING_F_HI = 80e3, 1.2e6  # ring frequency search range, Hz
MAX_BINS, MIN_BIN_PTS = 5, 6
ON_TIME_MIN = 50e-9  # on-time offset bound (s) when the dead time is shorter
GAIN_TOL = 0.01  # assumed Iout gain tolerance (shunt), relative
ZERO_TOL, ZERO_NOISE = 0.02, 0.002  # Iout zero: drift that marks a run disturbed, read noise floor, A


def zero_correct(iout, z0, z1):
    """Subtract the Iout zero read with the converter off before (z0) and after (z1) the sweep, interpolated
    over the sweep rows in order; returns (corrected, zero uncertainty, 1 sigma)."""
    n = len(iout)
    out = [io - (z0 + (z1 - z0) * i / max(1, n - 1)) for i, io in enumerate(iout)]
    return out, math.hypot(ZERO_NOISE, 0.2887 * abs(z1 - z0))  # read noise, drift bound |z1-z0|/2 as uniform


def _ring(c, w, x, nh):
    if not nh:
        return 0.0
    cw, sw = math.cos(w), math.sin(w)
    v = c[1] * cw + c[2] * sw + x * (c[2 * nh + 2] * cw + c[2 * nh + 3] * sw)
    if nh > 1:
        v += c[3] * (cw * cw - sw * sw) + c[4] * 2 * sw * cw
    return v


def _ring_solve(y, t, x, f, nh, env=False):
    """Least squares y ~ 1, cos/sin(2 pi h f t) (h <= nh), x [, x*cos, x*sin if env]; returns
    (rss, c, ((A^T A)^-1 of c0, of the x coefficient))."""
    sx = 1 + 2 * nh
    p = sx + (3 if nh and env else 1)
    N = [[0.0] * (p + 3) for _ in range(p)]
    for yi, ti, xi in zip(y, t, x):
        w = 2 * math.pi * f * ti
        cw, sw = math.cos(w), math.sin(w)
        a = [1.0, cw, sw]
        if nh > 1:
            a += [cw * cw - sw * sw, 2 * sw * cw]
        a = a[:sx] + [xi]
        if nh and env:
            a += [xi * cw, xi * sw]
        for r in range(p):
            for k in range(p):
                N[r][k] += a[r] * a[k]
            N[r][p] += a[r] * yi
    N[0][p + 1] = N[sx][p + 2] = 1.0
    for k in range(p):
        piv = max(range(k, p), key=lambda r: abs(N[r][k]))
        if abs(N[piv][k]) < 1e-9:
            return math.inf, None, None
        N[k], N[piv] = N[piv], N[k]
        for r in range(p):
            if r != k:
                m = N[r][k] / N[k][k]
                N[r] = [N[r][j] - m * N[k][j] for j in range(p + 3)]
    c = [N[k][p] / N[k][k] for k in range(p)] + [0.0] * (8 - p)
    v = (N[0][p + 1] / N[0][0], N[sx][p + 2] / N[sx][sx])
    rss = sum((yi - c[0] - _ring(c, 2 * math.pi * f * ti, xi, nh) - c[sx] * xi) ** 2 for yi, ti, xi in zip(y, t, x))
    return rss, c, v


def fit_l0(pts, fsw, zsig=0.0, ton=ON_TIME_MIN):
    """L from DCM points [(D, Vin, Vout, Iout), ...] (doc: coil-inductance.md, "Switch-node ringing").

    Per point 1/L = 2*Vout*fsw*Iout / ((Vin-Vout)*Vin*D^2). The switch-node ring makes 1/L_i periodic
    in the dead time t = (1 - D*Vin/Vout)/fsw; fit c0 + harmonics of a scanned ring frequency f + s*x, x the
    peak coil current Ipk = (Vin-Vout)*D/(fsw*L) minus its mean, and return L = 1/c0, the mean over whole
    ring periods. Below FIT_MIN_PTS points or without a significant (>= 1 %) periodic term: the median of
    the points (f = 0). Errors, relative: se (noise: residuals, Iout zero uncertainty zsig) and sys (1 sigma of
    the node charge, the on-time offset bound ton, the Iout gain, ring-model mismatch). Also L(Ipk): the
    apparent slope k = dL/dIpk / L(Ipk -> 0) with its noise kse and the slope the Iout/on-time offset bounds can
    produce ksys, and bins of the ring-corrected points [(Ipk lo, Ipk hi, L, se, offset bound), ...]."""
    ok = [(d, vi, vo, io) for d, vi, vo, io in pts if (vi - vo) * vi * d * d > 0 and io > 0 and vo > 0]
    y = [2 * vo * fsw * io / ((vi - vo) * vi * d * d) for d, vi, vo, io in ok]
    t = [(1 - d * vi / vo) / fsw for d, vi, vo, io in ok]
    x = [(vi - vo) * d / fsw for d, vi, vo, io in ok]
    n = len(y)
    nan = float("nan")
    r = dict(L=nan, Lmed=nan, f=0.0, amp=0.0, periods=0.0, se=nan, sys=nan, n=n, nh=0, k=nan, kse=nan, ksys=nan,
             bins=[])
    if n < 3:
        return r
    ymed = statistics.median(y)
    r["L"] = r["Lmed"] = 1 / ymed
    if n < FIT_MIN_PTS:
        return r
    vm = sum(max(vo, vi * vi / (2 * vo) - vo) for d, vi, vo, io in ok) / n
    y = [v / ymed for v in y]
    x = [v * ymed for v in x]
    xm = sum(x) / n
    x = [v - xm for v in x]
    rss0, _, _ = _ring_solve(y, t, x, 0.0, 0)
    span = max(t) - min(t)
    ts = sorted(t)
    dt = statistics.median([b - a for a, b in zip(ts, ts[1:])])
    nh = 2 if n >= 24 else 1
    best, f_best = math.inf, 0.0
    if span > 0 and dt > 0:
        f_lo, f_hi = max(RING_F_LO, 1 / span), min(RING_F_HI, 0.45 / dt)
        if f_hi > f_lo:
            df = 0.1 / span
            f = f_lo
            while f < f_hi:
                e, _, _ = _ring_solve(y, t, x, f, nh)
                if e < best:
                    best, f_best = e, f
                f += df
            for _ in range(2):  # refine around the grid minimum
                f0 = f_best
                for j in range(-5, 6):
                    e, _, _ = _ring_solve(y, t, x, f0 + j * df * 0.2, nh)
                    if e < best:
                        best, f_best = e, f0 + j * df * 0.2
                df *= 0.2
    rss = math.inf
    if best < 0.5 * rss0:
        rss, c, v = _ring_solve(y, t, x, f_best, nh, True)
        if c is not None and c[1] ** 2 + c[2] ** 2 > 1e-4 * c[0] ** 2:  # >= 1 % ripple: ring
            r.update(L=1 / (c[0] * ymed), f=f_best, amp=math.hypot(c[1], c[2]) / c[0], periods=span * f_best, nh=nh)
    nh = r["nh"]
    if not nh:
        rss, c, v = _ring_solve(y, t, x, 0.0, 0, True)
    sx = 1 + 2 * nh
    p = sx + (3 if nh else 1)
    if not (rss < math.inf and c[0] > 0):
        return r
    x = [(v + xm) * (1 + c[sx] * v / c[0]) for v in x]  # Ipk at L(Ipk) of the line, refit
    xm = sum(x) / n
    x = [v - xm for v in x]
    rss, c, v = _ring_solve(y, t, x, r["f"], nh, True)
    if not (rss < math.inf and c[0] > 0):
        return r
    sig = math.sqrt(rss / (n - p))
    # offsets as responses of the fit: dy/dIout, dy/dton (see coil_fit.h)
    dy = [yi / p_[3] for yi, p_ in zip(y, ok)]
    dd = [2 * fsw * yi / p_[0] for yi, p_ in zip(y, ok)]
    ez, cz, _ = _ring_solve(dy, t, x, r["f"], nh, True)
    ed, cd, _ = _ring_solve(dd, t, x, r["f"], nh, True)
    if not (ez < math.inf and ed < math.inf):
        return r
    e = [yi - c[0] - _ring(c, 2 * math.pi * r["f"] * ti, xi, nh) - c[sx] * xi for yi, ti, xi in zip(y, t, x)]
    a = min(0.9, max(0.0, sum(u * w for u, w in zip(e[1:], e)) / rss))  # residuals correlated: ring-model mismatch
    infl = math.sqrt((1 + a) / (1 - a))
    zn = vm * fsw / ((2 * math.pi * r["f"]) ** 2 * r["L"]) if r["f"] > 0 else 0.0
    sn = sig * math.sqrt(v[0]) / c[0]
    if nh:
        bz, bd = cz[0] / c[0] * zn, cd[0] / c[0] * ton
        r.update(se=math.hypot(sn, cz[0] / c[0] * zsig),
                 sys=math.sqrt((bz * bz + bd * bd + GAIN_TOL ** 2) / 3 + sn * sn * (infl * infl - 1)))
    rho, g = r["L"] * ymed, c[0] + c[sx] * xm
    r.update(k=-c[sx] * rho / g, kse=math.hypot(sig * math.sqrt(v[1]) * infl, cz[sx] * zsig) * rho / g,
             ksys=(abs(cz[sx]) * zn + abs(cd[sx]) * ton) * rho / g)
    pts_ = sorted(((xi + xm) / rho, yi - _ring(c, 2 * math.pi * r["f"] * ti, xi, nh), zi_, di_)
                  for xi, yi, ti, zi_, di_ in zip(x, y, t, dy, dd))
    nb = min(MAX_BINS, n // MIN_BIN_PTS)
    if r["f"] > 0:
        nb = min(nb, int(r["periods"] * 1.25))
    if nb < 2:  # under ~1.6 ring periods the slope is not separable from the ring
        r.update(k=nan, kse=nan, ksys=nan)
    else:
        for b in range(nb):
            seg = pts_[b * n // nb:(b + 1) * n // nb]
            u = sum(q[1] for q in seg) / len(seg)
            z = sum(q[2] for q in seg) / len(seg)
            zd = sum(q[3] for q in seg) / len(seg)
            r["bins"].append((seg[0][0], seg[-1][0], 1 / (u * ymed), math.hypot(sig / math.sqrt(len(seg)), z * zsig) / u,
                              (z * zn + zd * ton) / u))
    return r


def report_l0(r, L_ref=None):
    """Result lines shared in wording with the device (`L = ... uH` is parsed by the e2e test)."""
    if r["f"]:
        c_nf = 1e9 / ((2 * math.pi * r["f"]) ** 2 * r["L"])
        return (f"L = {r['L'] * 1e6:.2f} uH  (ring fit, {r['n']} DCM pts, +-{r['se'] * 100:.1f}% noise "
                f"+-{r['sys'] * 100:.1f}% syst; ring {r['f'] / 1e3:.0f} kHz = {c_nf:.1f} nF node C, ripple +-{r['amp'] * 100:.0f}%, "
                f"{r['periods']:.1f} periods; median of points {r['Lmed'] * 1e6:.2f} uH)")
    return (f"L = {r['L'] * 1e6:.2f} uH  (median of {r['n']} DCM pts; ring not resolved: "
            f"{'too few points' if r['n'] < FIT_MIN_PTS else 'no periodic term'}, expect up to +-5%)")


def report_split(r):
    """L(Ipk) lines, same wording as the device ([] without the split)."""
    if not r["bins"] or r["k"] != r["k"]:
        return []
    bins = " | ".join(f"{lo:.1f}-{hi:.1f}A {L * 1e6:.2f}uH+-{se * 100:.1f}/{sy * 100:.1f}%" for lo, hi, L, se, sy in r["bins"])
    w = r["ksys"] + 2 * r["kse"]
    lo = r["k"] - w
    return [f"L(Ipk), +-noise/offsets: {bins}",
            f"L(Ipk) slope {r['k'] * 100:+.2f} +-{r['kse'] * 100:.2f} %/A over {r['bins'][0][0]:.1f}-{r['bins'][-1][1]:.1f} A, "
            f"Iout/on-time offsets add up to +-{r['ksys'] * 100:.2f}: coil {lo * 100:+.2f}..{(r['k'] + w) * 100:+.2f} %/A, "
            + (f"drop <= {-lo * 100:.1f} %/A" if lo < 0 else "no drop resolved")]


def report_zero(z0, z1):
    """Iout zero line, and whether the run is disturbed (zero moved during the sweep)."""
    bad = abs(z1 - z0) > ZERO_TOL
    return (f"Iout zero {z0:+.3f}/{z1:+.3f} A (off, before/after), subtracted"
            + (f"; DISTURBED: moved > {ZERO_TOL:.2f} A, L not for coil.conf" if bad else "")), bad


def read_zero(con, tap, dwell):
    """Iout with the converter off, as the firmware reads it: `dc 0`, wait for a status line with H = 0
    (converter disabled, at most 10 s plus one status period), then the dwell, then the median of 5 `sensor avg`
    iout reads 50 ms apart (None if the converter did not stop or nothing was read)."""
    t0 = time.monotonic()
    if not safe_command(con, "dc 0"):
        return None
    while not any(p[4] == 0 for p in tap.collect(t0)):
        if time.monotonic() > t0 + 14.0:
            print("  converter did not stop for the Iout zero")
            return None
        time.sleep(0.2)
    time.sleep(dwell)
    xs = []
    for _ in range(5):
        a = read_sensor_avgs(con)
        if "iout" in a:
            xs.append(a["iout"])
        time.sleep(0.05)
    return median(xs) if xs else None


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


def query_gate_driver(con):
    """Active gate driver name (`rect-cal status` drv=...), or None on firmware without it."""
    for ln in con.command("rect-cal status", timeout=4.0):
        m = re.search(r"\bdrv=(\w+)", ln)
        if m:
            return m.group(1)
    return None


def query_dead_time(con):
    """Configured dead time (larger of hl/lh, s) from `dt`; 0 when bypassed, absent or unread."""
    try:
        for ln in con.command("dt", timeout=4.0):
            m = re.search(r"dt hl=(\d+\.?\d*) ns.*lh=(\d+\.?\d*) ns", ln)
            if m:
                return max(float(m.group(1)), float(m.group(2))) * 1e-9
    except Exception:
        pass
    return 0.0


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
        # The firmware ignores rect_offset_ns unless rect_offset_drv names the active gate driver.
        drv = query_gate_driver(con)
        if drv:
            safe_command(con, f"set-config coil.conf rect_offset_drv {drv}")
            safe_command(con, "set-config coil.conf rect_offset_cal 1")
        else:
            print("  WARNING: gate driver unknown (no rect-cal status); rect_offset_drv not set")
        print(f"  readback rect_offset_ns = {get_conf(con, 'coil.conf', 'rect_offset_ns')}")


class _Tee:
    """Duplicates writes to a stream into a file."""

    def __init__(self, stream, f):
        self.stream, self.f = stream, f

    def write(self, x):
        self.f.write(x)
        self.f.flush()
        return self.stream.write(x)

    def flush(self):
        self.stream.flush()

    def __getattr__(self, k):
        return getattr(self.stream, k)


def start_save(args, con):
    """Tee stdout/stderr into --save (default: measure-coil-runs/<utc>-<host>.log at the repo root)."""
    if args.no_save:
        return None
    path = args.save
    if not path:
        m = [re.search(r"Hostname: ([\w-]+)", ln) for ln in con.command("hostname", timeout=4.0)]
        host = next((x.group(1) for x in m if x), "dev")
        d = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "measure-coil-runs")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + f"-{host}.log")
    f = open(path, "a")
    sys.stdout, sys.stderr = _Tee(sys.stdout, f), _Tee(sys.stderr, f)
    print(f"saving output to {path}")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-p", "--port", help="serial port")
    g.add_argument("--ip", help="TCP/telnet host[:port]")
    g.add_argument("--ble", nargs="?", const="fugu", help="BLE NUS device name (default fugu)")
    ap.add_argument("--fsw", type=float, help="switching freq Hz (default: realized, from pwm-dump)")
    ap.add_argument("--pwm-max", type=int, help="PWM period in timer ticks (default: from pwm-dump)")
    ap.add_argument("--steps", type=int, default=48,
                    help="duty steps across the DCM band (>= 16 to resolve the switch-node ring)")
    ap.add_argument("--lo", type=float, default=0.25, help="start duty as fraction of boundary M")
    ap.add_argument("--hi", type=float, default=0.9, help="end duty as fraction of boundary M")
    ap.add_argument("--i-max", type=float, default=2.0, help="abort a step if Iout exceeds this (A)")
    ap.add_argument("--dwell", type=float, default=3.0, help="seconds to settle per step (>= 3 s: one status line)")
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
    ap.add_argument("--ton-ns", type=float,
                    help="bound of the HS on-time offset for the stated error (default: dead time, at least 50 ns)")
    ap.add_argument("--yes", action="store_true", help="skip the 'this moves power' confirmation")
    ap.add_argument("--save", help="write the whole output to this file (default: measure-coil-runs/)")
    ap.add_argument("--no-save", action="store_true", help="don't save the output")
    args = ap.parse_args()

    con, tap = connect(args)
    try:
        if not con.wait_ready(timeout=20):
            sys.exit("device not responding")
        start_save(args, con)

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
        z_dwell = max(args.dwell, 2.0)  # firmware: max(dwell, 2 s)
        z0 = read_zero(con, tap, z_dwell)
        z1 = None
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
            z1 = read_zero(con, tap, z_dwell)
        finally:
            safe_command(con, "mppt" if args.restore == "mppt" else "dc 0")

        zsig, disturbed = 0.0, z0 is None or z1 is None
        if not disturbed:  # Iout zero read with the converter off before/after the sweep
            ios, zsig = zero_correct([r[4] for r in rows], z0, z1)
            rows = [r[:4] + (io,) + r[5:] for r, io in zip(rows, ios)]
            zline, disturbed = report_zero(z0, z1)
        else:
            zline = "Iout zero not read; DISTURBED: L not for coil.conf"
        # fit only points whose current is well above the Iout offset/quantization floor; near
        # zero current a small sensor offset blows the estimate up (L proportional to 1/Iout).
        imax = max((r[4] for r in rows if r[5] == "DCM"), default=0.0)
        ifloor = max(0.15, 0.2 * imax)
        dcm = [r for r in rows if r[5] == "DCM" and r[6] == r[6] and r[4] >= ifloor]
        print()
        if len(dcm) < 3:
            print(f"only {len(dcm)} DCM points above {ifloor:.2f} A; widen the band or wait for sun")
            if all(r[5] == "CCM" for r in rows) and rows:
                print("all points CCM -> forced_pwm may be on (converter.conf); DCM needed")
            return
        ton = args.ton_ns * 1e-9 if args.ton_ns is not None else max(ON_TIME_MIN, query_dead_time(con))
        fit = fit_l0([(r[1], r[2], r[3], r[4]) for r in dcm], fsw, zsig, ton)
        ls = sorted(r[6] for r in dcm)
        q1, q3 = ls[len(ls) // 4], ls[(3 * len(ls)) // 4]  # rough IQR of the per-point values (uncorrected)
        print(report_l0(fit) + f"  Iout>={ifloor:.2f}A, IQR {(q3 - q1) / fit['Lmed'] * 100.0:.0f}%")
        for ln in [zline] + report_split(fit):
            print(ln)
        if not disturbed:
            print(f"  -> set coil.conf L0={fit['L']:.3e}   (was {l0})")
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
