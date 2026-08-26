#!/usr/bin/env python3
"""MCPWM gate-drive automated verifier. See docs/superpowers/specs/2026-05-23-pwm-gate-verifier-design.md.

Drives the device via console + captures HS/LS gates on a PicoScope 2000. Reports
PASS/FAIL per assertion and exits with the failed-row count.

Wiring: Ch A=HS gate (board.conf::pwm_hi), Ch B=LS gate (board.conf::pwm_li).
A free GPIO (default 14) wired to board.conf::pwm_fault_pin drives the fault test.

    etc/mcpwm_gate_verify.py --serial /dev/cu.usbmodem* [--skip-fault]
    etc/mcpwm_gate_verify.py --ip 192.168.1.173 --port 232
"""

import argparse
import os
import platform
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

_PICO_APP_RES = "/Applications/PicoScope 7 T&M.app/Contents/Resources"


def _reexec_under_rosetta_if_needed():
    """libps2000.dylib is x86_64-only — re-exec under Rosetta when running on arm64."""
    if platform.machine() == "arm64" and not os.environ.get("_PICO_X86"):
        os.execve(
            "/usr/bin/arch",
            ["arch", "-x86_64", "/usr/bin/python3", os.path.abspath(__file__)] + sys.argv[1:],
            dict(os.environ, _PICO_X86="1",
                 DYLD_LIBRARY_PATH=_PICO_APP_RES + ":" + os.environ.get("DYLD_LIBRARY_PATH", "")),
        )

LOGIC_TH = 1.65


def edges(v, th=LOGIC_TH):
    """Return (rising_idx, falling_idx) lists from a logic waveform (volts).
       Index 0 seeds initial state and is never reported as an edge."""
    if not v:
        return [], []
    rises, falls = [], []
    prev = v[0] > th
    for i in range(1, len(v)):
        cur = v[i] > th
        if cur and not prev:
            rises.append(i)
        elif prev and not cur:
            falls.append(i)
        prev = cur
    return rises, falls


_PWM_DUMP_RE = re.compile(r"(\w+)=(-?\d+)")


def parse_pwm_dump(line):
    """Parse a `pwm-dump` reply like `freq=39062 pwmMax=2048 hs_off=1024 ...`.
       Returns dict[str, int]; extra noise around the key=value pairs is ignored."""
    return {k: int(v) for k, v in _PWM_DUMP_RE.findall(line)}


def hs_grid(pwm_max):
    """HS sweep points: edge cases + log-spaced body."""
    assert pwm_max >= 2, f"hs_grid: pwm_max={pwm_max} too small"
    fractions = [1 / 128, 1 / 64, 1 / 16, 1 / 4, 1 / 2, 3 / 4]
    body = [int(round(f * pwm_max)) for f in fractions]
    return sorted(set([0, 1] + body + [pwm_max - 2, pwm_max - 1, pwm_max]))


def ls_grid(window):
    """LS sweep points within an upper bound `window = pwm_max - hs - 1`.
       Returns [] when window < 2 (no meaningful range)."""
    if window < 2:
        return []
    fractions = [0.05, 0.25, 0.5, 0.75, 0.99]
    body = [int(round(f * window)) for f in fractions]
    return sorted(set([0, 1] + body + [window - 1, window]))


@dataclass
class Result:
    name: str
    pass_: bool
    measured: object = None
    expected: object = None
    tol: object = None
    detail: str = ""


def open_console(args):
    # Lazy import so test-time imports of this module don't pull in pyserial/etc.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from fugu.transport import SerialTransport, SocketTransport
    from fugu.console import Console
    if args.serial:
        t = SerialTransport(args.serial, 115200)
    else:
        t = SocketTransport(args.ip, args.port)
    return Console(t)


def open_scope():
    # Lazy import — pulls in libps2000 via ctypes, which is x86-only.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import pico_capture as pc
    s = pc.PS2000()
    s.channel(pc.CH_A, pc.RANGES["5v"], pc.DC)
    s.channel(pc.CH_B, pc.RANGES["5v"], pc.DC)
    s.trigger(pc.CH_A, 0, rising=True, delay_pct=0, auto_ms=500)
    return s, pc


def capture(scope, pc, freq, samples=3000):
    """Capture HS (Ch A) + LS (Ch B); window sized to ≥4 switching periods."""
    tb, dt, n = scope.timebase_for(4 / freq, samples)
    a, b, ov = scope.capture(tb, n, True, arm_timeout=2.0)
    return dt, pc.to_volts(a, pc.RANGES["5v"]), pc.to_volts(b, pc.RANGES["5v"])


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="MCPWM gate-drive closed-loop verifier")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--serial", help="serial port, e.g. /dev/cu.usbmodem2101")
    g.add_argument("--ip",     help="device IP / hostname for telnet/socket")
    ap.add_argument("--port", type=int, default=23, help="socket port (with --ip)")
    ap.add_argument("--fault-driver-pin", type=int, default=14,
                    help="GPIO on the device wired to pwm_fault_pin")
    ap.add_argument("--skip-fault", action="store_true",
                    help="skip Phase 4 (fault brake)")
    ap.add_argument("--force-host", action="store_true",
                    help="bypass hostname allow-list (use for fry/flat ONLY when you know)")
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap.parse_args(argv)


def print_results(phase, rows):
    """Render one phase block; return # failing rows."""
    print(f"PHASE {phase}")
    n_fail = 0
    for r in rows:
        tag = "PASS" if r.pass_ else "FAIL"
        if not r.pass_:
            n_fail += 1
        m = f"{r.measured!s}" if r.measured is not None else ""
        e = f"exp {r.expected!s}" if r.expected is not None else ""
        t = f"tol {r.tol!s}" if r.tol is not None else ""
        d = f"  -- {r.detail}" if r.detail else ""
        print(f"  {r.name:<28} {tag}  {m:<14} {e:<14} {t:<14}{d}")
    return n_fail


BOARD_CONF_KEYS = ["pwm_freq", "pwm_deadtime_ns", "pwm_deadtime_hl_ns", "pwm_deadtime_lh_ns",
                   "pwm_hi", "pwm_li", "pwm_fault_pin"]
# per-transition overrides; each falls back to the common pwm_deadtime_ns when unset
_DT_EDGE_KEYS = ["pwm_deadtime_hl_ns", "pwm_deadtime_lh_ns"]
# Bench/mock hosts: bare `fugu` or `fugu-esp32s3-*`; real converters (fry, flat, fugu139C, …) are not matched.
SAFE_HOST_PATTERN = re.compile(r"^fugu(-esp32s3-.*)?$")
# get-config replies look like:  Conf '/littlefs/conf/board.conf:pwm_freq' = '39000'
# The dead-time keys are floats and a quantized value is genuinely fractional (193.75 ns), so the
# value pattern has to accept decimals and exponents -- an int-only pattern reads them as absent
# and silently substitutes the common key.
_CONF_REPLY_RE = re.compile(
    r"^Conf\s+'[^']*:(?P<k>\w+)'\s*=\s*'(?P<v>[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)?'\s*$",
    re.MULTILINE)


def setup_board(con) -> dict:
    """Read board.conf keys via `get-config`; populate `pwm_max` from a `pwm-dump` reply
    (it's runtime-derived from bestTiming(freq), not stored in the conf). Returns
    dict[str, int|None]; empty conf values become None unless they default elsewhere."""
    out: dict = {}
    for k in BOARD_CONF_KEYS:
        r = con.command(f"get-config board.conf {k}", timeout=3.0)
        m = _CONF_REPLY_RE.search(r.text)
        if m and m.group("v"):
            # float first: int("193.75") raises. Keep an integral value as int so pin/flag keys
            # still compare and format as they always did.
            fv = float(m.group("v"))
            out[k] = int(fv) if fv.is_integer() else fv
        else:
            # empty value: pwm_deadtime_ns -> 0 (sensible default), pwm_fault_pin -> -1
            # (unconfigured), the per-edge keys -> resolved from pwm_deadtime_ns below
            out[k] = 0 if k == "pwm_deadtime_ns" else (-1 if k == "pwm_fault_pin" else None)
    for k in _DT_EDGE_KEYS:
        if out[k] is None:
            out[k] = out["pwm_deadtime_ns"]
    dump = parse_pwm_dump(con.command("pwm-dump", timeout=3.0).text)
    out["pwm_max"] = dump.get("pwmMax")
    return out


BOARD_KEYS = BOARD_CONF_KEYS + ["pwm_max"]   # public-facing list (what main() checks for None)


def read_hostname(con):
    """Read hostname via the `hostname` console verb (always built, even with WITH_NETW=0).
       Falls back to `ip` if hostname returns nothing parseable. Returns str or None."""
    r = con.command("hostname", timeout=3.0)
    m = re.search(r"[Hh]ostname\s*[:=]\s*(\S+)", r.text)
    if m:
        return m.group(1)
    r = con.command("ip", timeout=3.0)
    m = re.search(r"[Hh]ostname\s*[:=]\s*(\S+)", r.text)
    return m.group(1) if m else None


def safety_check(con, args) -> Result:
    name = read_hostname(con) or "<unknown>"
    if args.force_host:
        return Result("hostname_check", True, measured=name,
                      detail="--force-host: safety bypassed")
    if not SAFE_HOST_PATTERN.match(name):
        return Result("hostname_check", False, measured=name,
                      expected="fugu-esp32s3-*",
                      detail="refusing to run against non-mock host "
                             "(use --force-host to override)")
    return Result("hostname_check", True, measured=name)


SIGNAL_STEP_MIN_V = 1.2   # minimum 0->1 swing the probe must show; catches wrong pin, swapped probes, >50x atten


def phase_signal_check(con, scope, pc, board: dict) -> list[Result]:
    """Pre-flight: drive each gate to near-full duty via MCPWM (`dc <pwm_max-1>` for HS,
    `dc 1 <pwm_max-2>` for LS, with `sync forced` to keep manual LS), and assert the matching
    scope channel mean voltage rises by >SIGNAL_STEP_MIN_V vs the all-LOW state (`dc 0`).
    `gpio <pin>` writes don't reach the pad while MCPWM owns it (MCPWM uses the GPIO matrix
    outside Arduino's periman), so this test uses MCPWM duties to produce the level steps.
    """
    rows: list[Result] = []
    pwm_max = board["pwm_max"]
    cap_freq = board["pwm_freq"]   # only sizes the capture window; any value works for level reads

    def _mean(con_cmd, ch_idx):
        con.command(con_cmd, timeout=2.0)
        time.sleep(0.15)
        _, hs_v, ls_v = capture(scope, pc, cap_freq)
        v = hs_v if ch_idx == 0 else ls_v
        return sum(v) / len(v)

    # Baseline: both gates LOW.
    try:
        v_low_a = _mean("dc 0", 0)
        v_low_b = _mean("dc 0", 1)
    except Exception as e:
        rows.append(Result("signal_baseline", False, detail=f"capture error (baseline): {e}"))
        return rows

    # HS test: `dc <pwm_max-1>` -> HS ~99.95% duty -> Ch A mean ~ Vdd.
    try:
        v_hs_a = _mean(f"dc {pwm_max - 1}", 0)
    except Exception as e:
        rows.append(Result(f"signal[HS=GPIO{board['pwm_hi']}, ChA]", False,
                           detail=f"capture error: {e}"))
        v_hs_a = v_low_a
    step_hs = v_hs_a - v_low_a
    rows.append(Result(f"signal[HS=GPIO{board['pwm_hi']}, ChA]", step_hs > SIGNAL_STEP_MIN_V,
                       measured=f"step={step_hs:+.2f}V",
                       expected=f">{SIGNAL_STEP_MIN_V}V",
                       detail=f"low={v_low_a:+.2f}V high={v_hs_a:+.2f}V"))

    # LS test: force-rect with HS=1, LS=pwm_max-2 -> LS HIGH most of the period -> Ch B mean ~ Vdd.
    try:
        con.command("sync forced", timeout=2.0); time.sleep(0.1)
        v_ls_b = _mean(f"dc 1 {pwm_max - 2}", 1)
    except Exception as e:
        rows.append(Result(f"signal[LS=GPIO{board['pwm_li']}, ChB]", False,
                           detail=f"capture error: {e}"))
        v_ls_b = v_low_b
    step_ls = v_ls_b - v_low_b
    rows.append(Result(f"signal[LS=GPIO{board['pwm_li']}, ChB]", step_ls > SIGNAL_STEP_MIN_V,
                       measured=f"step={step_ls:+.2f}V",
                       expected=f">{SIGNAL_STEP_MIN_V}V",
                       detail=f"low={v_low_b:+.2f}V high={v_ls_b:+.2f}V"))

    con.command("dc 0", timeout=2.0)
    return rows


def phase1_freq_duty(con, scope, pc, board: dict) -> list[Result]:
    """Sweep HS over hs_grid; assert measured freq + duty match expectations.
    Special-cases hs=0 (static-low) and hs=pwm_max (static-high) — no edges expected."""
    rows: list[Result] = []
    pwm_max = board["pwm_max"]
    freq    = board["pwm_freq"]
    tick_s  = 1.0 / (freq * pwm_max)

    for hs in hs_grid(pwm_max):
        con.command(f"dc {hs}", timeout=2.0)
        time.sleep(0.1)
        try:
            dt, hs_v, ls_v = capture(scope, pc, freq)
        except Exception as e:
            rows.append(Result(f"capture[hs={hs}]", False, detail=f"capture error: {e}"))
            continue
        rises, falls = edges(hs_v)

        if hs == 0:
            ok = (not rises and not falls and max(hs_v) < LOGIC_TH)
            rows.append(Result(f"static[hs=0]", ok, measured=f"vmax={max(hs_v):.2f}",
                               detail="HS static-low expected"))
            continue
        if hs == pwm_max:
            ok = (not rises and not falls and min(hs_v) > LOGIC_TH)
            rows.append(Result(f"static[hs={pwm_max}]", ok, measured=f"vmin={min(hs_v):.2f}",
                               detail="HS static-high expected"))
            continue

        if len(rises) < 2 or len(falls) < 1:
            rows.append(Result(f"duty[hs={hs}]", False,
                               detail=f"too few edges ({len(rises)} rises, {len(falls)} falls)"))
            continue
        period_s = (rises[1] - rises[0]) * dt
        if period_s <= 0:
            rows.append(Result(f"duty[hs={hs}]", False, detail=f"period_s={period_s}"))
            continue
        freq_meas = 1.0 / period_s
        f0 = next((i for i in falls if i > rises[0]), None)
        if f0 is None:
            rows.append(Result(f"duty[hs={hs}]", False, detail="no falling edge after first rise"))
            continue
        duty_meas = (f0 - rises[0]) * dt / period_s
        duty_exp  = hs / pwm_max
        ok_duty   = abs(duty_meas - duty_exp) <= 1.0 / pwm_max
        rows.append(Result(f"duty[hs={hs}]", ok_duty,
                           measured=f"{duty_meas*100:.3f}%",
                           expected=f"{duty_exp*100:.3f}%",
                           tol=f"1t ({1/pwm_max*100:.3f}%)"))
        ok_freq = abs(freq_meas - freq) / freq <= 1e-3
        rows.append(Result(f"freq[hs={hs}]", ok_freq,
                           measured=f"{freq_meas:.1f} Hz",
                           expected=f"{freq} Hz",
                           tol="0.1%"))
    con.command("dc 0", timeout=2.0)
    return rows


def phase2_ls_pos_width(con, scope, pc, board: dict) -> list[Result]:
    """HS × LS matrix sweep. Outer hs from hs_grid(pwm_max), inner ls from ls_grid(window)
    where window = pwm_max - hs - 1. Asserts measured LS rise time + width match the firmware's
    pwm-dump event ticks (±1 tick). Special cases for hs=0 and hs >= pwm_max - 1."""
    rows: list[Result] = []
    pwm_max = board["pwm_max"]
    freq    = board["pwm_freq"]
    tick_s  = 1.0 / (freq * pwm_max)
    tol_s   = tick_s

    for hs in hs_grid(pwm_max):
        window = pwm_max - hs - 1

        # hs=0: both gates should be quiet
        if hs == 0:
            con.command("dc 0", timeout=2.0)
            time.sleep(0.1)
            try:
                dt, hs_v, ls_v = capture(scope, pc, freq)
            except Exception as e:
                rows.append(Result("both_static_low[hs=0]", False, detail=f"capture error: {e}"))
                continue
            ok = max(hs_v) < LOGIC_TH and max(ls_v) < LOGIC_TH
            rows.append(Result("both_static_low[hs=0]", ok,
                               detail=f"HSvmax={max(hs_v):.2f}, LSvmax={max(ls_v):.2f}"))
            continue

        # hs >= pwm_max - 1: no LS window — HS static-high, LS static-low
        if hs >= pwm_max - 1:
            con.command(f"dc {hs}", timeout=2.0)
            time.sleep(0.1)
            try:
                dt, hs_v, ls_v = capture(scope, pc, freq)
            except Exception as e:
                rows.append(Result(f"no_window[hs={hs}]", False, detail=f"capture error: {e}"))
                continue
            ok = min(hs_v) > LOGIC_TH and max(ls_v) < LOGIC_TH
            rows.append(Result(f"no_window[hs={hs}]", ok,
                               detail=f"HSvmin={min(hs_v):.2f}, LSvmax={max(ls_v):.2f}"))
            continue

        for ls in ls_grid(window):
            con.command(f"dc {hs} {ls}", timeout=2.0)
            time.sleep(0.1)
            r = con.command("pwm-dump", timeout=2.0)
            dump = parse_pwm_dump(r.text)
            if not all(k in dump for k in ("hs_off", "ls_on", "ls_off", "pwmMax")):
                rows.append(Result(f"ls[hs={hs},ls={ls}]", False,
                                   detail=f"pwm-dump parse fail: {r.text!r}"))
                continue
            try:
                dt, hs_v, ls_v = capture(scope, pc, freq)
            except Exception as e:
                rows.append(Result(f"ls[hs={hs},ls={ls}]", False,
                                   detail=f"capture error: {e}"))
                continue
            hs_rises, _        = edges(hs_v)
            ls_rises, ls_falls = edges(ls_v)

            # ls=0 → firmware reports ls_off=0 → no LS rises expected
            if ls == 0 or dump["ls_off"] == 0:
                ok = (len(ls_rises) == 0)
                rows.append(Result(f"ls_off[hs={hs},ls=0]", ok,
                                   detail=f"{len(ls_rises)} LS rises (expect 0)"))
                continue

            if not hs_rises or not ls_rises or not ls_falls:
                rows.append(Result(f"ls[hs={hs},ls={ls}]", False,
                                   detail=f"missing edges (HSr={len(hs_rises)}, "
                                          f"LSr={len(ls_rises)}, LSf={len(ls_falls)})"))
                continue
            r0 = hs_rises[0]
            lr = next((i for i in ls_rises if i > r0), None)
            if lr is None:
                rows.append(Result(f"ls[hs={hs},ls={ls}]", False,
                                   detail="no LS rise after first HS rise"))
                continue
            lf = next((i for i in ls_falls if i > lr), None)
            if lf is None:
                rows.append(Result(f"ls[hs={hs},ls={ls}]", False,
                                   detail="no LS fall after LS rise"))
                continue
            ls_rise_s  = (lr - r0) * dt
            ls_high_s  = (lf - lr) * dt
            exp_rise_s = dump["ls_on"] * tick_s
            exp_high_s = (dump["ls_off"] - dump["ls_on"]) * tick_s
            ok_pos = abs(ls_rise_s - exp_rise_s) <= tol_s
            ok_wid = abs(ls_high_s - exp_high_s) <= tol_s
            rows.append(Result(f"ls_pos[hs={hs},ls={ls}]", ok_pos,
                               measured=f"{ls_rise_s*1e6:.3f}µs",
                               expected=f"{exp_rise_s*1e6:.3f}µs",
                               tol=f"{tol_s*1e6:.3f}µs"))
            rows.append(Result(f"ls_wid[hs={hs},ls={ls}]", ok_wid,
                               measured=f"{ls_high_s*1e6:.3f}µs",
                               expected=f"{exp_high_s*1e6:.3f}µs",
                               tol=f"{tol_s*1e6:.3f}µs"))
            # Cliff regression — when ls fills the window the LS must NOT be static-HIGH.
            if ls == window:
                static_high = (min(ls_v) > LOGIC_TH)
                rows.append(Result(f"ls_cliff[hs={hs},ls={ls}]", not static_high,
                                   detail=f"LSvmin={min(ls_v):.2f}"))
    con.command("dc 0", timeout=2.0)
    return rows


def phase3_deadtime(con, scope, pc, board: dict) -> list[Result]:
    """Measure both dead-bands and assert each matches its board.conf key within ±1 tick:
    dt_hl (HS-fall → LS-rise, the MCPWM RED register, pwm_deadtime_hl_ns) and dt_lh (LS-fall →
    next HS-rise at the period wrap, the pwmMax reservation, pwm_deadtime_lh_ns). Also assert
    there's no sample where HS and LS are both above the logic threshold."""
    rows: list[Result] = []
    pwm_max = board["pwm_max"]
    freq    = board["pwm_freq"]
    tick_s  = 1.0 / (freq * pwm_max)
    tol_s   = tick_s
    hl_ns_exp = board["pwm_deadtime_hl_ns"] or 0
    lh_ns_exp = board["pwm_deadtime_lh_ns"] or 0

    hs = pwm_max // 2
    ls = (pwm_max - hs - 1) // 2
    con.command(f"dc {hs} {ls}", timeout=2.0)
    time.sleep(0.1)
    try:
        dt, hs_v, ls_v = capture(scope, pc, freq)
    except Exception as e:
        rows.append(Result("dt_capture", False, detail=f"capture error: {e}"))
        return rows

    hs_rises, hs_falls = edges(hs_v)
    ls_rises, _        = edges(ls_v)
    if not (hs_rises and hs_falls and ls_rises):
        rows.append(Result("dt_setup", False,
                           detail=f"missing edges (HSr={len(hs_rises)} HSf={len(hs_falls)} LSr={len(ls_rises)})"))
        con.command("dc 0", timeout=2.0)
        return rows

    hf0 = hs_falls[0]
    lr_after = next((i for i in ls_rises if i > hf0), None)
    if lr_after is None:
        rows.append(Result("dt_setup", False, detail="no LS rise after first HS fall"))
        con.command("dc 0", timeout=2.0)
        return rows
    dt_hl_s = (lr_after - hf0) * dt
    ok_dt = abs(dt_hl_s - hl_ns_exp * 1e-9) <= tol_s
    rows.append(Result("dt_hl", ok_dt,
                       measured=f"{dt_hl_s*1e9:.0f}ns",
                       expected=f"{hl_ns_exp}ns",
                       tol=f"{tol_s*1e9:.1f}ns (1t)"))

    shoot = any(hs_v[i] > LOGIC_TH and ls_v[i] > LOGIC_TH for i in range(len(hs_v)))
    rows.append(Result("shoot_through", not shoot,
                       detail="none" if not shoot else "OVERLAP DETECTED"))

    rows += _dt_lh_row(con, scope, pc, pwm_max, freq, tick_s, tol_s, lh_ns_exp, hs)
    con.command("dc 0", timeout=2.0)
    return rows


def _dt_lh_row(con, scope, pc, pwm_max, freq, tick_s, tol_s, lh_ns_exp, hs) -> list[Result]:
    """Wrap-side band: LS-fall -> the next HS-rise, in its own capture with LS commanded to its
    maximum (`ls = pwm_max - hs - 1`).

    The band is `period_ticks - cmpLS`, so any commanded slack between cmpLS and pwm_max lands in
    the measurement on top of the reserved dead-time. That slack has to be converted to time with
    `tick_s`, which is the NOMINAL 1/(freq*pwm_max) and not the true 1/resolution_hz — at 39 kHz
    the two differ by ~0.05 ns per tick, negligible over the 32 ticks dt_hl spans but ~49 ns over
    the ~1019 ticks a half-width LS leaves, i.e. 8x the one-tick tolerance. Driving LS to its
    maximum collapses the slack to a single count and the error with it."""
    ls = pwm_max - hs - 1
    con.command(f"dc {hs} {ls}", timeout=2.0)
    time.sleep(0.1)
    try:
        dt, hs_v, ls_v = capture(scope, pc, freq)
    except Exception as e:
        return [Result("dt_lh", False, detail=f"capture error: {e}")]
    hs_rises, _   = edges(hs_v)
    _, ls_falls   = edges(ls_v)
    lf0 = next((i for i in ls_falls), None)
    hr_after = next((i for i in hs_rises if lf0 is not None and i > lf0), None)
    if lf0 is None or hr_after is None:
        return [Result("dt_lh", False, detail="no LS-fall / HS-rise pair at the period wrap")]
    dt_lh_s  = (hr_after - lf0) * dt
    slack_s  = (pwm_max - (hs + ls)) * tick_s   # = 1 count by construction
    exp_s    = lh_ns_exp * 1e-9 + slack_s
    return [Result("dt_lh", abs(dt_lh_s - exp_s) <= tol_s,
                   measured=f"{dt_lh_s*1e9:.0f}ns",
                   expected=f"{exp_s*1e9:.0f}ns",
                   tol=f"{tol_s*1e9:.1f}ns (1t)",
                   detail=f"{lh_ns_exp}ns reserved + 1 count slack (ls={ls})")]


def phase4_fault_brake(con, scope, pc, board: dict, drv_pin: int) -> list[Result]:
    """Drive a free device GPIO (wired to board.conf::pwm_fault_pin) and assert both gates
    go LOW in hardware. Then deassert and confirm switching resumes."""
    rows: list[Result] = []
    freq = board["pwm_freq"]
    fault_pin = board["pwm_fault_pin"]
    if fault_pin is None or fault_pin < 0:
        rows.append(Result("fault_pin_configured", False,
                           detail="board.conf::pwm_fault_pin not set; brake cannot be tested"))
        return rows

    # Pre-condition: fault driver low, both gates switching.
    con.command(f"gpio {drv_pin} 0", timeout=2.0)
    con.command(f"dc {board['pwm_max']//2} {board['pwm_max']//4}", timeout=2.0)
    time.sleep(0.1)
    try:
        dt, hs_v, ls_v = capture(scope, pc, freq, samples=3000)
    except Exception as e:
        rows.append(Result("brake_pre_switching", False, detail=f"capture error: {e}"))
        return rows
    pre_rises, _ = edges(hs_v)
    if len(pre_rises) < 2:
        rows.append(Result("brake_pre_switching", False,
                           detail=f"only {len(pre_rises)} HS rises before brake — gates already off"))
        return rows
    rows.append(Result("brake_pre_switching", True,
                       detail=f"{len(pre_rises)} HS rises in pre-brake capture"))

    # Assert fault, recapture: both gates must be quiet. Brake reacts in hardware (no CPU loop)
    # so 50 ms is plenty — full settle, not the "long enough for the firmware to notice" 100 ms.
    con.command(f"gpio {drv_pin} 1", timeout=2.0)
    time.sleep(0.05)
    try:
        dt, hs_v, ls_v = capture(scope, pc, freq, samples=3000)
    except Exception as e:
        rows.append(Result("brake_gates_quiet", False, detail=f"capture error: {e}"))
        con.command(f"gpio {drv_pin} 0", timeout=2.0)
        return rows
    rises, falls   = edges(hs_v)
    ls_rises, _    = edges(ls_v)
    ok_brake = (not rises and not falls and not ls_rises)
    rows.append(Result("brake_gates_quiet", ok_brake,
                       measured=f"HSr={len(rises)} HSf={len(falls)} LSr={len(ls_rises)}",
                       expected="all 0"))

    # Deassert + reissue duty, recapture: switching must resume.
    con.command(f"gpio {drv_pin} 0", timeout=2.0)
    con.command(f"dc {board['pwm_max']//2} {board['pwm_max']//4}", timeout=2.0)
    time.sleep(0.1)
    try:
        dt, hs_v, ls_v = capture(scope, pc, freq, samples=3000)
    except Exception as e:
        rows.append(Result("brake_recover", False, detail=f"capture error: {e}"))
        con.command("dc 0", timeout=2.0)
        return rows
    rises, _ = edges(hs_v)
    ok_recover = len(rises) >= 2
    rows.append(Result("brake_recover", ok_recover,
                       measured=f"{len(rises)} HS rises",
                       expected="≥2"))

    con.command("dc 0", timeout=2.0)
    return rows


def format_summary(summary: list) -> tuple:
    """summary: [(phase_label, n_fail | "SKIP"), ...] -> (one-line summary string, total n_fail)."""
    n_fail = 0
    parts = []
    for name, n in summary:
        if n == "SKIP":
            parts.append(f"{name} SKIP")
        else:
            parts.append(f"{name} " + ("PASS" if n == 0 else f"FAIL({n})"))
            n_fail += n
    return "SUMMARY  " + "  ".join(parts) + f"   exit={n_fail}", n_fail


def main():
    args = parse_args()
    _reexec_under_rosetta_if_needed()
    con = open_console(args)
    summary: list = []   # (phase_label, n_fail or "SKIP")
    try:
        chk = safety_check(con, args)
        n0 = print_results("0  preflight", [chk])
        summary.append(("0", n0))
        if not chk.pass_:
            line, _ = format_summary(summary)
            print(line)
            return n0
        board = setup_board(con)
        print(f"  board: {board}")
        if any(board[k] is None for k in BOARD_KEYS):
            print("  ERROR: setup_board could not read all keys; aborting", file=sys.stderr)
            return 1
        scope, pc = open_scope()
        try:
            sig_rows = phase_signal_check(con, scope, pc, board)
            n_sig = print_results("0.5  signal check (digitalWrite step ≥1.2V)", sig_rows)
            summary.append(("0.5", n_sig))
            if n_sig:
                print("  ERROR: signal check failed — fix probe placement/attenuation before sweeping; aborting",
                      file=sys.stderr)
                line, _ = format_summary(summary)
                print(line)
                return n_sig
            summary.append(("1", print_results("1  freq + HS duty",
                                                phase1_freq_duty(con, scope, pc, board))))
            summary.append(("2", print_results("2  LS pos + width (HS × LS)",
                                                phase2_ls_pos_width(con, scope, pc, board))))
            summary.append(("3", print_results("3  deadtime + shoot-through",
                                                phase3_deadtime(con, scope, pc, board))))
            if args.skip_fault:
                print("PHASE 4  fault brake  SKIPPED (--skip-fault)")
                summary.append(("4", "SKIP"))
            else:
                summary.append(("4", print_results("4  fault brake",
                                                    phase4_fault_brake(con, scope, pc, board,
                                                                       args.fault_driver_pin))))
        finally:
            scope.close()
    finally:
        con.close()

    line, n_fail = format_summary(summary)
    print(line)
    return n_fail


if __name__ == "__main__":
    sys.exit(main())
