#!/usr/bin/env python3
"""Fugu device health check — runs read-only console commands and prints a verdict table.

Reuses the etc/fugu console package and fugu_console.make_transport, so every transport works:
serial (-p), telnet/NAT (--ip host:port), BLE NUS (--ble [--address]), ESPHome BLE proxy
(--ble-proxy host [--address]) or MQTT (--mqtt broker). Sends only read-only commands
(ip/status/mem/svc list/sensor) — never drives the converter. Prints a rich table (like ota.py;
--plain for a borderless one). Requires: rich (pip install rich).

  python etc/fugu_health.py --ble --address <UUID>
  python etc/fugu_health.py --ip 192.168.1.173:232
  python etc/fugu_health.py -p /dev/cu.usbmodemXXXX
"""
import argparse
import os
import re
import sys
import time

from rich.box import SIMPLE
from rich.console import Console as RichConsole
from rich.table import Table

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fugu.console import Console
from fugu.transport import SocketTransport
import fugu_console

OK, WARN, BAD = "✅", "⚠️", "❌"

# CLion/PyCharm run consoles claim TTY but don't render ANSI — match ota.py's handling.
_in_jb_run = bool(os.environ.get("PYCHARM_HOSTED"))
_console = RichConsole(color_system=None if _in_jb_run else "auto",
                       force_terminal=False if _in_jb_run else None,
                       highlight=not _in_jb_run,
                       width=200 if _in_jb_run else None)

# one periodic status line: V=65.3/26.23 I=0.00/ 0.00A  0.0W 29℃37℃ 443sps  0㎅/s ... st=START,0 lag=68131㎲ N=.. rssi=-37
_NUM = r"([\d.+-]+|nan)"
STATUS_RE = re.compile(
    rf"V=\s*{_NUM}/\s*{_NUM}\s+I=\s*{_NUM}/\s*{_NUM}A\s+{_NUM}W\s+{_NUM}℃{_NUM}℃"
    rf"\s+(\d+)sps.*?st=(\S+).*?lag=(\d+)㎲\s+N=(\d+)\s+rssi=(-?\d+)"
)


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def parse_status(line):
    m = STATUS_RE.search(line or "")
    if not m:
        return None
    g = m.groups()
    return dict(vin=fnum(g[0]), vout=fnum(g[1]), iin=fnum(g[2]), iout=fnum(g[3]),
               watt=fnum(g[4]), t0=fnum(g[5]), t1=fnum(g[6]), sps=int(g[7]),
               st=g[8], lag_us=int(g[9]), n=int(g[10]), rssi=int(g[11]))


def parse_status_kv(lines):
    """Pull a few fields out of the multi-line `status` reply."""
    t = "\n".join(lines)
    out = {}
    for key, pat in (("mode", r"Charger:\s*(\S+)"),
                     ("vbat_max", r"Vbat_max=([\d.]+)V"),
                     ("v_term", r"v_term=([\d.]+)V"),
                     ("ibat", r"ibat=(-?[\d.]+)A"),
                     ("ah", r"ahSinceFull=([\d.]+)Ah")):
        m = re.search(pat, t)
        if m:
            out[key] = m.group(1)
    m = re.search(r"BMS\s+vcell_high=([\d.]+)V\s*\(([^)]*)\)", t)
    if m:
        out["vcell"], out["bms"] = m.group(1), m.group(2).strip()
    return out


def parse_mem(lines):
    out = {}
    for ln in lines:
        m = re.match(r"(Total|Free) heap:\s+(\d+)", ln)
        if m:
            out[m.group(1).lower()] = int(m.group(2))
    return out


def parse_svc(lines):
    svcs = []
    for ln in lines:
        m = re.match(r"(\w+)\s+(Running|Stopped|Failed)\s+(\w+)\s+(yes|no)\s*(.*)", ln)
        if m:
            svcs.append(dict(name=m.group(1), state=m.group(2), enabled=m.group(4) == "yes",
                            detail=m.group(5).strip()))
    return svcs


def parse_sensors(lines):
    out = []
    cur = None
    for ln in lines:
        m = re.match(r"Sensor `(\w+)` \(ch(\d+),\s*(\w+)\):", ln)
        if m:
            cur = dict(name=m.group(1), kind=m.group(3), num=0, stdpct=float("nan"))
            out.append(cur)
        elif cur:
            mn = re.search(r"num=\s*(\d+)", ln)
            if mn and "EWM" not in ln:
                cur["num"] = int(mn.group(1))
            ms = re.search(r"std%=\s*([\d.]+)", ln)
            if ms:
                cur["stdpct"] = float(ms.group(1))
    return out


def fmt_age(s):
    m = re.search(r"(\d+)s ago", s or "")
    return int(m.group(1)) if m else None


def fmt_ago(sec):
    sec = int(sec)
    if sec < 90:
        return f"{sec}s ago"
    if sec < 5400:
        return f"{sec // 60}m ago"
    if sec < 172800:
        return f"{sec // 3600}h ago"
    return f"{sec // 86400}d ago"


def parse_coredump(lines):
    """Returns None if not present, else dict(check=..., crashed=epoch or 0)."""
    t = "\n".join(lines)
    if "coredump: none" in t:
        return None
    m = re.search(r"coredump:\s*present.*?check=(\w+)", t)
    if not m:
        return None
    ts = re.search(r"crashed=(\d+)", t)
    return dict(check=m.group(1), crashed=int(ts.group(1)) if ts else 0)


def build_rows(st, status, mem, svcs, sensors, cd):
    rows = []  # (check, reading, verdict)

    # Identity
    ident = []
    if status.get("ip"):
        ident.append(status["ip"])
    if st:
        ident.append(f"rssi {st['rssi']}")
    rows.append(("Identity", ", ".join(ident) or "no reply", OK if ident else BAD))

    # Converter — an idle START with ~0 W can mean an aborted sweep parked it (not always night)
    if st:
        idle = "START" in st["st"] and st["watt"] < 1
        note = "  (idle — `sweep` to (re)start tracking?)" if idle else ""
        rows.append(("Converter",
                     f"st={st['st']}, {st['watt']:.1f} W, Vin {st['vin']:.1f} / Vout {st['vout']:.2f} V, "
                     f"{st['sps']} sps{note}", WARN if idle else OK))
    else:
        rows.append(("Converter", "no status line", WARN))

    # Charger / BMS
    bms = status.get("bms", "")
    age = fmt_age(bms)
    chg = []
    if status.get("mode"):
        chg.append(status["mode"])
    if status.get("vcell"):
        chg.append(f"cell {status['vcell']}V ({bms})")
    if status.get("ibat"):
        chg.append(f"ibat {status['ibat']}A")
    if status.get("ah"):
        chg.append(f"ahSinceFull {status['ah']}")
    bms_ok = "ok" in bms.lower() and (age is None or age < 180)
    v = OK if bms_ok else (WARN if status.get("vcell") or "n/a" in bms.lower() else BAD)
    rows.append(("Charger/BMS", ", ".join(chg) or "no data", v))

    # Temps
    if st:
        tmax = max(st["t0"], st["t1"])
        v = OK if tmax < 60 else (WARN if tmax < 75 else BAD)
        rows.append(("Temps", f"{st['t0']:.0f} ℃ / {st['t1']:.0f} ℃", v))

    # Heap
    if mem.get("total"):
        free, total = mem["free"], mem["total"]
        pct = 100 * free / total
        v = OK if free >= 45000 else (WARN if free >= 25000 else BAD)
        rows.append(("Heap", f"{free/1024:.1f} KB free / {total/1024:.0f} KB ({pct:.0f}%)", v))

    # Services
    if svcs:
        bad = [s for s in svcs if s["enabled"] and s["state"] != "Running"]
        running = [s["name"] for s in svcs if s["state"] == "Running"]
        off = [s["name"] for s in svcs if not s["enabled"]]
        detail = f"{len(running)} running ({', '.join(running)})"
        if off:
            detail += f"; disabled: {', '.join(off)}"
        v = BAD if bad else OK
        if bad:
            detail += f"; DOWN: {', '.join(s['name'] for s in bad)}"
        rows.append(("Services", detail, v))

    # Sensors
    if sensors:
        phys = [s for s in sensors if s["kind"] == "physical"]
        dead = [s["name"] for s in phys if s["num"] == 0]
        v = OK if not dead else BAD
        detail = f"{len(phys)} physical sampling" if not dead else f"DEAD: {', '.join(dead)}"
        rows.append(("Sensors", detail, v))

    # Coredump — a panic dump in flash. A crash within 24h is the headline; older is still a flag.
    if cd is None:
        rows.append(("Coredump", "none", OK))
    else:
        bad_chk = cd["check"] != "ok"
        if cd["crashed"]:
            age = time.time() - cd["crashed"]
            recent = age < 86400
            detail = f"present, crashed {fmt_ago(age)}"
            v = BAD if recent else WARN
        else:
            detail = "present, crash time unknown (pre-timestamp firmware)"
            v = WARN
        if bad_chk:
            detail += "; check=BAD"
            v = BAD
        detail += " — `coredump get` to decode, `erase` to clear"
        rows.append(("Coredump", detail, v))

    return rows


def render(rows, plain=False):
    t = Table(show_header=True, header_style="bold", box=(SIMPLE if plain else None))
    t.add_column("Check")
    t.add_column("Reading")
    t.add_column("Verdict", justify="center")
    for c, r, v in rows:
        t.add_row(c, r, v)
    _console.print(t)


def add_transport_args(ap):
    ap.add_argument("-p", "--port", default=None, help="serial port (default: $ESPPORT or autodetect)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--ble", action="store_true", help="direct macOS BLE NUS")
    ap.add_argument("--name", default="fugu", help="name substring (BLE adv / MQTT hostname)")
    ap.add_argument("--address", help="BLE address/UUID (with --ble / --ble-proxy)")
    ap.add_argument("--ble-proxy", metavar="HOST[:PORT]", help="reach BLE NUS via ESPHome proxy")
    ap.add_argument("--proxy-password", default=os.environ.get("ESPHOME_API_PASSWORD", ""))
    ap.add_argument("--ip", help="TCP/telnet host[:port] (NAT-forwarded endpoints)")
    ap.add_argument("--mqtt", metavar="BROKER", help="MQTT broker host")
    ap.add_argument("--mqtt-port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    ap.add_argument("--mqtt-user", default=os.environ.get("MQTT_USER", "pv"))
    ap.add_argument("--mqtt-pass", default=os.environ.get("MQTT_PASS"))
    ap.add_argument("--mqtt-readonly", action="store_true")


def main():
    fugu_console.load_env_file()
    ap = argparse.ArgumentParser(description="Fugu device health check (read-only)")
    add_transport_args(ap)
    ap.add_argument("--plain", action="store_true", help="borderless table (no box)")
    ap.add_argument("--timeout", type=float, default=6.0, help="per-command timeout (s)")
    ap.set_defaults(tls=None, adapter=None)  # read by fugu_console.make_transport
    args = ap.parse_args()

    last_status = {"line": ""}
    transport = fugu_console.make_transport(args)
    con = Console(transport, wait_banner=isinstance(transport, SocketTransport),
                  on_line=lambda ln: last_status.__setitem__("line", ln)
                  if parse_status(ln) else None)
    def q(cmd):
        return con.command(cmd, timeout=args.timeout, retry=2)  # type: ignore[arg-type]

    try:
        ip, status, mem, svc, sensor, cd = (q(c) for c in
                                            ("ip", "status", "mem", "svc list", "sensor",
                                             "coredump info"))
    finally:
        con.close()

    sd = parse_status_kv(status)
    m = re.search(r"Local IP Address:\s*(\S+)", ip.text)
    if m:
        sd["ip"] = m.group(1)
    st = parse_status(last_status["line"])

    rows = build_rows(st, sd, parse_mem(mem), parse_svc(svc), parse_sensors(sensor),
                      parse_coredump(cd))
    render(rows, plain=args.plain)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
