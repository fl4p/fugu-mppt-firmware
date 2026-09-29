#!/usr/bin/env python3
"""Cluster runner for the host-side e2e tests in this directory.

The standalone ``test_*.py`` scripts each need a different rig — a plain console, a mock build, a
bench unit you may deliberately crash, a real converter with a coil, or a controllable AP/router.
This runner groups them into **clusters** by that setup requirement, builds each test's argv from a
small set of shared options, and runs the ones whose prerequisites are satisfied (the rest are
SKIPped with the reason). It reports an aggregate PASS / FAIL / SKIP and exits non-zero on any FAIL.

Clusters
--------
  console      any device, console only (serial/telnet), non-destructive
  mock         a mock build (fake ADC) over serial
  destructive  bench unit ONLY — deliberately panics / reboots / long fuzz
  power        a real converter + coil (sun/headroom) — drives the half-bridge
  wifi         a controllable AP/router rig (restart webhook or a 2nd serial, SSIDs)

Each test takes the connection via the cluster's transport; tests needing extra setup (a broker, a
router webhook, SSIDs, a second serial) are SKIPped unless you pass it (flag or env).

Usage
-----
    python etc/e2e-test/run_e2e.py --list
    python etc/e2e-test/run_e2e.py --cluster console --serial /dev/cu.usbmodem1201
    python etc/e2e-test/run_e2e.py --cluster console --telnet 192.168.4.2:23 --mqtt-host 192.168.1.200
    python etc/e2e-test/run_e2e.py --cluster mock   --serial /dev/cu.usbmodem1201
    python etc/e2e-test/run_e2e.py --cluster wifi   --serial /dev/cu.usbmodem1201 \
        --restart-url http://192.168.1.173/scan --ssid pwr-station --other-ssid backup-ap
    python etc/e2e-test/run_e2e.py --cluster destructive --serial /dev/cu.usbmodem1201 --with-fuzz

Env fallbacks: $MQTT_HOST, $RESTART_URL, $E2E_SSID, $E2E_OTHER_SSID, $E2E_PSK, $E2E_ROUTER,
$E2E_ROUTER_WAN_IP.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable

CLUSTERS = ["console", "mock", "destructive", "power", "wifi"]


class Skip(Exception):
    """Raised by a test's argv builder when its prerequisites aren't satisfied."""


def serial_or_skip(o):
    if not o.serial:
        raise Skip("needs --serial")
    return o.serial


def net_target(o):
    """host:port for a telnet/ip transport, or None."""
    return o.telnet


# Each spec: how to run one test. `build(o)` returns the trailing argv (after the script path) or
# raises Skip(reason). `transport` documents what connection it consumes. Timeouts are wall-clock
# ceilings for the subprocess (wifi/coil tests run for minutes).
def _nettools(o):
    if o.serial:
        return ["--serial", o.serial]
    if net_target(o):
        return ["--telnet", net_target(o)]
    raise Skip("needs --serial or --telnet")


def _stdin_batch(o):
    if o.serial:
        return ["--serial", o.serial]
    if net_target(o):
        return ["--telnet", net_target(o)]
    raise Skip("needs --serial or --telnet")


def _mqtt_cmd(o):
    base = ["--serial", o.serial] if o.serial else (["--telnet", net_target(o)] if net_target(o) else None)
    if base is None:
        raise Skip("needs --serial or --telnet")
    if not o.mqtt_host:
        raise Skip("needs --mqtt-host / $MQTT_HOST (broker)")
    return base + ["--mqtt", o.mqtt_host]


def _console_plan(o, mock=False):
    if o.serial:
        base = ["--serial", o.serial]
    elif net_target(o):
        base = ["--telnet", net_target(o)]
    else:
        raise Skip("needs --serial or --telnet")
    if mock:
        base += ["--mock"]
    if o.include_network:
        base += ["--include-network"]
    return base


def _influx(o):
    return ["-p", serial_or_skip(o)] + (["--mock"] if o.mock else [])


def _coredump(o):
    return ["--port", serial_or_skip(o)]


def _measure_coil(o):
    if o.serial:
        return ["--serial", o.serial]
    if net_target(o):
        return ["--ip", net_target(o)]
    raise Skip("needs --serial or --telnet")


def _wifi_off(o):
    return ["--serial", serial_or_skip(o)] + (["--ssid", o.ssid] if o.ssid else [])


def _wifi_reconnect(o):
    return ["--serial", serial_or_skip(o)]


def _stick(o):
    serial_or_skip(o)
    if not o.restart_url:
        raise Skip("needs --restart-url / $RESTART_URL (AP restart webhook)")
    a = ["--mode", "stick", "--serial", o.serial, "--restart-url", o.restart_url]
    if o.ssid:
        a += ["--ssid", o.ssid]
    return a


def _roam(o):
    serial_or_skip(o)
    miss = [n for n, v in (("--restart-url", o.restart_url), ("--ssid", o.ssid),
                           ("--other-ssid", o.other_ssid)) if not v]
    if miss:
        raise Skip("needs " + ", ".join(miss))
    return ["--mode", "roam", "--serial", o.serial, "--restart-url", o.restart_url,
            "--ssid", o.ssid, "--other-ssid", o.other_ssid]


def _service_recovery(o):
    miss = [n for n, v in (("--serial (DUT)", o.serial), ("--router", o.router),
                           ("--router-wan-ip", o.router_wan_ip), ("--ssid", o.ssid),
                           ("--psk", o.psk)) if not v]
    if miss:
        raise Skip("needs " + ", ".join(miss))
    return ["--dut", o.serial, "--router", o.router, "--router-wan-ip", o.router_wan_ip,
            "--ssid", o.ssid, "--psk", o.psk]


# name, script, cluster, transport-note, build, timeout_s
SPECS = [
    ("nettools",        "test_nettools.py",                    "console", "serial|telnet", _nettools, 180),
    ("stdin-batch",     "test_stdin_batch.py",                 "console", "serial|telnet", _stdin_batch, 180),
    ("mqtt-cmd-input",  "test_mqtt_cmd_input.py",              "console", "serial|telnet + broker", _mqtt_cmd, 120),
    ("console-plan",    "test_console_plan.py",                "console", "serial|telnet", _console_plan, 240),
    ("console-plan",    "test_console_plan.py",                "mock",    "serial|telnet (mock fw)", lambda o: _console_plan(o, mock=True), 240),
    ("influx",          "influx_test.py",                      "mock",    "serial (mock fw)", _influx, 180),
    ("coredump",        "test_coredump.py",                    "destructive", "serial — PANICS the device", _coredump, 180),
    ("measure-coil",    "test_measure_coil.py",                "power",   "serial|telnet — real coil, needs Vin>Vout", _measure_coil, 600),
    ("wifi-off-timeout","test_wifi_off_timeout.py",            "wifi",    "serial — waits ~minutes", _wifi_off, 240),
    ("wifi-reconnect",  "test_wifi_reconnect_storm.py",        "wifi",    "serial — off/on storm, no rig", _wifi_reconnect, 180),
    ("wifi-stick",      "test_wifi_outage.py",                 "wifi",    "serial + restart webhook", _stick, 360),
    ("wifi-roam",       "test_wifi_outage.py",                 "wifi",    "serial + webhook + 2 SSIDs", _roam, 360),
    ("wifi-svc-recovery","test_wifi_outage_service_recovery.py","wifi",   "DUT + router serial + SSID/PSK", _service_recovery, 600),
]

# Long fuzzers — opt-in within the destructive cluster (no fixed runtime; they autodetect the port).
FUZZ = [
    ("fuzz-sequences", "fuzz_sequences.py", 900),
    ("fuzz-extreme",   "fuzz_extreme.py",   900),
]


def list_clusters():
    for c in CLUSTERS:
        print(f"\n[{c}]")
        for name, script, cl, note, _b, _t in SPECS:
            if cl == c:
                print(f"  {name:<20} {note}")
        if c == "destructive":
            for name, script, _t in FUZZ:
                print(f"  {name:<20} (--with-fuzz) tries to crash the device; long")


def run_one(name, script, build, timeout, o):
    try:
        extra = build(o)
    except Skip as s:
        print(f"[SKIP] {name:<20} {s}", flush=True)
        return "SKIP"
    argv = [PY, os.path.join(HERE, script)] + extra
    shown = " ".join([script] + extra)
    if o.dry_run:
        print(f"[DRY ] {name:<20} {shown}", flush=True)
        return "SKIP"
    print(f"\n===== {name}  ({shown}) =====", flush=True)
    try:
        rc = subprocess.run(argv, timeout=timeout).returncode
    except subprocess.TimeoutExpired:
        print(f"[FAIL] {name:<20} timeout after {timeout}s", flush=True)
        return "FAIL"
    except KeyboardInterrupt:
        print(f"[FAIL] {name:<20} interrupted", flush=True)
        return "FAIL"
    status = "PASS" if rc == 0 else "FAIL"
    print(f"[{status}] {name:<20} exit={rc}", flush=True)
    return status


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cluster", choices=CLUSTERS + ["all"], default="console")
    ap.add_argument("--serial", metavar="DEV", help="serial console / DUT port")
    ap.add_argument("--telnet", metavar="HOST[:PORT]", help="TCP/telnet console host")
    ap.add_argument("--mock", action="store_true", help="the connected build is a mock (for influx)")
    ap.add_argument("--include-network", action="store_true",
                    help="also run the console-plan's NVS/Wi-Fi-mutating commands (wifi on, hostname)")
    ap.add_argument("--with-fuzz", action="store_true", help="also run the long fuzzers (destructive cluster)")
    ap.add_argument("--list", action="store_true", help="list clusters and their tests, then exit")
    ap.add_argument("--dry-run", action="store_true", help="print the argv each test would run, don't run")
    # extra setup (flag or env)
    ap.add_argument("--mqtt-host", default=os.environ.get("MQTT_HOST"))
    ap.add_argument("--restart-url", default=os.environ.get("RESTART_URL"))
    ap.add_argument("--ssid", default=os.environ.get("E2E_SSID"))
    ap.add_argument("--other-ssid", default=os.environ.get("E2E_OTHER_SSID"))
    ap.add_argument("--psk", default=os.environ.get("E2E_PSK"))
    ap.add_argument("--router", default=os.environ.get("E2E_ROUTER"))
    ap.add_argument("--router-wan-ip", default=os.environ.get("E2E_ROUTER_WAN_IP"))
    o = ap.parse_args()

    if o.list:
        list_clusters()
        return 0

    clusters = CLUSTERS if o.cluster == "all" else [o.cluster]
    results = []
    ran_one = False
    for spec_name, script, cl, _note, build, timeout in SPECS:
        if cl in clusters:
            # The device's telnet server is single-client; settle between back-to-back tests so a
            # new connect doesn't race the prior test's not-yet-released socket (transient drops).
            if ran_one and not o.dry_run:
                time.sleep(2.0)
            results.append((spec_name, run_one(spec_name, script, build, timeout, o)))
            ran_one = True
    if "destructive" in clusters and o.with_fuzz:
        for name, script, timeout in FUZZ:
            results.append((name, run_one(name, script, lambda o: [], timeout, o)))

    print("\n" + "=" * 60)
    npass = sum(1 for _, s in results if s == "PASS")
    nfail = sum(1 for _, s in results if s == "FAIL")
    nskip = sum(1 for _, s in results if s == "SKIP")
    print(f"clusters: {', '.join(clusters)}")
    print(f"summary: {npass} passed, {nfail} failed, {nskip} skipped")
    if nfail:
        print("failed:", ", ".join(n for n, s in results if s == "FAIL"))
    if not nfail and not npass and not o.dry_run:
        print("nothing ran (all skipped): not a pass")
        return 2
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
