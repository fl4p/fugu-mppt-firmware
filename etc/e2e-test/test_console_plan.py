#!/usr/bin/env python3
"""E2E test: walk the device's console command surface and report PASS/FAIL/SKIP.

Exercises the firmware's string command protocol in a meaningful order — read-only diagnostics,
the MCU/ESP32 debug surface, network debug tools, config round-trips, harmless actuators, service
ops, then (opt-in) the converter/PWM commands and the NVS/Wi-Fi-mutating commands. Each step asserts
either an OK confirmation, a substring the firmware always emits, or — for steps that may legitimately
be declined on this hardware — tolerates a refusal as SKIP.

Three tiers, gated so it is safe to point at a live converter by default:
  always           safe read-only / non-destructive — run everywhere
  --mock           drives PWM or the live charger — only on a mock build (fake ADC, no switching)
  --include-network mutates NVS / Wi-Fi (wifi on, hostname) — opt-in

(This used to live in `etc/fugu_console.py` as its `--test`/`--mock` mode; the console client is now
just a client and this exerciser belongs with the rest of the e2e suite.)

Usage
-----
    python etc/e2e-test/test_console_plan.py --serial /dev/cu.usbmodem1201
    python etc/e2e-test/test_console_plan.py --telnet 192.168.4.2:23
    python etc/e2e-test/test_console_plan.py --serial /dev/cu.usbmodem1201 --mock --include-network
"""
import argparse
import os
import sys

ETC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # repo/etc (holds the fugu pkg)
sys.path.insert(0, ETC_DIR)
from fugu.transport import SerialTransport, SocketTransport
from fugu.console import Console
from _harness import Results

GROUP_ALWAYS, GROUP_MOCK, GROUP_NET = "always", "mock", "net"

# Per-command timeout overrides (seconds), keyed by command verb. The default 4 s fits most
# commands; the I2C bus scan is slower and the nettools reach the network.
TIMEOUT_OVERRIDE = {"scan-i2c": 12.0, "curl": 15.0, "ping": 8.0, "tcpconnect": 8.0, "run": 180.0}


def _timeout_for(cmd, default=4.0):
    verb = cmd.split(None, 1)[0] if cmd else cmd
    if verb == "sleep":
        try:
            return min(max(float(cmd.split(None, 1)[1]) + 2.0, default), 62.0)
        except (IndexError, ValueError):
            return default
    return TIMEOUT_OVERRIDE.get(verb, default)


# Each step: (command, expect_substr | None, group, tolerate_reject)
#   tolerate_reject — the firmware declining is an acceptable outcome (the final-else REJECT marker,
#                     or an early `return false` that warns but prints no OK): e.g. no fan / no panel
#                     switch / wrong topology / a default-off feature not compiled in.
PLAN = [
    # --- read-only diagnostics --------------------------------------------------------------
    ("mem", "Free heap", GROUP_ALWAYS, False),
    # peek 4 bytes of the masked ROM at 0x40000000 — an executable region mapped on both esp32 and
    # esp32-s3 (DRAM/DROM ranges differ per chip, so a RAM address isn't portable). Hits the
    # typed-print path; the device echoes `peek 0x... = 0x...`, a reliable expect-substring.
    ("peek 0x40000000 4", "peek 0x40000000 =", GROUP_ALWAYS, False),
    ("sensor", "Sensor", GROUP_ALWAYS, False),
    ("rt-stats", None, GROUP_ALWAYS, False),
    ("reset-lag", None, GROUP_ALWAYS, False),
    ("ip", "IP Address", GROUP_ALWAYS, False),
    ("scan-i2c", None, GROUP_ALWAYS, True),  # may report no devices on a mock
    ("svc list", "NAME", GROUP_ALWAYS, False),
    # --- MCU/ESP32 debug surface (unconditional commands) -----------------------------------
    ("tasks", "STKFREE", GROUP_ALWAYS, False),       # FreeRTOS table; header has STKFREE_B
    ("bootinfo", "reset reason", GROUP_ALWAYS, False),
    ("heap", "INTERNAL", GROUP_ALWAYS, False),
    ("heap check", "integrity", GROUP_ALWAYS, False),
    ("log wifi info", None, GROUP_ALWAYS, False),    # set a tag's log level (reversible, harmless)
    ("ls", "entries", GROUP_ALWAYS, False),          # lists /littlefs -> "ls: N entries in ..."
    ("ls conf", "entries", GROUP_ALWAYS, False),
    ("cat conf/board.conf", None, GROUP_ALWAYS, True),  # file presence depends on the board config
    # --- network debug tools (CONFIG_FUGU_WITH_NETTOOLS; default off -> unknown cmd -> SKIP) -----
    # tolerate=True covers both "feature not built" and "no connectivity"; the expect-substring is
    # still enforced when the command runs and returns OK. nslookup of a literal IP needs no DNS.
    ("netstat", "mac=", GROUP_ALWAYS, True),
    ("nslookup 8.8.8.8", "8.8.8.8", GROUP_ALWAYS, True),
    ("ping 8.8.8.8 1", "sent", GROUP_ALWAYS, True),  # summary prints even at 100% loss
    ("tcpconnect 8.8.8.8 53", None, GROUP_ALWAYS, True),  # 'open' with internet, else declined->SKIP
    ("curl https://example.com", "HTTP", GROUP_ALWAYS, True),
    ("curl -X POST -d hello=world https://example.com", "HTTP", GROUP_ALWAYS, True),
    # --- config: dump, then a non-destructive round-trip on a scratch file ------------------
    ("get-config board.conf", "board.conf", GROUP_ALWAYS, False),
    ("get-config converter.conf", "converter.conf", GROUP_ALWAYS, False),
    ("set-config selftest.conf probe 4242", None, GROUP_ALWAYS, False),
    ("get-config selftest.conf probe", "4242", GROUP_ALWAYS, False),
    # --- harmless actuators ------------------------------------------------------------------
    ("led 030", None, GROUP_ALWAYS, False),
    ("fan 30", None, GROUP_ALWAYS, True),  # declined if no fan is configured (e.g. mock)
    ("fan 0", None, GROUP_ALWAYS, True),
    ("led 000", None, GROUP_ALWAYS, False),
    # --- service management: log level + restart (reversible; skip if the service isn't built) -
    ("svc log scope info", None, GROUP_ALWAYS, True),
    ("svc restart scope", None, GROUP_ALWAYS, True),
    # --- BLE telemetry stream (CONFIG_FUGU_WITH_BLE_TELE; default off -> unknown cmd -> SKIP) ----
    ("tele-ble", "tele-ble", GROUP_ALWAYS, True),  # status only; `tele-ble 1` needs a BLE client
    # `set-time <ms>` intentionally omitted: it steps the device wall clock.
    # --- ADC backend re-init (brief; safe on a mock) ----------------------------------------
    ("adc-restart", None, GROUP_MOCK, True),
    ("adc-reset", None, GROUP_MOCK, True),
    # --- charger limit overrides (live params) ----------------------------------------------
    ("vset 28.5", None, GROUP_MOCK, False),
    ("iset 10", None, GROUP_MOCK, False),
    ("speed 1.0", None, GROUP_MOCK, False),
    # --- PWM / converter: enter manual mode, exercise switches, return to tracking ----------
    ("dc 0", None, GROUP_MOCK, False),  # switches to manual PWM at zero duty
    ("+5", None, GROUP_MOCK, False),
    ("-5", None, GROUP_MOCK, False),
    ("sync on", None, GROUP_MOCK, False),
    ("sync off", None, GROUP_MOCK, False),
    ("sync forced", None, GROUP_MOCK, False),
    ("sync off", None, GROUP_MOCK, False),
    ("bf 1", None, GROUP_MOCK, True),  # rejected if no backflow switch configured
    ("bf 0", None, GROUP_MOCK, True),
    ("short-ls", None, GROUP_MOCK, True),  # only valid in boost with Vin~0
    ("dc 0", None, GROUP_MOCK, False),
    ("mppt", None, GROUP_MOCK, False),  # back to tracking (valid only in manual mode)
    ("sweep", None, GROUP_MOCK, False),
    # --- PSU mode (enter/exit, restore to MPPT) ---------------------------------------------
    ("psu 28.5", None, GROUP_MOCK, False),  # enter PSU mode with a setpoint
    ("psu", None, GROUP_MOCK, False),  # print PSU state
    ("psu off", None, GROUP_MOCK, False),  # exit PSU mode back to MPPT
    # --- network / NVS / reboot (opt-in only) -----------------------------------------------
    ("wifi on", None, GROUP_NET, False),
    ("hostname fugu-test", None, GROUP_NET, False),
    # `ota <url>` and `wifi off`/`wifi-add` intentionally omitted: they flash/reboot or wipe NVS.
    # `wifi off <minutes>` (temporary, keeps the SSID) has its own test: test_wifi_off_timeout.py
]


def run_plan(con, res, mock, include_net):
    for cmd, expect, group, tolerate in PLAN:
        if group == GROUP_MOCK and not mock:
            res.skip(cmd, "drives PWM/charger — needs --mock")
            continue
        if group == GROUP_NET and not include_net:
            res.skip(cmd, "mutates NVS/Wi-Fi — needs --include-network")
            continue

        reply = con.command(cmd, timeout=_timeout_for(cmd))

        if reply.ok:
            if expect is not None and expect not in reply.text:
                res.check(cmd, False, f"expected {expect!r} in reply")
            else:
                res.check(cmd, True)
        elif tolerate:
            # explicit reject, or an early `return false` (warning, no OK) — both acceptable here
            res.skip(cmd, "declined by firmware (not applicable on this setup)")
        elif reply.rejected:
            res.check(cmd, False, "rejected")
        elif reply.timed_out and not reply:
            res.check(cmd, False, "no response (timeout)")
        else:
            res.check(cmd, False, "no OK confirmation")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--serial", metavar="DEV", help="control console over serial")
    g.add_argument("--telnet", metavar="HOST[:PORT]", help="control console over TCP/telnet")
    ap.add_argument("--mock", action="store_true",
                    help="mock build (fake ADC, no real PWM) — also run the PWM/charger commands")
    ap.add_argument("--include-network", action="store_true",
                    help="also run the NVS/Wi-Fi-mutating commands (wifi on, hostname)")
    args = ap.parse_args()

    if args.serial:
        transport, is_sock = SerialTransport(args.serial, timeout=0.2), False
        print(f"control=serial {args.serial}")
    else:
        host, _, port = args.telnet.partition(":")
        transport, is_sock = SocketTransport(host, port=int(port or 23)), True
        print(f"control=telnet {args.telnet}")

    con = Console(transport, wait_banner=is_sock)
    res = Results()
    try:
        run_plan(con, res, args.mock, args.include_network)
    finally:
        con.close()

    print("\n" + ("ALL CHECKS PASSED" if res.ok() else "SOME CHECKS FAILED"))
    return 0 if res.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
