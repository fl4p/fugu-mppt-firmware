#!/usr/bin/env python3
"""Console client for the Fugu MPPT firmware.

Talks the device's string command protocol (the same one served on UART/USB-CDC/telnet/MQTT/BLE,
see `website/docs/reference/console.md`) over any transport. Modes: one command or several (`-c`, repeatable; or
`--stdin` to read newline-separated commands — both run over a single connection; stdin batch mode
is auto-selected when no mode flag is given and stdin is piped), or — given a transport but no mode
flag — an interactive REPL (the default). With *no arguments at all* it scans every transport for
reachable devices (serial port globs, mDNS scope/telnet hosts, the NAT-forwarded telnet endpoints
in `nat.env`, local BLE NUS, BLE NUS seen through the ESPHome bluetooth_proxy in `$BLE_PROXY`, and —
when `$MQTT_HOST` is set — hostnames seen on the broker's `pv/log/`) and prints what to pass to
connect, without connecting. The transport and the line-console mechanics live in the `fugu`
package (`fugu.transport`, `fugu.console.Console`); this file is the CLI.
Defaults to serial; `--ble`/`--ip` select BLE or TCP/telnet, `--ble-proxy HOST` reaches BLE NUS
through an ESPHome bluetooth_proxy (plaintext API, no noise encryption).

The PASS/FAIL/SKIP command exerciser that used to be this client's `--test`/`--mock` mode now lives
in the e2e suite: `python etc/e2e-test/test_console_plan.py --serial … [--mock] [--include-network]`.

Requires `pyserial` (serial) and/or `bleak` (BLE):  pip install pyserial bleak

Examples:
    python etc/fugu_console.py                           # discover devices on all transports
    python etc/fugu_console.py -p /dev/cu.usbmodem1101   # interactive REPL over serial (default)
    python etc/fugu_console.py --ip 192.168.4.2          # interactive REPL over TCP/telnet
    python etc/fugu_console.py --ble                     # interactive REPL over BLE
    python etc/fugu_console.py --ble-proxy 192.168.1.50  # BLE via ESPHome bluetooth_proxy (by name)
    python etc/fugu_console.py --ble-proxy 192.168.1.50 --address AA:BB:CC:DD:EE:FF  # by MAC
    python etc/fugu_console.py --mqtt 192.168.1.134 --mqtt-port 1882 -c "svc list"   # over MQTT
    python etc/fugu_console.py --mqtt 192.168.1.134 --mqtt-readonly  # passive log monitor (REPL)
    python etc/fugu_console.py -c "svc list"             # run one command, print the reply
    python etc/fugu_console.py --ip 192.168.4.2 -c status -c sensor   # several commands, one connection
    python etc/fugu_console.py --ble --name fry --stdin  # commands from stdin (one per line), one connection
"""

import argparse
import asyncio
import glob
import os
import re
import signal
import sys
import threading
import time

try:  # works both as `python etc/fugu_console.py` and `python -m etc.fugu_console`
    from fugu.transport import (SerialTransport, SocketTransport, BleTransport,
                                EspHomeBleTransport, MqttTransport)
    from fugu.console import Console
    from fugu.discover import discover_scope_servers
except ImportError:
    from etc.fugu.transport import (SerialTransport, SocketTransport, BleTransport,
                                    EspHomeBleTransport, MqttTransport)
    from etc.fugu.console import Console
    from etc.fugu.discover import discover_scope_servers

# `peek <symbol>` resolver + `sym <pattern>` need the build ELF; the implementation lives in the
# vendored idf-devtools submodule (generic ESP-IDF symbol lookup).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'idf-devtools'))
import peek_symbols  # noqa: E402

_PORT_GLOBS = [
    "/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*", "/dev/cu.SLAB_USBtoUART*",
    "/dev/ttyUSB*", "/dev/ttyACM*",
]


def load_env_file(path=None):
    """Populate os.environ from a KEY=VALUE file (default `mqtt.env` beside this script).

    Shell-set vars win — only keys not already in the environment are filled, so an explicit
    `MQTT_HOST=… fugu_console.py` or exported value overrides the file.
    """
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mqtt.env")
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, val = line.partition("=")
                os.environ.setdefault(key.strip(), val.strip())
    except FileNotFoundError:
        pass


def autodetect_port() -> str:
    if os.environ.get("ESPPORT"):
        return os.environ["ESPPORT"]
    for pat in _PORT_GLOBS:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    sys.exit("no serial port found; pass --port or set $ESPPORT")


def scan_serial():
    """Glob the serial ports, then probe each for its hostname (`hn`) concurrently.

    Returns a list of (port, hostname|None); None means the port is busy or didn't answer.
    """
    from concurrent.futures import ThreadPoolExecutor
    ports = []
    for pat in _PORT_GLOBS:
        ports.extend(sorted(glob.glob(pat)))
    if not ports:
        return []
    with ThreadPoolExecutor(max_workers=len(ports)) as ex:
        names = ex.map(probe_serial_hostname, ports)
    return list(zip(ports, names))


_ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
_WELCOME_RE = re.compile(r"Welcome to (\S+)")  # banner: "Welcome to <hostname> (192.168.4.2)"
_HOSTNAME_RE = re.compile(r"Hostname:\s*(\S+)")  # reply of the bare `hostname` command


def query_hostname(con: Console) -> str | None:
    """Ask the device for its hostname (the `hn` command), or None on failure."""
    for ln in con.command("hn", timeout=2.0):
        m = _HOSTNAME_RE.search(ln)
        if m:
            return m.group(1)
    return None


def probe_serial_hostname(port, baud=115200, timeout=2.0) -> str | None:
    """Briefly open a serial port, run `hn`, and return the device hostname (or None if the
    port is busy / not a Fugu / unresponsive within `timeout`)."""
    try:
        con = Console(SerialTransport(port, baud=baud, timeout=0.2))
    except Exception:
        return None
    try:
        return query_hostname(con)
    finally:
        con.close()


def nat_endpoints():
    """Telnet endpoints behind the NAT router, from `$NAT_TELNET` (comma-separated host:port).

    Filled from `nat.env` beside this script (see the device table in CLAUDE.md). Returns a list
    of (host, port); empty when unset.
    """
    out = []
    if not os.environ.get("NAT_TELNET"):
        load_env_file(os.path.join(os.path.dirname(os.path.abspath(__file__)), "nat.env"))  # NAT_TELNET
    for ep in (os.environ.get("NAT_TELNET") or "").split(","):
        ep = ep.strip()
        if not ep:
            continue
        host, _, port = ep.partition(":")
        out.append((host.strip(), int(port) if port else SocketTransport.DEFAULT_PORT))
    return out


async def probe_welcome(host, port, timeout=4.0):
    """Connect to a telnet endpoint and return the hostname from its welcome banner (or None)."""
    try:
        writer: asyncio.StreamWriter
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(host, port), timeout)
    except (OSError, asyncio.TimeoutError):
        return None
    buf = b""
    deadline = time.monotonic() + timeout
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                chunk = await asyncio.wait_for(reader.read(1024), remaining)
            except (OSError, asyncio.TimeoutError):
                break
            if not chunk:
                break
            buf += chunk
            m = _WELCOME_RE.search(_ANSI.sub("", buf.decode("utf-8", "replace")))
            if m:
                return m.group(1)
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except OSError:
            pass
    return None


async def scan_nat_async(timeout=4.0, reachable_only=False):
    """asyncio twin of `scan_nat`: probe all NAT endpoints concurrently.

    Returns a list of (host, port, hostname|None) for every configured endpoint (None = no
    banner / unreachable), or None when none are configured.
    """
    endpoints = nat_endpoints()
    if not endpoints:
        return []
    names = await asyncio.gather(
        *(probe_welcome(host, port, timeout) for host, port in endpoints))
    return [(host, port, name) for (host, port), name in zip(endpoints, names) if
            not reachable_only or name is not None]


def scan_nat(timeout=4.0):
    """Probe the NAT-forwarded telnet endpoints in `nat.env`, resolving each hostname.

    Returns a list of (host, port, hostname|None) for every configured endpoint (None = no
    banner / unreachable), or None when none are configured.
    """
    return asyncio.run(scan_nat_async(timeout))


def scan_telnet(timeout=2.0):
    """mDNS-advertised scope/telnet hosts; None if zeroconf isn't installed."""
    try:
        return sorted(set(discover_scope_servers(timeout=timeout)))
    except ImportError:
        return None


def scan_ble(timeout=5.0):
    """BLE peripherals advertising the NUS console; None if bleak isn't installed."""
    try:
        from bleak import BleakScanner
    except ImportError:
        return None
    nus = BleTransport.NUS_SERVICE

    async def _scan():
        found = []
        for dev, adv in (await BleakScanner.discover(timeout=timeout, return_adv=True)).values():
            if nus in (s.lower() for s in (adv.service_uuids or [])) \
                    or BleTransport.is_tele_adv(adv.manufacturer_data):
                found.append((dev.name or "?", dev.address))
        return found

    return asyncio.run(_scan())


def scan_mqtt(timeout=5.0):
    """ Returns Hostnames a broker has seen publishing under `pv/log/<hostname>`.

    Needs a broker (no default is compiled in): `$MQTT_HOST` (`$MQTT_PORT`/`$MQTT_USER`/`$MQTT_PASS`
    refine it). Returns None when no broker is configured or `paho-mqtt` isn't installed.
    """
    host = os.environ.get("MQTT_HOST")
    if not host:
        return None
    try:
        import paho.mqtt.client as mqtt
        from paho.mqtt.enums import CallbackAPIVersion
    except ImportError:
        return None
    hosts = set()

    def on_message(_c, _u, msg):
        parts = msg.topic.split("/")
        if len(parts) == 3:  # pv/log/<hostname>, not the deeper .../cmd echo
            hosts.add(parts[2])

    c = mqtt.Client(CallbackAPIVersion.VERSION2)
    user = os.environ.get("MQTT_USER", "pv")
    if user:
        c.username_pw_set(user, os.environ.get("MQTT_PASS") or "")
    c.on_message = on_message
    try:
        c.connect(host, int(os.environ.get("MQTT_PORT", "1883")), 60)
        c.subscribe(MqttTransport.LOG_ROOT + "#")
        c.loop_start()
        time.sleep(timeout)
        c.loop_stop()
        c.disconnect()
    except OSError as e:
        print(f"  (mqtt {host}: {e})")
        return []
    return sorted(hosts)


def scan_ble_proxy(timeout=8.0):
    """BLE NUS devices seen through the ESPHome bluetooth_proxy in `$BLE_PROXY` (host[:port]).

    Filled from `nat.env` beside this script. Connects to the proxy over the plaintext native API,
    collects raw advertisements, and returns a list of (name, mac, address_type) for every NUS
    peripheral in range. None when no `$BLE_PROXY` is set or `aioesphomeapi` isn't installed.
    """
    if not os.environ.get("BLE_PROXY"):
        load_env_file(os.path.join(os.path.dirname(os.path.abspath(__file__)), "nat.env"))  # BLE_PROXY
    proxy = os.environ.get("BLE_PROXY")
    if not proxy:
        return None
    try:
        from aioesphomeapi import APIClient
    except ImportError:
        return None
    host, _, port = proxy.partition(":")
    port = int(port) if port else EspHomeBleTransport.API_PORT

    def mac(a):
        return ":".join(f"{(a >> (8 * i)) & 0xff:02x}" for i in reversed(range(6)))

    async def _scan():
        cli = APIClient(host, port, None)  # plaintext, no noise PSK
        try:
            # aioesphomeapi's own connect timeout is ~60 s; bound it to our budget so an
            # unreachable proxy can't stall the whole discovery scan.
            await asyncio.wait_for(cli.connect(login=True), timeout)
        except Exception as e:
            print(f"  (ble-proxy {proxy}: {e})")
            return []
        found = {}

        def on_raw(resp):
            for a in resp.advertisements:
                name, uuids, mfr = EspHomeBleTransport._parse_adv(bytes(a.data))
                if EspHomeBleTransport.NUS_SERVICE in uuids or BleTransport.is_tele_adv(mfr):
                    found[a.address] = (name, a.address_type)

        unsub = cli.subscribe_bluetooth_le_raw_advertisements(on_raw)
        try:
            await asyncio.sleep(timeout)
        finally:
            unsub()
            await cli.disconnect()
        return [(n or "?", mac(a), at) for a, (n, at) in sorted(found.items())]

    return asyncio.run(_scan())


def _print_serial(ports):
    print("\nserial:")
    for p, name in ports:
        label = name or "(no reply / busy)"
        print(f"  {label:<24} {p:<32} →  -p {p}")
    if not ports:
        print("  (none)")


def _print_telnet(hosts):
    print("\ntelnet/scope (mDNS):")
    if hosts is None:
        print("  (skipped — `pip install zeroconf`)")
    else:
        for addr, port, name in hosts:
            print(f"  {name} {addr}:{port:<8} →  --ip {addr}")
        if not hosts:
            print("  (none)")


def _print_nat(nat):
    print("\ntelnet via NAT (nat.env):")
    if nat is None:
        print("  (skipped — set $NAT_TELNET, or edit etc/nat.env)")
    else:
        for host, port, name in nat:
            label = name or "(unreachable)"
            print(f"  {label:<24} {host}:{port:<8} →  --ip {host}:{port}")
        if not nat:
            print("  (none)")


def _print_ble(devs):
    print("\nBLE (NUS):")
    if devs is None:
        print("  (skipped — `pip install bleak`)")
    else:
        for name, address in devs:
            print(f"  {name:<24} {address}  →  --ble --address {address}")
        if not devs:
            print("  (none)")


def _print_ble_proxy(proxy_devs):
    print("\nBLE via ESPHome proxy ($BLE_PROXY):")
    if proxy_devs is None:
        print("  (skipped — set $BLE_PROXY in nat.env, and `pip install aioesphomeapi`)")
    else:
        proxy = os.environ.get("BLE_PROXY")
        for name, address, _atype in proxy_devs:
            print(f"  {name:<24} {address}  →  --ble-proxy {proxy} --address {address}")
        if not proxy_devs:
            print("  (none)")


def _print_mqtt(mqtt_hosts):
    print("\nMQTT (pv/log):")
    if mqtt_hosts is None:
        print("  (skipped — set $MQTT_HOST, or pass --mqtt to connect)")
    else:
        broker = os.environ.get("MQTT_HOST")
        for h in mqtt_hosts:
            print(f"  {h:<24} →  --mqtt {broker} --name {h}")
        if not mqtt_hosts:
            print("  (none)")


def discover_devices():
    """Scan every transport for reachable Fugu devices and print what to pass to connect.

    The scans block for different durations (serial is instant, mDNS ~2 s, BLE/MQTT/proxy ~5-8 s),
    so run them concurrently and print each section the moment its scan returns (completion order,
    fast transports first) instead of joining all of them. Total time is still the slowest scan,
    but useful results appear immediately rather than after the slowest one finishes.
    """
    from concurrent.futures import ThreadPoolExecutor, as_completed
    print("scanning for Fugu devices on all transports …")

    jobs = [
        (scan_serial, _print_serial),
        (scan_telnet, _print_telnet),
        (scan_nat, _print_nat),
        (scan_ble, _print_ble),
        (scan_ble_proxy, _print_ble_proxy),
        (scan_mqtt, _print_mqtt),
    ]
    # The BLE/socket scans block in worker threads that can't be cancelled synchronously, so a
    # KeyboardInterrupt in the main thread can't unwind the executor's join (Ctrl+C would otherwise
    # hang until the slowest scan returns). Install a SIGINT handler that exits immediately — the
    # scan is read-only, nothing to clean up.
    def _on_sigint(*_):
        print("\nscan interrupted", file=sys.stderr, flush=True)
        os._exit(130)

    prev = signal.signal(signal.SIGINT, _on_sigint)
    try:
        with ThreadPoolExecutor(max_workers=len(jobs)) as ex:
            futs = {ex.submit(scan): printer for scan, printer in jobs}
            for fut in as_completed(futs):
                futs[fut](fut.result())
    finally:
        signal.signal(signal.SIGINT, prev)
    return 0


# Per-command timeout overrides (seconds), keyed by command verb (first token). The default 4 s
# fits most commands; the I2C bus scan is slower, and `ota <url>` blocks until the firmware
# finishes downloading and reboots (or its 10 s connect timeout fires + recovery) — ~40 s for a
# successful flash, so 180 s gives comfortable headroom.
TIMEOUT_OVERRIDE = {"scan-i2c": 12.0, "ota": 180.0, "curl": 15.0, "ping": 8.0, "tcpconnect": 8.0, "run": 180.0}


def _timeout_for(cmd: str, default: float = 4.0) -> float:
    verb = cmd.split(None, 1)[0] if cmd else cmd
    if verb == "sleep":
        try:
            return min(max(float(cmd.split(None, 1)[1]) + 2.0, default), 62.0)
        except (IndexError, ValueError):
            return default
    return TIMEOUT_OVERRIDE.get(verb, default)


_PEEK_TYPED_RE = re.compile(r"peek\s+0x[0-9a-fA-F]+\s+=\s+0x([0-9a-fA-F]+)")
_PEEK_DUMP_RE = re.compile(r"0x[0-9a-fA-F]+:\s*((?:[0-9a-fA-F]{2}(?:\s+|$))+)")


def _parse_peek_bytes(reply_lines, want_bytes: int) -> bytes:
    """Parse one `peek` reply into raw little-endian bytes — handles typed + dump formats."""
    buf = bytearray()
    for ln in reply_lines:
        m = _PEEK_TYPED_RE.search(ln)
        if m:
            val = int(m.group(1), 16)
            return val.to_bytes(want_bytes, 'little')
        m = _PEEK_DUMP_RE.search(ln)
        if m:
            for tok in m.group(1).split():
                if len(tok) == 2:
                    buf.append(int(tok, 16))
    return bytes(buf[:want_bytes])


def _chunked_peek(con: Console, addr: int, size: int, chunk: int = 256) -> bytes:
    """Read `size` bytes from `addr` via repeated peek calls; reassemble in order."""
    out = bytearray()
    while len(out) < size:
        n = min(chunk, size - len(out))
        cmd = f"peek 0x{addr + len(out):08x} {n}"
        reply = con.command(cmd, timeout=_timeout_for(cmd))
        if not reply.ok:
            raise RuntimeError(f"peek {cmd!r} failed: {reply.text.strip()[:200]}")
        out.extend(_parse_peek_bytes(reply, n))
    return bytes(out[:size])


def _handle_peek_struct(con: Console, args: str, elf_path: str | None) -> None:
    parts = args.strip().split()
    if not parts:
        print("peek-struct: expected <symbol>[.field…] [depth]", file=sys.stderr)
        return
    target = parts[0]
    depth = 2
    if len(parts) >= 2:
        try:
            depth = int(parts[1])
        except ValueError:
            print(f"peek-struct: bad depth {parts[1]!r}", file=sys.stderr)
            return
        if not 0 <= depth <= 16:
            print("peek-struct: depth out of range [0,16]", file=sys.stderr)
            return
    if not elf_path:
        print("peek-struct: no firmware ELF — build, set $FUGU_ELF, or pass --elf",
              file=sys.stderr)
        return
    try:
        addr, size, type_die, _ = peek_symbols.resolve_for_dump(elf_path, target)
    except (KeyError, ValueError) as e:
        print(f"peek-struct: {e}", file=sys.stderr)
        return
    if type_die.tag not in ('DW_TAG_structure_type', 'DW_TAG_class_type',
                            'DW_TAG_union_type'):
        print(f"peek-struct: {target} is not a struct/class/union "
              f"({peek_symbols._type_name(type_die)}); use `peek {target}` instead.",
              file=sys.stderr)
        return
    if size <= 0:
        print(f"peek-struct: {target} has zero size", file=sys.stderr)
        return
    if size > 4096:
        print(f"peek-struct: {target} is {size} B ({(size + 255) // 256} round-trips)",
              file=sys.stderr)
    try:
        image = _chunked_peek(con, addr, size)
    except RuntimeError as e:
        print(f"peek-struct: {e}", file=sys.stderr)
        return
    print(peek_symbols.format_struct_dump(elf_path, target, image, addr, max_depth=depth))


def resolve_command_mode(explicit_cmds, use_stdin, want_coredump, stdin_is_tty):
    """Decide how a non-discovery run dispatches commands. Pure (no I/O) so it's unit-testable.

    Returns (read_stdin, active, delimit):
      active     — run a command sequence and exit; False falls through to --coredump/REPL.
      read_stdin — append newline-separated stdin lines to the command list.
      delimit    — tag each reply with `=== cmd ===` (when running more than a lone `-c`).
    With no mode flag and stdin not a terminal (piped/heredoc), batch from stdin instead of the REPL.
    """
    auto_batch = (not explicit_cmds and not use_stdin
                  and not want_coredump and not stdin_is_tty)
    read_stdin = use_stdin or auto_batch
    active = bool(explicit_cmds) or read_stdin
    delimit = use_stdin or auto_batch or len(explicit_cmds) > 1
    return read_stdin, active, delimit


def dispatch_command(con: Console, cmd: str, elf_path: str | None) -> None:
    """Send `cmd` to the device, intercepting host-side verbs first.

    `sym <pattern>` never goes on the wire — it lists matching ELF symbols locally.
    `peek <symbol>[+off] [len]` is rewritten to `peek 0x<addr> <len>` before send.
    `peek-struct <symbol>[.field…]` reads the byte image via repeated `peek`s and renders
    each DWARF-described member with its decoded value.
    """
    head = cmd.strip().split(None, 1)
    verb = head[0] if head else ""
    if verb == "sym":
        pattern = head[1] if len(head) > 1 else ""
        print(peek_symbols.format_sym_list(elf_path, pattern))
        return
    if verb == "peek-struct":
        _handle_peek_struct(con, head[1] if len(head) > 1 else "", elf_path)
        return
    if verb == "peek":
        try:
            cmd = peek_symbols.preprocess_peek(cmd, elf_path)
        except (KeyError, FileNotFoundError, ValueError) as e:
            print(f"peek: {e}", file=sys.stderr)
            return
    for ln in con.command(cmd, timeout=_timeout_for(cmd)):
        print(ln)


def _install_readline_history(path: str = "~/.fugu_console_history", maxlen: int = 1000) -> None:
    """Persist input history across runs (up-arrow recall). No-op if readline is missing."""
    try:
        import atexit
        import readline
    except ImportError:
        return
    path = os.path.expanduser(path)
    try:
        readline.read_history_file(path)
    except (FileNotFoundError, OSError):
        pass
    readline.set_history_length(maxlen)
    atexit.register(lambda: _safe_write_history(readline, path))


def _safe_write_history(readline_mod, path: str) -> None:
    try:
        readline_mod.write_history_file(path)
    except OSError:
        pass


def _install_readline_completion(elf_path: str | None) -> None:
    """Tab-complete symbol names after `peek ` or `sym `. Silently no-ops if readline missing."""
    try:
        import readline
    except ImportError:
        return

    def completer(text: str, state: int):
        buf = readline.get_line_buffer()
        if not (buf.startswith("peek ") or buf.startswith("peek-struct ")
                or buf.startswith("sym ")):
            return None
        try:
            syms = peek_symbols.load_symbols(elf_path) if elf_path else {}
        except Exception:
            return None
        prefix = [s for s in syms if s.startswith(text)]
        if len(prefix) < 50 and text:
            prefix.extend(s for s in syms if text in s and not s.startswith(text))
        prefix.sort()
        return prefix[state] if state < len(prefix) else None

    readline.set_completer(completer)
    # default delim set includes '-' which appears in command names — narrow it to whitespace
    readline.set_completer_delims(" \t\n")
    readline.parse_and_bind("tab: complete")


class _PromptSafePrinter:
    """Print log lines from the reader thread without trampling the readline prompt.

    on_line fires both while `input()` is in flight (user typing — must not clobber) and between
    calls (during `con.command()` — no prompt on screen). The `active` flag gates the redraw so
    we only do the carriage-return / erase / reprint dance when there's actually a prompt to
    preserve.

    `last_command` defeats a libedit quirk (macOS Python's readline): `get_line_buffer()` keeps
    returning the previously *accepted* line while idle at the next prompt, so every status line
    would redraw `prompt + "<last cmd>"` until the user starts typing. We blank the buffer while
    it still equals that command — the user hasn't typed anything new yet.

    The trailing prompt is only re-drawn once the line stream goes quiet (`REDRAW_IDLE`). Erasing
    it costs nothing on a well-behaved TTY, but a device that floods (a protection trip looping at
    sample rate) emits hundreds of lines a second, and terminals that commit the pre-erase row to
    scrollback instead of dropping it — the JetBrains terminal does — then interleave a prompt copy
    between every log line. Deferring the redraw keeps a flood readable everywhere; the prompt is
    still restored as soon as the device stops talking, and readline redraws it on the first
    keypress regardless.
    """

    REDRAW_IDLE = 0.15  # s of silence before the prompt is painted back

    def __init__(self):
        self.prompt = ""
        self.active = False
        self.last_command = ""
        self._istty = sys.stdout.isatty()  # redraw only makes sense on a real terminal
        self._lock = threading.Lock()
        self._pending = None  # Timer that repaints the prompt once the flood stops

    def __call__(self, line: str) -> None:
        with self._lock:
            # The \r/\x1b[K dance erases the floating prompt before the next line, but only on a
            # TTY. Piped/captured (tee, file, ssh), those escapes are no-ops and each redraw would
            # commit another `prompt + buffer` copy — print plainly instead.
            if self.active and self._istty:
                try:
                    import readline
                except ImportError:
                    pass
                else:
                    if self._pending is not None:
                        self._pending.cancel()
                    sys.stdout.write("\r\x1b[K" + line + "\n")
                    sys.stdout.flush()
                    self._pending = threading.Timer(self.REDRAW_IDLE, self._redraw_prompt)
                    self._pending.daemon = True
                    self._pending.start()
                    return
            print(line)

    def _redraw_prompt(self) -> None:
        with self._lock:
            self._pending = None
            if not self.active:
                return
            try:
                import readline
                buf = readline.get_line_buffer()
            except ImportError:
                buf = ""
            if buf == self.last_command:  # stale leftover, not fresh typing
                buf = ""
            sys.stdout.write("\r\x1b[K" + self.prompt + buf)
            sys.stdout.flush()


def interactive(con: Console, elf_path: str | None = None):
    print("interactive console — type commands, Ctrl-C / EOF to quit (live output streams below)")
    hostname = query_hostname(con)
    prompt = f"{hostname}> " if hostname else "> "
    _install_readline_history()
    _install_readline_completion(elf_path)
    # Tap the reader's line stream so periodic status lines (and command replies) print as they
    # arrive, instead of being thrown away by command()'s drain() while we wait at the prompt.
    redisp = _PromptSafePrinter()
    redisp.prompt = prompt
    con.on_line = redisp
    try:
        while True:
            redisp.active = True
            try:
                raw = input(prompt)
            except (EOFError, KeyboardInterrupt):
                print()
                return
            finally:
                redisp.active = False
            redisp.last_command = raw  # libedit keeps reporting this until the next line is typed
            cmd = raw.strip()
            if not cmd:
                continue
            head = cmd.split(None, 1)
            verb = head[0] if head else ""
            if verb == "sym":
                pattern = head[1] if len(head) > 1 else ""
                print(peek_symbols.format_sym_list(elf_path, pattern))
                continue
            if verb == "peek-struct":
                # on_line will keep echoing status lines while chunked reads run, which is fine
                _handle_peek_struct(con, head[1] if len(head) > 1 else "", elf_path)
                continue
            if verb == "peek":
                try:
                    cmd = peek_symbols.preprocess_peek(cmd, elf_path)
                except (KeyError, FileNotFoundError, ValueError) as e:
                    print(f"peek: {e}", file=sys.stderr)
                    continue
            con.command(cmd)  # reply lines print via on_line
    finally:
        con.on_line = None


def pull_coredump(con, action, elf_path):
    """`coredump info|get|erase` over the console. `get` collects the base64 stream the firmware
    emits between ==COREDUMP-BEGIN size=N== / ==COREDUMP-END==, decodes it to coredump.bin, and
    runs esp-coredump (the streamed bytes are the raw partition image, hence --core-format raw)."""
    action = action or "info"
    if action in ("info", "erase"):
        rep = con.command(f"coredump {action}", timeout=8.0)
        print(rep.text or "(no reply)")
        return 0 if rep.ok else 1
    if action != "get":
        print(f"coredump: unknown action {action!r} (use info|get|erase)")
        return 2

    import base64, shutil, subprocess
    print("requesting core dump (base64 stream over the console, may take a while)…")
    rep = con.command("coredump get", timeout=180.0)
    size, b64, capturing = None, [], False
    for ln in rep:
        m = re.search(r"==COREDUMP-BEGIN size=(\d+)==", ln)
        if m:
            size, b64, capturing = int(m.group(1)), [], True
            continue
        if "==COREDUMP-END==" in ln:
            capturing = False
            continue
        if capturing:
            s = ln.strip()
            if s and re.fullmatch(r"[A-Za-z0-9+/=]+", s):  # skip any interleaved status lines
                b64.append(s)
    if size is None:
        print("coredump: no dump in reply (try `--coredump info`; device may have none)")
        print(rep.text)
        return 1
    data = base64.b64decode("".join(b64))
    if len(data) != size:
        print(f"coredump: WARNING decoded {len(data)} B, header said {size} B")
    out = "coredump.bin"
    with open(out, "wb") as f:
        f.write(data)
    print(f"coredump: wrote {len(data)} bytes to {out}")
    if not shutil.which("esp-coredump"):
        print("esp-coredump not on PATH — in the ESP-IDF env (. ./idf-export.sh) run:")
        print(f"  esp-coredump info_corefile --core-format raw -c {out} "
              f"{elf_path or 'build/fugu-firmware.elf'}")
        return 0
    # Without an explicit --elf, let the ELF archive match the dump to its build by the
    # app-SHA the dump embeds — esp-coredump is SHA-gated and a guessed build/ rarely matches.
    archiver = os.path.join(os.path.dirname(__file__), "idf-devtools", "elf_archive.py")
    if not elf_path and os.path.exists(archiver):
        cmd = [sys.executable, archiver, "decode", out]
        print("+ " + " ".join(cmd))
        if subprocess.run(cmd).returncode == 0:
            return 0
        print("elf-archive could not match this dump; falling back to build/ ELF")
    elf = elf_path or "build/fugu-firmware.elf"
    cmd = ["esp-coredump", "info_corefile", "--core-format", "raw", "-c", out, elf]
    print("+ " + " ".join(cmd))
    subprocess.run(cmd)
    return 0


def make_transport(args):
    if args.ble_proxy:
        host, _, port = args.ble_proxy.partition(":")
        port = int(port) if port else EspHomeBleTransport.API_PORT
        target = args.address or f"name~{args.name!r}"
        print(f"connecting via ESPHome bluetooth_proxy {host}:{port} → BLE NUS ({target})")
        return EspHomeBleTransport(host, proxy_port=port, password=args.proxy_password,
                                   address=args.address, name=args.name)
    if args.ble:
        print(f"scanning for BLE NUS (name contains {args.name!r}) …")
        tele_cb = None
        if getattr(args, "tele", False):
            try:
                from fugu.teledec import TeleStream
            except ImportError:
                from etc.fugu.teledec import TeleStream
            stream = TeleStream(on_error=lambda e: print(f"tele decode error: {e} (resync)",
                                                         file=sys.stderr))
            tele_cb = lambda data: [print("tele| " + ln) for ln in stream.feed(data)]
        return BleTransport(name=args.name, address=args.address, tele_cb=tele_cb,
                            adapter=args.adapter)
    if args.mqtt:
        ro = " (read-only)" if args.mqtt_readonly else ""
        print(f"connecting to MQTT broker {args.mqtt}:{args.mqtt_port}, device ~{args.name!r}{ro}")
        return MqttTransport(args.mqtt, port=args.mqtt_port,
                             username=args.mqtt_user, password=args.mqtt_pass,
                             device=args.name, writable=not args.mqtt_readonly)
    if args.ip:
        host, _, port = args.ip.partition(":")  # host:port for NAT-forwarded endpoints
        port = int(port) if port else SocketTransport.DEFAULT_PORT
        print(f"connecting to {host}:{port} (telnet)")
        return SocketTransport(host, port=port)
    port = args.port or autodetect_port()
    print(f"opening {port} @ {args.baud}")
    return SerialTransport(port, baud=args.baud, timeout=0.2)


def main():
    load_env_file()  # fill MQTT_* from etc/mqtt.env (shell-set vars still win)
    ap = argparse.ArgumentParser(description="Console client + exerciser for Fugu MPPT firmware")
    ap.add_argument("-p", "--port", default=None, help="serial port (default: $ESPPORT or autodetect)")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="baud rate (default 115200)")
    ap.add_argument("--ble", action="store_true", help="use the BLE NUS transport instead of serial")
    ap.add_argument("--name", default="fugu",
                    help="device-name substring filter (with --ble: advertised name; with --mqtt: hostname)")
    ap.add_argument("--address", help="BLE address to connect to (with --ble / --ble-proxy)")
    ap.add_argument("--adapter", default=None, metavar="hciN",
                    help="BlueZ adapter for --ble (default $BLE_ADAPTER, else hci0). A host whose\n"
                         "on-board controller shares its antenna with wlan0 connects far more\n"
                         "reliably on a separate dongle.")
    ap.add_argument("--ble-proxy", metavar="HOST[:PORT]",
                    help="reach BLE NUS through an ESPHome bluetooth_proxy at this host "
                         "(plaintext API, no noise); scans by --name unless --address is given")
    ap.add_argument("--proxy-password", default=os.environ.get("ESPHOME_API_PASSWORD", ""),
                    help="ESPHome API password for --ble-proxy (default: $ESPHOME_API_PASSWORD)")
    ap.add_argument("--ip", help="use TCP/telnet to this address instead of serial")
    ap.add_argument("--mqtt", metavar="BROKER", help="use MQTT via this broker host instead of serial")
    ap.add_argument("--mqtt-port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")),
                    help="MQTT broker port (default: $MQTT_PORT or 1883)")
    ap.add_argument("--mqtt-user", default=os.environ.get("MQTT_USER", "pv"), help="MQTT username")
    ap.add_argument("--mqtt-pass", default=os.environ.get("MQTT_PASS"), help="MQTT password")
    ap.add_argument("--mqtt-readonly", action="store_true",
                    help="read-only MQTT monitor: stream output, never publish commands")
    ap.add_argument("-c", "--command", action="append", metavar="CMD",
                    help="send a command and print the reply, then exit; repeat -c to run several "
                         "commands over one connection")
    ap.add_argument("--stdin", action="store_true",
                    help="read newline-separated commands from stdin and run them over one "
                         "connection (blank lines and # comments skipped); exits on EOF. "
                         "Auto-enabled when no mode flag is given and stdin is not a terminal")
    ap.add_argument("--elf", default=None,
                    help="firmware ELF for `peek <symbol>`/`sym` resolution (default: $FUGU_ELF or "
                         "newest build*/fugu-firmware.elf)")
    ap.add_argument("--tele", action="store_true",
                    help="with --ble: subscribe the telemetry stream (sends set-time + tele-ble 1) "
                         "and print decoded points as 'tele| <influx line>' (needs "
                         "CONFIG_FUGU_WITH_BLE_TELE firmware)")
    ap.add_argument("--coredump", nargs="?", const="info", metavar="info|get|erase",
                    help="pull the saved panic core dump: `info` (default) shows presence/size, "
                         "`get` streams it (base64), decodes to coredump.bin and runs esp-coredump, "
                         "`erase` clears it")
    args = ap.parse_args()

    if len(sys.argv) == 1:  # no arguments: search every transport, don't connect
        return discover_devices()

    if args.tele and not args.ble:
        print("--tele requires --ble", file=sys.stderr)
        return 1

    try:
        transport = make_transport(args)
        # Telnet drops the first byte sent during the post-connect handshake; wait for the banner.
        con = Console(transport, wait_banner=isinstance(transport, SocketTransport))
    except Exception as e:
        print(e)
        return 1
    elf_path = peek_symbols.find_elf(args.elf)
    try:
        if args.tele:
            for cmd in (f"set-time {int(time.time() * 1000)}", "tele-ble 1"):
                r = con.command(cmd)
                print(f"{cmd} -> {'OK' if r.ok else 'ERR'}: {r.text.strip()}")
                if not r.ok:
                    return 1
        commands = list(args.command or [])
        read_stdin, active, delimit = resolve_command_mode(
            commands, args.stdin, bool(args.coredump), sys.stdin.isatty())
        if read_stdin:
            commands += [ln for ln in (raw.strip() for raw in sys.stdin)
                         if ln and not ln.startswith("#")]
        if active:
            for cmd in commands:
                if delimit:
                    print(f"=== {cmd} ===")
                dispatch_command(con, cmd, elf_path)
            return 0
        if args.coredump:
            return pull_coredump(con, args.coredump, elf_path)

        interactive(con, elf_path)  # default
        return 0
    except (OSError, EOFError, ConnectionError) as e:
        # A mid-session connection drop (e.g. the firmware's single-client telnet server reused
        # while a prior client lingers) should exit cleanly, not dump a traceback.
        print(f"connection error: {e}", file=sys.stderr)
        return 1
    finally:
        con.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
