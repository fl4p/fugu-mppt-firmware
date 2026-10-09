#!/usr/bin/env python3
"""Decode the firmware's binary symbol-table wire protocol back to InfluxDB line
protocol and forward it. Decoder lives in etc/fugu/teledec.py.

UDP mode (device tele.conf::binary=1): datagram = <compressor_id:1B><payload>, id 0=raw 1=tamp.
A symbol table is kept per source device (UDP source ip:port).

BLE mode (firmware built with CONFIG_FUGU_WITH_BLE_TELE): connects to the NUS console,
sends `set-time` + `tele-ble 1`, then decodes <0x7E><varint len><cid><payload> records
from the TELE notify characteristic.

Usage:
  influx_binary_proxy.py --listen 0.0.0.0:8086              # decode + print
  influx_binary_proxy.py --ble fugu-esp32s3                 # BLE by device name
  influx_binary_proxy.py --ble AA:BB:.. --address           # BLE by MAC
  influx_binary_proxy.py ... --influx https://influx.fabi.me:8086 --db open_pe \\
      --user openpe --password ... --precision ms           # decode + forward
  influx_binary_proxy.py --test path/to/blob.bin            # offline self-test
"""
import argparse, socket, sys, time

def use_bluek():
    """Shadow bleak with fl4p/bluek (kernel-direct, no bluetoothd D-Bus races; connections die
    with the process). Needs bluek's duplicate-reporting scanner (Start Service Discovery with
    a no-op filter) — plain mgmt discovery gets kernel-deduped to one report per scan cycle,
    which starves continuous observers."""
    if sys.platform == 'linux':
        try:
            import bluek.shadow  # noqa: F401
        except ImportError:
            pass

sys.path.insert(0, __file__.rsplit('/', 1)[0])  # etc/ on the path -> fugu package
from fugu.teledec import decode_payload, TeleStream, tamp


def adapter_kw(args):
    """--adapter for the scanners. Unset means hci0, which can be another process's controller
    (a bumble HCI_USER_CHANNEL collector owns it exclusively and discovery is refused)."""
    return {'adapter': args.adapter} if args.adapter else {}


def make_fwd(args):
    if args.forward_udp:
        # Feed decoded lines to another line-protocol UDP listener (e.g. the rpi's
        # influxdb-udp-relay on 127.0.0.1:8086, which owns batching + InfluxDB auth).
        host, _, port = args.forward_udp.rpartition(':')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dst = (host or '127.0.0.1', int(port))
        def fwd_udp(lines):
            for i in range(0, len(lines), 8):   # stay under the relay's 2 KB recv buffer
                sock.sendto(('\n'.join(lines[i:i + 8])).encode(), dst)
        return fwd_udp
    if not args.influx:
        return None
    import urllib.request, base64
    url = f"{args.influx}/write?db={args.db}&precision={args.precision}"
    hdr = {}
    if args.user:
        hdr['Authorization'] = 'Basic ' + base64.b64encode(f'{args.user}:{args.password}'.encode()).decode()
    def fwd(lines):
        data = '\n'.join(lines).encode()
        req = urllib.request.Request(url, data=data, headers=hdr, method='POST')
        try: urllib.request.urlopen(req, timeout=5).read()
        except Exception as e: print(f"forward error: {e}", file=sys.stderr)
    return fwd


def emit(fwd, lines):
    if not lines:
        return
    if fwd: fwd(lines)
    else:
        for ln in lines: print(ln)


def self_test(path):
    blob = open(path, 'rb').read()
    print(f"raw frames: {len(blob)} B")
    for cid, label in ((0, 'raw'), (1, 'tamp')):
        if cid == 1 and tamp is None:
            print("(skip tamp: not installed)"); continue
        payload = tamp.compress(blob) if cid == 1 else blob
        dg = bytes([cid]) + payload
        lines = list(decode_payload(dg[0], dg[1:], {}))
        print(f"\nid={cid} ({label}): datagram {len(dg)} B -> {len(lines)} points")
        for ln in lines[:3]:
            print("  " + ln)


def serve_udp(args, fwd):
    host, _, port = args.listen.rpartition(':')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host or '0.0.0.0', int(port)))
    print(f"listening udp {host or '0.0.0.0'}:{port}", file=sys.stderr)
    tables = {}
    while True:
        dg, src = sock.recvfrom(65535)
        if not dg:
            continue
        tab = tables.setdefault(src, {})   # full (ip,port): NAT shares ip, distinct ports
        try:
            lines = list(decode_payload(dg[0], dg[1:], tab))
        except Exception as e:
            print(f"decode error from {src}: {e}", file=sys.stderr); continue
        emit(fwd, lines)


def serve_ble_all(args, fwd):
    """One service for every fugu device in range: scan for NUS/fugu-* advertisers, run one
    worker (connection + enable sequence + decode) per device, re-discover on loss."""
    use_bluek()
    import queue, threading, asyncio
    from fugu.transport import BleTransport
    from fugu.console import Console
    from bleak import BleakScanner

    lineq = queue.Queue()

    def sender():   # single sink thread: notify callbacks never block on HTTP/UDP
        while True:
            lines = [lineq.get()]
            while not lineq.empty():
                lines.append(lineq.get_nowait())
            emit(fwd, lines)
    threading.Thread(target=sender, daemon=True).start()

    active = {}
    lock = threading.Lock()
    # Connects (incl. Console.reconnect inside command(retry=True)) self-serialize via
    # BleTransport.connect_serialize; the discovery scan below must hold the same lock.

    def worker(name, addr):
        stream = TeleStream(on_error=lambda e: print(f"[{name}] decode error: {e}", file=sys.stderr))
        con = None
        try:
            t = BleTransport(name=name, address=addr,
                             tele_cb=lambda data: [lineq.put(ln) for ln in stream.feed(data)])
            con = Console(t)   # opens the transport (scan + connect, serialized in open())
            if not con.wait_ready():   # let the log backlog flush + conn params apply
                raise RuntimeError("console not ready")
            for cmd in (f"set-time {int(time.time() * 1000)}", "tele-ble 1"):
                r = con.command(cmd, timeout=10.0, retry=True)
                print(f"[{name}] {cmd} -> {'OK' if r.ok else 'ERR'}", file=sys.stderr)
                if not r.ok:
                    raise RuntimeError(f"enable failed: {cmd}")
            while True:   # health probe; raises / times out when the link is dead
                time.sleep(60)
                if not con.command("tele-ble", timeout=10.0, retry=True).ok:
                    raise RuntimeError("stream probe failed")
        except Exception as e:
            print(f"[{name}] worker exit: {e}", file=sys.stderr)
        finally:
            if con is not None:
                try: con.close()
                except Exception: pass
            with lock:
                active.pop(addr, None)

    async def scan():
        nus = BleTransport.NUS_SERVICE
        devs = await BleakScanner.discover(timeout=12.0, return_adv=True, **adapter_kw(args))
        out = []
        for d, adv in devs.values():
            nm = d.name or adv.local_name or ""
            if nus in (u.lower() for u in (adv.service_uuids or [])) or nm.lower().startswith("fugu-"):
                out.append((nm or d.address, d.address))
        return out

    print("scanning for fugu devices (BLE)…", file=sys.stderr)
    while True:
        try:
            with BleTransport.connect_serialize:
                found = asyncio.run(scan())
        except Exception as e:
            print(f"scan error: {e}", file=sys.stderr)
            found = []
        for name, addr in found:
            with lock:
                if addr in active:
                    continue
                print(f"discovered {name} [{addr}]", file=sys.stderr)
                th = threading.Thread(target=worker, args=(name, addr), daemon=True)
                active[addr] = th
                th.start()
        time.sleep(args.scan_interval)


def serve_adv(args, fwd):
    """Connectionless: decode the telemetry broadcast (WITH_BLE_ADV) from advertising
    manufacturer data — any number of observers, no connection, works while another host
    holds the (exclusive) NUS link. Observer stamps the time. Record: see src/tele/tele_adv.h."""
    use_bluek()
    import asyncio, struct, queue, threading
    from bleak import BleakScanner

    lineq = queue.Queue()

    def sender():
        while True:
            lines = [lineq.get()]
            while not lineq.empty():
                lines.append(lineq.get_nowait())
            emit(fwd, lines)
    threading.Thread(target=sender, daemon=True).start()

    names, lastseq = {}, {}

    def on_adv(dev, adv):
        blob = (adv.manufacturer_data or {}).get(0xFFFF)
        if not blob or len(blob) != 17 or blob[0] != 0xF7:
            return
        nm = adv.local_name or getattr(dev, 'name', None)
        if nm:
            names[dev.address] = nm   # name arrives via scan response (connectable phases only)
        if lastseq.get(dev.address) == blob[1]:
            return                    # same record re-broadcast (adv interval < adv_ms)
        lastseq[dev.address] = blob[1]
        _, seq, ui, uo, i, p, mcu, ntc, duty, lag, state = struct.unpack('<BB4e2b2HB', blob)
        # tag = hostname, matching the NUS/UDP telemetry series (BLE name is "fugu-<hostname>";
        # the name rides the adv payload every 8th slot). Until it arrives (<=4 s), drop the
        # record — a MAC fallback would fragment the series on every bridge restart.
        name = names.get(dev.address)
        if not name:
            return
        name = name[5:] if name.startswith('fugu-') and len(name) > 5 else name
        ts = int(time.time() * 1000)
        temps = ''.join(f",{k}={v:.1f}" for k, v in (('mcu_temp', mcu), ('ntc_temp', ntc))
                        if v != -128)   # -128 = NaN sentinel (e.g. no NTC fitted)
        lineq.put(f"mppt,device={name} Ui={ui:.2f},Uo={uo:.2f},I={i:.3f},P={p:.2f}{temps},"
                  f"pwm_duty={duty}i,lag={lag}i,"
                  f"mppt_state={state & 0xF}i,cv_lim_idx={state >> 4}i {ts}")
        if args.verbose:
            print(f"[{name}] seq={seq} P={p:.1f}W Ui={ui:.1f} Uo={uo:.1f}", file=sys.stderr)

    async def run():
        scanner = BleakScanner(detection_callback=on_adv, **adapter_kw(args))
        await scanner.start()
        print("observing telemetry broadcasts (BLE adv, mfr id 0xFFFF)…", file=sys.stderr)
        while True:
            await asyncio.sleep(3600)

    asyncio.run(run())


def serve_ble(args, fwd):
    use_bluek()
    import queue
    from fugu.transport import BleTransport
    from fugu.console import Console
    stream = TeleStream(on_error=lambda e: print(f"decode error: {e}", file=sys.stderr))
    lineq = queue.Queue()
    # The notify callback runs on bleak's event-loop thread: only decode + enqueue there.
    # The blocking InfluxDB POST (up to 5 s on a dead host) runs on the main thread below —
    # inline it would stall all BLE I/O, including the console chars.
    t = BleTransport(name=None if args.address else args.ble,
                     address=args.ble if args.address else None,
                     tele_cb=lambda data: [lineq.put(ln) for ln in stream.feed(data)])
    con = Console(t)   # opens the transport
    print("connected, enabling telemetry stream", file=sys.stderr)
    for cmd in (f"set-time {int(time.time() * 1000)}", "tele-ble 1"):
        r = con.command(cmd)
        print(f"{cmd} -> {'OK' if r.ok else 'ERR'}: {r.text.strip()}", file=sys.stderr)
        if not r.ok:
            con.close(); sys.exit(1)
    try:
        while True:
            lines = [lineq.get()]
            while not lineq.empty():
                lines.append(lineq.get_nowait())
            emit(fwd, lines)
    except KeyboardInterrupt:
        try: con.command("tele-ble 0")
        except Exception: pass
        con.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--listen', default='0.0.0.0:8086')
    ap.add_argument('--ble', metavar='NAME', help='pull over BLE from this device instead of UDP')
    ap.add_argument('--ble-all', action='store_true',
                    help='pull over BLE from EVERY fugu/NUS device in range (continuous discovery)')
    ap.add_argument('--adv', action='store_true',
                    help='observe the connectionless telemetry broadcast (WITH_BLE_ADV, no connection)')
    ap.add_argument('--verbose', action='store_true', help='--adv: print each decoded record')
    ap.add_argument('--scan-interval', type=int, default=30, help='--ble-all rescan period (s)')
    ap.add_argument('--adapter', metavar='hciN|MAC',
                    help='Linux controller for --adv/--ble-all scans; a MAC survives re-enumeration')
    ap.add_argument('--address', action='store_true', help='--ble arg is a MAC address, not a name')
    ap.add_argument('--forward-udp', metavar='HOST:PORT',
                    help='forward decoded lines as UDP line-protocol datagrams (e.g. to a local relay)')
    ap.add_argument('--influx'); ap.add_argument('--db', default='open_pe')
    ap.add_argument('--user'); ap.add_argument('--password', default='')
    ap.add_argument('--precision', default='ms')
    ap.add_argument('--test', metavar='BLOB')
    a = ap.parse_args()
    fwd = make_fwd(a)
    if a.test: self_test(a.test)
    elif a.adv: serve_adv(a, fwd)
    elif a.ble_all: serve_ble_all(a, fwd)
    elif a.ble: serve_ble(a, fwd)
    else: serve_udp(a, fwd)
