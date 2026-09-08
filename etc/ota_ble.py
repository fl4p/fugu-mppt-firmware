#!/usr/bin/env python3
"""Push a firmware image to a Fugu device over BLE (no WiFi).

Mirrors the trigger/poll structure of ota.py, but the host *pushes* the image over the BLE NUS link
instead of the device pulling it over HTTP. The device must run a WITH_BLE build with the `ble`
service enabled.

The link is either a direct bleak connection (default) or — with `--ble-proxy HOST` — an ESPHome
`bluetooth_proxy` (active connections), which lets you OTA a device that's only in range of the
proxy, not the host. The proxy path talks the ESPHome native API (plaintext, no noise PSK) and
reuses the same NUS RX/TX + OTA FW characteristics over the proxied GATT connection.

Protocol (device side in src/etc/ota_ble.cpp):
  host -> RX  : "ota-ble begin <size> <sha256hex>\n" arm + erase passive partition
  dev  -> TX  : "OTAB READY ..."                    erased, ready to receive
  dev  -> TX  : "OTAB CRED <G>"                      may stream up to cumulative byte offset G
  host -> FW  : raw firmware bytes (write-no-response), throttled to the credit window
  dev  -> TX  : "OTAB PROG <w>/<size>" ...           progress
  host -> RX  : "ota-ble end\n"                       finalize
  dev  -> TX  : "OTAB OK rebooting" | "OTAB FAIL <reason>"

Usage:
    python -m etc.ota_ble [-f|--force] [build/fugu-firmware.bin] [device-name-or-address]
    python -m etc.ota_ble -n fry build/fugu-firmware.bin                   # name as a flag
    python -m etc.ota_ble --ble-proxy 192.168.1.231 --name fry            # via ESPHome proxy (by name)
    python -m etc.ota_ble --ble-proxy 192.168.1.231 --address AA:BB:..    # via ESPHome proxy (by MAC)

Skips the push if the device already reports the image's version (probed via `uptime` over the
BLE console); pass --force to push regardless. Requires: pip install bleak (and aioesphomeapi for
--ble-proxy).
"""
import argparse
import asyncio
import hashlib

# The link/protocol module ships with the device-side receiver.
import os as _os, sys as _sys
_HERE = _os.path.dirname(_os.path.abspath(__file__))
for _cand in (_os.path.join(_HERE, "..", "..", "esp-ota-ble", "host"),
              _os.path.expanduser("~/dev/pv/esp-ota-ble/host"),
              _HERE):
    if _os.path.isfile(_os.path.join(_cand, "esp_ota_ble.py")):
        _sys.path.insert(0, _cand)
        break
import esp_ota_ble as O
import os
import re
import struct
import sys

NUS_SVC = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write: console commands
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify: status/log mirror
FW_UUID = "6e400004-b5a3-f393-e0a9-e50e24dcca9e"  # write-no-response: firmware bytes
ESPHOME_API_PORT = 6053

APP_DESC_MAGIC = 0xABCD5432
RE_APP_LINE = re.compile(r'App:\s+(\S+)\s+v\S+\s+(\S+)\s+\(built (.+),\s+IDF\s+(\S+)\)')

# Strings present only in a WITH_BLE build: the otab receiver + NUS console (both #ifdef WITH_BLE) and
# the NimBLE host (absent when CONFIG_BT_ENABLED is off). Flashing an image missing all of these over
# BLE would disable the very console/OTA path we're using — so we warn before doing it.
BLE_SIGNATURES = (b"OTAB CRED", b"OTAB READY", b"(NUS console)", b"BLE_HS")


def image_has_ble(data):
    """Heuristic: does this firmware image look like a BLE-enabled (WITH_BLE) build?"""
    return any(sig in data for sig in BLE_SIGNATURES)


def read_local_app_desc(bin_path):
    """Parse the firmware version string out of a .bin's esp_app_desc_t, or None."""
    try:
        with open(bin_path, 'rb') as f:
            data = f.read(0x200)
    except FileNotFoundError:
        return None
    off = data.find(struct.pack('<I', APP_DESC_MAGIC))
    if off < 0:
        return None
    return data[off + 0x10:off + 0x30].split(b'\x00', 1)[0].decode('utf-8', 'replace')


def parse_device_version(lines):
    """Pull the version from the App: line of an `uptime` reply, or None."""
    for l in lines:
        if m := RE_APP_LINE.search(l):
            return m.group(2)
    return None


def progress_bar(done, total, label, width=30):
    frac = done / total if total else 0
    filled = int(frac * width)
    bar = "#" * filled + "-" * (width - filled)
    sys.stdout.write(f"\r  {label} [{bar}] {frac * 100:5.1f}% {done}/{total}")
    sys.stdout.flush()
    if done >= total:
        sys.stdout.write("\n")
        sys.stdout.flush()


class BleakLink:
    """Direct bleak GATT link to the device's NUS + OTA FW characteristics."""

    def __init__(self, name_or_addr):
        self.target = name_or_addr
        self.mtu = 23
        self.disconnected = asyncio.Event()
        self._cli = None
        self._cb = None

    def set_notify(self, cb):
        self._cb = cb

    @staticmethod
    async def _find(name_or_addr):
        from bleak import BleakScanner
        if name_or_addr and re.fullmatch(r"[0-9A-Fa-f:\-]{11,}", name_or_addr):
            return await BleakScanner.find_device_by_address(name_or_addr, timeout=15)
        # Scan once and match locally. bleak's find_device_by_name keys off the advertised local_name,
        # which CoreBluetooth frequently omits (it sets the cached d.name instead) — so it misses devices
        # that discover() plainly sees. Match against both names here.
        items = list((await BleakScanner.discover(timeout=15, return_adv=True)).values())
        names = lambda d, ad: [n for n in ((d.name or ""), (ad.local_name or "")) if n]
        if name_or_addr:
            hits = [d for d, ad in items if name_or_addr in names(d, ad)]  # prefer exact
            if not hits:
                hits = [d for d, ad in items if any(name_or_addr in n for n in names(d, ad))]
            if len(hits) > 1:
                lst = ", ".join(f"{d.name} [{d.address}]" for d in hits)
                raise RuntimeError(f"{name_or_addr!r} matches several devices — be more specific: {lst}")
            return hits[0] if hits else None
        # No name given: restrict to fugu peripherals (firmware always advertises a `fugu-` name) so we
        # don't grab some other NUS device. Refuse to guess when several are in range — one may be a live
        # converter — and make the user disambiguate with a name/address.
        fugu = [d for d, ad in items
                if NUS_SVC in (s.lower() for s in (ad.service_uuids or []))
                and (d.name or ad.local_name or "").startswith("fugu-")]
        if len(fugu) > 1:
            lst = ", ".join(f"{d.name} [{d.address}]" for d in fugu)
            raise RuntimeError(f"multiple fugu devices in range — pass a name to pick one: {lst}")
        return fugu[0] if fugu else None

    async def open(self):
        from bleak import BleakClient
        dev = await self._find(self.target)
        if not dev:
            raise RuntimeError("no fugu BLE device found (advertising? ble service enabled?)")
        self.target = dev.address  # pin to this exact device so verify() re-finds it after reboot
        print(f"connecting to {dev.name or dev.address} ...")
        # macOS/CoreBluetooth intermittently rejects connect or the notify subscription with CBATTError 17
        # ("resources are insufficient") when a prior connection's CCCD wasn't released — dropping the link
        # and retrying after a short settle clears it. Retry the whole connect+subscribe a few times.
        last = None
        for attempt in range(1, 4):
            self._cli = BleakClient(dev, disconnected_callback=lambda _: self.disconnected.set())
            try:
                await self._cli.connect()
                await self._cli.start_notify(TX_UUID, lambda _, p: self._cb(bytes(p)))
                # Ask BlueZ for the MTU it negotiated. Without this bleak reports the
                # 23-byte default and adapt_link() then chunks at 20 bytes: measured
                # 2026-09-08, a 1.76 MB push from a Pi took 6m33s where the same image
                # from a Mac took 64 s. usable_chunk() caps the result, which is what
                # makes asking safe -- see acquire_bluez_mtu().
                await O.acquire_bluez_mtu(self._cli)
                self.mtu = self._cli.mtu_size
                self.disconnected.clear()  # a failed attempt's disconnect may have set it
                return
            except Exception as e:
                last = e
                print(f"  connect attempt {attempt}/3 failed: {e}")
                try:
                    await self._cli.disconnect()
                except Exception:
                    pass
                self._cli = None
                if attempt < 3:
                    await asyncio.sleep(2.0)
        raise RuntimeError(f"could not establish BLE link after 3 attempts: {last}")

    async def write_cmd(self, data):
        await self._cli.write_gatt_char(RX_UUID, data, response=True)

    async def write_fw(self, data):
        await self._cli.write_gatt_char(FW_UUID, data, response=False)

    async def release(self):
        if self._cli and self._cli.is_connected:
            try:
                await self._cli.disconnect()
            except Exception:
                pass

    async def verify(self):
        for _ in range(20):
            await asyncio.sleep(2)
            if await self._find(self.target):
                return True
        return False

    async def aclose(self):
        await self.release()


class ProxyLink:
    """NUS + OTA FW link reached through an ESPHome `bluetooth_proxy` (active connections).

    Talks the ESPHome native API (aioesphomeapi, plaintext) to a proxy at <host:port>, opens an
    active GATT connection to the target peripheral by MAC (`address`) or by scanning the proxy's
    advertisements for `name`/NUS, and exposes the same write_cmd/write_fw/notify surface as
    BleakLink over the proxied connection.
    """

    def __init__(self, host, port, password, address, name, address_type=0):
        self.host = host
        self.port = port
        self.password = password or ""
        self.address = address
        self.name = name
        self.address_type = address_type
        self.mtu = 23
        self.disconnected = asyncio.Event()
        self._api = None
        self._addr = None
        self._cb = None
        self._stop_notify = None
        self._closing = False
        self._rx = self._tx = self._fw = None

    def set_notify(self, cb):
        self._cb = cb

    @staticmethod
    def _mac_to_int(mac):
        return int(str(mac).replace(":", "").replace("-", ""), 16)

    @staticmethod
    def _parse_adv(data):
        """Parse BLE AD structures → (local_name, [128-bit-uuid-str, …]).

        Modern ESPHome proxies forward raw advertisement bytes; pull out the local name (AD types
        0x08/0x09) and the 128-bit service-UUID lists (0x06/0x07) so name/NUS matching still works.
        """
        name, uuids = "", []
        i, n = 0, len(data)
        while i + 1 < n:
            ln = data[i]
            if ln == 0 or i + 1 + ln > n:
                break
            typ, val = data[i + 1], data[i + 2:i + 1 + ln]
            if typ in (0x08, 0x09):
                name = val.decode("utf-8", "replace")
            elif typ in (0x06, 0x07):
                for off in range(0, len(val) - 15, 16):
                    h = val[off:off + 16][::-1].hex()
                    uuids.append(f"{h[:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:]}")
            i += 1 + ln
        return name, uuids

    async def _scan(self, timeout=15.0):
        """Scan the proxy's raw advertisements; return (address, address_type, name)."""
        want = (self.name or "").lower()
        fut = asyncio.get_running_loop().create_future()

        def on_raw(resp):
            if fut.done():
                return
            for a in resp.advertisements:
                name, uuids = self._parse_adv(bytes(a.data))
                ok = (want in name.lower()) if want else (NUS_SVC in uuids)
                if ok:
                    fut.set_result((a.address, a.address_type, name))
                    return

        unsub = self._api.subscribe_bluetooth_le_raw_advertisements(on_raw)
        try:
            return await asyncio.wait_for(fut, timeout)
        finally:
            unsub()

    async def open(self):
        from aioesphomeapi import APIClient, BluetoothProxyFeature
        self._api = APIClient(self.host, self.port, self.password or None)
        await self._api.connect(login=True)
        info = await self._api.device_info()
        flags = info.bluetooth_proxy_feature_flags_compat(self._api.api_version)
        if not flags & BluetoothProxyFeature.ACTIVE_CONNECTIONS:
            raise RuntimeError(
                f"{self.host} is a passive bluetooth_proxy — needs `bluetooth_proxy: active: true`")
        if self.address:
            self._addr, addr_type = self._mac_to_int(self.address), self.address_type
        else:
            self._addr, addr_type, dev_name = await self._scan()
            print(f"  proxy {self.host} saw {dev_name}")
        await self._bringup(flags, addr_type)

    async def _bringup(self, flags, addr_type):
        # bluetooth_device_connect resolves on the first connection response (success OR failure),
        # so capture the outcome+MTU via the state callback and re-raise a failure ourselves.
        state = asyncio.get_running_loop().create_future()

        def on_state(connected, mtu, error):
            self.mtu = mtu or self.mtu
            if not state.done():
                if connected:
                    state.set_result(True)
                else:
                    state.set_exception(RuntimeError(f"BLE connect rejected (gatt error {error})"))
            elif not connected and not self._closing:
                self.disconnected.set()

        await self._api.bluetooth_device_connect(
            self._addr, on_state, timeout=20, feature_flags=flags,
            has_cache=False, address_type=addr_type)
        await state

        svcs = await self._api.bluetooth_gatt_get_services(self._addr)
        for s in svcs.services:
            for c in s.characteristics:
                u = c.uuid.lower()
                if u == RX_UUID:
                    self._rx = c.handle
                elif u == TX_UUID:
                    self._tx = c.handle
                elif u == FW_UUID:
                    self._fw = c.handle
        if None in (self._rx, self._tx, self._fw):
            raise RuntimeError("peripheral missing NUS RX/TX or OTA FW characteristic")
        self._stop_notify, _ = await self._api.bluetooth_gatt_start_notify(
            self._addr, self._tx, lambda h, d: self._cb(bytes(d)))

    async def write_cmd(self, data):
        await self._api.bluetooth_gatt_write(self._addr, self._rx, data, True)

    async def write_fw(self, data):
        await self._api.bluetooth_gatt_write(self._addr, self._fw, data, False)

    async def release(self):
        # Drop the peripheral GATT link (it's about to reboot) but keep the API up for verify().
        self._closing = True
        for coro in (self._stop_notify() if self._stop_notify else None,
                     self._api.bluetooth_device_disconnect(self._addr)
                     if (self._api and self._addr is not None) else None):
            if coro is None:
                continue
            try:
                await coro
            except Exception:
                pass
        self._stop_notify = None

    async def verify(self):
        if not self._api:
            return False
        self._closing = True
        try:
            await self._scan(timeout=40)
            return True
        except Exception:
            return False

    async def aclose(self):
        self._closing = True
        if self._api:
            try:
                await self._api.disconnect()
            except Exception:
                pass
            self._api = None


async def push(bin_path, link, force=False, assume_yes=False):
    data = open(bin_path, "rb").read()
    sha = hashlib.sha256(data).hexdigest()
    local_ver = read_local_app_desc(bin_path)
    print(f"image {bin_path}: {len(data)} bytes, sha256={sha}, version={local_ver}")

    # Pushing a non-BLE image over BLE bricks the BLE console/OTA on the device (serial or WiFi to
    # recover) — confirm before doing something self-defeating.
    if not image_has_ble(data):
        print("⚠️  this image looks like a build WITHOUT BLE (no NUS console / otab receiver found).")
        print("    Flashing it over BLE will disable the BLE console and BLE-OTA on the device —")
        print("    you'd then need serial or WiFi to recover it.")
        if assume_yes:
            print("    -y/--yes given, proceeding anyway.")
        elif not sys.stdin.isatty():
            print("    non-interactive; aborting (pass -y/--yes to override).")
            return False
        elif input("    Continue anyway? [y/N] ").strip().lower() not in ("y", "yes"):
            print("    aborted.")
            return False

    # THE PROTOCOL NOW COMES FROM THE SHARED MODULE in the esp-ota-ble repo -
    # the same repo as the device-side receiver this is talking to. What is
    # left here is fugu's own: the BLE-image guard above, the version probe
    # below, and the two transports, one of which is an ESPHome bluetooth
    # proxy that the module deliberately does not know about.
    #
    # adapt_link() bridges this file's transport shape (set_notify with raw
    # payloads) to the module's, so ProxyLink keeps working untouched.
    ml = O.adapt_link(link)

    rx_lines = []

    def line(text):
        rx_lines.append(text)
        print("  <", text)

    ml.set_line_handler(line)

    try:
        await link.open()
    except Exception as e:
        print(f"link setup failed: {e}")
        await link.aclose()
        return False

    print(f"connected, mtu={ml.mtu}, chunk={ml.chunk}")

    try:
        await ml.write_cmd("ping")

        # Skip the push if the device already runs this exact version.
        rx_lines.clear()
        await ml.write_cmd("uptime")
        await asyncio.sleep(2)
        dev_ver = parse_device_version(rx_lines)
        print(f"  device version: {dev_ver or '?'}")
        if local_ver and dev_ver == local_ver and not force:
            print(f"  \u2611\ufe0f skip: already at {local_ver} (use --force to push anyway)")
            return True

        ml.set_line_handler(line)
        ok = await O.push_image(
            ml, data, sha=sha, cmd_prefix="ota-ble ",
            on_line=line,
            on_progress=lambda s, t: progress_bar(s, t, "upload"))
        if ok:
            print("\ndevice accepted the image and is rebooting; waiting for it to advertise again")
            if await link.verify():
                print("  device is advertising again")
            else:
                # FAIL, do not just warn. Before the module conversion this
                # returned False here, and it must keep doing so: the shared
                # push_image() treats a link drop after `end` as success, which
                # is a guess - the receiver selects the boot slot AFTER the last
                # PROG - so re-advertising is the only evidence this tool has
                # that the device came back at all. Printing a warning and
                # returning success made that guess authoritative.
                print("  device did NOT come back within the wait - check it")
                return False
        return ok
    except O.OtaBleError as exc:
        print(f"\nOTA over BLE failed: {exc}")
        return False
    finally:
        await link.aclose()



def make_link(args):
    if args.ble_proxy:
        host, _, port = args.ble_proxy.partition(":")
        port = int(port) if port else ESPHOME_API_PORT
        target = args.address or f"name~{(args.name or 'fugu')!r}"
        print(f"OTA via ESPHome bluetooth_proxy {host}:{port} → BLE NUS ({target})")
        return ProxyLink(host, port, args.proxy_password, args.address, args.name or "fugu")
    return BleakLink(args.address or args.name)


def main():
    ap = argparse.ArgumentParser(description="Push firmware to a Fugu device over BLE")
    ap.add_argument("bin", nargs="?", default="build/fugu-firmware.bin",
                    help="firmware image (default build/fugu-firmware.bin)")
    ap.add_argument("name", nargs="?", default=None,
                    help="device name (or BLE address) filter (positional; or use -n/--name)")
    ap.add_argument("-n", "--name", dest="name_opt", metavar="NAME",
                    default=os.environ.get("BLE_NAME"),
                    help="device name (or BLE address) filter (default $BLE_NAME)")
    ap.add_argument("-f", "--force", action="store_true",
                    help="push even if the device already reports this version")
    ap.add_argument("-y", "--yes", action="store_true",
                    help="skip the confirmation prompt for a non-BLE image")
    ap.add_argument("--ble-proxy", metavar="HOST[:PORT]",
                    help="reach the device through an ESPHome bluetooth_proxy at this host "
                         "(plaintext API, no noise); scans by name unless --address is given")
    ap.add_argument("--address", help="target BLE address/MAC (direct or via --ble-proxy)")
    ap.add_argument("--proxy-password", default=os.environ.get("ESPHOME_API_PASSWORD", ""),
                    help="ESPHome API password for --ble-proxy (default: $ESPHOME_API_PASSWORD)")
    args = ap.parse_args()
    args.name = args.name or args.name_opt  # positional takes precedence over -n/--name/$BLE_NAME

    if not os.path.exists(args.bin):
        print(f"no such file: {args.bin}"); return 1
    ok = asyncio.run(push(args.bin, make_link(args), force=args.force, assume_yes=args.yes))
    print("OTA over BLE:", "✅ success (device rebooting)" if ok else "❌ failed")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
