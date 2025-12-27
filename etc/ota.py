fallback_hosts = [
    # (ip, port, name),
    # ('192.168.4.13', 0, 'flat'),
    ('192.168.4.2', 0, 'fry'),
]

"""

1. build
2. start web server to server firmware binary
3. discover hosts
4 iterate hosts
    > ota http://192.168.1.161:9000/build/fugu-firmware.bin 

idf.py build
#  python3 -m http.server 9000

"""""
import asyncio
import re
import sys
import time

from etc.fugu.discover import discover_scope_servers
from etc.fugu.fugu import FuguDevice
from etc.fugu.transport import SocketTransport

hosts = discover_scope_servers()

# hosts = hosts or fallback_hosts

if not hosts:
    print('no hosts discovered!')
    sys.exit(1)

print(hosts)

listening = set()


async def send_ota_command(addr, name):
    print('\n', name)
    st = SocketTransport(addr)
    fd = FuguDevice(st, block=True, prefix=name)
    fd.verbose = True
    ota_progress = 0

    def on_message(rx):
        nonlocal ota_progress
        m = re.match(r'.*ota: Download Progress:\s*([0-9.]+)\s*%.*', rx)
        if m:
            ota_progress = float(m[1])

    fd.on_message = on_message

    private_ip, *_ = st.sock.getsockname()
    if private_ip not in listening:
        # http.server.SimpleHTTPRequestHandler()
        # TODO we currently spawn the http server in ota.sh
        listening.add(private_ip)
    # fd.wait_for_pwm_state()
    fd.write(f"ota http://{private_ip}:9000/build/fugu-firmware.bin\n")
    # fd.write(f"reset\n")
    print(fd.pwm_state)
    while ota_progress < 100:
        if not st.check_connection():
            print(fd.prefix, 'connection to device unexpectedly closed @ota_progress=', ota_progress)
            return False
        await asyncio.sleep(.3)

    print(fd.prefix, 'waiting for device to close the connection..')
    while st.check_connection():
        time.sleep(.02)
    print(fd.prefix, 'closed the connection')
    fd.close()

    # now try to re-connect
    print(fd.prefix, 'waiting for device to come online again')
    for _ in range(10):
        time.sleep(1)
        st = SocketTransport(addr)
        try:
            fd = FuguDevice(st, block=True, prefix=name)
        except (ConnectionRefusedError, TimeoutError):
            continue
        print(fd.prefix, 'device back online! OTA successful (probably TODO check ver)')
        break
    else:
        print(fd.prefix, 'device didnt come online in time')
        return False

    return True


async def main():
    res = await asyncio.gather(*[send_ota_command(ip, name) for ip, port, name in hosts])
    res = dict(zip((name for _, _, name in hosts), res))
    print(res)
    return all(res.values())


sys.exit(0 if asyncio.run(main()) else 1)
