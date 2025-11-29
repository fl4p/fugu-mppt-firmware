
"""

1. build
2. start web server to server firmware binary
3. discover hosts
4 iterate hosts
    > ota http://192.168.1.129:9000/build/fugu-firmware.bin 

idf.py build
#  python3 -m http.server 9000

"""""
import re
import time

from etc.fugu.discover import discover_scope_servers
from etc.fugu.fugu import FuguDevice
from etc.fugu.transport import SocketTransport

import http.server

hosts = discover_scope_servers()

print(hosts)

listening = set()

for ip,port,name in hosts:
    print('\n', name)
    st = SocketTransport(ip)
    fd = FuguDevice(st, block=True, prefix=name)
    fd.verbose = True
    ota_progress = 0
    def on_message(rx):
        global ota_progress
        m = re.match(r'.+ota: Download Progress:\s*([0-9.]+)\s*%.+', rx)
        if m:
            ota_progress = float(m[1])
    fd.on_message = on_message
    private_ip, *_  = st.sock.getsockname()
    if private_ip not in listening:
        #http.server.SimpleHTTPRequestHandler()
        listening.add(private_ip)
    #fd.wait_for_pwm_state()
    fd.write(f"ota http://{private_ip}:9000/build/fugu-firmware.bin\n")
    print(fd.pwm_state)
    while ota_progress < 100:
        time.sleep(.1)
    fd.close()
    #print('update installed, waiting for device to come online again..')
    #fd = FuguDevice(SocketTransport(ip), block=True, prefix=name)