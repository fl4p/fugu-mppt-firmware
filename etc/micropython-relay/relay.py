"""
- thread support experimental
- there is uaiohttpclient, but it doesnt support https (http only)

https://github.com/pfalcon/pycopy-lib/tree/master/uaiohttpclient
https://github.com/miguelgrinberg/microdot
"""

import socket

import machine
import network
import urequests

wlan = network.WLAN()
wlan.active(True)

if not wlan.isconnected():
    wlan.scan()
    wlan.connect('mentha', 'modellbau')
    print('connecting..')
    while not wlan.isconnected():
        machine.idle()

print('addr4:', wlan.ipconfig('addr4'))


def influxdb_write(host, db, user, password, data):
    url = f'https://{host}:8086/api/v2/write?bucket={db}/autogen&precision=ms'
    headers = {
        "Authorization": f"Token {user}:{password}",
        "Content-Type": "text/plain; charset=utf-8",
    }
    print('posting', url, headers, len(data))
    response = urequests.post(url, headers=headers, data=data)
    print(response.status_code, response.text)
    #response.close()


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 8086))

buf = b''
while True:
    data, addr = sock.recvfrom(1024 * 2)
    print('received', len(data), 'bytes from', addr)
    if data:
        buf += data
        if len(buf) >= 20000:
            print('sending', len(buf), 'bytes')
            influxdb_write("influx.fabi.me", "open_pe", "openpe", "0ffgrid", buf)
            buf = b''

import mip

mip.install("uaiohttpclient")
import uaiohttpclient as aiohttp
import asyncio


async def req():
    resp = await aiohttp.request("GET", "http://open.pe/")
    r = await resp.read()
    print(len(r))


asyncio.run(req())
