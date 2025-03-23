import ftplib
import os
import time
from os import environ

HOST = environ['HOST']
USER = environ['USERNAME']
PASSWORD = environ['PASSWORD']

print(os.stat('../build/fugu-firmware.elf').st_mtime - time.time())


with ftplib.FTP(HOST, USER, PASSWORD) as session, open('../build/fugu-firmware.bin', 'rb') as file:
    sent = 0
    file.seek(0, os.SEEK_END)
    total = file.tell()
    file.seek(0, os.SEEK_SET)


    def cb(buf):
        global sent
        sent += len(buf)
        print(round(sent / total * 100, 2), '%')

    session.storbinary(f'STOR {HOST}/fugu-firmware.bin', file, callback=cb)  # send the file

print(f'ota http://{HOST}/fugu-firmware.bin')
