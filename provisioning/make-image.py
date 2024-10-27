"""

littlefs-python create --block-size 128 --fs-size 262144 fmetal conf.bin
littlefs-python list --block-size 128 conf.bin

parttool.py --port /dev/cu.usbserial-1101 write_partition --partition-name littlefs --input conf.bin

littlefs-python create provisioning/fmetal littlefs.bin -v --fs-size=0x20000 --name-max=64 --block-size=4096 && parttool.py --port /dev/cu.usbserial-1101 write_partition --partition-name littlefs --input littlefs.bin

"""
from littlefs import LittleFS

# Initialize the File System according to your specifications
fs = LittleFS(block_size=128, block_count=256)

# Open a file and write some content
with fs.open('first-file.txt', 'w') as fh:
    fh.write('Some text to begin with\n')

# Dump the filesystem content to a file
with open('FlashMemory.bin', 'wb') as fh:
    fh.write(fs.context.buffer)