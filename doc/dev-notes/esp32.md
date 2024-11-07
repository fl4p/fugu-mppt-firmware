# programming
```
# dump flash
esptool.py --baud 115200 --port COM8 read_flash 0x0 0x100000 fw-backup-1M.bin 

# read partition table (https://iot.stackexchange.com/questions/4287/how-can-i-list-the-partition-table-of-a-currently-running-esp32-devboard
esptool.py read_flash 0x8000 0xc00 ptable.img
(esptool.py read_flash 0x8000 0xc00 /dev/fd/3 >&2) 3>&1|gen_esp32part.py /dev/fd/0
esptool.py read_flash 0x187000  0x40000 fugu-grey-littlefs-data-partition.img

# fugu-grey-littlefs-data-partition.img


# change target
TARGET=esp32s3; rm -rf managed_components build && idf.py fullclean && idf.py set-target $TARGET && idf.py flash

```

# usb
- even if logging is set to
- menuconfig
  - ESP_CONSOLE_USB_CDC_SUPPORT_ETS_PRINTF(=n) "Enable esp_rom_printf / ESP_EARLY_LOG via USB CDC"
