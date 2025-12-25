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

if the chip is stuck in a boot loop or hands otherwise, and you don't have access to a physical reset button,
interrupt power supply and use `idf.py monitor` Ctrl+T Ctrl+P to boot the chip into bootloader

# usb

- even if logging is set to
- menuconfig
    - ESP_CONSOLE_USB_CDC_SUPPORT_ETS_PRINTF(=n) "Enable esp_rom_printf / ESP_EARLY_LOG via USB CDC"

# adc

* ESP32: 2 Msps
* ESP32S3 (and all other boards): 100 ksps

# ledcpwm

supports duty cycle fading
ledc: esp32 has high-speed mode
"This mode is implemented in hardware and offers automatic and glitch-free changing of the PWM duty cycle."
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html#introduction
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html#ledc-api-high-low-speed-mode
https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#ledpwm

# mcpwm

https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#mcpwm


# rf comm

espnow vs ble
https://www.reddit.com/r/esp32/comments/j2j1df/is_espnow_or_ble_more_power_efficient/
https://pastebin.com/uuGtCAZj