
mkdir -p web-flash

env=e2_esp32dev

pio run -e $env

"$HOME/.platformio/penv/bin/python" "$HOME/.platformio/packages/tool-esptoolpy@1.40500.0/esptool.py" \
  --chip esp32 merge_bin \
  -o web-flash/$env.bin \
  --flash_mode dio \
  --flash_freq 40m \
  --flash_size 4MB \
  0x1000 .pio/build/e2_esp32dev/bootloader.bin \
  0x8000 .pio/build/e2_esp32dev/partitions.bin \
  0xe000 $HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/$env/firmware.bin



env=e1_esp32s3dev

chip=esp32s3
variant=adafruit_feather_esp32s3
arduino_esp_sdk=$HOME/.platformio/packages/framework-arduinoespressif32

pio run -e $env

"$HOME/.platformio/penv/bin/python" "$HOME/.platformio/packages/tool-esptoolpy@1.40500.0/esptool.py" \
 --chip $chip merge_bin \
  -o web-flash/$env.bin \
  --flash_mode dio \
  --flash_freq 80m \
  --flash_size 4MB \
  0x0000 $arduino_esp_sdk/variants/adafruit_feather_esp32s3/bootloader-tinyuf2.bin \
  0x8000 .pio/build/$env/partitions.bin \
  0xe000 $arduino_esp_sdk/tools/partitions/boot_app0.bin \
  0x2d0000 $arduino_esp_sdk/variants/$variant/tinyuf2.bin \
  0x10000 .pio/build/$env/firmware.bin

