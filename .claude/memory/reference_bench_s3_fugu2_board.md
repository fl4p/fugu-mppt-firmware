---
name: Bench ESP32-S3 fugu-esp32s3-* is a Fugu2 board
description: Bench ESP32-S3 (MAC 34:85:18:82:40:28, hostname fugu-esp32s3-284082188534) is a Fugu2 board (same pins as fmetal); INA226 at 0x40 works, internal ADC works after atten-normalization fix
created: 2026-08-12T21:23:52.897Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: reference
  originSessionId: ses_00826dadfffeNbdVew49f9Z8v4
---

Bench ESP32-S3 device: MAC `34:85:18:82:40:28`, hostname `fugu-esp32s3-284082188534`, serial port `/dev/cu.usbmodem1101` (subject to macOS USB-JTAG re-enumeration — see [[macos_esp32s3_usb_jtag_port_enumeration]]).

Hardware: Fugu2 board (same pin mapping as `config/fmetal`):
- HiLi gate driver: pwm_hi=21, pwm_li=14, pwm_sd=47
- I2C: sda=42, scl=2, 800kHz
- INA226: addr=0x40, alert=41, confirmed working (452 SPS, DeviceID 0x2260)
- panel_sd=40, LED WS2812 pin=1
- Internal ADC: channels 3 (voltage divider) and 7 (NTC) — works after the uniform-attenuation fix ([[project_adc_uniform_atten_requirement]])

Previously mis-diagnosed as having "broken internal ADC" — the actual issue was mixed ADC attenuation (IDF requires uniform atten per ADC unit). Not broken.

For boost topology configs on this board, use INA226 for Vin (LV side) and internal ADC for Vout (HV side) — see `config/solar-boost` and `config/psu/boost80V`.
