---
name: reference_bench_rigs
description: Bench hardware: what is reachable (11-24 only bench139c + fry/flat), fbuck/fboost IDs + rpi BLE telemetry relay, fboost duty 2499, bench S3 Fugu2 pins, HW Stories, PicoScope ps2000, S3 USB port wedge, rpi BLE adapter
metadata:
  type: reference
---

## Hardware available
2026-11-24, Fab: "fbuck is not here, we only have a bench board without power stage and 2 converters". bench139c runs vconv/mock (no real BF or battery); real-power-stage tests mean fry or flat.
**Why:** I proposed fbuck for a measure-coil test; it had had no BLE/telemetry for 7 days.
**How to apply:** don't plan HW validation on fbuck/fboost; ask Fab before using fry/flat ([[feedback_device_handling]]).

## fbuck/fboost identities
Bench devices (not live converters), flash freely.
- fbuck: macOS BLE UUID `C7513EAF-691F-DA42-411D-755D97AFD6BB` (`fugu-fbuck`), chip ID `fugu-esp32s3-344082188534`, NVS hostname `fbuck`, config `config/lab/buck_bench/`.
- fboost: BLE UUID `C839CEE8-3D60-F1B7-9278-2FF4F4870803` (`fugu-fboost`), no tele.conf.
- Connect: `.venv/bin/python3 etc/fugu_console.py --ble --address <UUID> -c status`.

## fboost duty 2499
2026-08-10: rig acquisition refuses ("monitor-linearity exit 1") unless duty is exactly 2499 counts; set by `tracker.conf::target_duty_cycle=0.6138` (2499/4070), boots in MANU with no PD/MPPT. `mppt` leaves manual (clears boot target).
**Why:** every rig ladder was calibrated at 2499.
**How to apply:** after OTAing fboost check `get-config tracker.conf target_duty_cycle` = 0.6138 and `status` `st=MANU H=2499`; else set-config + `restart`. See [[project_tracker_sweep]].

## fbuck/fboost telemetry
2026-08-10: BLE ADV → rpi.local (use 192.168.178.26, `fab@`; .35 alias was down) `/opt/fugu-ble-bridge/influx_binary_proxy.py --adv --forward-udp 127.0.0.1:8086` → systemd `influxdb-udp-relay` (`/home/fab/influxdb-udp-relay/`) → `influx.fabi.me:8086` (SSL) db `open_pe`. Creds: `etc/influx.env`. Measurement `mppt` tag `device=fbuck|fboost`; rig loggers in `smart_shunt`, `logger_cfg`. fbuck's tele.conf UDP host .180 was unreachable; the BLE relay is the real path. fry/flat use a different DB: [[project_telemetry]].

## Bench S3 board
2026-08-12: MAC `34:85:18:82:40:28`, hostname `fugu-esp32s3-284082188534`, usually `/dev/cu.usbmodem1101`. Fugu2 board, fmetal pins: HiLi pwm_hi=21 pwm_li=14 pwm_sd=47; I2C sda=42 scl=2 800 kHz; INA226 0x40 alert=41 (452 SPS); panel_sd=40; WS2812 pin 1; internal ADC ch3 divider, ch7 NTC. "Broken internal ADC" was mixed attenuation ([[project_adc_sampler]]). Boost configs: INA226 for Vin (LV), internal ADC for Vout (HV), see `config/solar-boost`, `config/psu/boost80V`.

## HW Stories
`/Users/fab/dev/ee/hw/Fugu2/HW Stories/<board>.md` (flat, flu, fboost, ftall, fugu1, banana, …): as-built FETs, gate drive, coil, caps, unfitted parts, dated incidents. Read before reasoning about a board's hardware (e.g. flat's missing Vout divider cap, [[project_backflow_wire_guard]]); add findings there as dated sections.

## PicoScope ps2000
Legacy ps2000 (2204/2205-class; `ps2000a` API rejects it), driven by `etc/pico_capture.py`. Dylibs in `/Applications/PicoScope 7 T&M.app/Contents/Resources/` are x86_64-only: script re-execs under `arch -x86_64 /usr/bin/python3` with that dir on `DYLD_LIBRARY_PATH`. `ps2000_get_timebase` returns 0 if samples exceed buffer (cap 3968; request ≤3000). tb=1 = 50 MS/s (tb 0 needs ETS), doubles per index; `time_interval` always ns. Ranges 20mV=1..20V=10, DC=1/AC=0, trigger NONE=5, MAX_ADC 32767. Clamp: Hantek CC-65, DC-20 kHz, 100 mV/A (20 A) or 10 mV/A (65 A). Use: CCM-ripple check of `coil.conf::L0` ([[project_coil_and_ringing]]).

## S3 USB-JTAG port
Canonical: ~/dev/kb/esp32/usb-console-and-recovery.md. Project bits: the port can re-enumerate with a new suffix seconds after plug-in, leaving `idf-export.sh`'s `ESPPORT` empty; wait and re-`ls`, or find `IOCalloutDevice` via `ioreg -l -r -c AppleUSBACMData`, then export `ESPPORT`. Wedge (2026-11-18, bench139c, after a boot-looping Unity test image): enumerated but esptool "No serial data received"; a libusb reset fixed it without replug (`.venv` has pyusb, brew libusb):
`.venv/bin/python3 -c "import usb.core,usb.backend.libusb1 as b; d=usb.core.find(idVendor=0x303a, backend=b.get_backend(find_library=lambda x:'/opt/homebrew/lib/libusb-1.0.dylib')); d.reset()"`, then wait ~4 s. (kb records cases where this reset failed and a replug was needed.)

## rpi BLE adapter
2026-08-06: rpi.local has no ethernet; its on-board BT shares radio with wlan0 and lost coexistence (connects to fbuck 20/30 ok, median 3.2 s). Separate USB adapter: 30/30 at 0.4 s, 20/20 under wlan0 load. Signature (btmon): Connection Complete Success then ~75 ms later `0x3e Connection Failed to be Established`; bluetoothd's ~5 s retry backoff made connects take 5-17 s. REFUTED: BLE ADV racing connect (`adv_ms=0` worse), bsync promiscuous WiFi (p≈0.23).
**How to apply:** adapter `10:20:BA:05:4C:8E` (Espressif HCI over USB CDC-ACM); its hciN index changes, select by MAC (`fugu_console.py --adapter`, `$BLE_ADAPTER`, `BleTransport(adapter=)`; unmatched MAC raises). `$BLE_ADAPTER` not set persistently on the rpi.
Replacement dongles: TP-Link UB500 / ASUS USB-BT500 (RTL8761B), StarTech USBA-BLUETOOTH-V5-C2, CSR8510 generics are plug-and-play `btusb`; Seeed nRF52840 MDK works after flashing Zephyr `hci_usb`; Ezurio BL654 USB (451-00003) cannot be an HCI adapter (locked to smartBASIC).
