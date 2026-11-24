---
name: Classic ESP32 target: IRAM, dead console RX, test order
description: Classic-ESP32 target traps: IRAM overlay, dead serial RX (and how to configure it anyway), buck-test UART hijack
created: 2026-11-23T09:46:06.829Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

The classic ESP32 is a secondary target; four things bite, all verified 2026-05-29/30.

**IRAM.** A classic build overflows `iram0_0_seg` by ~4.5 KB (WiFi + BLE + `ISR_IRAM_SAFE` ADC all
want IRAM, and classic has 128 KB). `sdkconfig.defaults.esp32` (committed) sets
`# CONFIG_ESP_WIFI_RX_IRAM_OPT is not set`, freeing ~16 KB; IDF auto-layers
`sdkconfig.defaults.<TARGET>`. Cost is negligible (WiFi is pinned to core 0, the RT loop to core 1).
Use a separate build dir — `IDF_TARGET=esp32 idf.py -B build-esp32 build`; reusing `build/` fails
because its CMakeCache locks the target. Occupancy lands at ~93% IRAM / 4% flash free.

**Serial console input is dead.** Logs come out of UART0 fine but no input is accepted (no echo, no
`OK:<cmd>`): `console.cpp` installs the UART RX driver (`uartInit(0)`, what `uart_read_bytes` needs)
only under `#if CONFIG_IDF_TARGET_ESP32S3`, and Arduino `Serial.begin()` alone doesn't feed it. So
`wifi-add` / `set-config` are silently dropped on classic — the S3 bench uses USB-CDC, which masks
this. Workarounds:

- **WiFi without a console:** bake creds into littlefs and reflash the data partition.
  `wifi.conf` is `ssid_<name>=<SSID>` + `ssid_<name>_psk=<psk>`. Copy a config to /tmp, add
  `conf/wifi.conf` there, then `littlefs-python create /tmp/cfg out.bin --fs-size=0x20000
  --name-max=64 --block-size=4096` and `esptool --chip esp32 write_flash 0x3b9000 out.bin`.
  Validated: the classic associated at RSSI -32 this way after `wifi-add` had silently never run.
- **e2e / console once on WiFi:** drive it over **telnet** (`run_e2e --cluster console --telnet
  <ip>:23 --mqtt-host <broker>`); that path waits for the banner and doesn't reset, while serial
  resets on open and races the ~9 s boot.
- The destructive/fuzz scripts are serial-only (hardcoded `SerialTransport`). To fuzz over telnet:
  `fe.SerialTransport = lambda *a,**k: SocketTransport(ip,port=23,is_telnet=True)` plus
  `Console(eol="\n")`, and `FUZZ_SWEEP_SPEED=0` for `fuzz_extreme` (a reboot drops telnet). That
  combination is what found [[project_crash_rootcauses]]. Coredump streaming over telnet
  stalls ~13 KB — pull the partition with `parttool.py read_partition --partition-name coredump`.
- Remaining e2e console-cluster failures are harness portability, not firmware: `console-plan`
  hardcodes an S3 DRAM peek address, `stdin-batch` shells `fugu_console.py --telnet` (valid flag is
  `--ip`), and `mqtt-cmd-input`'s `identify()` is a `NotImplementedError` stub.

**Full Unity suite reboot-loops at `test_buck`.** `test_buck.cpp::initConvEx` configures LEDC on
`pwm_hi=1`/`pwm_li=2`, and on classic **GPIO1 = U0TXD**, so the buck test hijacks the console TX
pin: every later log line and the `N Tests M Failures` summary is lost and the suite resets forever.
Harmless on S3 (GPIO1/2 aren't the console). Workaround used: run the `tracker.h` block **before**
the buck block in `test/main.cpp`; the real fix is non-UART LEDC pins for `test_buck` on classic.
Test build: `IDF_TARGET=esp32 RUN_TESTS=1 idf.py -B build-tests-esp32 -DSDKCONFIG=sdkconfig.esp32test
build`, `ESPPORT=/dev/cu.usbserial-0001` (the root `sdkconfig` is pinned to esp32s3).
