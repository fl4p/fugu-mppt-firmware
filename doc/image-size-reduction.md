*this document is an LLM generated placeholder*

# Reducing the app image size (esp32s3, BLE + WiFi kept)

Measured 2026-09-29 on scratch copies of the repo (`../_fugu_commit` = IDF 6.0.3 / arduino-esp32 3.3.12,
`../_fugu_commit55` = IDF 5.5 / arduino-esp32 3.3.7). The numbers are sizes of `fugu-firmware.bin`, the file
that goes into an OTA slot. The ELF is not measured. Every build started from a fresh `sdkconfig` and exited with rc=0.
Builds are byte-reproducible: rebuilding the same config gave the same size every time.

OTA slot: `0x1c9000` = 1,871,872 B.

| | IDF 6.0.3 | IDF 5.5 |
|---|---|---|
| baseline (HEAD of the scratch copies) | 1,842,320 (`0x1c1c90`), 29,552 B free | 1,783,600 (`0x1b3730`), 88,272 B free |
| **safe set (below)** | **1,608,784 (`0x188c50`), 263,088 B free** | **1,565,200 (`0x17e210`), 306,672 B free** |
| saving | **-233,536 B (-12.7 %)** | **-218,400 B (-12.2 %)** |

On IDF 6 the safe set also frees 12.6 KB of internal RAM: IRAM text -8.9 KB and .data -3.7 KB.

The safe set also clears the DROM page cliff described in `doc/2026-09-08-image-layout-stability-for-ota.md`.
DROM now ends at file offset `0x47f4c` (IDF 6) and `0x4b9cc` (IDF 5.5), and IROM starts at `0x50018`.
That leaves about 32 KB (IDF 6) and 18 KB (IDF 5.5) of rodata growth before the next 64 KB page. Before, the margin was about 2 KB.

## Ranked candidates

"Standalone" means one change on top of the baseline. "LOO" (leave-one-out) means the saving lost when only this
item is removed from the full safe set. It is smaller when items overlap. RT means an effect on the core-1 control loop.

### Safe set (applied in both scratch copies)

| # | change | IDF 6 saving | IDF 5.5 saving | risk / functional impact | RT impact |
|---|---|---|---|---|---|
| 1 | `CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT=y` | 84,592 standalone, 81,952 LOO | 81,840 LOO | Low. `assert()` still aborts and gives a backtrace + coredump, but the `file:line expr` text is gone. Decode the location with the ELF archive. | none (asserts get slightly cheaper) |
| 2 | `-Os` on core-0-only components, `-O2` kept elsewhere (CMake block below) | 56,048 standalone | — | Low. Slower code in the BT host, lwIP, mDNS, littlefs, WiFi glue, MQTT, FTP, telnet, OTA and coredump paths. | none by construction. RT-path components (freertos, esp_timer, hal/esp_hal_*, esp_hw_support, drivers mcpwm/i2c/gpio/adc/ledc/pcnt/gptimer, spi_flash, heap, libc, log, INA226/ADS libs) stay `-O2`. |
| 2b | the same for arduino-esp32, with `-O2` kept on `esp32-hal-i2c*.c`, `esp32-hal-gpio.c`, `esp32-hal-misc.c`, `esp32-hal-periman.c`, `Wire.cpp` | +20,064 (2+2b: 76,112 standalone, 71,216 LOO) | 2+2b: 59,744 LOO | Low | none: the files the RT loop calls (Wire for INA226/ADS1x15, digitalWrite, micros/millis) stay `-O2` |
| 3 | `__attribute__((cold))` on setup-only functions (list below) | 11,232 | in set, not isolated | none: GCC optimises cold functions for size | none. All of them run in `setup()` or `loopLF` (core 0). |
| 4 | `ESP32_ARDUINO_NO_RGB_BUILTIN` on the arduino target | 9,744 | in set, not isolated | none. `digitalWrite(LED_BUILTIN=97)` would no longer drive an RGB LED; nothing uses it. This drops `esp32-hal-rmt`, `esp32-hal-rgb-led` and the IDF RMT RX driver. | slightly positive: one less compare in `digitalWrite` |
| 5 | `CONFIG_BT_CTRL_BLE_SCAN=n` | 10,384 | in set | Low. The controller can no longer scan. The host is peripheral + broadcaster only and never scans. **Bench-verify** advertising, connect, bonding reconnect, OTA-BLE, the ESPHome proxy and `tele_adv`. | none |
| 6 | `CONFIG_ESP_SYSTEM_MEMPROT=n` (IDF 6) / `CONFIG_ESP_SYSTEM_MEMPROT_FEATURE=n` (IDF 5.5) | 9,984 | in set | Low. Removes the PMS IRAM/DRAM W^X hardening. No functional change. | none |
| 7 | `CONFIG_MQTT_TRANSPORT_WEBSOCKET=n` (+ `_SECURE=n`) | 9,200 | in set | none: only `mqtt://` is used | none |
| 8 | mDNS: `CONFIG_MDNS_ENABLE_BROWSE=n`, `..._CONSOLE_CLI=n`, `..._PREDEF_NETIF_AP=n`, `..._PREDEF_NETIF_ETH=n` | 8,752 | in set | Low. Only `begin/addService/queryHost` on STA are used. | none |
| 9 | `CONFIG_BT_NIMBLE_PRINT_ERR_NAME=n` | 6,560 | in set | NimBLE logs numeric rc instead of names | none |
| 10 | `CONFIG_MBEDTLS_TLS_CLIENT_ONLY=y` | 6,272 | in set | none. The existing `CONFIG_MBEDTLS_TLS_SERVER_AND_CLIENT=n` line never took effect: setting a choice member to `n` does not select another one, so the server code was linked. | none |
| 11 | `CONFIG_BT_NIMBLE_LOG_LEVEL_WARNING=y` | 4,288 | in set | Low: NimBLE info logs are gone | none |
| 12 | `CONFIG_BT_CTRL_DTM_ENABLE=n` | 1,072 | in set | none (RF direct test mode) | none |

### Extended options (measured on top of the safe set, IDF 6 unless noted)

| change | saving | risk / impact | RT impact |
|---|---|---|---|
| `CONFIG_COMPILER_OPTIMIZATION_CHECKS_SILENT=y` | 46,656 (IDF 5.5: 47,856) | **Medium, diagnostics.** IDF `ESP_RETURN_ON_*`/`ESP_GOTO_ON_*` stop logging their reason, e.g. `E mcpwm: invalid arg` lines. `ESP_ERROR_CHECK` aborts without the message. Return codes are unchanged. | none |
| `-Os` on the RT translation units (`main.cpp`, `mppt.cpp`, `adc_esp32_cont.cpp`) | 27,344 | **RT RISK.** `loopRT`, `mppt.update`, the PD controllers, the sampler and the DMA helpers are compiled for size. Needs `rt-stats`/`lag` before-and-after on a bench board. | **yes** |
| global `CONFIG_COMPILER_OPTIMIZATION_SIZE=y` (everything, including the row above) | 58,832 | **RT RISK.** FreeRTOS, esp_timer, HAL, MCPWM/I2C/ADC drivers and IRAM ISR code become `-Os`. `likely()/unlikely()` turn into no-ops (`esp_compiler.h`). This is the ceiling; standalone from the baseline it is 153,680. | **yes** |
| `CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT=n` | 14,336 (16,336 standalone) | Low to medium. The 2M PHY request in `console_ble.cpp` compiles out (it is `#if`-gated). 2M was measured NOT to speed up OTA-BLE (see the comment in `bleTxDrain`) and only helps console latency. | none |
| drop `-fno-merge-constants` (`main/CMakeLists.txt`) | 9,856 (13,056 without the safe set) | Trade-off. It brings back the OTA sector churn that the layout doc fixed: more sectors rewritten per delta OTA. | none |
| `CONFIG_ESP_ERR_TO_NAME_LOOKUP=n` | 7,872 | Low to medium, diagnostics. `esp_err_to_name()` returns `UNKNOWN ERROR`, so logs show hex codes only. | none |
| `CONFIG_ESP_COREDUMP_LOGS=n` | ~6,600 (derived: a 4-knob group gave 7,152, of which loopback was 464 and the other two 80) | Low. The coredump is still written and read by `coredump`; only the log lines during the dump are lost. The help text says these strings sit in DRAM, so this also frees about 5 KB of RAM. | none |
| tier-2 `-Os`: `esp_system esp_driver_uart esp_driver_usb_serial_jtag esp_ringbuf esp_stdio pthread cxx esp_mm esp_pm nvs_sec_provider` | 3,296 | Low. These include the panic/IPC code. Kept out of the safe set for that reason. | marginal (IPC during flash ops) |
| flash vendor drivers off (`SPI_FLASH_SUPPORT_{ISSI,MXIC,BOYA,TH,MXIC_OPI}_CHIP=n`) | 3,248 standalone | Medium relative to the saving: a module with one of those chips would fall back to the generic driver, and QE-bit/HPM handling differs. Keep GD/XMC/Winbond. | none |
| `MQTT_TRANSPORT_SSL=n` + `ESP_HTTP_CLIENT_ENABLE_HTTPS=n` | 2,816 | Drops HTTPS OTA and `mqtts://`. Neither works today: `doOta()` passes no CA/bundle and MQTT sets no verification, so esp-tls would refuse the handshake. The saving is small because MQTT's plain TCP transport lives in `transport_ssl.c` and keeps esp-tls + mbedtls SSL linked. IDF 6.0.3 has no `MBEDTLS_TLS_DISABLED`. | none |
| `CONFIG_BT_NIMBLE_ENABLE_CONN_REATTEMPT=n` | ~1,250 (derived) | Low | none |
| `ESP_WIFI_GMAC_SUPPORT=n` + `ESP_WIFI_AMPDU_TX_ENABLED=n` | 1,168 | AMPDU off lowers WiFi throughput. Not worth it. | none |
| `LWIP_NETIF_LOOPBACK=n` | 464 | Low | none |
| extra mbedtls trims (`KEY_EXCHANGE_RSA`, `CCM`, `DHM`) beyond client-only | ~290 | — | none |

**Rejected: `CONFIG_BT_CTRL_BLE_MASTER=n`** (with scan off, 21,552 standalone). Kconfig help: *"Enable BLE connection
feature. If disabled, it is not recommended to use connectable ADV."* It gates all connections, not only the central role, so it would break the NUS console and OTA-BLE.

## Exact changes (safe set)

`sdkconfig.defaults` (append):

```
CONFIG_MBEDTLS_TLS_CLIENT_ONLY=y
CONFIG_MQTT_TRANSPORT_WEBSOCKET=n
CONFIG_MQTT_TRANSPORT_WEBSOCKET_SECURE=n
CONFIG_MDNS_ENABLE_CONSOLE_CLI=n
CONFIG_MDNS_PREDEF_NETIF_AP=n
CONFIG_MDNS_PREDEF_NETIF_ETH=n
CONFIG_MDNS_ENABLE_BROWSE=n
CONFIG_ESP_SYSTEM_MEMPROT=n
CONFIG_ESP_SYSTEM_MEMPROT_FEATURE=n
CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT=y
```

`sdkconfig.ble` (append):

```
CONFIG_BT_CTRL_BLE_SCAN=n
CONFIG_BT_CTRL_DTM_ENABLE=n
CONFIG_BT_NIMBLE_PRINT_ERR_NAME=n
CONFIG_BT_NIMBLE_LOG_LEVEL_WARNING=y
```

Top-level `CMakeLists.txt`, after `project(fugu-firmware)`. The component names cover both IDF versions; names that are not in the build are skipped:

```cmake
set(FUGU_OS_COMPONENTS bt lwip espressif__mdns joltwallet__littlefs wpa_supplicant espressif__mqtt http_parser
        esp_http_client esp-tls tcp_transport esp_https_ota SimpleFTPServer ESPTelnet SimpleCLI nvs_flash esp_netif
        esp_wifi esp_event espcoredump esp-ota-ble tamp espressif__esp_delta_ota app_update esp_partition vfs esp_phy
        esp_coex espressif__network_provisioning protocomm protobuf-c espressif__led_strip esp_driver_rmt esp_hal_rmt
        console bootloader_support efuse esp_app_format Arduino-LiquidCrystal-I2C esp_security
        mqtt wifi_provisioning)
idf_build_get_property(_fugu_comps BUILD_COMPONENTS)
foreach (_c ${FUGU_OS_COMPONENTS})
    if (_c IN_LIST _fugu_comps)
        idf_component_get_property(_l ${_c} COMPONENT_LIB)
        get_target_property(_t ${_l} TYPE)
        if (NOT _t STREQUAL "INTERFACE_LIBRARY")
            target_compile_options(${_l} PRIVATE -Os)
        endif ()
    endif ()
endforeach ()
idf_component_get_property(_al espressif__arduino-esp32 COMPONENT_LIB)
idf_component_get_property(_ad espressif__arduino-esp32 COMPONENT_DIR)
target_compile_options(${_al} PRIVATE -Os)
set_source_files_properties(
        ${_ad}/cores/esp32/esp32-hal-i2c-ng.c ${_ad}/cores/esp32/esp32-hal-i2c.c ${_ad}/cores/esp32/esp32-hal-gpio.c
        ${_ad}/cores/esp32/esp32-hal-misc.c ${_ad}/cores/esp32/esp32-hal-periman.c ${_ad}/libraries/Wire/src/Wire.cpp
        TARGET_DIRECTORY ${_al} PROPERTIES COMPILE_OPTIONS "-O2")
target_compile_definitions(${_al} PRIVATE ESP32_ARDUINO_NO_RGB_BUILTIN)
```

`compile_commands.json` confirmed the flag order: `tcp.c`/`ble_gap.c`/`mqtt_client.c` get `-O2 … -Os`, `Wire.cpp` gets
`-O2 … -Os … -O2`, and `tasks.c`/`main.cpp` stay `-O2`. The last `-O` flag wins.

`__attribute__((cold))` added to: `setup()`, `lfStatusLine()`, `loopLF()` (`main.cpp`), `BatChargerParams::load`,
`BatteryCharger::begin`, `BatteryCharger::beginMqtt` (`charger.h`), `MpptController::begin` (decl, `mppt.h`),
`SynchronousConverter::drvInit`, `SynchronousConverter::init` (`buck.h`), `pdLoadGains` (`pd_control.h`),
`Plot::_plotSeries` (`etc/plot.h`). Every call site is in `setup()` or on core 0. More setup-only code could be marked the same way;
the remaining large ones are `setupSensors`, `ConfFile` parsing, `Limits(const ConfFile&)`, `haMqttSendDiscovery` and
`setupNetworkAtBoot`.

## Checked, did not help (or cannot be done without patching a vendor lib)

- `CONFIG_HEAP_POISONING_LIGHT` instead of COMPREHENSIVE: +592 B, no flash benefit. The cost of COMPREHENSIVE is CPU time and RAM, not flash.
- `CONFIG_BT_NIMBLE_LL_CFG_FEAT_LE_CODED_PHY=n`: 0 B.
- `CONFIG_BT_CTRL_BLE_SCAN_DUPL=n`: 0 B on top of scan off. `BLE_ADV_REPORT_FLOW_CTRL_SUPP` is not settable.
- `CONFIG_BT_NIMBLE_GATT_CLIENT=n`: symbol absent in this config, so nothing to gain.
- `CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=n`: forced back to `y` by a `select`.
- `CONFIG_LWIP_DHCPS=n`: link error. arduino `NetworkInterface::config` references `esp_netif_dhcps_*` (dhcpserver.c is 4.5 KB).
- `SPI_FLASH_ENABLE_ENCRYPTED_READ_WRITE=n` + `ESP_PHY_ENABLE_VERSION_PRINT=n`: 80 B.
- network_provisioning / protocomm / protobuf: linked through arduino `WiFiGeneric`, but only ~3 KB survive `--gc-sections`.
- smartconfig / FTM: ~2.5 KB, pulled in by arduino `WiFiSTA`. Removing them needs a patch to arduino.
- mbedtls certificate bundle: not linked (no `x509_crt_bundle`), nothing to gain.
- `pow(2, x)` → `ldexp` in `float16.cpp`: no gain, because esp-dsp's `dsps_biquad_gen_f32` also needs `pow`.
- picolibc tinystdio (IDF 6): already small (vfprintf 3.5 KB, vfscanf 2.9 KB). There is no smaller printf knob.
- `libesp_stdio.a` shows as 164 KB in `esp_idf_size`. That is the merged string pool attributed to its first contributor, not real code.
- Lower `LOG_DEFAULT/MAXIMUM_LEVEL`: not tried. `ESP_LOGI` is the firmware's user-facing console output, so this would be a functional change.
- LTO: not attempted. IDF links with `-fno-lto`, and `linker.lf`/ldgen place code by archive/object, which LTO's ltrans objects would bypass.

## Estimated, not measured

- Replace the arduino BLE wrapper (`BLEDevice/BLEServer/BLECharacteristic/...`, 29.1 KB after `-Os`, including 7.4 KB of
  `BLEScan`/`BLEAdvertisedDevice` that `BLEDevice` drags in) with the NimBLE GATT server API in `console_ble.cpp`,
  `tele_ble.cpp` and `tele_adv.cpp`. Estimated net gain is ~20 KB. This is a medium-sized rewrite, and the bond/MTU/notify-backpressure behaviour would have to be re-validated.
- arduino `chip-debug-report.cpp` (~2 KB) is kept alive by a weak `shouldPrintChipDebugReport()` in arduino's `main.cpp`.
  Removing it needs a vendor patch.

## Validation still owed before landing the safe set

No hardware was touched. Before OTAing any of this to fry/flat:

- **BT scan off:** check on a bench board that BLE advertising, pairing and bond reconnect still work, and that OTA-BLE, the ESPHome proxy and BLE ADV telemetry work.
- **Everything else:** these are compile-flag changes on core-0 code. Compare `rt-stats` and InfluxDB `lag` before and after as a sanity check. The RT loop itself should not change.
- **Silent asserts:** after a crash, check that a coredump still decodes to the right location through `etc/idf-devtools/elf_archive.py decode`.
