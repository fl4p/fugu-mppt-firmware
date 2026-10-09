---
title: Binary size
sidebar_position: 9
---

# Binary size

This page lists the changes already made to shrink the firmware image and the ones still open. The 1.87 MB OTA slot
(`partitions.csv`) is the hard ceiling for WITH_BLE=1 builds. Inspect the image with `idf.py size`,
`idf.py size-components` and `idf.py size-files`.

## Size progression (WITH_BLE=1, esp32s3)

The following table shows how each change moved the bin size and the free share of the OTA slot.

| Stage                                                                                       |       Bin size |        Δ |       Free |
|---------------------------------------------------------------------------------------------|---------------:|---------:|-----------:|
| Pre-2026-05-25 baseline                                                                     |  1,803,008 B |      —   |       4 %  |
| + mbedTLS prune (curves / PEM-write / X509-CRL/CSR / SHA-512 / TLS_SERVER / SSL renegotiate) |              |          |            |
| + WiFi SoftAP=n, WPA3 (SAE / SAE-PK / SAE-H2E / OWE) off                                    |              |          |            |
| + Arduino selective compilation + 15 stubbed managed components                             |  1,709,968 B | **−91 KB** |     9 %  |
| + measure_coil gated behind WITH_MEASURE_COIL (default off)                                 |  1,695,936 B |    −14 KB |     9 %  |
| + per-file `-Os` on cold libmain TUs                                                        |**1,669,872 B** |    −25 KB | **11 %** |
| + rtcount `unordered_map` → fixed array (2026-05-30)                                        |              |    −80 KB | 8 %* |
| + `std::filesystem` → POSIX dirent (2026-05-30)                                             |              |    −62 KB | 12 %* |

Total saved: 133 KB (2026-05-25) + 142 KB (2026-05-30) = 275 KB (~14 % of the OTA slot).

\* The 2026-05-30 Δ values are A/B measurements (same build, change toggled). The absolute bin size is omitted because
that working tree carried other uncommitted changes, so its free % does not continue the 1,669,872 B lineage cleanly.
Re-measure the absolutes once these changes are on a clean tree.

## Done

The following changes predate the dated rounds below:

- Drop `<fstream>` from `src/conf.h`. `std::ifstream` + `std::getline` pulled in `locale_init.o` and the entire
  classic-locale facet table for both `char` and `wchar_t` (`wlocale-inst.o`, `locale-inst.o`, `cxx11-*`). `fopen`/`fgets`
  replace them. Saved ~166 KB.
- Drop `<sstream>` / `std::stringbuf` everywhere. The image doesn't link on xtensa-esp-elf 14.2 without this
  (`undefined reference to 'basic_stringbuf_nop'`).
- `CONFIG_LWIP_IPV6=n`: `~15–20 KB`. The source has no `AF_INET6`/`in6_*`.
- `CONFIG_LIBC_NEWLIB_NANO_FORMAT=y`: `~37–46 KB`. All `src/` format strings were first audited for `%hh*`/`%ll*`,
  which newlib-nano silently misparses.
- Make sprofiler build-time optional: `~10 KB`. The option is `WITH_SPROFILER=1` (now `CONFIG_FUGU_WITH_SPROFILER`).
  When it is unset, the top-level CMakeLists sets `EXCLUDE_COMPONENTS esp32-semihosting-profiler`.

### 2026-05-25 round (sdkconfig.defaults + CMakeLists.txt + main/idf_component.yml)

This round pruned unused IDF and Arduino features and fixed the build breaks that the pruning caused:

- mbedTLS prune. The firmware uses TLS only as a client (HTTPS OTA, MQTT over TLS), so this disabled `TLS_SERVER`,
  `SSL_RENEGOTIATION`, `SERVER_SSL_SESSION_TICKETS`, `PKCS7`, `SHA512`, `ECDSA_DETERMINISTIC`, `PEM_WRITE`,
  `X509_CSR/CRL_PARSE`, `PK_PARSE_EC_{EXTENDED,COMPRESSED}`, `ECDH_{ECDSA,RSA}`, `GCM_SUPPORT_NON_AES_CIPHER`, and all
  curves except `SECP256R1`.
- `CONFIG_ESP_WIFI_SOFTAP_SUPPORT=n` + WPA3 (`SAE`, `SAE_PK`, `SAE_H2E`, `OWE_STA`) off. The firmware is STA-only
  and home APs are WPA2. arduino-esp32's `WiFiGeneric.cpp:298` still calls `esp_netif_create_default_wifi_ap()`
  unconditionally, so `main.cpp` has a weak inline stub returning `nullptr` to satisfy the link.
- `CONFIG_ARDUINO_SELECTIVE_COMPILATION=y` + per-library opt-out. RainMaker / Insights / PPP / WiFiProv /
  OpenThread / Matter / WebServer / Zigbee / ESP-SR / SimpleBLE / BluetoothSerial / SD / SD_MMC / SPIFFS / FFat /
  NetBIOS / EEPROM / Ticker / ArduinoOTA are off. Kept: WiFi / Network / SPI / Wire / Update / HTTPClient / FS /
  LittleFS / AsyncUDP / DNSServer / ESPmDNS / NetworkClientSecure / Hash / Preferences / BLE.
- 15 managed components stubbed via `override_path: ../components/_idf_stubs/<name>` in `main/idf_component.yml`:
  `esp_rainmaker`, `esp_insights`, `esp_diagnostics`, `esp_diag_data_store`, `esp_modem`, `esp_rcp_update`,
  `esp_secure_cert_mgr`, `esp-sr`, `dl_fft`, `esp-zboss-lib`, `esp-zigbee-lib`, `esp-modbus`, `libsodium`, `qrcode`,
  `chmorgan/esp-libhelix-mp3`. Kept: `cbor`, `mdns`, `network_provisioning`, `rmaker_common`, `esp-dsp`,
  `led_strip`. Stubbing via `override_path` is needed because `EXCLUDE_COMPONENTS` doesn't apply to the IDF component
  manager's managed deps (see [Considerations specific to this codebase](#considerations-specific-to-this-codebase)).
- arduino-esp32 REQUIRES patch in the top-level CMakeLists.txt. arduino-esp32's manifest is missing `esp_wifi`,
  `esp_event`, `esp_netif` and `network_provisioning`, even though its public `WiFi.h` → `WiFiType.h` transitively
  includes `esp_wifi_types.h` (and `WiFiGeneric.h` includes `network_provisioning/manager.h` when
  `CONFIG_NETWORK_PROV_NETWORK_TYPE_WIFI=y`, which auto-selects from `ESP_WIFI_ENABLED`). The strict-include checker
  tolerated this until the stubs broke the transitive chain. The build now injects the missing PUBLIC requires via
  `target_link_libraries(${arduino_lib} PUBLIC idf::esp_wifi …)`.
- ESPTelnet REQUIRES esp_wifi. The root cause is the same. Here the component's own REQUIRES list was the cleanest
  place to add it.
- SimpleFTPServer `DEFAULT_STORAGE_TYPE_ESP32=7` → PUBLIC. `FtpServerKey.h` re-resolves `STORAGE_TYPE` at every
  include site. With `SELECTIVE_FFat=n` removing `FFat.h` from the path, consumers (main.cpp via `ftp_service.h`) need
  the LITTLEFS define to fall through correctly. The define therefore moved from `component_compile_definitions`
  (PRIVATE) to `target_compile_definitions(${COMPONENT_LIB} PUBLIC …)`.
- `src/web/server.cpp` trimmed from 36 lines of dead WebServer/SPIFFS/ArduinoOTA/AsyncTCP includes around one
  `MDNS.addService(...)` call to 6 lines.

### 2026-05-25 round (gating)

`WITH_MEASURE_COIL` (default off) drops `src/measure_coil.cpp` (~11 KB), the `measure-coil` console command, and the
`isMeasuring()` guards in `cli.cpp` / `main.cpp`. Together these shrank the image by 14 KB (1,709,968 → 1,695,936 B). It is a bench-only tool ported from `etc/measure_coil.py`. To run
coil sweeps on the bench, enable it with `idf.py menuconfig` (CONFIG_FUGU_WITH_MEASURE_COIL), then `idf.py build`.

### 2026-05-25 round (per-file optimization)

Per-file `-Os` on cold libmain TUs saved ~25 KB with no measurable RT regression. The build applies
`set_source_files_properties(... COMPILE_OPTIONS "-Os")` to
`cli.cpp / sensor_setup.cpp / logging.cpp / util.cpp / viz/lcd.cpp / console.cpp / etc/rt.cpp / math/float16.cpp`,
plus all `NETW_SRC` (telemetry, ftp_service, telnet_service, scope_service, mqtt, HAMqttDevice, home_assistant,
tamp_compress, web/server, etc/ota), `BLE_SRC` (console_ble, etc/ota_ble), and `measure_coil.cpp` when enabled.

These files keep `-O2` (`CONFIG_COMPILER_OPTIMIZATION_PERF`): `main.cpp` (hosts `loopRT`), `mppt.cpp`
(`mppt.update` per ADC sample), and `adc/adc_esp32_cont.cpp` (DMA ISR helpers).

### 2026-05-30 round (STL template bloat)

This round removed STL templates that no other TU shares:

- Drop `std::filesystem` (`src/main.cpp`, `src/cli.cpp`): `~62 KB` (measured 8 % → 12 % free this build).
  Two trivial directory operations (the `main.cpp` conf-dir listing on a bad board.conf and the `cli.cpp` `ls` console
  command) used `std::filesystem::{exists,directory_iterator,file_size}`. `std::filesystem::path` is built on
  `<codecvt>` (wchar UTF-8↔UTF-16), which pulled `locale-inst.o` (the single largest libstdc++ object) and the entire
  wide-char locale + iostream + sstream cascade. The replacement is POSIX `opendir`/`readdir`/`stat`, which the
  littlefs VFS supports. After the swap, `fs_path.o`, `ios.o`, `iostream-inst.o`, `sstream-inst.o`, `codecvt` and
  `wlocale` all went to 0 refs.

  This is the same trap as the earlier `<fstream>`/`<sstream>` bans. One `std::filesystem` use pulls in the locale
  code, so grep for `std::filesystem` to keep it out. The `<iostream>` include in `tele/scope.h` now costs 0 KB
  because the cascade is gone. It is dead, though, and should be removed so that a future `std::cout` cannot pull the
  chain back in.

- Drop `std::unordered_map` from `rtcount` (`src/etc/rt.{h,cpp}`): `~80 KB` (measured 4 % → 8 % free this build).
  The per-section profiler keyed its stats by `const char*` in an `unordered_map`, which instantiated the whole
  hash-table + node-allocator template. Its `operator[]` also heap-allocated on a first-seen key from the RT core, and
  once tripped a TLSF heap assert in the middle of `mppt.update()`. The replacement is a fixed `rtcount_entry[64]` table
  matched by interned-literal pointer and appended via an atomic index. It uses no heap, looks up faster, and drops the
  template code. Most of the saving is libstdc++ hashtable code that no other TU pulls in.

- `ConfFile` `unordered_map<string,string>` + `unordered_set<string>` → flat `vector<pair>` (`src/conf.h`):
  only `~3.4 KB` (measured), much less than estimated. Other TUs already pull in `std::string`, `std::vector` and the
  string-hashing helpers, so only the `<string,string>`-specific hashtable code dropped. Removing a hashtable only pays
  when its key/value types aren't already instantiated elsewhere. The `const char*`-keyed one (rtcount) was unique and
  saved 80 KB, and the `string`-keyed one wasn't.

  The change was kept for the boot-heap benefit. It removes the per-key `malloc` the hashtable did while parsing each
  conf at boot (now a few `vector` reallocs), and a linear scan over ~10–20 keys is fast enough.
  `add()`/`addFast()`/the in-mem ctor now take `std::initializer_list`, so the `{{k,v},…}` call sites are unchanged.
  - `_Rb_tree` (467 map refs in the image) comes from arduino-esp32's BLE (`BLEDescriptorMap`/`BLECharacteristicMap`
    use `std::map`), not from firmware code. BLE needs it, and it cannot be removed without patching the vendor
    library.

### 2026-10-08 round (ADC driver gating)

`CONFIG_FUGU_WITH_INA226` / `CONFIG_FUGU_WITH_ADS` (default on) gate the INA226 and ADS1x15 backends with their init
self-tests. On boards that use the internal ADC, turning both off saves ~13 KB (1,571,168 → 1,557,424 B, ESP32-S3).

### 2026-10-08 round (TLS gating)

`CONFIG_FUGU_WITH_HTTPS` (default off) gates the TLS client. When it is off, `sdkconfig.defaults` sets
`CONFIG_ESP_HTTP_CLIENT_ENABLE_HTTPS=n` and `CONFIG_MQTT_TRANSPORT_SSL=n`, nothing attaches the mbedTLS CA bundle, and
only `http://` OTA and `mqtt://` brokers work. When it is on, it selects both options and
`CONFIG_MBEDTLS_CERTIFICATE_BUNDLE`, and OTA (`src/etc/ota.cpp`), `curl` and MQTT (`src/tele/mqtt.cpp`) verify the
server against the CA bundle. The option adds about 72 KB of flash, most of it the CA bundle.

## Candidates (not applied)

The following options remain open, grouped by how safe they are for the current config.

### High-confidence, safe for current config

- `CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL=0`: `~5–15 KB`. Assertions are currently full
  (`__FILE__/__LINE__` strings in flash). Silent assertions strip them, at the cost of opaque crashes.

### Code-level STL bloat

These candidates follow the same pattern as the 2026-05-30 wins: a heavy template that no other TU shares. Find them
with `grep -rn 'std::unordered_map\|std::map\|std::function\|std::filesystem\|std::to_string\|<iostream>\|<sstream>\|<fstream>\|<regex>' src/`,
then check the "Archive member included because of file" section of `build/fugu-firmware.map` to see what each one
pulls in.

- `std::function` (18 sites, `src/`): unverified, partly invasive. Each distinct signature instantiates a
  type-erasure thunk + vtable. The ADC `SampleCallback` (`std::function<void(uint8_t,float)>`) is on the RT path, so
  replacing it with a function pointer or template saves both size and time. The `void()`/`float()` ones
  (`enqueue_task`, virtual-sensor reads) are easy swaps to function pointers. Do this opportunistically.
- `std::to_string` (adc_esp32*.h, util.cpp, HAMqttDevice.cpp): small now that the locale cascade is gone
  (`to_string` uses `__to_chars`, not locale). Replace it with `snprintf` only if a TU shows up large in size-files.

### Bigger levers, need verification

- Global `CONFIG_COMPILER_OPTIMIZATION_SIZE=y` with per-file `-O2` overrides on the RT path: potentially
  `~100–150 KB`. The current per-file approach only affects libmain. Flipping the global default recompiles all of
  IDF + arduino-esp32 + lwip + mbedtls at `-Os` too. Before doing this, measure `rtcount` numbers before and after, and
  override `-O2` on `main.cpp / mppt.cpp / adc/adc_esp32_cont.cpp` plus any IDF code on the sample-to-PWM path (likely
  `esp_adc/esp_adc_continuous.c`, parts of `freertos`, the ADC continuous ISR).
- `-fno-asynchronous-unwind-tables`: `~80 KB`. `.eh_frame` is ~22 KB in libmain and ~61 KB in arduino-esp32. With
  `-fexceptions` ON, C++ exception throw/catch does not need async-unwind tables. They serve stack walking at arbitrary
  instructions (signal handlers, gcore-style dumps). Verify that panic backtraces still resolve before merging.
- Disable `-fexceptions` globally + rewire `service.h` / `cli.cpp` / `main.cpp` try-catches as error returns:
  `~60–80 KB` (drops `libstdc++.a`'s 66 KB `.rodata` exception unwind tables). The work is mechanical but invasive.
  Every `throw std::runtime_error(...)` in conf / sensor_setup / buck / mppt construction / ADC init would have to
  become a returned bool or `std::optional`. It is worth doing only after the other options are used up.

## Finding STL template bloat (the method behind the 2026-05-30 wins)

A single innocuous STL use can pull in a large libstdc++ object that nothing else shares, and removing that one use
drops the whole chain. Both 2026-05-30 wins (`unordered_map`, `std::filesystem`) were found with these steps:

1. Grep the suspects:
   `grep -rn 'std::unordered_map\|std::map\|std::function\|std::filesystem\|std::to_string\|<iostream>\|<sstream>\|<fstream>\|<regex>\|std::shared_ptr' src/`.
2. Ask the linker why a heavy object is in the image. `build/fugu-firmware.map` has an
   "Archive member included because of file (symbol)" section, which lists each pulled `libstdc++.a(foo.o)` with the
   object + symbol that referenced it. Walk the chain back to the first non-libstdc++ object (a firmware TU). That
   object is the trigger. Example: `fs_path.o ← main.cpp.obj (std::filesystem::path::...)` → `codecvt` → `locale-inst.o`.
3. Confirm the cascade dropped. After the change, `grep -c '<obj>.o' build/fugu-firmware.map` should read `0` for
   the whole chain (`fs_path.o`, `locale-inst.o`, `iostream-inst.o`, `sstream-inst.o`, …). 0 refs ⇒ the member is no
   longer linked.
4. A/B the bin size on the same tree (toggle the change), since absolute sizes drift with other edits.

The largest offenders on xtensa-esp-elf 14.2 fall into two groups:

- Anything that instantiates `std::locale` (`<iostream>`/`<sstream>`/`<fstream>`/`std::filesystem::path` via
  `<codecvt>`). It pulls `locale-inst.o` + the wide-char facet table (~100 KB+).
- Node-based containers (`unordered_map`/`map`/`unordered_set`). They pull hashtable/rb-tree + allocator code per
  key/value type.

Prefer `fopen`/`fgets`, POSIX `dirent`, `snprintf`, and flat `vector<pair>` over the STL equivalents.

## Considerations specific to this codebase

- `EXCLUDE_COMPONENTS` doesn't reach managed components. It only works for components from `EXTRA_COMPONENT_DIRS`
  and top-level `components/`, because the IDF component manager registers managed deps independently. To exclude
  managed deps from the build, use `override_path` to an empty stub directory.
- Stubs need matching directory names. `override_path` swaps the path, but the resolved component still has to be
  registered under the original name (`espressif__esp-modbus`) because other components reference it in
  `REQUIRES espressif__esp-modbus`. Each overridden component therefore needs its own stub directory.
- `-fexceptions` has to stay global as long as `service.h`'s try/catch is included widely. The cost is the 66 KB of
  unwind data in `libstdc++.a` and the 22 KB in libmain.
- `esp_idf_size` attributes ~185 KB of `.rodata` to `esp_app_desc.c.obj`. This is a tooling artifact: unowned
  `.rodata` is attributed to the first object alphabetically. The actual `.obj` content is ~1 KB. Ignore that row in
  size-files.

## Matrix build (pre-2026-05-25)

The following table shows the image size for each target and feature combination before the 2026-05-25 changes:

```
  ┌─────────┬──────────┬───────────┬─────────────┬───────────────────┐
  │ target  │ WITH_BLE │ WITH_NETW │    size     │   Δ vs baseline   │
  ├─────────┼──────────┼───────────┼─────────────┼───────────────────┤
  │ esp32s3 │ 1        │ 1         │ 1,804,032 B │ baseline          │
  ├─────────┼──────────┼───────────┼─────────────┼───────────────────┤
  │ esp32s3 │ 0        │ 1         │ 1,552,432 B │ −252 KB (BLE)     │
  ├─────────┼──────────┼───────────┼─────────────┼───────────────────┤
  │ esp32s3 │ 1        │ 0         │ 1,515,776 B │ −288 KB (NETW)    │
  ├─────────┼──────────┼───────────┼─────────────┼───────────────────┤
  │ esp32s3 │ 0        │ 0         │ 1,255,136 B │ −549 KB (both)    │
  ├─────────┼──────────┼───────────┼─────────────┼───────────────────┤
  │ esp32   │ 1        │ 1         │ 1,790,096 B │ 4% partition free │
  ├─────────┼──────────┼───────────┼─────────────┼───────────────────┤
  │ esp32   │ 0        │ 1         │ 1,534,720 B │ —                 │
  └─────────┴──────────┴───────────┴─────────────┴───────────────────┘
```

Re-run `etc/matrix_build.sh` after the 2026-05-25 changes to refresh this table.
