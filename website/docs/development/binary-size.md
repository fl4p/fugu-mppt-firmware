---
title: Binary size
sidebar_position: 9
---

# Binary size

Notes on what's been done and what's still on the table to shrink the firmware image. Run `idf.py size`,
`idf.py size-components`, `idf.py size-files` to inspect. The 1.87 MB OTA slot (`partitions.csv`) is the hard ceiling
for WITH_BLE=1 builds.

## Size progression (WITH_BLE=1, esp32s3)

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

**Total saved: 133 KB (2026-05-25) + 142 KB (2026-05-30) = 275 KB (~14 % of the OTA slot).**

\* The 2026-05-30 Δ are measured A/B (same build, change toggled); absolute bin size is omitted because that
working tree carried other uncommitted changes, so its absolute free % isn't a clean continuation of the
1,669,872 B lineage. Re-measure absolutes after these land on a clean tree.

## Done

- **Drop `<fstream>` from `src/conf.h`** — `std::ifstream` + `std::getline` pulled in `locale_init.o` and the entire
  classic-locale facet table for both `char` and `wchar_t` (`wlocale-inst.o`, `locale-inst.o`, `cxx11-*`). Replaced with
  `fopen`/`fgets`. Saved ~166 KB.
- **Drop `<sstream>` / `std::stringbuf` everywhere** — required to link at all on xtensa-esp-elf 14.2 (
  `undefined reference to 'basic_stringbuf_nop'`).
- **`CONFIG_LWIP_IPV6=n`** — `~15–20 KB`. No `AF_INET6`/`in6_*` in source.
- **`CONFIG_LIBC_NEWLIB_NANO_FORMAT=y`** — `~37–46 KB`. Audited all `src/` format strings for `%hh*`/`%ll*` first
  (newlib-nano silently misparses them).
- **Make sprofiler build-time optional** — `~10 KB`. `WITH_SPROFILER=1` (now `CONFIG_FUGU_WITH_SPROFILER`); top-level CMakeLists `EXCLUDE_COMPONENTS
  esp32-semihosting-profiler` when unset.

### 2026-05-25 round (sdkconfig.defaults + CMakeLists.txt + main/idf_component.yml)

- **mbedTLS prune** — disabled `TLS_SERVER`, `SSL_RENEGOTIATION`, `SERVER_SSL_SESSION_TICKETS`, `PKCS7`, `SHA512`,
  `ECDSA_DETERMINISTIC`, `PEM_WRITE`, `X509_CSR/CRL_PARSE`, `PK_PARSE_EC_{EXTENDED,COMPRESSED}`, `ECDH_{ECDSA,RSA}`,
  `GCM_SUPPORT_NON_AES_CIPHER`, all curves except `SECP256R1`. We're TLS-client only (HTTPS-OTA, MQTT-TLS).
- **`CONFIG_ESP_WIFI_SOFTAP_SUPPORT=n`** + WPA3 (`SAE`, `SAE_PK`, `SAE_H2E`, `OWE_STA`) off. We're STA-only, home APs
  are WPA2. arduino-esp32's `WiFiGeneric.cpp:298` calls `esp_netif_create_default_wifi_ap()` unconditionally though, so
  there's a weak inline stub returning `nullptr` in `main.cpp` to satisfy the link.
- **`CONFIG_ARDUINO_SELECTIVE_COMPILATION=y`** + per-library opt-out: RainMaker / Insights / PPP / WiFiProv /
  OpenThread / Matter / WebServer / Zigbee / ESP-SR / SimpleBLE / BluetoothSerial / SD / SD_MMC / SPIFFS / FFat /
  NetBIOS / EEPROM / Ticker / ArduinoOTA off. Kept: WiFi / Network / SPI / Wire / Update / HTTPClient / FS / LittleFS /
  AsyncUDP / DNSServer / ESPmDNS / NetworkClientSecure / Hash / Preferences / BLE.
- **15 managed components stubbed** via `override_path: ../components/_idf_stubs/<name>` in `main/idf_component.yml`:
  `esp_rainmaker`, `esp_insights`, `esp_diagnostics`, `esp_diag_data_store`, `esp_modem`, `esp_rcp_update`,
  `esp_secure_cert_mgr`, `esp-sr`, `dl_fft`, `esp-zboss-lib`, `esp-zigbee-lib`, `esp-modbus`, `libsodium`, `qrcode`,
  `chmorgan/esp-libhelix-mp3`. `EXCLUDE_COMPONENTS` doesn't apply to the IDF component manager's managed deps; stubbing
  via `override_path` is the working lever. Kept: `cbor`, `mdns`, `network_provisioning`, `rmaker_common`, `esp-dsp`,
  `led_strip`.
- **arduino-esp32 REQUIRES patch** in top-level CMakeLists.txt — arduino-esp32's manifest is missing `esp_wifi`,
  `esp_event`, `esp_netif`, `network_provisioning` even though its public `WiFi.h` → `WiFiType.h` transitively includes
  `esp_wifi_types.h` (and `WiFiGeneric.h` includes `network_provisioning/manager.h` when
  `CONFIG_NETWORK_PROV_NETWORK_TYPE_WIFI=y`, which auto-selects from `ESP_WIFI_ENABLED`). Before stubbing the optional
  managed deps the strict-include checker tolerated this — once the stubs broke the transitive chain it didn't, so we
  inject the missing PUBLIC requires via `target_link_libraries(${arduino_lib} PUBLIC idf::esp_wifi …)`.
- **ESPTelnet REQUIRES esp_wifi** — same root cause, but its own REQUIRES list was the cleanest place to add it.
- **SimpleFTPServer `DEFAULT_STORAGE_TYPE_ESP32=7` → PUBLIC** — `FtpServerKey.h` re-resolves `STORAGE_TYPE` at every
  include site. With `SELECTIVE_FFat=n` removing `FFat.h` from the path, consumers (main.cpp via `ftp_service.h`) need
  the LITTLEFS define to fall through correctly; moved from `component_compile_definitions` (PRIVATE) to
  `target_compile_definitions(${COMPONENT_LIB} PUBLIC …)`.
- **`src/web/server.cpp` trimmed** — was 36 lines of dead WebServer/SPIFFS/ArduinoOTA/AsyncTCP includes around one
  `MDNS.addService(...)` call. Now 6 lines.

### 2026-05-25 round (gating)

- **`WITH_MEASURE_COIL` (default off)** — drops `src/measure_coil.cpp` (~11 KB), the `measure-coil` console command,
  and the `isMeasuring()` guards in `cli.cpp` / `main.cpp`. Bench-only tool ported from `etc/measure_coil.py`; enable
  with `idf.py menuconfig` (CONFIG_FUGU_WITH_MEASURE_COIL) then `idf.py build` when running coil sweeps on the bench.

### 2026-05-25 round (per-file optimization)

- **Per-file `-Os` on cold libmain TUs** — `set_source_files_properties(... COMPILE_OPTIONS "-Os")` on
  `cli.cpp / sensor_setup.cpp / logging.cpp / util.cpp / viz/lcd.cpp / console.cpp / etc/rt.cpp / math/float16.cpp`
  plus all `NETW_SRC` (telemetry, ftp_service, telnet_service, scope_service, mqtt, HAMqttDevice, home_assistant,
  tamp_compress, web/server, etc/ota) and `BLE_SRC` (console_ble, etc/ota_ble), and `measure_coil.cpp` when enabled.
  Kept `-O2` (`CONFIG_COMPILER_OPTIMIZATION_PERF`) on: `main.cpp` (hosts `loopRT`), `mppt.cpp` (`mppt.update` per ADC
  sample), `adc/adc_esp32_cont.cpp` (DMA ISR helpers). Saved ~25 KB; no measurable RT regression.

### 2026-05-30 round (STL template bloat)

- **Drop `std::filesystem` (`src/main.cpp`, `src/cli.cpp`)** — `~62 KB` (measured 8 % → 12 % free this build).
  Two trivial directory ops (`main.cpp` conf-dir listing on a bad board.conf; `cli.cpp` `ls` console command) used
  `std::filesystem::{exists,directory_iterator,file_size}`. `std::filesystem::path` is built on `<codecvt>`
  (wchar UTF-8↔UTF-16), which pulled `locale-inst.o` (the *single largest* libstdc++ object) and the entire wide-char
  locale + iostream + sstream cascade — `fs_path.o`, `ios.o`, `iostream-inst.o`, `sstream-inst.o`, `codecvt`, `wlocale`
  all went to **0 refs** after the swap. Replaced with POSIX `opendir`/`readdir`/`stat` (littlefs VFS supports them).
  This is the same "one std::filesystem use detonates locale" trap behind the earlier `<fstream>`/`<sstream>` bans —
  grep `std::filesystem` before it creeps back. (Note: the `<iostream>` include in `tele/scope.h` is now 0 KB since the
  cascade is gone, but it's dead — remove it so a future `std::cout` can't re-detonate the chain.)

- **Drop `std::unordered_map` from `rtcount` (`src/etc/rt.{h,cpp}`)** — `~80 KB` (measured 4 % → 8 % free this build).
  The per-section profiler keyed its stats by `const char*` in an `unordered_map`, which instantiated the whole
  hash-table + node-allocator template (and, worse, `operator[]` heap-allocated on a first-seen key *from the RT core*,
  once tripping a TLSF heap assert mid-`mppt.update()`). Replaced with a fixed `rtcount_entry[64]` table matched by
  interned-literal pointer and appended via an atomic index — no heap, faster lookup, and the template bloat is gone.
  The win is mostly libstdc++ hashtable code that no other TU pulls in.

- **`ConfFile` `unordered_map<string,string>` + `unordered_set<string>` → flat `vector<pair>` (`src/conf.h`)** —
  only `~3.4 KB` (measured). Much smaller than estimated: `std::string`/`std::vector`/the string-hashing helpers are
  already pulled by other TUs, so only the `<string,string>`-specific hashtable code dropped. Kept anyway — it also
  removes the per-key `malloc` the hashtable did while parsing each conf at boot (now a few `vector` reallocs), and
  linear scan over ~10–20 keys is fine. `add()`/`addFast()`/the in-mem ctor now take `std::initializer_list` so the
  `{{k,v},…}` call sites are unchanged. **Lesson: removing a hashtable only pays when its key/value types aren't
  already instantiated elsewhere — `const char*`-keyed (rtcount) was unique and paid 80 KB; `string`-keyed wasn't.**
  - `_Rb_tree` (467 map refs in the image) traces to **arduino-esp32's BLE** (`BLEDescriptorMap`/`BLECharacteristicMap`
    use `std::map`), not our code — load-bearing for BLE, not removable without patching the vendor lib. Don't re-chase.

## Candidates (not applied)

### High-confidence, safe for current config

1. **Disable mbedTLS CA cert bundle** — `~70 KB`. **Biggest remaining low-effort lever — recommended next.**
   The bundle is a ~70 KB `.rodata` blob of ~200 root CA certs (`x509_crt_bundle`), embedded so a TLS client can verify
   *any* public server without shipping a cert. We never make an outbound TLS connection today:
   - `src/etc/ota.cpp:176` sets `cert_pem = 0` (HTTP OTA, not HTTPS),
   - every `mqtt.conf` broker URI is `mqtt://` (plaintext), and
   - `grep -rn 'https://' src/` is empty.

   **How:** in `sdkconfig.defaults` add `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=n` (drops the blob; the `esp_crt_bundle_*`
   API goes away). This does **not** remove mbedTLS itself (the TLS/AES/SHA code stays — it's a small slice vs the cert
   blob), so MQTT-TLS / HTTPS-OTA can be *re-enabled later* — you'd just have to provide trust explicitly:
   set the flag back to `=y` (full bundle), or `…BUNDLE_DEFAULT_CMN` (fewer certs, smaller), or pin a single CA via
   `esp_tls_cfg.cacert_pem_buf` / OTA's `cert_pem`. So it's reversible, not a one-way door — just a policy choice that
   "no outbound TLS until someone wires a cert."

   **Trade-off:** the day someone points OTA/MQTT at an `https://`/`mqtts://` endpoint expecting it to "just work"
   against public CAs, it won't — they'll hit a verify failure until they re-enable the bundle or pin a cert. For
   installations that stay LAN-local with a private MQTT broker, that day may never come. **A/B it** (toggle the flag, rebuild)
   to confirm the ~70 KB on the current tree before committing, since the bundle size depends on `…BUNDLE_DEFAULT_*`.

2. **`CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL=0`** — `~5–15 KB`.
   Currently full assertions (`__FILE__/__LINE__` strings in flash). Silent assertions strip them. Trade-off: crashes
   become opaque.

### Code-level STL bloat (same pattern as the 2026-05-30 wins — heavy template no other TU shares)

Find them with: `grep -rn 'std::unordered_map\|std::map\|std::function\|std::filesystem\|std::to_string\|<iostream>\|<sstream>\|<fstream>\|<regex>' src/`,
then check `build/fugu-firmware.map` ("Archive member included because of file" section) for what each pulls.

6. ~~`ConfFile` `unordered_map` → flat vector~~ — **DONE 2026-05-30, only ~3.4 KB** (see Done above). The string-keyed
   hashtable bloat was already mostly shared with other TUs, so the win was small. Kept for the boot-heap benefit.

7. **`std::function` (18 sites, `src/`)** — unverified, partly invasive. Each distinct signature instantiates a
   type-erasure thunk + vtable. The ADC `SampleCallback` (`std::function<void(uint8_t,float)>`) is also on the RT path,
   so replacing it with a function pointer or template is a size *and* speed win; the `void()`/`float()` ones
   (`enqueue_task`, virtual-sensor reads) are easy swaps to function pointers. Do opportunistically.

8. **`std::to_string` (adc_esp32*.h, util.cpp, HAMqttDevice.cpp)** — small now that the locale cascade is gone
   (`to_string` uses `__to_chars`, not locale). Replace with `snprintf` only if a TU shows up hot in size-files.

### Bigger levers, need verification

3. **Global `CONFIG_COMPILER_OPTIMIZATION_SIZE=y`** with per-file `-O2` overrides on the RT path —
   potentially `~100–150 KB`. The current per-file approach only affects libmain; flipping the global default
   recompiles all of IDF + arduino-esp32 + lwip + mbedtls at `-Os` too. Pre-conditions: measure `rtcount` numbers
   before/after, and override `-O2` on `main.cpp / mppt.cpp / adc/adc_esp32_cont.cpp` plus any IDF code on the
   sample-to-PWM path (likely `esp_adc/esp_adc_continuous.c`, parts of `freertos`, the ADC continuous ISR).

4. **`-fno-asynchronous-unwind-tables`** — `~80 KB`.
   `.eh_frame` is ~22 KB in libmain and ~61 KB in arduino-esp32. With `-fexceptions` ON, async-unwind tables aren't
   required for C++ exception throw/catch — they're for stack walking at arbitrary instructions (signal handlers,
   gcore-style dumps). Verify panic backtraces still resolve before merging.

5. **Disable `-fexceptions` globally + rewire `service.h` / `cli.cpp` / `main.cpp` try-catches as error returns** —
   `~60–80 KB` (drops `libstdc++.a`'s 66 KB `.rodata` exception unwind tables). Mechanical but invasive: every
   `throw std::runtime_error(...)` in conf / sensor_setup / buck / mppt construction / ADC init would have to become
   a returned bool or `std::optional`. Not worth it until other levers are exhausted.

## Finding STL template bloat (the method behind the 2026-05-30 wins)

A single innocuous STL use can pull a large libstdc++ object that nothing else shares — and removing that one use
drops the whole chain. The two 2026-05-30 wins (`unordered_map`, `std::filesystem`) were both found this way:

1. **Grep the suspects** —
   `grep -rn 'std::unordered_map\|std::map\|std::function\|std::filesystem\|std::to_string\|<iostream>\|<sstream>\|<fstream>\|<regex>\|std::shared_ptr' src/`.
2. **Ask the linker *why* a heavy object is in the image.** `build/fugu-firmware.map` has an
   *"Archive member included because of file (symbol)"* section: each pulled `libstdc++.a(foo.o)` is listed with the
   object + symbol that referenced it. Walk the chain back to the first **non-libstdc++** object (one of *our* TUs) —
   that's the trigger. Example: `fs_path.o ← main.cpp.obj (std::filesystem::path::...)` → `codecvt` → `locale-inst.o`.
3. **Confirm the cascade dropped.** After the change, `grep -c '<obj>.o' build/fugu-firmware.map` should read `0` for
   the whole chain (`fs_path.o`, `locale-inst.o`, `iostream-inst.o`, `sstream-inst.o`, …). 0 refs ⇒ the member is no
   longer linked.
4. **A/B the bin size** on the *same* tree (toggle the change), since absolute sizes drift with other edits.

The biggest detonators on xtensa-esp-elf 14.2: anything that instantiates `std::locale` (`<iostream>`/`<sstream>`/
`<fstream>`/`std::filesystem::path` via `<codecvt>`) pulls `locale-inst.o` + the wide-char facet table (~100 KB+);
node-based containers (`unordered_map`/`map`/`unordered_set`) pull hashtable/rb-tree + allocator code per key/value
type. Prefer `fopen`/`fgets`, POSIX `dirent`, `snprintf`, and flat `vector<pair>` over the STL equivalents.

## Considerations specific to this codebase

- **EXCLUDE_COMPONENTS doesn't reach managed components.** It only works for components from `EXTRA_COMPONENT_DIRS` and
  top-level `components/`. The IDF component manager registers managed deps independently. Use `override_path` to an
  empty stub directory to exclude managed deps from the build.
- **Stubs need matching directory names.** `override_path` swaps the *path*; the resolved component still has to be
  registered under the original name (`espressif__esp-modbus`), because other components' `REQUIRES espressif__esp-modbus`
  reference the name. So one stub directory per overridden component.
- **`-fexceptions` has to stay global** as long as `service.h`'s try/catch is included widely. The 66 KB of unwind data
  in `libstdc++.a` and the 22 KB in libmain are the cost.
- **`esp_idf_size`'s `.rodata` attribution to `esp_app_desc.c.obj` (~185 KB)** is a tooling artifact — unowned `.rodata`
  is attributed to the first object alphabetically. The actual `.obj` content is ~1 KB. Ignore that row in size-files.

## matrix build (pre-2026-05-25)

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
