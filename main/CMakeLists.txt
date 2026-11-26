
if ($ENV{RUN_TESTS})
    set(MAIN_SRC "../test/main.cpp"
            "../test/test_meter.cpp"
            "../test/test_sampler.cpp"
            "../test/test_math.cpp"
            "../test/test_charger.cpp"
            "../test/test_ascii.cpp"
            "../test/test_statmath.cpp"
            "../test/test_ripple_freq.cpp"
            "../test/test_pd_control.cpp"
            "../test/test_buck.cpp"
            "../test/test_conf.cpp"
            "../test/test_security.cpp"
            "../test/test_pwm.cpp"
            "../test/test_ina226.cpp"
            "../test/test_adc_hw.cpp"
            "../test/test_adc_min.cpp"
            "../test/test_tracker.cpp"
            "../test/test_vconv.cpp"
            "../test/test_dawn.cpp"
            "../test/test_psu_mode.cpp"
            "../test/test_pv_sim.cpp"
            # vconv plant: gated by CONFIG_FUGU_WITH_VCONV in normal builds (VCONV_SRC), but its
            # pure-C++ plant is always linked here for test_vconv.cpp. CONFIG_FUGU_WITH_VCONV defaults off, so
            # VCONV_SRC stays empty in test builds -> no duplicate symbol.
            "../src/sim/vconv.cpp"
    )
elseif (DEFINED ENV{MAIN_SRC})
    set(MAIN_SRC $ENV{MAIN_SRC})
else ()
    # cli.cpp, bsync.cpp and measure_coil.cpp extern main.cpp's globals, so they're only built
    # alongside the real main (not test mains).
    set(MAIN_SRC "../src/main.cpp" "../src/cli.cpp" "../src/tele/alert.cpp" "../src/selftest/bflow_test.cpp")
    if (CONFIG_FUGU_WITH_BSYNC)
        list(APPEND MAIN_SRC "../src/sync/bsync.cpp")
    endif ()
    # CONFIG_FUGU_WITH_MEASURE_COIL compiles in the on-device coil inductance sweep + `measure-coil`
    # console command (src/selftest/measure_coil.cpp). Bench-only tool — saves ~16 KB in production builds.
    if (CONFIG_FUGU_WITH_MEASURE_COIL)
        list(APPEND MAIN_SRC "../src/selftest/measure_coil.cpp")
    endif ()
    if (CONFIG_FUGU_WITH_USB_MSC)
        list(APPEND MAIN_SRC "../src/usb/usb_dev.cpp")
    endif ()
    if (CONFIG_FUGU_WITH_PWR_KNOBS)
        list(APPEND MAIN_SRC "../src/etc/pwr_knobs.cpp")
    endif ()
endif ()

set(NETW_SRC
        "../src/tele/telemetry.cpp"
        "../src/tele/mqtt.cpp"
        "../src/etc/ota.cpp"
)

# CONFIG_FUGU_WITH_NETW (default y) compiles the WiFi/mDNS/MQTT/telemetry/HTTPS-OTA path; off drops
# all of the above and the firmware runs UART/USB/BLE consoles only. The linker drops the
# unreferenced network components, so no sdkconfig fragment is needed.
if (NOT CONFIG_FUGU_WITH_NETW)
    set(NETW_SRC "")
endif ()
if (CONFIG_FUGU_WITH_TELNET)
    list(APPEND NETW_SRC "../src/tele/telnet_service.cpp")
endif ()
if (CONFIG_FUGU_WITH_TLS_CONSOLE)
    list(APPEND NETW_SRC "../src/tele/tls_console.cpp")
endif ()
if (CONFIG_FUGU_WITH_SCOPE)
    list(APPEND NETW_SRC "../src/tele/scope_service.cpp")
endif ()
if (CONFIG_FUGU_WITH_FTP)
    list(APPEND NETW_SRC "../src/tele/ftp_service.cpp" "../src/tele/ftp_server.cpp")
endif ()
if (CONFIG_FUGU_WITH_HTTP_CONF)
    list(APPEND NETW_SRC "../src/tele/http_conf.cpp" "../src/tele/http_conf_service.cpp")
endif ()
if (CONFIG_FUGU_WITH_HA)
    list(APPEND NETW_SRC "../src/tele/HAMqttDevice.cpp" "../src/tele/home_assistant.cpp")
endif ()
if (CONFIG_FUGU_WITH_TELE_MQTT)
    list(APPEND NETW_SRC "../src/tele/tele_mqtt.cpp")
endif ()

set(BLE_SRC
        "../src/tele/console_ble.cpp"
)

# CONFIG_FUGU_WITH_BLE (default y) compiles the NimBLE console + OTA path; the sdkconfig.d/ble layering
# in the top-level CMakeLists tracks the same symbol.
if (NOT CONFIG_FUGU_WITH_BLE)
    set(BLE_SRC "")
endif ()

# WiFi-free telemetry core (point queue, symbol table, wire conf) — needed by the UDP path (NETW) and
# the BLE stream (CONFIG_FUGU_WITH_BLE_TELE, adds src/tele/tele_ble.cpp to BLE_SRC). tamp only with
# the binary wire (CONFIG_FUGU_WITH_TELE_BINARY).
set(TELE_CORE_SRC "")
if (CONFIG_FUGU_WITH_NETW OR CONFIG_FUGU_WITH_BLE_TELE)
    set(TELE_CORE_SRC "../src/tele/tele_core.cpp")
endif ()
if (CONFIG_FUGU_WITH_TELE_BINARY)
    list(APPEND TELE_CORE_SRC "../src/tele/tamp_compress.cpp")
endif ()
if (CONFIG_FUGU_WITH_TELE_PMINI)
    list(APPEND TELE_CORE_SRC "../src/tele/pmini_codec.cpp" "../src/tele/pmini_packet.cpp" "../src/tele/tele_packet.cpp")
endif ()
if (CONFIG_FUGU_WITH_BLE_TELE)
    list(APPEND BLE_SRC "../src/tele/tele_ble.cpp")
endif ()
if (CONFIG_FUGU_WITH_BLE_ADV)
    list(APPEND BLE_SRC "../src/tele/tele_adv.cpp")
endif ()
if (CONFIG_FUGU_WITH_IMPROV)
    list(APPEND BLE_SRC "../src/tele/improv_ble.cpp")
endif ()

# CONFIG_FUGU_WITH_VCONV replaces the physical gate driver + ADC with a software converter plant
# (src/sim/vconv.*); one of the Kconfig gate-driver choices. See
# doc/superpowers/specs/2026-05-23-virtual-converter-design.md.
if (CONFIG_FUGU_WITH_VCONV)
    set(VCONV_SRC "../src/sim/vconv.cpp")
else ()
    set(VCONV_SRC "")
endif ()

# FUGU_RT_FLASH=1 keeps the RT path in flash, for a first rtprof profile of all of it
if (CONFIG_FUGU_WITH_RTPROF)
    set(RTPROF_SRC "../src/etc/rtprof.cpp")
endif ()
# S3 only: the classic ESP32 has no IRAM to spare and no flash auto-suspend
if (CONFIG_IDF_TARGET_ESP32S3 AND NOT "$ENV{FUGU_RT_FLASH}")
    set(RT_IRAM_LF "rt_iram.lf" "rt_iram_extra.lf")
endif ()

idf_component_register(SRCS
        ${MAIN_SRC}
        ${VCONV_SRC}
        "../src/adc/sensor_setup.cpp"
        "../src/mppt.cpp"
        "../src/logging.cpp"
        "../src/util.cpp"
        "../src/adc/adc_esp32_cont.cpp"
        "../src/viz/lcd.cpp"
        "../src/console.cpp"
        "../src/conf_hash.cpp"
        "../src/etc/rt.cpp"
        "../src/etc/flash_probe.cpp"
        "../src/etc/heap_diag.cpp"
        ${RTPROF_SRC}
        "../src/math/float16.cpp"
        ${NETW_SRC}
        ${BLE_SRC}
        ${TELE_CORE_SRC}
        INCLUDE_DIRS "." "../src"
        LDFRAGMENTS "linker.lf" ${RT_IRAM_LF}
        REQUIRES
        # if we start to list components here we need to list them all
        # put dependencies in component/Arduion-LiquidCrystal as only needed for linking
        #arduino
        #esp_system
        #Arduino-LiquidCrystal-I2C
        #arduino
        #esp_system
        # NetworkClientSecure # OTA
)

target_linker_script(${COMPONENT_LIB} INTERFACE "rtc_pin.ld")

if ($ENV{FUGU_BAT_V})
    # 14.25, 28.5, 57
    component_compile_definitions("FUGU_BAT_V=$ENV{FUGU_BAT_V}")
endif ()

# TEST_ADC_HW (with RUN_TESTS): run ONLY the internal-ADC + INA226 liveness tests (test_adc_hw.cpp)
# and skip the PWM/MCPWM tests that drive gate pins — safe to flash onto a live converter.
if ($ENV{TEST_ADC_HW})
    component_compile_definitions("TEST_ADC_HW=1")
endif ()

# FUGU_FORCE_BFLOW_OFF: bench-safety override that keeps the backflow/panel switch open
# (BackflowDriver::enable() forced false) so battery current can't enter the converter
# while testing a new gate-driver path. Off by default.
if ($ENV{FUGU_FORCE_BFLOW_OFF})
    component_compile_definitions("FUGU_FORCE_BFLOW_OFF=1")
endif ()

if (CONFIG_FUGU_WITH_NETW AND NOT CONFIG_FUGU_WITH_MCPWM)
    message(STATUS "bsync compiled out: needs the MCPWM gate driver")
endif ()

# CONFIG_FUGU_WITH_USB_MSC: CDC console + emulated FAT config drive on espressif/tinyusb (pulled in
# by idf_component.yml only for this flag). TinyUSB finds its config header through this include dir.
if (CONFIG_FUGU_WITH_USB_MSC)
    if (NOT CONFIG_IDF_TARGET_ESP32S3 OR CONFIG_FUGU_WITH_WSYNC)
        message(FATAL_ERROR "FUGU_WITH_USB_MSC needs an ESP32-S3 and FUGU_WITH_WSYNC off (both use the USB pads)")
    endif ()
    idf_component_get_property(_tusb_lib espressif__tinyusb COMPONENT_LIB)
    target_include_directories(${_tusb_lib} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../src/usb")
elseif (CONFIG_IDF_TARGET_ESP32S3 AND CONFIG_ESP_CONSOLE_SECONDARY_NONE)
    # sdkconfig.d/usb_msc only seeds a fresh sdkconfig; switching the flag off keeps its console choice
    message(WARNING "USB-Serial-JTAG console is off (left over from FUGU_WITH_USB_MSC?): delete sdkconfig to restore it")
endif ()

# Per-file optimization: keep -O2 (CONFIG_COMPILER_OPTIMIZATION_PERF default) on the RT path —
# main.cpp (loopRT itself), mppt.cpp (mppt.update per ADC sample), adc_esp32_cont.cpp (DMA ISR
# helpers). Everything else in libmain is cold: console + cli, services, telemetry, setup helpers,
# bench tools — compile -Os to claw back flash. -fexceptions has to stay global because service.h
# try/catch is included by everything; selectively dropping it would mean rewiring error paths.
set(_size_opt_srcs
        ../src/cli.cpp
        ../src/selftest/bflow_test.cpp
        ../src/sync/bsync.cpp
        ../src/adc/sensor_setup.cpp
        ../src/logging.cpp
        ../src/util.cpp
        ../src/viz/lcd.cpp
        ../src/console.cpp
        ../src/conf_hash.cpp
        ../src/tele/alert.cpp
        ../src/etc/heap_diag.cpp
        ../src/etc/rt.cpp
        ../src/math/float16.cpp
)
if (CONFIG_FUGU_WITH_NETW)
    list(APPEND _size_opt_srcs ${NETW_SRC})
endif ()
if (CONFIG_FUGU_WITH_BLE)
    list(APPEND _size_opt_srcs ${BLE_SRC})
endif ()
if (CONFIG_FUGU_WITH_MEASURE_COIL)
    list(APPEND _size_opt_srcs ../src/selftest/measure_coil.cpp)
endif ()
if (CONFIG_FUGU_WITH_USB_MSC)
    list(APPEND _size_opt_srcs ../src/usb/usb_dev.cpp)
endif ()
if (CONFIG_FUGU_WITH_PWR_KNOBS)
    list(APPEND _size_opt_srcs ../src/etc/pwr_knobs.cpp)
endif ()
list(APPEND _size_opt_srcs ${TELE_CORE_SRC})
set_source_files_properties(${_size_opt_srcs} PROPERTIES COMPILE_OPTIONS "-Os")

component_compile_options(-Werror=missing-field-initializers)
component_compile_options(-Werror=attributes) # IRAM_ATTR used in forward decl

component_compile_options(--std=gnu++20)
component_compile_options(
      #  -fsanitize=address
      #  -fsanitize=shadow-call-stack # esp32: not supported
      #  -fsanitize=thread
        #-fsanitize=leak
        #-fsanitize=undefined -fno-sanitize=shift-base
      #  -fsanitize-address-use-after-scope
)

#target_compile_options(__idf_FastLED PRIVATE -Wno-error=reorder)

#component_compile_options(-fexceptions) # enable exceptions in menuconfig (CONFIG_COMPILER_CXX_EXCEPTIONS)!

# Keep this component's string literals OUT of the linker's global merged string pool.
# GCC emits string literals into SHF_MERGE|SHF_STRINGS sections (.rodata.<fn>.str1.4); ld
# coalesces every such section in .flash.rodata into ONE blob (187 KB here) placed at its
# first contributor -- near the START of rodata, and unaffected by EXCLUDE_FILE, so the
# linker.lf placement cannot reach it. Adding one string literal to this component then
# grew that early blob and shifted ~70% of .flash.rodata, which is what forces a full-image
# OTA rewrite. -fno-merge-constants leaves the literals in ordinary per-function .rodata.*
# sections, which linker.lf can and does place last.
#
# Cost: +14,352 B, ALL of it in .flash.rodata. The DROM length pins the IROM file offset to a
# 64 KB MMU page; crossing a page edge either way churns nearly every IROM sector.
# etc/idf-devtools/image_layout.py <build dir> prints both margins (CI checks the fleet build);
# see doc/2026-09-08-image-layout-stability-for-ota.md.
#
# The flag is broader than its name: GCC's -fmerge-constants covers identical string AND
# floating-point constants across translation units, so this also disables pooling within
# this component's own 26 TUs. It is NOT applied to libesp-ota-ble.a, whose merge sections
# still join the early pool.
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-merge-constants)
if (CONFIG_HEAP_TASK_TRACKING)
    foreach (_f alloc free realloc)
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=heap_caps_update_per_task_info_${_f}")
    endforeach ()
endif ()
if (NOT CONFIG_ESP_ERR_TO_NAME_LOOKUP)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_err_to_name") # src/util.cpp
endif ()
if (CONFIG_FUGU_PBUF_PROBE)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=pbuf_free")
endif ()
# GPIO ISR service with ESP_INTR_FLAG_IRAM (Kconfig, default off): only where its handler chain is
# IRAM-resident (rt_iram_extra.lf)
if (RT_IRAM_LF AND CONFIG_FUGU_GPIO_ISR_IRAM)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FUGU_GPIO_ISR_IRAM=1)
endif ()
# Extra compile options for this component only, for size and speed experiments:
#   idf.py -B build/x -DFUGU_MAIN_COPTS="-O2;-DLOG_LOCAL_LEVEL=3" build
if (FUGU_MAIN_COPTS)
    target_compile_options(${COMPONENT_LIB} PRIVATE ${FUGU_MAIN_COPTS})
endif ()
