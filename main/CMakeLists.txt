
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
            # pure-C++ plant is always linked here for test_vconv.cpp. WITH_VCONV defaults off, so
            # VCONV_SRC stays empty in test builds -> no duplicate symbol.
            "../src/sim/vconv.cpp"
    )
elseif ($ENV{MAIN_SRC})
    set(MAIN_SRC $ENV{MAIN_SRC})
else ()
    # cli.cpp, bsync.cpp and measure_coil.cpp extern main.cpp's globals, so they're only built
    # alongside the real main (not test mains).
    set(MAIN_SRC "../src/main.cpp" "../src/cli.cpp")
    if (CONFIG_FUGU_WITH_BSYNC)
        list(APPEND MAIN_SRC "../src/sync/bsync.cpp")
    endif ()
    # CONFIG_FUGU_WITH_MEASURE_COIL compiles in the on-device coil inductance sweep + `measure-coil`
    # console command (src/selftest/measure_coil.cpp). Bench-only tool — saves ~11 KB in production builds.
    if (CONFIG_FUGU_WITH_MEASURE_COIL)
        list(APPEND MAIN_SRC "../src/selftest/measure_coil.cpp")
    endif ()
endif ()

set(NETW_SRC
        "../src/tele/telemetry.cpp"
        "../src/tele/ftp_service.cpp"
        "../src/tele/telnet_service.cpp"
        "../src/tele/scope_service.cpp"
        "../src/tele/mqtt.cpp"
        "../src/tele/HAMqttDevice.cpp"
        "../src/tele/home_assistant.cpp"
        "../src/web/server.cpp"
        "../src/etc/ota.cpp"
)

# CONFIG_FUGU_WITH_NETW (default y) compiles the WiFi/mDNS/MQTT/telemetry/HTTPS-OTA path; off drops
# all of the above and the firmware runs UART/USB/BLE consoles only. The sdkconfig.no_netw layering
# in the top-level CMakeLists tracks the same symbol.
if (CONFIG_FUGU_WITH_NETW)
    set(WITH_NETW 1)
else ()
    set(WITH_NETW 0)
endif ()

if (NOT WITH_NETW)
    set(NETW_SRC "")
endif ()

set(BLE_SRC
        "../src/tele/console_ble.cpp"
)

# CONFIG_FUGU_WITH_BLE (default y) compiles the NimBLE console + OTA path; the sdkconfig.ble layering
# in the top-level CMakeLists tracks the same symbol.
if (CONFIG_FUGU_WITH_BLE)
    set(WITH_BLE 1)
else ()
    set(WITH_BLE 0)
    set(BLE_SRC "")
endif ()

# WiFi-free telemetry core (point queue, symbol table, wire conf, tamp) — needed by the UDP path
# (NETW) and the BLE stream (CONFIG_FUGU_WITH_BLE_TELE, adds src/tele/tele_ble.cpp to BLE_SRC).
set(TELE_CORE_SRC "")
if (WITH_NETW OR CONFIG_FUGU_WITH_BLE_TELE)
    set(TELE_CORE_SRC
            "../src/tele/tele_core.cpp"
            "../src/tele/tamp_compress.cpp"
    )
endif ()
if (CONFIG_FUGU_WITH_BLE_TELE)
    list(APPEND BLE_SRC "../src/tele/tele_ble.cpp")
endif ()
if (CONFIG_FUGU_WITH_BLE_ADV)
    list(APPEND BLE_SRC "../src/tele/tele_adv.cpp")
endif ()

# CONFIG_FUGU_WITH_VCONV replaces the physical gate driver + ADC with a software converter plant
# (src/sim/vconv.*). Kconfig enforces mutual exclusion with the MCPWM driver. See
# docs/superpowers/specs/2026-05-23-virtual-converter-design.md.
if (CONFIG_FUGU_WITH_VCONV)
    set(WITH_VCONV 1)
    set(VCONV_SRC "../src/sim/vconv.cpp")
else ()
    set(WITH_VCONV 0)
    set(VCONV_SRC "")
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
        "../src/etc/rt.cpp"
        "../src/math/float16.cpp"
        ${NETW_SRC}
        ${BLE_SRC}
        ${TELE_CORE_SRC}
        INCLUDE_DIRS "." "../src"
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

if (WITH_NETW)
    component_compile_definitions("WITH_NETW=1")
endif ()

# CONFIG_FUGU_WITH_NETTOOLS (default n, depends on NETW) compiles the `curl`/`ping` console commands.
if (CONFIG_FUGU_WITH_NETTOOLS)
    component_compile_definitions("WITH_NETTOOLS=1")
endif ()


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

if (WITH_BLE)
    component_compile_definitions("WITH_BLE=1")
endif ()
if (CONFIG_FUGU_WITH_BLE_ADV)
    component_compile_definitions("WITH_BLE_ADV=1")
endif ()
if (CONFIG_FUGU_WITH_BLE_TELE)
    component_compile_definitions("WITH_BLE_TELE=1")
endif ()

# Gate-driver back-ends. CONFIG_FUGU_WITH_MCPWM (src/pwm/mcpwm.h): hardware dead-time, GPIO OST brake,
# glitch-free comparator updates (see doc/mcpwm-sync-buck-driver.md). CONFIG_FUGU_WITH_LEDC (src/pwm/ledc.h):
# the LEDC driver + fan PWM + the `anaw` command. With both on, the buck driver is picked at runtime from
# converter.conf::pwm_driver; with LEDC off the whole LEDC peripheral (incl. esp32-hal-ledc) drops out.
if (CONFIG_FUGU_WITH_MCPWM)
    component_compile_definitions("WITH_MCPWM=1")
endif ()
if (CONFIG_FUGU_WITH_LEDC)
    component_compile_definitions("WITH_LEDC=1")
endif ()
if (CONFIG_FUGU_WITH_BSYNC)
    component_compile_definitions("WITH_BSYNC=1")
elseif (CONFIG_FUGU_WITH_NETW AND NOT CONFIG_FUGU_WITH_MCPWM)
    message(WARNING "bsync compiled out: FUGU_WITH_BSYNC pruned because FUGU_WITH_MCPWM is off")
endif ()
if (CONFIG_FUGU_WITH_WSYNC)
    component_compile_definitions("WITH_WSYNC=1")
endif ()

if (WITH_VCONV)
    component_compile_definitions("WITH_VCONV=1")
endif ()

# CONFIG_FUGU_WITH_SPROFILER compiles in the semihosting sampling profiler (sprofiler_initialize,
# only useful with OpenOCD attached). Off by default; the top-level CMakeLists then
# EXCLUDE_COMPONENTS the esp32-semihosting-profiler component so its flash + ~8 KB .bss is dropped.
if (CONFIG_FUGU_WITH_SPROFILER)
    component_compile_definitions("WITH_SPROFILER=1")
endif ()

if (CONFIG_FUGU_WITH_MEASURE_COIL)
    component_compile_definitions("WITH_MEASURE_COIL=1")
endif ()

# Per-file optimization: keep -O2 (CONFIG_COMPILER_OPTIMIZATION_PERF default) on the RT path —
# main.cpp (loopRT itself), mppt.cpp (mppt.update per ADC sample), adc_esp32_cont.cpp (DMA ISR
# helpers). Everything else in libmain is cold: console + cli, services, telemetry, setup helpers,
# bench tools — compile -Os to claw back flash. -fexceptions has to stay global because service.h
# try/catch is included by everything; selectively dropping it would mean rewiring error paths.
set(_size_opt_srcs
        ../src/cli.cpp
        ../src/sync/bsync.cpp
        ../src/adc/sensor_setup.cpp
        ../src/logging.cpp
        ../src/util.cpp
        ../src/viz/lcd.cpp
        ../src/console.cpp
        ../src/etc/rt.cpp
        ../src/math/float16.cpp
)
if (WITH_NETW)
    list(APPEND _size_opt_srcs ${NETW_SRC})
endif ()
if (WITH_BLE)
    list(APPEND _size_opt_srcs ${BLE_SRC})
endif ()
if (CONFIG_FUGU_WITH_MEASURE_COIL)
    list(APPEND _size_opt_srcs ../src/selftest/measure_coil.cpp)
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
