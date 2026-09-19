cmake_minimum_required(VERSION 3.16)

# Feature flags moved from WITH_* env vars to Kconfig (main/Kconfig.projbuild, CONFIG_FUGU_WITH_*).
# Reject stale env usage so an old `WITH_BLE=0 idf.py build` can't silently build the feature *on*.
foreach (_legacy WITH_BLE WITH_NETW WITH_SPROFILER WITH_VCONV WITH_MCPWM WITH_MEASURE_COIL WITH_WSYNC)
    if (DEFINED ENV{${_legacy}})
        message(FATAL_ERROR
                "${_legacy} env flag is gone — use CONFIG_FUGU_${_legacy} (idf.py menuconfig -> "
                "'Fugu MPPT firmware', or an sdkconfig fragment). See main/Kconfig.projbuild.")
    endif ()
endforeach ()

# Three flags must act *before* project() runs Kconfig (sdkconfig fragment layering,
# EXCLUDE_COMPONENTS), so they can't read CONFIG_* directly. Resolve them by reading the same
# CONFIG_FUGU_* symbols out of the sdkconfig defaults chain (+ any live sdkconfig). main/CMakeLists
# reads CONFIG_FUGU_* normally for everything that happens after project().
if (NOT SDKCONFIG_DEFAULTS)
    if (DEFINED ENV{SDKCONFIG_DEFAULTS})
        set(SDKCONFIG_DEFAULTS $ENV{SDKCONFIG_DEFAULTS})
    else ()
        set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
    endif ()
endif ()

# Resolve a CONFIG_FUGU_* bool from the defaults chain then the live sdkconfig (later wins),
# falling back to `fallback` when set nowhere (mirrors the Kconfig default).
function(fugu_kconfig_bool out sym fallback)
    set(_v ${fallback})
    foreach (_f ${SDKCONFIG_DEFAULTS} sdkconfig)
        set(_p "${CMAKE_CURRENT_SOURCE_DIR}/${_f}")
        if (EXISTS "${_p}")
            file(STRINGS "${_p}" _on REGEX "^${sym}=y")
            file(STRINGS "${_p}" _off REGEX "^(${sym}=n|# ${sym} is not set)")
            if (_on)
                set(_v 1)
            elseif (_off)
                set(_v 0)
            endif ()
        endif ()
    endforeach ()
    set(${out} ${_v} PARENT_SCOPE)
endfunction()

# Layer optional sdkconfig fragments / drop components so flag-gated features (and their flash cost)
# are only baked into builds that ask for them. Order matters — later fragments win.
fugu_kconfig_bool(_fugu_ble CONFIG_FUGU_WITH_BLE 1)
if (_fugu_ble)
    # NimBLE host selection + radio/thermal tunings Kconfig can't default from a projbuild.
    list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.ble")
endif ()
fugu_kconfig_bool(_fugu_netw CONFIG_FUGU_WITH_NETW 1)
if (NOT _fugu_netw)
    list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.no_netw")
endif ()
fugu_kconfig_bool(_fugu_sprof CONFIG_FUGU_WITH_SPROFILER 0)
if (NOT _fugu_sprof)
    # main.cpp guards its use; dropping the component removes its flash + ~8 KB .bss entirely.
    set(EXCLUDE_COMPONENTS esp32-semihosting-profiler)
endif ()

# arduino-esp32 declares a lot of optional managed deps (RainMaker / Insights / Modem / ESP-SR /
# Zigbee / Modbus / etc.) that we never use. With ARDUINO_SELECTIVE_COMPILATION (sdkconfig.defaults)
# we already drop the corresponding arduino library .cpp's, so the only references left are inside
# the unused managed components themselves — exclude those archives so they don't compile or link.
# Kept: cbor (transitive header), mdns, network_provisioning (auto-selected by ESP_WIFI_ENABLED).
# Unused arduino-esp32 transitive deps (RainMaker / Insights / Modem / ESP-SR / Zigbee / Modbus
# / etc.) are stubbed out via override_path in main/idf_component.yml. EXCLUDE_COMPONENTS doesn't
# work for managed components, so the override-to-empty-stub is the next-best lever.

# The OTA-over-BLE receiver lives outside this repo: it is shared with smart-shunt-fw, which is a
# PlatformIO/Arduino build, so the module carries both a CMakeLists.txt and a library.json. Override
# with -DESP_OTA_BLE_DIR=... or the env var if it is checked out elsewhere.
if (NOT ESP_OTA_BLE_DIR)
    if (DEFINED ENV{ESP_OTA_BLE_DIR})
        set(ESP_OTA_BLE_DIR $ENV{ESP_OTA_BLE_DIR})
    else ()
        set(ESP_OTA_BLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../esp-ota-ble")
    endif ()
endif ()
if (NOT EXISTS "${ESP_OTA_BLE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "esp-ota-ble not found at ${ESP_OTA_BLE_DIR} — clone it beside this repo or "
            "set ESP_OTA_BLE_DIR. It provides the OTA-over-BLE receiver (ota_ble.h).")
endif ()
list(APPEND EXTRA_COMPONENT_DIRS "${ESP_OTA_BLE_DIR}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# esp-bootguard (S3 only): the bootloader parks the chip in download mode after 3 crash resets.
# The bootloader only reaches a board by serial flash (0x0), OTA leaves the old one in place.
if (IDF_TARGET STREQUAL "esp32s3")
    if (NOT ESP_BOOTGUARD_DIR)
        if (DEFINED ENV{ESP_BOOTGUARD_DIR})
            set(ESP_BOOTGUARD_DIR $ENV{ESP_BOOTGUARD_DIR})
        else ()
            set(ESP_BOOTGUARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../esp/esp-bootguard")
        endif ()
    endif ()
    if (NOT EXISTS "${ESP_BOOTGUARD_DIR}/bootloader_components")
        message(FATAL_ERROR "esp-bootguard not found at ${ESP_BOOTGUARD_DIR} — clone "
                "github.com/fl4p/esp-bootguard there or set ESP_BOOTGUARD_DIR.")
    endif ()
    list(APPEND EXTRA_COMPONENT_DIRS "${ESP_BOOTGUARD_DIR}/components")
    idf_build_set_property(BOOTLOADER_EXTRA_COMPONENT_DIRS "${ESP_BOOTGUARD_DIR}/bootloader_components" APPEND)
endif ()

project(fugu-firmware)

# arduino-esp32's CMakeLists.txt doesn't list esp_wifi / network_provisioning in REQUIRES even
# though its public WiFi.h → WiFiType.h transitively includes esp_wifi_types.h and (when
# CONFIG_NETWORK_PROV_NETWORK_TYPE_WIFI=y, auto-selected from ESP_WIFI_ENABLED) WiFiGeneric.h
# includes network_provisioning/manager.h. Normally this works only because some unrelated
# managed transitive dep happens to PUBLIC-require these; stubbing the optional deps (RainMaker
# etc.) breaks that chain and the IDF strict-include checker errors any TU that touches WiFi.h.
# Patch by appending the missing PUBLIC requires from here.
idf_component_get_property(_arduino_lib espressif__arduino-esp32 COMPONENT_LIB)
target_link_libraries(${_arduino_lib} PUBLIC
        idf::esp_wifi idf::esp_event idf::esp_netif
        idf::espressif__network_provisioning)

set (CMAKE_CXX_FLAGS "-fexceptions") # -mfp16-format=ieee

if (IDF_TARGET STREQUAL "esp32")
    set(FUGU_LITTLEFS_SRC config/fugu1/fugu1_esp32)
else ()
    set(FUGU_LITTLEFS_SRC config/lab/dry_mock)
endif ()
littlefs_create_partition_image(littlefs ${FUGU_LITTLEFS_SRC}
        FLASH_IN_PROJECT
)

