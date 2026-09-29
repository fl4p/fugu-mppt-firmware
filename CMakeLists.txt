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

# The OTA-over-BLE receiver (fl4p/esp-ota-ble) is a git dependency in main/idf_component.yml. A local
# checkout overrides it: -DESP_OTA_BLE_DIR=... or the env var, else ../esp-ota-ble if present.
if (NOT ESP_OTA_BLE_DIR AND DEFINED ENV{ESP_OTA_BLE_DIR})
    set(ESP_OTA_BLE_DIR $ENV{ESP_OTA_BLE_DIR})
endif ()
if (ESP_OTA_BLE_DIR)
    if (NOT EXISTS "${ESP_OTA_BLE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "esp-ota-ble not found at ESP_OTA_BLE_DIR=${ESP_OTA_BLE_DIR}")
    endif ()
elseif (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../esp-ota-ble/CMakeLists.txt")
    set(ESP_OTA_BLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../esp-ota-ble")
endif ()
if (ESP_OTA_BLE_DIR)
    message(STATUS "esp-ota-ble: local ${ESP_OTA_BLE_DIR}")
    list(APPEND EXTRA_COMPONENT_DIRS "${ESP_OTA_BLE_DIR}")
else ()
    message(STATUS "esp-ota-ble: from main/idf_component.yml")
endif ()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# esp-bootguard (S3 only, optional dev tool): the bootloader parks the chip in download mode after 3 crash
# resets. Used when found at the default path or ESP_BOOTGUARD_DIR, else the stock bootloader is built.
# The bootloader only reaches a board by serial flash (0x0), OTA leaves the old one in place.
if (IDF_TARGET STREQUAL "esp32s3")
    if (NOT ESP_BOOTGUARD_DIR AND DEFINED ENV{ESP_BOOTGUARD_DIR})
        set(ESP_BOOTGUARD_DIR $ENV{ESP_BOOTGUARD_DIR})
    endif ()
    if (ESP_BOOTGUARD_DIR)
        if (NOT EXISTS "${ESP_BOOTGUARD_DIR}/bootloader_components")
            message(FATAL_ERROR "esp-bootguard not found at ESP_BOOTGUARD_DIR=${ESP_BOOTGUARD_DIR}")
        endif ()
    elseif (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../esp/esp-bootguard/bootloader_components")
        set(ESP_BOOTGUARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../esp/esp-bootguard")
    endif ()
    if (ESP_BOOTGUARD_DIR)
        message(STATUS "esp-bootguard: ${ESP_BOOTGUARD_DIR}")
        list(APPEND EXTRA_COMPONENT_DIRS "${ESP_BOOTGUARD_DIR}/components")
        idf_build_set_property(BOOTLOADER_EXTRA_COMPONENT_DIRS "${ESP_BOOTGUARD_DIR}/bootloader_components" APPEND)
    else ()
        message(STATUS "esp-bootguard: not found, using the stock bootloader")
    endif ()
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

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions") # keep toolchain flags (IDF 6: picolibc specs). -mfp16-format=ieee

if (IDF_TARGET STREQUAL "esp32")
    set(FUGU_LITTLEFS_SRC config/fugu1/fugu1_esp32)
else ()
    set(FUGU_LITTLEFS_SRC config/lab/dry_mock)
endif ()
littlefs_create_partition_image(littlefs ${FUGU_LITTLEFS_SRC}
        FLASH_IN_PROJECT
)

