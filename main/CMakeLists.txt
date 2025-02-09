
if ($ENV{RUN_TESTS})
    set(MAIN_SRC "../test/main.cpp"
            "../test/test_meter.cpp"
            ../test/test_sampler.cpp
    )
elseif ($ENV{MAIN_SRC})
    set(MAIN_SRC $ENV{MAIN_SRC})
else ()
    set(MAIN_SRC "../src/main.cpp")
endif ()

idf_component_register(SRCS
        ${MAIN_SRC}
        "../src/logging.cpp"
        "../src/util.cpp"
        "../src/adc/adc_esp32_cont.cpp"
        "../src/viz/lcd.cpp"
        "../src/web/server.cpp"
        "../src/console.cpp"
        "../src/etc/rt.cpp"
        "../src/mqtt.cpp"
        "../src/math/float16.cpp"
        INCLUDE_DIRS "." "../src" "../src/asciichart"
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


if ($ENV{FUGU_BAT_V})
    # 14.25, 28.5, 57
    component_compile_definitions("FUGU_BAT_V=$ENV{FUGU_BAT_V}")
endif ()

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
