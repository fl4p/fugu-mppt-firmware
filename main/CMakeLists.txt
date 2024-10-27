
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
        #"../src/main2.cpp"
        "../src/lcd.cpp"
        "../src/web/server.cpp"
        "../src/math/float16.cpp"
        INCLUDE_DIRS "." "../src"
        REQUIRES
        # NetworkClientSecure # OTA
)


if ($ENV{FUGU_BAT_V})
    # 14.25, 28.5, 57
    component_compile_definitions("FUGU_BAT_V=$ENV{FUGU_BAT_V}")
endif ()

component_compile_options(--std=gnu++17)

#target_compile_options(__idf_FastLED PRIVATE -Wno-error=reorder)

#component_compile_options(-fexceptions) # enable exceptions in menuconfig (CONFIG_COMPILER_CXX_EXCEPTIONS)!
