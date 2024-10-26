
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
        #"../src/main2.cpp"
        "../src/lcd.cpp"
        "../src/web/server.cpp"
        "../src/math/float16.cpp"
        INCLUDE_DIRS "." "../src")


# FUGU_HV_DIV
# 4.7: lab version
# 5.1: original
if ($ENV{FUGU_HV_DIV})
    set(FUGU_HV_DIV $ENV{FUGU_HV_DIV})
else ()
    set(FUGU_HV_DIV 4.7)
endif ()
component_compile_definitions("FUGU_HV_DIV=${FUGU_HV_DIV}")

if ($ENV{FUGU_BAT_V})
    # 14.25, 28.5, 57
    component_compile_definitions("FUGU_BAT_V=$ENV{FUGU_BAT_V}")
endif ()

component_compile_options(--std=gnu++17)

#target_compile_options(__idf_FastLED PRIVATE -Wno-error=reorder)

#component_compile_options(-fexceptions) # enable exceptions in menuconfig (CONFIG_COMPILER_CXX_EXCEPTIONS)!
