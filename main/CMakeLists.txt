idf_component_register(SRCS
        "../src/main.cpp"
        "../src/lcd.cpp"
        "../src/web/server.cpp"
        INCLUDE_DIRS "." "../src")


# FUGU_HV_DIV
# 4.7: lab version
# 5.1: original
if($ENV{FUGU_HV_DIV})
    set(FUGU_HV_DIV $ENV{FUGU_HV_DIV})
else()
    set(FUGU_HV_DIV 4.7)
endif()
component_compile_definitions("FUGU_HV_DIV=${FUGU_HV_DIV}")

if($ENV{FUGU_BAT_V})
    # 14.25, 28.5, 57
    component_compile_definitions("FUGU_BAT_V=$ENV{FUGU_BAT_V}")
endif()
