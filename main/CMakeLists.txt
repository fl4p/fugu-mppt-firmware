idf_component_register(SRCS
        "../src/main.cpp"
        "../src/lcd.cpp"
        "../src/web/server.cpp"
        INCLUDE_DIRS "." "../src")


component_compile_definitions("FUGU_HV_DIV=4.7")
