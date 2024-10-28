idf_component_register(
        SRCS
        esp32-semihosting-profiler/lib/sprofiler/sprofiler.c
        esp32-semihosting-profiler/lib/sprofiler/esp32_perfmon.c

        INCLUDE_DIRS
        esp32-semihosting-profiler/lib/sprofiler
        REQUIRES
            driver
            vfs
            perfmon
)

project(esp32-semihosting-profiler)
