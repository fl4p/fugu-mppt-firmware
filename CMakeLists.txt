cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(main)

set (CMAKE_CXX_FLAGS "-fexceptions") # -mfp16-format=ieee

littlefs_create_partition_image(littlefs provisioning/fmetal
#        FLASH_IN_PROJECT
)

# Instrumentation Profiling TODO
# -fprofile-arcs
# https://gcc.gnu.org/onlinedocs/gcc/Instrumentation-Options.html