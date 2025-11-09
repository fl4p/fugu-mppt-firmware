cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(fugu-firmware)

set (CMAKE_CXX_FLAGS "-fexceptions") # -mfp16-format=ieee

littlefs_create_partition_image(littlefs config/fmetal
#        FLASH_IN_PROJECT
)

