if(ESP_PLATFORM)
    idf_component_register(SRCS "src/vfat.cpp" INCLUDE_DIRS "include")
    target_compile_options(${COMPONENT_LIB} PRIVATE -Os)   # vfat.cpp was compiled -Os in libmain before
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(virtual-fat12 LANGUAGES CXX)

add_library(virtual-fat12 src/vfat.cpp)
add_library(virtual-fat12::virtual-fat12 ALIAS virtual-fat12)
target_compile_features(virtual-fat12 PUBLIC cxx_std_17)
target_include_directories(virtual-fat12 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

option(VIRTUAL_FAT12_BUILD_TESTS "Build the virtual-fat12 tests" ON)
option(VIRTUAL_FAT12_BUILD_EXAMPLES "Build the virtual-fat12 examples" ON)

enable_testing()
if(VIRTUAL_FAT12_BUILD_TESTS)
    add_subdirectory(test)
endif()
if(VIRTUAL_FAT12_BUILD_EXAMPLES)
    add_subdirectory(examples/host)
endif()
